#include <Wire.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "driver/adc.h"

#define SERVICE_UUID        "4fa87100-87ee-11e7-802c-09173f13e4c6"
#define CHARACTERISTIC_UUID "4fa87101-87ee-11e7-802c-09173f13e4c6"

const int FUEL_GATE_PIN = 23;
const int BUZZER_PIN = 18;
const int IR_PIN = 35;

const int I2C_SDA = 21;
const int I2C_SCL = 22;

HardwareSerial gpsSerial(2);

const int GPS_RX = 16;
const int GPS_TX = 17;
const uint32_t GPS_BAUD = 9600;

const double LAT = 12.843583;
const double LNG = 80.156194;

BLECharacteristic *pCharacteristic = nullptr;
bool deviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
public:
    void onConnect(BLEServer* pServer) override {
      deviceConnected = true;
      Serial.println("🔵 BLE Client Connected");
    }

    void onDisconnect(BLEServer* pServer) override {
      deviceConnected = false;
      Serial.println("🔴 BLE Client Disconnected - Restarting Advertising");
      BLEDevice::startAdvertising();
    }
};

const int PIEZO_ADC_PIN = 34;
const adc_attenuation_t PIEZO_ATTEN = ADC_11db;

const float SEAT_HP_ALPHA = 0.95f;
const int SEAT_EN_AVG_SAMPLES = 40;
const float SEAT_ENERGY_THRESH = 12.0f;

const float ACC_SENS = 16384.0f;
const float GYRO_SENS = 131.0f;

const float GRAVITY_ALPHA = 0.98f;
const float A_SMOOTH_ALPHA = 0.85f;

const float SAMPLE_HZ = 200.0f;
const uint32_t SAMPLE_US = (uint32_t)(1000000.0f / SAMPLE_HZ);

const uint32_t IMPACT_WINDOW_MS = 150;
const uint32_t VALIDATION_WINDOW_MS = 2000;
const uint32_t LATCH_HOLDOFF_MS = 10000;
const uint32_t FUEL_RESTORE_MS = 15000;

const float A_MINOR_G = 2.5f, J_MINOR = 5.0f, W_MINOR = 250.0f;
const float A_MOD_G = 4.0f, J_MOD = 8.0f, W_MOD = 300.0f;
const float A_SEV_G = 6.0f, J_SEV = 15.0f, W_SEV = 500.0f;

const int IR_CHANGE_THRESHOLD = 80;
const int IR_PULSE_MIN = 4;

int lastIR = 0;
int irPulseCount = 0;

bool waitingWearable = false;
uint32_t wearableWindowStart = 0;

MPU6050 mpu;
TinyGPSPlus gps;

float gX=0,gY=0,gZ=1;
float prevA_smooth=0;

uint32_t lastSample=0,lastDbg=0;

float axBias=0,ayBias=0,azBias=0;

bool seatOccupied=true;

float seatHP=0,seatEnergyAccum=0,seatEnergy=0;
int seatCount=0;

uint32_t lastTriggerMs=0;
bool fuelCutoffActive = false;
uint32_t fuelCutoffTime = 0;

struct PeakBuf { float a_peak,j_peak,w_peak; uint32_t start_ms; } win;

static inline float lpf(float prev,float x,float alpha){
  return alpha*prev+(1-alpha)*x;
}

void setFuel(bool on){
  digitalWrite(FUEL_GATE_PIN,on?HIGH:LOW);
}

void buzz(int ms){
  digitalWrite(BUZZER_PIN,HIGH);
  delay(ms);
  digitalWrite(BUZZER_PIN,LOW);
}

void sendBLEAccident(double lat, double lng) {
  String msg = "ACCIDENT," + String(lat,6) + "," + String(lng,6) + "\n";
  if (deviceConnected && pCharacteristic != nullptr) {
    pCharacteristic->setValue(msg.c_str());
    pCharacteristic->notify();
    Serial.println("🔵 BLE -> " + msg);
  } else {
    Serial.println("🔵 BLE (Not Connected) -> " + msg);
  }
}

String classifySeverity(float a,float j,float w){
  if(a>=A_SEV_G||j>=J_SEV||w>=W_SEV) return "SEVERE";
  if(a>=A_MOD_G||j>=J_MOD||w>=W_MOD) return "MODERATE";
  if(a>=A_MINOR_G||j>=J_MINOR||w>=W_MINOR) return "MINOR";
  return "NONE";
}

void emitAccident(const String& sev,float a,float j,float w){

  double lat=gps.location.isValid()?gps.location.lat():LAT;
  double lng=gps.location.isValid()?gps.location.lng():LNG;

  Serial.printf("ACCIDENT,%s,%.2f,%.1f,%.0f,%.6f,%.6f\n",
                sev.c_str(),a,j,w,lat,lng);

  sendBLEAccident(lat,lng);

  buzz(800);
}

void checkWearableIR(){

  int val = analogRead(IR_PIN);

  if(waitingWearable && abs(val-lastIR) > IR_CHANGE_THRESHOLD){
    irPulseCount++;
  }

  lastIR = val;
}

void setup(){

  Serial.begin(115200);

  // Initialize BLE
  BLEDevice::init("ESP32_Accident_BLE");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("BT_READY\r\n");

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("BLE Ready and Advertising");

  pinMode(BUZZER_PIN,OUTPUT);
  pinMode(FUEL_GATE_PIN,OUTPUT);
  pinMode(IR_PIN,INPUT);

  setFuel(true);

  analogSetPinAttenuation(PIEZO_ADC_PIN,PIEZO_ATTEN);

  Wire.begin(I2C_SDA,I2C_SCL);

  mpu.initialize();

  if(!mpu.testConnection()){
    Serial.println("MPU6050 not found!");
    while(true);
  }

  Serial.println("Calibrating MPU");

  long axSum=0,aySum=0,azSum=0;

  for(int i=0;i<200;i++){

    int16_t ax,ay,az,gx,gy,gz;

    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

    axSum+=ax;
    aySum+=ay;
    azSum+=az;

    delay(5);
  }

  axBias=axSum/200.0;
  ayBias=aySum/200.0;
  azBias=azSum/200.0;

  gpsSerial.begin(GPS_BAUD,SERIAL_8N1,GPS_RX,GPS_TX);

  win={0,0,0,millis()};
  lastSample=micros();

  Serial.println("System Ready");
}

void loop(){

  uint32_t nowUs=micros();

  if(nowUs-lastSample < SAMPLE_US){
    while(gpsSerial.available()){
      gps.encode(gpsSerial.read());
    }
    return;
  }

  lastSample += SAMPLE_US;

  float dt=SAMPLE_US/1e6;
  uint32_t nowMs=millis();

  int16_t axr,ayr,azr,gxr,gyr,gzr;

  mpu.getMotion6(&axr,&ayr,&azr,&gxr,&gyr,&gzr);

  float ax=(axr-axBias)/ACC_SENS;
  float ay=(ayr-ayBias)/ACC_SENS;
  float az=(azr-azBias)/ACC_SENS;

  float gx=gxr/GYRO_SENS;
  float gy=gyr/GYRO_SENS;
  float gz=gzr/GYRO_SENS;

  float w=sqrt(gx*gx+gy*gy+gz*gz);

  gX=lpf(gX,ax,GRAVITY_ALPHA);
  gY=lpf(gY,ay,GRAVITY_ALPHA);
  gZ=lpf(gZ,az,GRAVITY_ALPHA);

  float linX=ax-gX;
  float linY=ay-gY;
  float linZ=az-gZ;

  float a=sqrt(linX*linX+linY*linY+linZ*linZ);

  float a_smooth=lpf(prevA_smooth,a,A_SMOOTH_ALPHA);

  float jerk=(a_smooth-prevA_smooth)/dt;

  prevA_smooth=a_smooth;

  checkWearableIR();

  static int prevSeat=0;

  int x=analogRead(PIEZO_ADC_PIN);
  int dx=x-prevSeat;
  prevSeat=x;

  seatHP=SEAT_HP_ALPHA*(seatHP+dx);

  float seatAbs=fabs(seatHP);

  seatEnergyAccum+=seatAbs;
  seatCount++;

  if(seatCount>=SEAT_EN_AVG_SAMPLES){

    seatEnergy=seatEnergyAccum/seatCount;

    seatEnergyAccum=0;
    seatCount=0;

    seatOccupied=(seatEnergy>=SEAT_ENERGY_THRESH);
  }

  if(a_smooth > win.a_peak) win.a_peak=a_smooth;
  if(fabs(jerk) > win.j_peak) win.j_peak=fabs(jerk);
  if(w > win.w_peak) win.w_peak=w;

  if(nowMs-win.start_ms > IMPACT_WINDOW_MS){

    String sev=classifySeverity(win.a_peak,win.j_peak,win.w_peak);
if (sev!="NONE" &&
   seatOccupied &&
   !waitingWearable &&
   lastSpeedKmph >= 8 &&
   (nowMs-lastTriggerMs>LATCH_HOLDOFF_MS)){

      Serial.println("Vehicle impact detected");

      waitingWearable=true;
      irPulseCount=0;
      wearableWindowStart=millis();
    }

    win={0,0,0,nowMs};
  }

  if(waitingWearable){

    if(millis()-wearableWindowStart > VALIDATION_WINDOW_MS){

      if(irPulseCount >= IR_PULSE_MIN){

        Serial.println("Wearable confirmed accident");

        lastTriggerMs=millis();

  setFuel(false);

fuelCutoffActive = true;
fuelCutoffTime = millis();

buzz(1000);

emitAccident("CONFIRMED",win.a_peak,win.j_peak,win.w_peak);

      }else{

        Serial.println("Wearable not detected, ignoring bump");
      }

      waitingWearable=false;
    }
  }

  if(nowMs-lastDbg>500){

    lastDbg=nowMs;

    Serial.printf("a=%.2fg jerk=%5.1fg/s w=%3.0fdps seat=%c IR=%d\n",
                  a_smooth,jerk,w,seatOccupied?'Y':'N',irPulseCount);
  }
    if (fuelCutoffActive &&
      millis() - fuelCutoffTime >= FUEL_RESTORE_MS) {

    setFuel(true);

    fuelCutoffActive = false;

    Serial.println("Fuel restored automatically");
  }
}
