#include <Wire.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>
#include <BluetoothSerial.h>
#include "driver/adc.h"

const int FUEL_GATE_PIN = 23;
const int BUZZER_PIN = 18;
const int I2C_SDA = 21;
const int I2C_SCL = 22;
HardwareSerial gpsSerial(2);
const int GPS_RX = 16;
const double LAT = 12.843583;
const double LNG = 80.156194;
const int GPS_TX = 17;
const uint32_t GPS_BAUD = 9600;

BluetoothSerial SerialBT;

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
const uint32_t LATCH_HOLDOFF_MS = 10000;
const uint32_t FUEL_RESTORE_MS = 15000;

const float A_MINOR_G = 2.5f, J_MINOR = 5.0f, W_MINOR = 250.0f;
const float A_MOD_G = 4.0f, J_MOD = 8.0f, W_MOD = 300.0f;
const float A_SEV_G = 6.0f, J_SEV = 15.0f, W_SEV = 500.0f;

MPU6050 mpu;
TinyGPSPlus gps;

float gX=0, gY=0, gZ=1.0f;
float prevA_smooth=0.0f;
uint32_t lastSample=0, lastDbg=0;

float axBias=0, ayBias=0, azBias=0;
bool seatOccupied = true;
float seatHP=0, seatEnergyAccum=0, seatEnergy=0; 
int seatCount=0;

uint32_t lastTriggerMs=0;
double lastLat=0, lastLng=0, lastSpeedKmph=0;

struct PeakBuf { float a_peak, j_peak, w_peak; uint32_t start_ms; } win;

static inline float lpf(float prev, float x, float alpha){ return alpha*prev + (1.0f-alpha)*x; }
void setFuel(bool on){ digitalWrite(FUEL_GATE_PIN, on ? HIGH : LOW); }
void buzz(int ms){ digitalWrite(BUZZER_PIN, HIGH); delay(ms); digitalWrite(BUZZER_PIN, LOW); }

void sendBTAccident(double lat, double lng) {
  String msg = "ACCIDENT," + String(lat,6) + "," + String(lng,6);
  SerialBT.print(msg);
  SerialBT.print("\r\n");   
  SerialBT.flush();
  Serial.println("🔵 BT -> " + msg);
}

String classifySeverity(float a, float j, float w) {
  if (a >= A_SEV_G || j >= J_SEV || w >= W_SEV) return "SEVERE";
  if (a >= A_MOD_G || j >= J_MOD || w >= W_MOD) return "MODERATE";
  if (a >= A_MINOR_G || j >= J_MINOR || w >= W_MINOR) return "MINOR";
  return "NONE";
}

void emitAccident(const String& sev, float a, float j, float w) {
  double lat = gps.location.isValid() ? gps.location.lat() : LAT;
  double lng = gps.location.isValid() ? gps.location.lng() : LNG;
  Serial.printf("ACCIDENT,%s,%.2f,%.1f,%.0f,%.1f,%.6f,%.6f\n",
                sev.c_str(), a, j, w, lastSpeedKmph, lat, lng);
  sendBTAccident(lat, lng);
  buzz(800);
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Accident_BT");
  delay(200);
  Serial.println("Bluetooth ready");

  SerialBT.print("BT_READY\r\n");
  SerialBT.flush();

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FUEL_GATE_PIN, OUTPUT);
  setFuel(true);

  analogSetPinAttenuation(PIEZO_ADC_PIN, PIEZO_ATTEN);

  Wire.begin(I2C_SDA, I2C_SCL);
  mpu.initialize();
  if (!mpu.testConnection()) { Serial.println("MPU6050 not found!"); while(true) delay(1000); }

  Serial.print("Calibrating...");
  long axSum=0, aySum=0, azSum=0;
  for (int i=0; i<200; i++) {
    int16_t ax,ay,az,gx,gy,gz;
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    axSum+=ax; aySum+=ay; azSum+=az;
    delay(5);
  }
  axBias=axSum/200.0f; ayBias=aySum/200.0f; azBias=azSum/200.0f;

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  gpsSerial.setRxBufferSize(2048);

  win = {0,0,0, millis()};
  lastSample = micros();

  Serial.println(" System Ready");
}

void loop() {
  if (Serial.available()) {
    int c = Serial.read();
    if (c == 'T' || c == 't') {
      double lat = gps.location.isValid() ? gps.location.lat() : LAT;
      double lng = gps.location.isValid() ? gps.location.lng() : LNG;
      sendBTAccident(lat, lng);
    }
  }

  uint32_t nowUs = micros();
  if (nowUs - lastSample < SAMPLE_US) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
      if (gps.location.isUpdated()) { lastLat = gps.location.lat(); lastLng = gps.location.lng(); }
      if (gps.speed.isUpdated()) { lastSpeedKmph = gps.speed.kmph(); }
    }
    return;
  }
  lastSample += SAMPLE_US;
  float dt = SAMPLE_US / 1e6f;
  uint32_t nowMs = millis();

  int16_t axr, ayr, azr, gxr, gyr, gzr;
  mpu.getMotion6(&axr,&ayr,&azr,&gxr,&gyr,&gzr);
  float ax=(axr-axBias)/ACC_SENS, ay=(ayr-ayBias)/ACC_SENS, az=(azr-azBias)/ACC_SENS;
  float gx=gxr/GYRO_SENS, gy=gyr/GYRO_SENS, gz=gzr/GYRO_SENS;
  float w = sqrtf(gx*gx+gy*gy+gz*gz);

  gX = lpf(gX, ax, GRAVITY_ALPHA);
  gY = lpf(gY, ay, GRAVITY_ALPHA);
  gZ = lpf(gZ, az, GRAVITY_ALPHA);
  float linX=ax-gX, linY=ay-gY, linZ=az-gZ;
  float a = sqrtf(linX*linX + linY*linY + linZ*linZ);
  float a_smooth = lpf(prevA_smooth, a, A_SMOOTH_ALPHA);
  float jerk = (a_smooth - prevA_smooth)/dt;
  prevA_smooth = a_smooth;

  static int prevSeat = 0;
  int x = analogRead(PIEZO_ADC_PIN);
  int dx = x - prevSeat; prevSeat = x;
  seatHP = SEAT_HP_ALPHA * (seatHP + dx);
  float seatAbs = fabsf(seatHP);
  seatEnergyAccum += seatAbs;
  seatCount++;
  if (seatCount >= SEAT_EN_AVG_SAMPLES) {
    seatEnergy = seatEnergyAccum / seatCount;
    seatEnergyAccum = 0; seatCount = 0;
    seatOccupied = (seatEnergy >= SEAT_ENERGY_THRESH);
  }

  if (nowMs - win.start_ms > IMPACT_WINDOW_MS) {
    String sev = classifySeverity(win.a_peak, win.j_peak, win.w_peak);
    if (sev != "NONE" && seatOccupied && (nowMs - lastTriggerMs > LATCH_HOLDOFF_MS)) {
      lastTriggerMs = nowMs;
      setFuel(false);
      buzz(1000);
      emitAccident(sev, win.a_peak, win.j_peak, win.w_peak);
    }
    win = {0,0,0, nowMs};
  }

  if (a_smooth > win.a_peak) win.a_peak = a_smooth;
  if (fabsf(jerk) > win.j_peak) win.j_peak = fabsf(jerk);
  if (w > win.w_peak) win.w_peak = w;


  if (nowMs - lastDbg > 500) {
    lastDbg = nowMs;
    Serial.printf("a=%.2fg jerk=%5.1fg/s w=%3.0fdps seat=%c E=%.1f gps=%c spd=%.1f\n",
                  a_smooth, jerk, w, seatOccupied?'Y':'N', seatEnergy,
                  gps.location.isValid()?'Y':'N', lastSpeedKmph);
  }

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()) { lastLat = gps.location.lat(); lastLng = gps.location.lng(); }
    if (gps.speed.isUpdated()) { lastSpeedKmph = gps.speed.kmph(); }
  }
}
