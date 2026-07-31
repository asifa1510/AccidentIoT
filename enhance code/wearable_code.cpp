

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

static const int PIN_IR_LED = 3;

static const int SCREEN_W = 128;
static const int SCREEN_H = 64;

static const float        IMPACT_G        = 1.8f;
static const unsigned long COOLDOWN_MS    = 5000;

static const int  IR_PULSES           = 24;
static const int  IR_HALF_PERIOD_MS   = 35;

static const unsigned long DISPLAY_INTERVAL_MS = 250;

static Adafruit_SSD1306        display(SCREEN_W, SCREEN_H, &Wire, -1);
static Adafruit_ADXL345_Unified accel(12345);

static unsigned long lastImpactMs  = 0;
static unsigned long lastDisplayMs = 0;

static void sendIR() {
    Serial.println("IR signal sent");
    for (int i = 0; i < IR_PULSES; i++) {
        digitalWrite(PIN_IR_LED, HIGH);
        delay(IR_HALF_PERIOD_MS);
        digitalWrite(PIN_IR_LED, LOW);
        delay(IR_HALF_PERIOD_MS);
    }
}

static void showImpactScreen(float gVal) {
    display.clearDisplay();
    display.fillRect(0, 0, SCREEN_W, SCREEN_H, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setTextSize(2);
    display.setCursor(10, 8);
    display.println("IMPACT!");
    display.setTextSize(1);
    display.setCursor(20, 40);
    display.print("G = ");
    display.println(gVal, 2);
    display.setCursor(20, 54);
    display.println("Sending IR");
    display.display();
}

static void showIdleScreen(float ax, float ay, float az, float gVal) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    display.setCursor(0, 0);
    display.print("X: ");
    display.println(ax, 1);

    display.setCursor(0, 14);
    display.print("Y: ");
    display.println(ay, 1);

    display.setCursor(0, 28);
    display.print("Z: ");
    display.println(az, 1);

    display.setTextSize(2);
    display.setCursor(0, 44);
    display.print("G:");
    display.print(gVal, 2);

    display.display();
}

void setup() {
    Serial.begin(115200);

    pinMode(PIN_IR_LED, OUTPUT);
    digitalWrite(PIN_IR_LED, LOW);

    Wire.begin();
    Wire.setClock(400000);

    delay(300);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("OLED init failed");
        while (true);
    }

    if (!accel.begin()) {
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(10, 20);
        display.println("ADXL ERR");
        display.display();
        Serial.println("ADXL345 not detected");
        while (true);
    }

    accel.setRange(ADXL345_RANGE_16_G);

    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(18, 8);
    display.println("READY");
    display.setTextSize(1);
    display.setCursor(18, 38);
    display.println("ADXL + OLED OK");
    display.display();
    delay(1200);

    Serial.println("Wearable ready");
}

void loop() {
    sensors_event_t event;
    accel.getEvent(&event);

    float ax = event.acceleration.x;
    float ay = event.acceleration.y;
    float az = event.acceleration.z;

    float mag  = sqrtf(ax*ax + ay*ay + az*az);
    float gVal = mag / 9.80665f;

    Serial.print("X:"); Serial.print(ax, 2);
    Serial.print(" Y:"); Serial.print(ay, 2);
    Serial.print(" Z:"); Serial.print(az, 2);
    Serial.print(" G:"); Serial.println(gVal, 2);

    unsigned long now = millis();

    if (gVal > IMPACT_G && (now - lastImpactMs > COOLDOWN_MS)) {
        lastImpactMs = now;

        Serial.println("BODY IMPACT DETECTED");

        showImpactScreen(gVal);
        sendIR();

        delay(800);
        return;
    }

    if (now - lastDisplayMs > DISPLAY_INTERVAL_MS) {
        lastDisplayMs = now;
        showIdleScreen(ax, ay, az, gVal);
    }

    delay(60);
}
