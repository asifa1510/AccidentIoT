/*  wearable.ino  — Body-worn impact sensor with ADXL345 + OLED
 *
 *  CHANGE: On impact, OLED now flashes a large "HELP!" screen with
 *  the G-value, making it easier for bystanders to recognise distress.
 *  IR signal logic unchanged.
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define IR_LED        3
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

Adafruit_SSD1306      display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

const float         IMPACT_G    = 1.8f;
const unsigned long COOLDOWN_MS = 5000;

unsigned long lastImpactMs  = 0;
unsigned long lastDisplayMs = 0;

// ── IR pulse burst to vehicle ─────────────────────────────────────────────────
void sendIR() {
  Serial.println("IR SIGNAL SENT");
  for (int i = 0; i < 24; i++) {
    digitalWrite(IR_LED, HIGH); delay(35);
    digitalWrite(IR_LED, LOW);  delay(35);
  }
}

// ── NEW: Flash HELP screen on OLED ────────────────────────────────────────────
// Alternates between white-on-black and black-on-white for 3 seconds
// so the screen itself acts as a visual distress signal.
void flashHelpScreen(float gVal) {
  const int FLASH_COUNT = 6;           // 6 flashes × ~500 ms = ~3 s
  for (int f = 0; f < FLASH_COUNT; f++) {
    bool inv = (f % 2 == 0);
    display.clearDisplay();
    if (inv) {
      display.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }

    // Large "HELP!" text
    display.setTextSize(3);
    display.setCursor(10, 4);
    display.println("HELP!");

    // Smaller sub-line
    display.setTextSize(1);
    display.setCursor(10, 46);
    display.print("ACCIDENT  G=");
    display.print(gVal, 1);

    display.display();
    delay(500);
  }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(IR_LED, OUTPUT);
  digitalWrite(IR_LED, LOW);

  Wire.begin();
  Wire.setClock(400000);
  delay(300);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
    while (true);
  }

  if (!accel.begin()) {
    Serial.println("ADXL345 not detected");
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 20);
    display.println("ADXL ERR");
    display.display();
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

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  sensors_event_t event;
  accel.getEvent(&event);

  float ax = event.acceleration.x;
  float ay = event.acceleration.y;
  float az = event.acceleration.z;
  float mag = sqrtf(ax*ax + ay*ay + az*az);
  float gVal = mag / 9.80665f;

  Serial.printf("X:%.2f Y:%.2f Z:%.2f G:%.2f\n", ax, ay, az, gVal);

  unsigned long now = millis();

  // ── Impact detection ────────────────────────────────────────────────────────
  if (gVal > IMPACT_G && (now - lastImpactMs > COOLDOWN_MS)) {
    lastImpactMs = now;
    Serial.println("BODY IMPACT DETECTED");

    // Send IR signal to vehicle first (time-critical)
    sendIR();

    // Then flash visual distress on OLED (NEW)
    flashHelpScreen(gVal);

    // Settle display after flashing
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(10, 8);
    display.println("IMPACT!");
    display.setTextSize(1);
    display.setCursor(20, 40);
    display.print("G = "); display.println(gVal, 2);
    display.setCursor(20, 54);
    display.println("Help called");
    display.display();
    delay(800);
    return;
  }

  // ── Normal sensor display ───────────────────────────────────────────────────
  if (now - lastDisplayMs > 250) {
    lastDisplayMs = now;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);  display.print("X: "); display.println(ax, 1);
    display.setCursor(0, 14); display.print("Y: "); display.println(ay, 1);
    display.setCursor(0, 28); display.print("Z: "); display.println(az, 1);
    display.setTextSize(2);
    display.setCursor(0, 44);
    display.print("G:"); display.print(gVal, 2);
    display.display();
  }

  delay(60);
}
