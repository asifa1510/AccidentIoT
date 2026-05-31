

#include <Wire.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>
#include <BluetoothSerial.h>
#include "driver/adc.h"

static const int PIN_FUEL_GATE = 23;
static const int PIN_BUZZER    = 18;
static const int PIN_IR        = 35;
static const int PIN_PIEZO     = 34;
static const int I2C_SDA       = 21;
static const int I2C_SCL       = 22;
static const int GPS_RX        = 16;
static const int GPS_TX        = 17;

static const float    SAMPLE_HZ = 200.0f;
static const uint32_t SAMPLE_US = (uint32_t)(1000000.0f / SAMPLE_HZ);

static const uint32_t IMPACT_WINDOW_MS     = 150;
static const uint32_t VALIDATION_WINDOW_MS = 2000;
static const uint32_t LATCH_HOLDOFF_MS     = 10000;
static const uint32_t STABILITY_WINDOW_MS  = 800;
static const uint32_t SPEED_DROP_WINDOW_MS = 3000;

static const float ACC_SENS   = 16384.0f;
static const float GYRO_SENS  = 131.0f;
static const float GRAV_ALPHA = 0.98f;
static const float A_SMOOTH   = 0.80f;

static const float A_MINOR = 2.5f, J_MINOR = 5.0f,  W_MINOR = 250.0f;
static const float A_MOD   = 4.0f, J_MOD   = 8.0f,  W_MOD   = 300.0f;
static const float A_SEV   = 6.0f, J_SEV   = 15.0f, W_SEV   = 500.0f;

static const float TILT_SEVERE_DEG = 75.0f;
static const float TILT_MOD_DEG    = 50.0f;

static const int IR_CHANGE_THRESH = 80;
static const int IR_PULSE_MIN     = 4;

static const float SEAT_HP_ALPHA      = 0.95f;
static const int   SEAT_AVG_SAMPLES   = 40;
static const float SEAT_ENERGY_THRESH = 12.0f;

static const float MIN_SPEED_KMPH = 8.0f;

static const double DEFAULT_LAT = 12.843583;
static const double DEFAULT_LNG = 80.156194;

struct ImpactWindow {
    float    a_peak;
    float    j_peak;
    float    w_peak;
    float    tilt_peak;
    uint32_t start_ms;
};

struct ImpactSnapshot {
    float a, j, w, tilt;
    bool  valid;
};

struct BuzzerJob {
    uint32_t endMs;
    bool     active;
};

struct SpeedHistory {
    static const int SLOTS = 8;
    float    kmph[SLOTS];
    uint32_t ts[SLOTS];
    int      head;
    int      count;

    SpeedHistory() : head(0), count(0) {
        memset(kmph, 0, sizeof(kmph));
        memset(ts,   0, sizeof(ts));
    }

    void push(float v, uint32_t t) {
        kmph[head] = v;
        ts[head]   = t;
        head       = (head + 1) % SLOTS;
        if (count < SLOTS) count++;
    }

    float maxInWindow(uint32_t nowMs, uint32_t windowMs) const {
        float mx = 0;
        for (int i = 0; i < count; i++) {
            int idx = (head - 1 - i + SLOTS) % SLOTS;
            if (nowMs - ts[idx] <= windowMs && kmph[idx] > mx)
                mx = kmph[idx];
        }
        return mx;
    }
};

struct StabilityTracker {
    bool     active;
    uint32_t startMs;
    float    accum;
    int      samples;
    bool     result;

    StabilityTracker() : active(false), startMs(0), accum(0), samples(0), result(false) {}

    void begin(uint32_t now) {
        active  = true;
        startMs = now;
        accum   = 0;
        samples = 0;
        result  = false;
    }

    bool update(float a, uint32_t nowMs) {
        if (!active) return false;
        accum += a;
        samples++;
        if (nowMs - startMs >= STABILITY_WINDOW_MS) {
            active = false;
            float avg = (samples > 0) ? accum / samples : 0;
            result = (avg > 0.8f);
            return result;
        }
        return false;
    }
};

struct MultiImpact {
    static const int      MAX       = 4;
    enum { WINDOW_MS = 5000 };
    uint32_t ts[MAX];
    int      count;

    MultiImpact() : count(0) { memset(ts, 0, sizeof(ts)); }

    void record(uint32_t now) {
        if (count < MAX) ts[count++] = now;
    }

    int countInWindow(uint32_t now) const {
        int n = 0;
        for (int i = 0; i < count; i++)
            if (now - ts[i] <= WINDOW_MS) n++;
        return n;
    }

    void reset() { count = 0; }
};

static MPU6050        mpu;
static TinyGPSPlus    gps;
static BluetoothSerial SerialBT;
static HardwareSerial  gpsSerial(2);

static float gX = 0, gY = 0, gZ = 1;
static float prevA = 0;
static float axBias = 0, ayBias = 0, azBias = 0;

static bool  seatOccupied    = true;
static float seatHP          = 0;
static float seatEnergyAccum = 0;
static float seatEnergy      = 0;
static int   seatCount       = 0;

static float    currentSpeedKmph  = 0;
static uint32_t lastSpeedUpdateMs = 0;

static int      lastIR             = 0;
static int      irPulseCount       = 0;
static bool     waitingWearable    = false;
static uint32_t wearableWindowStart = 0;

static uint32_t lastSampleUs = 0;
static uint32_t lastDbgMs    = 0;
static uint32_t lastTriggerMs = 0;

static ImpactWindow     win;
static ImpactSnapshot   savedPeak;
static BuzzerJob        buzzerJob;
static SpeedHistory     speedHist;
static StabilityTracker stability;
static MultiImpact      multiImpact;

static bool     postImpactUnstable = false;

static inline float lpf(float prev, float x, float alpha) {
    return alpha * prev + (1.0f - alpha) * x;
}

static void setFuel(bool on) {
    digitalWrite(PIN_FUEL_GATE, on ? HIGH : LOW);
}

static void buzzerStart(uint32_t durationMs) {
    digitalWrite(PIN_BUZZER, HIGH);
    buzzerJob.active = true;
    buzzerJob.endMs  = millis() + durationMs;
}

static void buzzerService(uint32_t nowMs) {
    if (buzzerJob.active && nowMs >= buzzerJob.endMs) {
        digitalWrite(PIN_BUZZER, LOW);
        buzzerJob.active = false;
    }
}

static float tiltAngleDeg(float gx, float gy, float gz) {
    float mag = sqrtf(gx*gx + gy*gy + gz*gz);
    if (mag < 0.01f) return 0;
    float c = constrain(gz / mag, -1.0f, 1.0f);
    return degrees(acosf(c));
}

static float adaptiveMultiplier(float speedKmph) {
    if (speedKmph < 20.0f) return 1.6f;
    if (speedKmph < 40.0f) return 1.3f;
    if (speedKmph < 60.0f) return 1.1f;
    return 1.0f;
}

static int computeConfidence(const ImpactSnapshot& s, float speedKmph,
                              bool seatOcc, int impactCount,
                              bool speedDrop, bool unstable) {
    int score = 0;

    if (s.a >= A_SEV)        score += 35;
    else if (s.a >= A_MOD)   score += 25;
    else if (s.a >= A_MINOR) score += 15;

    if (s.j >= J_SEV)        score += 20;
    else if (s.j >= J_MOD)   score += 13;
    else if (s.j >= J_MINOR) score += 7;

    if (s.w >= W_SEV)        score += 15;
    else if (s.w >= W_MOD)   score += 10;
    else if (s.w >= W_MINOR) score += 5;

    if (s.tilt >= TILT_SEVERE_DEG)  score += 15;
    else if (s.tilt >= TILT_MOD_DEG) score += 8;

    if (seatOcc)          score += 5;
    if (speedDrop)        score += 5;
    if (unstable)         score += 3;
    if (impactCount >= 2) score += 5;
    if (speedKmph > 40)   score += 2;

    return min(score, 100);
}

static const char* classifySeverity(const ImpactSnapshot& s) {
    if (s.a >= A_SEV || s.j >= J_SEV || s.w >= W_SEV || s.tilt >= TILT_SEVERE_DEG)
        return "SEVERE";
    if (s.a >= A_MOD || s.j >= J_MOD || s.w >= W_MOD || s.tilt >= TILT_MOD_DEG)
        return "MODERATE";
    if (s.a >= A_MINOR || s.j >= J_MINOR || s.w >= W_MINOR)
        return "MINOR";
    return "NONE";
}

static bool detectSpeedDrop(uint32_t nowMs) {
    float prevMax = speedHist.maxInWindow(nowMs, SPEED_DROP_WINDOW_MS);
    if (prevMax < MIN_SPEED_KMPH) return false;
    float drop = prevMax - currentSpeedKmph;
    return (drop / prevMax) > 0.35f;
}

static void sendBTAccident(double lat, double lng) {
    String msg = "ACCIDENT," + String(lat, 6) + "," + String(lng, 6);
    SerialBT.println(msg);
    Serial.println("BT -> " + msg);
}

static void emitAccident(const ImpactSnapshot& s, int confidence) {
    const char* sev = classifySeverity(s);
    double lat = gps.location.isValid() ? gps.location.lat() : DEFAULT_LAT;
    double lng = gps.location.isValid() ? gps.location.lng() : DEFAULT_LNG;

    Serial.printf("ACCIDENT,%s,conf=%d,a=%.2f,j=%.1f,w=%.0f,tilt=%.1f,%.6f,%.6f\n",
                  sev, confidence, s.a, s.j, s.w, s.tilt, lat, lng);

    setFuel(false);
    buzzerStart(1000);
    sendBTAccident(lat, lng);
}

static void checkWearableIR() {
    int val = analogRead(PIN_IR);
    if (waitingWearable && abs(val - lastIR) > IR_CHANGE_THRESH)
        irPulseCount++;
    lastIR = val;
}

static void feedGPS(uint32_t nowMs) {
    while (gpsSerial.available())
        gps.encode(gpsSerial.read());

    if (gps.speed.isValid() && gps.speed.isUpdated()) {
        currentSpeedKmph  = (float)gps.speed.kmph();
        lastSpeedUpdateMs = nowMs;
        speedHist.push(currentSpeedKmph, nowMs);
    }
}

static void calibrateMPU() {
    Serial.println("Calibrating MPU - keep still");
    long sx = 0, sy = 0, sz = 0;
    const int N = 200;
    for (int i = 0; i < N; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        sx += ax; sy += ay; sz += az;
        delay(5);
    }
    axBias = (float)sx / N;
    ayBias = (float)sy / N;
    azBias = (float)sz / N;
}

void setup() {
    Serial.begin(115200);
    SerialBT.begin("ESP32_Accident_BT");

    pinMode(PIN_BUZZER,    OUTPUT);
    pinMode(PIN_FUEL_GATE, OUTPUT);
    pinMode(PIN_IR,        INPUT);

    digitalWrite(PIN_BUZZER,    LOW);
    setFuel(true);

    analogSetPinAttenuation(PIN_PIEZO, ADC_11db);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 not found");
        while (true);
    }

    calibrateMPU();

    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

    win       = { 0, 0, 0, 0, millis() };
    savedPeak = { 0, 0, 0, 0, false };
    buzzerJob = { 0, false };

    lastSampleUs = micros();
    Serial.println("System ready");
}

void loop() {
    uint32_t nowUs = micros();
    uint32_t nowMs = millis();

    feedGPS(nowMs);
    buzzerService(nowMs);

    if (nowUs - lastSampleUs < SAMPLE_US) return;
    lastSampleUs += SAMPLE_US;

    float dt = SAMPLE_US / 1e6f;

    int16_t axr, ayr, azr, gxr, gyr, gzr;
    mpu.getMotion6(&axr, &ayr, &azr, &gxr, &gyr, &gzr);

    float ax = (axr - axBias) / ACC_SENS;
    float ay = (ayr - ayBias) / ACC_SENS;
    float az = (azr - azBias) / ACC_SENS;

    float gx = gxr / GYRO_SENS;
    float gy = gyr / GYRO_SENS;
    float gz = gzr / GYRO_SENS;

    float w = sqrtf(gx*gx + gy*gy + gz*gz);

    gX = lpf(gX, ax, GRAV_ALPHA);
    gY = lpf(gY, ay, GRAV_ALPHA);
    gZ = lpf(gZ, az, GRAV_ALPHA);

    float linX = ax - gX;
    float linY = ay - gY;
    float linZ = az - gZ;

    float a_raw    = sqrtf(linX*linX + linY*linY + linZ*linZ);
    float a_smooth = lpf(prevA, a_raw, A_SMOOTH);

    float jerk = (a_smooth - prevA) / dt;
    prevA = a_smooth;

    float tilt = tiltAngleDeg(gX, gY, gZ);

    static int prevSeatRaw = 0;
    int seatRaw = analogRead(PIN_PIEZO);
    int dx      = seatRaw - prevSeatRaw;
    prevSeatRaw = seatRaw;

    seatHP           = SEAT_HP_ALPHA * (seatHP + (float)dx);
    seatEnergyAccum += fabsf(seatHP);
    seatCount++;

    if (seatCount >= SEAT_AVG_SAMPLES) {
        seatEnergy      = seatEnergyAccum / seatCount;
        seatEnergyAccum = 0;
        seatCount       = 0;
        seatOccupied    = (seatEnergy >= SEAT_ENERGY_THRESH);
    }

    checkWearableIR();

    if (stability.update(a_smooth, nowMs))
        postImpactUnstable = stability.result;

    if (a_smooth    > win.a_peak)  win.a_peak    = a_smooth;
    if (fabsf(jerk) > win.j_peak)  win.j_peak    = fabsf(jerk);
    if (w           > win.w_peak)  win.w_peak    = w;
    if (tilt        > win.tilt_peak) win.tilt_peak = tilt;

    if (nowMs - win.start_ms > IMPACT_WINDOW_MS) {

        float mult = adaptiveMultiplier(currentSpeedKmph);

        bool threshMet =
            (win.a_peak    >= A_MINOR * mult) ||
            (win.j_peak    >= J_MINOR * mult) ||
            (win.w_peak    >= W_MINOR)         ||
            (win.tilt_peak >= TILT_MOD_DEG);

        bool hardBrakeOnly =
            (win.a_peak >= A_MINOR * mult) &&
            (win.j_peak <  J_MINOR)        &&
            (win.w_peak <  50.0f)          &&
            (win.tilt_peak < 10.0f);

        if (hardBrakeOnly && currentSpeedKmph < 30.0f)
            threshMet = false;

        if (threshMet &&
            !waitingWearable &&
            seatOccupied &&
            currentSpeedKmph >= MIN_SPEED_KMPH &&
            (nowMs - lastTriggerMs > LATCH_HOLDOFF_MS))
        {
            
            savedPeak = { win.a_peak, win.j_peak, win.w_peak, win.tilt_peak, true };
            multiImpact.record(nowMs);
            postImpactUnstable = false;
            stability.begin(nowMs);

            Serial.println("Vehicle impact detected");
            waitingWearable      = true;
            irPulseCount         = 0;
            wearableWindowStart  = nowMs;
        }

        win = { 0, 0, 0, 0, nowMs };
    }

    if (waitingWearable && (nowMs - wearableWindowStart > VALIDATION_WINDOW_MS)) {

        if (irPulseCount >= IR_PULSE_MIN && savedPeak.valid) {

            bool speedDrop = detectSpeedDrop(nowMs);
            int  impacts   = multiImpact.countInWindow(nowMs);
            int  conf      = computeConfidence(savedPeak, currentSpeedKmph,
                                               seatOccupied, impacts,
                                               speedDrop, postImpactUnstable);

            Serial.printf("Wearable confirmed - conf=%d impacts=%d drop=%d unstable=%d\n",
                          conf, impacts, (int)speedDrop, (int)postImpactUnstable);

            lastTriggerMs = nowMs;
            emitAccident(savedPeak, conf);

        } else {
            Serial.println("Wearable not detected - ignoring bump");
        }

        savedPeak.valid = false;
        waitingWearable = false;
    }

    if (nowMs - lastDbgMs > 500) {
        lastDbgMs = nowMs;
        Serial.printf("a=%.2fg j=%5.1fg/s w=%3.0fdps tilt=%4.1fdeg spd=%.1fkm/h seat=%c IR=%d\n",
                      a_smooth, jerk, w, tilt, currentSpeedKmph,
                      seatOccupied ? 'Y' : 'N', irPulseCount);
    }
}
