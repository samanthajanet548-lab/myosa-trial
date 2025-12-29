#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Calibration storage
bool hasBaseline = false;
float g0x = 0, g0y = 0, g0z = 0;

// Thresholds (tune if needed)
float goodAngleDeg = 12.0;   // angle from baseline considered GOOD
float badAngleDeg  = 15.0;   // angle considered BAD (hysteresis upper bound)

enum State {UNKNOWN, GOOD, BAD};
State posture = UNKNOWN;

void setup() {
  Serial.begin(115200);
  delay(100);

  // Init MPU at 0x69, fallback to 0x68
  if (!mpu.begin(0x69)) {
    Serial.println("MPU6050 not found at 0x69, trying 0x68...");
    if (!mpu.begin(0x68)) {
      Serial.println("MPU6050 not found. Check wiring.");
      while (1) { delay(10); }
    }
  }
  Serial.println("MPU6050 Found.");

  // Stability settings
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Sit upright and send 'C' in Serial Monitor to calibrate baseline.");
}

void loop() {
  // Listen for calibration command
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'C' || c == 'c') {
      calibrateBaseline();
    }
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  if (hasBaseline) {
    float angleDeg = angleBetweenVectorsDeg(ax, ay, az, g0x, g0y, g0z);

    // Hysteresis decision
    State newPosture = posture;
    if (posture == UNKNOWN) {
      newPosture = (angleDeg <= goodAngleDeg) ? GOOD : BAD;
    } else if (posture == GOOD) {
      if (angleDeg >= badAngleDeg) newPosture = BAD;
    } else { // BAD
      if (angleDeg <= goodAngleDeg) newPosture = GOOD;
    }

    // Only print when state changes
    if (newPosture != posture) {
      posture = newPosture;
      if (posture == GOOD) {
        Serial.println("GOOD");
      } else {
        Serial.println("BAD");
      }
    }
  } else {
    // Before calibration, print once every few seconds
    static unsigned long lastMsg = 0;
    if (millis() - lastMsg > 3000) {
      Serial.println("UNCALI");
      lastMsg = millis();
    }
  }

  delay(100);
}

void calibrateBaseline() {
  // Average a short burst to get a stable baseline
  const int M = 30;
  float sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < M; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sx += a.acceleration.x;
    sy += a.acceleration.y;
    sz += a.acceleration.z;
    delay(20);
  }
  g0x = sx / M;
  g0y = sy / M;
  g0z = sz / M;
  hasBaseline = true;

  Serial.println("Baseline calibrated.");
  posture = UNKNOWN; // reset state
}

float angleBetweenVectorsDeg(float ax, float ay, float az,
                             float bx, float by, float bz) {
  float dot = ax*bx + ay*by + az*bz;
  float ma = sqrt(ax*ax + ay*ay + az*az);
  float mb = sqrt(bx*bx + by*by + bz*bz);
  if (ma == 0 || mb == 0) return NAN;
  float c = dot / (ma * mb);
  if (c > 1.0f) c = 1.0f;
  if (c < -1.0f) c = -1.0f;
  return acos(c) * 180.0f / PI;
}
