// ST3020_CalibrateLimits.ino
//
// Calibration utility for the ST3020 (or ST3215) servo.
// Records physical end-stop positions as EEPROM angle limits.
//
// The servo will refuse to move outside the stored range — hardware enforced,
// survives power-off.  Useful for protecting mechanical linkages.
//
// ST3020 tick range: 0 – 4095  (center = tick 2048 = 180°)
//
// Typical calibration workflow:
//   1. Manually move (or jog) the servo to the desired minimum position.
//   2. Send 'l' → current position is saved as the lower limit in EEPROM.
//   3. Move the servo to the desired maximum position.
//   4. Send 'h' → current position is saved as the upper limit in EEPROM.
//   5. Send 't' to verify the servo stops exactly at both limits.
//   6. Send 'r' to restore the full 0–4095 range if needed.
//
// At startup the sketch reads the limits already stored in EEPROM so it
// stays consistent across power cycles.
//
// Serial commands (115200 baud):
//   l  →  Set current position as minimum (lower) limit  [EEPROM write]
//   h  →  Set current position as maximum (upper) limit  [EEPROM write]
//   p  →  Print current position and stored limits
//   t  →  Test: move to min limit, then to max limit
//   r  →  Reset to full range 0 – 4095                  [EEPROM write]
//
// Wiring (ESP32):
//   GPIO17 TX --[1kΩ]--+-- SERVO BUS DATA
//   GPIO18 RX ----------+

#include <Arduino.h>
#include <FeetechBus.h>
#include <FeetechST3020.h>

static constexpr int      TX_PIN   = 17;
static constexpr int      RX_PIN   = 18;
static constexpr uint32_t BAUD     = 1000000;
static constexpr uint8_t  SERVO_ID = 1;   // <-- your servo ID

FeetechBus    bus(Serial2);
FeetechST3020 st(bus, SERVO_ID);

// Mirror of what is stored in EEPROM.
// Initialised from the servo at startup via readAngleLimits().
uint16_t savedMin = 0;
uint16_t savedMax = 4095;

// ---------------------------------------------------------------------------

// Set current physical position as the lower angle limit.
// The servo will stop at this tick on the low side.
void setCurrentAsMin() {
  uint16_t pos = 0;
  if (!st.readPresentPosition(pos)) {
    Serial.println("  ERROR: could not read position");
    return;
  }
  if (pos >= savedMax) {
    Serial.println("  ERROR: current position is at or above max limit");
    Serial.println("         Move the servo towards 0° first, then try again.");
    return;
  }
  savedMin = pos;
  if (!st.setAngleLimits(savedMin, savedMax)) {
    Serial.println("  ERROR: setAngleLimits failed");
    return;
  }
  Serial.print("  Min limit → tick ");
  Serial.print(savedMin);
  Serial.print("  (");
  Serial.print(savedMin * 360.0f / 4095.0f, 1);
  Serial.println("°)  [written to EEPROM]");
}

// Set current physical position as the upper angle limit.
// The servo will stop at this tick on the high side.
void setCurrentAsMax() {
  uint16_t pos = 0;
  if (!st.readPresentPosition(pos)) {
    Serial.println("  ERROR: could not read position");
    return;
  }
  if (pos <= savedMin) {
    Serial.println("  ERROR: current position is at or below min limit");
    Serial.println("         Move the servo towards 360° first, then try again.");
    return;
  }
  savedMax = pos;
  if (!st.setAngleLimits(savedMin, savedMax)) {
    Serial.println("  ERROR: setAngleLimits failed");
    return;
  }
  Serial.print("  Max limit → tick ");
  Serial.print(savedMax);
  Serial.print("  (");
  Serial.print(savedMax * 360.0f / 4095.0f, 1);
  Serial.println("°)  [written to EEPROM]");
}

// ---------------------------------------------------------------------------

void printLimits() {
  uint16_t pos = 0;
  st.readPresentPosition(pos);
  Serial.print("pos="); Serial.print(pos);
  Serial.print("  ("); Serial.print(pos * 360.0f / 4095.0f, 1); Serial.println("°)");
  Serial.print("min="); Serial.print(savedMin);
  Serial.print("  ("); Serial.print(savedMin * 360.0f / 4095.0f, 1); Serial.println("°)");
  Serial.print("max="); Serial.print(savedMax);
  Serial.print("  ("); Serial.print(savedMax * 360.0f / 4095.0f, 1); Serial.println("°)");
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(200);

  bus.beginPins(BAUD, RX_PIN, TX_PIN);

  if (!st.ping()) {
    Serial.println("ERROR: ST3020 not found — check ID, baud, wiring");
    while (true) delay(1000);
  }
  Serial.println("ST3020 found.");

  st.init();

  // Read limits already stored in EEPROM so we stay consistent.
  if (st.readAngleLimits(savedMin, savedMax)) {
    Serial.print("EEPROM limits: min="); Serial.print(savedMin);
    Serial.print("  max="); Serial.println(savedMax);
  } else {
    Serial.println("WARNING: could not read EEPROM limits — using defaults 0 / 4095");
    savedMin = 0;
    savedMax = 4095;
  }

  Serial.println("\n=== ST3020 Calibration Utility — Angle Limits ===");
  Serial.println("  l  set current position as minimum limit  [EEPROM]");
  Serial.println("  h  set current position as maximum limit  [EEPROM]");
  Serial.println("  p  print current position and limits");
  Serial.println("  t  test: move to min limit, then to max limit");
  Serial.println("  r  reset to full range 0–4095  [EEPROM]");
}

// ---------------------------------------------------------------------------

void loop() {
  // Print position every 500 ms
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    uint16_t pos = 0;
    if (st.readPresentPosition(pos)) {
      Serial.print("pos=");
      Serial.print(pos);
      Serial.print("  (");
      Serial.print(pos * 360.0f / 4095.0f, 1);
      Serial.print("°)  limits=[");
      Serial.print(savedMin);
      Serial.print(", ");
      Serial.print(savedMax);
      Serial.println("]");
    }
  }

  if (!Serial.available()) return;
  char cmd = (char)Serial.read();
  while (Serial.available()) Serial.read();  // flush

  switch (cmd) {
    case 'l': case 'L':
      Serial.println("\n[set min limit]");
      setCurrentAsMin();
      break;

    case 'h': case 'H':
      Serial.println("\n[set max limit]");
      setCurrentAsMax();
      break;

    case 'p': case 'P':
      Serial.println();
      printLimits();
      break;

    case 't': case 'T':
      Serial.println("\n[test: move to limits]");
      Serial.print("  → min tick "); Serial.println(savedMin);
      st.moveTime(savedMin, 1200, 500);
      delay(1800);
      Serial.print("  → max tick "); Serial.println(savedMax);
      st.moveTime(savedMax, 1200, 500);
      delay(1800);
      break;

    case 'r': case 'R':
      savedMin = 0;
      savedMax = 4095;
      if (st.setAngleLimits(0, 4095)) {
        Serial.println("\n[reset] Full range 0–4095 (360°) restored  [EEPROM]");
      } else {
        Serial.println("\n[reset] ERROR: EEPROM write failed");
      }
      break;

    default:
      break;
  }
}
