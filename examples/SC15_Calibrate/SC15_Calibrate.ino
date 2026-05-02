// SC15_Calibrate.ino
//
// Software calibration utility for the SC15 servo.
//
// The SC15 uses the SCS protocol which has NO hardware position-offset register.
// (The STS protocol used by ST3020/ST3215 does have one.)
// Calibration is therefore done in software: the current position is recorded
// as a reference point, and all subsequent moves are expressed relative to it.
//
// SC15 tick range: 0 – 1023  (hardware center = tick 512 = 90°)
//
// What 'calibrateCenter' does:
//   Reads the current position and stores it as homeTicks.
//   After that, moveFromHome(deg) moves the servo by ±deg from that reference.
//   Example: if the servo is physically centred at its mechanism's mid-point and
//   you call calibrateCenter(), the servo will treat that position as its origin
//   regardless of what tick number it happens to be at.
//
// Note: homeTicks lives in RAM only — it resets to 512 on every power-on.
//       If you need a persistent offset, use the ST3020 which supports
//       st.setPositionOffset() in EEPROM.
//
// Serial commands (115200 baud):
//   c  →  Store current position as home / center reference
//   p  →  Print current position and home reference
//   t  →  Test sweep: −60° → home → +60° relative to home
//   r  →  Reset home to tick 512 (90°, factory center)
//
// Wiring (ESP32):
//   GPIO17 TX --[1kΩ]--+-- SERVO BUS DATA
//   GPIO18 RX ----------+

#include <Arduino.h>
#include <FeetechBus.h>
#include <FeetechSC15.h>

static constexpr int      TX_PIN   = 17;
static constexpr int      RX_PIN   = 18;
static constexpr uint32_t BAUD     = 1000000;
static constexpr uint8_t  SERVO_ID = 1;   // <-- your servo ID

FeetechBus  bus(Serial2);
FeetechSC15 sc(bus, SERVO_ID);

// Software reference point.  Moves are relative to this tick value.
// Default: tick 512 = 90° = hardware center of the SC15's 0–180° range.
uint16_t homeTicks = 512;

// ---------------------------------------------------------------------------

// Record the current physical position as the home / center reference.
void calibrateCenter() {
  uint16_t pos = 0;
  if (!sc.readPresentPosition(pos)) {
    Serial.println("  ERROR: could not read position");
    return;
  }
  homeTicks = pos;
  Serial.print("  Home set → tick ");
  Serial.print(homeTicks);
  Serial.print("  (");
  Serial.print(homeTicks * 180.0f / 1023.0f, 1);
  Serial.println("°)");
}

// Move the servo by deltaDeg degrees relative to the stored home reference.
// Positive = towards 180°, negative = towards 0°.  Clamped to 0–180°.
void moveFromHome(float deltaDeg) {
  float homeDeg   = homeTicks * 180.0f / 1023.0f;
  float targetDeg = homeDeg + deltaDeg;
  if (targetDeg < 0)      targetDeg = 0;
  if (targetDeg > 180.0f) targetDeg = 180.0f;

  uint16_t ticks = sc.degToTicks(targetDeg);
  Serial.print("  → ");
  if (deltaDeg >= 0) Serial.print("+");
  Serial.print(deltaDeg, 0);
  Serial.print("° from home  (tick ");
  Serial.print(ticks);
  Serial.println(")");
  sc.moveTime(ticks, 700, 500);
  delay(900);
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(200);

  bus.beginPins(BAUD, RX_PIN, TX_PIN);

  if (!sc.ping()) {
    Serial.println("ERROR: SC15 not found — check ID, baud, wiring");
    while (true) delay(1000);
  }
  Serial.println("SC15 found.");

  sc.init();

  Serial.println("\n=== SC15 Calibration Utility ===");
  Serial.println("  c  calibrate: store current position as home / center reference");
  Serial.println("  p  print current position and home reference");
  Serial.println("  t  test: sweep -60° / home / +60° relative to home");
  Serial.println("  r  reset home to tick 512 (90°, factory center)");
  Serial.println();
  Serial.print("home = tick 512 (90°, factory center)");
  Serial.println("  — send 'c' to recalibrate to current position");
}

// ---------------------------------------------------------------------------

void loop() {
  // Continuously print position every 500 ms
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    uint16_t pos = 0;
    if (sc.readPresentPosition(pos)) {
      int16_t delta = (int16_t)pos - (int16_t)homeTicks;
      Serial.print("pos=");
      Serial.print(pos);
      Serial.print("  (");
      Serial.print(pos * 180.0f / 1023.0f, 1);
      Serial.print("°)  offset_from_home=");
      Serial.println(delta);
    }
  }

  if (!Serial.available()) return;
  char cmd = (char)Serial.read();
  while (Serial.available()) Serial.read();  // flush remainder

  switch (cmd) {
    case 'c': case 'C':
      Serial.println("\n[calibrate]");
      calibrateCenter();
      break;

    case 'p': case 'P': {
      uint16_t pos = 0;
      sc.readPresentPosition(pos);
      Serial.print("\npos="); Serial.print(pos);
      Serial.print("  ("); Serial.print(pos * 180.0f / 1023.0f, 1); Serial.println("°)");
      Serial.print("home="); Serial.print(homeTicks);
      Serial.print("  ("); Serial.print(homeTicks * 180.0f / 1023.0f, 1); Serial.println("°)");
      Serial.print("delta="); Serial.println((int16_t)pos - (int16_t)homeTicks);
      break;
    }

    case 't': case 'T':
      Serial.println("\n[test sweep relative to home]");
      moveFromHome(-60.0f);
      delay(300);
      moveFromHome(  0.0f);   // return to home
      delay(300);
      moveFromHome(+60.0f);
      delay(300);
      moveFromHome(  0.0f);   // return to home
      break;

    case 'r': case 'R':
      homeTicks = 512;
      Serial.println("\n[reset] home = tick 512 (90°, factory center)");
      break;

    default:
      break;
  }
}
