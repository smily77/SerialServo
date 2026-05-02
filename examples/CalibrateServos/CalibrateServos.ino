// CalibrateServos.ino
//
// Combined calibration utility for ST3020 + SC15 on a shared bus.
//
// Three calibration functions — all write to EEPROM, survive power-off:
//
//   setCenterST()  — ST3020 / ST3215
//     Uses setPositionOffset() to shift the encoder zero-point so that
//     the current physical position is reported as tick 2048 (= 180°, center).
//     After this call, wherever the servo is physically, that becomes the new
//     "180°" reference.  Reset with offset = 0 to restore the factory zero.
//
//   setMinSC()     — SC15
//     Uses setAngleLimits() to save the current position as the lower
//     (minimum) angle limit.  The servo will refuse to move below this tick.
//
//   setMaxSC()     — SC15
//     Uses setAngleLimits() to save the current position as the upper
//     (maximum) angle limit.  The servo will refuse to move above this tick.
//
// Typical workflow:
//   ST3020  →  move servo to the desired neutral / center position, send 'c'
//   SC15    →  move servo to the desired minimum stop, send 'l'
//              move servo to the desired maximum stop, send 'h'
//
// Serial commands (115200 baud):
//   c  →  ST3020: store current position as tick 2048 (180°, center)
//   l  →  SC15:   store current position as minimum angle limit
//   h  →  SC15:   store current position as maximum angle limit
//   p  →  print current positions and stored limits for both servos
//   t  →  test: move ST3020 to 2048; move SC15 to min, then to max
//   r  →  reset all (ST3020 offset = 0; SC15 limits = 0 – 1023)
//
// Wiring (ESP32):
//   GPIO17 TX --[1kΩ]--+-- SERVO BUS DATA
//   GPIO18 RX ----------+
//   Both servos on the same bus with unique IDs.

#include <Arduino.h>
#include <FeetechBus.h>
#include <FeetechST3020.h>
#include <FeetechSC15.h>

static constexpr int      TX_PIN    = 17;
static constexpr int      RX_PIN    = 18;
static constexpr uint32_t BAUD      = 1000000;
static constexpr uint8_t  ST_ID     = 1;   // <-- ST3020 servo ID
static constexpr uint8_t  SC_ID     = 2;   // <-- SC15  servo ID

FeetechBus    bus(Serial2);
FeetechST3020 st(bus, ST_ID);
FeetechSC15   sc(bus, SC_ID);

// Local mirror of the SC15 angle limits (read from EEPROM at startup).
uint16_t scMin = 0;
uint16_t scMax = 1023;

// ---------------------------------------------------------------------------
// Calibration functions
// ---------------------------------------------------------------------------

// ST3020: write a position offset so the current physical position
// reports as tick 2048 (= 180°, the center of the 0–4095 range).
void setCenterST() {
  uint16_t pos = 0;
  if (!st.readPresentPosition(pos)) {
    Serial.println("  ERROR: could not read ST3020 position");
    return;
  }
  int16_t offset = (int16_t)2048 - (int16_t)pos;
  if (!st.setPositionOffset(offset)) {
    Serial.println("  ERROR: setPositionOffset failed");
    return;
  }
  Serial.print("  ST3020 center calibrated: physical tick ");
  Serial.print(pos);
  Serial.print(" now reports as 2048 (180°)  offset=");
  Serial.print(offset);
  Serial.println("  [EEPROM]");
}

// SC15: store the current position as the lower angle limit.
void setMinSC() {
  uint16_t pos = 0;
  if (!sc.readPresentPosition(pos)) {
    Serial.println("  ERROR: could not read SC15 position");
    return;
  }
  if (pos >= scMax) {
    Serial.println("  ERROR: current position is at or above the max limit");
    Serial.println("         Move the SC15 towards 0° first.");
    return;
  }
  scMin = pos;
  if (!sc.setAngleLimits(scMin, scMax)) {
    Serial.println("  ERROR: setAngleLimits failed");
    return;
  }
  Serial.print("  SC15 min limit → tick ");
  Serial.print(scMin);
  Serial.print("  (");
  Serial.print(scMin * 180.0f / 1023.0f, 1);
  Serial.println("°)  [EEPROM]");
}

// SC15: store the current position as the upper angle limit.
void setMaxSC() {
  uint16_t pos = 0;
  if (!sc.readPresentPosition(pos)) {
    Serial.println("  ERROR: could not read SC15 position");
    return;
  }
  if (pos <= scMin) {
    Serial.println("  ERROR: current position is at or below the min limit");
    Serial.println("         Move the SC15 towards 180° first.");
    return;
  }
  scMax = pos;
  if (!sc.setAngleLimits(scMin, scMax)) {
    Serial.println("  ERROR: setAngleLimits failed");
    return;
  }
  Serial.print("  SC15 max limit → tick ");
  Serial.print(scMax);
  Serial.print("  (");
  Serial.print(scMax * 180.0f / 1023.0f, 1);
  Serial.println("°)  [EEPROM]");
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void printStatus() {
  uint16_t stPos = 0, scPos = 0;
  st.readPresentPosition(stPos);
  sc.readPresentPosition(scPos);

  Serial.println();
  Serial.print("ST3020  pos="); Serial.print(stPos);
  Serial.print("  ("); Serial.print(stPos * 360.0f / 4095.0f, 1); Serial.println("°)");
  Serial.print("        center=2048 after calibration");
  Serial.println();

  Serial.print("SC15    pos="); Serial.print(scPos);
  Serial.print("  ("); Serial.print(scPos * 180.0f / 1023.0f, 1); Serial.println("°)");
  Serial.print("        min="); Serial.print(scMin);
  Serial.print("  ("); Serial.print(scMin * 180.0f / 1023.0f, 1); Serial.print("°)");
  Serial.print("  max="); Serial.print(scMax);
  Serial.print("  ("); Serial.print(scMax * 180.0f / 1023.0f, 1); Serial.println("°)");
}

void resetAll() {
  if (st.setPositionOffset(0)) {
    Serial.println("  ST3020 offset reset to 0  [EEPROM]");
  } else {
    Serial.println("  ERROR: ST3020 setPositionOffset(0) failed");
  }
  scMin = 0;
  scMax = 1023;
  if (sc.setAngleLimits(0, 1023)) {
    Serial.println("  SC15 limits reset to 0–1023 (full range)  [EEPROM]");
  } else {
    Serial.println("  ERROR: SC15 setAngleLimits failed");
  }
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(200);

  bus.beginPins(BAUD, RX_PIN, TX_PIN);

  bool stOk = st.ping();
  bool scOk = sc.ping();
  Serial.print("ST3020 (ID "); Serial.print(ST_ID); Serial.print("): ");
  Serial.println(stOk ? "found" : "NOT FOUND");
  Serial.print("SC15   (ID "); Serial.print(SC_ID); Serial.print("): ");
  Serial.println(scOk ? "found" : "NOT FOUND");

  if (!stOk || !scOk) {
    Serial.println("Check ID, baud, wiring.  Continuing anyway.");
  }

  FeetechST3020::Profile pSt;
  pSt.returnDelayUnits  = 0;
  pSt.torqueOnAfterInit = true;

  FeetechSC15::Profile pSc;
  pSc.returnDelayUnits  = 4;   // stagger replies on shared bus
  pSc.torqueOnAfterInit = true;

  if (stOk) st.init(pSt);
  if (scOk) {
    sc.init(pSc);
    // Read SC15 limits already in EEPROM so our local mirror is accurate.
    if (!sc.readAngleLimits(scMin, scMax)) {
      scMin = 0;  scMax = 1023;  // fallback
    }
  }

  Serial.println("\n=== Servo Calibration Utility ===");
  Serial.println("  c  ST3020: set current position as center (tick 2048 = 180°)");
  Serial.println("  l  SC15:   set current position as minimum angle limit");
  Serial.println("  h  SC15:   set current position as maximum angle limit");
  Serial.println("  p  print current positions and limits");
  Serial.println("  t  test moves");
  Serial.println("  r  reset all (ST3020 offset=0, SC15 full range)");

  printStatus();
}

// ---------------------------------------------------------------------------

void loop() {
  // Live position readout every 500 ms
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    uint16_t stPos = 0, scPos = 0;
    st.readPresentPosition(stPos);
    sc.readPresentPosition(scPos);
    Serial.print("ST="); Serial.print(stPos);
    Serial.print("("); Serial.print(stPos * 360.0f / 4095.0f, 1); Serial.print("°)  ");
    Serial.print("SC="); Serial.print(scPos);
    Serial.print("("); Serial.print(scPos * 180.0f / 1023.0f, 1); Serial.print("°)");
    Serial.print("  SC-limits=["); Serial.print(scMin); Serial.print(","); Serial.print(scMax); Serial.println("]");
  }

  if (!Serial.available()) return;
  char cmd = (char)Serial.read();
  while (Serial.available()) Serial.read();

  switch (cmd) {
    case 'c': case 'C':
      Serial.println("\n[ST3020 → center at 2048]");
      setCenterST();
      break;

    case 'l': case 'L':
      Serial.println("\n[SC15 → set min limit]");
      setMinSC();
      break;

    case 'h': case 'H':
      Serial.println("\n[SC15 → set max limit]");
      setMaxSC();
      break;

    case 'p': case 'P':
      printStatus();
      break;

    case 't': case 'T':
      Serial.println("\n[test]");
      Serial.println("  ST3020 → tick 2048 (calibrated center)");
      st.moveTime(2048, 1000, 600);
      delay(1500);
      Serial.print("  SC15 → min tick "); Serial.println(scMin);
      sc.moveTime(scMin, 1000, 400);
      delay(1500);
      Serial.print("  SC15 → max tick "); Serial.println(scMax);
      sc.moveTime(scMax, 1000, 400);
      delay(1500);
      break;

    case 'r': case 'R':
      Serial.println("\n[reset all]");
      resetAll();
      break;

    default:
      break;
  }
}
