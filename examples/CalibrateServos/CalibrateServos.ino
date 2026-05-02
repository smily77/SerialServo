// CalibrateServos.ino
//
// Combined calibration utility for ST3020 + SC15 on a shared bus.
//
// Three calibration functions — all write to EEPROM, survive power-off:
//
//   setCenterST()  — ST3020 / ST3215
//     Uses setPositionOffset() to shift the encoder zero-point so that
//     the current physical position is reported as tick 2048 (= 180°, center).
//
//   setMinSC()     — SC15
//     Uses setAngleLimits() to save the current position as the lower
//     (minimum) angle limit.  The servo will refuse to move below this tick.
//
//   setMaxSC()     — SC15
//     Uses setAngleLimits() to save the current position as the upper
//     (maximum) angle limit.  The servo will refuse to move above this tick.
//
// Serial commands (115200 baud) — send as newline-terminated lines:
//   c          ST3020: set current position as center (tick 2048 = 180°)  [EEPROM]
//   l          SC15:   set current position as minimum angle limit         [EEPROM]
//   h          SC15:   set current position as maximum angle limit         [EEPROM]
//   p          print full status for both servos
//   t          test moves (ST3020 → 2048; SC15 → min, then max)
//   r          reset all (ST3020 offset=0; SC15 limits=0–1023; dead zones=0)
//   mST <n>    move ST3020 by +/- n ticks relative to current position
//   mSC <n>    move SC15  by +/- n ticks relative to current position
//   dST <n>    set ST3020 dead zone to n ticks (symmetric CW+CCW)  [EEPROM]
//   dSC <n>    set SC15  dead zone to n ticks (symmetric CW+CCW)   [EEPROM]
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
// Per-servo status prints (called after every relevant command)
// ---------------------------------------------------------------------------

void printStatusST() {
  uint16_t pos = 0;
  uint8_t  cwD = 0, ccwD = 0;
  st.readPresentPosition(pos);
  st.readDeadBand(cwD, ccwD);
  Serial.print("  ST3020  pos="); Serial.print(pos);
  Serial.print(" ("); Serial.print(pos * 360.0f / 4095.0f, 1); Serial.print("°)");
  Serial.print("  dead=[CW:"); Serial.print(cwD);
  Serial.print(" CCW:"); Serial.print(ccwD); Serial.println("]");
}

void printStatusSC() {
  uint16_t pos = 0;
  uint8_t  cwD = 0, ccwD = 0;
  sc.readPresentPosition(pos);
  sc.readDeadBand(cwD, ccwD);
  Serial.print("  SC15    pos="); Serial.print(pos);
  Serial.print(" ("); Serial.print(pos * 180.0f / 1023.0f, 1); Serial.print("°)");
  Serial.print("  min="); Serial.print(scMin);
  Serial.print(" ("); Serial.print(scMin * 180.0f / 1023.0f, 1); Serial.print("°)");
  Serial.print("  max="); Serial.print(scMax);
  Serial.print(" ("); Serial.print(scMax * 180.0f / 1023.0f, 1); Serial.print("°)");
  Serial.print("  dead=[CW:"); Serial.print(cwD);
  Serial.print(" CCW:"); Serial.print(ccwD); Serial.println("]");
}

void printStatus() {
  Serial.println();
  printStatusST();
  printStatusSC();
}

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
  // Confirm by reading back the new reported position.
  uint16_t newPos = 0;
  st.readPresentPosition(newPos);
  Serial.print("  ST3020 center set: physical tick "); Serial.print(pos);
  Serial.print(" now reports as "); Serial.print(newPos);
  Serial.print(" (offset="); Serial.print(offset); Serial.println(")  [EEPROM]");
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
    uint16_t rMin = 0, rMax = 0;
    sc.readAngleLimits(rMin, rMax);
    Serial.print("  ERROR: setAngleLimits failed  (EEPROM reads back min=");
    Serial.print(rMin); Serial.print(" max="); Serial.print(rMax); Serial.println(")");
    return;
  }
  Serial.print("  SC15 min limit → tick "); Serial.print(scMin);
  Serial.print(" ("); Serial.print(scMin * 180.0f / 1023.0f, 1); Serial.println("°)  [EEPROM]");
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
    uint16_t rMin = 0, rMax = 0;
    sc.readAngleLimits(rMin, rMax);
    Serial.print("  ERROR: setAngleLimits failed  (EEPROM reads back min=");
    Serial.print(rMin); Serial.print(" max="); Serial.print(rMax); Serial.println(")");
    return;
  }
  Serial.print("  SC15 max limit → tick "); Serial.print(scMax);
  Serial.print(" ("); Serial.print(scMax * 180.0f / 1023.0f, 1); Serial.println("°)  [EEPROM]");
}

// ---------------------------------------------------------------------------
// Move helpers (relative, in ticks)
// ---------------------------------------------------------------------------

void moveST(int32_t delta) {
  uint16_t pos = 0;
  if (!st.readPresentPosition(pos)) {
    Serial.println("  ERROR: could not read ST3020 position");
    return;
  }
  uint16_t newPos = (uint16_t)constrain((int32_t)pos + delta, 0, 4095);
  st.moveTime(newPos, 800, 300);
  Serial.print("  ST3020: "); Serial.print(pos);
  Serial.print(" → "); Serial.print(newPos);
  Serial.print(" ("); Serial.print(newPos * 360.0f / 4095.0f, 1); Serial.println("°)");
}

void moveSC(int32_t delta) {
  uint16_t pos = 0;
  if (!sc.readPresentPosition(pos)) {
    Serial.println("  ERROR: could not read SC15 position");
    return;
  }
  uint16_t newPos = (uint16_t)constrain((int32_t)pos + delta, (int32_t)scMin, (int32_t)scMax);
  sc.moveTime(newPos, 800, 200);
  Serial.print("  SC15:   "); Serial.print(pos);
  Serial.print(" → "); Serial.print(newPos);
  Serial.print(" ("); Serial.print(newPos * 180.0f / 1023.0f, 1); Serial.println("°)");
}

// ---------------------------------------------------------------------------
// Dead zone helpers
// ---------------------------------------------------------------------------

void setDeadZoneST(uint8_t ticks) {
  if (!st.setDeadBand(ticks, ticks)) {
    uint8_t rCw = 0, rCcw = 0;
    st.readDeadBand(rCw, rCcw);
    Serial.print("  ERROR: setDeadBand failed  (EEPROM reads back CW=");
    Serial.print(rCw); Serial.print(" CCW="); Serial.print(rCcw); Serial.println(")");
    return;
  }
  Serial.print("  ST3020 dead zone → "); Serial.print(ticks); Serial.println(" ticks  [EEPROM]");
}

void setDeadZoneSC(uint8_t ticks) {
  if (!sc.setDeadBand(ticks, ticks)) {
    uint8_t rCw = 0, rCcw = 0;
    sc.readDeadBand(rCw, rCcw);
    Serial.print("  ERROR: setDeadBand failed  (EEPROM reads back CW=");
    Serial.print(rCw); Serial.print(" CCW="); Serial.print(rCcw); Serial.println(")");
    return;
  }
  Serial.print("  SC15   dead zone → "); Serial.print(ticks); Serial.println(" ticks  [EEPROM]");
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

void resetAll() {
  if (st.setPositionOffset(0))
    Serial.println("  ST3020 offset reset to 0  [EEPROM]");
  else
    Serial.println("  ERROR: ST3020 setPositionOffset(0) failed");

  if (st.setDeadBand(0, 0))
    Serial.println("  ST3020 dead zone reset to 0  [EEPROM]");
  else
    Serial.println("  ERROR: ST3020 setDeadBand(0,0) failed");

  scMin = 0;  scMax = 1023;
  if (sc.setAngleLimits(0, 1023))
    Serial.println("  SC15 limits reset to 0–1023 (full range)  [EEPROM]");
  else
    Serial.println("  ERROR: SC15 setAngleLimits failed");

  if (sc.setDeadBand(0, 0))
    Serial.println("  SC15 dead zone reset to 0  [EEPROM]");
  else
    Serial.println("  ERROR: SC15 setDeadBand(0,0) failed");
}

// ---------------------------------------------------------------------------
// Command dispatcher
// ---------------------------------------------------------------------------

void processCommand(const char* line) {
  // Single-character commands
  if (line[0] != '\0' && line[1] == '\0') {
    switch (line[0]) {
      case 'c': case 'C':
        Serial.println("\n[ST3020 → center at 2048]");
        setCenterST();
        printStatusST();
        return;
      case 'l': case 'L':
        Serial.println("\n[SC15 → set min limit]");
        setMinSC();
        printStatusSC();
        return;
      case 'h': case 'H':
        Serial.println("\n[SC15 → set max limit]");
        setMaxSC();
        printStatusSC();
        return;
      case 'p': case 'P':
        printStatus();
        return;
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
        printStatus();
        return;
      case 'r': case 'R':
        Serial.println("\n[reset all]");
        resetAll();
        printStatus();
        return;
      default:
        break;
    }
  }

  // Multi-token commands: "mST <n>", "mSC <n>", "dST <n>", "dSC <n>"
  const char* sp = strchr(line, ' ');
  if (!sp || sp == line) {
    Serial.print("  unknown: "); Serial.println(line);
    return;
  }
  int32_t val = (int32_t)atol(sp + 1);

  if (strncasecmp(line, "mST", 3) == 0) {
    Serial.print("\n[mST "); Serial.print(val); Serial.println("]");
    moveST(val);
    printStatusST();

  } else if (strncasecmp(line, "mSC", 3) == 0) {
    Serial.print("\n[mSC "); Serial.print(val); Serial.println("]");
    moveSC(val);
    printStatusSC();

  } else if (strncasecmp(line, "dST", 3) == 0) {
    if (val < 0 || val > 255) { Serial.println("  ERROR: dead zone must be 0–255"); return; }
    Serial.print("\n[dST "); Serial.print(val); Serial.println("]");
    setDeadZoneST((uint8_t)val);
    printStatusST();

  } else if (strncasecmp(line, "dSC", 3) == 0) {
    if (val < 0 || val > 255) { Serial.println("  ERROR: dead zone must be 0–255"); return; }
    Serial.print("\n[dSC "); Serial.print(val); Serial.println("]");
    setDeadZoneSC((uint8_t)val);
    printStatusSC();

  } else {
    Serial.print("  unknown: "); Serial.println(line);
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
  if (!stOk || !scOk) Serial.println("Check ID, baud, wiring.  Continuing anyway.");

  FeetechST3020::Profile pSt;
  pSt.returnDelayUnits  = 0;
  pSt.torqueOnAfterInit = true;

  FeetechSC15::Profile pSc;
  pSc.returnDelayUnits  = 4;   // stagger replies on shared bus
  pSc.torqueOnAfterInit = true;

  if (stOk) st.init(pSt);
  if (scOk) {
    sc.init(pSc);
    // Load limits already stored in SC15 EEPROM so our local mirror is accurate.
    if (!sc.readAngleLimits(scMin, scMax)) { scMin = 0; scMax = 1023; }
  }

  Serial.println("\n=== Servo Calibration Utility ===");
  Serial.println("  c          ST3020: set current pos as center (2048 = 180°)  [EEPROM]");
  Serial.println("  l          SC15:   set current pos as min angle limit        [EEPROM]");
  Serial.println("  h          SC15:   set current pos as max angle limit        [EEPROM]");
  Serial.println("  mST <n>    ST3020: relative move +/- n ticks");
  Serial.println("  mSC <n>    SC15:   relative move +/- n ticks");
  Serial.println("  dST <n>    ST3020: set dead zone to n ticks                  [EEPROM]");
  Serial.println("  dSC <n>    SC15:   set dead zone to n ticks                  [EEPROM]");
  Serial.println("  p          print full status (pos, limits, dead zones)");
  Serial.println("  t          test moves");
  Serial.println("  r          reset all to defaults");

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

  // Line-buffered command input.
  // Both '\n' and '\r' act as terminators so the sketch works regardless of
  // whether the Serial Monitor is set to "Newline", "Carriage return", or
  // "Both NL & CR".  An empty line (linePos==0) is silently ignored, which
  // prevents double-firing for "\r\n".
  static char    lineBuf[32];
  static uint8_t linePos = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      lineBuf[linePos] = '\0';
      if (linePos > 0) processCommand(lineBuf);
      linePos = 0;
    } else if (linePos < (uint8_t)(sizeof(lineBuf) - 1)) {
      lineBuf[linePos++] = c;
    }
  }
}
