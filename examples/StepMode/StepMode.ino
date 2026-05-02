// StepMode.ino
//
// Demonstrates step mode (ServoMode::STEP) on the ST3020.
//
// Step mode = continuous rotation with multi-turn position memory.
//
// The servo is commanded with the same setTargetVelocity() call as in
// VELOCITY mode, but the position register now ACCUMULATES across full
// rotations instead of wrapping at 4096.  This lets you track how many
// total steps (and therefore how many complete turns) the shaft has made.
//
//   In VELOCITY mode:  pos always shows 0–4095 (current angle only)
//   In STEP mode:      pos climbs to 5000, 10000, … as the servo spins
//
// 1 full turn of ST3020 = 4096 ticks.
//
//   Full turns  = delta / 4096          (integer part)
//   Angle left  = (delta % 4096) × 360° / 4096
//
// The position register is 16-bit (0–65535), so the counter wraps after
// ~16 turns from a fixed reference point.  For longer travel, detect wraps
// by comparing successive readings or use a 32-bit accumulator (see comment
// in the loop helper below).
//
// Pattern (repeating):
//   1. Spin forward at speed 500 for 4 s — position counter climbs
//   2. Stop, report steps and full turns travelled
//   3. Spin backward at speed –500 for 4 s — counter falls back
//   4. Stop, report net displacement, pause 1 s
//
// IMPORTANT — EEPROM vs RAM:
//   setMode()           → EEPROM (0x21).  Call once in setup().
//   setTargetVelocity() → RAM (0x2E).     Safe to call every iteration.
//
// Wiring (ESP32):
//   GPIO17 TX --[1kΩ]--+-- SERVO BUS DATA
//   GPIO18 RX ----------+
//   GND   -------------- SERVO GND
//   5-8.4V ------------- SERVO VCC

#include <Arduino.h>
#include <FeetechBus.h>
#include <FeetechST3020.h>

static constexpr int      TX_PIN = 17;
static constexpr int      RX_PIN = 18;
static constexpr uint32_t BAUD   = 1000000;

FeetechBus    bus(Serial2);
FeetechST3020 st(bus, 1);   // <-- set to your servo ID

// ---------------------------------------------------------------------------

// Spin at 'vel' for 'durationMs', printing live step count every 150 ms.
// Returns the signed step delta relative to 'startPos'.
// Uses int16 subtraction — safe for ±32767 ticks (±8 turns) from startPos.
int16_t spinAndCount(int16_t vel, uint32_t durationMs, uint16_t startPos) {
  st.setTargetVelocity(vel);

  uint32_t t0 = millis();
  uint16_t pos = startPos;

  while (millis() - t0 < durationMs) {
    if (st.readPresentPosition(pos)) {
      int16_t delta = (int16_t)(pos - startPos); // wraps correctly for ±8 turns
      int16_t turns = delta / 4096;
      int16_t rem   = delta % 4096;
      Serial.print("  pos="); Serial.print(pos);
      Serial.print("  delta="); Serial.print(delta);
      Serial.print("  turns="); Serial.print(turns);
      Serial.print("+"); Serial.print((int32_t)rem * 360 / 4096);
      Serial.println("°");
    }
    delay(150);
  }

  st.setTargetVelocity(0);
  delay(200); // let the servo settle

  uint16_t finalPos = 0;
  st.readPresentPosition(finalPos);
  return (int16_t)(finalPos - startPos);
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(200);

  bus.beginPins(BAUD, RX_PIN, TX_PIN);

  if (!st.ping()) {
    Serial.println("ERROR: servo not found — check ID, baud, wiring");
    while (true) delay(1000);
  }
  Serial.println("ST3020 found.");

  st.init();

  // Switch to step mode — written to EEPROM, survives power-off.
  // setMode() reads the current mode first and skips the write if it already
  // matches, preserving EEPROM write-cycle budget.
  Serial.println("Setting STEP mode...");
  if (!st.setMode(ServoMode::STEP)) {
    Serial.println("ERROR: setMode failed");
    while (true) delay(1000);
  }

  uint16_t startPos = 0;
  st.readPresentPosition(startPos);
  Serial.print("STEP mode active.  Starting position: ");
  Serial.println(startPos);
}

// ---------------------------------------------------------------------------

void loop() {
  uint16_t refPos = 0;
  st.readPresentPosition(refPos);
  Serial.print("\nReference position: "); Serial.println(refPos);

  // ── Phase 1: forward ────────────────────────────────────────────────────
  Serial.println("\n[forward  speed=500  4 s]");
  int16_t fwdDelta = spinAndCount(500, 4000, refPos);
  Serial.print("  => "); Serial.print(fwdDelta); Serial.print(" steps  = ");
  Serial.print(fwdDelta / 4096); Serial.print(" full turns + ");
  Serial.print((int32_t)(fwdDelta % 4096) * 360 / 4096); Serial.println("°");

  delay(500);

  // ── Phase 2: backward ───────────────────────────────────────────────────
  uint16_t midPos = 0;
  st.readPresentPosition(midPos);
  Serial.println("\n[backward speed=-500  4 s]");
  int16_t bwdDelta = spinAndCount(-500, 4000, midPos);
  Serial.print("  => "); Serial.print(bwdDelta); Serial.println(" steps (should be ≈ negative of forward)");

  uint16_t endPos = 0;
  st.readPresentPosition(endPos);
  int16_t net = (int16_t)(endPos - refPos);
  Serial.print("\nNet displacement from reference: "); Serial.print(net);
  Serial.println(" steps (ideally 0)");

  delay(1000);
}
