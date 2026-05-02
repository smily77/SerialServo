// StepModeIndexer.ino
//
// Demonstrates multi-turn absolute position control in step mode (ST3020).
//
// Unlike position mode (limited to 0–4095, i.e. one revolution), step mode
// allows the servo to be commanded to any absolute step count — making it
// suitable for winding machines, telescopes, conveyor drives, or any
// application that needs precise multi-turn travel.
//
// How it works:
//   1. Read the current step counter at startup → treat it as the origin.
//   2. Add multiples of STEPS_PER_TURN to compute target positions.
//   3. Call moveTime(target, timeMs, speed) just like in position mode.
//   4. After each move, read back the actual position to confirm arrival.
//
// The position register is uint16 (0–65535 = ~16 turns from any reference).
// All arithmetic is kept in int32 and only cast to uint16 for the final
// moveTime() call.  Stay within ±8 turns of the origin to avoid wrap issues.
//
// Demo sequence (repeating):
//   Origin  →  +1 turn  →  +2 turns  →  +3 turns
//           →  +2 turns  →  +1 turn   →  Origin
//
// Adjust NUM_STEPS and STEP_SIZE_TURNS to match your application.
//
// IMPORTANT — EEPROM vs RAM:
//   setMode()  → EEPROM.  Written once in setup(); persists after power-off.
//   moveTime() → RAM.     Safe to call in every loop iteration.
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

static constexpr int32_t  STEPS_PER_TURN  = 4096; // ST3020: 4096 ticks = 360°
static constexpr int32_t  STEP_SIZE_TURNS = 1;     // advance 1 turn per index step
static constexpr int32_t  NUM_STEPS       = 3;     // go forward 3 steps, then back
static constexpr uint16_t MOVE_TIME_MS    = 1200;  // time to complete each step
static constexpr uint16_t MOVE_SPEED      = 600;   // speed cap (0 = no cap)

FeetechBus    bus(Serial2);
FeetechST3020 st(bus, 1);   // <-- set to your servo ID

int32_t origin = 0; // step count at startup (read from servo)

// ---------------------------------------------------------------------------

// Move to an absolute step count (relative to origin) and wait until done.
void indexTo(int32_t turnsFromOrigin) {
  int32_t targetStep = origin + turnsFromOrigin * STEPS_PER_TURN;

  // Clamp to uint16 range (0–65535)
  if (targetStep < 0)     targetStep = 0;
  if (targetStep > 65535) targetStep = 65535;

  Serial.print("-> "); Serial.print(turnsFromOrigin);
  Serial.print(" turn(s) from origin  [target step=");
  Serial.print(targetStep); Serial.print("]");

  st.moveTime((uint16_t)targetStep, MOVE_TIME_MS, MOVE_SPEED);

  // Wait for movement to complete
  delay(60);
  bool moving = true;
  while (moving) {
    if (!st.isMoving(moving)) { Serial.print(" [read err]"); break; }
    delay(20);
  }

  // Read back and verify
  uint16_t actual = 0;
  st.readPresentPosition(actual);
  int32_t err = (int32_t)actual - targetStep;
  Serial.print("  arrived="); Serial.print(actual);
  Serial.print("  error="); Serial.print(err); Serial.println(" steps");

  delay(400);
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

  Serial.println("Setting STEP mode...");
  if (!st.setMode(ServoMode::STEP)) {
    Serial.println("ERROR: setMode failed");
    while (true) delay(1000);
  }

  // Record starting step count as the origin for all subsequent moves.
  uint16_t raw = 0;
  st.readPresentPosition(raw);
  origin = (int32_t)raw;

  Serial.print("STEP mode active.  Origin = step ");
  Serial.println(origin);
  Serial.print("  Max forward reach: +");
  Serial.print((65535 - origin) / STEPS_PER_TURN);
  Serial.println(" turns from origin");
  Serial.print("  Max backward reach: -");
  Serial.println(origin / STEPS_PER_TURN);
}

// ---------------------------------------------------------------------------

void loop() {
  Serial.println("\n=== Forward sequence ===");
  for (int32_t i = 1; i <= NUM_STEPS; i++) {
    indexTo(i * STEP_SIZE_TURNS);
  }

  delay(600);

  Serial.println("\n=== Return sequence ===");
  for (int32_t i = NUM_STEPS - 1; i >= 0; i--) {
    indexTo(i * STEP_SIZE_TURNS);
  }

  delay(1000);
}
