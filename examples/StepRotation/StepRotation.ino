// StepRotation.ino
//
// Demonstrates endless circular rotation in small increments (POSITION mode).
//
// The servo advances STEP_TICKS positions at a time, pauses briefly, prints
// the current position, then repeats — spinning continuously in one direction.
//
// Why POSITION mode (not STEP/VELOCITY)?
//   • POSITION mode uses shortest-path routing.  A 5-tick step that crosses
//     the 0/4095 boundary goes forward (5 steps), not backward (4091 steps).
//   • No uint16 overflow: the 0–4095 register wraps naturally without
//     accumulating large step counts that would overflow uint16.
//
// The position register is 0–4095 (exactly one revolution for ST3020).
//   target = (current + STEP_TICKS) % 4096
//
// Wiring (ESP32):
//   GPIO17 TX --[1kΩ]--+-- SERVO BUS DATA
//   GPIO18 RX ----------+
//   GND   -------------- SERVO GND
//   5-8.4V ------------- SERVO VCC

#include <Arduino.h>
#include <FeetechBus.h>
#include <FeetechST3020.h>

static constexpr int      TX_PIN      = 17;
static constexpr int      RX_PIN      = 18;
static constexpr uint32_t BAUD        = 1000000;

static constexpr uint16_t STEP_TICKS  = 5;    // ticks to advance each iteration
static constexpr uint16_t STEP_TIME   = 200;  // ms allowed for each step move
static constexpr uint16_t STEP_SPEED  = 300;  // speed cap (0 = unlimited → servo won't move!)
static constexpr uint32_t PAUSE_MS    = 300;  // pause after each step before printing

FeetechBus    bus(Serial2);
FeetechST3020 st(bus, 1);   // <-- set to your servo ID

uint32_t totalSteps = 0;    // cumulative step count (for revolution display)

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

  Serial.println("Setting POSITION mode...");
  if (!st.setMode(ServoMode::POSITION)) {
    Serial.println("ERROR: setMode failed");
    while (true) delay(1000);
  }

  uint16_t startPos = 0;
  st.readPresentPosition(startPos);
  Serial.print("POSITION mode active.  Start = ");
  Serial.print(startPos);
  Serial.print(" (");
  Serial.print((uint32_t)startPos * 360 / 4096);
  Serial.println("°)");
  Serial.println("Rotating in steps of " + String(STEP_TICKS) + " ticks...\n");
}

// ---------------------------------------------------------------------------

void loop() {
  uint16_t pos = 0;
  if (!st.readPresentPosition(pos)) {
    Serial.println("[read error — retrying]");
    delay(200);
    return;
  }

  uint16_t target = (pos + STEP_TICKS) % 4096;
  st.moveTime(target, STEP_TIME, STEP_SPEED);

  delay(PAUSE_MS);

  uint16_t actual = 0;
  st.readPresentPosition(actual);

  totalSteps += STEP_TICKS;
  uint32_t revolutions = totalSteps / 4096;
  uint32_t remainder   = totalSteps % 4096;

  Serial.print("pos=");
  Serial.print(actual);
  Serial.print("  (");
  Serial.print((uint32_t)actual * 360 / 4096);
  Serial.print("°)   revolutions=");
  Serial.print(revolutions);
  Serial.print("  step=");
  Serial.println(remainder);
}
