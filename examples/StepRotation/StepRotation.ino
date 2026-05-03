// StepRotation.ino
//
// Demonstrates endless rotation in small increments (POSITION mode, ST3020).
//
// The servo advances STEP_TICKS positions at a time, pauses briefly, then
// prints the current position — repeating forever.
//
// Angle-limit handling:
//   • If angle limits are set in EEPROM (min > 0 or max < 4095), the servo
//     bounces: it reverses direction whenever the next step would cross a
//     limit boundary.
//   • If no limits are set (full range 0–4095), it wraps around continuously
//     in one direction (endless circle).
//
// Why POSITION mode (not STEP/VELOCITY)?
//   • Shortest-path routing cleanly crosses the 0/4095 wrap boundary.
//   • No uint16 accumulation overflow after ~16 turns (as in STEP mode).
//
// Wiring (ESP32):
//   GPIO17 TX --[1kΩ]--+-- SERVO BUS DATA
//   GPIO18 RX ----------+
//   GND   -------------- SERVO GND
//   5-8.4V ------------- SERVO VCC

#include <Arduino.h>
#include <FeetechBus.h>
#include <FeetechST3020.h>

static constexpr int      TX_PIN     = 17;
static constexpr int      RX_PIN     = 18;
static constexpr uint32_t BAUD       = 1000000;

static constexpr uint16_t STEP_TICKS = 5;    // ticks to advance each iteration
static constexpr uint16_t STEP_TIME  = 200;  // ms allowed for each step move
static constexpr uint16_t STEP_SPEED = 300;  // speed cap (must be > 0)
static constexpr uint32_t PAUSE_MS   = 300;  // pause after each step

FeetechBus    bus(Serial2);
FeetechST3020 st(bus, 1);   // <-- set to your servo ID

uint16_t limitMin  = 0;
uint16_t limitMax  = 4095;
bool     hasLimits = false;
int8_t   dir       = +1;    // +1 = forward, -1 = backward (only used with limits)

uint32_t totalSteps = 0;

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

  // Read angle limits and decide operating mode.
  if (st.readAngleLimits(limitMin, limitMax)) {
    hasLimits = (limitMin > 0 || limitMax < 4095);
  }

  if (hasLimits) {
    Serial.print("Angle limits active: ");
    Serial.print(limitMin); Serial.print(" (");
    Serial.print((uint32_t)limitMin * 360 / 4096);
    Serial.print("°) – ");
    Serial.print(limitMax); Serial.print(" (");
    Serial.print((uint32_t)limitMax * 360 / 4096);
    Serial.println("°)  → bouncing between limits");
  } else {
    Serial.println("No angle limits → endless circular rotation");
  }

  uint16_t startPos = 0;
  st.readPresentPosition(startPos);
  Serial.print("Start = "); Serial.print(startPos);
  Serial.print(" ("); Serial.print((uint32_t)startPos * 360 / 4096);
  Serial.println("°)\n");
}

// ---------------------------------------------------------------------------

void loop() {
  uint16_t pos = 0;
  if (!st.readPresentPosition(pos)) {
    Serial.println("[read error — retrying]");
    delay(200);
    return;
  }

  uint16_t target;

  if (hasLimits) {
    int32_t next = (int32_t)pos + dir * STEP_TICKS;
    if (next >= (int32_t)limitMax) {
      next = limitMax;
      dir  = -1;
    } else if (next <= (int32_t)limitMin) {
      next = limitMin;
      dir  = +1;
    }
    target = (uint16_t)next;
  } else {
    target = (uint16_t)((pos + STEP_TICKS) % 4096);
  }

  st.moveTime(target, STEP_TIME, STEP_SPEED);
  delay(PAUSE_MS);

  uint16_t actual = 0;
  st.readPresentPosition(actual);

  totalSteps += STEP_TICKS;
  Serial.print("pos="); Serial.print(actual);
  Serial.print("  ("); Serial.print((uint32_t)actual * 360 / 4096); Serial.print("°)");
  if (!hasLimits) {
    Serial.print("   rev="); Serial.print(totalSteps / 4096);
  } else {
    Serial.print(dir == +1 ? "  →" : "  ←");
  }
  Serial.println();
}
