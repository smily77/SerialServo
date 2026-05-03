// StepRotation.ino
//
// Demonstrates endless rotation in small increments (ST3020).
//
// The servo advances STEP_TICKS positions at a time, pauses briefly, then
// prints the current position — repeating forever.
//
// Angle-limit handling:
//   • No limits set (min=0, max=4095): STEP mode — position counter
//     accumulates indefinitely; uint16 wraps after ~16 turns (65535/4096)
//     but signed-int16 delta arithmetic handles the wrap transparently.
//   • Limits active: POSITION mode — servo bounces (reverses direction)
//     whenever the next step would cross a limit boundary.
//
// Why not POSITION mode for endless rotation?
//   POSITION mode treats 0 and 4095 as opposite ends of a range, not as a
//   circular ring.  Commanding position 3 from position 4093 sends the servo
//   4090 steps backward instead of 6 steps forward.
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
int8_t   dir       = +1;    // +1 = forward, -1 = backward (limits mode only)

uint32_t totalTicks = 0;    // cumulative tick count (circle mode only)

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

  // Read angle limits to decide the operating mode.
  if (st.readAngleLimits(limitMin, limitMax)) {
    hasLimits = (limitMin > 0 || limitMax < 4095);
  }

  if (hasLimits) {
    // POSITION mode: servo stays within the configured range and bounces.
    Serial.println("Setting POSITION mode (angle limits active)...");
    if (!st.setMode(ServoMode::POSITION)) {
      Serial.println("ERROR: setMode failed");
      while (true) delay(1000);
    }
    Serial.print("Bouncing between ");
    Serial.print(limitMin); Serial.print(" (");
    Serial.print((uint32_t)limitMin * 360 / 4096); Serial.print("°) and ");
    Serial.print(limitMax); Serial.print(" (");
    Serial.print((uint32_t)limitMax * 360 / 4096); Serial.println("°)");
  } else {
    // STEP mode: position counter accumulates; uint16 wrap handled by signed delta.
    Serial.println("Setting STEP mode (no angle limits — endless circle)...");
    if (!st.setMode(ServoMode::STEP)) {
      Serial.println("ERROR: setMode failed");
      while (true) delay(1000);
    }
    Serial.println("Rotating endlessly. Counter wraps after ~16 turns.");
  }

  uint16_t startPos = 0;
  st.readPresentPosition(startPos);
  Serial.print("Start = "); Serial.print(startPos);
  Serial.print("  ("); Serial.print((uint32_t)(startPos % 4096) * 360 / 4096);
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
    // POSITION mode: bounce within [limitMin, limitMax]
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
    // STEP mode: plain uint16 addition — overflow wraps from 65535 to 0.
    // The servo computes direction as (int16_t)(target - current), so a
    // 5-tick overflow (65533 → 2) is correctly seen as +5, not −65531.
    target = (uint16_t)(pos + STEP_TICKS);
    totalTicks += STEP_TICKS;
  }

  st.moveTime(target, STEP_TIME, STEP_SPEED);
  delay(PAUSE_MS);

  uint16_t actual = 0;
  st.readPresentPosition(actual);

  uint32_t angleDeg = (uint32_t)(actual % 4096) * 360 / 4096;
  Serial.print("pos="); Serial.print(actual);
  Serial.print("  ("); Serial.print(angleDeg); Serial.print("°)");

  if (!hasLimits) {
    Serial.print("   rev="); Serial.print(totalTicks / 4096);
  } else {
    Serial.print(dir == +1 ? "  →" : "  ←");
  }
  Serial.println();
}
