// StepRotation.ino
//
// Servo rotates STEP_TICKS per step, pauses briefly, prints current position
// (always 0–4095), then repeats endlessly.
//
// Mode selection (automatic, based on EEPROM angle limits):
//
//   No limits (min=0, max=4095)  →  VELOCITY mode
//     setTargetVelocity() spins the servo; the position register wraps
//     naturally at 4096 and is always displayed directly (0–4095).
//     "Step" = spin until (int16)(pos − ref) ≥ STEP_TICKS; handles the
//     4095→0 boundary via signed-int16 arithmetic.
//     No uint16 accumulation, no sign-bit collision at 32768.
//
//   Limits set  →  POSITION mode
//     moveTime() targets within [limitMin, limitMax]; direction reverses
//     (bounces) at each boundary.
//
// Why not STEP mode for endless rotation?
//   The ST3020 treats the goal-position register as a signed int16, so
//   positions ≥ 32768 (bit 15 set) are interpreted as negative — the servo
//   stops or reverses at 32769.  VELOCITY mode has no such limitation.
//
// Wiring (ESP32):
//   GPIO1  TX --[1kΩ]--+-- SERVO BUS DATA
//   GPIO2  RX ----------+
//   GND   -------------- SERVO GND
//   5-8.4V ------------- SERVO VCC

#include <Arduino.h>
#include <FeetechBus.h>
#include <FeetechST3020.h>

static constexpr int      TX_PIN     = 1;
static constexpr int      RX_PIN     = 2;
static constexpr uint32_t BAUD       = 1000000;

static constexpr int16_t  STEP_TICKS = 5;    // ticks per step (velocity mode)
static constexpr int16_t  STEP_SPEED = 200;  // running speed  (velocity mode)
static constexpr uint16_t MOVE_SPEED = 300;  // speed cap      (position mode)
static constexpr uint32_t PAUSE_MS   = 300;  // pause between steps

FeetechBus    bus(Serial2);
FeetechST3020 st(bus, 2);   // servo ID = 2

uint16_t limitMin  = 0;
uint16_t limitMax  = 4095;
bool     hasLimits = false;
int8_t   dir       = +1;   // +1 forward / -1 backward  (position/bounce mode)

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);   // allow USB serial to stabilise

  bus.beginPins(BAUD, RX_PIN, TX_PIN);

  if (!st.ping()) {
    Serial.println("ERROR: servo not found — check ID, baud, wiring");
    while (true) delay(1000);
  }
  Serial.println("ST3020 found.");

  st.init();

  // Read EEPROM angle limits to decide the operating mode.
  if (st.readAngleLimits(limitMin, limitMax)) {
    hasLimits = (limitMin > 0 || limitMax < 4095);
  }

  if (hasLimits) {
    if (!st.setMode(ServoMode::POSITION)) {
      Serial.println("ERROR: setMode POSITION failed");
      while (true) delay(1000);
    }
    Serial.print("POSITION mode — bouncing between ");
    Serial.print(limitMin); Serial.print(" (");
    Serial.print((uint32_t)limitMin * 360 / 4096);
    Serial.print("°) and ");
    Serial.print(limitMax); Serial.print(" (");
    Serial.print((uint32_t)limitMax * 360 / 4096);
    Serial.println("°)");
  } else {
    if (!st.setMode(ServoMode::VELOCITY)) {
      Serial.println("ERROR: setMode VELOCITY failed");
      while (true) delay(1000);
    }
    Serial.println("VELOCITY mode — endless circle, position always 0–4095");
  }

  uint16_t startPos = 0;
  st.readPresentPosition(startPos);
  Serial.print("Start = "); Serial.print(startPos);
  Serial.print("  ("); Serial.print((uint32_t)startPos * 360 / 4096);
  Serial.println("°)\n");
}

// ---------------------------------------------------------------------------

void loop() {

  // ── VELOCITY mode: endless circle ────────────────────────────────────────
  if (!hasLimits) {
    uint16_t ref = 0;
    if (!st.readPresentPosition(ref)) { delay(100); return; }

    // Spin forward until STEP_TICKS have been covered.
    // Forward distance in the 0–4095 ring: (pos + 4096 − ref) % 4096
    // e.g. ref=4093, pos=2  →  (2 + 4096 − 4093) % 4096 = 5  ✓
    // Note: the signed-int16 trick does NOT work here because the register
    // wraps at 4096, not 65536.
    st.setTargetVelocity(STEP_SPEED);
    uint16_t pos = ref;
    while (((uint32_t)pos + 4096U - (uint32_t)ref) % 4096U < (uint32_t)STEP_TICKS) {
      delay(5);
      st.readPresentPosition(pos);
    }

    st.setTargetVelocity(0);   // stop and hold position
    delay(PAUSE_MS);

    st.readPresentPosition(pos);
    // pos is always 0–4095 in VELOCITY mode — print directly
    Serial.print("pos="); Serial.print(pos);
    Serial.print("  ("); Serial.print((uint32_t)pos * 360 / 4096);
    Serial.println("°)");
    return;
  }

  // ── POSITION mode: bounce between angle limits ────────────────────────────
  uint16_t pos = 0;
  if (!st.readPresentPosition(pos)) { delay(100); return; }

  int32_t next = (int32_t)pos + dir * STEP_TICKS;
  if (next >= (int32_t)limitMax) { next = limitMax; dir = -1; }
  else if (next <= (int32_t)limitMin) { next = limitMin; dir  = +1; }

  st.moveTime((uint16_t)next, 200, MOVE_SPEED);
  delay(PAUSE_MS);

  st.readPresentPosition(pos);
  Serial.print("pos="); Serial.print(pos);
  Serial.print("  ("); Serial.print((uint32_t)pos * 360 / 4096);
  Serial.print("°)");
  Serial.println(dir == +1 ? "  →" : "  ←");
}
