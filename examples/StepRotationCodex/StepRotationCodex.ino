// StepRotationCodex.ino
//
// Variant of StepRotation that explicitly uses the new APIs:
//   - moveTimeSigned(...) in POSITION mode
//   - moveEx(...) in VELOCITY mode (ACC + speed write pattern)
//
// ST3020 only.

#include <Arduino.h>
#include <FeetechBus.h>
#include <FeetechST3020.h>

static constexpr int      TX_PIN      = 1;
static constexpr int      RX_PIN      = 2;
static constexpr uint32_t BAUD        = 1000000;

static constexpr int16_t  STEP_TICKS  = 5;    // ticks per step
static constexpr int16_t  STEP_SPEED  = 200;  // velocity mode speed
static constexpr uint16_t MOVE_SPEED  = 300;  // position mode speed cap
static constexpr uint8_t  MOVE_ACC    = 20;   // acceleration for moveEx
static constexpr uint32_t PAUSE_MS    = 300;

FeetechBus    bus(Serial2);
FeetechST3020 st(bus, 2);

uint16_t limitMin  = 0;
uint16_t limitMax  = 4095;
bool     hasLimits = false;
int8_t   dir       = +1;

void setup() {
  Serial.begin(115200);
  delay(1000);

  bus.beginPins(BAUD, RX_PIN, TX_PIN);

  if (!st.ping()) {
    Serial.println("ERROR: servo not found — check ID, baud, wiring");
    while (true) delay(1000);
  }

  st.init();

  if (st.readAngleLimits(limitMin, limitMax)) {
    hasLimits = (limitMin > 0 || limitMax < 4095);
  }

  if (hasLimits) {
    if (!st.setMode(ServoMode::POSITION)) {
      Serial.println("ERROR: setMode POSITION failed");
      while (true) delay(1000);
    }
    Serial.println("POSITION mode (moveTimeSigned)");
  } else {
    if (!st.setMode(ServoMode::VELOCITY)) {
      Serial.println("ERROR: setMode VELOCITY failed");
      while (true) delay(1000);
    }
    Serial.println("VELOCITY mode (moveEx)");
  }
}

void loop() {
  if (!hasLimits) {
    uint16_t ref = 0;
    if (!st.readPresentPosition(ref)) { delay(100); return; }

    // Start spinning via moveEx (ACC + position/time/speed packet layout)
    // Position/time are 0 in velocity mode; speed controls rotation.
    st.moveEx(0, 0, (uint16_t)STEP_SPEED, MOVE_ACC);

    uint16_t pos = ref;
    while (((uint32_t)pos + 4096U - (uint32_t)ref) % 4096U < (uint32_t)STEP_TICKS) {
      delay(5);
      st.readPresentPosition(pos);
    }

    st.setTargetVelocity(0);
    delay(PAUSE_MS);

    st.readPresentPosition(pos);
    Serial.print("pos="); Serial.println(pos);
    return;
  }

  uint16_t pos = 0;
  if (!st.readPresentPosition(pos)) { delay(100); return; }

  int32_t next = (int32_t)pos + dir * STEP_TICKS;
  if (next >= (int32_t)limitMax) { next = limitMax; dir = -1; }
  else if (next <= (int32_t)limitMin) { next = limitMin; dir = +1; }

  // Explicitly use signed API in position mode.
  st.moveTimeSigned((int16_t)next, 200, MOVE_SPEED);
  delay(PAUSE_MS);

  st.readPresentPosition(pos);
  Serial.print("pos="); Serial.print(pos);
  Serial.println(dir == +1 ? "  ->" : "  <-");
}
