// SpeedMove.ino
//
// Moves an ST3020 to a target position as fast as the hardware allows,
// without any speed cap or time constraint.
//
// Key parameters:
//   moveTime(pos, time=0, speed=0)
//     time  = 0  →  no duration limit; servo picks its own pace
//     speed = 0  →  no speed cap; full hardware speed
//   setAcceleration(0)
//     acc   = 0  →  no ramp-up; servo drives at full torque from the start
//
// WARNING: with acc=0 and speed=0 the servo moves hard and fast.
//          Make sure the load and mounting can handle the impact at end-stops.
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
FeetechST3020 st(bus, 1);  // <-- your servo ID (use BusScan to find it)

// ---------------------------------------------------------------------------

// Move to target and block until the servo stops.
void snapTo(float deg) {
  uint16_t ticks = st.degToTicks(deg);

  Serial.print("-> ");
  Serial.print(deg, 0);
  Serial.print(" deg  (ticks=");
  Serial.print(ticks);
  Serial.println(")");

  // time=0, speed=0  →  full hardware speed, no ramp
  st.moveTime(ticks, 0, 0);

  // Give the servo a moment to start moving before polling
  delay(40);

  bool moving = true;
  while (moving) {
    if (!st.isMoving(moving)) {
      Serial.println("   read error");
      break;
    }
    delay(10);
  }

  uint16_t pos = 0;
  int16_t  spd = 0;
  st.readPresentPosition(pos);
  st.readCurrentSpeed(spd);

  Serial.print("   arrived: pos=");
  Serial.print(pos);
  Serial.print("  speed=");
  Serial.println(spd);
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
  Serial.println("Servo found.");

  FeetechST3020::Profile p;
  p.torqueOnAfterInit = true;
  st.init(p);

  // Disable acceleration ramp — maximum torque from standstill.
  // Remove this line (or set acc > 0) to get a softer start.
  st.setAcceleration(0);
}

void loop() {
  snapTo(  0.0f);
  delay(600);
  snapTo(180.0f);
  delay(600);
  snapTo( 90.0f);
  delay(600);
  snapTo(270.0f);
  delay(600);
}
