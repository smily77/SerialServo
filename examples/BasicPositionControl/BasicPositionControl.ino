// BasicPositionControl.ino
//
// Moves an ST3020 through several angles, waits for each move to finish
// using isMoving(), then reads back the actual position and speed.
//
// Wiring (ESP32):
//   GPIO17 --[1kΩ]--> SERVO BUS DATA
//   GPIO17 <--------- SERVO BUS DATA
//   GND   ----------- SERVO GND
//   5-8.4V ---------- SERVO VCC

#include <Arduino.h>
#include <FeetechBus.h>
#include <FeetechST3020.h>

static constexpr int      BUS_PIN = 17;     // <-- your ESP32 GPIO
static constexpr uint32_t BAUD    = 1000000;

FeetechBus    bus(Serial2);
FeetechST3020 st(bus, 11);  // <-- your servo ID

// ---------------------------------------------------------------------------

// Move to a target angle and block until the servo stops moving.
void moveTo(float deg, uint16_t timeMs, uint16_t speed) {
  uint16_t ticks = st.degToTicks(deg);

  Serial.print("-> ");
  Serial.print(deg, 0);
  Serial.print(" deg  (");
  Serial.print(ticks);
  Serial.println(" ticks)");

  st.moveTime(ticks, timeMs, speed);

  // Short pause so the servo has time to start before we poll isMoving()
  delay(60);

  bool moving = true;
  while (moving) {
    if (!st.isMoving(moving)) {
      Serial.println("   read error");
      break;
    }
    delay(20);
  }

  // Read back actual position and speed
  uint16_t pos = 0;
  int16_t  spd = 0;
  st.readPresentPosition(pos);
  st.readCurrentSpeed(spd);

  Serial.print("   arrived: pos=");
  Serial.print(pos);
  Serial.print("  speed=");
  Serial.println(spd);

  delay(400);
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(200);

  bus.begin1Wire(BAUD, BUS_PIN);

  if (!st.ping()) {
    Serial.println("ERROR: servo not found — check ID, baud, wiring");
    while (true) delay(1000);
  }
  Serial.println("Servo found.");

  FeetechST3020::Profile p;
  p.returnDelayUnits  = 0;
  p.torqueOnAfterInit = true;
  st.init(p);
}

void loop() {
  moveTo(  0.0f, 900, 600);
  moveTo( 90.0f, 600, 900);
  moveTo(180.0f, 600, 900);
  moveTo(270.0f, 600, 900);
  moveTo(  0.0f, 900, 600);
  delay(1000);
}
