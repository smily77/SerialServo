// SC15_PositionControl.ino
//
// Moves an SC15 through several angles using degToTicks().
// Blocks after each move until the servo stops, then reads back position and speed.
//
// SC15 quick facts:
//   Protocol : SCS (big-endian, bit-10 sign)
//   Range    : 0 – 180°   →   0 – 1023 ticks  (≈ 5.68 ticks/°)
//   Speed    : ±1023  (positive = forward, negative = reverse)
//
// Wiring (ESP32):
//   GPIO17 TX --[1kΩ]--+-- SERVO BUS DATA
//   GPIO18 RX ----------+
//   GND   -------------- SERVO GND
//   5-6V  -------------- SERVO VCC

#include <Arduino.h>
#include <FeetechBus.h>
#include <FeetechSC15.h>

static constexpr int      TX_PIN = 17;
static constexpr int      RX_PIN = 18;
static constexpr uint32_t BAUD   = 1000000;

FeetechBus bus(Serial2);
FeetechSC15 sc(bus, 1);  // <-- your servo ID (use BusScan to find it)

// ---------------------------------------------------------------------------

// Move to target angle and block until the servo stops.
void moveTo(float deg, uint16_t timeMs, uint16_t speed) {
  uint16_t ticks = sc.degToTicks(deg);

  Serial.print("-> ");
  Serial.print(deg, 0);
  Serial.print(" deg  (");
  Serial.print(ticks);
  Serial.println(" ticks)");

  sc.moveTime(ticks, timeMs, speed);

  // Short pause so the servo has time to start before we poll isMoving()
  delay(60);

  bool moving = true;
  while (moving) {
    if (!sc.isMoving(moving)) {
      Serial.println("   read error");
      break;
    }
    delay(20);
  }

  uint16_t pos = 0;
  int16_t  spd = 0;
  sc.readPresentPosition(pos);
  sc.readCurrentSpeed(spd);

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

  bus.beginPins(BAUD, RX_PIN, TX_PIN);

  if (!sc.ping()) {
    Serial.println("ERROR: servo not found — check ID, baud, wiring");
    while (true) delay(1000);
  }
  Serial.println("SC15 found.");

  FeetechSC15::Profile p;
  p.returnDelayUnits  = 0;
  p.torqueOnAfterInit = true;
  sc.init(p);
}

void loop() {
  moveTo(  0.0f, 900, 600);
  moveTo( 90.0f, 700, 700);
  moveTo(180.0f, 900, 600);
  moveTo( 90.0f, 700, 700);
  moveTo(  0.0f, 900, 600);
  delay(1000);
}
