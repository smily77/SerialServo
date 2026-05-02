// SC15_ContinuousRotation.ino
//
// Demonstrates continuous rotation (velocity mode) on the SC15.
// The SC15 supports endless rotation via ServoMode::VELOCITY.
//
// Pattern:
//   1. Spin forward at speed 600 for 2 s (reads live speed)
//   2. Brake to stop
//   3. Spin backward at speed -400 for 2 s
//   4. Brake to stop, pause 1 s, repeat
//
// SC15 speed range: -1023 … +1023
//   Positive = forward, negative = reverse, 0 = stop.
//   (ST3020 range is wider: ±32767)
//
// IMPORTANT — EEPROM vs RAM:
//   setMode()           → writes EEPROM (0x21).  Call ONCE in setup().
//                         EEPROM is rated for ~100 000 write cycles.
//                         The library reads current mode first and skips the
//                         write if the servo is already in the requested mode.
//   setTargetVelocity() → writes RAM (0x2E).  Safe to call every loop().
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
FeetechSC15 sc(bus, 1);  // <-- your servo ID

// ---------------------------------------------------------------------------

// Spin for durationMs, printing live speed every 100 ms.
void spinFor(int16_t targetVel, uint32_t durationMs) {
  sc.setTargetVelocity(targetVel);

  uint32_t start = millis();
  while (millis() - start < durationMs) {
    int16_t spd = 0;
    if (sc.readCurrentSpeed(spd)) {
      Serial.print("  speed=");
      Serial.println(spd);
    }
    delay(100);
  }
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

  sc.init();

  // Switch to velocity (continuous rotation) mode.
  // This is written to EEPROM and survives power-off.
  // The library reads the current mode first; if already VELOCITY, the EEPROM
  // write is skipped to preserve write-cycle budget.
  Serial.println("Setting VELOCITY mode...");
  if (!sc.setMode(ServoMode::VELOCITY)) {
    Serial.println("ERROR: setMode failed");
    while (true) delay(1000);
  }
  Serial.println("Ready — continuous rotation active.");
}

void loop() {
  Serial.println("\nForward 600:");
  spinFor(600, 2000);

  Serial.println("Stop.");
  sc.setTargetVelocity(0);
  delay(600);

  Serial.println("\nReverse -400:");
  spinFor(-400, 2000);

  Serial.println("Stop.");
  sc.setTargetVelocity(0);
  delay(1000);
}
