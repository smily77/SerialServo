// ContinuousRotation.ino
//
// Demonstrates continuous rotation (velocity mode) on the ST3020.
// The ST3020 supports 360° endless rotation via ServoMode::VELOCITY.
//
// Pattern:
//   1. Spin forward at speed 800 for 2 s (reads live speed)
//   2. Brake to stop
//   3. Spin backward at speed -600 for 2 s
//   4. Brake to stop, pause 1 s, repeat
//
// IMPORTANT — EEPROM vs RAM:
//   setMode()           → writes EEPROM (0x21).  Call ONCE in setup().
//                         EEPROM is rated for ~100 000 write cycles.
//   setTargetVelocity() → writes RAM (0x2E).  Safe to call every loop().
//
// Speed range: -32767 … +32767
//   Positive = forward, negative = reverse, 0 = stop.
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

// Spin for durationMs, printing live speed every 100 ms.
void spinFor(int16_t targetVel, uint32_t durationMs) {
  st.setTargetVelocity(targetVel);

  uint32_t start = millis();
  while (millis() - start < durationMs) {
    int16_t spd = 0;
    if (st.readCurrentSpeed(spd)) {
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

  bus.begin1Wire(BAUD, BUS_PIN);

  if (!st.ping()) {
    Serial.println("ERROR: servo not found — check ID, baud, wiring");
    while (true) delay(1000);
  }
  Serial.println("Servo found.");

  st.init();

  // Switch to velocity (continuous rotation) mode.
  // This is written to EEPROM and survives power-off.
  Serial.println("Setting VELOCITY mode...");
  if (!st.setMode(ServoMode::VELOCITY)) {
    Serial.println("ERROR: setMode failed");
    while (true) delay(1000);
  }
  Serial.println("Ready — continuous rotation active.");
}

void loop() {
  Serial.println("\nForward 800:");
  spinFor(800, 2000);

  Serial.println("Stop.");
  st.setTargetVelocity(0);
  delay(600);

  Serial.println("\nReverse -600:");
  spinFor(-600, 2000);

  Serial.println("Stop.");
  st.setTargetVelocity(0);
  delay(1000);
}
