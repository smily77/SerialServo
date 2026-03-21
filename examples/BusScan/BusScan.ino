// BusScan.ino
//
// Scans all 253 valid servo IDs on the bus and prints each one that responds.
// Useful for discovering which servos are connected and what IDs they have.
//
// Output example:
//   Scanning bus (IDs 1..253) ...
//   [  3] FOUND  ping ok
//   [ 11] FOUND  ping ok
//   Scan complete: 2 servo(s) found.
//
// Wiring (ESP32):
//   GPIO17 --[1kΩ]--> SERVO BUS DATA
//   GPIO17 <--------- SERVO BUS DATA
//   GND   ----------- SERVO GND
//   5-8.4V ---------- SERVO VCC

#include <Arduino.h>
#include <FeetechBus.h>
#include <FeetechDevice.h>

static constexpr int      BUS_PIN = 17;     // <-- your ESP32 GPIO
static constexpr uint32_t BAUD    = 1000000;

FeetechBus bus(Serial2);

void setup() {
  Serial.begin(115200);
  delay(200);

  bus.begin1Wire(BAUD, BUS_PIN);

  Serial.println("Scanning bus (IDs 1..253) ...");

  int found = 0;

  for (uint8_t id = 1; id <= 253; id++) {
    FeetechDevice dev(bus, id);

    if (dev.ping()) {
      found++;
      Serial.print("[");
      if (id < 100) Serial.print(" ");
      if (id <  10) Serial.print(" ");
      Serial.print(id);
      Serial.println("] FOUND  ping ok");
    }

    // Short pause between pings to avoid flooding the bus.
    // Each unanswered ping already waits for the timeout inside FeetechBus.
    delay(5);
  }

  Serial.println();
  Serial.print("Scan complete: ");
  Serial.print(found);
  Serial.println(" servo(s) found.");
}

void loop() {}
