// BusScan.ino
//
// Scans all 253 valid servo IDs on the bus and prints each one that responds.
// Tries multiple baud rates so you can find servos even if the baud rate is unknown.
//
// Output example:
//   === Baud: 1000000 ===
//   [  1] FOUND  ping ok
//   [ 11] FOUND  ping ok
//   Scan at 1000000 baud complete: 2 servo(s) found.
//
//   === Baud: 500000 ===
//   No servos found at this baud rate.
//
// Wiring (ESP32):
//   GPIO17 TX --[1kΩ]--+-- SERVO BUS DATA
//   GPIO18 RX ----------+
//   GND   -------------- SERVO GND
//   5-8.4V ------------- SERVO VCC

#include <Arduino.h>
#include <FeetechBus.h>
#include <FeetechDevice.h>

static constexpr int TX_PIN = 17;    // <-- TX through 1kΩ to bus
static constexpr int RX_PIN = 18;    // <-- RX directly to bus

// Feetech servos ship at 1 Mbps from factory.
// List additional rates here if yours was changed.
static const uint32_t BAUD_RATES[] = { 1000000, 500000, 115200 };
static constexpr int  NUM_RATES    = sizeof(BAUD_RATES) / sizeof(BAUD_RATES[0]);

FeetechBus bus(Serial2);

void scanAtBaud(uint32_t baud) {
  Serial.print("\n=== Baud: ");
  Serial.print(baud);
  Serial.println(" ===");

  bus.beginPins(baud, RX_PIN, TX_PIN);

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

    delay(5); // short gap between pings to avoid flooding the bus
  }

  Serial.print("Scan at ");
  Serial.print(baud);
  Serial.print(" baud complete: ");
  Serial.print(found);
  Serial.println(" servo(s) found.");
  if (found == 0) Serial.println("No servos found at this baud rate.");

  bus.end();
  delay(50);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Feetech Bus Scanner");
  Serial.println("Wiring: TX(GPIO17) --[1k]--> BUS <-- RX(GPIO18). Servo powered.");

  for (int i = 0; i < NUM_RATES; i++) {
    scanAtBaud(BAUD_RATES[i]);
  }

  Serial.println("\nDone. Reset to scan again.");
}

void loop() {}
