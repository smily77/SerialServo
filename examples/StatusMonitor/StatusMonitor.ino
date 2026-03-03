// StatusMonitor.ino
//
// Continuously reads and prints position, speed, temperature, and
// current draw from an ST3020.
//
// Useful for:
//   - Verifying wiring and communication
//   - Characterising a servo under load
//   - Tuning acceleration and torque limits
//   - Spotting overheating (temperature) or overloading (current)
//
// Output format (tab-separated, paste into a spreadsheet or plotter):
//   pos  speed  temp_C  current_A  moving
//
// Wiring (ESP32):
//   GPIO17 --[1kΩ]--> SERVO BUS DATA
//   GPIO17 <--------- SERVO BUS DATA
//   GND   ----------- SERVO GND
//   5-8.4V ---------- SERVO VCC

#include <Arduino.h>
#include <FeetechBus.h>
#include <FeetechST3020.h>

static constexpr int      BUS_PIN          = 17;    // <-- your ESP32 GPIO
static constexpr uint32_t BAUD             = 1000000;
static constexpr uint32_t PRINT_INTERVAL   = 200;   // ms between readings
static constexpr uint32_t SWEEP_INTERVAL   = 3000;  // ms between sweep moves

FeetechBus    bus(Serial2);
FeetechST3020 st(bus, 11);  // <-- your servo ID

// ---------------------------------------------------------------------------

void printHeader() {
  Serial.println("pos\tspeed\ttemp_C\tcurrent_A\tmoving");
}

void printStatus() {
  uint16_t pos    = 0;
  int16_t  speed  = 0;
  uint8_t  temp   = 0;
  float    amps   = 0.0f;
  bool     moving = false;

  // Read sequentially — never read two servos at the same time on a shared bus.
  bool ok = st.readPresentPosition(pos)
         && st.readCurrentSpeed(speed)
         && st.readCurrentTemperature(temp)
         && st.readCurrentCurrent(amps)
         && st.isMoving(moving);

  if (ok) {
    Serial.print(pos);          Serial.print('\t');
    Serial.print(speed);        Serial.print('\t');
    Serial.print(temp);         Serial.print('\t');
    Serial.print(amps, 3);      Serial.print('\t');
    Serial.println(moving ? "yes" : "no");
  } else {
    Serial.println("ERR\tERR\tERR\tERR\tERR");
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
  Serial.println("Servo found. Starting monitor...\n");
  printHeader();

  FeetechST3020::Profile p;
  p.torqueOnAfterInit = true;
  st.init(p);

  // Start at 0 degrees
  st.moveTime(st.degToTicks(0), 600, 600);
}

void loop() {
  // Sweep back and forth so the monitor shows non-zero speed and moving=yes
  static uint32_t lastSweep  = 0;
  static bool     sweepState = false;

  uint32_t now = millis();
  if (now - lastSweep > SWEEP_INTERVAL) {
    lastSweep  = now;
    sweepState = !sweepState;
    uint16_t target = sweepState ? st.degToTicks(270) : st.degToTicks(0);
    st.moveTime(target, 1200, 700);
  }

  printStatus();
  delay(PRINT_INTERVAL);
}
