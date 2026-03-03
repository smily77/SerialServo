// SyncedMove.ino
//
// Two servos (ST3020 + SC15) start their moves at exactly the same moment.
//
// How it works:
//   moveTimeAsync()  → sends REGWRITE — servo stores the target but does NOT start yet.
//   triggerAction()  → broadcasts ACTION — both servos start simultaneously.
//
// Use case: robot arms, pan-tilt rigs, or any multi-servo mechanism where
// coordinated motion matters.
//
// Wiring (ESP32):
//   GPIO17 --[1kΩ]--> SERVO BUS DATA
//   GPIO17 <--------- SERVO BUS DATA
//   GND   ----------- SERVO GND
//   5-8.4V ---------- SERVO VCC

#include <Arduino.h>
#include <FeetechBus.h>
#include <FeetechST3020.h>
#include <FeetechSC15.h>

static constexpr int      BUS_PIN = 17;     // <-- your ESP32 GPIO
static constexpr uint32_t BAUD    = 1000000;

FeetechBus    bus(Serial2);
FeetechST3020 st(bus, 11);  // ST3020 servo ID 11
FeetechSC15   sc(bus, 21);  // SC15   servo ID 21

// ---------------------------------------------------------------------------

// Queue a move on both servos, then fire both at once.
void syncedMove(uint16_t stPos, uint16_t stTime, uint16_t stSpeed,
                uint16_t scPos, uint16_t scTime, uint16_t scSpeed) {
  // REGWRITE: servo receives the target but does not start.
  st.moveTimeAsync(stPos, stTime, stSpeed);
  sc.moveTimeAsync(scPos, scTime, scSpeed);

  // ACTION broadcast: both servos start simultaneously.
  st.triggerAction();   // triggerAction() is a bus-level broadcast; calling it
                        // on either servo object has the same effect.
  Serial.println("Both servos started.");
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(200);

  bus.begin1Wire(BAUD, BUS_PIN);

  Serial.print("ST3020 ping: "); Serial.println(st.ping() ? "OK" : "FAIL");
  Serial.print("SC15   ping: "); Serial.println(sc.ping() ? "OK" : "FAIL");

  // Stagger return delays so replies don't collide on the shared bus.
  FeetechST3020::Profile pSt;
  pSt.returnDelayUnits  = 0;   // ST3020 replies immediately
  pSt.torqueOnAfterInit = true;

  FeetechSC15::Profile pSc;
  pSc.returnDelayUnits  = 4;   // SC15 waits ~500 µs before reply
  pSc.torqueOnAfterInit = true;

  st.init(pSt);
  sc.init(pSc);
}

void loop() {
  Serial.println("\n-> ST3020 to 180 deg, SC15 to center (512)");
  syncedMove(
    st.degToTicks(180), 900, 800,
    512,                900, 600
  );
  delay(1400);

  Serial.println("\n-> ST3020 to 90 deg, SC15 to quarter (256)");
  syncedMove(
    st.degToTicks(90), 700, 900,
    256,               700, 700
  );
  delay(1100);

  Serial.println("\n-> ST3020 to 0 deg, SC15 to 0");
  syncedMove(
    st.degToTicks(0), 900, 800,
    0,                900, 600
  );
  delay(1400);
}
