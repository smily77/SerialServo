#include <Arduino.h>
#include <FeetechBus.h>
#include <FeetechST3020.h>
#include <FeetechSC15.h>

static constexpr int      TX_PIN = 17;    // <-- TX through 1kΩ to bus
static constexpr int      RX_PIN = 18;    // <-- RX directly to bus
static constexpr uint32_t BAUD   = 1000000;

FeetechBus bus(Serial2);

FeetechST3020 st(bus, 11); // ST3020 ID
FeetechSC15   sc(bus, 21); // SC15 ID

void setup() {
  Serial.begin(115200);
  delay(200);

  bus.beginPins(BAUD, RX_PIN, TX_PIN);

  Serial.println("Pinging servos...");
  Serial.print("ST3020 ping: "); Serial.println(st.ping() ? "OK" : "FAIL");
  Serial.print("SC15   ping: "); Serial.println(sc.ping() ? "OK" : "FAIL");

  // Init: keep bus quiet + stagger any potential replies
  FeetechST3020::Profile pSt;
  pSt.statusReturnLevel = 1;
  pSt.returnDelayUnits  = 0;
  pSt.torqueOnAfterInit = true;

  FeetechSC15::Profile pSc;
  pSc.statusReturnLevel = 1;
  pSc.returnDelayUnits  = 3;
  pSc.torqueOnAfterInit = true;

  st.init(pSt);
  sc.init(pSc);

  // Example moves
  st.moveTime(st.degToTicks(90), 300, 800);
  delay(400);

  sc.moveTime(512, 300, 600);
}

void loop() {
  // Avoid SYNC READ collisions: read one at a time
  uint16_t pos = 0;

  if (st.readPresentPosition(pos)) {
    Serial.print("ST pos: "); Serial.println(pos);
  } else {
    Serial.println("ST read fail");
  }
  delay(200);

  if (sc.readPresentPosition(pos)) {
    Serial.print("SC pos: "); Serial.println(pos);
  } else {
    Serial.println("SC read fail");
  }
  delay(800);
}