#include "FeetechSC15.h"

FeetechSC15::FeetechSC15(FeetechBus& bus, uint8_t id, RegMap map)
: FeetechDevice(bus, id, map) {
  // SCS protocol (SC15): 16-bit registers are big-endian, sign is bit 10 (not bit 15)
  _reg.bigEndian  = true;
  _reg.signBit15  = false;
  // On SCS, the EEPROM write-lock lives at the TORQUE_LIMIT address (0x30)
  _reg.ADDR_WRITE_LOCK = 0x30;
}

uint16_t FeetechSC15::degToTicks(float deg) const {
  if (deg < 0) deg = 0;
  if (deg > 300.0f) deg = 300.0f;
  float t = (deg / 300.0f) * 1023.0f;
  return (uint16_t)(t + 0.5f);
}

bool FeetechSC15::init(const Profile& p) {
  if (!ping()) return false;

  (void)setStatusReturnLevel(p.statusReturnLevel);
  (void)setReturnDelay(p.returnDelayUnits);

  if (p.torqueOnAfterInit) (void)setTorque(true);
  return true;
}
