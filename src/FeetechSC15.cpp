#include "FeetechSC15.h"

FeetechSC15::FeetechSC15(FeetechBus& bus, uint8_t id, RegMap map)
: FeetechDevice(bus, id, map) {}

bool FeetechSC15::init(const Profile& p) {
  if (!ping()) return false;

  (void)setStatusReturnLevel(p.statusReturnLevel);
  (void)setReturnDelay(p.returnDelayUnits);

  if (p.torqueOnAfterInit) (void)setTorque(true);
  return true;
}
