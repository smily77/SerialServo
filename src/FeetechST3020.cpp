#include "FeetechST3020.h"

FeetechST3020::FeetechST3020(FeetechBus& bus, uint8_t id, RegMap map)
: FeetechDevice(bus, id, map) {}

bool FeetechST3020::init(const Profile& p) {
  if (!ping()) return false;

  (void)setStatusReturnLevel(p.statusReturnLevel);
  (void)setReturnDelay(p.returnDelayUnits);

  if (p.torqueOnAfterInit) (void)setTorque(true);
  return true;
}

uint16_t FeetechST3020::degToTicks(float deg) const {
  while (deg < 0) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;

  float t = (deg / 360.0f) * 4095.0f;
  if (t < 0) t = 0;
  if (t > 4095) t = 4095;
  return (uint16_t)(t + 0.5f);
}
