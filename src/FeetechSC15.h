#pragma once
#include "FeetechDevice.h"

class FeetechSC15 : public FeetechDevice {
public:
  struct Profile {
    uint16_t posMin;
    uint16_t posMax;       // informational
    uint8_t  statusReturnLevel;
    uint8_t  returnDelayUnits;
    bool     torqueOnAfterInit;

    Profile() : posMin(0), posMax(1023), statusReturnLevel(1), returnDelayUnits(3), torqueOnAfterInit(true) {}
  };

  FeetechSC15(FeetechBus& bus, uint8_t id, RegMap map = RegMap());

  bool init(const Profile& p = Profile());
};
