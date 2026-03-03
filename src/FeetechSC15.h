#pragma once
#include "FeetechDevice.h"

class FeetechSC15 : public FeetechDevice {
public:
  struct Profile {
    uint16_t posMin = 0;
    uint16_t posMax = 1023; // informational

    uint8_t  statusReturnLevel = 1;
    uint8_t  returnDelayUnits  = 3;
    bool     torqueOnAfterInit = true;
  };

  FeetechSC15(FeetechBus& bus, uint8_t id, RegMap map = RegMap());

  bool init(const Profile& p = Profile());
};
