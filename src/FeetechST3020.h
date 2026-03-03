#pragma once
#include "FeetechDevice.h"

class FeetechST3020 : public FeetechDevice {
public:
  struct Profile {
    uint16_t posMin = 0;
    uint16_t posMax = 4095;

    uint8_t  statusReturnLevel = 1; // 0=none, 1=read only, 2=all (model dependent)
    uint8_t  returnDelayUnits  = 0; // stagger replies to avoid collisions
    bool     torqueOnAfterInit = true;
  };

  FeetechST3020(FeetechBus& bus, uint8_t id, RegMap map = RegMap());

  bool init(const Profile& p = Profile());

  // Best-effort: assumes 360deg span => 0..4095 ticks
  uint16_t degToTicks(float deg) const;
};
