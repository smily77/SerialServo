#pragma once
#include "FeetechDevice.h"

// STS-family servos (ST3020, ST3215, and others) share the same register map,
// protocol (STS, little-endian, bit-15 sign), and 12-bit encoder (0-4095 ticks/rev).
// FeetechST3215 is a type alias — use either name, they are identical.

class FeetechST3020 : public FeetechDevice {
public:
  struct Profile {
    uint16_t posMin;
    uint16_t posMax;
    uint8_t  statusReturnLevel; // 0=none, 1=read only, 2=all (model dependent)
    uint8_t  returnDelayUnits;  // stagger replies to avoid collisions
    bool     torqueOnAfterInit;

    Profile() : posMin(0), posMax(4095), statusReturnLevel(1), returnDelayUnits(0), torqueOnAfterInit(true) {}
  };

  FeetechST3020(FeetechBus& bus, uint8_t id, RegMap map = RegMap());

  bool init(const Profile& p = Profile());

  // Best-effort: assumes 360deg span => 0..4095 ticks
  uint16_t degToTicks(float deg) const;
};

// ST3215 uses the same STS protocol and identical register map as ST3020.
using FeetechST3215 = FeetechST3020;
