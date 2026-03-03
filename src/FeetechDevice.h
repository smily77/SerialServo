#pragma once
#include <Arduino.h>
#include "FeetechBus.h"

class FeetechDevice {
public:
  struct RegMap {
    // Common-ish addresses (vary slightly across models; override per instance if needed)
    uint8_t ADDR_ID                 = 0x05;
    uint8_t ADDR_RETURN_DELAY       = 0x07;
    uint8_t ADDR_STATUS_RETURN_LVL  = 0x08; // might differ by model; override if needed

    uint8_t ADDR_TORQUE_ENABLE      = 0x28;
    uint8_t ADDR_GOAL_POSITION_L    = 0x2A;
    uint8_t ADDR_GOAL_TIME_L        = 0x2C;
    uint8_t ADDR_GOAL_SPEED_L       = 0x2E;

    uint8_t ADDR_PRESENT_POSITION_L = 0x38; // model dependent; override if needed
  };

  FeetechDevice(FeetechBus& bus, uint8_t id, RegMap map = RegMap());

  uint8_t id() const { return _id; }
  void setId(uint8_t id) { _id = id; }

  bool ping();

  bool setTorque(bool enable);
  bool setReturnDelay(uint8_t delayUnits);
  bool setStatusReturnLevel(uint8_t level);

  bool write8(uint8_t addr, uint8_t v);
  bool write16(uint8_t addr, uint16_t v);
  bool read8(uint8_t addr, uint8_t& v);
  bool read16(uint8_t addr, uint16_t& v);

  // Write 6 bytes starting at GOAL_POSITION_L: pos(2) + time(2) + speed(2)
  bool moveTime(uint16_t position, uint16_t time, uint16_t speed);

  bool readPresentPosition(uint16_t& pos);

  const RegMap& reg() const { return _reg; }

protected:
  FeetechBus& _bus;
  uint8_t _id;
  RegMap _reg;
};
