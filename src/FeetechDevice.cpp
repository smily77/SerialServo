#include "FeetechDevice.h"

FeetechDevice::FeetechDevice(FeetechBus& bus, uint8_t id, RegMap map)
: _bus(bus), _id(id), _reg(map) {}

bool FeetechDevice::ping() {
  return _bus.ping(_id);
}

bool FeetechDevice::write8(uint8_t addr, uint8_t v) {
  return _bus.writeData(_id, addr, &v, 1);
}

bool FeetechDevice::write16(uint8_t addr, uint16_t v) {
  uint8_t b[2] = { (uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF) };
  return _bus.writeData(_id, addr, b, 2);
}

bool FeetechDevice::read8(uint8_t addr, uint8_t& v) {
  return _bus.readData(_id, addr, 1, &v);
}

bool FeetechDevice::read16(uint8_t addr, uint16_t& v) {
  uint8_t b[2] = {0,0};
  if (!_bus.readData(_id, addr, 2, b)) return false;
  v = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
  return true;
}

bool FeetechDevice::setTorque(bool enable) {
  return write8(_reg.ADDR_TORQUE_ENABLE, enable ? 1 : 0);
}

bool FeetechDevice::setReturnDelay(uint8_t delayUnits) {
  return write8(_reg.ADDR_RETURN_DELAY, delayUnits);
}

bool FeetechDevice::setStatusReturnLevel(uint8_t level) {
  return write8(_reg.ADDR_STATUS_RETURN_LVL, level);
}

bool FeetechDevice::moveTime(uint16_t position, uint16_t time, uint16_t speed) {
  uint8_t b[6] = {
    (uint8_t)(position & 0xFF), (uint8_t)((position >> 8) & 0xFF),
    (uint8_t)(time & 0xFF),     (uint8_t)((time >> 8) & 0xFF),
    (uint8_t)(speed & 0xFF),    (uint8_t)((speed >> 8) & 0xFF)
  };
  return _bus.writeData(_id, _reg.ADDR_GOAL_POSITION_L, b, sizeof(b));
}

bool FeetechDevice::readPresentPosition(uint16_t& pos) {
  return read16(_reg.ADDR_PRESENT_POSITION_L, pos);
}
