#include "FeetechDevice.h"

FeetechDevice::FeetechDevice(FeetechBus& bus, uint8_t id, RegMap map)
: _bus(bus), _id(id), _reg(map) {}

// ---------------------------------------------------------------------------
// Low-level register helpers (endian-aware)
// ---------------------------------------------------------------------------

bool FeetechDevice::write8(uint8_t addr, uint8_t v) {
  return _bus.writeData(_id, addr, &v, 1);
}

bool FeetechDevice::write16(uint8_t addr, uint16_t v) {
  uint8_t b[2];
  if (_reg.bigEndian) {
    b[0] = (uint8_t)((v >> 8) & 0xFF);
    b[1] = (uint8_t)(v & 0xFF);
  } else {
    b[0] = (uint8_t)(v & 0xFF);
    b[1] = (uint8_t)((v >> 8) & 0xFF);
  }
  return _bus.writeData(_id, addr, b, 2);
}

bool FeetechDevice::writeSigned16(uint8_t addr, int16_t v) {
  // Encode sign into the raw uint16 according to the servo family convention
  uint16_t raw;
  if (_reg.signBit15) {
    // STS: bit 15 = sign, bits 14..0 = magnitude
    raw = (uint16_t)(v < 0 ? (-v) | 0x8000 : v);
  } else {
    // SCS: bit 10 = sign, bits 9..0 = magnitude
    raw = (uint16_t)(v < 0 ? (-v) | 0x0400 : v);
  }
  uint8_t b[2];
  if (_reg.bigEndian) {
    b[0] = (uint8_t)((raw >> 8) & 0xFF);
    b[1] = (uint8_t)(raw & 0xFF);
  } else {
    b[0] = (uint8_t)(raw & 0xFF);
    b[1] = (uint8_t)((raw >> 8) & 0xFF);
  }
  return _bus.writeData(_id, addr, b, 2);
}

bool FeetechDevice::read8(uint8_t addr, uint8_t& v) {
  return _bus.readData(_id, addr, 1, &v);
}

bool FeetechDevice::read16(uint8_t addr, uint16_t& v) {
  uint8_t b[2] = {0, 0};
  if (!_bus.readData(_id, addr, 2, b)) return false;
  if (_reg.bigEndian) {
    v = ((uint16_t)b[0] << 8) | b[1];
  } else {
    v = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
  }
  return true;
}

bool FeetechDevice::readSigned16(uint8_t addr, int16_t& v) {
  uint8_t b[2] = {0, 0};
  if (!_bus.readData(_id, addr, 2, b)) return false;
  uint16_t raw;
  if (_reg.bigEndian) {
    raw = ((uint16_t)b[0] << 8) | b[1];
  } else {
    raw = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
  }
  if (_reg.signBit15) {
    // STS: bit 15 = sign
    int16_t mag = (int16_t)(raw & 0x7FFF);
    v = (raw & 0x8000) ? -mag : mag;
  } else {
    // SCS: bit 10 = sign
    int16_t mag = (int16_t)(raw & 0x03FF);
    v = (raw & 0x0400) ? -mag : mag;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

bool FeetechDevice::ping() {
  return _bus.ping(_id);
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

bool FeetechDevice::setMode(ServoMode mode) {
  // OPERATION_MODE (0x21) liegt im EEPROM-Bereich → EEPROM muss entsperrt werden
  if (!write8(_reg.ADDR_WRITE_LOCK, 0)) return false;
  delay(5);
  bool ok = write8(_reg.ADDR_OPERATION_MODE, static_cast<uint8_t>(mode));
  delay(5);
  (void)write8(_reg.ADDR_WRITE_LOCK, 1);
  return ok;
}

bool FeetechDevice::setAcceleration(uint8_t acc) {
  return write8(_reg.ADDR_TARGET_ACCELERATION, acc);
}

bool FeetechDevice::setTorqueLimit(uint16_t limit) {
  return write16(_reg.ADDR_TORQUE_LIMIT_L, limit);
}

// ---------------------------------------------------------------------------
// Motion
// ---------------------------------------------------------------------------

bool FeetechDevice::moveTimeImpl(uint16_t position, uint16_t time, uint16_t speed, bool async) {
  // Encode three uint16 values back-to-back, respecting endianness
  uint8_t b[6];
  auto enc = [&](uint8_t* dst, uint16_t val) {
    if (_reg.bigEndian) {
      dst[0] = (uint8_t)((val >> 8) & 0xFF);
      dst[1] = (uint8_t)(val & 0xFF);
    } else {
      dst[0] = (uint8_t)(val & 0xFF);
      dst[1] = (uint8_t)((val >> 8) & 0xFF);
    }
  };
  enc(&b[0], position);
  enc(&b[2], time);
  enc(&b[4], speed);
  return _bus.writeData(_id, _reg.ADDR_GOAL_POSITION_L, b, sizeof(b), async);
}

bool FeetechDevice::moveTime(uint16_t position, uint16_t time, uint16_t speed) {
  return moveTimeImpl(position, time, speed, false);
}

bool FeetechDevice::moveTimeAsync(uint16_t position, uint16_t time, uint16_t speed) {
  return moveTimeImpl(position, time, speed, true);
}

bool FeetechDevice::setTargetVelocity(int16_t velocity) {
  return writeSigned16(_reg.ADDR_GOAL_SPEED_L, velocity);
}

bool FeetechDevice::triggerAction() {
  return _bus.triggerAction();
}

// ---------------------------------------------------------------------------
// EEPROM-protected settings (unlock -> write -> relock)
// ---------------------------------------------------------------------------

bool FeetechDevice::setId(uint8_t newId) {
  if (newId >= 0xFE) return false;

  if (!write8(_reg.ADDR_WRITE_LOCK, 0)) return false; // unlock (old ID)
  delay(5);
  if (!write8(_reg.ADDR_ID, newId)) return false;     // servo switches to newId
  delay(5);
  _id = newId;                                         // update before next write
  (void)write8(_reg.ADDR_WRITE_LOCK, 1);              // relock (new ID)
  return true;
}

bool FeetechDevice::setPositionOffset(int16_t offset) {
  if (!write8(_reg.ADDR_WRITE_LOCK, 0)) return false;
  delay(5);
  bool ok = writeSigned16(_reg.ADDR_POSITION_CORRECTION, offset);
  delay(5);
  (void)write8(_reg.ADDR_WRITE_LOCK, 1);
  return ok;
}

// ---------------------------------------------------------------------------
// Status reads
// ---------------------------------------------------------------------------

bool FeetechDevice::readPresentPosition(uint16_t& pos) {
  return read16(_reg.ADDR_PRESENT_POSITION_L, pos);
}

bool FeetechDevice::readCurrentSpeed(int16_t& speed) {
  return readSigned16(_reg.ADDR_CURRENT_SPEED_L, speed);
}

bool FeetechDevice::readCurrentTemperature(uint8_t& degC) {
  return read8(_reg.ADDR_CURRENT_TEMPERATURE, degC);
}

bool FeetechDevice::readCurrentCurrent(float& amps) {
  int16_t raw;
  if (!readSigned16(_reg.ADDR_CURRENT_CURRENT_L, raw)) return false;
  amps = raw * 0.0065f; // 6.5 mA per count
  return true;
}

bool FeetechDevice::isMoving(bool& moving) {
  uint8_t v;
  if (!read8(_reg.ADDR_MOVING_STATUS, v)) return false;
  moving = (v != 0);
  return true;
}
