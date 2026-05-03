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
  // Read current mode first — EEPROM write cycles are limited to ~100k.
  // Skip the write entirely if the servo is already in the requested mode.
  uint8_t current = 0xFF;
  if (read8(_reg.ADDR_OPERATION_MODE, current) &&
      current == static_cast<uint8_t>(mode)) {
    return true;  // already correct, no EEPROM write needed
  }
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
  if (_reg.ADDR_TORQUE_LIMIT_L == 0xFF) return false;
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


bool FeetechDevice::moveTimeSignedImpl(int16_t position, uint16_t time, uint16_t speed, bool async) {
  uint16_t rawPos;
  if (_reg.signBit15) {
    rawPos = (uint16_t)(position < 0 ? (uint16_t)(-position) | 0x8000 : (uint16_t)position);
  } else {
    rawPos = (uint16_t)(position < 0 ? (uint16_t)(-position) | 0x0400 : (uint16_t)position);
  }
  return moveTimeImpl(rawPos, time, speed, async);
}
bool FeetechDevice::moveTime(uint16_t position, uint16_t time, uint16_t speed) {
  return moveTimeImpl(position, time, speed, false);
}

bool FeetechDevice::moveTimeSigned(int16_t position, uint16_t time, uint16_t speed) {
  return moveTimeSignedImpl((int16_t)position, time, speed, false);
}

bool FeetechDevice::moveTimeAsync(uint16_t position, uint16_t time, uint16_t speed) {
  return moveTimeImpl(position, time, speed, true);
}

bool FeetechDevice::moveTimeSignedAsync(int16_t position, uint16_t time, uint16_t speed) {
  return moveTimeSignedImpl(position, time, speed, true);
}

bool FeetechDevice::moveEx(int16_t position, uint16_t time, uint16_t speed, uint8_t acc) {
  uint16_t rawPos;
  if (_reg.signBit15) rawPos = (uint16_t)(position < 0 ? (uint16_t)(-position) | 0x8000 : (uint16_t)position);
  else rawPos = (uint16_t)(position < 0 ? (uint16_t)(-position) | 0x0400 : (uint16_t)position);

  uint8_t b[7];
  b[0] = acc;
  auto enc = [&](uint8_t* dst, uint16_t val) {
    if (_reg.bigEndian) { dst[0]=(uint8_t)((val>>8)&0xFF); dst[1]=(uint8_t)(val&0xFF); }
    else { dst[0]=(uint8_t)(val&0xFF); dst[1]=(uint8_t)((val>>8)&0xFF); }
  };
  enc(&b[1], rawPos);
  enc(&b[3], time);
  enc(&b[5], speed);
  return _bus.writeData(_id, _reg.ADDR_TARGET_ACCELERATION, b, sizeof(b), false);
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

bool FeetechDevice::setAngleLimits(uint16_t minPos, uint16_t maxPos) {
  if (!write8(_reg.ADDR_WRITE_LOCK, 0)) return false;
  delay(20);
  // Write all 4 bytes (min_L, min_H, max_L, max_H) in one packet so the servo
  // never receives a second EEPROM write while the first cell is still being
  // programmed (EEPROM write cycle ~1-10 ms; inter-frame gap is only 200 µs).
  uint8_t buf[4];
  if (_reg.bigEndian) {
    buf[0] = (uint8_t)((minPos >> 8) & 0xFF);
    buf[1] = (uint8_t)(minPos & 0xFF);
    buf[2] = (uint8_t)((maxPos >> 8) & 0xFF);
    buf[3] = (uint8_t)(maxPos & 0xFF);
  } else {
    buf[0] = (uint8_t)(minPos & 0xFF);
    buf[1] = (uint8_t)((minPos >> 8) & 0xFF);
    buf[2] = (uint8_t)(maxPos & 0xFF);
    buf[3] = (uint8_t)((maxPos >> 8) & 0xFF);
  }
  bool ok = _bus.writeData(_id, _reg.ADDR_MIN_ANGLE_LIMIT, buf, 4);
  delay(20);
  (void)write8(_reg.ADDR_WRITE_LOCK, 1);
  if (!ok) return false;
  delay(5);
  uint16_t rMin = 0, rMax = 0;
  if (!readAngleLimits(rMin, rMax)) return false;
  return (rMin == minPos && rMax == maxPos);
}

bool FeetechDevice::setPositionOffset(int16_t offset) {
  // ADDR_POSITION_CORRECTION == 0xFF means the register does not exist for this
  // servo family (SCS / SC15). The SCS register map has no calibration offset.
  if (_reg.ADDR_POSITION_CORRECTION == 0xFF) return false;

  if (!write8(_reg.ADDR_WRITE_LOCK, 0)) return false;
  delay(5);
  bool ok = writeSigned16(_reg.ADDR_POSITION_CORRECTION, offset);
  delay(5);
  (void)write8(_reg.ADDR_WRITE_LOCK, 1);
  return ok;
}

bool FeetechDevice::setDeadBand(uint8_t cwTicks, uint8_t ccwTicks) {
  if (!write8(_reg.ADDR_WRITE_LOCK, 0)) return false;
  delay(20);
  // Write both bytes (CW at 0x1A, CCW at 0x1B) in one packet — same reason as
  // setAngleLimits: consecutive EEPROM cells must not be written separately.
  uint8_t buf[2] = { cwTicks, ccwTicks };
  bool ok = _bus.writeData(_id, _reg.ADDR_CW_DEAD_BAND, buf, 2);
  delay(20);
  (void)write8(_reg.ADDR_WRITE_LOCK, 1);
  if (!ok) return false;
  delay(5);
  uint8_t rCw = 0, rCcw = 0;
  if (!readDeadBand(rCw, rCcw)) return false;
  return (rCw == cwTicks && rCcw == ccwTicks);
}

bool FeetechDevice::readDeadBand(uint8_t& cwTicks, uint8_t& ccwTicks) {
  if (!read8(_reg.ADDR_CW_DEAD_BAND, cwTicks)) return false;
  return read8(_reg.ADDR_CCW_DEAD_BAND, ccwTicks);
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

bool FeetechDevice::readAngleLimits(uint16_t& minPos, uint16_t& maxPos) {
  if (!read16(_reg.ADDR_MIN_ANGLE_LIMIT, minPos)) return false;
  return read16(_reg.ADDR_MAX_ANGLE_LIMIT, maxPos);
}
