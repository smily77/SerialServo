#pragma once
#include <Arduino.h>
#include "FeetechBus.h"

// Servo operating mode (register ADDR_OPERATION_MODE)
enum class ServoMode : uint8_t {
  POSITION = 0, // Default: position control
  VELOCITY = 1, // Continuous rotation with speed target
  STEP     = 3  // Step / multi-turn mode
};

class FeetechDevice {
public:
  struct RegMap {
    // EEPROM area (common SCS/STS)
    uint8_t ADDR_ID;
    uint8_t ADDR_RETURN_DELAY;
    uint8_t ADDR_STATUS_RETURN_LVL;
    uint8_t ADDR_MIN_ANGLE_LIMIT;       // lower position limit stored in EEPROM
    uint8_t ADDR_MAX_ANGLE_LIMIT;       // upper position limit stored in EEPROM
    uint8_t ADDR_CW_DEAD_BAND;          // clockwise dead band in ticks (EEPROM, 1 byte)
    uint8_t ADDR_CCW_DEAD_BAND;         // counter-clockwise dead band in ticks (EEPROM, 1 byte)
    uint8_t ADDR_POSITION_CORRECTION;   // signed calibration offset (STS only; 0xFF = not supported)
    uint8_t ADDR_OPERATION_MODE;        // ServoMode enum

    // RAM area
    uint8_t ADDR_TORQUE_ENABLE;
    uint8_t ADDR_TARGET_ACCELERATION; // 0=no limit; unit depends on model
    uint8_t ADDR_GOAL_POSITION_L;
    uint8_t ADDR_GOAL_TIME_L;
    uint8_t ADDR_GOAL_SPEED_L;        // speed limit (pos mode) or target (vel mode)
    uint8_t ADDR_TORQUE_LIMIT_L;
    uint8_t ADDR_WRITE_LOCK;          // STS default 0x37; SCS uses 0x30 (overridden in SC15)

    uint8_t ADDR_PRESENT_POSITION_L;
    uint8_t ADDR_CURRENT_SPEED_L;     // signed: direction encoded via sign bit
    uint8_t ADDR_CURRENT_TEMPERATURE; // degrees Celsius, 1 byte
    uint8_t ADDR_MOVING_STATUS;       // 0 = stopped, >0 = moving
    uint8_t ADDR_CURRENT_CURRENT_L;   // raw current; multiply by 0.0065 for Amps

    // Protocol encoding flags (set per servo family in subclass constructors)
    bool bigEndian; // true for SCS (SC15): 16-bit regs sent MSB first
    bool signBit15; // false for SCS: sign is bit 10 (0x0400), not bit 15

    RegMap()
      : ADDR_ID(0x05),
        ADDR_RETURN_DELAY(0x07),
        ADDR_STATUS_RETURN_LVL(0x08),
        ADDR_MIN_ANGLE_LIMIT(0x09),
        ADDR_MAX_ANGLE_LIMIT(0x0B),
        ADDR_CW_DEAD_BAND(0x1A),
        ADDR_CCW_DEAD_BAND(0x1B),
        ADDR_POSITION_CORRECTION(0x1F),
        ADDR_OPERATION_MODE(0x21),
        ADDR_TORQUE_ENABLE(0x28),
        ADDR_TARGET_ACCELERATION(0x29),
        ADDR_GOAL_POSITION_L(0x2A),
        ADDR_GOAL_TIME_L(0x2C),
        ADDR_GOAL_SPEED_L(0x2E),
        ADDR_TORQUE_LIMIT_L(0x30),
        ADDR_WRITE_LOCK(0x37),
        ADDR_PRESENT_POSITION_L(0x38),
        ADDR_CURRENT_SPEED_L(0x3A),
        ADDR_CURRENT_TEMPERATURE(0x3F),
        ADDR_MOVING_STATUS(0x42),
        ADDR_CURRENT_CURRENT_L(0x45),
        bigEndian(false),
        signBit15(true)
    {}
  };

  FeetechDevice(FeetechBus& bus, uint8_t id, RegMap map = RegMap());

  uint8_t id() const { return _id; }
  void setId_local(uint8_t id) { _id = id; } // update local ID without writing to servo

  bool ping();

  // --- Configuration (RAM) ---
  bool setTorque(bool enable);
  bool setReturnDelay(uint8_t delayUnits);
  bool setStatusReturnLevel(uint8_t level);
  bool setMode(ServoMode mode);
  bool setAcceleration(uint8_t acc);
  bool setTorqueLimit(uint16_t limit);

  // --- Motion ---
  // Write Goal Position + Time + Speed in one packet (synchronous WRITE)
  bool moveTime(uint16_t position, uint16_t time, uint16_t speed);
  // Signed variant (mainly for ST step mode): negative values are encoded
  // using the family-specific sign bit (STS: bit15, SCS: bit10).
  bool moveTimeSigned(int16_t position, uint16_t time, uint16_t speed);
  // Same but uses REGWRITE (async); execute with triggerAction()
  bool moveTimeAsync(uint16_t position, uint16_t time, uint16_t speed);
  bool moveTimeSignedAsync(int16_t position, uint16_t time, uint16_t speed);
  // Extended move: writes ACC + Position + Time + Speed in one packet
  // (official ST/SMS command layout at ADDR_TARGET_ACCELERATION).
  bool moveEx(int16_t position, uint16_t time, uint16_t speed, uint8_t acc);
  // Velocity/step mode: set running speed (signed: direction matters)
  bool setTargetVelocity(int16_t velocity);
  // Trigger execution of all pending async (REGWRITE) commands on all servos
  bool triggerAction();

  // --- EEPROM-protected settings (unlocks/relocks EEPROM automatically) ---
  bool setId(uint8_t newId);
  // Clamp position range in EEPROM (both families).
  // Units are raw ticks: 0–4095 for ST3020, 0–1023 for SC15.
  // Use degToTicks() to convert: sc.setAngleLimits(sc.degToTicks(30), sc.degToTicks(150))
  bool setAngleLimits(uint16_t minPos, uint16_t maxPos);
  // Signed calibration offset — STS (ST3020/ST3215) only.
  // Returns false on SC15 because the SCS register map has no offset register.
  bool setPositionOffset(int16_t offset);
  // Dead band: servo ignores position error smaller than this threshold.
  // Both CW and CCW values are stored in EEPROM and verified by read-back.
  // Pass the same value for both to set a symmetric dead zone.
  bool setDeadBand(uint8_t cwTicks, uint8_t ccwTicks);
  bool readDeadBand(uint8_t& cwTicks, uint8_t& ccwTicks);

  // --- Status reads ---
  bool readPresentPosition(uint16_t& pos);
  bool readCurrentSpeed(int16_t& speed);    // signed (direction-aware)
  bool readCurrentTemperature(uint8_t& degC);
  bool readCurrentCurrent(float& amps);     // converted: count * 0.0065 A
  bool isMoving(bool& moving);
  bool readAngleLimits(uint16_t& minPos, uint16_t& maxPos);

  // --- Low-level register access ---
  bool write8(uint8_t addr, uint8_t v);
  bool write16(uint8_t addr, uint16_t v);             // unsigned, endian-aware
  bool writeSigned16(uint8_t addr, int16_t v);        // signed, endian + sign-bit aware
  bool read8(uint8_t addr, uint8_t& v);
  bool read16(uint8_t addr, uint16_t& v);             // unsigned, endian-aware
  bool readSigned16(uint8_t addr, int16_t& v);        // signed, endian + sign-bit aware

  const RegMap& reg() const { return _reg; }

protected:
  FeetechBus& _bus;
  uint8_t _id;
  RegMap _reg;

private:
  bool moveTimeImpl(uint16_t position, uint16_t time, uint16_t speed, bool async);
  bool moveTimeSignedImpl(int16_t position, uint16_t time, uint16_t speed, bool async);
};
