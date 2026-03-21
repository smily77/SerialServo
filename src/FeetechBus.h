#pragma once
#include <Arduino.h>

/**
 * Feetech bus packet format (common):
 * 0xFF 0xFF ID LEN INST PARAM... CHK
 * CHK = ~(ID + LEN + INST + sum(PARAM)) & 0xFF
 *
 * Status packet (common):
 * 0xFF 0xFF ID LEN ERROR PARAM... CHK
 * CHK = ~(ID + LEN + ERROR + sum(PARAM)) & 0xFF
 */
class FeetechBus {
public:
  struct Options {
    uint32_t baud = 1000000;        // common Feetech default; change if needed
    int8_t   dirPin = -1;           // optional direction pin (HIGH=TX, LOW=RX)
    bool     dirHighIsTx = true;    // if false: LOW=TX, HIGH=RX
    uint16_t interFrameGapUs = 200; // gap between frames
    uint16_t rxTimeoutMs = 20;      // default read timeout
    bool     forbidBroadcast = true;// block ID 0xFE for safety
    uint32_t serialConfig = SERIAL_8N1;
  };

  explicit FeetechBus(HardwareSerial& port);

  // Generic begin (optionally with direction pin)
  bool begin(const Options& opt);

  // ESP32-friendly: 1-wire UART mapping (RX=TX=busPin), no direction pin needed
  bool begin1Wire(uint32_t baud, int8_t busPin, uint32_t serialConfig = SERIAL_8N1);

  // Optional convenience: explicitly set different RX/TX pins (still no dir pin)
  bool beginPins(uint32_t baud, int8_t rxPin, int8_t txPin, uint32_t serialConfig = SERIAL_8N1);

  void end();

  // Low-level send/receive
  bool writePacket(uint8_t id, uint8_t inst, const uint8_t* params, uint8_t paramLen);

  // Read status packet; if inOutParamLen >= actual param len => strict checksum validation
  bool readStatusPacket(uint8_t expectedId,
                        uint8_t* outError,
                        uint8_t* outParams,
                        uint8_t* inOutParamLen,
                        uint16_t timeoutMs = 0);

  // Convenience: write/read "data" using instruction 0x03 (WRITE) or 0x04 (REGWRITE if async)
  bool writeData(uint8_t id, uint8_t address, const uint8_t* data, uint8_t len, bool async = false);
  bool readData(uint8_t id, uint8_t address, uint8_t len, uint8_t* out);

  // Ping (0x01) -> status packet
  bool ping(uint8_t id);

  // ACTION (0x05) broadcast: executes all pending async (REGWRITE) commands simultaneously
  bool triggerAction();

  // SYNC WRITE (0x83) broadcast: write dataLen bytes to startAddr on multiple servos at once.
  // data layout: [id0_byte0..id0_byte(dataLen-1), id1_byte0..., ...] (row-major, count rows)
  bool syncWrite(uint8_t startAddr, uint8_t dataLen,
                 uint8_t count, const uint8_t* ids, const uint8_t* data);

  const Options& options() const { return _opt; }

private:
  HardwareSerial& _ser;
  Options _opt;

  void setTxMode();
  void setRxMode();
  void flushInput();
  void drainEcho(uint8_t count); // 1-Wire: wait for exactly N echo bytes after TX

  static uint8_t checksum(uint8_t id, uint8_t len, uint8_t instOrErr, const uint8_t* params, uint8_t paramLen);
};