#include "FeetechBus.h"

static inline void dirWrite(int8_t pin, bool level) {
  if (pin >= 0) digitalWrite(pin, level ? HIGH : LOW);
}

FeetechBus::FeetechBus(HardwareSerial& port) : _ser(port) {}

bool FeetechBus::begin(const Options& opt) {
  _opt = opt;

  if (_opt.dirPin >= 0) {
    pinMode(_opt.dirPin, OUTPUT);
    setRxMode();
  }

  // NOTE: For AVR, begin(baud) exists. For ESP32, begin(baud, config, rx, tx) exists.
  // We keep begin(opt) generic; user should call begin1Wire/beginPins on ESP32.
  _ser.begin(_opt.baud);
  flushInput();
  return true;
}

bool FeetechBus::begin1Wire(uint32_t baud, int8_t busPin, uint32_t serialConfig) {
  Options opt;
  opt.baud = baud;
  opt.serialConfig = serialConfig;
  opt.dirPin = -1;
  opt.forbidBroadcast = true;
  opt.rxTimeoutMs = 25;
  opt.interFrameGapUs = 200;
  _opt = opt;

  // ESP32: map RX and TX to the same pin for 1-wire UART
  _ser.begin(_opt.baud, _opt.serialConfig, busPin, busPin);
  flushInput();
  return true;
}

bool FeetechBus::beginPins(uint32_t baud, int8_t rxPin, int8_t txPin, uint32_t serialConfig) {
  Options opt;
  opt.baud = baud;
  opt.serialConfig = serialConfig;
  opt.dirPin = -1;
  opt.forbidBroadcast = true;
  opt.rxTimeoutMs = 25;
  opt.interFrameGapUs = 200;
  _opt = opt;

  _ser.begin(_opt.baud, _opt.serialConfig, rxPin, txPin);
  flushInput();
  return true;
}

void FeetechBus::end() {
  _ser.end();
}

void FeetechBus::setTxMode() {
  if (_opt.dirPin < 0) return;
  bool level = _opt.dirHighIsTx ? true : false;
  dirWrite(_opt.dirPin, level);
  delayMicroseconds(5);
}

void FeetechBus::setRxMode() {
  if (_opt.dirPin < 0) return;
  bool level = _opt.dirHighIsTx ? false : true;
  dirWrite(_opt.dirPin, level);
  delayMicroseconds(5);
}

void FeetechBus::flushInput() {
  while (_ser.available()) (void)_ser.read();
}

uint8_t FeetechBus::checksum(uint8_t id, uint8_t len, uint8_t instOrErr, const uint8_t* params, uint8_t paramLen) {
  uint16_t sum = id + len + instOrErr;
  for (uint8_t i = 0; i < paramLen; i++) sum += params[i];
  return (uint8_t)(~sum & 0xFF);
}

bool FeetechBus::writePacket(uint8_t id, uint8_t inst, const uint8_t* params, uint8_t paramLen) {
  if (_opt.forbidBroadcast && id == 0xFE) return false;

  const uint8_t len = (uint8_t)(paramLen + 2); // INST + CHK
  const uint8_t chk = checksum(id, len, inst, params, paramLen);

  delayMicroseconds(_opt.interFrameGapUs);
  flushInput();

  setTxMode();

  _ser.write(0xFF);
  _ser.write(0xFF);
  _ser.write(id);
  _ser.write(len);
  _ser.write(inst);
  for (uint8_t i = 0; i < paramLen; i++) _ser.write(params[i]);
  _ser.write(chk);

  _ser.flush();
  setRxMode();
  return true;
}

bool FeetechBus::readStatusPacket(uint8_t expectedId,
                                  uint8_t* outError,
                                  uint8_t* outParams,
                                  uint8_t* inOutParamLen,
                                  uint16_t timeoutMs) {
  if (timeoutMs == 0) timeoutMs = _opt.rxTimeoutMs;

  const uint32_t t0 = millis();

  // Find header 0xFF 0xFF
  int prev = -1;
  bool found = false;
  while (millis() - t0 < timeoutMs) {
    if (_ser.available()) {
      int b = _ser.read();
      if (prev == 0xFF && b == 0xFF) { found = true; break; }
      prev = b;
    }
    yield();
  }
  if (!found) return false;

  auto readByte = [&](uint8_t& v) -> bool {
    uint32_t t = millis();
    while (!_ser.available()) {
      if (millis() - t > timeoutMs) return false;
      yield();
    }
    v = (uint8_t)_ser.read();
    return true;
  };

  uint8_t id=0, len=0, err=0;
  if (!readByte(id)) return false;
  if (!readByte(len)) return false;
  if (!readByte(err)) return false;

  const uint8_t paramLen = (uint8_t)(len - 2); // after ERROR, before CHK

  // Read params into temp (so we can validate checksum strictly even if caller requests fewer)
  uint8_t tmp[64];
  if (paramLen > sizeof(tmp)) return false;

  for (uint8_t i = 0; i < paramLen; i++) {
    if (!readByte(tmp[i])) return false;
  }

  uint8_t chk=0;
  if (!readByte(chk)) return false;

  // Validate checksum strictly
  const uint8_t calc = checksum(id, len, err, tmp, paramLen);
  if (calc != chk) return false;

  if (id != expectedId) return false;

  if (outError) *outError = err;

  if (inOutParamLen) {
    uint8_t want = *inOutParamLen;
    uint8_t copyLen = (want < paramLen) ? want : paramLen;
    if (outParams && copyLen > 0) memcpy(outParams, tmp, copyLen);
    *inOutParamLen = paramLen;
  } else {
    // If no length pointer provided, still copy nothing; checksum already validated.
    (void)outParams;
  }

  return true;
}

bool FeetechBus::writeData(uint8_t id, uint8_t address, const uint8_t* data, uint8_t len) {
  if (!data || len == 0) return false;
  if (len > 60) return false;

  uint8_t buf[1 + 60];
  buf[0] = address;
  memcpy(&buf[1], data, len);

  return writePacket(id, 0x03 /*WRITE*/, buf, (uint8_t)(len + 1));
}

bool FeetechBus::readData(uint8_t id, uint8_t address, uint8_t len, uint8_t* out) {
  if (!out || len == 0) return false;

  uint8_t params[2] = { address, len };
  if (!writePacket(id, 0x02 /*READ*/, params, 2)) return false;

  uint8_t err = 0;
  uint8_t outLen = len; // caller wants exactly len
  if (!readStatusPacket(id, &err, out, &outLen, _opt.rxTimeoutMs)) return false;

  // outLen is actual param len; we require at least len
  return (err == 0 && outLen >= len);
}

bool FeetechBus::ping(uint8_t id) {
  if (!writePacket(id, 0x01 /*PING*/, nullptr, 0)) return false;

  uint8_t err = 0;
  uint8_t params[1];
  uint8_t plen = sizeof(params);
  return readStatusPacket(id, &err, params, &plen, _opt.rxTimeoutMs) && (err == 0);
}
