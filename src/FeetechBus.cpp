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

  // NOTE: For AVR, begin(baud, config) exists. For ESP32, begin(baud, config, rx, tx) exists.
  // We keep begin(opt) generic; user should call begin1Wire/beginPins on ESP32.
  _ser.begin(_opt.baud, _opt.serialConfig);
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

void FeetechBus::drainEcho(uint8_t count) {
  // In 1-wire mode the ESP32 UART RX sees everything TX sends.
  // The RX FIFO interrupt may fire slightly after _ser.flush() returns,
  // so we must wait for exactly `count` bytes rather than using the
  // non-blocking flushInput().
  const uint32_t deadline = millis() + 10; // 10 ms hard cap
  while (count > 0 && (int32_t)(millis() - deadline) < 0) {
    if (_ser.available()) { (void)_ser.read(); count--; }
    yield();
  }
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
  // 1-Wire: drain exactly the bytes we just sent (FF FF ID LEN INST [params] CHK)
  if (_opt.dirPin < 0) drainEcho((uint8_t)(paramLen + 6));
  else                 flushInput();
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

  if (len < 2) return false; // Mindestlänge: ERROR + CHK
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

bool FeetechBus::writeData(uint8_t id, uint8_t address, const uint8_t* data, uint8_t len, bool async) {
  if (!data || len == 0) return false;
  if (len > 60) return false;

  uint8_t buf[1 + 60];
  buf[0] = address;
  memcpy(&buf[1], data, len);

  const uint8_t inst = async ? 0x04 /*REGWRITE*/ : 0x03 /*WRITE*/;
  return writePacket(id, inst, buf, (uint8_t)(len + 1));
}

bool FeetechBus::triggerAction() {
  // ACTION (0x05) is always sent to broadcast 0xFE — bypasses forbidBroadcast intentionally
  const uint8_t id   = 0xFE;
  const uint8_t len  = 2; // INST + CHK
  const uint8_t inst = 0x05;
  const uint8_t chk  = (uint8_t)(~((uint16_t)id + len + inst) & 0xFF);

  delayMicroseconds(_opt.interFrameGapUs);
  flushInput();
  setTxMode();

  _ser.write(0xFF); _ser.write(0xFF);
  _ser.write(id);   _ser.write(len);
  _ser.write(inst); _ser.write(chk);

  _ser.flush();
  // ACTION is 6 bytes: FF FF FE LEN INST CHK
  if (_opt.dirPin < 0) drainEcho(6);
  else                 flushInput();
  setRxMode();
  return true;
}

bool FeetechBus::syncWrite(uint8_t startAddr, uint8_t dataLen,
                           uint8_t count, const uint8_t* ids, const uint8_t* data) {
  if (!ids || !data || count == 0 || dataLen == 0) return false;

  // LEN = INST(1) + startAddr(1) + dataLen(1) + count*(id(1)+data(dataLen)) + CHK(1)
  const uint16_t len16 = 4u + (uint16_t)count * (1u + dataLen);
  if (len16 > 255u) return false;
  const uint8_t len  = (uint8_t)len16;
  const uint8_t inst = 0x83; // SYNC WRITE

  uint16_t chkSum = (uint16_t)0xFE + len + inst + startAddr + dataLen;
  for (uint8_t i = 0; i < count; i++) {
    chkSum += ids[i];
    for (uint8_t j = 0; j < dataLen; j++)
      chkSum += data[(uint16_t)i * dataLen + j];
  }
  const uint8_t chk = (uint8_t)(~chkSum & 0xFF);

  delayMicroseconds(_opt.interFrameGapUs);
  flushInput();
  setTxMode();

  _ser.write(0xFF); _ser.write(0xFF);
  _ser.write((uint8_t)0xFE); _ser.write(len);
  _ser.write(inst);
  _ser.write(startAddr); _ser.write(dataLen);
  for (uint8_t i = 0; i < count; i++) {
    _ser.write(ids[i]);
    for (uint8_t j = 0; j < dataLen; j++)
      _ser.write(data[(uint16_t)i * dataLen + j]);
  }
  _ser.write(chk);

  _ser.flush();
  // SYNC WRITE: FF FF FE LEN INST startAddr dataLen [id data...] CHK
  // total = 2 + 1 + 1 + len + 1 = len + 5   where len already includes INST(1)+..+CHK(1)? No:
  // len field = 4 + count*(1+dataLen), total wire bytes = len + 4 (FF FF ID LEN already counted)
  // Simpler: total = 2(hdr) + 1(id) + 1(len) + len(payload incl inst & chk) = len + 4
  if (_opt.dirPin < 0) drainEcho((uint8_t)(len + 4));
  else                 flushInput();
  setRxMode();
  return true;
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
  uint8_t plen = 0; // Ping-Response hat keine Parameter
  return readStatusPacket(id, &err, nullptr, &plen, _opt.rxTimeoutMs) && (err == 0);
}
