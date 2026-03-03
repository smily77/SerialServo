# FeetechBusServos

Arduino library for Feetech bus servos (SCS/STS protocol) on ESP32 and compatible boards.

Supports **ST3020** (STS, 360°, continuous rotation) and **SC15** (SCS, 180°) on a shared half-duplex UART bus.

---

## Features

- Half-duplex 1-Wire UART — single GPIO, no direction pin needed
- Typed device classes — no accidental cross-model register access
- Endian-correct 16-bit register I/O (STS = little-endian, SCS = big-endian)
- Sign-bit encoding per family (STS bit 15, SCS bit 10)
- Broadcast protection (ID 0xFE is blocked)
- Checksum validation on every response
- Synchronized multi-servo moves via `moveTimeAsync` + `triggerAction`
- Three operating modes: Position, Velocity (continuous rotation), Step (multi-turn)
- EEPROM auto-unlock for protected settings (ID, mode, calibration offset)

---

## Hardware: 1-Wire Half-Duplex (ESP32)

```
ESP32 GPIO17 ---[1kΩ]--- SERVO BUS DATA
ESP32 GPIO17 <---------- SERVO BUS DATA
GND         ------------ SERVO GND
5–8.4 V     ------------ SERVO VCC
```

The 1 kΩ resistor decouples TX from the bus and prevents echo corruption.
Recommended, though many setups work without it.

For separate RX/TX pins use `bus.beginPins(baud, rxPin, txPin)` instead.

---

## Quick Start

```cpp
#include <FeetechBus.h>
#include <FeetechST3020.h>

FeetechBus    bus(Serial2);
FeetechST3020 st(bus, 11);   // servo ID 11

void setup() {
  bus.begin1Wire(1000000, 17);  // 1 MBaud, GPIO17
  st.init();

  st.moveTime(st.degToTicks(90), 500, 800);  // go to 90°, 500 ms, speed 800
}

void loop() {}
```

---

## API Reference

### Bus initialisation

| Method | Description |
|--------|-------------|
| `bus.begin1Wire(baud, pin)` | 1-Wire half-duplex: RX = TX = pin |
| `bus.beginPins(baud, rx, tx)` | Separate RX / TX pins |

### Servo setup

| Method | Description |
|--------|-------------|
| `ping()` | Returns `true` if the servo responds on the bus |
| `init(Profile)` | Apply Profile settings (return delay, torque, status level) |

### Motion

| Method | Description |
|--------|-------------|
| `moveTime(pos, time, speed)` | Move to position; `time` in ms, `speed` 0–32767 |
| `moveTimeAsync(pos, time, speed)` | Queue move (REGWRITE); does not start until `triggerAction()` |
| `triggerAction()` | Broadcast ACTION — starts all queued async moves simultaneously |
| `setTargetVelocity(v)` | Velocity mode: signed speed, negative = reverse, 0 = stop |

### Status reads

| Method | Description |
|--------|-------------|
| `readPresentPosition(pos)` | Current position in ticks |
| `readCurrentSpeed(speed)` | Signed speed — negative means reverse direction |
| `readCurrentTemperature(degC)` | Temperature in °C |
| `readCurrentCurrent(amps)` | Current draw in Amperes (1 count = 6.5 mA) |
| `isMoving(moving)` | `true` while the servo is in motion |

### Configuration (RAM — safe to call anytime)

| Method | Description |
|--------|-------------|
| `setTorque(bool)` | Enable or disable holding torque |
| `setAcceleration(acc)` | Acceleration ramp (0 = no limit) |
| `setTorqueLimit(limit)` | Maximum torque (0–1000) |

### EEPROM settings (persistent across power cycles)

> These methods automatically unlock and relock EEPROM. Call them **once at setup**, not in `loop()`. EEPROM is rated for ~100 000 write cycles.

| Method | Description |
|--------|-------------|
| `setMode(ServoMode)` | Operating mode: POSITION, VELOCITY, or STEP |
| `setId(newId)` | Change servo ID |
| `setPositionOffset(offset)` | Signed calibration offset |

### ST3020-specific

| Method | Description |
|--------|-------------|
| `degToTicks(deg)` | Convert degrees to ticks (0–360° maps to 0–4095) |

---

## Operating Modes

```cpp
enum class ServoMode : uint8_t {
  POSITION = 0,  // Default: position control with moveTime()
  VELOCITY = 1,  // Continuous rotation with setTargetVelocity()
  STEP     = 3   // Multi-turn step mode (32-bit position counter)
};
```

### Position Mode (default)

```cpp
st.moveTime(st.degToTicks(90), 300, 800);   // 90°, 300 ms, speed 800
```

### Velocity Mode — continuous rotation (ST3020)

`setMode()` writes EEPROM, `setTargetVelocity()` writes RAM:

```cpp
// setup() — once:
st.setMode(ServoMode::VELOCITY);

// loop() — safe to call every iteration:
st.setTargetVelocity( 800);   // forward
st.setTargetVelocity(-800);   // reverse
st.setTargetVelocity(   0);   // stop

// Switch back to position mode later:
st.setMode(ServoMode::POSITION);
st.moveTime(st.degToTicks(0), 500, 500);
```

---

## Examples

| Example | What it shows |
|---------|---------------|
| `DualServo_SameUART` | ST3020 + SC15 on one bus — init, move, sequential position read |
| `BasicPositionControl` | Move through several angles, wait for completion via `isMoving()`, read position and speed |
| `ContinuousRotation` | Velocity mode — forward / stop / reverse / stop with live speed readback |
| `SyncedMove` | Two servos start simultaneously with `moveTimeAsync` + `triggerAction` |
| `StatusMonitor` | Live readout of position, speed, temperature, and current |

---

### Example: BasicPositionControl

Moves an ST3020 through 0° → 90° → 180° → 270° → 0°, blocking until each move finishes.

```cpp
void moveTo(float deg, uint16_t timeMs, uint16_t speed) {
  st.moveTime(st.degToTicks(deg), timeMs, speed);

  delay(60);  // let the servo start before polling isMoving()

  bool moving = true;
  while (moving) {
    st.isMoving(moving);
    delay(20);
  }

  uint16_t pos; int16_t spd;
  st.readPresentPosition(pos);
  st.readCurrentSpeed(spd);
  Serial.print("pos="); Serial.print(pos);
  Serial.print("  speed="); Serial.println(spd);
}

void loop() {
  moveTo(  0, 900, 600);
  moveTo( 90, 600, 900);
  moveTo(180, 600, 900);
  moveTo(270, 600, 900);
}
```

---

### Example: ContinuousRotation (ST3020)

```cpp
void setup() {
  bus.begin1Wire(1000000, 17);
  st.init();
  st.setMode(ServoMode::VELOCITY);  // written to EEPROM — once at setup
}

void loop() {
  st.setTargetVelocity(800);   // forward
  delay(2000);

  st.setTargetVelocity(0);     // stop
  delay(600);

  st.setTargetVelocity(-600);  // reverse
  delay(2000);

  st.setTargetVelocity(0);     // stop
  delay(1000);
}
```

---

### Example: SyncedMove (ST3020 + SC15)

```cpp
// Queue on both servos — neither starts yet
st.moveTimeAsync(st.degToTicks(180), 900, 800);
sc.moveTimeAsync(512,                900, 600);

// Fire: both start at exactly the same moment
st.triggerAction();
```

---

### Example: StatusMonitor

```cpp
uint16_t pos;  int16_t speed;  uint8_t temp;  float amps;  bool moving;

st.readPresentPosition(pos);
st.readCurrentSpeed(speed);
st.readCurrentTemperature(temp);
st.readCurrentCurrent(amps);    // 1 count = 6.5 mA
st.isMoving(moving);

Serial.print(pos);   Serial.print('\t');
Serial.print(speed); Serial.print('\t');
Serial.print(temp);  Serial.print('\t');
Serial.print(amps, 3); Serial.print('\t');
Serial.println(moving ? "yes" : "no");
```

---

## Multi-Servo Bus Tips

1. **Unique IDs required** — two servos with the same ID will both respond, corrupting the bus.

2. **Read sequentially** — never read two servos at the same time on a shared bus:
   ```cpp
   st.readPresentPosition(pos);
   delay(2);
   sc.readPresentPosition(pos);
   ```

3. **Stagger return delays** — set different `returnDelayUnits` per servo in the Profile so replies don't collide:
   ```cpp
   FeetechST3020::Profile pSt;  pSt.returnDelayUnits = 0;
   FeetechSC15::Profile   pSc;  pSc.returnDelayUnits = 4;
   ```

4. **No broadcast writes** — ID 0xFE is blocked by the library.

---

## Troubleshooting

| Symptom | Likely cause |
|---------|-------------|
| `ping()` returns `false` | Wrong ID, wrong baud, no common GND, no power |
| Checksum errors | Missing 1 kΩ resistor, cable too long, baud too high |
| `setMode()` has no effect | EEPROM was locked — library now unlocks automatically |
| Servo jitters at target position | `setAcceleration()` too low, increase or set 0 |
| Two servos reply to one read | Duplicate IDs on bus |
| `setTargetVelocity()` ignored | Servo still in POSITION mode — call `setMode(VELOCITY)` first |

---

## Architecture

```
FeetechBus            protocol framing, checksum, UART I/O
  └── FeetechDevice   register map, endian + sign encoding, full API
        ├── FeetechST3020   STS family, 4096 ticks/rev, degToTicks()
        └── FeetechSC15     SCS family, 1024 ticks, big-endian
```

Bus knows only the wire protocol. Device knows registers. Subclass knows the model.

---

## Adding a New Model

```cpp
class FeetechXYZ : public FeetechDevice {
public:
  FeetechXYZ(FeetechBus& bus, uint8_t id) : FeetechDevice(bus, id) {
    // Override only what differs from the STS defaults:
    _reg.bigEndian               = false;
    _reg.signBit15               = true;
    _reg.ADDR_PRESENT_POSITION_L = 0x38;
  }
};
```

---

## Supported Platforms

- ESP32 (recommended)
- ESP32-S3, ESP32-C3
- AVR (use `beginPins()` — `begin1Wire()` is ESP32-only)

---

## License

MIT
