# FeetechBusServos

Arduino library for Feetech bus servos (SCS/STS protocol) on ESP32 and compatible boards.

Supports **ST3020 / ST3215** (STS, 360°, continuous rotation) and **SC15** (SCS, 180°, continuous rotation) on a shared half-duplex UART bus.

---

## Features

- Half-duplex UART — two GPIOs, no direction pin needed (TX → 1kΩ → bus, RX direct to bus)
- Typed device classes — no accidental cross-model register access
- Endian-correct 16-bit register I/O (STS = little-endian, SCS = big-endian)
- Sign-bit encoding per family (STS bit 15, SCS bit 10)
- Broadcast protection (ID 0xFE is blocked)
- Checksum validation on every response
- Synchronized multi-servo moves via `moveTimeAsync` + `triggerAction`
- Three operating modes: Position, Velocity (continuous rotation), Step (multi-turn)
- EEPROM auto-unlock for protected settings (ID, mode, calibration offset)
- `setMode()` reads current mode before writing — skips EEPROM write if already correct

---

## Hardware: Half-Duplex Wiring (ESP32)

```
ESP32 GPIO17 TX ---[1kΩ]---+--- SERVO BUS DATA
ESP32 GPIO18 RX ------------+
GND                --------- SERVO GND
5–8.4 V            --------- SERVO VCC
```

The **1 kΩ resistor** sits between the TX pin and the shared bus node.
The **RX pin** connects **directly** to the bus node (same junction, no resistor).

This is critical: if both RX and TX use the same pin or if RX is behind the resistor,
the servo's response will be blocked by the ESP32 TX output driving the bus HIGH.

Initialise with:
```cpp
bus.beginPins(1000000, 18, 17);  // beginPins(baud, rxPin, txPin)
```

---

## Quick Start — ST3020 / ST3215

```cpp
#include <FeetechBus.h>
#include <FeetechST3020.h>

FeetechBus    bus(Serial2);
FeetechST3020 st(bus, 1);    // servo ID (factory default = 1; use BusScan to find yours)

void setup() {
  bus.beginPins(1000000, 18, 17);  // 1 MBaud, RX=GPIO18, TX=GPIO17
  st.init();

  st.moveTime(st.degToTicks(90), 500, 800);  // go to 90°, 500 ms, speed 800
}

void loop() {}
```

## Quick Start — SC15

```cpp
#include <FeetechBus.h>
#include <FeetechSC15.h>

FeetechBus  bus(Serial2);
FeetechSC15 sc(bus, 1);    // servo ID (factory default = 1; use BusScan to find yours)

void setup() {
  bus.beginPins(1000000, 18, 17);  // 1 MBaud, RX=GPIO18, TX=GPIO17
  sc.init();

  sc.moveTime(sc.degToTicks(150), 500, 600);  // go to 150°, 500 ms, speed 600
}

void loop() {}
```

---

## Servo Model Comparison

| Class | Alias | Protocol | Encoder | Range | Speed range | `degToTicks` |
|-------|-------|----------|---------|-------|-------------|--------------|
| `FeetechST3020` | — | STS (little-endian, bit-15 sign) | 4096 ticks/rev | 0–360° | ±32767 | `degToTicks(0–360)` |
| `FeetechST3215` | = `FeetechST3020` | STS (identical register map) | 4096 ticks/rev | 0–360° | ±32767 | `degToTicks(0–360)` |
| `FeetechSC15` | — | SCS (big-endian, bit-10 sign) | 1024 ticks/180° | 0–180° | ±1023 | `degToTicks(0–180)` |

ST3020 and ST3215 share the same register map, protocol, and 12-bit encoder.
The differences are purely physical (form factor, torque, voltage range).

---

## API Reference

### Bus initialisation

| Method | Description |
|--------|-------------|
| `bus.beginPins(baud, rx, tx)` | Half-duplex: TX → 1kΩ → bus, RX → bus direct |

### Servo setup

| Method | Description |
|--------|-------------|
| `ping()` | Returns `true` if the servo responds on the bus |
| `init(Profile)` | Apply Profile settings (return delay, torque, status level) |

### Motion

| Method | Description |
|--------|-------------|
| `moveTime(pos, time, speed)` | Move to position; `time` in ms (0 = no limit), `speed` 0 = no cap |
| `moveTimeAsync(pos, time, speed)` | Queue move (REGWRITE); does not start until `triggerAction()` |
| `triggerAction()` | Broadcast ACTION — starts all queued async moves simultaneously |
| `setTargetVelocity(v)` | Velocity mode: signed speed, negative = reverse, 0 = stop |

### Position helpers

| Method | Class | Description |
|--------|-------|-------------|
| `degToTicks(deg)` | `FeetechST3020` | 0–360° → 0–4095 ticks |
| `degToTicks(deg)` | `FeetechSC15` | 0–180° → 0–1023 ticks; values outside range are clamped |

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

> These methods automatically unlock and relock EEPROM. Call them **once at setup**, not in `loop()`.
> EEPROM is rated for ~100 000 write cycles.
> `setMode()` reads the current mode before writing and skips the EEPROM write if the mode is already correct.

| Method | Families | Description |
|--------|----------|-------------|
| `setMode(ServoMode)` | ST + SC | Operating mode: POSITION, VELOCITY, or STEP |
| `setId(newId)` | ST + SC | Change servo ID (1–253) |
| `setAngleLimits(minTicks, maxTicks)` | ST + SC | Clamp the allowed position range in EEPROM |
| `setPositionOffset(offset)` | **ST only** | Signed calibration offset — not available on SC15 (returns `false`) |

#### `setAngleLimits` — position clamping

The servo will refuse to move outside the stored min/max range. Units are raw ticks.
Use `degToTicks()` to convert:

```cpp
// ST3020: restrict to 45°–270° (0–360° = 0–4095 ticks)
st.setAngleLimits(st.degToTicks(45), st.degToTicks(270));

// SC15: restrict to 30°–150° (0–180° = 0–1023 ticks)
sc.setAngleLimits(sc.degToTicks(30), sc.degToTicks(150));
```

Restore full range:
```cpp
st.setAngleLimits(0, 4095);   // ST3020
sc.setAngleLimits(0, 1023);   // SC15
```

#### `setPositionOffset` — calibration offset (STS / ST3020 only)

The SCS protocol (SC15) has **no calibration-offset register**. Calling `setPositionOffset()` on an SC15 returns `false` immediately without touching the bus.

The STS (ST3020 / ST3215) offset is a signed 16-bit value stored in EEPROM at register 0x1F.
It shifts the zero-point of the encoder without changing the physical range:

```cpp
st.setPositionOffset(100);    // shift zero-point by +100 ticks
st.setPositionOffset(0);      // reset to factory zero
```

---

## Operating Modes

```cpp
enum class ServoMode : uint8_t {
  POSITION = 0,  // Default: position control with moveTime()
  VELOCITY = 1,  // Continuous rotation with setTargetVelocity()
  STEP     = 3   // Multi-turn step mode (32-bit position counter)
};
```

Both ST3020 and SC15 support all three modes. The only difference is the speed range.

### Position Mode (default)

```cpp
// ST3020 — 360° range, 4096 ticks
st.moveTime(st.degToTicks(90), 300, 800);   // 90°, 300 ms, speed 800

// SC15 — 300° range, 1024 ticks
sc.moveTime(sc.degToTicks(90), 300, 600);   // 90°, 300 ms, speed 600
```

### Velocity Mode — continuous rotation

`setMode()` writes EEPROM (once in setup). `setTargetVelocity()` writes RAM (safe in loop).

```cpp
// ST3020 — speed range ±32767
st.setMode(ServoMode::VELOCITY);    // setup() — once
st.setTargetVelocity( 800);         // forward
st.setTargetVelocity(-800);         // reverse
st.setTargetVelocity(   0);         // stop

// SC15 — speed range ±1023
sc.setMode(ServoMode::VELOCITY);    // setup() — once
sc.setTargetVelocity( 600);         // forward
sc.setTargetVelocity(-600);         // reverse
sc.setTargetVelocity(   0);         // stop
```

Switch back to position mode:
```cpp
st.setMode(ServoMode::POSITION);
st.moveTime(st.degToTicks(0), 500, 500);
```

---

## Examples

| Example | Servo | What it shows |
|---------|-------|---------------|
| `BusScan` | any | Scan all IDs 1–253 at multiple baud rates — find connected servos and their IDs |
| `BasicPositionControl` | ST3020 | Move through several angles, wait for completion via `isMoving()`, read position and speed |
| `SpeedMove` | ST3020 | Snap to position at full hardware speed — `time=0, speed=0, acc=0` explained |
| `ContinuousRotation` | ST3020 | Velocity mode — forward / stop / reverse / stop with live speed readback |
| `SC15_PositionControl` | SC15 | Position control using `degToTicks()`, wait for completion, read back position |
| `SC15_ContinuousRotation` | SC15 | Velocity mode for SC15 — same API, speed range ±1023 |
| `StatusMonitor` | ST3020 | Live readout of position, speed, temperature, and current |
| `DualServo_SameUART` | ST3020 + SC15 | Both servo types on one bus — init, move, sequential position read |
| `SyncedMove` | ST3020 + SC15 | Two servos start simultaneously with `moveTimeAsync` + `triggerAction` |
| `RawDiag` | any | Raw hex dump of TX/RX bytes — diagnose wiring without any library abstraction |

---

### Example: BasicPositionControl (ST3020)

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

### Example: SC15_PositionControl

Moves an SC15 through 0° → 90° → 180° → 90° → 0°, blocking until each move finishes.
SC15 range is 0–180°; `degToTicks()` clamps values outside this range.

```cpp
FeetechSC15 sc(bus, 1);

void moveTo(float deg, uint16_t timeMs, uint16_t speed) {
  sc.moveTime(sc.degToTicks(deg), timeMs, speed);

  delay(60);
  bool moving = true;
  while (moving) { sc.isMoving(moving); delay(20); }

  uint16_t pos; int16_t spd;
  sc.readPresentPosition(pos);
  sc.readCurrentSpeed(spd);
  Serial.print("pos="); Serial.print(pos);
  Serial.print("  speed="); Serial.println(spd);
}

void loop() {
  moveTo(  0, 900, 600);
  moveTo( 90, 700, 700);
  moveTo(180, 900, 600);
}
```

---

### Example: ContinuousRotation (ST3020)

```cpp
void setup() {
  bus.beginPins(1000000, 18, 17);
  st.init();
  st.setMode(ServoMode::VELOCITY);  // EEPROM — once at setup
}

void loop() {
  st.setTargetVelocity( 800); delay(2000);  // forward
  st.setTargetVelocity(   0); delay(600);   // stop
  st.setTargetVelocity(-600); delay(2000);  // reverse
  st.setTargetVelocity(   0); delay(1000);  // stop
}
```

---

### Example: SC15_ContinuousRotation

Same API as ST3020 — only the speed range differs (±1023 instead of ±32767).

```cpp
void setup() {
  bus.beginPins(1000000, 18, 17);
  sc.init();
  sc.setMode(ServoMode::VELOCITY);  // EEPROM — once at setup
}

void loop() {
  sc.setTargetVelocity( 600); delay(2000);  // forward
  sc.setTargetVelocity(   0); delay(600);   // stop
  sc.setTargetVelocity(-400); delay(2000);  // reverse
  sc.setTargetVelocity(   0); delay(1000);  // stop
}
```

---

### Example: SyncedMove (ST3020 + SC15)

Both servo types on one bus, started simultaneously.

```cpp
// Queue on both servos — neither starts yet
st.moveTimeAsync(st.degToTicks(180), 900, 800);
sc.moveTimeAsync(sc.degToTicks(150), 900, 600);

// Fire: both start at exactly the same moment
st.triggerAction();
```

---

### Example: SpeedMove

Move to a position as fast as the hardware allows — no time constraint, no speed cap, no ramp.

```cpp
// setup() — once:
st.setAcceleration(0);  // 0 = no ramp; full torque from standstill

// Snap to any position at maximum hardware speed:
st.moveTime(st.degToTicks(180), 0, 0);
//                              ^  ^
//                        time=0  speed=0 → no limits
```

> **WARNING:** With `acc=0` and `speed=0` the servo moves hard and fast.
> Make sure the mechanical end-stops and the load can handle the impact.

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
| `ping()` returns `false` | Wrong ID, wrong baud, no common GND, no power, wrong wiring (see below) |
| RX sees nothing | RX pin is behind the 1kΩ instead of directly on the bus node |
| Checksum errors | Missing 1 kΩ resistor, cable too long, baud too high |
| `setMode()` has no effect | EEPROM was locked — library now unlocks automatically |
| Servo jitters at target position | `setAcceleration()` too low, increase or set 0 |
| Two servos reply to one read | Duplicate IDs on bus |
| `setTargetVelocity()` ignored | Servo still in POSITION mode — call `setMode(VELOCITY)` first |
| SC15 stops before target angle | Angle > 180° — SC15 servo mode range is 0–180°; `degToTicks()` clamps at 180° |
| SC15 stops before target angle | `setAngleLimits()` may have been called with a reduced range — check EEPROM limits |
| `setPositionOffset()` returns `false` on SC15 | Expected — the SCS register map has no offset register; use `setAngleLimits()` instead |

---

## Architecture

```
FeetechBus            protocol framing, checksum, UART I/O
  └── FeetechDevice   register map, endian + sign encoding, full API
        ├── FeetechST3020   STS family, 4096 ticks/rev, degToTicks(0–360°)
        │     FeetechST3215  (type alias — identical register map)
        └── FeetechSC15     SCS family, 1024 ticks/180°, degToTicks(0–180°)
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
- AVR (use `beginPins()` with hardware serial that supports pin remapping)

---

## License

MIT
