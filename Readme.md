\# FeetechBusServos



Feetech Bus Servo Library für Arduino \& ESP32  

Unterstützt:



\- ST3020

\- SC15

\- Gemeinsamer UART-Bus

\- ESP32 1-Wire Half-Duplex (RX = TX = gleicher GPIO)



---



\## ✨ Features



\- Gemeinsamer Bus für mehrere Feetech-Servos

\- Broadcast-Schutz (ID 0xFE wird blockiert)

\- Strikte Checksum-Validierung

\- 1-Wire UART für ESP32 (kein Direction-Pin nötig)

\- Typisierte Geräteklassen (ST3020, SC15)

\- Kollisionvermeidung über:

&nbsp; - Eindeutige IDs

&nbsp; - Return-Delay Staffelung

&nbsp; - Sequenzielles Read



---



\# 🔌 Hardware Setup (ESP32 – 1-Wire UART)



Diese Library unterstützt echten 1-Wire Half-Duplex Betrieb.



Das bedeutet:



\- Ein GPIO

\- RX und TX liegen auf demselben Pin

\- Beide Servos hängen an derselben Datenleitung



---



\## Empfohlene Verdrahtung



ESP32 GPIO17  --->  1kΩ  --->  SERVO BUS  

ESP32 GPIO17  <----------------- SERVO BUS  

GND           ------------------- GND  



Warum 1kΩ?



\- Entkoppelt TX vom Bus

\- Verhindert Signalverzerrung

\- Sehr empfohlen bei 1-Wire UART



In vielen Fällen funktioniert es auch ohne – aber mit Widerstand ist es robuster.



---



\# 🚀 Quick Start



```cpp

\#include <FeetechBus.h>

\#include <FeetechST3020.h>

\#include <FeetechSC15.h>



static constexpr int BUS\_PIN = 17;

static constexpr uint32\_t BAUD = 1000000;



FeetechBus bus(Serial2);



FeetechST3020 st(bus, 11);

FeetechSC15   sc(bus, 21);



void setup() {

&nbsp; Serial.begin(115200);



&nbsp; // RX = TX = BUS\_PIN

&nbsp; bus.begin1Wire(BAUD, BUS\_PIN);



&nbsp; st.init();

&nbsp; sc.init();



&nbsp; st.moveTime(st.degToTicks(90), 300, 800);

&nbsp; delay(400);



&nbsp; sc.moveTime(512, 300, 600);

}



void loop() {

}

```



---



\# ⚙️ Initialisierung



\## ESP32 1-Wire Modus



```cpp

bus.begin1Wire(1000000, 17);

```



Das mappt:



RX = GPIO17  

TX = GPIO17  



Kein Direction-Pin nötig.



---



\## Alternative: Separate Pins



```cpp

bus.beginPins(1000000, RX\_PIN, TX\_PIN);

```



---



\# 🧠 Konzept zur Bus-Sicherheit



Diese Library verhindert Fehlinterpretationen durch:



1\. Keine Broadcast-Pakete

2\. Feste ID pro Servo

3\. Typisierte Klassen (kein falscher Registerzugriff)

4\. Checksum-Validierung

5\. Optional gestaffelte ReturnDelay



---



\# 🔧 ST3020 API



```cpp

FeetechST3020 st(bus, 11);

```



\### Initialisieren



```cpp

FeetechST3020::Profile p;

p.returnDelayUnits = 0;

st.init(p);

```



\### Bewegung



```cpp

st.moveTime(positionTicks, time, speed);

```



\### Grad → Ticks



```cpp

uint16\_t ticks = st.degToTicks(90.0);

```



\### Position lesen



```cpp

uint16\_t pos;

st.readPresentPosition(pos);

```



---



\# 🔧 SC15 API



```cpp

FeetechSC15 sc(bus, 21);

```



\### Initialisieren



```cpp

FeetechSC15::Profile p;

p.returnDelayUnits = 3;

sc.init(p);

```



\### Bewegung



```cpp

sc.moveTime(512, 300, 600);

```



---



\# 📡 Register Mapping



Standard-Adressen (modellabhängig anpassbar):



| Funktion              | Adresse |

|-----------------------|----------|

| ID                    | 0x05 |

| Return Delay          | 0x07 |

| Status Return Level   | 0x08 |

| Torque Enable         | 0x28 |

| Goal Position (L)     | 0x2A |

| Goal Time (L)         | 0x2C |

| Goal Speed (L)        | 0x2E |

| Present Position (L)  | 0x38 |



Falls dein Servo abweicht:



```cpp

FeetechDevice::RegMap map;

map.ADDR\_PRESENT\_POSITION\_L = 0x36;



FeetechST3020 st(bus, 11, map);

```



---



\# 🛑 Wichtige Regeln für stabilen Betrieb



1️⃣ IDs müssen verschieden sein  

Wenn zwei Servos die gleiche ID haben, reagieren beide.



2️⃣ Kein Broadcast im Betrieb  

ID 0xFE ist gesperrt.



3️⃣ Nicht SYNC READ verwenden  

Stattdessen:



```cpp

st.readPresentPosition(pos);

delay(2);

sc.readPresentPosition(pos);

```



---



\# 🧪 Troubleshooting



\### Servo antwortet nicht



\- Baudrate prüfen (oft 1’000’000)

\- ID prüfen

\- 1kΩ Widerstand einsetzen

\- Gemeinsame Masse sicherstellen



\### Zufällige Checksum-Fehler



\- Widerstand fehlt

\- Leitung zu lang

\- Baudrate zu hoch



---



\# 🏗 Architektur



FeetechBus  

&nbsp;   ↓  

FeetechDevice  

&nbsp;   ↓  

FeetechST3020 / FeetechSC15  



Bus kennt nur Protokoll.  

Device kennt Registermap.  

Typklasse kapselt Modell.



---



\# 📦 Unterstützte Plattformen



\- ESP32 (empfohlen)

\- ESP32-S3

\- ESP32-C3

\- AVR (ohne 1-Wire Mapping)



---



\# 🔮 Erweiterbar



Du kannst weitere Modelle hinzufügen durch:



```cpp

class FeetechXYZ : public FeetechDevice

```



Nur Registermap anpassen.



---



\# 📄 Lizenz



MIT

