// RawDiag.ino
//
// Diagnose-Sketch: sendet rohe Feetech-Ping-Pakete OHNE Library
// und gibt alle empfangenen Bytes als HEX aus.
//
// Damit kann man unabhängig von der Library prüfen:
//  (a) Ob der ESP32 überhaupt sendet
//  (b) Ob der Servo antwortet
//  (c) Ob Echo-Bytes sichtbar sind
//
// WICHTIG: Servoverbindung prüfen
//   Variante A – 1-Wire (RX=TX selber Pin):
//     GPIO17 --[1kΩ]--> Servo DATA
//     GPIO17 <--------- Servo DATA
//     (beide Signale auf demselben Pin, 1kΩ schützt bei Bus-Konflikt)
//
//   Variante B – separate RX/TX (Standard-Halbduplex):
//     GPIO17 TX --[1kΩ]--> Servo DATA --> GPIO16 RX
//     (TX durch Widerstand, RX direkt am Bus)
//
// Setze WIRING_MODE unten auf A oder B.

#include <Arduino.h>

// -----------------------------------------------------------------------
// Konfiguration
// -----------------------------------------------------------------------
#define WIRING_MODE 'A'   // 'A' = 1-Wire, 'B' = separate RX/TX

static constexpr int      TX_PIN  = 17;
static constexpr int      RX_PIN  = 16;   // nur für Variante B relevant
static constexpr uint32_t BAUD    = 1000000;

// Ping-Paket für IDs 1..5 (Feetech Factory-Default = ID 1)
// Format: FF FF ID 02 01 CHK    CHK = ~(ID+02+01) & 0xFF
// -----------------------------------------------------------------------

static void sendPing(HardwareSerial& ser, uint8_t id) {
  uint8_t chk = (uint8_t)(~((uint16_t)id + 0x02 + 0x01) & 0xFF);
  uint8_t pkt[6] = { 0xFF, 0xFF, id, 0x02, 0x01, chk };
  ser.write(pkt, 6);
  ser.flush();
}

static void printHex(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
  Serial.print(' ');
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Feetech Raw Diagnose ===");

#if WIRING_MODE == 'A'
  Serial.print("Variante A: 1-Wire, GPIO"); Serial.println(TX_PIN);
  Serial2.begin(BAUD, SERIAL_8N1, TX_PIN, TX_PIN);
#else
  Serial.print("Variante B: RX=GPIO"); Serial.print(RX_PIN);
  Serial.print("  TX=GPIO"); Serial.println(TX_PIN);
  Serial2.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
#endif

  delay(100);
  while (Serial2.available()) Serial2.read(); // flush

  Serial.println("Sende PING an IDs 1..5 (je 3x), zeige alle RX-Bytes:");
  Serial.println("Erwartete Antwort: FF FF <ID> 02 00 <CHK>");
  Serial.println();

  for (uint8_t id = 1; id <= 5; id++) {
    Serial.print("--- ID ");
    Serial.print(id);
    Serial.println(" ---");

    for (int attempt = 0; attempt < 3; attempt++) {
      // Kurze Pause zwischen Versuchen
      delay(10);
      while (Serial2.available()) Serial2.read(); // flush vorher

      Serial.print("TX: FF FF ");
      printHex(id);
      Serial.print("02 01 ");
      uint8_t chk = (uint8_t)(~((uint16_t)id + 0x02 + 0x01) & 0xFF);
      printHex(chk);
      Serial.println();

      sendPing(Serial2, id);

      // Warte 30ms und sammle alles was kommt
      delay(30);
      Serial.print("RX: ");
      int rxCount = 0;
      while (Serial2.available()) {
        printHex((uint8_t)Serial2.read());
        rxCount++;
      }
      if (rxCount == 0) {
        Serial.print("(nichts)");
      }
      Serial.println();
    }
    Serial.println();
  }

  Serial.println("=== Fertig. Ausgabe interpretieren: ===");
  Serial.println("  Nur Echo (= TX-Bytes zurueck): Bus-Verbindung OK, Servo antwortet nicht");
  Serial.println("  Nichts:  Kein Echo -> Verdrahtung pruefen (Pin, Spannung)");
  Serial.println("  FF FF xx 02 00 yy nach Echo: Servo antwortet -> Library-Problem");
}

void loop() {}
