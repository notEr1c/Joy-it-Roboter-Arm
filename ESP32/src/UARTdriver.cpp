#include <Arduino.h>
#include "UARTdriver.h"
#include "paket.h"


// Pins der zweiten seriellen Schnittstelle (Serial2) des ESP32
static const int RX2_PIN = 16;   // Empfang (wird hier nicht benutzt)
static const int TX2_PIN = 17;   // Senden: Verbindung zum Servo-Treiber

// Baut ein Paket [Start | Motor | Winkel | Ende] und schreibt die 4 Bytes auf Serial2.
void sendePaket (uint8_t motor, uint8_t winkel) {
    Paket p = { PKT_START, motor, winkel, PKT_END};
    Serial2.write((uint8_t*)&p, sizeof(p));   // die Struktur als Bytefolge senden
    // Kontrollausgabe auf dem USB-Serial-Monitor (Bytes hexadezimal)
    Serial.printf("Gesendet: %02X %02X %02X %02X\n", p.start, p.motor, p.winkel, p.ende);
}

// 9600 Baud, 8 Datenbits, keine Parität, 1 Stoppbit (SERIAL_8N1).
// Der letzte Parameter (true) invertiert das Signal: Pegel High/Low sind vertauscht.
void UART_init() {
    Serial2.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN, true);
}
