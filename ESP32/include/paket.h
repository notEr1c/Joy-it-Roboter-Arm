#pragma once
#include <stdint.h>

// Datenpaket für die serielle Übertragung zwischen den beiden Mikrocontrollern
// (ESP32-Gateway -> Servo-Treiber). Ein Paket ist immer genau 4 Bytes lang:
//
//   [ Start 0xE0 ] [ Motor ] [ Winkel ] [ Ende 0xEE ]
//
// Start- und Endbyte dienen dazu, den Anfang eines Pakets im Bytestrom wiederzufinden
// und unvollständige oder beschädigte Pakete zu erkennen.
const uint8_t PKT_START = 0xE0;   // Markiert den Anfang eines Pakets
const uint8_t PKT_END = 0xEE;     // Markiert das Ende eines Pakets

// "packed" verhindert Füllbytes: die Struktur ist genau 4 Bytes groß und kann
// direkt als Byte-Folge über die serielle Schnittstelle geschrieben werden.
struct __attribute__ ((packed)) Paket {
    uint8_t start;    // immer PKT_START
    uint8_t motor;    // Motornummer = Kanal am Servo-Treiber (0..5)
    uint8_t winkel;   // Soll-Servowinkel in Grad (0..180)
    uint8_t ende;     // immer PKT_END
};
