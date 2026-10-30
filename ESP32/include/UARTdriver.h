#pragma once

// Sendet ein Paket (Motornummer + Winkel) über die serielle Schnittstelle (Serial2)
// an den Servo-Treiber.
void sendePaket(uint8_t motor, uint8_t winkel);

// Initialisiert die serielle Schnittstelle Serial2. Wird einmal in setup() aufgerufen.
void UART_init();
