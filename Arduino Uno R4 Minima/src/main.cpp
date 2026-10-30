// Servo-Treiber: empfängt Pakete (paket.h) vom ESP32-Gateway über die serielle
// Schnittstelle und stellt die Servos über einen PWM-Servotreiber (PCA9685, I2C) ein.
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "paket.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Pulslängen in "Ticks". Der PCA9685 teilt eine Periode (20 ms bei 50 Hz) in 4096 Ticks.
//   SERVOMIN = 100 Ticks ≈ 0,49 ms  -> Servowinkel 0 Grad
//   SERVOMAX = 500 Ticks ≈ 2,44 ms  -> Servowinkel 180 Grad
const uint16_t SERVOMAX = 500;
const uint16_t SERVOMIN = 100;
// Pulslänge für 90 Grad (Servomitte), hier 300 Ticks
const uint16_t mittelStellung = map(90, 0, 180, SERVOMIN, SERVOMAX);
uint8_t servo;    //Servos von unten nach oben nummerriert
uint8_t winkel;   // (servo und winkel werden im Programm nicht benutzt)

void setup() {
    Serial.begin(115200);    // USB-Monitor für Kontrollausgaben
    Serial1.begin(9600);     // Empfang der Pakete vom ESP32-Gateway
    pwm.begin();
    pwm.setPWMFreq(50);      // Servos erwarten 50 Pulse pro Sekunde
    delay(10);
    // Beim Start alle 6 Servos auf Mittelstellung (90 Grad) fahren
    for (int i = 0; i < 6; i++) {
        pwm.setPWM(i, 0, mittelStellung);
    }
}

// Liest ein Paket aus dem Empfangspuffer. Gibt true zurück, wenn ein vollständiges,
// gültiges Paket in p steht, sonst false.
bool lesePaket(Paket &p) {
    while (Serial1.available() >= 1) {
        // Bytes verwerfen, bis ein Startbyte kommt (so findet man nach Störungen wieder den Paketanfang)
        if (Serial1.peek() != PKT_START) {
            Serial1.read();
            continue;
        }
        // Startbyte gefunden, aber das Paket ist noch nicht vollständig angekommen: später weiter
        if (Serial1.available() < (int)sizeof(Paket)) {
            return false;
        }

        Serial1.readBytes((uint8_t*)&p, sizeof(p));   // alle 4 Bytes auf einmal lesen
        if (p.ende == PKT_END) {
            return true;      // Endbyte stimmt: Paket ist gültig
        }
        // Endbyte falsch: Paket verwerfen und weitersuchen
    }
    return false;
}

void loop() {
    Paket p;
    if (lesePaket(p)) {
        // Kontrollausgabe auf dem USB-Monitor
        Serial.print("Motor: ");
        Serial.println(p.motor);
        Serial.print("Winkel: ");
        Serial.println(p.winkel);
        // Winkel (0..180 Grad) in eine Pulslänge umrechnen und am Kanal "motor" ausgeben
        pwm.setPWM(p.motor, 0, map(p.winkel, 0, 180, SERVOMIN, SERVOMAX));
    }
}
