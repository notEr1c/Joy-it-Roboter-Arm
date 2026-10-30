// ESP32-Gateway: nimmt MQTT-Befehle aus dem WLAN entgegen und gibt sie als serielle
// Pakete an den Servo-Treiber weiter (siehe main_1.cpp).
//
//   PC-App --MQTT--> Broker --MQTT--> [ESP32] --UART (paket.h)--> Servo-Treiber --PWM--> Servos
#include <Arduino.h>
#include "paket.h"
#include <WiFi.h>
#include "UARTdriver.h"
#include "MQTTdriver.h"
#include <PubSubClient.h>

const long aliveDelay = 15000;     // Abstand der Lebenszeichen in Millisekunden
unsigned long lastAlive = 0;       // Zeitpunkt des letzten Lebenszeichens

void setup() {
    Serial.begin(115200);          // USB-Monitor für Kontrollausgaben
    UART_init();                   // serielle Schnittstelle zum Servo-Treiber

    connectWiFi();

    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);   // diese Funktion verarbeitet eingehende Nachrichten
    mqttClient.setBufferSize(256);          // maximale Nachrichtengröße in Bytes
    mqttClient.setKeepAlive(60);            // Verbindung gilt nach 60 s ohne Signal als abgebrochen

    connectMQTT();
}

void loop() {
    // Verbindungen bei Bedarf wiederherstellen
    if (WiFi.status() != WL_CONNECTED) {
        connectWiFi();
    }

    if (!mqttClient.connected()) {
        connectMQTT();
    }

    mqttClient.loop();   // verarbeitet eingehende Nachrichten (ruft mqttCallback auf)

    delay(2);

    // Alle 15 Sekunden ein Lebenszeichen senden
    if (millis() - lastAlive >= aliveDelay) {
        lastAlive = millis();
        publishStatus(1);
    }
}
