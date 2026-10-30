#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "MQTTdriver.h"
#include "UARTdriver.h"

// --- Verbindungsdaten ---
const char* WIFI_SSID = "SmartFactoryLab";
const char* WIFI_PASSWORD = "smartfactorylab";

const char* MQTT_HOST = "10.20.3.82";
const uint16_t MQTT_PORT = 1883;

// Topics: Befehle kommen auf Robo/Command an, das Lebenszeichen geht auf Robo/AliveStatus raus
const char* MQTT_TOPIC_COMMAND = "Robo/Command";
const char* MQTT_TOPIC_STATUS  = "Robo/AliveStatus";

// Zuletzt empfangene Werte (global, damit sie auch außerhalb der Callback-Funktion bekannt sind)
uint8_t winkel;
uint8_t motor;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Verbindet mit dem WLAN. Blockiert, bis die Verbindung steht (alle 500 ms ein Punkt im Monitor).
void connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        return;   // schon verbunden
    }

    WiFi.mode(WIFI_STA);                       // Station-Modus: ESP32 verbindet sich mit einem Router
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("Verbinde mit WLAN");

    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }

    Serial.println("\r\nWLAN verbunden");
}

// Sendet ein Lebenszeichen, damit man am Broker sieht, dass der ESP32 läuft.
void publishStatus(uint8_t mode) {
    if (!mqttClient.connected()) {
        return;   // ohne Verbindung nichts senden
    }
    if (mode == 1) {
        mqttClient.publish(MQTT_TOPIC_STATUS, "Ich lebe noch!");
    }
    else if (mode == 2) {
        mqttClient.publish(MQTT_TOPIC_STATUS, "Ich bin wieder da!");
    }
}

// Wird bei jeder eingehenden Nachricht aufgerufen. Erwartet den Text "<Motor> <Winkel>",
// z. B. "3 175". Die Nachricht wird geprüft, in zwei Zahlen zerlegt und als Paket über
// die serielle Schnittstelle an den Servo-Treiber weitergegeben.
// (Das Topic wird nicht geprüft: abonniert ist nur das Befehls-Topic.)
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    char buf[16];
    if (length >= sizeof(buf)) return;   // zu lang, verwerfen
    memcpy(buf, payload, length);        // Payload in einen Textpuffer kopieren ...
    buf[length] = '\0';                  // ... und mit Nullbyte als C-String abschließen

    unsigned int a, b;
    // sscanf liefert die Anzahl erfolgreich gelesener Zahlen. Beide müssen in ein Byte (0..255) passen.
    if (sscanf(buf, "%u %u", &a, &b) == 2 && a <= 255 && b <= 255) {
        motor = (uint8_t)a;
        winkel = (uint8_t)b;
        Serial.printf("Motor: %u Winkel: %u\n", motor, winkel);
        sendePaket(motor, winkel);       // weiter an den Servo-Treiber
    } else {
        Serial.printf("Ungültige Nachricht: %s\n", buf);
    }

}


// Verbindet mit dem MQTT-Broker. Versucht es alle 3 Sekunden, bis es klappt.
void connectMQTT() {
    while (!mqttClient.connected()) {
        Serial.print("Verbinde mit MQTT ... ");

        // Eindeutige Client-ID aus der MAC-Adresse des ESP32 ("ESP32-AABBCCDDEEFF")
        String clientId = "ESP32-";
        clientId += WiFi.macAddress();
        clientId.replace(":", "");

        if (mqttClient.connect(clientId.c_str())) {
            Serial.println("verbunden");

            mqttClient.subscribe(MQTT_TOPIC_COMMAND);   // ab jetzt kommen Befehle in mqttCallback an

            Serial.println("MQTT-Topic abonniert");

            publishStatus(2);   // meldet "Ich bin wieder da!"
        }
        else {
            // Fehlercode der Bibliothek ausgeben (z. B. -2 = Broker nicht erreichbar)
            Serial.print("Fehler: ");
            Serial.println(mqttClient.state());

            delay(3000);
        }
    }
}
