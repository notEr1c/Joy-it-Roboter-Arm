#pragma once
#include <WiFi.h>
#include <PubSubClient.h>

// Die Werte sind in MQTTdriver.cpp definiert ("extern" = hier nur bekannt gemacht).

// WLAN in Wokwi
extern const char* WIFI_SSID;       // Name des WLANs
extern const char* WIFI_PASSWORD;   // Passwort des WLANs

// MQTT-Broker
extern const char* MQTT_HOST;       // IP-Adresse des Brokers
extern const uint16_t MQTT_PORT;    // Port des Brokers (1883 = Standard ohne Verschlüsselung)

// MQTT-Topics
extern const char* MQTT_TOPIC_COMMAND;   // Befehle an den Arm: "<Motor> <Winkel>"
extern const char* MQTT_TOPIC_STATUS;    // Lebenszeichen des ESP32

extern WiFiClient wifiClient;        // TCP-Verbindung über das WLAN
extern PubSubClient mqttClient;      // MQTT-Client, benutzt wifiClient

// Verbindet mit dem WLAN und wartet, bis die Verbindung steht.
void connectWiFi();

// Veröffentlicht ein Lebenszeichen auf MQTT_TOPIC_STATUS.
// mode 1: "Ich lebe noch!" (regelmäßig), mode 2: "Ich bin wieder da!" (nach Verbindungsaufbau)
void publishStatus(uint8_t mode);

// Wird von der MQTT-Bibliothek aufgerufen, wenn eine Nachricht eintrifft.
void mqttCallback(char* topic, byte* payload, unsigned int length);

// Verbindet mit dem MQTT-Broker und abonniert das Befehls-Topic.
void connectMQTT();
