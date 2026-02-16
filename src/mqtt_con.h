#ifndef MQTT_CON_H
#define MQTT_CON_H

#include <WiFi.h>
#include <PubSubClient.h>

extern WiFiClient espClient;
extern PubSubClient mqtt;

extern unsigned long lastMqtt;
extern unsigned long lastLcd;

void connectMQTT();
void publishSensors();

#endif
