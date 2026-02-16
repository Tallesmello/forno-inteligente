#include "mqtt_con.h"
#include <Arduino.h>

extern double temp;
extern int med_sen_gas;
extern int med_sen_luz;

// Broker público
const char* MQTT_BROKER    = "broker.hivemq.com";
const int   MQTT_PORT      = 1883;
const char* MQTT_PUB_TOPIC = "fornoiot/sensores";


WiFiClient espClient;
PubSubClient mqtt(espClient);

unsigned long lastMqtt = 0;
unsigned long lastLcd  = 0;

void connectMQTT() {

  mqtt.setServer(MQTT_BROKER, MQTT_PORT);

  while (!mqtt.connected()) {

    Serial.print("[MQTT] Conectando... ");

    String clientId = "FornoIoT-";
    clientId += String(random(1000, 9999));

    if (mqtt.connect(clientId.c_str())) {
      Serial.println("conectado!");
    } else {
      Serial.print("falhou, rc=");
      Serial.print(mqtt.state());
      Serial.println(" tentando em 2s...");
      delay(2000);
    }
  }
}

void publishSensors() {

  if (!mqtt.connected()) return;

  char payload[256];

  snprintf(payload, sizeof(payload),
    "{"
      "\"temperatura\": %.2f,"
      "\"gas\": \"%s\","
      "\"luz\": \"%s\""
    "}",
    temp,
    med_sen_gas ? "detectado" : "normal",
    med_sen_luz ? "ligada" : "desligada"
  );

  mqtt.publish(MQTT_PUB_TOPIC, payload);

  Serial.print("[MQTT] Publicado: ");
  Serial.println(payload);
}
