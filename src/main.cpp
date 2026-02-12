#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ========= WIFI =========
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// ========= MQTT =========
const char* mqtt_server = "broker.hivemq.com";
WiFiClient espClient;
PubSubClient client(espClient);

// ========= PINOS =========
#define NTC_PIN 34
#define POT_PIN 35
#define LED_PIN 26
#define BTN_PIN 27

// ========= VARIÁVEIS =========
float temperatura = 0;
float setpoint = 180;
bool fornoLigado = false;
bool resistencia = false;

float histerese = 2.0;

// ========= WIFI =========
void conectarWiFi() {
  Serial.println("Conectando ao WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi conectado!");
}

// ========= MQTT =========
void conectarMQTT() {
  while (!client.connected()) {
    Serial.println("Conectando ao MQTT...");
    client.connect("forno_talles_simples");
  }
  Serial.println("MQTT conectado!");
}

// ========= LEITURA TEMPERATURA =========
float lerTemperatura() {
  int valor = analogRead(NTC_PIN);
  float temp = map(valor, 0, 4095, 20, 300);
  return temp;
}

// ========= CONTROLE DO FORNO =========
void controlarForno() {

  if (!fornoLigado) {
    resistencia = false;
  } else {

    if (temperatura < setpoint - histerese) {
      resistencia = true;
    }

    if (temperatura > setpoint + histerese) {
      resistencia = false;
    }
  }

  digitalWrite(LED_PIN, resistencia);
}

// ========= TELEMETRIA MQTT =========
void enviarTelemetria() {

  StaticJsonDocument<200> doc;

  doc["temperatura"] = temperatura;
  doc["setpoint"] = setpoint;
  doc["forno"] = fornoLigado;
  doc["resistencia"] = resistencia;

  char jsonBuffer[200];
  serializeJson(doc, jsonBuffer);

  client.publish("talles/forno/telemetria", jsonBuffer);

  // ===== SERIAL COMPLETO =====
  Serial.println("------------ STATUS DO FORNO ------------");
  Serial.print("Temperatura Atual: ");
  Serial.print(temperatura);
  Serial.println(" °C");

  Serial.print("Setpoint: ");
  Serial.print(setpoint);
  Serial.println(" °C");

  Serial.print("Forno Ligado: ");
  Serial.println(fornoLigado ? "SIM" : "NAO");

  Serial.print("Resistencia Ativa: ");
  Serial.println(resistencia ? "SIM" : "NAO");

  Serial.print("JSON Enviado: ");
  Serial.println(jsonBuffer);

  Serial.println("------------------------------------------\n");
}

// ========= SETUP =========
void setup() {

  Serial.begin(115200);
  delay(1000);
  Serial.println("Iniciando Forno IoT Simplificado...");

  pinMode(LED_PIN, OUTPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);

  conectarWiFi();

  client.setServer(mqtt_server, 1883);
}

// ========= LOOP =========
void loop() {

  if (!client.connected()) conectarMQTT();
  client.loop();

  temperatura = lerTemperatura();

  setpoint = map(analogRead(POT_PIN), 0, 4095, 50, 250);

  // Botão toggle
  if (digitalRead(BTN_PIN) == LOW) {
    fornoLigado = !fornoLigado;
    Serial.println(">>> BOTAO PRESSIONADO - Alternando estado do forno");
    delay(500);
  }

  controlarForno();

  enviarTelemetria();

  delay(2000);
}