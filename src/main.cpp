#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>

// ========= CONFIGURAÇÕES DE REDE E BROKER =========
const char* ssid = "Wokwi-GUEST";            // Nome da rede WiFi
const char* password = "";                   // Senha da rede WiFi
const char* mqtt_server = "broker.hivemq.com"; // Endereço do Broker MQTT gratuito

WiFiClient espClient;                        // Cliente WiFi do ESP32
PubSubClient client(espClient);              // Cliente MQTT que utiliza o cliente WiFi

// ========= CONFIGURAÇÕES HTTP =========
const char* ubidotsToken = "BBUS-X7z3rOgZQmfO7DQwb4mP7FHSzBNFR6";
const char* ubidotsDevice = "forno_talles";
String thingSpeakApiKey = "8X2M83ELJL3JDA4E"; 

// ========= DEFINIÇÃO DOS PINOS (HARDWARE) =========
#define NTC_PIN 34  // Sensor de Temperatura (Analógico)
#define POT_PIN 35  // Potenciômetro para ajustar o Setpoint (Analógico)
#define LED_PIN 26  // LED que representa a Resistência de aquecimento
#define BTN_PIN 27  // Botão de pressão para ligar/desligar o sistema

// ========= VARIÁVEIS DE CONTROLE DO SISTEMA =========
float temperatura = 0;       
float setpoint = 180;        
bool fornoLigado = false;    
bool resistencia = false;    
float histerese = 2.0;       

// ========= VARIÁVEIS DE TEMPO E ESTADO =========
unsigned long ultimoEnvio = 0;      
const long intervalo = 2000;        
bool estadoAnteriorBotao = HIGH;    

// ========= PROTÓTIPOS DAS FUNÇÕES HTTP =========
void enviarUbidotsHTTP(int temp, int sp, bool forno, bool res);
void enviarThingSpeakHTTP(int temp, int sp, bool forno, bool res);

// ========= FUNÇÃO: CONEXÃO WIFI =========
void conectarWiFi() {
  Serial.print("Conectando ao WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); 
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");
}

// ========= FUNÇÃO: CONEXÃO MQTT =========
void conectarMQTT() {
  while (!client.connected()) {
    Serial.println("Conectando ao MQTT...");
    if (client.connect("forno_talles_simples")) {
      Serial.println("MQTT conectado!");
    } else {
      delay(2000);
    }
  }
}

// ========= FUNÇÃO: LEITURA DOS SENSORES =========
float lerTemperatura() {
  return map(analogRead(NTC_PIN), 0, 4095, 20, 300);
}

// ========= FUNÇÃO: LÓGICA DO TERMOSTATO =========
void controlarForno() {
  if (!fornoLigado) {
    resistencia = false;
  } else {
    if (temperatura < setpoint - histerese) resistencia = true;
    if (temperatura > setpoint + histerese) resistencia = false;
  }
  digitalWrite(LED_PIN, resistencia);
}

// ========= FUNÇÃO: ENVIO DE DADOS (JSON) =========
void enviarTelemetria() {
  StaticJsonDocument<200> doc;
  
  doc["temperatura"] = (int)temperatura;
  doc["setpoint"] = (int)setpoint;
  doc["forno"] = fornoLigado;
  doc["resistencia"] = resistencia;

  char jsonBuffer[200];
  serializeJson(doc, jsonBuffer);
  
  client.publish("talles/forno/telemetria", jsonBuffer);

  Serial.println("------------ STATUS DO FORNO ------------");
  Serial.print("Temperatura Atual: "); Serial.print(temperatura); Serial.println(" °C");
  Serial.print("Setpoint: ");          Serial.print(setpoint);    Serial.println(" °C");
  Serial.print("Forno Ligado: ");      Serial.println(fornoLigado ? "SIM" : "NAO");
  Serial.print("Resistencia Ativa: "); Serial.println(resistencia ? "SIM" : "NAO");
  Serial.print("JSON Enviado: ");      Serial.println(jsonBuffer);
  Serial.println("------------------------------------------\n");

  enviarUbidotsHTTP((int)temperatura, (int)setpoint, fornoLigado, resistencia);
  enviarThingSpeakHTTP((int)temperatura, (int)setpoint, fornoLigado, resistencia);
}

// ========= FUNÇÃO: ENVIO UBIDOTS VIA HTTP =========
void enviarUbidotsHTTP(int temp, int sp, bool forno, bool res) {

  if (WiFi.status() == WL_CONNECTED) {

    HTTPClient http;
    String url = "http://industrial.api.ubidots.com/api/v1.6/devices/";
    url += ubidotsDevice;

    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-Auth-Token", ubidotsToken);

    StaticJsonDocument<200> doc;
    doc["temperatura"] = temp;
    doc["setpoint"] = sp;
    doc["forno"] = forno;
    doc["resistencia"] = res;

    String json;
    serializeJson(doc, json);

    int httpCode = http.POST(json);

    Serial.print("HTTP Ubidots Code: ");
    Serial.println(httpCode);

    http.end();
  }
}

// ========= FUNÇÃO: ENVIO THINGSPEAK VIA HTTP =========
void enviarThingSpeakHTTP(int temp, int sp, bool forno, bool res) {

  if (WiFi.status() == WL_CONNECTED) {

    HTTPClient http;

    String url = "http://api.thingspeak.com/update?";
    url += "api_key=" + thingSpeakApiKey;
    url += "&field1=" + String(temp);
    url += "&field2=" + String(sp);
    url += "&field3=" + String(forno);
    url += "&field4=" + String(res);

    http.begin(url);

    int httpCode = http.GET();

    Serial.print("HTTP ThingSpeak Code: ");
    Serial.println(httpCode);

    http.end();
  }
}

// ========= SETUP =========
void setup() {
  Serial.begin(115200);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(BTN_PIN, INPUT_PULLUP); 

  conectarWiFi();
  client.setServer(mqtt_server, 1883);
}

// ========= LOOP =========
void loop() {
  if (!client.connected()) conectarMQTT();
  client.loop();

  bool estadoAtualBotao = digitalRead(BTN_PIN);
  
  if (estadoAtualBotao == LOW && estadoAnteriorBotao == HIGH) {
    fornoLigado = !fornoLigado;
    
    if (fornoLigado) {
      Serial.println("SISTEMA LIGADO");
      client.publish("talles/forno/status", "LIGADO");
    } else {
      Serial.println("SISTEMA DESLIGADO");
      client.publish("talles/forno/status", "DESLIGADO");
      enviarTelemetria(); 
    }
    delay(150);
  }
  estadoAnteriorBotao = estadoAtualBotao;

  temperatura = lerTemperatura();
  setpoint = map(analogRead(POT_PIN), 0, 4095, 50, 250);
  controlarForno();

  unsigned long tempoAtual = millis();
  if (fornoLigado && (tempoAtual - ultimoEnvio >= intervalo)) {
    ultimoEnvio = tempoAtual;
    enviarTelemetria();
  }
}
