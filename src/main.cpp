#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ========= CONFIGURAÇÕES DE REDE E BROKER =========
const char* ssid = "Wokwi-GUEST";            // Nome da rede WiFi
const char* password = "";                   // Senha da rede WiFi
const char* mqtt_server = "broker.hivemq.com"; // Endereço do Broker MQTT gratuito

WiFiClient espClient;                        // Cliente WiFi do ESP32
PubSubClient client(espClient);              // Cliente MQTT que utiliza o cliente WiFi

// ========= DEFINIÇÃO DOS PINOS (HARDWARE) =========
#define NTC_PIN 34  // Sensor de Temperatura (Analógico)
#define POT_PIN 35  // Potenciômetro para ajustar o Setpoint (Analógico)
#define LED_PIN 26  // LED que representa a Resistência de aquecimento
#define BTN_PIN 27  // Botão de pressão para ligar/desligar o sistema

// ========= VARIÁVEIS DE CONTROLE DO SISTEMA =========
float temperatura = 0;       // Armazena a leitura atual do sensor
float setpoint = 180;        // Temperatura alvo desejada
bool fornoLigado = false;    // Estado geral do forno (ligado/desligado pelo botão)
bool resistencia = false;    // Estado da resistência (ligada/desligada pelo termostato)
float histerese = 2.0;       // Margem de erro para evitar que a resistência ligue/desligue freneticamente

// ========= VARIÁVEIS DE TEMPO E ESTADO =========
unsigned long ultimoEnvio = 0;      // Armazena o tempo do último envio MQTT
const long intervalo = 2000;        // Tempo de espera entre envios (2 segundos)
bool estadoAnteriorBotao = HIGH;    // Armazena o estado anterior do botão para detectar o clique

// ========= FUNÇÃO: CONEXÃO WIFI =========
void conectarWiFi() {
  Serial.print("Conectando ao WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); 
    Serial.print("."); // Feedback visual enquanto tenta conectar
  }
  Serial.println("\nWiFi conectado!");
}

// ========= FUNÇÃO: CONEXÃO MQTT =========
void conectarMQTT() {
  // Loop que tenta conectar ao Broker até conseguir
  while (!client.connected()) {
    Serial.println("Conectando ao MQTT...");
    // Tenta conectar com um ID de cliente único
    if (client.connect("forno_talles_simples")) {
      Serial.println("MQTT conectado!");
    } else {
      delay(2000); // Espera 2 segundos antes de tentar novamente
    }
  }
}

// ========= FUNÇÃO: LEITURA DOS SENSORES =========
float lerTemperatura() {
  // Lê o valor analógico (0-4095) e mapeia para a faixa de 20°C a 300°C
  return map(analogRead(NTC_PIN), 0, 4095, 20, 300);
}

// ========= FUNÇÃO: LÓGICA DO TERMOSTATO =========
void controlarForno() {
  // Se o botão geral estiver desligado, a resistência deve estar desligada sempre
  if (!fornoLigado) {
    resistencia = false;
  } else {
    // Lógica com Histerese: 
    // Liga se estiver abaixo do setpoint (com margem)
    if (temperatura < setpoint - histerese) resistencia = true;
    // Desliga se estiver acima do setpoint (com margem)
    if (temperatura > setpoint + histerese) resistencia = false;
  }
  // Ativa ou desativa o pino físico do LED/Resistência
  digitalWrite(LED_PIN, resistencia);
}

// ========= FUNÇÃO: ENVIO DE DADOS (JSON) =========
void enviarTelemetria() {
  StaticJsonDocument<200> doc;
  
  // Monta o objeto JSON com os dados atuais
  doc["temperatura"] = (int)temperatura;
  doc["setpoint"] = (int)setpoint;
  doc["forno"] = fornoLigado;
  doc["resistencia"] = resistencia;

  char jsonBuffer[200];
  serializeJson(doc, jsonBuffer); // Converte o objeto para texto (string)
  
  // Publica no tópico de telemetria
  client.publish("talles/forno/telemetria", jsonBuffer);

  // Exibe o status formatado no Monitor Serial para depuração
  Serial.println("------------ STATUS DO FORNO ------------");
  Serial.print("Temperatura Atual: "); Serial.print(temperatura); Serial.println(" °C");
  Serial.print("Setpoint: ");          Serial.print(setpoint);    Serial.println(" °C");
  Serial.print("Forno Ligado: ");      Serial.println(fornoLigado ? "SIM" : "NAO");
  Serial.print("Resistencia Ativa: "); Serial.println(resistencia ? "SIM" : "NAO");
  Serial.print("JSON Enviado: ");      Serial.println(jsonBuffer);
  Serial.println("------------------------------------------\n");
}

// ========= SETUP: CONFIGURAÇÃO INICIAL =========
void setup() {
  Serial.begin(115200);
  
  pinMode(LED_PIN, OUTPUT);
  // INPUT_PULLUP ativa o resistor interno. O botão deve ligar o pino ao GND.
  pinMode(BTN_PIN, INPUT_PULLUP); 

  conectarWiFi();
  client.setServer(mqtt_server, 1883); // Configura o servidor e a porta do MQTT
}

// ========= LOOP: EXECUÇÃO CONTÍNUA =========
void loop() {
  // Garante que o MQTT esteja sempre conectado
  if (!client.connected()) conectarMQTT();
  client.loop(); // Mantém a comunicação interna com o broker ativa

  // --- 1. DETECÇÃO DE CLIQUE DO BOTÃO ---
  bool estadoAtualBotao = digitalRead(BTN_PIN);
  
  // Só entra aqui se o botão for apertado AGORA (evita repetir a ação se segurar o botão)
  if (estadoAtualBotao == LOW && estadoAnteriorBotao == HIGH) {
    fornoLigado = !fornoLigado; // Inverte o estado (liga se estava desligado e vice-versa)
    
    if (fornoLigado) {
      Serial.println("SISTEMA LIGADO");
      client.publish("talles/forno/status", "LIGADO");
    } else {
      Serial.println("SISTEMA DESLIGADO");
      client.publish("talles/forno/status", "DESLIGADO");
      // Envia uma última telemetria para atualizar o Dashboard com o estado "desligado"
      enviarTelemetria(); 
    }
    delay(150); // Debounce: pequeno tempo para ignorar ruído elétrico do botão
  }
  estadoAnteriorBotao = estadoAtualBotao;

  // --- 2. ATUALIZAÇÃO DOS SENSORES E CONTROLE ---
  temperatura = lerTemperatura();
  // Ajusta o setpoint via potenciômetro (faixa de 50°C a 250°C)
  setpoint = map(analogRead(POT_PIN), 0, 4095, 50, 250);
  controlarForno();

  // --- 3. ENVIO TEMPORIZADO (ESTILO MULTITAREFA) ---
  unsigned long tempoAtual = millis();
  // Só envia dados repetitivos se o forno estiver LIGADO e o tempo de intervalo passou
  if (fornoLigado && (tempoAtual - ultimoEnvio >= intervalo)) {
    ultimoEnvio = tempoAtual;
    enviarTelemetria();
  }
}