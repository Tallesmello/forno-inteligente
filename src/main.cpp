#include <Arduino.h>
#include <LiquidCrystal.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>

#include "wifi_con.h"
#include "mqtt_con.h"

//-------------------------------------------------
// Definição dos pinos
//-------------------------------------------------
#define ANG_TMP     11
#define DIG_LDR     12
#define DIG_GAS     13

#define SIN_DE_GAS  2
#define RELE_LUZ    14

//-------------------------------------------------
// Variáveis dos sensores
//-------------------------------------------------
int med_sen_gas;
int med_sen_luz;
int med_sen_tmp;

const int BETA = 3950;
const double k_coef = 273.15;
double temp;

//-------------------------------------------------
// LCD 20x4
//-------------------------------------------------
LiquidCrystal lcd(48, 38, 39, 40, 41, 42);

//-------------------------------------------------
// Controle do pisca-alerta de gás
//-------------------------------------------------
bool gasBlinkState = false;
unsigned long lastGasBlink = 0;
const unsigned long GAS_BLINK_INTERVAL = 1000;

//-------------------------------------------------
// Setup
//-------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(DIG_GAS, INPUT);
  pinMode(DIG_LDR, INPUT);
  pinMode(ANG_TMP, INPUT);

  pinMode(SIN_DE_GAS, OUTPUT);
  pinMode(RELE_LUZ, OUTPUT);

  lcd.begin(20, 4);
  lcd.clear();

  lcd.setCursor(5, 0);
  lcd.print("Sistema de");

  lcd.setCursor(1, 1);
  lcd.print("monitoramento IOT");

  lcd.setCursor(5, 2);
  lcd.print("feito pela");

  lcd.setCursor(3, 4);
  lcd.print("Equipe X");

  connectWiFi();
  delay(2000);
  lcd.clear();

  lcd.setCursor(1, 0);
  lcd.print("Para o programa de");

  lcd.setCursor(2, 1);
  lcd.print("especializacao em");

  lcd.setCursor(0, 2);
  lcd.print("eletronica embarcada");

  lcd.setCursor(0, 3);
  lcd.print("FIAP + CPQD + SOFTEX");  
  connectMQTT();
  delay(2000);
  lcd.clear();
}

//-------------------------------------------------
// Loop principal
//-------------------------------------------------
void loop() {

  if (!mqtt.connected()) {
    connectMQTT();
  }
  mqtt.loop();

  //-------------------------------------------------
  // Leitura dos sensores
  //-------------------------------------------------
  med_sen_gas = !digitalRead(DIG_GAS);
  med_sen_luz = digitalRead(DIG_LDR);
  med_sen_tmp = analogRead(ANG_TMP);

  temp = 1 / (log(1 / (4095.0 / med_sen_tmp - 1)) / BETA + 1.0 / 298.15) - k_coef;

  //-------------------------------------------------
  // Controle da luz:
  //-------------------------------------------------
  digitalWrite(RELE_LUZ, med_sen_luz);

  //-------------------------------------------------
  // Alerta de gás
  //-------------------------------------------------
  unsigned long now = millis();

  if (med_sen_gas) {
    if (now - lastGasBlink >= GAS_BLINK_INTERVAL) {
      lastGasBlink = now;
      gasBlinkState = !gasBlinkState;
      digitalWrite(SIN_DE_GAS, gasBlinkState);
    }
  } else {
    digitalWrite(SIN_DE_GAS, LOW);
    gasBlinkState = false;
  }

  //-------------------------------------------------
  // LCD 20x4 (1 Hz)
  //-------------------------------------------------
  if (now - lastLcd >= 1000) {
    lastLcd = now;

    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("Temp. atual: ");
    lcd.print(temp);
    lcd.print((char)223);
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("Estado da  luz: ");
    lcd.print(med_sen_luz ? "ON " : "OFF");

    lcd.setCursor(0, 2);
    lcd.print("Estado do  gas: ");
    lcd.print(med_sen_gas ? "SOS" : "OK");

    lcd.setCursor(0, 3);
    lcd.print("Estado do MQTT: ");
    lcd.print(mqtt.connected() ? "ON" : "OFF");
  }

  //-------------------------------------------------
  // MQTT a cada 2 segundos
  //-------------------------------------------------
  if (now - lastMqtt >= 2000) {
    lastMqtt = now;
    publishSensors();
  }
}
