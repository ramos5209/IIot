#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define sensorPin 4 // Use um pino digital para o DS18B20 (D4, por exemplo)

const char* ssid = "linksys";
const char* password = ""; 

const char* apiKey = "TXA3UOMIOI7XREFK";

// Configuração do barramento OneWire para o DS18B20
OneWire oneWire(sensorPin);
DallasTemperature sensors(&oneWire);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.begin(115000);
  WiFi.begin(ssid, password);

  Serial.println("Conectando ao Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("\nWi-Fi conectado!");

  // Inicializar o sensor DS18B20
  sensors.begin();
}

void loop() {
  // Solicitar a leitura da temperatura do DS18B20
  sensors.requestTemperatures();
  
  // Obter a temperatura em Celsius do primeiro sensor conectado
  float temperatura = sensors.getTempCByIndex(0);  

  Serial.print("Temperatura: ");
  Serial.print(temperatura);
  Serial.println(" °C");

  if(WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    HTTPClient http;

    String url = "http://api.thingspeak.com/update?api_key=";
    url += apiKey;
    url += "&field1=" + String(temperatura);

    Serial.println("Enviando: " + url);
    
    http.begin(client, url);
    
    int httpResponseCode = http.GET();
    
    if(httpResponseCode > 0) {
      Serial.println("Resposta do servidor: " + String(httpResponseCode));
    } else {
      Serial.println("Erro ao enviar: " + http.errorToString(httpResponseCode));
    }

    http.end();
  }

  digitalWrite(LED_BUILTIN, HIGH);
  delay(5000);
  digitalWrite(LED_BUILTIN, LOW);

  delay(15000);  // Enviar a cada 15 segundos
}
