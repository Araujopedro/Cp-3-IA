#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h> // Biblioteca MQTT
#include <ArduinoJson.h>  // Para formatação JSON
#include <Wire.h>
#include <Adafruit_BME280.h> // Biblioteca para sensor BME280 (temperatura, umidade, pressão)

// Identificadores
const char* ID        = "gp3"; // Substitua pelo ID do seu grupo
const char* moduleID  = "ESP32_01";  // Ou outro ID para identificar seu ESP32

// Wi-Fi
const char* SSID      = "Wokwi-GUEST";  // Mantenha esse SSID para o Wokwi
const char* PASSWORD  = "";             // Mantenha vazio para o Wokwi

// MQTT Broker
const char* BROKER_MQTT  = "4.242.226.168";  // IP da sua VM Azure
const int   BROKER_PORT  = 1883;             // Porta padrão MQTT
const char* mqttUser     = "";               // Deixe vazio se não configurou autenticação
const char* mqttPassword = "";               // Deixe vazio se não configurou autenticação

// Tópico MQTT
#define TOPICO_PUBLISH  "gp3/esp32/dados"     // Tópico para publicar dados
#define TOPICO_SUBSCRIBE "seugrupo/esp32/comandos"  // Tópico para receber comandos
// MQTT Broker
const char* BROKER_MQTT  = "test.mosquitto.org";  // Broker público para teste
const int   BROKER_PORT  = 1883;
const char* mqttUser     = "Pedro";
const char* mqttPassword = "Pedro";
// Definições de pinos
const int SERVO_PIN = 27;
const int BUZZER_PIN = 13;
const int LIGHT_PIN = 15;
const int LED_R = 21;
const int LED_G = 22;
const int LED_B = 23;

Servo myservo;
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BME280 bme; // Sensor BME280

// Variáveis para simulação de sensores
float temperatura, umidade, pressao, altitude;
unsigned long lastSensorRead = 0;
const long sensorInterval = 5000; // Intervalo de leitura dos sensores (5 segundos)

// Função para conectar ao WiFi
void connectWiFi() {
  Serial.print("Conectando ao WiFi ");
  Serial.println(SSID);
  
  WiFi.begin(SSID, PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("Endereço IP: ");
  Serial.println(WiFi.localIP());
}

// Função de callback para mensagens MQTT recebidas
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensagem recebida [");
  Serial.print(topic);
  Serial.print("] ");
  
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);
  
  // Processamento de comandos aqui
  // (código de controle de atuadores do exemplo anterior)
}

// Função para reconectar ao broker MQTT
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Conectando ao broker MQTT...");
    
    // Tenta conectar com ID exclusivo
    if (client.connect(moduleID, mqttUser, mqttPassword)) {
      Serial.println("conectado");
      
      // Inscreve no tópico de comandos
      client.subscribe(TOPICO_SUBSCRIBE);
      
      // Publica mensagem de status
      client.publish(TOPICO_PUBLISH, "ESP32 conectado!");
    } else {
      Serial.print("falha, rc=");
      Serial.print(client.state());
      Serial.println(" tentando novamente em 5 segundos");
      delay(5000);
    }
  }
}

// Lê os sensores (ou simula leituras)
void readSensors() {
  // Tentar ler do sensor BME280 se disponível
  bool sensorFound = false;
  
  if (bme.begin(0x76)) {  // Endereço padrão do BME280
    sensorFound = true;
    temperatura = bme.readTemperature();
    umidade = bme.readHumidity();
    pressao = bme.readPressure() / 100.0F; // Convertendo para hPa
    altitude = bme.readAltitude(1013.25);  // Altitude relativa à pressão ao nível do mar
  }
  
  // Se o sensor não foi encontrado ou as leituras estão fora do intervalo, simular valores
  if (!sensorFound) {
    // Simulação de valores dentro dos intervalos especificados
    temperatura = random(200, 350) / 10.0;   // 20.0 a 35.0°C
    umidade = random(40, 80);                // 40% a 80%
    pressao = random(9800, 10500) / 10.0;    // 980 a 1050 hPa
    altitude = random(0, 500);               // 0 a 500 m
  }
  
  // Garantir que os valores estejam dentro dos intervalos especificados
  temperatura = constrain(temperatura, 20.0, 35.0);
  umidade = constrain(umidade, 40.0, 80.0);
  pressao = constrain(pressao, 980.0, 1050.0);
  altitude = constrain(altitude, 0.0, 500.0);
  
  Serial.println("Leitura de sensores:");
  Serial.print("Temperatura: "); Serial.print(temperatura); Serial.println(" °C");
  Serial.print("Umidade: "); Serial.print(umidade); Serial.println(" %");
  Serial.print("Pressão: "); Serial.print(pressao); Serial.println(" hPa");
  Serial.print("Altitude: "); Serial.print(altitude); Serial.println(" m");
}

// Envia dados dos sensores via MQTT
void sendSensorData() {
  // Cria um objeto JSON para os dados
  StaticJsonDocument<200> doc;
  doc["device_id"] = moduleID;
  doc["temperatura"] = temperatura;
  doc["umidade"] = umidade;
  doc["pressao"] = pressao;
  doc["altitude"] = altitude;
  
  // Serializa o JSON para uma string
  char jsonBuffer[200];
  serializeJson(doc, jsonBuffer);
  
  // Publica a mensagem
  if (client.publish(TOPICO_PUBLISH, jsonBuffer)) {
    Serial.println("Dados enviados com sucesso");
  } else {
    Serial.println("Falha ao enviar dados");
  }
}

void setup() {
  Serial.begin(9600);
  
  // Configuração dos pinos
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  // Inicializa servo
  myservo.attach(SERVO_PIN, 500, 2400);
  
  // Inicializa I2C para o sensor
  Wire.begin();
  
  // Conecta ao WiFi
  connectWiFi();
  
  // Configura servidor MQTT
  client.setServer(BROKER_MQTT, BROKER_PORT);
  client.setCallback(callback);
  
  // Leitura inicial dos sensores
  readSensors();
  
  Serial.println("Sistema de monitoramento ambiental iniciado!");
}

void loop() {
  // Verifica conexão MQTT e reconecta se necessário
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();
  
  // Lê e envia dados dos sensores a cada intervalo definido
  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorRead >= sensorInterval) {
    lastSensorRead = currentMillis;
    
    // Lê os sensores
    readSensors();
    
    // Envia os dados
    sendSensorData();
    
    // Indicação visual de envio (pisca o LED verde)
    digitalWrite(LED_G, HIGH);
    delay(100);
    digitalWrite(LED_G, LOW);
  }
}