#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// Configurações de WiFi
const char *SSID = "Wokwi-GUEST";
const char *PASSWORD = "";  // Substitua pelo sua senha

// Configurações de MQTT
const char *BROKER_MQTT = "broker.hivemq.com";
const int BROKER_PORT = 1883;
const char *ID_MQTT = "esp32_atuadores";

// Tópicos MQTT - personalize com seu nome/identificação
const char *TOPIC_LED = "fiap/iot/seu_nome/led";
const char *TOPIC_BUZZER = "fiap/iot/seu_nome/buzzer";
const char *TOPIC_RGB = "fiap/iot/seu_nome/rgb";
const char *TOPIC_SERVO = "fiap/iot/seu_nome/servo";
const char *TOPIC_TODOS = "fiap/iot/seu_nome/todos";
const char *TOPIC_ESTADO = "fiap/iot/seu_nome/estado";

// Configurações de Hardware
#define PIN_LED 2       // LED embutido do ESP32
#define PIN_BUZZER 12   // Pino para o buzzer
#define PIN_RGB_R 25    // Pino vermelho do LED RGB
#define PIN_RGB_G 26    // Pino verde do LED RGB
#define PIN_RGB_B 27    // Pino azul do LED RGB
#define PIN_SERVO 13    // Pino para o servo motor

// Variáveis globais
WiFiClient espClient;
PubSubClient MQTT(espClient);
Servo servo;
const int TAMANHO = 256;  // Tamanho do buffer JSON

// Estado atual dos atuadores
bool estado_led = false;
bool estado_buzzer = false;
String cor_rgb = "off";
int posicao_servo = 0;

// Protótipos de funções
void initWiFi();
void initMQTT();
void initHardware();
void callbackMQTT(char *topic, byte *payload, unsigned int length);
void reconnectMQTT();
void reconnectWiFi();
void checkWiFIAndMQTT();
void setRGB(String cor);
void publishEstado();

void initWiFi() {
  Serial.print("Conectando com a rede: ");
  Serial.println(SSID);
  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Conectado com sucesso: ");
  Serial.println(SSID);
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void initMQTT() {
  MQTT.setServer(BROKER_MQTT, BROKER_PORT);
  MQTT.setCallback(callbackMQTT);
}

void initHardware() {
  // Configuração dos pinos
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_RGB_R, OUTPUT);
  pinMode(PIN_RGB_G, OUTPUT);
  pinMode(PIN_RGB_B, OUTPUT);
  
  // Inicializa o servo
  ESP32PWM::allocateTimer(0);
  servo.setPeriodHertz(50);     // PWM a 50Hz padrão para servos
  servo.attach(PIN_SERVO, 500, 2400); // Ajuste os valores conforme o servo
  
  // Desliga todos os atuadores inicialmente
  digitalWrite(PIN_LED, LOW);
  digitalWrite(PIN_BUZZER, LOW);
  setRGB("off");
  servo.write(0);
}

void callbackMQTT(char *topic, byte *payload, unsigned int length) {
  String mensagem = String((char*)payload).substring(0, length);
  
  Serial.printf("Mensagem recebida via MQTT: %s do tópico: %s\n", mensagem.c_str(), topic);

  StaticJsonDocument<TAMANHO> doc;
  DeserializationError error = deserializeJson(doc, mensagem);
  
  if (error) {
    Serial.print("Erro ao analisar JSON: ");
    Serial.println(error.c_str());
    return;
  }

  // Verifica qual tópico recebeu a mensagem e executa a ação correspondente
  if (String(topic) == TOPIC_LED) {
    if (doc.containsKey("estado")) {
      estado_led = doc["estado"].as<bool>();
      digitalWrite(PIN_LED, estado_led ? HIGH : LOW);
      Serial.print("LED: ");
      Serial.println(estado_led ? "LIGADO" : "DESLIGADO");
    }
  } 
  else if (String(topic) == TOPIC_BUZZER) {
    if (doc.containsKey("estado")) {
      estado_buzzer = doc["estado"].as<bool>();
      if (estado_buzzer) {
        // Buzzer ligado em 1kHz
        tone(PIN_BUZZER, 1000);
      } else {
        // Desliga o buzzer
        noTone(PIN_BUZZER);
      }
      Serial.print("Buzzer: ");
      Serial.println(estado_buzzer ? "LIGADO" : "DESLIGADO");
    }
  } 
  else if (String(topic) == TOPIC_RGB) {
    if (doc.containsKey("cor")) {
      cor_rgb = doc["cor"].as<String>();
      setRGB(cor_rgb);
      Serial.print("RGB: ");
      Serial.println(cor_rgb);
    }
  } 
  else if (String(topic) == TOPIC_SERVO) {
    if (doc.containsKey("posicao")) {
      posicao_servo = doc["posicao"].as<int>();
      servo.write(posicao_servo);
      Serial.print("Servo: ");
      Serial.println(posicao_servo);
    }
  } 
  else if (String(topic) == TOPIC_TODOS) {
    // Comando para todos os atuadores de uma vez
    if (doc.containsKey("led")) {
      estado_led = doc["led"].as<bool>();
      digitalWrite(PIN_LED, estado_led ? HIGH : LOW);
    }
    
    if (doc.containsKey("buzzer")) {
      estado_buzzer = doc["buzzer"].as<bool>();
      if (estado_buzzer) {
        tone(PIN_BUZZER, 1000);
      } else {
        noTone(PIN_BUZZER);
      }
    }
    
    if (doc.containsKey("rgb")) {
      cor_rgb = doc["rgb"].as<String>();
      setRGB(cor_rgb);
    }
    
    if (doc.containsKey("servo")) {
      posicao_servo = doc["servo"].as<int>();
      servo.write(posicao_servo);
    }
    
    Serial.println("Todos os atuadores atualizados");
  }

  // Publica o estado atual dos atuadores após qualquer alteração
  publishEstado();
}

void reconnectMQTT() {
  while (!MQTT.connected()) {
    Serial.print("Tentando conectar com o Broker MQTT: ");
    Serial.println(BROKER_MQTT);

    if (MQTT.connect(ID_MQTT)) {
      Serial.println("Conectado ao broker MQTT!");
      
      // Inscreve-se nos tópicos
      MQTT.subscribe(TOPIC_LED);
      MQTT.subscribe(TOPIC_BUZZER);
      MQTT.subscribe(TOPIC_RGB);
      MQTT.subscribe(TOPIC_SERVO);
      MQTT.subscribe(TOPIC_TODOS);
      
      // Publica mensagem informando que está online
      MQTT.publish("fiap/iot/seu_nome/status", "online");
      
      // Publica o estado atual dos atuadores
      publishEstado();
    } else {
      Serial.println("Falha na conexão com MQTT. Tentando novamente em 2 segundos.");
      delay(2000);
    }
  }
}

void checkWiFIAndMQTT() {
  if (WiFi.status() != WL_CONNECTED) reconnectWiFi();
  if (!MQTT.connected()) reconnectMQTT();
}

void reconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED)
    return;

  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Wifi conectado com sucesso");
  Serial.print(SSID);
  Serial.println("IP: ");
  Serial.println(WiFi.localIP());
}

// Função para definir a cor do LED RGB
void setRGB(String cor) {
  if (cor == "red" || cor == "vermelho") {
    digitalWrite(PIN_RGB_R, HIGH);
    digitalWrite(PIN_RGB_G, LOW);
    digitalWrite(PIN_RGB_B, LOW);
  } 
  else if (cor == "green" || cor == "verde") {
    digitalWrite(PIN_RGB_R, LOW);
    digitalWrite(PIN_RGB_G, HIGH);
    digitalWrite(PIN_RGB_B, LOW);
  } 
  else if (cor == "blue" || cor == "azul") {
    digitalWrite(PIN_RGB_R, LOW);
    digitalWrite(PIN_RGB_G, LOW);
    digitalWrite(PIN_RGB_B, HIGH);
  } 
  else if (cor == "yellow" || cor == "amarelo") {
    digitalWrite(PIN_RGB_R, HIGH);
    digitalWrite(PIN_RGB_G, HIGH);
    digitalWrite(PIN_RGB_B, LOW);
  } 
  else if (cor == "purple" || cor == "roxo") {
    digitalWrite(PIN_RGB_R, HIGH);
    digitalWrite(PIN_RGB_G, LOW);
    digitalWrite(PIN_RGB_B, HIGH);
  } 
  else if (cor == "cyan" || cor == "ciano") {
    digitalWrite(PIN_RGB_R, LOW);
    digitalWrite(PIN_RGB_G, HIGH);
    digitalWrite(PIN_RGB_B, HIGH);
  } 
  else if (cor == "white" || cor == "branco") {
    digitalWrite(PIN_RGB_R, HIGH);
    digitalWrite(PIN_RGB_G, HIGH);
    digitalWrite(PIN_RGB_B, HIGH);
  } 
  else {
    // Desliga todos os pinos (off)
    digitalWrite(PIN_RGB_R, LOW);
    digitalWrite(PIN_RGB_G, LOW);
    digitalWrite(PIN_RGB_B, LOW);
  }
}

// Função para publicar o estado atual de todos os atuadores
void publishEstado() {
  StaticJsonDocument<TAMANHO> doc;
  
  doc["led"] = estado_led;
  doc["buzzer"] = estado_buzzer;
  doc["rgb"] = cor_rgb;
  doc["servo"] = posicao_servo;
  
  char buffer[TAMANHO];
  serializeJson(doc, buffer);
  
  // Publica o estado no tópico de estado
  MQTT.publish(TOPIC_ESTADO, buffer);
  Serial.print("Estado publicado: ");
  Serial.println(buffer);
}

void setup() {
  Serial.begin(115200);
  
  initHardware();
  initWiFi();
  initMQTT();
}

void loop() {
  checkWiFIAndMQTT();
  MQTT.loop();
  
  // Outras ações contínuas, se necessário
  delay(10);
}