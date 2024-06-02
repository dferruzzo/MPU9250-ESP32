/**
 * @file main.cpp
 * @brief Este arquivo contém o código-fonte para um projeto ESP32 que utiliza o sensor MPU9250 para coletar dados de aceleração, giroscópio, magnetômetro e temperatura. Os dados coletados são enviados para um broker MQTT.
 * 
 * @library Arduino.h: Biblioteca padrão do Arduino.
 * @library ArduinoJson.h e ArduinoJson.hpp: Bibliotecas para manipulação de JSON.
 * @library MPU9250.h: Biblioteca para manipulação do sensor MPU9250.
 * @library Wire.h: Biblioteca para comunicação I2C.
 * @library WiFi.h: Biblioteca para manipulação de WiFi.
 * @library sMQTTBroker.h: Biblioteca para manipulação de broker MQTT.
 * @library PubSubClient.h: Biblioteca para cliente MQTT.
 * 
 * @var sMQTTBroker broker: Instância do broker MQTT.
 * @var IPAddress local_IP: IP local.
 * @var IPAddress gateway: Gateway.
 * @var IPAddress subnet: Subnet.
 * @var const char* ssid: SSID da rede WiFi.
 * @var const char* password: Senha da rede WiFi.
 * @var const unsigned short mqttPort: Porta do broker MQTT.
 * @var const char* topic: Tópico MQTT onde os dados serão publicados.
 * @var const char ALIAS1[] a ALIAS10[]: Aliases para as variáveis da plataforma.
 * @var float ax, ay, az, gx, gy, gz, mx, my, mz, temperature: Variáveis para armazenar os dados do sensor.
 * @var MPU9250 IMU: Instância do sensor MPU9250.
 * @var int status: Variável para armazenar o status do sensor.
 * 
 * @function void setup(): Função de configuração inicial do ESP32.
 */
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include "MPU9250.h" /* https://github.com/bolderflight/MPU9250/blob/master/README.md */
#include <Wire.h>
#include <WiFi.h>
#include"sMQTTBroker.h"
#include <PubSubClient.h>

sMQTTBroker broker;

IPAddress local_IP(192, 168, 0, 73); //Defina o IP de acordo com a sua rede
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 0, 0);

const char* ssid = "MILENA";
const char* password = "juli1234";
const unsigned short mqttPort = 1883;

const char* topic = "Dados";
const char* topic1 = "Giroscopio";
const char* topic2 = "Acelerometro";
const char* topic3 = "Magnetometro";
const char* topic4 = "Temperatura";



//Declaracao das variaveis que armazenam os Aliases das variaveis da plataforma
const char ALIAS1[] = "AccX";
const char ALIAS2[] = "AccY";
const char ALIAS3[] = "AccZ";
const char ALIAS4[] = "GyrX";
const char ALIAS5[] = "GyrY";
const char ALIAS6[] = "GyrZ";
const char ALIAS7[] = "MagX";
const char ALIAS8[] = "MagY";
const char ALIAS9[] = "MagZ";
const char ALIAS10[] = "Temp";
const char ALIAS11[] = "Time";

float ax, ay, az, gx, gy, gz, mx, my, mz, temperature;

MPU9250 IMU(Wire, 0x68);
int status;

void setup() {
  delay(100);
  Wire.begin();
  Serial.begin(115200);
  while(!Serial) {}

  /* MPU setup function */
  status = IMU.begin();
  if (status<0){
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  } 

  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);  

  status = IMU.calibrateGyro();
  if(status>0){
    Serial.println("Gyro bias successfully removed\n");
    delay(1000);
  }

  // Wifi connection
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  // Set MQTT server details
  broker.init(mqttPort);
}

void loop() {
  // read the sensor
  IMU.readSensor();
  broker.update();
  // get the data
  ax = IMU.getAccelX_mss();
  ay = IMU.getAccelY_mss();
  az = IMU.getAccelZ_mss();
  gx = IMU.getGyroX_rads();
  gy = IMU.getGyroY_rads();
  gz = IMU.getGyroZ_rads();
  mx = IMU.getMagX_uT();
  my = IMU.getMagY_uT();
  mz = IMU.getMagZ_uT();
  temperature = IMU.getTemperature_C();

  //Cria o objeto dinamico "json" com tamanho "10" para a biblioteca
  /*
  DynamicJsonDocument json(11);
  //Atrela ao objeto "json" as leitura do sensor com os Aliases definidos
  json[ALIAS1] = ax;
  json[ALIAS2] = ay;
  json[ALIAS3] = az;
  json[ALIAS4] = gx;
  json[ALIAS5] = gy;
  json[ALIAS6] = gz;
  json[ALIAS7] = mx;
  json[ALIAS8] = my;
  json[ALIAS9] = mz;
  json[ALIAS10] = temperature;
  json[ALIAS11] = millis();   // Adiciona o carimbo de data/hora ao objeto JSON
  //
  //Mede o tamanho da mensagem "json" e atrela o valor somado em uma unidade ao objeto "tamanho_mensagem"
  size_t tamanho_mensagem = measureJson(json) + 1;
  //Cria a string "mensagem" de acordo com o tamanho do objeto "tamanho_mensagem"
  char mensagem[tamanho_mensagem];
  //Copia o objeto "json" para a variavel "mensagem" e com o "tamanho_mensagem"
  serializeJson(json, mensagem, tamanho_mensagem);
  //Publica a variavel "mensagem" no servidor utilizando a variavel "TOPICO"
  Serial.println("");
  Serial.print("Mensagem enviada: ");
  Serial.print(mensagem);
  broker.publish(topic, mensagem);
  */
  // Giroscopio
  DynamicJsonDocument json1(3);
  //Atrela ao objeto "json" as leitura do sensor com os Aliases definidos
  json1[ALIAS4] = gx;
  json1[ALIAS5] = gy;
  json1[ALIAS6] = gz;
  //Mede o tamanho da mensagem "json" e atrela o valor somado em uma unidade ao objeto "tamanho_mensagem"
  size_t tamanho_mensagem_1 = measureJson(json1) + 1;
  //Cria a string "mensagem" de acordo com o tamanho do objeto "tamanho_mensagem"
  char mensagem_1[tamanho_mensagem_1];
  //Copia o objeto "json" para a variavel "mensagem" e com o "tamanho_mensagem"
  serializeJson(json1, mensagem_1, tamanho_mensagem_1);
  //Publica a variavel "mensagem" no servidor utilizando a variavel "TOPICO"
  Serial.println("");
  Serial.print("Mensagem "); Serial.print(topic1); Serial.println(" enviada.");  
  Serial.print(mensagem_1);
  broker.publish(topic1, mensagem_1);
  //
  // Aceleometro
  DynamicJsonDocument json2(3);
  //Atrela ao objeto "json" as leitura do sensor com os Aliases definidos
  json2[ALIAS1] = ax;
  json2[ALIAS2] = ay;
  json2[ALIAS3] = az;
  //Mede o tamanho da mensagem "json" e atrela o valor somado em uma unidade ao objeto "tamanho_mensagem"
  size_t tamanho_mensagem_2 = measureJson(json2) + 1;
  //Cria a string "mensagem" de acordo com o tamanho do objeto "tamanho_mensagem"
  char mensagem_2[tamanho_mensagem_2];
  //Copia o objeto "json" para a variavel "mensagem" e com o "tamanho_mensagem"
  serializeJson(json2, mensagem_2, tamanho_mensagem_2);
  //Publica a variavel "mensagem" no servidor utilizando a variavel "TOPICO"
  Serial.println("");
  Serial.print("Mensagem "); Serial.print(topic2); Serial.println(" enviada.");  
  Serial.print(mensagem_2);
  broker.publish(topic2, mensagem_2);
  //
  // Magnetometro
  DynamicJsonDocument json3(3);
  //Atrela ao objeto "json" as leitura do sensor com os Aliases definidos
  json3[ALIAS7] = mx;
  json3[ALIAS8] = my;
  json3[ALIAS9] = mz;
  //Mede o tamanho da mensagem "json" e atrela o valor somado em uma unidade ao objeto "tamanho_mensagem"
  size_t tamanho_mensagem_3 = measureJson(json3) + 1;
  //Cria a string "mensagem" de acordo com o tamanho do objeto "tamanho_mensagem"
  char mensagem_3[tamanho_mensagem_3];
  //Copia o objeto "json" para a variavel "mensagem" e com o "tamanho_mensagem"
  serializeJson(json3, mensagem_3, tamanho_mensagem_3);
  //Publica a variavel "mensagem" no servidor utilizando a variavel "TOPICO"
  Serial.println("");
  Serial.print("Mensagem "); Serial.print(topic3); Serial.println(" enviada.");  
  Serial.print(mensagem_3);
  broker.publish(topic3, mensagem_3);
  //
  // Temperatura
  DynamicJsonDocument json4(1);
  //Atrela ao objeto "json" as leitura do sensor com os Aliases definidos
  json4[ALIAS10] = temperature;
  //Mede o tamanho da mensagem "json" e atrela o valor somado em uma unidade ao objeto "tamanho_mensagem"
  size_t tamanho_mensagem_4 = measureJson(json4) + 1;
  //Cria a string "mensagem" de acordo com o tamanho do objeto "tamanho_mensagem"
  char mensagem_4[tamanho_mensagem_4];
  //Copia o objeto "json" para a variavel "mensagem" e com o "tamanho_mensagem"
  serializeJson(json4, mensagem_4, tamanho_mensagem_4);
  //Publica a variavel "mensagem" no servidor utilizando a variavel "TOPICO"
  Serial.println("");
  Serial.print("Mensagem "); Serial.print(topic4); Serial.println(" enviada.");
  Serial.print(mensagem_4);
  broker.publish(topic4, mensagem_4);
  //
  delay(500);
}

