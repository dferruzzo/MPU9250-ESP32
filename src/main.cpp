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
  DynamicJsonDocument json(10);
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

  //Mede o tamanho da mensagem "json" e atrela o valor somado em uma unidade ao objeto "tamanho_mensagem"
  size_t tamanho_mensagem = measureJson(json) + 1;

  //Cria a string "mensagem" de acordo com o tamanho do objeto "tamanho_mensagem"
  char mensagem[tamanho_mensagem];

  //Copia o objeto "json" para a variavel "mensagem" e com o "tamanho_mensagem"
  serializeJson(json, mensagem, tamanho_mensagem);

  //Publica a variavel "mensagem" no servidor utilizando a variavel "TOPICO"
  Serial.println("");
  Serial.print("Mensagem enviada: ");
  Serial.println(mensagem);
  broker.publish(topic, mensagem);
  delay(1000);

}

