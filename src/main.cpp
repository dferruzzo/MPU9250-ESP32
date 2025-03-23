#include <Arduino.h>
#include <Wire.h>
#include "MPU9250.h" /* https://github.com/bolderflight/MPU9250/blob/master/README.md */
#include "mpu9250_utils.h"
#include "imu_fusion.h" // Adicionar a nova biblioteca de fusão

// Configuração dos pinos I2C para ESP32
const int SDA_PIN = 21;
const int SCL_PIN = 22;

MPU9250 IMU(Wire, 0x68);
int status;

// Variáveis para armazenar os dados
float ax, ay, az;          // Aceleração em m/s²
float gx, gy, gz;          // Velocidade angular em rad/s
float mx, my, mz;          // Campo magnético em μT
float temperature;         // Temperatura em °C

// Variáveis para temporização
unsigned long lastTime = 0;
float deltaTime = 0.0f;

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
  
  // Aplicar configurações usando a função dedicada
  configureSensor(IMU);
  
  Serial.println("MPU9250 inicializado com sucesso");

  status = IMU.calibrateGyro();
  if(status>0){
    Serial.println("Gyro bias successfully removed\n");
    delay(1000);
  }
  
  // Inicializar o filtro de fusão - isso agora também zera as variáveis de atitude
  setupIMUFusion();
  
  // Garantir que os valores estão realmente zerados
  resetAttitudeAndRates();
  
  Serial.println("MPU9250 e filtro de fusão inicializados com sucesso");
  Serial.println("Atitude e taxas inicializadas em zero");

  // Inicializar o timestamp
  lastTime = millis();
  lastPrintTime = lastTime;

}

void loop() {
  // Calcular delta de tempo em segundos
  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0f;
  lastTime = currentTime;

  // read the sensor
  if (!readSensorData(IMU, ax, ay, az, gx, gy, gz, mx, my, mz, temperature)) {
    Serial.println("Erro na leitura do sensor");
    delay(100);
    return;
  }

   // Atualizar o filtro de fusão com os novos dados
  updateIMUFusion(ax, ay, az, gx, gy, gz, mx, my, mz, deltaTime);

  /*
    // Imprimir dados
  printSensorData(lastPrintTime, PRINT_INTERVAL, 
    ax, ay, az, gx, gy, gz, mx, my, mz, temperature);
*/

    // Imprimir dados de atitude estimados pelo filtro
  if (millis() - lastPrintTime > PRINT_INTERVAL) {
    char attitudeBuffer[128];
    getAttitudeJson(attitudeBuffer, sizeof(attitudeBuffer));
    
    //Serial.println("--- ATITUDE ESTIMADA ---");
    Serial.print("Roll: "); Serial.print(roll, 1);
    Serial.print("° | Pitch: "); Serial.print(pitch, 1);
    Serial.print("° | Yaw: "); Serial.print(yaw, 1);
    Serial.println("°");
    //Serial.print("JSON: "); Serial.println(attitudeBuffer);
    Serial.println();
  }
  
  // Pequeno delay para estabilidade
  delay(5);

}

