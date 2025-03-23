#include <Arduino.h>
#include <Wire.h>
#include "MPU9250.h" /* https://github.com/bolderflight/MPU9250/blob/master/README.md */
#include "mpu9250_utils.h"
#include "imu_fusion.h" 
#include "oled_display.h" // Biblioteca do display OLED

// Configuração dos pinos I2C para ESP32
const int SDA_PIN = 21;
const int SCL_PIN = 22;
// Para botão (opcional)
const int BUTTON_PIN = 0; // Botão BOOT do ESP32

MPU9250 IMU(Wire, 0x68);
int status;

// Cria instância do display OLED
OledDisplay oled;

// Variáveis para armazenar os dados
float ax, ay, az;          // Aceleração em m/s²
float gx, gy, gz;          // Velocidade angular em rad/s
float mx, my, mz;          // Campo magnético em μT
float temperature;         // Temperatura em °C

// Variáveis para temporização
unsigned long lastTime = 0;
unsigned long lastPrintTime = 0;  // Essa é a definição real que será usada
unsigned long lastButtonCheck = 0;
float deltaTime = 0.0f;

// Estado do botão
int lastButtonState = HIGH;

void setup() {
  delay(100);
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
  while(!Serial) {}
  
  // Configurar pino do botão (opcional)
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Inicializar display OLED
  if (!oled.begin()) {
    Serial.println("Falha ao inicializar o display OLED");
  }
  
  oled.showMessage("Sistema IMU", "Inicializando...");
  delay(1000);

  /* MPU setup function */
  status = IMU.begin();
  if (status<0){
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    
    oled.showMessage("ERRO", "IMU nao detectado\nVerifique conexoes");
    while(1) {}
  } 
  
  // Aplicar configurações usando a função dedicada
  configureSensor(IMU);
  
  oled.showMessage("Calibrando", "Aguarde...");
  
  status = IMU.calibrateGyro();
  if(status>0){
    Serial.println("Gyro bias successfully removed");
    oled.showMessage("Sucesso", "Giroscopio calibrado");
    delay(500);
  }
  
  // Inicializar o filtro de fusão
  setupIMUFusion();
  
  // Garantir que os valores estão zerados
  resetAttitudeAndRates();
  
  oled.showMessage("Pronto", "Iniciando leituras...");
  delay(1000);

  // Inicializar os timestamps
  lastTime = millis();
  lastPrintTime = lastTime;
  lastButtonCheck = lastTime;
}

void loop() {
  // Calcular delta de tempo em segundos
  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0f;
  lastTime = currentTime;

  // Ler o sensor
  if (!readSensorData(IMU, ax, ay, az, gx, gy, gz, mx, my, mz, temperature)) {
    Serial.println("Erro na leitura do sensor");
    oled.showMessage("ERRO", "Falha na leitura");
    delay(100);
    return;
  }

  // Atualizar o filtro de fusão com os novos dados
  updateIMUFusion(ax, ay, az, gx, gy, gz, mx, my, mz, deltaTime);

  // Verificar botão para mudar o modo de exibição (opcional)
  if (currentTime - lastButtonCheck > 200) {  // Debounce de 200ms
    int buttonState = digitalRead(BUTTON_PIN);
    if (buttonState == LOW && lastButtonState == HIGH) {
      oled.toggleDisplayMode();
    }
    lastButtonState = buttonState;
    lastButtonCheck = currentTime;
  }

  // Atualizar display e Serial a cada 100ms
  if (currentTime - lastPrintTime > 100) {
    lastPrintTime = currentTime;
    
    // Atualizar o display OLED
    oled.update(roll, pitch, yaw, 
               ax, ay, az, 
               gx * RAD_TO_DEG, gy * RAD_TO_DEG, gz * RAD_TO_DEG);
    
    // Mostrar dados no monitor serial
    Serial.print("Roll: "); Serial.print(roll, 1);
    Serial.print("° | Pitch: "); Serial.print(pitch, 1);
    Serial.print("° | Yaw: "); Serial.print(yaw, 1);
    Serial.println("°");
  }
  
  // Pequeno delay para estabilidade
  delay(5);
}

