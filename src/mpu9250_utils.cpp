#include "mpu9250_utils.h"

// Definição das variáveis globais
const unsigned long PRINT_INTERVAL = 500;
unsigned long lastPrintTime = 0;

void configureSensor(MPU9250 &sensor) {
  // setting the accelerometer full scale range to +/-8G 
  sensor.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  sensor.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  sensor.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  sensor.setSrd(19);
}

bool readSensorData(MPU9250 &sensor, 
                   float &ax_ref, float &ay_ref, float &az_ref,
                   float &gx_ref, float &gy_ref, float &gz_ref,
                   float &mx_ref, float &my_ref, float &mz_ref,
                   float &temp_ref) {
  
  // Lê os dados do sensor (verifica se readSensor retorna 0 para sucesso)
  if (sensor.readSensor() != 1) {
    return false;  // Erro na leitura
  }
  
  // Obtém os dados e os armazena nas variáveis passadas por referência
  ax_ref = sensor.getAccelX_mss();
  ay_ref = sensor.getAccelY_mss();
  az_ref = sensor.getAccelZ_mss();
  
  gx_ref = sensor.getGyroX_rads();
  gy_ref = sensor.getGyroY_rads();
  gz_ref = sensor.getGyroZ_rads();
  
  mx_ref = sensor.getMagX_uT();
  my_ref = sensor.getMagY_uT();
  mz_ref = sensor.getMagZ_uT();
  
  temp_ref = sensor.getTemperature_C();
  
  return true;  // Leitura bem-sucedida
}

bool printSensorData(unsigned long &lastPrintTime_ref, unsigned long printInterval,
                    float ax, float ay, float az,
                    float gx, float gy, float gz,
                    float mx, float my, float mz,
                    float temp) {
  // Verificar se está na hora de imprimir
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime_ref <= printInterval) {
    return false;  // Não é hora de imprimir ainda
  }

  // Imprimir os dados no formato solicitado
  Serial.print("AccX: "); Serial.print(ax); Serial.print(" m/s^2");
  Serial.print(" | AccY: "); Serial.print(ay); Serial.print(" m/s^2");
  Serial.print(" | AccZ: "); Serial.print(az); Serial.print(" m/s^2");
  Serial.print(" | GyrX: "); Serial.print(gx); Serial.print(" rad/s");
  Serial.print(" | GyrY: "); Serial.print(gy); Serial.print(" rad/s");
  Serial.print(" | GyrZ: "); Serial.print(gz); Serial.print(" rad/s");
  Serial.print(" | MagX: "); Serial.print(mx); Serial.print(" uT");
  Serial.print(" | MagY: "); Serial.print(my); Serial.print(" uT");
  Serial.print(" | MagZ: "); Serial.print(mz); Serial.print(" uT");
  Serial.print(" | Temp: "); Serial.print(temp); Serial.println(" C");
  Serial.println("");

  // Atualizar o timestamp da última impressão
  lastPrintTime_ref = currentTime;

  return true;  // Dados impressos com sucesso
}