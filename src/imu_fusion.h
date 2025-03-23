/**
 * @file imu_fusion.h
 * @brief Interface para uso da fusão de sensores IMU com filtro de Kalman
 */

#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include "kalman_filter.h"

// Instância do filtro de Kalman
KalmanFilter imuFilter;

// Variáveis para armazenar os resultados
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;          // Ângulos em graus
float rollRate = 0.0f, pitchRate = 0.0f, yawRate = 0.0f;  // Taxas em rad/s

/**
 * Zera explicitamente todos os valores de atitude e taxas
 */
void resetAttitudeAndRates() {
    roll = pitch = yaw = 0.0f;
    rollRate = pitchRate = yawRate = 0.0f;
}

/**
 * Inicializa o filtro de fusão de sensores
 * @param gyroNoise Desvio padrão do ruído do giroscópio (rad/s)
 * @param accelNoise Desvio padrão do ruído do acelerômetro (normalizado)
 * @param magNoise Desvio padrão do ruído do magnetômetro (normalizado)
 */
void setupIMUFusion(float gyroNoise = 0.01f, float accelNoise = 0.05f, float magNoise = 0.1f) {
    // Zerar explicitamente todos os valores de atitude e taxas
    resetAttitudeAndRates();
    
    // Inicializar o filtro de Kalman com os parâmetros de ruído
    imuFilter.begin(gyroNoise, accelNoise, magNoise);
}

/**
 * Atualiza o filtro de fusão com novos dados dos sensores
 * @param ax, ay, az Aceleração nos 3 eixos (m/s²)
 * @param gx, gy, gz Velocidade angular nos 3 eixos (rad/s)
 * @param mx, my, mz Campo magnético nos 3 eixos (µT)
 * @param dt Tempo decorrido desde a última atualização (s)
 */
void updateIMUFusion(float ax, float ay, float az, 
                     float gx, float gy, float gz, 
                     float mx, float my, float mz,
                     float dt) {
    
    // Atualiza o filtro com os dados dos sensores
    imuFilter.update(ax, ay, az, gx, gy, gz, mx, my, mz, dt);
    
    // Obtém os resultados estimados
    imuFilter.getEulerDegrees(roll, pitch, yaw);
    imuFilter.getAngularRates(rollRate, pitchRate, yawRate);
}

/**
 * Formata os dados de atitude para JSON
 * @param buffer Buffer para armazenar o JSON formatado
 * @param bufferSize Tamanho do buffer
 */
void getAttitudeJson(char* buffer, size_t bufferSize) {
    snprintf(buffer, bufferSize, 
             "{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,\"rollRate\":%.4f,\"pitchRate\":%.4f,\"yawRate\":%.4f}",
             roll, pitch, yaw, rollRate, pitchRate, yawRate);
}

#endif // IMU_FUSION_H