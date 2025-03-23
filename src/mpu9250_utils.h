#ifndef MPU9250_UTILS_H
#define MPU9250_UTILS_H

#include <Arduino.h>
#include "MPU9250.h"

// Constantes para o intervalo de impressão
extern const unsigned long PRINT_INTERVAL;
extern unsigned long lastPrintTime;

/**
 * Configura os parâmetros do sensor MPU9250
 * @param sensor Referência para o objeto do sensor
 */
void configureSensor(MPU9250 &sensor);

/**
 * Lê todos os dados do sensor MPU9250 e os armazena nas variáveis passadas por referência
 * @param sensor Referência para o objeto do sensor
 * @param ax_ref, ay_ref, az_ref Referências para as variáveis de aceleração
 * @param gx_ref, gy_ref, gz_ref Referências para as variáveis do giroscópio
 * @param mx_ref, my_ref, mz_ref Referências para as variáveis do magnetômetro
 * @param temp_ref Referência para a variável de temperatura
 * @return true se a leitura foi bem-sucedida, false caso contrário
 */
bool readSensorData(MPU9250 &sensor, 
                   float &ax_ref, float &ay_ref, float &az_ref,
                   float &gx_ref, float &gy_ref, float &gz_ref,
                   float &mx_ref, float &my_ref, float &mz_ref,
                   float &temp_ref);

/**
 * Imprime os dados do sensor no monitor serial se o intervalo foi atingido
 * @param lastPrintTime_ref Referência para o timestamp da última impressão
 * @param printInterval Intervalo mínimo entre impressões em ms
 * @param ax, ay, az Valores de aceleração
 * @param gx, gy, gz Valores do giroscópio
 * @param mx, my, mz Valores do magnetômetro
 * @param temp Valor da temperatura
 * @return true se os dados foram impressos, false caso contrário
 */
bool printSensorData(unsigned long &lastPrintTime_ref, unsigned long printInterval,
                    float ax, float ay, float az,
                    float gx, float gy, float gz,
                    float mx, float my, float mz,
                    float temp);

#endif // MPU9250_UTILS_H