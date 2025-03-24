/**
 * @file kalman_filter.h
 * @brief Filtro de Kalman para fusão de sensores IMU e estimação de atitude
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Arduino.h>

class KalmanFilter {
public:
    // Estrutura para armazenar quaternions
    struct Quaternion {
        float w, x, y, z;
        
        Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
        Quaternion(float _w, float _x, float _y, float _z) : w(_w), x(_x), y(_y), z(_z) {}
        
        // Normalização do quaternion
        void normalize() {
            float norm = sqrtf(w*w + x*x + y*y + z*z);
            if (norm > 0.0f) {
                w /= norm;
                x /= norm;
                y /= norm;
                z /= norm;
            }
        }
    };
    
    // Estrutura para armazenar ângulos de Euler (em radianos)
    struct EulerAngles {
        float roll, pitch, yaw;
        
        EulerAngles() : roll(0.0f), pitch(0.0f), yaw(0.0f) {}
        EulerAngles(float _roll, float _pitch, float _yaw) : roll(_roll), pitch(_pitch), yaw(_yaw) {}
    };
    
    // Constructor
    KalmanFilter();
    
    // Inicialização do filtro
    void begin(float gyroNoise = 0.01f, float accelNoise = 0.1f, float magNoise = 0.1f);
    
    // Função principal para atualização do filtro
    void update(
        float ax, float ay, float az,    // Acelerômetro (m/s²)
        float gx, float gy, float gz,    // Giroscópio (rad/s)
        float mx, float my, float mz,    // Magnetômetro (uT)
        float dt                         // Delta de tempo (s)
    );
    
    // Obter ângulos de Euler estimados (em graus)
    EulerAngles getEulerAngles();
    
    // Obter ângulos de Euler estimados em graus
    void getEulerDegrees(float &roll, float &pitch, float &yaw);
    
    // Obter taxas angulares corrigidas (bias removido) em rad/s
    void getAngularRates(float &rollRate, float &pitchRate, float &yawRate);
    
    // Obter quaternion estimado
    Quaternion getQuaternion();
    
private:
    // Estado do filtro
    Quaternion q;               // Quaternion atual (orientação)
    float gyroBias[3];         // Bias do giroscópio
    
    // Matrizes de covariância
    float P[6][6];             // Matriz de covariância do estado
    
    // Parâmetros de ruído
    float gyroNoiseVar;        // Variância do ruído do giroscópio
    float accelNoiseVar;       // Variância do ruído do acelerômetro
    float magNoiseVar;         // Variância do ruído do magnetômetro
    
    // Funções auxiliares
    void predictState(float gx, float gy, float gz, float dt);
    void updateWithAccel(float ax, float ay, float az);
    void updateWithMag(float mx, float my, float mz);
    EulerAngles quaternionToEuler(const Quaternion &q);
    Quaternion eulerToQuaternion(const EulerAngles &euler);
};

#endif // KALMAN_FILTER_H