/**
 * @file kalman_filter.cpp
 * @brief Implementação do Filtro de Kalman para fusão de sensores IMU
 */

#include "kalman_filter.h"
#include <math.h>

KalmanFilter::KalmanFilter() {
    // Inicializar quaternion como identidade (sem rotação)
    q = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    
    // Inicializar bias do giroscópio como zero
    gyroBias[0] = gyroBias[1] = gyroBias[2] = 0.0f;
}

void KalmanFilter::begin(float gyroNoise, float accelNoise, float magNoise) {
    // Inicializa quaternion como identidade (sem rotação)
    q = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    
    // Garante que o bias do giroscópio seja zero
    gyroBias[0] = gyroBias[1] = gyroBias[2] = 0.0f;
    
    // Armazenar parâmetros de ruído
    gyroNoiseVar = gyroNoise * gyroNoise;
    accelNoiseVar = accelNoise * accelNoise;
    magNoiseVar = magNoise * magNoise;
    
    // Inicializar matriz de covariância P com valores diagonais
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            P[i][j] = (i == j) ? 0.1f : 0.0f;
        }
    }
}

void KalmanFilter::update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt) {
    // 1. Etapa de predição - integra dados do giroscópio
    predictState(gx - gyroBias[0], gy - gyroBias[1], gz - gyroBias[2], dt);
    
    // 2. Etapa de correção com acelerômetro - corrige roll e pitch
    if (ax != 0.0f || ay != 0.0f || az != 0.0f) {
        updateWithAccel(ax, ay, az);
    }
    
    // 3. Etapa de correção com magnetômetro - corrige yaw
    if (mx != 0.0f || my != 0.0f || mz != 0.0f) {
        updateWithMag(mx, my, mz);
    }
    
    // Normalizar quaternion para garantir unidade
    q.normalize();
}

void KalmanFilter::predictState(float gx, float gy, float gz, float dt) {
    // Implementação simplificada da integração do quaternion
    // usando o método de primeira ordem
    
    // Converter taxas angulares para quaternion
    Quaternion q_dot;
    q_dot.w = -0.5f * (q.x * gx + q.y * gy + q.z * gz);
    q_dot.x =  0.5f * (q.w * gx + q.y * gz - q.z * gy);
    q_dot.y =  0.5f * (q.w * gy + q.z * gx - q.x * gz);
    q_dot.z =  0.5f * (q.w * gz + q.x * gy - q.y * gx);
    
    // Integrar para atualizar o quaternion
    q.w += q_dot.w * dt;
    q.x += q_dot.x * dt;
    q.y += q_dot.y * dt;
    q.z += q_dot.z * dt;
    
    // Atualizar matriz de covariância P (implementação simplificada)
    // Aumenta a incerteza baseado na variância do giroscópio
    for (int i = 0; i < 3; i++) {
        P[i][i] += gyroNoiseVar * dt * dt;
    }
}

void KalmanFilter::updateWithAccel(float ax, float ay, float az) {
    // Normalizar o vetor de aceleração
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.001f) return; // Evitar divisão por zero
    
    ax /= norm;
    ay /= norm;
    az /= norm;
    
    // Converter o quaternion atual para matriz de rotação
    // e obter o vetor gravidade no sistema de coordenadas do corpo
    float R[3][3];
    R[0][0] = 1.0f - 2.0f * (q.y*q.y + q.z*q.z);
    R[0][1] = 2.0f * (q.x*q.y - q.w*q.z);
    R[0][2] = 2.0f * (q.x*q.z + q.w*q.y);
    R[1][0] = 2.0f * (q.x*q.y + q.w*q.z);
    R[1][1] = 1.0f - 2.0f * (q.x*q.x + q.z*q.z);
    R[1][2] = 2.0f * (q.y*q.z - q.w*q.x);
    R[2][0] = 2.0f * (q.x*q.z - q.w*q.y);
    R[2][1] = 2.0f * (q.y*q.z + q.w*q.x);
    R[2][2] = 1.0f - 2.0f * (q.x*q.x + q.y*q.y);
    
    // Vetor de referência de gravidade (apontando para baixo)
    float vRef[3] = {0.0f, 0.0f, 1.0f};
    
    // Projetar vetor de referência com a matriz de rotação
    float estimated[3];
    estimated[0] = R[0][0]*vRef[0] + R[0][1]*vRef[1] + R[0][2]*vRef[2];
    estimated[1] = R[1][0]*vRef[0] + R[1][1]*vRef[1] + R[1][2]*vRef[2];
    estimated[2] = R[2][0]*vRef[0] + R[2][1]*vRef[1] + R[2][2]*vRef[2];
    
    // Calcular erro entre medição e estimativa
    float error[3];
    error[0] = ax - estimated[0];
    error[1] = ay - estimated[1];
    error[2] = az - estimated[2];
    
    // Ganhos de Kalman (simplificado)
    float K[3];
    for (int i = 0; i < 3; i++) {
        K[i] = P[i][i] / (P[i][i] + accelNoiseVar);
    }
    
    // Aplicar correções ao quaternion e bias do giroscópio
    float correction[3];
    for (int i = 0; i < 3; i++) {
        correction[i] = K[i] * error[i];
    }
    
    // Implementação simplificada da atualização do quaternion
    // Usa aproximação de primeiro grau
    q.w += 0.5f * (-correction[0]*q.x - correction[1]*q.y - correction[2]*q.z);
    q.x += 0.5f * ( correction[0]*q.w + correction[2]*q.y - correction[1]*q.z);
    q.y += 0.5f * ( correction[1]*q.w - correction[2]*q.x + correction[0]*q.z);
    q.z += 0.5f * ( correction[2]*q.w + correction[1]*q.x - correction[0]*q.y);
    
    // Atualizar bias do giroscópio
    const float biasCorrectionFactor = 0.01f;
    gyroBias[0] += biasCorrectionFactor * correction[0];
    gyroBias[1] += biasCorrectionFactor * correction[1];
    gyroBias[2] += biasCorrectionFactor * correction[2];
    
    // Atualizar matriz de covariância
    for (int i = 0; i < 3; i++) {
        P[i][i] *= (1.0f - K[i]);
    }
}

void KalmanFilter::updateWithMag(float mx, float my, float mz) {
    // Normalizar o vetor do magnetômetro
    float norm = sqrtf(mx*mx + my*my + mz*mz);
    if (norm < 0.001f) return; // Evitar divisão por zero
    
    mx /= norm;
    my /= norm;
    mz /= norm;
    
    // Obter pitch e roll do quaternion atual
    EulerAngles euler = quaternionToEuler(q);
    
    // Compensar o magnetômetro pela inclinação
    float cosRoll = cosf(euler.roll);
    float sinRoll = sinf(euler.roll);
    float cosPitch = cosf(euler.pitch);
    float sinPitch = sinf(euler.pitch);
    
    // Aplicar rotação inversa para compensar inclinação
    float bx = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;
    float by = my * cosRoll - mz * sinRoll;
    
    // Calcular o ângulo de yaw do magnetômetro
    float magYaw = atan2f(-by, bx);
    
    // Comparar com o yaw atual
    float currentYaw = euler.yaw;
    
    // Calcular diferença de ângulo (considerando wrapping)
    float yawError = magYaw - currentYaw;
    if (yawError > M_PI) yawError -= 2.0f * M_PI;
    if (yawError < -M_PI) yawError += 2.0f * M_PI;
    
    // Ganho de Kalman para o yaw (simplificado)
    float K = 0.1f;
    
    // Aplicar correção ao quaternion apenas para o yaw
    // Convertemos de volta para quaternion para manter a representação consistente
    EulerAngles correctedEuler(euler.roll, euler.pitch, euler.yaw + K * yawError);
    q = eulerToQuaternion(correctedEuler);
}

KalmanFilter::EulerAngles KalmanFilter::quaternionToEuler(const Quaternion &q) {
    EulerAngles euler;
    
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    euler.roll = atan2f(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1.0f)
        euler.pitch = copysignf(M_PI / 2.0f, sinp); // Usar 90° nos polos
    else
        euler.pitch = asinf(sinp);
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    euler.yaw = atan2f(siny_cosp, cosy_cosp);
    
    return euler;
}

KalmanFilter::Quaternion KalmanFilter::eulerToQuaternion(const EulerAngles &euler) {
    Quaternion q;
    
    float cr = cosf(euler.roll * 0.5f);
    float sr = sinf(euler.roll * 0.5f);
    float cp = cosf(euler.pitch * 0.5f);
    float sp = sinf(euler.pitch * 0.5f);
    float cy = cosf(euler.yaw * 0.5f);
    float sy = sinf(euler.yaw * 0.5f);
    
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    
    q.normalize();
    return q;
}

KalmanFilter::EulerAngles KalmanFilter::getEulerAngles() {
    return quaternionToEuler(q);
}

void KalmanFilter::getEulerDegrees(float &roll, float &pitch, float &yaw) {
    EulerAngles euler = quaternionToEuler(q);
    
    // Converter de radianos para graus
    roll = euler.roll * 180.0f / M_PI;
    pitch = euler.pitch * 180.0f / M_PI;
    yaw = euler.yaw * 180.0f / M_PI;
}

void KalmanFilter::getAngularRates(float &rollRate, float &pitchRate, float &yawRate) {
    // Retorna as taxas angulares com o bias removido
    rollRate = -gyroBias[0];
    pitchRate = -gyroBias[1];
    yawRate = -gyroBias[2];
}

KalmanFilter::Quaternion KalmanFilter::getQuaternion() {
    return q;
}