/**
 * @file oled_display.h
 * @brief Biblioteca simples para display OLED 0.96" (128x64)
 */

#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Definições do display
#define SCREEN_WIDTH 128  // Largura em pixels
#define SCREEN_HEIGHT 64  // Altura em pixels
#define OLED_ADDRESS 0x3C // Endereço I2C padrão (pode ser 0x3D)
#define OLED_RESET   -1   // Pino de reset (-1 = compartilha reset do Arduino)

class OledDisplay {
public:
  // Construtor
  OledDisplay();
  
  // Inicializa o display
  bool begin();
  
  // Limpa o display
  void clear();
  
  // Mostra uma mensagem na tela
  void showMessage(const char *title, const char *message);
  
  // Exibe os dados de atitude (roll, pitch, yaw)
  void showAttitude(float roll, float pitch, float yaw);
  
  // Desenha o horizonte artificial
  void drawHorizon(float roll, float pitch);
  
  // Mostra dados completos do sensor
  void showSensorData(
    float roll, float pitch, float yaw,
    float ax, float ay, float az,
    float gx, float gy, float gz);
    
  // Modo de visualização (alterna com botão)
  void toggleDisplayMode();
  
  // Atualiza display com dados do sensor
  void update(
    float roll, float pitch, float yaw,
    float ax, float ay, float az, 
    float gx, float gy, float gz);

private:
  Adafruit_SSD1306 display;
  uint8_t displayMode; // 0=attitude, 1=horizon, 2=sensor data
};

#endif // OLED_DISPLAY_H