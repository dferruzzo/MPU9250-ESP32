/**
 * @file oled_display.cpp
 * @brief Implementação da biblioteca para display OLED
 */

#include "oled_display.h"

OledDisplay::OledDisplay() : display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) {
  displayMode = 0;
}

bool OledDisplay::begin() {
  // Inicializa o display com tensão interna
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    return false;
  }
  
  // Configuração inicial
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  
  // Tela de boas-vindas
  display.setCursor(0, 0);
  display.println(F("MPU9250 + ESP32"));
  display.println(F("Sistema IMU"));
  display.println();
  display.println(F("Inicializando..."));
  display.display();
  
  return true;
}

void OledDisplay::clear() {
  display.clearDisplay();
  display.display();
}

void OledDisplay::showMessage(const char *title, const char *message) {
  display.clearDisplay();
  
  // Título em negrito
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(title);
  
  // Linha separadora
  display.drawLine(0, 10, SCREEN_WIDTH, 10, SSD1306_WHITE);
  
  // Mensagem
  display.setCursor(0, 16);
  display.println(message);
  
  display.display();
}

void OledDisplay::showAttitude(float roll, float pitch, float yaw) {
  display.clearDisplay();
  
  // Título
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("ATITUDE"));
  
  // Linha separadora
  display.drawLine(0, 8, SCREEN_WIDTH, 8, SSD1306_WHITE);
  
  // Valores
  display.setCursor(0, 16);
  display.print(F("R: "));
  display.print(roll, 1);
  display.println(F(" d"));
  
  display.setCursor(0, 26);
  display.print(F("P: "));
  display.print(pitch, 1);
  display.println(F(" d"));
  
  display.setCursor(0, 36);
  display.print(F("Y: "));
  display.print(yaw, 1);
  display.println(F(" d"));
  
  // Indicador visual
  int barWidth = 45;
  int barHeight = 8;
  int barX = 70;
  
  // Roll bar
  display.setCursor(barX, 16);
  display.print(F("R:"));
  display.drawRect(barX+10, 16, barWidth, barHeight, SSD1306_WHITE);
  int fillWidth = constrain(map(abs(roll), 0, 90, 0, barWidth), 0, barWidth);
  display.fillRect(barX+10, 16, fillWidth, barHeight, SSD1306_WHITE);
  
  // Pitch bar
  display.setCursor(barX, 26);
  display.print(F("P:"));
  display.drawRect(barX+10, 26, barWidth, barHeight, SSD1306_WHITE);
  fillWidth = constrain(map(abs(pitch), 0, 90, 0, barWidth), 0, barWidth);
  display.fillRect(barX+10, 26, fillWidth, barHeight, SSD1306_WHITE);
  
  // Yaw compass
  display.setCursor(barX, 36);
  display.print(F("Y:"));
  display.drawCircle(barX+30, 50, 10, SSD1306_WHITE);
  float radYaw = yaw * PI / 180.0;
  int needleX = barX+30 + 8 * sin(radYaw);
  int needleY = 50 - 8 * cos(radYaw);
  display.drawLine(barX+30, 50, needleX, needleY, SSD1306_WHITE);
  
  display.display();
}

void OledDisplay::drawHorizon(float roll, float pitch) {
  display.clearDisplay();
  
  // Título
  display.setCursor(0, 0);
  display.println(F("HORIZONTE ARTIFICIAL"));
  
  // Limitar pitch para visualização
  pitch = constrain(pitch, -90, 90);
  
  // Centro do horizonte
  int centerX = SCREEN_WIDTH / 2;
  int centerY = 32;
  int radius = 24;
  
  // Ângulos em radianos
  float rollRad = roll * PI / 180.0;
  float pitchOffset = map(pitch, -90, 90, 15, -15);
  
  // Desenhar círculo de referência
  display.drawCircle(centerX, centerY, radius, SSD1306_WHITE);
  
  // Calcular pontos do horizonte
  float sinRoll = sin(rollRad);
  float cosRoll = cos(rollRad);
  
  // Linha do horizonte
  int x1 = centerX - radius * cosRoll;
  int y1 = centerY - radius * sinRoll + pitchOffset;
  int x2 = centerX + radius * cosRoll;
  int y2 = centerY + radius * sinRoll + pitchOffset;
  display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
  
  // Marcas centrais
  display.drawLine(centerX-3, centerY, centerX+3, centerY, SSD1306_WHITE);
  display.drawLine(centerX, centerY-3, centerX, centerY+3, SSD1306_WHITE);
  
  // Mostrar valores
  display.setCursor(0, 54);
  display.print(F("R:"));
  display.print(roll, 1);
  display.print(F("  P:"));
  display.print(pitch, 1);
  
  display.display();
}

void OledDisplay::showSensorData(
    float roll, float pitch, float yaw,
    float ax, float ay, float az,
    float gx, float gy, float gz) {
  
  display.clearDisplay();
  
  // Título
  display.setCursor(0, 0);
  display.println(F("DADOS DO SENSOR"));
  
  // Acelerômetro
  display.setCursor(0, 10);
  display.println(F("Acel(m/s2):"));
  display.setCursor(0, 20);
  display.print(ax, 1);
  display.print(" ");
  display.print(ay, 1);
  display.print(" ");
  display.print(az, 1);
  
  // Giroscópio
  display.setCursor(0, 30);
  display.println(F("Giro(deg/s):"));
  display.setCursor(0, 40);
  display.print(gx, 1);
  display.print(" ");
  display.print(gy, 1);
  display.print(" ");
  display.print(gz, 1);
  
  // Atitude
  display.setCursor(0, 50);
  display.print(F("R:"));
  display.print(roll, 1);
  display.print(F(" P:"));
  display.print(pitch, 1);
  display.print(F(" Y:"));
  display.print(yaw, 1);
  
  display.display();
}

void OledDisplay::toggleDisplayMode() {
  // Alternar entre os modos (0, 1, 2)
  displayMode = (displayMode + 1) % 3;
}

void OledDisplay::update(
    float roll, float pitch, float yaw,
    float ax, float ay, float az, 
    float gx, float gy, float gz) {
  
  switch (displayMode) {
    case 0:
      showAttitude(roll, pitch, yaw);
      break;
    case 1:
      drawHorizon(roll, pitch);
      break;
    case 2:
      showSensorData(roll, pitch, yaw, ax, ay, az, gx, gy, gz);
      break;
  }
}