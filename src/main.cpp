#include <Arduino.h>
#include "MPU9250.h" /* https://github.com/bolderflight/MPU9250/blob/master/README.md */
#include <Wire.h>

#define LED_PIN 2
#define Red_LED 18

MPU9250 IMU(Wire, 0x68);
int status;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(Red_LED,OUTPUT);
  digitalWrite(Red_LED, LOW);

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
    digitalWrite(Red_LED, HIGH);
    delay(1000);
    digitalWrite(Red_LED, LOW);
  }
}

void loop() {
  // read the sensor
  IMU.readSensor();
  // display the data
  /*
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print("\t");
  */
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.print("\n");
    /*
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(),6);
  Serial.print("\t");
  */
  /*
  Serial.println(IMU.getTemperature_C(),6);
  Serial.print("\n");
  */
  delay(100);
  /*
  digitalWrite(Red_LED, HIGH);
  delay(1000);
  digitalWrite(Red_LED, LOW);
  delay(1000);
  */
}