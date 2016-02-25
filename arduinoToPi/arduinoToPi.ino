#include <Wire.h>
#include <Kalman.h>
#include "I2Cdev.h"
#include "MPU6050"

#define SLAVE_ADDRESS 0x4

Kalman kalmanX;
Kalman kalmanY;
MPU6050 accelgyro

int const LED_PIN = 3;
int const PI_RECIEVE_PIN = 4;
int const PI_SEND_PIN = 1;
float ax, ay, az;
float gx, gy, gz;
float KalmanX, KalmanY;
uint32_t timer;

void setup(){
  Wire.begin(SLAVE_ADDRESS)
  
  //filter set up
  accelgyro.initialize();
  pinMode(LED_PIN, OUTPUT);
  accelgyro.getMotion6(&ax, &ay, $az, &gx, &gy, &gz);
  kalmanX.setAngle(roll());
  kalmanY.setAngle(pitch());
  timer = micros()
  
  //pi conmuncaion setup
  pinMode(PI_RECIEVE_PIN, INPUT);
  pinMode(PI_SEND_PIN, OUTPUT);
}

void loop(){
  filter();
  piCom();
}

// filter parts
float roll(){
  return atan(ay / sqrt(ax * ax + az * az)) * RAD_TO_DEG;
}
float pitch(){
  return atan2(-ax, az) * RAD_TO_DEG;
}
void filter(){
  accelgyro.getMotion6(&ax, &ay, $az, &gx, &gy, &gz);
  
  double dt = (double)(micros() - timer / 1000000;
  timer = micros();
  
  double gyroXrate = gy / 131.0;
  double gyroYrate = gy / 131.0;
  
  KalmanX = kalmanX.getAngle(roll, gyroXrate, dt);
  KalmanY = kalmanY.getAngle(pitch, gyroYrate, dt);
}
// pi comunation

void piCom(){
  if digitalRead(PI_RECIECVE_PIN) analogWrite(PI_SEND_PIN, map(KalmanY,-90,90,0,1023));
}
