#include <Wire.h>
#include <Kalman.h>
#include <I2Cdev.h>
#include <MPU6050.h>


Kalman kalmanX;
Kalman kalmanY;
MPU6050 accelgyro;

int const LED_PIN = 12;
int const PI_RECIEVE_PIN = 4;
int const PI_SEND_PIN = 1;
int const HEART_BEAT_Rate = 250;
boolean const DEBUG = true;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float KalmanX, KalmanY;
boolean ledOn = false;
int32_t timer;
int32_t cycleCount = 1;
int32_t setTimer;

void setup(){
  Wire.begin(4);
  
  //filter set up
  accelgyro.initialize();
  pinMode(LED_PIN, OUTPUT);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  kalmanX.setAngle(roll());
  kalmanY.setAngle(pitch());
  timer = micros();
  
  //pi conmuncaion setup
    pinMode(PI_SEND_PIN, OUTPUT);
  
  //debug
  Serial.begin(9600);
}

void loop(){
  if(DEBUG) setTimer = millis();
  filter();
  piCom();
  led();
  if(DEBUG){
    Serial.print("Time it take to run loop = ");
    Serial.println(millis() - setTimer);
    Serial.print("filtered pitch = ");
    Serial.println(KalmanY);
    Serial.print("Raw pitch= ");
    Serial.println(gy);
    Serial.print("Is led on =");
    Serial.println(ledOn);
  }
}

// filter parts
float roll(){
  return atan(ay / sqrt(ax * ax + az * az)) * RAD_TO_DEG;
}
float pitch(){
  return atan2(-ax, az) * RAD_TO_DEG;
}
void filter(){
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  double dt = (double)(micros() - timer / 1000000);
  timer = micros();
  
  double gyroXrate = gy / 131.0;
  double gyroYrate = gy / 131.0;
  
  KalmanX = kalmanX.getAngle(roll(), gyroXrate, dt);
  KalmanY = kalmanY.getAngle(pitch(), gyroYrate, dt);
}
// pi comunation

void piCom(){
  digitalWrite(PI_SEND_PIN, map(KalmanY,-90,90,0,255));
}

// led contorl
void led(){
  if(ledOn) digitalWrite(LED_PIN, HIGH);
  else digitalWrite(LED_PIN, LOW);
  ledOn = !ledOn;
}

