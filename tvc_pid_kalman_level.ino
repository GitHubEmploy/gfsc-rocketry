#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <PID_v1.h>

Servo servoX;
Servo servoY;

const int servoXPin = 6;
const int servoYPin = 7;

const int minX = 20;  // Minimum X position
const int maxX = 120; // Maximum X position
const int minY = 50;  // Minimum Y position
const int maxY = 140; // Maximum Y position

MPU6050 mpu;

double SetpointX, InputX, OutputX;
double SetpointY, InputY, OutputY;

double KpX = 2.0, KiX = 5.0, KdX = 1.0;
double KpY = 2.0, KiY = 5.0, KdY = 1.0;

PID myPIDX(&InputX, &OutputX, &SetpointX, KpX, KiX, KdX, DIRECT);
PID myPIDY(&InputY, &OutputY, &SetpointY, KpY, KiY, KdY, DIRECT);

float angleX, angleY;
float biasX = 0, biasY = 0;
float rateX, rateY;
float P[2][2] = {{0,0},{0,0}}, K[2];
float S, y;
float dt = 0.01;

void kalmanUpdate(float &angle, float &bias, float rate, int measurement, float &P00, float &P01, float &P10, float &P11) {
  angle += dt * (rate - bias);
  P00 += dt * (dt*P11 - P01 - P10 + 1);
  P01 -= dt * P11;
  P10 -= dt * P11;
  P11 += dt;

  S = P00 + 1;
  K[0] = P00 / S;
  K[1] = P10 / S;
  y = measurement - angle;
  angle += K[0] * y;
  bias += K[1] * y;
  P00 -= K[0] * P00;
  P01 -= K[0] * P01;
  P10 -= K[1] * P00;
  P11 -= K[1] * P01;
}

void setup() {
  Wire.begin();
  servoX.attach(servoXPin);
  servoY.attach(servoYPin);
  mpu.initialize();
  if (!mpu.testConnection()) {
    while (1);
  }
  servoX.write((minX + maxX) / 2);
  servoY.write((minY + maxY) / 2);
  delay(1000);
  SetpointX = SetpointY = 0;
  myPIDX.SetMode(AUTOMATIC);
  myPIDY.SetMode(AUTOMATIC);
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate tilt angles from accelerometer data
  float angleAccX = atan2(ay, az) * 180 / PI;
  float angleAccY = atan2(ax, az) * 180 / PI;

  // Update Kalman filter for gyroscope rates
  rateX = gx / 131.0;
  rateY = gy / 131.0;
  kalmanUpdate(angleX, biasX, rateX, angleAccX, P[0][0], P[0][1], P[1][0], P[1][1]);
  kalmanUpdate(angleY, biasY, rateY, angleAccY, P[0][0], P[0][1], P[1][0], P[1][1]);

  // Update PID inputs with the filtered angles
  InputX = angleX;
  InputY = angleY;

  // Adjust setpoints dynamically to maintain level (i.e., 0 degree tilt)
  SetpointX = 0;
  SetpointY = 0;

  myPIDX.Compute();
  myPIDY.Compute();

  int servoPosX = map(OutputX, -180, 180, minX, maxX);
  int servoPosY = map(OutputY, -180, 180, minY, maxY);

  moveTo(servoPosX, servoPosY);
}

void moveTo(int x, int y) {
  x = constrain(x, minX, maxX);
  y = constrain(y, minY, maxY);
  servoX.write(x);
  servoY.write(y);
}
