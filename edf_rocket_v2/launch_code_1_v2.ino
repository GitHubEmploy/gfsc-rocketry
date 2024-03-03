#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <ESP32Servo.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

#define BMP_CS 10
Servo parachuteServo;
Servo edfServo; // EDF control using a servo object

bool parachuteDeployed = false;
float maxAltitude = 0; // Variable to store the maximum altitude reached relative to starting altitude
float altitudeThreshold = 3; // Altitude drop threshold to confirm descent

const float targetAltitude = 50; // Target altitude in meters, relative to starting altitude
const float reverseThrustAltitude = 90; // Altitude to activate reverse thrust, relative to starting altitude
const float groundLevelAltitude = 5; // Altitude to start EDF at full power upwards, relative to starting altitude
bool edfActivated = false; // Track if EDF has been activated

float startingAltitude = 0; // The altitude where the system was initialized
float estimatedAltitude = 0; // The initial estimate of altitude relative to starting altitude
float estimationError = 5; // Initial guess of the estimation error
float measurementNoise = 2; // How noisy we expect the measurements to be
float processNoise = 0.1; // The expected error in the prediction step
float kalmanGain = 0; // Initial Kalman gain

void kalmanUpdate(float currentAltitude) {

  currentAltitude -= startingAltitude;

  float predictedAltitude = estimatedAltitude;
  estimationError += processNoise;
  
  kalmanGain = estimationError / (estimationError + measurementNoise);
  estimatedAltitude = predictedAltitude + kalmanGain * (currentAltitude - predictedAltitude);
  estimationError = (1 - kalmanGain) * estimationError;

  // Log Kalman filter update data
  Serial.print("Kalman Gain: ");
  Serial.print(kalmanGain);
  Serial.print(", Estimated Altitude (relative): ");
  Serial.println(estimatedAltitude);
}

void setup() {
  Serial.begin(115200);
  
  parachuteServo.attach(6);
  edfServo.attach(9); 

  Serial.println("start edf boot");
  int speed = 0;
  for (speed = 1500; speed <= 2000; speed += 10) {
    edfServo.writeMicroseconds(speed);
    delay(5);
  }
  Serial.println("stop edf boot");

  parachuteServo.write(95); 

  if (!bmp.begin_SPI(BMP_CS)) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while(1);
  }

  bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setPressureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);

  // Perform an initial reading to set the starting altitude
  if (bmp.performReading()) {
    startingAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    Serial.print("Starting Altitude: ");
    Serial.println(startingAltitude);
  } else {
    Serial.println("Failed to perform initial reading");
    while(1);
  }

  edfServo.writeMicroseconds(1525);

  delay(2000);

  edfServo.writeMicroseconds(1450);
  delay(300);
  edfServo.writeMicroseconds(1525);
  delay(300);
  edfServo.writeMicroseconds(1450);
  delay(300);
  edfServo.writeMicroseconds(1525);

  Serial.println("Initialized");
}

void loop() {
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading");
    return;
  }

  float currentAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  kalmanUpdate(currentAltitude);

  Serial.print("Current Altitude (relative): ");
  Serial.print(currentAltitude - startingAltitude);
  Serial.print(", Max Altitude (relative): ");
  Serial.println(maxAltitude);

  if (estimatedAltitude > maxAltitude) {
    maxAltitude = estimatedAltitude;
  }

  controlEDF();

  if ((maxAltitude - estimatedAltitude > altitudeThreshold) && !parachuteDeployed) {
    deployParachute();
    Serial.println("Parachute Deployed!");
    edfServo.write(90); 
  }

  delay(1);
}

void deployParachute() {
  parachuteServo.write(150);
  parachuteDeployed = true;
  Serial.println("Deploying Parachute...");
}

void controlEDF() {
  if (!parachuteDeployed) {
    if (estimatedAltitude < groundLevelAltitude && !edfActivated && groundLevelAltitude > 5) {
      edfServo.writeMicroseconds(810);
      edfActivated = true;
      Serial.println("EDF Activated: Full Power Upwards");
    } 

    else if (estimatedAltitude >= targetAltitude - 5 && !edfActivated) {
      edfServo.writeMicroseconds(2120);
      edfActivated = true;
      Serial.println("EDF Activated: Full Power Downwards");
    }
  }

  if (parachuteDeployed) {
    edfServo.writeMicroseconds(1525);
    Serial.println("EDF Stopped");
  }
}
