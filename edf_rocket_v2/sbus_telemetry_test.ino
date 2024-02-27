#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP3XX.h>
#include <TinyGPS++.h>
#include <Servo.h> 
#include <SBUS.h>

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
Adafruit_BMP3XX bmp;

#define SEALEVELPRESSURE_HPA (1013.25)
#define TARGET_ALTITUDE_METERS (800 * 0.3048)
#define EDF_REVERSE_MAX 90 

Servo esc;
float previousAltitude = 0;
float currentVelocity = 0;
unsigned long previousMillis = 0;
bool isEDFFiring = false;

SBUS x8r(Serial2);

HardwareSerial GPS_Serial(1);
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 4800;
TinyGPSPlus gps;

Servo esc;
Servo parachuteServo;

float previousAltitude = 0;
float currentVelocity = 0;
float previousVelocity = 0;
bool parachuteDeployed = false;

void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

  esc.attach(9);
  parachuteServo.attach(4);

  if(!mag.begin()) {
    Serial.println("Could not find a valid HMC5883 sensor");
    while(1);
  }

  if (!bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {
    Serial.println("Could not find a valid BMP3 sensor");
    while(1);
  }

  bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setPressureOversampling(BMP3_NO_OVERSAMPLING);

  bmp.setOutputDataRate(BMP3_ODR_200_HZ);

  parachuteServo.write(0);

  previousMillis = millis();

  x8r.begin();

  Serial.println("Intialized !! :3");
}


void loop() {
  if (x8r.read(&channels[0], &failSafe, &lostFrame)) {
    int edfValue = map(channels[0], 172, 1811, 0, 180);

    esc.write(edfValue);
  }
}


void adjustEDFThrust(float currentAltitude, float futureAltitude, float currentAcceleration) {
    float altitudeError = TARGET_ALTITUDE_METERS - futureAltitude;
    int thrustSetting;

    if (currentAltitude + altitudeError < TARGET_ALTITUDE_METERS && !isEDFFiring) {
        isEDFFiring = true;
        thrustSetting = EDF_REVERSE_MAX;
    } else if (isEDFFiring) {
        thrustSetting = calculateDynamicThrustSetting(altitudeError, currentAcceleration);
    } else {
        thrustSetting = 0;
    }

    esc.write(thrustSetting);
}

int calculateDynamicThrustSetting(float altitudeError, float currentAcceleration) {
    int thrustSetting = map(abs(altitudeError), 0, 10, 0, EDF_REVERSE_MAX);
    thrustSetting = constrain(thrustSetting, 0, EDF_REVERSE_MAX);
    return thrustSetting;
}

void controlEDF(int speed) {
  int reverseMaxBound = 0;
  int stopBound = 90;
  int forwardMaxBound = 180;

  int servoPosition;
  if (speed <= 0) {
    servoPosition = map(speed, -100, 0, reverseMaxBound, stopBound);
  } else {
    servoPosition = map(speed, 0, 100, stopBound, forwardMaxBound);
  }
  servoPosition = constrain(servoPosition, 0, 180);
  esc.write(servoPosition);
}


void deployParachute() {
  Serial.println("Deploying parachute!");
  parachuteServo.write(90);
  parachuteDeployed = true;
}
