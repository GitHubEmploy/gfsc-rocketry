#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP3XX.h>
#include <TinyGPS++.h>
#include <ESP32Servo.h>
#include <Arduino_Constrained_MPC.h>

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

const float deltaT = 1.0; 
const float mass = 0.65;
const float g = 9.81; 
const float umaxThrust = 81.0;
const int N_STATES = 2; // Number of states: altitude and velocity
const int N_CONTROLS = 1; // Number of control inputs
const int N_OUTPUTS = 2; // Number of outputs
const int HORIZON = 10; // Prediction horizon

float A[N_STATES * N_STATES] = {1, deltaT, 0, 1};
float B[N_STATES * N_CONTROLS] = {0, deltaT / mass};
float C[N_OUTPUTS * N_STATES] = {1, 0, 0, 1};
float Q[N_OUTPUTS * N_OUTPUTS] = {10, 0, 0, 1}; 
float R[N_CONTROLS * N_CONTROLS] = {0.01};

float U_min[N_CONTROLS] = {0}; 
float U_max[N_CONTROLS] = {umaxThrust}; 

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

  mpc = KCMPC<N_STATES, N_CONTROLS, N_OUTPUTS, HORIZON>(A, B, C, Q, R, U_min, U_max);

  Serial.println("Intialized !! :3");

  previousMillis = millis();
}

void loop() {
  if (!parachuteDeployed) {
    unsigned long currentMillis = millis();
    float deltaTime = (currentMillis - previousMillis) / 1000.0;
    if (!bmp.performReading()) {
      Serial.println("Failed to perform reading");
      return;
    }

    float currentAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    if (deltaTime > 0) {
      currentVelocity = (currentAltitude - previousAltitude) / deltaTime;
    }

    float x[N_STATES] = {currentAltitude, currentVelocity};
    float yref[N_OUTPUTS * HORIZON];
    for (int i = 0; i < HORIZON; ++i) {
      yref[i * N_OUTPUTS] = TARGET_ALTITUDE_METERS;
      yref[i * N_OUTPUTS + 1] = 0;
    }

    float u[N_CONTROLS];
    bool mpcSuccess = mpc.solve(x, yref, u);

    if (mpcSuccess) {
      int escSignal = map(u[0], U_min[0], U_max[0], 0, EDF_REVERSE_MAX);
      esc.write(escSignal);
    } else {
      Serial.println("i hate mpc this is where u give up");
    }

    previousAltitude = currentAltitude;
    previousMillis = currentMillis;

    if (!bmp.performReading()) {
      Serial.println("Failed to perform reading :(");
      return;
    }
    Serial.print("Approx. Altitude = ");
    Serial.print(currentAltitude);
    Serial.println(" m");

    if (currentAltitude < previousAltitude && !parachuteDeployed) {
      deployParachute();
    }

    previousAltitude = currentAltitude;
  }

  sensors_event_t event;
  mag.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  float declinationAngle = 0.22;
  heading += declinationAngle;
  if(heading < 0)
    heading += 2*PI;
  if(heading > 2*PI)
    heading -= 2*PI;
  float headingDegrees = heading * 180/M_PI;
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
    while (GPS_Serial.available() > 0)
    if (gps.encode(GPS_Serial.read())) {
      if (gps.location.isValid()) {
        Serial.print("Latitude = "); Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude = "); Serial.println(gps.location.lng(), 6);
        Serial.print("Satellites = "); Serial.println(gps.satellites.value());
      } else {
        Serial.println("GPS signal not found");
      }
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
