#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP3XX.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// HMC5883 Compass Setup
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// BMP3XX Sensor Setup
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
Adafruit_BMP3XX bmp;

#define SEALEVELPRESSURE_HPA (1013.25)

// TinyGPS++ and SoftwareSerial Setup
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 4800;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(115200);

  // Compass initialization
  if(!mag.begin()) {
    Serial.println("Could not find a valid HMC5883 sensor, check wiring!");
    while(1);
  }

  // BMP388/390 initialization
  if (!bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) { // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while(1);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // GPS Module initialization
  ss.begin(GPSBaud);
}

void loop() {

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

  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");


  while (ss.available() > 0)
    gps.encode(ss.read());
  if (gps.location.isValid()) {
    Serial.print("Latitude = "); Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude = "); Serial.println(gps.location.lng(), 6);
    Serial.print("Satellites = "); Serial.println(gps.satellites.value());
  } else {
    Serial.println("GPS signal not found");
  }

  delay(2000); 
}
