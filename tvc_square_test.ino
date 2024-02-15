#include <Servo.h>

Servo servoX;
Servo servoY;

const int servoXPin = 6;
const int servoYPin = 7;

const int minX = 20;  // Minimum X position
const int maxX = 110; // Maximum X position
const int minY = 50;  // Minimum Y position
const int maxY = 140; // Maximum Y position

void setup() {
  servoX.attach(servoXPin);
  servoY.attach(servoYPin);

  servoX.write((minX + maxX) / 2);
  servoY.write((minY + maxY) / 2);
  delay(1000);
}

void loop() {
  for (int i = 0; i < 2; i++) {
    moveTo(minX, minY);
    delay(300); 

    moveTo(maxX, minY);
    delay(300); 

    moveTo(maxX, maxY);
    delay(300); 

    moveTo(minX, maxY);
    delay(300); 

    // Return to center before starting the pattern over
    moveTo((minX + maxX) / 2, (minY + maxY) / 2);
    delay(300); 
  }
  for (int i = 0; i < 5; i++) {
    moveTo(minX, minY);
    delay(50); 

    moveTo(maxX, minY);
    delay(50);

    moveTo(maxX, maxY);
    delay(50); 

    moveTo(minX, maxY);
    delay(50); 
  }
}

void moveTo(int x, int y) {
  x = constrain(x, minX, maxX);
  y = constrain(y, minY, maxY);

  servoX.write(x);
  servoY.write(y);
}
