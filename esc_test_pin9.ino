#include <Servo.h>

Servo esc; // Create a servo object to control the ESC

void setup() {
  esc.attach(9); // Attach the ESC signal cable to pin 9
}

void loop() {
  int speed = 1500; // Set the speed (microseconds). 1500 is usually the midpoint (stop for a bidirectional ESC).
  // esc.writeMicroseconds(speed); // Send the speed to the ESC
  
  // Example: Increase speed gradually
  for (speed = 1500; speed <= 2000; speed += 10) {
    esc.writeMicroseconds(speed);
    delay(20);
  }
  
  delay(5000);

  // // Example: Decrease speed gradually
  // for (speed = 2000; speed >= 1000; speed -= 10) {
  //   esc.writeMicroseconds(speed);
  //   delay(20);
  // }
}
