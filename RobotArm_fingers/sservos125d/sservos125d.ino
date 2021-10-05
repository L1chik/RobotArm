#include <Servo.h>
Servo servo[5];

void setup() {
  servo[0].attach(8);
  servo[0].write(0);
  servo[1].attach(9);
  servo[1].write(90);
  servo[2].attach(10);
  servo[2].write(90);
  servo[3].attach(11);
  servo[3].write(90);
  servo[4].attach(12);              
  servo[4].write(90);
}

void loop() {
  // put your main code here, to run repeatedly:

}
