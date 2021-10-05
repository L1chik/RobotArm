#include <Servo.h>

Servo servo[4];

void setup() {
  servo[0].attach(8);
  servo[0].write(0);
  servo[1].attach(10);
  servo[1].write(90);
  servo[2].attach(11);
  servo[2].write(180);
  servo[3].attach(13);

}

void loop() {
  for (int i=0; i<=180; i++){
    servo[3].write(i);
    delay(10);
  }
  delay(2000);

  for (int i=180; i>=0; i--){
    servo[3].write(i);
    delay(10);
  }
  delay(2000);
}
