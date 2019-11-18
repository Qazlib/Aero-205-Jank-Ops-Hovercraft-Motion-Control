#include <Servo.h>
const int pin = 7;
float val = 1200;
Servo servo;

void setup() {
  analogReference(EXTERNAL);

  servo.attach(7);
  Serial.begin(9600);
}

void loop() {
  /*
  for (int i = 0; val < 1500; i++){
     servo.writeMicroseconds(val);
     val += 20;
     Serial.print(val);
     Serial.print('\n');
     delay(300);
  }
  */
  
  servo.writeMicroseconds(1500);
  delay(1);
  
}
