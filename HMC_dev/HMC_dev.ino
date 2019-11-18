#include <Servo.h>

//pin numbers and values for each actuator
const int propServoPin_in = 12;
float propServoValue_in;
const int propServoPin_out = 7;
float propServoValue_out = 180;

const int propMotorPin_in = 11;
float propMotorValue_in;
const int propMotorPin_out = 5;
float propMotorValue_out = 0;

const int impMotorPin_in = 10;
float impMotorValue_in;
const int impMotorPin_out = 6;
float impMotorValue_out;

const int hippoServoPin_in = 9;
float hippoServoValue_in;
const int hippoServoPin_out = 8;  
float hippoServoValue_out;

const int dirPin_out = 3;
float dir;

Servo propServo;
Servo hippoServo;


void setup() {
  analogReference(EXTERNAL);
  
  pinMode(13,OUTPUT);
  pinMode(propServoPin_in, INPUT);
  pinMode(propServoPin_out, OUTPUT);

  pinMode(propMotorPin_in, INPUT);
  pinMode(propMotorPin_out, OUTPUT);

  pinMode(impMotorPin_in, INPUT);
  pinMode(impMotorPin_out, OUTPUT);

  pinMode(hippoServoPin_in, INPUT);
  pinMode(hippoServoPin_out, OUTPUT);


  propServo.attach(propServoPin_out);
  hippoServo.attach(hippoServoPin_out);
  Serial.begin(9600);
  
}

void loop() {
  digitalWrite(13, HIGH);

  propServoValue_in = readSensor(3, 2, propServoPin_in);
  propServoValue_out = -0.436*propServoValue_in + 1909.745;

  //control signal bounds
  if (propServoValue_out > 1500){
    propServoValue_out = 1500;
  }
  else if (propServoValue_out < 1140){
    propServoValue_out = 1140; 
  }
  //deadband
  if (abs(propServoValue_in - 1350) < 50){
    propServoValue_out = (1140+1500)/2;
  }




  hippoServoValue_in = readSensor(3, 2, hippoServoPin_in);
  Serial.print(hippoServoValue_in);
  Serial.print("      ");
  hippoServoValue_out = 1.994*hippoServoValue_in - 1159;

  //control signal bounds
  if (hippoServoValue_out > 2230){
    hippoServoValue_out = 2230;
  }
  else if (hippoServoValue_out < 615){
    hippoServoValue_out = 615;
  }

  //deadband around neutral
  if (abs(hippoServoValue_in - 1485) < 50){
    hippoServoValue_out = 1380;
  }






  propMotorValue_in = pulseIn(propMotorPin_in, HIGH);

  //direction logic
  if (propMotorValue_in > 1288){
    propMotorValue_out = 0.623*propMotorValue_in - 803.03;
    dir = LOW;
  }
  else{
    propMotorValue_out = -0.251*propMotorValue_in + 323.62;
    dir = HIGH;
  }

  //control signal bounds
  if (propMotorValue_out < 0){
    propMotorValue_out = 0;
  }
  else if (propMotorValue_out > 255){
    propMotorValue_out = 255;
  }

  //deadband around neutral
  if (abs(propMotorValue_in - 1288) < 30){
    propMotorValue_out = 0;
  }




  if (propMotorValue_in > 1288){
    impMotorValue_out = 0.6235*propMotorValue_in - 803.03;
  }
  else{
    impMotorValue_out = -0.6235*propMotorValue_in + 809.89;
  }
  
  
  //control signal bounds
  if (impMotorValue_out < 0){
    impMotorValue_out = 0;
  }
  else if (impMotorValue_out > 255){
    impMotorValue_out = 255;
  }

  //deadband around neutral
  if (abs(propMotorValue_in - 1288) < 30){
    impMotorValue_out = 0;
  }

  //Output
  digitalWrite(dirPin_out, dir);
  propServo.writeMicroseconds(propServoValue_out);
  hippoServo.writeMicroseconds(hippoServoValue_out);
  analogWrite(impMotorPin_out, impMotorValue_out);
  analogWrite(propMotorPin_out, propMotorValue_out);
  
    
  /*
  Serial.print("Prop Motor Input Value = ");
  Serial.print(propMotorValue_in);
  Serial.print("      ");
  Serial.print("Impeller Motor Input Value = ");
  Serial.print(impMotorValue_in);
  Serial.print("      ");
  Serial.print("Prop Servo Input Value = ");
  Serial.print(propServoValue_in);
  Serial.print("      ");
  Serial.print("Hippo Servo Input Value = ");
  Serial.print(hippoServoValue_in);
  Serial.print("      ");
  
  
  Serial.print('\n');
  */
  
  delay(10);
  
  digitalWrite(13, LOW);
  delay(10);
}


//Averages sensor readings over i samples spaced j ms apart
float readSensor(int i, int j, int pin){
  float sum = 0;
  for (int q = 0; q < i; q++)
  {
    sum += pulseIn(pin, HIGH);
    delay(j);
  }
  return sum/i;   
}
