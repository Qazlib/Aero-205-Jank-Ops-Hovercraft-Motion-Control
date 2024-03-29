#include <Adafruit_Sensor.h>

#include <Adafruit_L3GD20_U.h>
#include <Adafruit_L3GD20.h>

#include <Servo.h>
#include <Wire.h>

//program clock (ms)
const float ts = 20.0;
unsigned long current_time = millis();
unsigned long time_last_event  =  millis();


//pin numbers and values for each actuator
const int propServoPin_in = 12;
float propServoValue_in;
const int propServoPin_out = 7;
float propServoValue_out = 180;

const int propMotorPin_in = 11;
float propMotorValue_in;
const int propMotorPin_out = 5;
float propMotorValue_out = 0;

const int impMotorPin_out = 6;
float impMotorValue_out;

const int hippoServoPin_in = 10;
float hippoServoValue_in;
const int hippoServoPin_out = 8;  
float hippoServoValue_out;

const int dirPin_out = 3;
float dir;


Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);


//Yaw Rate (yr) controller variables
float yr_targ;
float yr;
float yr_err;
float yr_err_k1;
float yr_err_total;
bool yrc_active;
bool yrc_active_k1;
int yrc_int_active;
int yrc_int_active_k1;

float yr_limit = 0.5;
bool yr_limit_active;

float kp = -150;
float ki = -1150;
float kd = 0;

bool led_on = true;


Servo propServo;
Servo hippoServo;


void setup() {
  Serial.begin(9600);
  analogReference(EXTERNAL);
  gyro.enableAutoRange(true);

  //initialize sensor
    if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
  
  sensor_t sensor;
  gyro.getSensor(&sensor);
  
  pinMode(13,OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(propServoPin_in, INPUT);
  pinMode(propServoPin_out, OUTPUT);

  pinMode(propMotorPin_in, INPUT);
  pinMode(propMotorPin_out, OUTPUT);

  pinMode(impMotorPin_out, OUTPUT);

  pinMode(hippoServoPin_in, INPUT);
  pinMode(hippoServoPin_out, OUTPUT);


  propServo.attach(propServoPin_out);
  hippoServo.attach(hippoServoPin_out);

  
  
}

void loop() {
  //side LEDs
  digitalWrite(4, HIGH);


  //cycle main LED at 1 Hz
  current_time = millis();
  if (abs(current_time - time_last_event) > 1000){
    if (led_on == true){
      led_on = false;
    }
    else{
      led_on = true;
    }
  }

  if (led_on == true){
    digitalWrite(13, HIGH);
  }
  else{
    digitalWrite(13, LOW);
  }

  /* Get a new sensor event */ 
  sensors_event_t event; 
  gyro.getEvent(&event);

  /* Display the results (speed is measured in rad/s) */

  yr = event.gyro.z;
  yr = yr-0.01;

  //deadband around zero yaw rate
  if (abs(yr) < 0.02){
    yr = 0;
  }
  Serial.print(yr);
  Serial.print("\n");




  propMotorValue_in = pulseIn(propMotorPin_in, HIGH);

  //direction logic
  if (propMotorValue_in > 1288){
    //forwards
    propMotorValue_out = 0.623*propMotorValue_in - 803.03;
    dir = HIGH;
  }
  else{
    //backwards
    propMotorValue_out = -0.427*propMotorValue_in +550.15;
    dir = LOW;
  }

  //control signal bounds
  if (propMotorValue_out < 0){
    propMotorValue_out = 0;
  }
  else if (propMotorValue_out > 255){
    propMotorValue_out = 255;
  }

  //deadband around neutral
  if (abs(propMotorValue_in - 1288) < 50){
    propMotorValue_out = 0;
  }





  //-----------------------------------------------------------------B I G   B O I---------------------------------------------------------------


  propServoValue_in = readSensor(3, 2, propServoPin_in);
  
  if (propServoValue_in > 1288){
    yr_targ = 0.0009*propServoValue_in - 1.189;
  }
  else{
    yr_targ = 0.001049869*propServoValue_in - 1.38583;
  }
  
  //deadband
  if (abs(propServoValue_in - 1288) < 80){
    yr_targ = 0;
  }

  //yaw rate request limits
  if (yr_targ > 0.4){
    yr_targ = 0.4;
  }
  else if (yr_targ < -0.4){
    yr_targ = -0.4;
  }


  //activate yaw rate control when trying to go straight (steady forward motion or countersteer maneuver)
  yr_limit_active = false;
  if (yr_targ == 0){
    yrc_active = true;

    //activate integral term if in steady forward motion
    if (abs(yr) < 0.28){
      yrc_int_active = 1;
    }
    else{
      yrc_int_active = 0;
      yr_err_total = 0;
    }

    //brake if performing severe countersteer movement (see impeller motor control logic)
    Serial.print(abs(yr));
    Serial.print("      ");
    Serial.print(yr_limit);
    Serial.print("     ");
    if (abs(yr) > yr_limit){
      yr_limit_active = true;
    }

      
    //reset integral if initializing steady straight motion
    if (yrc_int_active_k1 == 0 && yrc_int_active == 1){
      yr_err_total = 0;
    }



    //compute  errors
    yr_err = yr - yr_targ;
    yr_err_total += yr_err * ts/1000;



    //Output signal
    //direction logic
    if (dir == LOW){
      //moving forwards
      propServoValue_out = 1320 + kp*yr_err + yrc_int_active*ki*yr_err_total + kd*(yr_err - yr_err_k1)/(ts/1000);
    }
    else{
      //moving backwards
      propServoValue_out = 1320 - kp*yr_err - yrc_int_active*ki*yr_err_total - kd*(yr_err - yr_err_k1)/(ts/1000);

    }

    
  }
  
  //deactivate yrc if user requests a turn
  else {
 
    yrc_active = false;
    if (propServoValue_in > 1288){
    propServoValue_out = 0.6573*propServoValue_in + 440.563;
    }
    else{
      propServoValue_out = 0.602*propServoValue_in + 574.716;
    }
    
    //control signal bounds
    if (propServoValue_out > 1600){
      propServoValue_out = 1600;
    }
    else if (propServoValue_out < 1140){
      propServoValue_out = 1140; 
    }
    
  }


    //control signal bounds
    if (propServoValue_out > 2330){
      propServoValue_out = 2330;
    }
    else if (propServoValue_out < 650){
      propServoValue_out = 650; 
    }

    //deadband not needed here since yrc is used if trying to fly straight


  //update variables
  yr_err_k1 = yr_err;
  yrc_active_k1 = yrc_active;
  yrc_int_active_k1 = yrc_int_active;

  


//--------------------------------------------------------------------E N D   B I G   B O I------------------------------------------------------------------




  if (propMotorValue_in > 1288){
    impMotorValue_out = 0.6235*propMotorValue_in - 803.03;
  }
  else{
    impMotorValue_out = -0.6235*propMotorValue_in + 809.89;
  }

    
  //brake if countersteering
  if (yr_limit_active == true){
    impMotorValue_out = -212.5*abs(yr) + 255;
  }
  
  //control signal bounds
  if (impMotorValue_out < 0){
    impMotorValue_out = 0;
  }
  else if (impMotorValue_out > 255){
    impMotorValue_out = 255;
  }

  

  //deadband around neutral
  if (abs(propMotorValue_in - 1288) < 50){
    impMotorValue_out = 0;
  }




  hippoServoValue_in = readSensor(3,2,hippoServoPin_in);
 
  if (hippoServoValue_in > 1700){
    hippoServoValue_out = 2125;
  }
  else if (hippoServoValue_in < 1200){
    hippoServoValue_out = 590;
  }
  else{
    hippoServoValue_out = 2050;
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
  
  delay(ts);
  digitalWrite(4, LOW);
}

//Moving average filter
//i: number of samples
//j: sample period samples (ms)
//pin: input pin number
float readSensor(int i, int j, int pin){
  float sum = 0;
  for (int q = 0; q < i; q++)
  {
    sum += pulseIn(pin, HIGH);
    delay(j);
  }
  return sum/i;   
}
