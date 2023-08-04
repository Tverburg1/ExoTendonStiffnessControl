signed int des_pos;
float des_stiffness;
#include "loadcell_read.h"
#include "motor_control.h"
#include "stiffness_control.h"

#include <Servo.h>

// Servo --------------------------------------
int servo_pin = 10;
Servo stiffservo;
bool startStiffTest;
int servoPos = 180;
int stage = 0;
int counter = 0;
int counter_threshold = 100;
int millis_threshold = 5000;
int start_delay = 2000;
int angleStep = 90;
int t0 = 0;
int t1 = 0;
int t2 = 0;
bool start = false;
bool reverse = false;
int testStiffnesses[] = {200, 190, 180, 170, 160, 150, 140, 130, 120};


String  incomingSerialData;   // for incoming serial data
void setup() {

  Serial.begin(115200);
  Serial.setTimeout(10);
  initialize_loadcell();
  initialize_motor();
  setup_rotary_encoder();
  unlock_motor();

  configure_motor();

  stiffservo.attach(servo_pin);
  stiffservo.write(servoPos);
}

void loop() {
  if (Serial.available() > 0)
  {
    incomingSerialData = Serial.readString();
    if (incomingSerialData == "S\n"){
      startStiffTest = true;
    }
  }
//  read_position();
  stiffnessMotorControl(des_stiffness);
  Serial.print(", Loadcell reading: ");Serial.print(calc_loadcell_output());
  Serial.print(", Servo position: ");Serial.println(servoPos);
//  printData();
  delay(100);
  //Serial.print(calc_loadcell_output());

  if (startStiffTest) {
    if (t0 == 0){
      t0 = millis();
      stiffservo.write(servoPos);
    }
    
    if (true){
      if( stage > sizeof(testStiffnesses)){
        des_stiffness = 0;
        servoPos = 180;
        stiffservo.write(servoPos);
      }
      else{
        des_stiffness = testStiffnesses[stage];
        t1 = millis();
  
        if (t1-t0 >= start_delay && not start){
          start = true;
          t0 = millis();
        }
  
        servoPos = 180 - servoPosConstantForce(200, testStiffnesses[stage], 0.05);
        stiffservo.write(servoPos);
        
        
        if (t1-t0 >= millis_threshold && start){
            start = false;
            stage ++;
        }
      }
    }


    // if (stage == 1){
    //   des_stiffness = 75;
    //   t1 = millis();

    //   if (t1-t0 >= start_delay && not start){
    //     start = true;
    //     t0 = millis();
    //   }

    //   if (t1-t0 >= millis_threshold && start){


    //     if (reverse) {
    //       servoPos += angleStep;
    //       t0 = millis();
    //     }
    //     else{
    //       servoPos -= angleStep;
    //       t0 = millis();
    //     }

    //     stiffservo.write(servoPos);

    //     if (servoPos == 0){
    //       reverse = true;
    //     }
    //     else if (servoPos == 180){
    //       reverse = false;
    //       start = false;
    //       stage ++;
    //     }
    //   }
    // }

    
    
  } 
}
