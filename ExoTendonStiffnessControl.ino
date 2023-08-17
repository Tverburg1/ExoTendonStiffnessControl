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
int servoPos = 4095;
int servoLimit = 4095;
int stage = 0;
int counter = 0;
int counter_threshold = 100;
int millis_threshold = 4000;
int start_delay = 4000;
int angleStep = 2048;
int t0 = 0;
int t1 = 0;
int t2 = 0;
bool start = false;
bool reverse = false;
bool restart = false;
int testStiffnesses[] = {400, 375, 350, 325, 300, 275, 250, 225, 200, 175, 150};


String  incomingSerialData;   // for incoming serial data
void setup() {

  Serial.begin(115200);
//  Serial.setTimeout(10);
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
    }
    
    if (true){
      if( stage == 11){
        des_stiffness = 0;
        servoPos = 4095;
      }
      else{
        des_stiffness = testStiffnesses[stage];
        servoPos = 4095 - servoPosConstantForce(400, testStiffnesses[stage], 0.03);
        t1 = millis();
  
        if (t1-t0 >= start_delay && not start){
          start = true;
          t0 = millis();
        }
        
        if (t1-t0 >= millis_threshold && start){
            start = false;
            stage ++;
        }
      }
    }    
  } 
}
