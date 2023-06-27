signed int des_pos;
float des_stiffness;
#include "loadcell_read.h"
#include "motor_control.h"

#include <Servo.h>

// Servo --------------------------------------
int servo_pin = 10;
Servo stiffservo;
bool startStiffTest;
int servoPos = 180;
int stage = 0;
int counter = 0;
int counter_threshold = 100;
int millis_threshold = 10000;
int start_delay = 2000;
int t0 = 0;
int t1 = 0;
int t2 = 0;
bool start = false;
bool reverse = false;



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
    }


    if (stage == 0){
      des_stiffness = 50;
      t1 = millis();

      if (t1-t0 >= start_delay && not start){
        start = true;
        t0 = millis();
      }

      if (t1-t0 >= millis_threshold && start){


        if (reverse) {
          servoPos += 90;
          t0 = millis();
        }
        else{
          servoPos -= 90;
          t0 = millis();
        }

        stiffservo.write(servoPos);

        if (servoPos == 0){
          reverse = true;
        }
        else if (servoPos == 180){
          reverse = false;
          start = false;
          stage ++;
        }
      }
    }

    if (stage == 1){
      des_stiffness = 75;
      t1 = millis();

      if (t1-t0 >= start_delay && not start){
        start = true;
        t0 = millis();
      }

      if (t1-t0 >= millis_threshold && start){


        if (reverse) {
          servoPos += 90;
          t0 = millis();
        }
        else{
          servoPos -= 90;
          t0 = millis();
        }

        stiffservo.write(servoPos);

        if (servoPos == 0){
          reverse = true;
        }
        else if (servoPos == 180){
          reverse = false;
          start = false;
          stage ++;
        }
      }
    }

    if (stage == 2){
      des_stiffness = 100;
      t1 = millis();

      if (t1-t0 >= start_delay && not start){
        start = true;
        t0 = millis();
      }

      if (t1-t0 >= millis_threshold && start){


        if (reverse) {
          servoPos += 90;
          t0 = millis();
        }
        else{
          servoPos -= 90;
          t0 = millis();
        }

        stiffservo.write(servoPos);

        if (servoPos == 0){
          reverse = true;
        }
        else if (servoPos == 180){
          reverse = false;
          start = false;
          stage ++;
        }
      }
    }

    if (stage == 3){
      des_stiffness = 125;
      t1 = millis();

      if (t1-t0 >= start_delay && not start){
        start = true;
        t0 = millis();
      }

      if (t1-t0 >= millis_threshold && start){


        if (reverse) {
          servoPos += 90;
          t0 = millis();
        }
        else{
          servoPos -= 90;
          t0 = millis();
        }

        stiffservo.write(servoPos);

        if (servoPos == 0){
          reverse = true;
        }
        else if (servoPos == 180){
          reverse = false;
          start = false;
          stage ++;
        }
      }
    }

    if (stage == 4){
      des_stiffness = 150;
      t1 = millis();

      if (t1-t0 >= start_delay && not start){
        start = true;
        t0 = millis();
      }

      if (t1-t0 >= millis_threshold && start){


        if (reverse) {
          servoPos += 90;
          t0 = millis();
        }
        else{
          servoPos -= 90;
          t0 = millis();
        }

        stiffservo.write(servoPos);

        if (servoPos == 0){
          reverse = true;
        }
        else if (servoPos == 180){
          reverse = false;
          start = false;
          stage ++;
        }
      }
    }

    if (stage == 5){
      des_stiffness = 175;
      t1 = millis();

      if (t1-t0 >= start_delay && not start){
        start = true;
        t0 = millis();
      }

      if (t1-t0 >= millis_threshold && start){


        if (reverse) {
          servoPos += 90;
          t0 = millis();
        }
        else{
          servoPos -= 90;
          t0 = millis();
        }

        stiffservo.write(servoPos);

        if (servoPos == 0){
          reverse = true;
        }
        else if (servoPos == 180){
          reverse = false;
          start = false;
          stage ++;
        }
      }
    }
    
    if (stage == 6){
      des_stiffness = 200;
      t1 = millis();

      if (t1-t0 >= start_delay && not start){
        start = true;
        t0 = millis();
      }

      if (t1-t0 >= millis_threshold && start){


        if (reverse) {
          servoPos += 90;
          t0 = millis();
        }
        else{
          servoPos -= 90;
          t0 = millis();
        }

        stiffservo.write(servoPos);

        if (servoPos == 0){
          reverse = true;
        }
        else if (servoPos == 180){
          reverse = false;
          start = false;
          stage ++;
        }
      }
    }
    
  } 
}



