signed int des_pos;
float des_stiffness;
#include "loadcell_read.h"
#include "motor_control.h"

//#include <Servo.h>

// Servo --------------------------------------
int servo_pin = 10;
//Servo stiffservo;
bool startStiffTest;
int servoPos = 4095;
int servoLimit = 4095;
int stage = 1;
int counter = 0;
int counter_threshold = 100;
int millis_threshold = 2000;
int start_delay = 2000;
int angleStep = 2048;
int t0 = 0;
int t1 = 0;
int t2 = 0;
bool start = false;
bool reverse = false;
bool restart = false;
int resetStiffness = 100000;



String  incomingSerialData;   // for incoming serial data
void setup() {

  Serial.begin(115200);
  Serial.setTimeout(10);
//  initialize_loadcell();
  initialize_motor();
  setup_rotary_encoder();
  unlock_motor();

  configure_motor();
//
//  stiffservo.attach(servo_pin);
//  stiffservo.write(servoPos);
}

void loop() {
  if (Serial.available() > 0)
  {
    incomingSerialData = Serial.readString();
    if (incomingSerialData.equals("S\n")){
      startStiffTest = true;
    }
    else{
      // Split the received string into two integer values using parseInt
      des_stiffness = incomingSerialData.toInt();
      
//      int spaceIndex = incomingSerialData.indexOf(' ');
//      
//      if (spaceIndex != -1) {
//        des_stiffness = incomingSerialData.substring(0, spaceIndex).toInt();
//        servoPos = incomingSerialData.substring(spaceIndex + 1).toInt();
//      }
    }
  }

  
  stiffnessMotorControl(des_stiffness);
  
  Serial.print(", Loadcell reading: ");Serial.print(calc_loadcell_output());
  Serial.print(", Servo position: ");Serial.println(servoPos);
//  printData();
  delay(100);

}
