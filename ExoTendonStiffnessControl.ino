signed int des_pos;
float des_stiffness;
#include "loadcell_read.h"
#include "motor_control.h"




String  incomingSerialData;   // for incoming serial data
void setup() {

  Serial.begin(115200);
  Serial.setTimeout(10);
  initialize_loadcell();
  initialize_motor();
  setup_rotary_encoder();
  unlock_motor();

  configure_motor();
}

void loop() {
  if (Serial.available() > 0)
  {
    incomingSerialData = Serial.readString();
    // des_pos = incomingSerialData.toInt();
    des_stiffness = incomingSerialData.toInt();
  }
//  read_position();
  stiffnessMotorControl(des_stiffness);
  Serial.print(", Loadcell reading: ");Serial.println(calc_loadcell_output());
//  printData();
  delay(100);
  //Serial.print(calc_loadcell_output());

}


