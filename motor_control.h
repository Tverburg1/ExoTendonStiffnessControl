#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
extern signed int des_pos;

double pidControl(int currentInt, int referenceInt);
void initialize_motor();
void unlock_motor();
void lock_motor();
void motorControllerCallback();
void read_position();
void setup_rotary_encoder();
void configure_motor();
void stiffnessMotorControl(float des_stiffness);

#endif
