#ifndef STIFFNESS_CONTROL_H
#define STIFFNESS_CONTROL_H

#include <Arduino.h>

int stiffnessToMotorPosition(float targetStiffness);
float motorPositionToStiffness(int motorPosition);

#endif