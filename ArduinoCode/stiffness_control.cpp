#include <math.h>

int encoderResolution = 4096;
int springstiffness = 144;

float stabiliserArmDiameter     = 0.068;  // m
float pulleyDiameter            = 0.0313; // m
float wormGearTransmissionRatio = 90;   	//factor

float finalTranmission = (stabiliserArmDiameter/pulleyDiameter);//*wormGearTransmissionRatio;

float targetMotorPosition;


int stiffnessToMotorPosition(float targetStiffness){
  float targetAngle = atan2(sqrt(targetStiffness), sqrt(4*springstiffness)); // In degrees
  int targetMotorPosition = encoderResolution*(targetAngle*finalTranmission/(2*M_PI));
  return targetMotorPosition;
}

float motorPositionToStiffness(int motorPosition){
  float actualAngle = 2*M_PI*motorPosition/(encoderResolution*finalTranmission);
  float actualStiffness = tan(2*actualAngle)*springstiffness;
  return actualStiffness;
}

int servoPosConstantForce(int k_1, int k_2, float x){
  float theta1 = atan2(sqrt(k_1), sqrt(4*springstiffness)); 
  float theta2 = atan2(sqrt(k_2), sqrt(4*springstiffness));

  
  float dx = (x*k_1/k_2) - x - 0.0065*(sin(theta1)-sin(theta2));
  float servoDegrees = dx/(M_PI*0.02);
  int servoSteps = 4095*servoDegrees;
  return servoSteps;
}
