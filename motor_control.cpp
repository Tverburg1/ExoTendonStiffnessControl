#include "motor_control.h"
#include <iostream>
#include <cmath>


#include <AS5600.h>
#include "stiffness_control.h"


//Motor stuff ------------------------------------
AS5600 encoder;
signed int pos = 0;
int prev_angle = 0;
bool motor_initalized = false;
bool motor_locked = false;
double control_output = 0;
int prev_des_pos = 0;
double prev_position = 0;
int stop_counter = 0;

int motor_pin = 11;
int dir_pin = 7;
int enable_pin = 12;

double max_extension = 0.3; // m
int max_extension_steps = max_extension * 4096 / (PI * 0.04);

int init_angle = 0;

int cycle_counter = 0;

int zero_slack_counter = 0;

// For oscilation ------------------------------
double t_start = 0;
double t_current = 0;

// PID controller parameters -------------------
const double kP = 10.00;
const double kI = 10.0;
const double kD = 0.00;

// Current and previous error values ----------
double prevError = 0.0;
double integral = 0.0;

// Time step (in seconds) ---------------------
const double dt = 0.001;

double last_print = 0;
double last_stop_time = 0;



double pidControl(int currentInt, int referenceInt) {

  double current = static_cast<double>(currentInt);
  double reference = static_cast<double>(referenceInt);
  // Calculate error
  double error = reference - current;

  // Proportional term
  double Pout = kP * error;

  // Integral term
  integral += error * dt;

  
  if (abs(integral) > 250){
    if (integral > 0){
      integral = 250;
    }
    else {
      integral = -250;
    }
  }

  double Iout = kI * integral;
  // Serial.println(Iout);

  // Derivative term
  double derivative = abs(pos - prev_position) / dt;
  double Dout = kD * derivative;

  // Calculate total output
  double output = Pout + Iout + Dout;

  // Apply saturation limits
  double maxOutput = 1500;
  double minOutput = -1500;
  if (output > maxOutput) output = maxOutput;
  if (output < minOutput) output = minOutput;

  if (abs(output) < 1)
  {
    output = 0;
  }

  // Save error to previous error
  prevError = error;

  return output;
}

void initialize_motor() {

  pinMode(motor_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(enable_pin, OUTPUT);

  // Lock motor, set direction to low, and set output to 0
//  digitalWrite(enable_pin, HIGH);
  digitalWrite(enable_pin, LOW);
  digitalWrite(dir_pin, LOW);
  analogWrite(motor_pin, 0);

  motor_initalized = true;
  motor_locked = true;
  init_angle = encoder.readAngle();
  prev_angle = init_angle;

}

void lock_motor() {
  if (motor_initalized) {
    if (motor_locked) {
      Serial.println("Motor is already locked");
    }
    else {
      digitalWrite(enable_pin, LOW);
      motor_locked = true;
    }
  }
  else {
    Serial.println("Motor not initialized");
  }
}

void unlock_motor() {
  if (motor_initalized) {
    if (not motor_locked) {
      Serial.println("Motor is already unlocked");
    }
    else {
      digitalWrite(enable_pin, HIGH);
      motor_locked = false;
    }
  }
  else {
    Serial.println("Motor not initialized");
  }
}

void motorControllerCallback() {
  if (motor_initalized) {
    if (not motor_locked) {
      read_position();
      control_output = pidControl(pos, des_pos);
      int output_value = 0;

      if (control_output < 0) {
        output_value  = -1 * 255 * control_output / 4096;
        digitalWrite(dir_pin, LOW);
      }
      else if (control_output >= 0) {
        output_value  = 255 * control_output / 4096;
        digitalWrite(dir_pin, HIGH);
      }

      if (abs(control_output) < 5){
        control_output = 0;
      }

      if (abs(pos-prev_position) < 20){
        if (last_stop_time == 0){
          last_stop_time = millis();
        }
          if (millis() - last_stop_time > 4000){
            last_stop_time = millis();
            analogWrite(motor_pin, 0);
            // Serial.println("RESET ---------------------------------------------------------------------------------");
            output_value = 0;

            if (control_output < 0) {
              digitalWrite(dir_pin, HIGH);
              delay(10);
              digitalWrite(dir_pin, LOW);
            }
            else if (control_output >= 0) {
              digitalWrite(dir_pin, LOW);
              delay(10);
              digitalWrite(dir_pin, HIGH);
          }
        }
      }
      else{
        last_stop_time = 0;
      }
      // Serial.print("M Position: "); Serial.print(pos);
      // Serial.print(", M Desired PosPosition: "); Serial.print(des_pos);
      // Serial.print(", M Output value: "); Serial.println(output_value);
      analogWrite(motor_pin, output_value);
    // }
    //  }

     prev_des_pos = des_pos;
     prev_position = pos;
     
    }
    else {
      Serial.println("Motor is locked");
    }
  }
  else {
    Serial.println("Motor not initialized");
  }

  // digitalWrite(dir_pin, HIGH);
  // analogWrite(motor_pin, 50);

  
}

void setup_rotary_encoder() {
  encoder.begin(9);
  encoder.setDirection(AS5600_CLOCK_WISE);
  int b = encoder.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);

  des_pos = init_angle;
  pos = init_angle;
  prev_angle = init_angle;
}

void read_position() {

  int angle = encoder.readAngle();
  signed int d_angle = angle - prev_angle;

  if ( d_angle >= 300 )
  {
    cycle_counter -= 1;

  }
  else if ( d_angle <= -300)
  {
    cycle_counter += 1;
  }

  pos = 4096 * cycle_counter + angle;

  // int control_ouput = pidControl(pos, des_pos);
  // Serial.print(", Position: "); Serial.println(pos);
  //  Serial.print(", Position: "); Serial.print(pos);
  //  Serial.print(", Angle: "); Serial.print(angle);
  //  Serial.print(", Difference: "); Serial.print(d_angle);
  //  Serial.print(", Desired pos: ");Serial.print(des_pos);
  //  Serial.print(", Control output: ");Serial.print(control_output);
  //  Serial.print(", Velocity: ");Serial.println(encoder.getAngularSpeed());
  prev_angle = angle;
}

void oscilatate_des_input () {
  if (t_start == 0) {
    t_start = millis();
  }

  t_current = millis();

  if (t_current - t_start > 5000) {
    if (des_pos > init_angle) {
      des_pos = init_angle + 500;
    }
  }

}

void configure_motor() {
  bool motorConfigured = false;
  String  input;
  String compareString = "V\n";


  Serial.println("Configuring motor, tune the initial position by sending a position. When finished send and V to confirm the configuration and enable control");
  while (not motorConfigured)
  {
    Serial.println("Waiting for configuration");
    if (Serial.available() > 0)
    {
      input = Serial.readString();

      for (int i = 0; i < input.length(); i++) {
        if (!isDigit(input[i])) {

          if (input.equals(compareString)) {
            motorConfigured = true;
            init_angle = encoder.readAngle();
            des_pos = encoder.readAngle();
            cycle_counter = 0;
            Serial.println("Motor configured");
          }
        }
        else {
          des_pos = input.toInt();
        }
      }
    }

    read_position();
    motorControllerCallback();
    Serial.print("Desired pos: "); Serial.print(des_pos);
    Serial.print(", Position: "); Serial.print(pos);
    Serial.print(", Control output: "); Serial.println(pidControl(pos, des_pos));
    delay(10);
//
//    Serial.print(", Position: "); Serial.print(pos);
//    Serial.print(", Desired pos: ");Serial.print(des_pos);
//    Serial.print(", Control output: ");Serial.print(control_output);
//    Serial.print(", Velocity: ");Serial.println(encoder.getAngularSpeed());
  }
}

void stiffnessMotorControl(float des_stiffness){
  int targetPositionOffset = stiffnessToMotorPosition(des_stiffness);
  des_pos = init_angle - targetPositionOffset;
  Serial.print("Time: "); Serial.print(millis());
  Serial.print(", Desired stiffness: "); Serial.print(des_stiffness);
  Serial.print(", Actual Stiffness: "); Serial.print(motorPositionToStiffness(init_angle-pos));
  Serial.print(", Init angle: "); Serial.print(init_angle);
  Serial.print(", Desired position: "); Serial.print(des_pos);
  Serial.print(", Actual position: "); Serial.print(pos);
  Serial.print(", Position offset: "); Serial.print(targetPositionOffset);
  Serial.print(", Control output: "); Serial.print(pidControl(pos, des_pos));

  motorControllerCallback();
}