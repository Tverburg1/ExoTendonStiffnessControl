#include "loadcell_read.h"

#include "HX711.h"

#define calibration_factor -890.0 //This value is obtained using the SparkFun_HX711_Calibration sketch

#define DOUT  5
#define CLK  4

HX711 scale;

float output_force = 0;


void initialize_loadcell() {
  scale.begin(DOUT, CLK);
  scale.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale.tare(); //Assuming there is no weight on the scale at start up, reset the scale to 0
}

float calc_loadcell_output() {
  output_force = scale.get_units()*0.00981; //scale.get_units() returns a float
  return output_force;
}
