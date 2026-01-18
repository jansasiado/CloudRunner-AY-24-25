#include "Arduino.h"

#ifndef CLOUDRUNNER_CONSTANTS_H
#define CLOUDRUNNER_CONSTANTS_H

//GPIO assignment
#define L_SPEED_PIN 10 //D10
#define R_SPEED_PIN 9 //D9
#define L_FORWARD A1  //A0
#define L_BACKWARD A0 //A1
#define R_FORWARD A2 //A2
#define R_BACKWARD A3 //A3        

//sensor GPIOs
#define SENSOR_5_0 2
#define SENSOR_5_1 4
#define SENSOR_5_2 5
#define SENSOR_5_3 7
#define SENSOR_5_4 8

#define SENSOR_3_0 SENSOR_5_1
#define SENSOR_3_1 SENSOR_5_2
#define SENSOR_3_2 SENSOR_5_3


//Constants for driveMotor functions
#define STOP_MOTORS 0
#define NO_CHANGE 255
#define FORWARD_MOTORS 1
#define REVERSE_MOTORS 2
#define TURN_LEFT  21
#define TURN_RIGHT 22
#define TURN_BACK 23     


//Constants for sensor functions/routines
#define SENSOR_NUM 5        //Number of PID sensors  # set either to 3 or 5
#define SENSOR_BASELINE_FACTOR 2
#define NOISE_THRESHOLD 500
#define NEG_NOISE_THRESHOLD 500
#define LOW_BASELINE_RESET 3

#define BLACK_THRESHOLD_MARGIN 2000

#define L_TURN_PIN 8        //Left turn sensor pin
#define R_TURN_PIN 2        //Right turn sensor pin

//Constants for Math/Limits
#define MAX_LIMIT_SENSE 10000
#endif