#include "Arduino.h"

#define j1_step_pin 0
#define j1_dir_pin 1
#define j1_encA_pin 15
#define j1_encB_pin 14
#define j1_hs_pin 24

#define j2_step_pin 2
#define j2_dir_pin 3
#define j2_encA_pin 17
#define j2_encB_pin 16
#define j2_hs_pin 25

#define j3_step_pin 4
#define j3_dir_pin 5
#define j3_encA_pin 19
#define j3_encB_pin 18
#define j3_hs_pin 38

#define j4_step_pin 6
#define j4_dir_pin 7
#define j4_encA_pin 20
#define j4_encB_pin 21
#define j4_hs_pin 39

#define j5_step_pin 8
#define j5_dir_pin 9
#define j5_encA_pin 22
#define j5_encB_pin 23
#define j5_hs_pin 40

#define j6_step_pin 10
#define j6_dir_pin 11
#define j6_encA_pin 28
#define j6_encB_pin 29
#define j6_hs_pin 41

// step motor step pin, step motor dir pin, hall sensor pin, maxAngularVel degree/sec, encoder a pin, encoder b pin,
// angleDegMin, angleDegMax, home position, calibration direction, moving direction, revsteps, revpulses
const float servoConfig[6][13] = {
    {j1_step_pin, j1_dir_pin, j1_hs_pin, 300 * DEG_TO_RAD, j1_encA_pin, j1_encB_pin, -90.00 * DEG_TO_RAD, 90.00 * DEG_TO_RAD, 0, -1, 1, 64000, 81914},
    {j2_step_pin, j2_dir_pin, j2_hs_pin, 300 * DEG_TO_RAD, j2_encA_pin, j2_encB_pin, -130.00 * DEG_TO_RAD, 30.00 * DEG_TO_RAD, 0, -1, -1, 40000 * 2, 51200 * 2},
    {j3_step_pin, j3_dir_pin, j3_hs_pin, 300 * DEG_TO_RAD, j3_encA_pin, j3_encB_pin, -45.00 * DEG_TO_RAD, 90.00 * DEG_TO_RAD, 0, 1, -1, 80000, 102300},
    {j4_step_pin, j4_dir_pin, j4_hs_pin, 300 * DEG_TO_RAD, j4_encA_pin, j4_encB_pin, -90.00 * DEG_TO_RAD, 90.00 * DEG_TO_RAD, 0, -1, -1, 43948 * 1.41, 56258 * 1.41},
    {j5_step_pin, j5_dir_pin, j5_hs_pin, 300 * DEG_TO_RAD, j5_encA_pin, j5_encB_pin, -90.00 * DEG_TO_RAD, 100.00 * DEG_TO_RAD, 0, -1, -1, 5000 * 6.5, 3197 * 6.5},
    {j6_step_pin, j6_dir_pin, j6_hs_pin, 300 * DEG_TO_RAD, j6_encA_pin, j6_encB_pin, -130.00 * DEG_TO_RAD, 130.00 * DEG_TO_RAD, 0, 1, -1, 14500 * 2.15, 18549 * 2.15}};

// mor mp-robot-a/mp-robot-kit
float geometry[5][3] = {
    {0.7, 1, 2.5},
    {0, 0, 4.8},
    {0, -1, 0.8},
    {3, 0, 0},
    {0.3, 0, 0}};

// E.g. joint 0 cant be < 90° to not crash into itself
float logicAngleLimits[6][2] = {
    {servoConfig[0][6],
     servoConfig[0][7]},
    {servoConfig[1][6],
     servoConfig[1][7]},
    {servoConfig[2][6],
     servoConfig[2][7]},
    {servoConfig[3][6],
     servoConfig[3][7]},
    {servoConfig[4][6],
     servoConfig[4][7]},
    {servoConfig[5][6],
     servoConfig[5][7]}};

// relation between physical and logical angles based on robot kinematic coupling.
void logicalToPhysicalAngles(float angles[6])
{
  //angles[2] += angles[1];
}

void physicalToLogicalAngles(float angles[6])
{
  //angles[2] -= angles[1];
}

// 4 axis
// const float servoConfig[6][7] = {
//     { pin_robot_servo_0,  250,  570.00, 2400.00,  -77.00,  83.00,   0 },
//     { pin_robot_servo_1,  250, 1190.00, 2400.00,  -90.00,  18.00,   0 },
//     { pin_robot_servo_2,  250, 2175.00,  968.00, -110.00,  -9.00, -30 },
//     { pin_robot_servo_3,  250, 1500.00, 1500.00,  -90.00,  75.00,   0 },
//     { pin_robot_servo_4,  250, 1500.00, 1500.00,  -20.00, 135.00,  30 },
//     { pin_robot_servo_5,  250, 2281.00,  712.00,  -75.00,  75.00,   0 }
// };

// float geometry[5][3] = { { 2.5 + 2.3, 7.3, 0 }, { 0, 13.0, 0 }, { 1, 0, 0 }, { 12.6, 0, 0 }, { 0, -3.6, 0 } };

unsigned int pinMap[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // todo
