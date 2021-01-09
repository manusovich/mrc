#include "Arduino.h"

#define j1_step_pin 0
#define j1_dir_pin 1
#define j2_step_pin 2
#define j2_dir_pin 3
#define j3_step_pin 4
#define j3_dir_pin 5
#define j4_step_pin 6
#define j4_dir_pin 7
#define j5_step_pin 8
#define j5_dir_pin 9
#define j6_step_pin 10
#define j6_dir_pin 11

// pinNumber, maxAngularVel degree/sec, calibMin, calibMax, angleDegMin, angleDegMax, home position
const float servoConfig[6][8] = {
    { j1_step_pin, j1_dir_pin,  300 * DEG_TO_RAD,  700.00, 2380.00,  -90.00 * DEG_TO_RAD,  90.00 * DEG_TO_RAD, 0 },
    { j2_step_pin, j2_dir_pin,  300 * DEG_TO_RAD,  710.00, 1909.00,  -45.00 * DEG_TO_RAD,  90.00 * DEG_TO_RAD, 0 },
    { j3_step_pin, j3_dir_pin,  300 * DEG_TO_RAD, 2290.00,  650.00,  -45.00 * DEG_TO_RAD, 135.00 * DEG_TO_RAD, 0 },
    { j4_step_pin, j4_dir_pin,  300 * DEG_TO_RAD,  740.00, 2260.00,  -90.00 * DEG_TO_RAD,  85.00 * DEG_TO_RAD, 0 },
    { j5_step_pin, j5_dir_pin,  300 * DEG_TO_RAD,  730.00, 2340.00, -140.00 * DEG_TO_RAD,  15.00 * DEG_TO_RAD, 0 },
    { j6_step_pin, j6_dir_pin,  300 * DEG_TO_RAD,  740.00, 2200.00,  -90.00 * DEG_TO_RAD,  60.00 * DEG_TO_RAD, 0 }
};

// mor mp-robot-a/mp-robot-kit
float geometry[5][3] = {
    {    4.6, 0,    7.9 },
    {      0, 0,   11.7 },
    {      1, 0,    1.5 },
    {  12.15, 0,      0 },
    {      0, 0,     -3 }
};

// E.g. joint 0 cant be < 90° to not crash into itself
float logicAngleLimits[6][2] = {
    { servoConfig[0][5],
      servoConfig[0][6] },
    { servoConfig[1][5],
      servoConfig[1][6] },
    { servoConfig[2][5],
      servoConfig[2][6] },
    { servoConfig[3][5],
      servoConfig[3][6] },
    { servoConfig[4][5],
      servoConfig[4][6] },
    { servoConfig[5][5],
      servoConfig[5][6] }
};

// relation between physical and logical angles based on robot kinematic coupling.
void logicalToPhysicalAngles(float angles[6]) {
    angles[2] += angles[1];
}

void physicalToLogicalAngles(float angles[6]) {
    angles[2] -= angles[1];
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

unsigned int pinMap[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // todo
