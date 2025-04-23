#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <Bluepad32.h>

typedef unsigned char byte_t;

enum Direction {
    North = 20,
    NorthEast = 22,
    East = 2,
    SouthEast = 12,
    South = 10,
    SouthWest = 11,
    West = 1,
    NorthWest = 21,
    Stop = 0,
    MotorsOff = 35
};

void motor_setup();
void controlMotors(byte_t dir, float angle);
byte_t parseController(ControllerPtr gp);

#endif
