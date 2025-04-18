// File: custom_types.hpp
// Custom Variable/Object types for the ESPCam Rover project
// Written by Jordan Kooyman (jordan.kooyman@colorado.edu)
// Last modified on 4/18/2025
// This file contains the definitions for the custom types used in the ESPCam Rover project.

#include <freertos/semphr.h>

#ifndef CUSTOM_TYPES_HPP
#define CUSTOM_TYPES_HPP
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

class motorCommand_t
{
public:
    motorCommand_t() : dir(0), timeout(0) {xUpdatingSemaphore = xSemaphoreCreateBinary(); xSemaphoreGive(xUpdatingSemaphore);}
    motorCommand_t(byte_t d, byte_t t) : dir(d), timeout(t) {xUpdatingSemaphore = xSemaphoreCreateBinary(); xSemaphoreGive(xUpdatingSemaphore);}
    void setMotorSpeed(byte_t d, byte_t t)
    {
        // Wait for the semaphore to be available
        if (xSemaphoreTake(xUpdatingSemaphore, portMAX_DELAY) == pdTRUE)
        {
            // Update the motor command
            dir = d;
            timeout = t;

            // Give the semaphore back
            xSemaphoreGive(xUpdatingSemaphore);
        }
    }
    void getMotorSpeed(byte_t* d, byte_t* t)
    {
        // Wait for the semaphore to be available
        if (xSemaphoreTake(xUpdatingSemaphore, portMAX_DELAY) == pdTRUE)
        {
            // Get the motor command
            *d = dir;
            *t = timeout;

            // Give the semaphore back
            xSemaphoreGive(xUpdatingSemaphore);
        }
    }
private:
    SemaphoreHandle_t xUpdatingSemaphore; // Used to ensure atomic access to the motor commands
    volatile byte_t dir;
    volatile byte_t timeout;
};
#endif // CUSTOM_TYPES_HPP