#ifndef BLUETOOTH_CONTROL_H
#define BLUETOOTH_CONTROL_H

#include <Bluepad32.h>
#include "motor_control.h" // For Direction and parseController()

void bluetooth_setup();
byte_t bluetooth_update();

#endif
