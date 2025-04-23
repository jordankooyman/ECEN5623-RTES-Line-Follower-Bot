#include "bluetooth_control.h"

// Globals
ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

// Controller connected callback
void onConnectedController(ControllerPtr ctl) {
    Serial.println("Controller connected!");

    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        if (!myControllers[i]) {
            myControllers[i] = ctl;
            Serial.print("Assigned to slot "); Serial.println(i);
            break;
        }
    }
}

// Controller disconnected callback
void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            Serial.print("Controller slot "); Serial.print(i); Serial.println(" disconnected");
            break;
        }
    }
}

// Bluetooth setup
void bluetooth_setup() {
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    Serial.println("Bluetooth setup complete.");
}

// Bluetooth update - call this in loop()
byte_t bluetooth_update() {
    BP32.update();

    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        ControllerPtr gp = myControllers[i];
        if (gp && gp->isConnected()) {
            return parseController(gp);
        }
    }

    return Stop; // Default if no controller connected
}

