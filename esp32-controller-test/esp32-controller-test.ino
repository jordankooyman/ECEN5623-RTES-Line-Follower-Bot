// Copyright 2021 - 2023, Ricardo Quesada
// SPDX-License-Identifier: Apache 2.0 or LGPL-2.1-or-later

// This program allows you to connect and test your bluetooth controller

#include <Bluepad32.h>

// Max number of controllers
ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

// Arduino setup function. Runs in CPU 1
void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for Serial Monitor to open
  }

  String fv = BP32.firmwareVersion();
  Serial.print("Firmware version installed: ");
  Serial.println(fv);

  // Get Bluetooth Device Address
  const uint8_t* addr = BP32.localBdAddress();
  Serial.print("BD Address: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(addr[i], HEX);
    if (i < 5)
      Serial.print(":");
    else
      Serial.println();
  }

  // Initialize Bluepad32
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // Forget previous Bluetooth keys (optional, but sometimes helps with reconnection issues)
  BP32.forgetBluetoothKeys();
}

// Called when a new controller connects
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.print("CALLBACK: Controller is connected, index=");
      Serial.println(i);
      myControllers[i] = ctl;
      foundEmptySlot = true;

      // Optional: print basic info
      ControllerProperties properties = ctl->getProperties();
      char buf[80];
      sprintf(buf,
              "BTAddr: %02x:%02x:%02x:%02x:%02x:%02x, VID/PID: %04x:%04x, "
              "flags: 0x%02x",
              properties.btaddr[0], properties.btaddr[1], properties.btaddr[2],
              properties.btaddr[3], properties.btaddr[4], properties.btaddr[5],
              properties.vendor_id, properties.product_id, properties.flags);
      Serial.println(buf);
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but no empty slot found");
  }
}

// Called when a controller disconnects
void onDisconnectedController(ControllerPtr ctl) {
  bool foundGamepad = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.print("CALLBACK: Controller disconnected from index=");
      Serial.println(i);
      myControllers[i] = nullptr;
      foundGamepad = true;
      break;
    }
  }
  if (!foundGamepad) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

// Process the input from a connected controller
void processGamepad(ControllerPtr gamepad) {
  if (!gamepad || !gamepad->isConnected()) {
    return;
  }

  // Read buttons
  uint16_t buttons = gamepad->buttons();

  bool aPressed = buttons & 0x0002;
  bool bPressed = buttons & 0x0001;
  bool xPressed = buttons & 0x0008;
  bool yPressed = buttons & 0x0004;

  // Read left joystick X axis
  int16_t axisX = gamepad->axisX(); // Range: -512 to +512
  
  // Example output: A:1 B:0 X:0 Y:1 AxisX:-123
  char buf[100];
  snprintf(buf, sizeof(buf),
           "A:%d B:%d X:%d Y:%d AxisX:%d",
           aPressed ? 1 : 0,
           bPressed ? 1 : 0,
           xPressed ? 1 : 0,
           yPressed ? 1 : 0,
           axisX);

  Serial.println(buf);
}

void loop() {
  BP32.update(); // Poll controllers

  // Check all connected controllers
  for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
    ControllerPtr myController = myControllers[i];

    if (myController && myController->isConnected()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      }
      //ignore Mouse and BalanceBoard controllers
    }
  }

  delay(150); // Poll every 150ms
}
