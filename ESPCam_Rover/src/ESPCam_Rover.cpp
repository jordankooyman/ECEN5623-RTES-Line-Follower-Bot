// File: ESPCam_Rover.cpp
// ESPCam Camera Line Following Real-time Rover
// Using the AI Thinker ESP32-CAM microcontroller
// Written by Jordan Kooyman (jordan.kooyman@colorado.edu) ~and Eric Percin (eric.percin@colorado.edu)
// Last modified on 4/18/2025

// Includes
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#ifndef SHIFT_LED_DISPLAY
    #include <FastLED.h>
#endif
#include "rover_config.h"
#include "custom_types.hpp"


// Constants
#define SHIFT_REG_BITS (8)
#define MANUAL_CONTROL_TIMEOUT (25) // How many motor ticks to wait before current command expires


// Global Variables
// Task handles
static TaskHandle_t shiftRegService = NULL;
static TaskHandle_t motorControlService = NULL;
static TaskHandle_t cameraParsingService = NULL;
static TaskHandle_t controllerParsingService = NULL;
// Global State Trackers
motorCommand_t motorCommand;
byte_t ButtonBits;
byte_t LedBits; 
#ifndef SHIFT_LED_DISPLAY
    CRGB leds[PIXEL_COUNT];
#endif

// Function Prototypes
void vPrvRunShiftRegisters(void *pvParameters);
void vPrvControlMotors(void *pvParameters);
void vPrvCameraParse(void *pvParameters);
void vPrvControllerParse(void *pvParameters);
byte_t xPrvParseButtons();


// Function Definitions
void setup()
{
    // Initialize motor control pins
    pinMode(MOTOR_LSPEED, OUTPUT);
    pinMode(MOTOR_LDIR, OUTPUT);
    pinMode(MOTOR_RSPEED, OUTPUT);
    pinMode(MOTOR_RDIR, OUTPUT);

    #ifdef ENABLE_FLASH_LED
        // Initialize Camera Flash LED pin
        pinMode(FLASH_LED, OUTPUT);
        digitalWrite(FLASH_LED, LOW); // Turn off flash LED
    #endif
    
    // Initialize shift register pins
    pinMode(SHIFT_CLK, OUTPUT);
    pinMode(LATCH_PIN, OUTPUT);
    pinMode(DATA_IN, INPUT);
    #ifdef SHIFT_LED_DISPLAY
        pinMode(DATA_OUT, OUTPUT);
    #else
        FastLED.addLeds<WS2812, DATA_OUT, GRB>(leds, PIXEL_COUNT);
    #endif
    
    // Set default states
    digitalWrite(SHIFT_CLK, HIGH);  // Clock idle state
    digitalWrite(LATCH_PIN, HIGH);  // Latch idle state
    
    // Stop motors initially
    analogWrite(MOTOR_LSPEED, MOTOR_STOP);
    digitalWrite(MOTOR_LDIR, LEFT_FORWARD);
    analogWrite(MOTOR_RSPEED, MOTOR_STOP);
    digitalWrite(MOTOR_RDIR, RIGHT_FORWARD);
    
    /*Serial.begin(115200); // If using serial, can only receive (cannot send any data, ESP RX Pin is used as LATCH_PIN)
    Serial.println("ESP32-CAM Shift Register Test");*/

    ButtonBits = 0;
    LedBits = 0; 

    // Create tasks
    xTaskCreatePinnedToCore(vPrvRunShiftRegisters, "Shift Register Service", SHIFT_REG_SERVICE_STACK_SIZE, NULL, SHIFT_REG_SERVICE_PRIORITY, &shiftRegService, SHIFT_REG_SERVICE_CORE);
    xTaskCreatePinnedToCore(vPrvControlMotors, "Motor Control Service", MOTOR_SERVICE_STACK_SIZE, NULL, MOTOR_SERVICE_PRIORITY, &motorControlService, MOTOR_SERVICE_CORE);
    xTaskCreatePinnedToCore(vPrvCameraParse, "Camera Parsing Service", CAMERA_SERVICE_STACK_SIZE, NULL, CAMERA_SERVICE_PRIORITY, &cameraParsingService, CAMERA_SERVICE_CORE);
    xTaskCreatePinnedToCore(vPrvControllerParse, "Controller Parsing Service", CONTROLLER_SERVICE_STACK_SIZE, NULL, CONTROLLER_SERVICE_PRIORITY, &controllerParsingService, CONTROLLER_SERVICE_CORE);
}

void loop() {
    // Do Nothing
}

void vPrvRunShiftRegisters(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(SHIFT_REG_SERVICE_PERIOD_MS);
    while(true)
    {
        byte_t buttonStates = 0;
        byte_t ledStates = LedBits;
        #ifdef ENABLE_FLASH_LED
            byte_t prevPinState = digitalRead(FLASH_LED); // Preserve Flash Pin State afterwards
        #endif
        #ifndef SHIFT_LED_DISPLAY
            // Output data via the WS2812b LED Ring
            // Clear all LEDs
            fill_solid(leds, PIXEL_COUNT, CRGB::Black);

            // Set the LED corresponding to the current direction to green
            leds[LedBits] = CRGB::Green; 
            FastLED.setBrightness(255);

            // Show the updated LED states
            FastLED.show();
        #endif
        // Latch toggle to load parallel data into 74HC165n
        digitalWrite(LATCH_PIN, LOW);
        delayMicroseconds(5);  // Small delay for latch to take effect
        digitalWrite(LATCH_PIN, HIGH);
        
        // Read/Write 8 bits from the shift register
        for (int i = 0; i < SHIFT_REG_BITS; i++) {
            // Clock low to prepare for reading/writing
            digitalWrite(SHIFT_CLK, LOW);
            
            #ifdef SHIFT_LED_DISPLAY
                // Write the current bit
                digitalWrite(DATA_OUT, (ledStates >> i) & 0x01);
            #endif

            // Read the current bit and store it
            buttonStates |= (digitalRead(DATA_IN) << i);
            
            // Clock high to shift to next bit
            digitalWrite(SHIFT_CLK, HIGH);
        }
        
        #ifdef SHIFT_LED_DISPLAY
            // Latch toggle to save serial data into 74HC595n
            digitalWrite(LATCH_PIN, LOW);
            delayMicroseconds(5);  // Small delay for latch to take effect
            digitalWrite(LATCH_PIN, HIGH);
            #ifdef ENABLE_FLASH_LED
                digitalWrite(FLASH_LED, prevPinState);
            #endif
        #elif defined(ENABLE_FLASH_LED)
            // Restore the flash LED state
            // This is necessary because the flash LED pin is shared with the WS2812b data pin
            // and the WS2812b library may have changed its state
            digitalWrite(FLASH_LED, prevPinState);
        #endif

        ButtonBits = buttonStates;
        // Parse button states
        byte_t buttonDirection = xPrvParseButtons();

        // Update motor state
        motorCommand.setMotorSpeed(buttonDirection, MANUAL_CONTROL_TIMEOUT);

        // Delay until next period
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}


byte_t xPrvParseButtons()
{
    byte_t buttonDirection = Stop;

    if (ButtonBits & (1 << BTN_UP))     buttonDirection += North;
    if (ButtonBits & (1 << BTN_DOWN))   buttonDirection += South;
    if (ButtonBits & (1 << BTN_LEFT))   buttonDirection += West;
    if (ButtonBits & (1 << BTN_RIGHT))  buttonDirection += East;
    if (ButtonBits & (1 << BTN_MODE))   buttonDirection += MotorsOff;

    return buttonDirection;
}


void vPrvControlMotors(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(MOTOR_SERVICE_PERIOD_MS);
    bool RunMotors = true;
    bool MotorStateChanged = false;
    while(true)
    {
        byte_t dir;
        byte_t timeout; // How many motor ticks to wait before current command expires
        motorCommand.getMotorSpeed(&dir, &timeout);

        if (timeout > 0)
            timeout--;
        else
            dir = MotorsOff;

        if (dir >= MotorsOff)
        {
            if (!MotorStateChanged)
            {
                RunMotors = !RunMotors;
                MotorStateChanged = true;
            }
            dir -= MotorsOff;
        }
        else
            MotorStateChanged = false;

        byte_t leftSpeed = MOTOR_STOP;
        byte_t rightSpeed = MOTOR_STOP;
        byte_t leftDir = LEFT_FORWARD;
        byte_t rightDir = RIGHT_FORWARD;

        LedBits = 0;
        switch(dir)
        {
            case North:
                LedBits |= (1 << LED_NORTH);
                if (RunMotors)
                {
                    leftSpeed = L_N_SPD;
                    rightSpeed = R_N_SPD;
                    leftDir = LEFT_FORWARD;
                    rightDir = RIGHT_FORWARD;
                }
                break;
            case East:
                LedBits |= (1 << LED_EAST);
                if (RunMotors)
                {
                    leftSpeed = L_E_SPD;
                    rightSpeed = R_E_SPD;
                    leftDir = LEFT_FORWARD;
                    rightDir = RIGHT_REVERSE;
                }
                break;
            case South:
                LedBits |= (1 << LED_SOUTH);
                if (RunMotors)
                {
                    leftSpeed = L_S_SPD;
                    rightSpeed = R_S_SPD;
                    leftDir = LEFT_REVERSE;
                    rightDir = RIGHT_REVERSE;
                }
                break;
            case West:
                LedBits |= (1 << LED_WEST);
                if (RunMotors)
                {
                    leftSpeed = L_W_SPD;
                    rightSpeed = R_W_SPD;
                    leftDir = LEFT_REVERSE;
                    rightDir = RIGHT_FORWARD;
                }
                break;
            case NorthEast:
                LedBits |= (1 << LED_NORTHEAST);
                if (RunMotors)
                {
                    leftSpeed = L_NE_SPD;
                    rightSpeed = R_NE_SPD;
                    leftDir = LEFT_FORWARD;
                    rightDir = RIGHT_FORWARD;
                }
                break;
            case SouthEast:
                LedBits |= (1 << LED_SOUTHEAST);
                if (RunMotors)
                {
                    leftSpeed = L_SE_SPD;
                    rightSpeed = R_SE_SPD;
                    leftDir = LEFT_REVERSE;
                    rightDir = RIGHT_REVERSE;
                }
                break;
            case SouthWest:
                LedBits |= (1 << LED_SOUTHWEST);
                if (RunMotors)
                {
                    leftSpeed = L_SW_SPD;
                    rightSpeed = R_SW_SPD;
                    leftDir = LEFT_REVERSE;
                    rightDir = RIGHT_REVERSE;
                }
                break;
            case NorthWest:
                LedBits |= (1 << LED_NORTHWEST);
                if (RunMotors)
                {
                    leftSpeed = L_NW_SPD;
                    rightSpeed = R_NW_SPD;
                    leftDir = LEFT_FORWARD;
                    rightDir = RIGHT_FORWARD;
                }
                break;
            case Stop:
            default: // Conflicting Directions Requested, Stop
                break; // Use initialization values
        }

        // Update outputs
        analogWrite(MOTOR_LSPEED, leftSpeed);
        digitalWrite(MOTOR_LDIR, leftDir);
        analogWrite(MOTOR_RSPEED, rightSpeed);
        digitalWrite(MOTOR_RDIR, rightDir);

        // Delay until next period
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void vPrvCameraParse(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(CAMERA_SERVICE_PERIOD_MS);
    while(true)
    {
        // Placeholder for camera parsing logic
        // This function should be implemented to handle camera data

        // Delay until next period
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}
void vPrvControllerParse(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(CONTROLLER_SERVICE_PERIOD_MS);
    while(true)
    {
        // Placeholder for bluetooth controller parsing logic
        // This function should be implemented to handle controller data

        // Delay until next period
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}
