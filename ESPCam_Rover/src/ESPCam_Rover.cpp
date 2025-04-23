// File: ESPCam_Rover.cpp
// ESPCam Camera Line Following Real-time Rover
// Using the AI Thinker ESP32-CAM microcontroller
// Written by Jordan Kooyman (jordan.kooyman@colorado.edu) ~and Eric Percin (eric.percin@colorado.edu)
// @brief This project implements a real-time line-following rover using the AI Thinker ESP32-CAM microcontroller using Rate Monotonic principles.
// It integrates motor control, shift register handling, camera parsing, and controller parsing tasks using FreeRTOS.
// The rover processes input from a shift register and optionally a camera or controller to determine motor commands,
// enabling autonomous navigation or manual control.
// Last modified on 4/21/2025

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
#define MANUAL_CONTROL_TIMEOUT (5) // How many motor ticks to wait before current command expires


// Global Variables
// Task handles
static TaskHandle_t shiftRegService = NULL;
static TaskHandle_t motorControlService = NULL;
static TaskHandle_t cameraParsingService = NULL;
static TaskHandle_t controllerParsingService = NULL;
// Global State Trackers
motorCommand_t motorCommand;
volatile bool autonomousMode = false; // Flag to indicate if the rover is in autonomous mode
volatile byte_t LedBits; 
#ifndef SHIFT_LED_DISPLAY
    CRGB leds[PIXEL_COUNT];
    CRGB currentColor = CRGB::Teal;
#endif

// Function Prototypes
void vPrvRunShiftRegisters(void *pvParameters);
void vPrvControlMotors(void *pvParameters);
void vPrvCameraParse(void *pvParameters);
void vPrvControllerParse(void *pvParameters);
byte_t xPrvParseButtons(byte_t ButtonBits);


// Function Definitions
/**
 * Setup function for the ESP32-CAM Rover.
 * Initializes motor control pins, shift register pins, and creates tasks for various services.
 * The function also sets the initial states for the motors and the shift registers.
 * Written by Jordan Kooyman
 */
void setup()
{
    // Initialize shift register pins
    #ifdef SHIFT_LED_DISPLAY
        pinMode(DATA_OUT, OUTPUT);
    #else
        FastLED.addLeds<WS2812, DATA_OUT, GRB>(leds, PIXEL_COUNT);
        fill_solid(leds, PIXEL_COUNT, CRGB::BlueViolet);
        FastLED.setBrightness(100);
        FastLED.show();
        delay(1000); // Startup delay for show
    #endif
    pinMode(SHIFT_CLK, OUTPUT);
    pinMode(LATCH_PIN, OUTPUT);
    pinMode(DATA_IN, INPUT);

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

    #ifdef RM_ANALYSIS_MODE
        pinMode(RM_OUTPUT_PIN, OUTPUT);
        digitalWrite(RM_OUTPUT_PIN, LOW); // Set the output pin low to indicate the start of the task
    #endif
    
    // Set default states
    digitalWrite(SHIFT_CLK, HIGH);  // Clock idle state
    digitalWrite(LATCH_PIN, HIGH);  // Latch idle state
    
    // Stop motors initially
    analogWrite(MOTOR_LSPEED, MOTOR_STOP);
    digitalWrite(MOTOR_LDIR, LEFT_FORWARD);
    analogWrite(MOTOR_RSPEED, MOTOR_STOP);
    digitalWrite(MOTOR_RDIR, RIGHT_FORWARD);

    LedBits = 0; 

    // Create tasks
    xTaskCreatePinnedToCore(vPrvRunShiftRegisters, "Shift Register Service", SHIFT_REG_SERVICE_STACK_SIZE, NULL, SHIFT_REG_SERVICE_PRIORITY, &shiftRegService, SHIFT_REG_SERVICE_CORE);
    xTaskCreatePinnedToCore(vPrvControlMotors, "Motor Control Service", MOTOR_SERVICE_STACK_SIZE, NULL, MOTOR_SERVICE_PRIORITY, &motorControlService, MOTOR_SERVICE_CORE);
    xTaskCreatePinnedToCore(vPrvCameraParse, "Camera Parsing Service", CAMERA_SERVICE_STACK_SIZE, NULL, CAMERA_SERVICE_PRIORITY, &cameraParsingService, CAMERA_SERVICE_CORE);
    xTaskCreatePinnedToCore(vPrvControllerParse, "Controller Parsing Service", CONTROLLER_SERVICE_STACK_SIZE, NULL, CONTROLLER_SERVICE_PRIORITY, &controllerParsingService, CONTROLLER_SERVICE_CORE);
}

/**
 * Main loop function for the ESP32-CAM Rover.
 * The function does nothing and is used to keep the initial thread running.
 * All services are handled in their respective tasks.
 */
void loop() {
    // Do Nothing
}


/**
 * Reads the shift register and updates the motor control based on button states.
 * The function uses bitwise operations to check which buttons are pressed and sets the motor speed
 * accordingly. The function runs in a loop with a specified period, allowing for periodic updates.
 * Written by Jordan Kooyman
 *
 * @param pvParameters In the provided code snippet, the function `vPrvRunShiftRegisters` takes a void
 * pointer `pvParameters` as a parameter. In this specific implementation, the `pvParameters` is not
 * being used within the function.
 */
void vPrvRunShiftRegisters(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(SHIFT_REG_SERVICE_PERIOD_MS);

    while(true)
    {
        #ifdef RM_ANALYSIS_MODE
            #if RM_ANALYSIS_MODE == RM_FOCUS_SHIFT_REG
                #warning    ---   Rate Monotonic Analysis Mode Enabled for Shift Register Service   ---
                digitalWrite(RM_OUTPUT_PIN, HIGH); // Set the output pin high to indicate the start of the task
            #endif
        #endif

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
            if (ledStates)
                leds[ledStates-1] = currentColor; 
            FastLED.setBrightness(100);

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
            #else
                digitalWrite(FLASH_LED, LOW);
            #endif
        #elif defined(ENABLE_FLASH_LED)
            // Restore the flash LED state
            // This is necessary because the flash LED pin is shared with the WS2812b data pin
            // and the WS2812b library may have changed its state
            digitalWrite(FLASH_LED, prevPinState);
        #endif

        // Parse button states
        byte_t buttonDirection = xPrvParseButtons(buttonStates);

        // Update motor state if in manual mode
        if(!autonomousMode)
            motorCommand.setMotorSpeed(buttonDirection, MANUAL_CONTROL_TIMEOUT);

        #ifdef RM_ANALYSIS_MODE
            #if RM_ANALYSIS_MODE == RM_FOCUS_SHIFT_REG
                digitalWrite(RM_OUTPUT_PIN, LOW); // Set the output pin high to indicate the start of the task
            #endif
        #endif

        // Delay until next period
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}


/**
 * Parses the button states from the shift register and returns the corresponding direction.
 * The function uses bitwise operations to check which buttons are pressed and assigns a direction
 * based on the button states. The function returns a byte_t value representing the direction.
 * Written by Jordan Kooyman
 *
 * @return byte_t representing the direction based on button states.
 */
byte_t xPrvParseButtons(byte_t ButtonBits)
{
    byte_t buttonDirection = Stop;
    static bool buttonStateChanged = false;

    if (ButtonBits & (1 << BTN_UP))     buttonDirection += North;
    if (ButtonBits & (1 << BTN_DOWN))   buttonDirection += South;
    if (ButtonBits & (1 << BTN_LEFT))   buttonDirection += West;
    if (ButtonBits & (1 << BTN_RIGHT))  buttonDirection += East;
    
    if (ButtonBits & (1 << BTN_MODE))   
    {
        if (!buttonStateChanged) // Only toggle once per button press
        {
            buttonStateChanged = true; // Set state
            autonomousMode = !autonomousMode; // Toggle autonomous mode
        }
        
    }
    else
    {
        buttonStateChanged = false; // Reset state
    }

    return buttonDirection;
}



/**
 * Controls the speed and direction of motors based on received commands and updates the motor outputs periodically.
 * Reads the motor direction from a global motorCommand object, which is updated by other tasks.
 * Uses the motor direction to set the speed and direction of the left and right motors using hardcoded PWM signals,
 * split into 8 distinct directions (North, NorthEast, East, SouthEast, South, SouthWest, West, NorthWest).
 * Written by Jordan Kooyman
 *
 * 
 * @param pvParameters In the provided code snippet, the function `vPrvControlMotors` takes a void
 * pointer `pvParameters` as a parameter. In this specific implementation, the `pvParameters` is not
 * being used within the function.
 */
void vPrvControlMotors(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(MOTOR_SERVICE_PERIOD_MS);
    bool RunMotors = true;
    
    while(true)
    {
        #ifdef RM_ANALYSIS_MODE
            #if RM_ANALYSIS_MODE == RM_FOCUS_MOTOR
                #warning    ---   Rate Monotonic Analysis Mode Enabled for Motor Control Service   ---
                digitalWrite(RM_OUTPUT_PIN, HIGH); // Set the output pin high to indicate the start of the task
            #endif
        #endif

        byte_t dir;
        byte_t timeout; // How many motor ticks to wait before current command expires
        motorCommand.getMotorSpeedTimeout(&dir, &timeout);

        if (timeout > 0)
            timeout--;
        else
            dir = Stop;

        byte_t leftSpeed = MOTOR_STOP;
        byte_t rightSpeed = MOTOR_STOP;
        byte_t leftDir = LEFT_FORWARD;
        byte_t rightDir = RIGHT_FORWARD;
        byte_t ledBits = 0;

        switch(dir)
        {
            case North:
                #ifdef SHIFT_LED_DISPLAY
                    ledBits |= (1 << LED_NORTH);
                #else
                    ledBits = LED_NORTH;
                #endif
                if (RunMotors)
                {
                    leftSpeed = L_N_SPD;
                    rightSpeed = R_N_SPD;
                    leftDir = LEFT_FORWARD;
                    rightDir = RIGHT_FORWARD;
                }
                break;
            case East:
                #ifdef SHIFT_LED_DISPLAY
                    ledBits |= (1 << LED_EAST);
                #else
                    ledBits = LED_EAST;
                #endif
                if (RunMotors)
                {
                    leftSpeed = L_E_SPD;
                    rightSpeed = R_E_SPD;
                    leftDir = LEFT_FORWARD;
                    rightDir = RIGHT_REVERSE;
                }
                break;
            case South:
                #ifdef SHIFT_LED_DISPLAY
                    ledBits |= (1 << LED_SOUTH);
                #else
                    ledBits = LED_SOUTH;
                #endif 
                if (RunMotors)
                {
                    leftSpeed = L_S_SPD;
                    rightSpeed = R_S_SPD;
                    leftDir = LEFT_REVERSE;
                    rightDir = RIGHT_REVERSE;
                }
                break;
            case West:
                #ifdef SHIFT_LED_DISPLAY
                    ledBits |= (1 << LED_WEST);
                #else
                    ledBits = LED_WEST;
                #endif 
                if (RunMotors)
                {
                    leftSpeed = L_W_SPD;
                    rightSpeed = R_W_SPD;
                    leftDir = LEFT_REVERSE;
                    rightDir = RIGHT_FORWARD;
                }
                break;
            case NorthEast:
                #ifdef SHIFT_LED_DISPLAY
                    ledBits |= (1 << LED_NORTHEAST);
                #else
                    ledBits = LED_NORTHEAST;
                #endif 
                if (RunMotors)
                {
                    leftSpeed = L_NE_SPD;
                    rightSpeed = R_NE_SPD;
                    leftDir = LEFT_FORWARD;
                    rightDir = RIGHT_FORWARD;
                }
                break;
            case SouthEast:
                #ifdef SHIFT_LED_DISPLAY
                    ledBits |= (1 << LED_SOUTHEAST);
                #else
                    ledBits = LED_SOUTHEAST;
                #endif 
                if (RunMotors)
                {
                    leftSpeed = L_SE_SPD;
                    rightSpeed = R_SE_SPD;
                    leftDir = LEFT_REVERSE;
                    rightDir = RIGHT_REVERSE;
                }
                break;
            case SouthWest:
                #ifdef SHIFT_LED_DISPLAY
                    ledBits |= (1 << LED_SOUTHWEST);
                #else
                    ledBits = LED_SOUTHWEST;
                #endif 
                if (RunMotors)
                {
                    leftSpeed = L_SW_SPD;
                    rightSpeed = R_SW_SPD;
                    leftDir = LEFT_REVERSE;
                    rightDir = RIGHT_REVERSE;
                }
                break;
            case NorthWest:
                #ifdef SHIFT_LED_DISPLAY
                    ledBits |= (1 << LED_NORTHWEST);
                #else
                    ledBits = LED_NORTHWEST;
                #endif 
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

        // Update Global LED State Tracking Variable
        LedBits = ledBits;

        #ifdef RM_ANALYSIS_MODE
            #if RM_ANALYSIS_MODE == RM_FOCUS_MOTOR
                digitalWrite(RM_OUTPUT_PIN, LOW); // Set the output pin high to indicate the start of the task
            #endif
        #endif

        // Delay until next period
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

/**
 * The function `vPrvCameraParse` is a placeholder for camera parsing logic that runs periodically.
 * It uses FreeRTOS to manage timing and task scheduling.
 * Written by Eric Percin
 * 
 * @param pvParameters In the provided code snippet, the function `vPrvCameraParse` takes a void
 * pointer `pvParameters` as a parameter. In this specific implementation, the `pvParameters` is not
 * being used within the function.
 */
void vPrvCameraParse(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(CAMERA_SERVICE_PERIOD_MS);
    while(true)
    {
        #ifdef RM_ANALYSIS_MODE
            #if RM_ANALYSIS_MODE == RM_FOCUS_CAMERA
                #warning    ---   Rate Monotonic Analysis Mode Enabled for Camera Parsing Service   ---
                digitalWrite(RM_OUTPUT_PIN, HIGH); // Set the output pin high to indicate the start of the task
            #endif
        #endif

        // Placeholder for camera parsing logic
        // This function should be implemented to handle camera data

        #ifdef RM_ANALYSIS_MODE
            #if RM_ANALYSIS_MODE == RM_FOCUS_CAMERA
                digitalWrite(RM_OUTPUT_PIN, LOW); // Set the output pin high to indicate the start of the task
            #endif
        #endif

        // Delay until next period
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}


/**
 * The function `vPrvCameraParse` is a placeholder for camera parsing logic that runs periodically.
 * It uses FreeRTOS to manage timing and task scheduling.
 * Written by Eric Percin
 * 
 * @param pvParameters In the provided code snippet, the function `vPrvCameraParse` takes a void
 * pointer `pvParameters` as a parameter. In this specific implementation, the `pvParameters` is not
 * being used within the function.
 */
void vPrvControllerParse(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(CONTROLLER_SERVICE_PERIOD_MS);
    while(true)
    {
        #ifdef RM_ANALYSIS_MODE
            #if RM_ANALYSIS_MODE == RM_FOCUS_CONTROLLER
                #warning    ---   Rate Monotonic Analysis Mode Enabled for Controller Parsing Service   ---
                digitalWrite(RM_OUTPUT_PIN, HIGH); // Set the output pin high to indicate the start of the task
            #endif
        #endif

        // Placeholder for bluetooth controller parsing logic
        // This function should be implemented to handle controller data

        #ifdef RM_ANALYSIS_MODE
            #if RM_ANALYSIS_MODE == RM_FOCUS_CONTROLLER
                digitalWrite(RM_OUTPUT_PIN, LOW); // Set the output pin high to indicate the start of the task
            #endif
        #endif

        // Delay until next period
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}
