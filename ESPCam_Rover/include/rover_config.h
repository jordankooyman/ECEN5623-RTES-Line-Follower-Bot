// File: rover_config.h
// Configuration values for the ESPCam Rover, hardware dependent
// Written by Jordan Kooyman (jordan.kooyman@colorado.edu) & Eric Percin (eric.percin@colorado.edu)
// Last modified on 4/27/2025

#include <Arduino.h>

#ifndef ROVER_CONFIG_H
#define ROVER_CONFIG_H
// ***Basic Configurations***

//---Service Configurations--- (2 cores, Core0 must be used for Wifi/Bluetooth - 320KB RAM available total)
#define SHIFT_REG_SERVICE_STACK_SIZE (2048) // Stack size (words) for the shift register service task
#define SHIFT_REG_SERVICE_PERIOD_MS (100) // Period for the shift register service task
#define SHIFT_REG_SERVICE_PRIORITY (6) // Priority for the shift register service task
#define SHIFT_REG_SERVICE_CORE (1) // Core for the shift register service task
#define MOTOR_SERVICE_STACK_SIZE (2048) // Stack size (words) for the motor control service task
#define MOTOR_SERVICE_PERIOD_MS (20) // Period for the motor control service task
#define MOTOR_SERVICE_PRIORITY (10) // Priority for the motor control service task
#define MOTOR_SERVICE_CORE (1) // Core for the motor control service task
#define CAMERA_SERVICE_STACK_SIZE (10240) // Stack size (words) for the camera parsing service task
#define CAMERA_SERVICE_PERIOD_MS (110) // Period for the camera parsing service task ---- WCET Measurement: 80.96ms!!!
#define CAMERA_SERVICE_PRIORITY (5) // Priority for the camera parsing service task
#define CAMERA_SERVICE_CORE (0) // Core for the camera parsing service task (core 0 for now since it uses WiFi)
#define CONTROLLER_SERVICE_STACK_SIZE (4096) // Stack size (words) for the controller parsing service task
#define CONTROLLER_SERVICE_PERIOD_MS (84) // Period for the controller parsing service task
#define CONTROLLER_SERVICE_PRIORITY (8) // Priority for the controller parsing service task
#define CONTROLLER_SERVICE_CORE (0) // Core for the controller parsing service task (must be core 0 for Bluetooth)

//---Rate Monotonic Analysis Mode---
// Available tasks to monitor
#define RM_FOCUS_SHIFT_REG (1)
#define RM_FOCUS_MOTOR (2)
#define RM_FOCUS_CAMERA (3)
#define RM_FOCUS_CONTROLLER (4)
// Analysis mode configurations
//#define RM_ANALYSIS_MODE (RM_FOCUS_CAMERA) // Comment out to disable, or set to the value of the desired task to monitor
#define RM_ANALYSIS_PRIORITY_OVERRIDE (50) // Comment out to disable, or set to a priority greater than all others in the system to assign to desired task
#define RM_OUTPUT_PIN (1) // TX Pin for the ESP32-CAM (GPIO 1) - used for timing output in analysis mode

//---Pin definitions---
#define MOTOR_LSPEED (12)
#define MOTOR_LDIR (13)
#define MOTOR_RSPEED (14)
#define MOTOR_RDIR (15)
#define DATA_IN (2)     // 74HC165 serial data input pin
#define DATA_OUT (4)    // 74HC595 serial data output pin
#define FLASH_LED (4)   // Onboard Camera Flash LED Pin, IO4
#define SHIFT_CLK (16)  // Shared clock for both shift registers
#define LATCH_PIN (3)   // Shared latch for both shift registers (RX pin)


#define SHIFT_LED_DISPLAY // Comment out to use WS2812b/NeoPixel display for directions instead of shift register display ring

#define CAMERA_ENABLE

#define ENABLE_DEBUG_LED // Use S/SE/SW LED bits for debugging


#define BLUETOOTH_CONTROLLER // Enables the connection of a Bluetooth controller, overriding the shift register controller. Comment out to disable
#define CONTROLLER_DEADZONE (100) // Joystick range of [-512, 512] will ignore any values below this threshold
#define CONTROLLER_A (0x0002)
#define CONTROLLER_B (0x0001)
#define CONTROLLER_X (0x0008)
#define CONTROLLER_Y (0x0004)

//---Motor Speed Configurations---
#define J_ROVER // Select Rover Configuration (J_ROVER or E_ROVER)
// Jordan's Rover
#ifdef J_ROVER
    #define LEFT_FORWARD (HIGH)
    #define RIGHT_FORWARD (LOW)
    #define L_N_SPD (100)
    #define R_N_SPD (90)
    #define L_NE_SPD (100)
    #define R_NE_SPD (0)
    #define L_E_SPD (100)
    #define R_E_SPD (100)
    #define L_SE_SPD (200)
    #define R_SE_SPD (100)
    #define L_S_SPD (255)
    #define R_S_SPD (235)
    #define L_SW_SPD (100)
    #define R_SW_SPD (200)
    #define L_W_SPD (100)
    #define R_W_SPD (100)
    #define L_NW_SPD (0)
    #define R_NW_SPD (90)
#elif defined(E_ROVER)
    #define LEFT_FORWARD (HIGH)
    #define RIGHT_FORWARD (LOW)
    #define L_N_SPD (100)
    #define R_N_SPD (90)
    #define L_NE_SPD (100)
    #define R_NE_SPD (0)
    #define L_E_SPD (100)
    #define R_E_SPD (100)
    #define L_SE_SPD (200)
    #define R_SE_SPD (100)
    #define L_S_SPD (255)
    #define R_S_SPD (235)
    #define L_SW_SPD (100)
    #define R_SW_SPD (200)
    #define L_W_SPD (100)
    #define R_W_SPD (100)
    #define L_NW_SPD (0)
    #define R_NW_SPD (90)
#else
    #error No Rover Configuration Selected
#endif
#define MOTOR_STOP (0)
#define LEFT_REVERSE (!LEFT_FORWARD)
#define RIGHT_REVERSE (!RIGHT_FORWARD)
#define LEFT_MIN_SPEED 0
#define LEFT_MAX_SPEED 255
#define RIGHT_MIN_SPEED 0
#define RIGHT_MAX_SPEED 255


// ***System Configurations***
// Do not modify these values unless you know what you're doing


// Button bit positions in the shift register (tested, calibrated)
enum ButtonBits {
    BTN_UP = 7,
    BTN_DOWN = 6,
    BTN_LEFT = 5,
    BTN_RIGHT = 4,
    BTN_MODE = 3
};
#define BTN_COUNT (5)

// LED bit positions in the shift register (tested, calibrated)
#ifdef SHIFT_LED_DISPLAY
    enum LEDBits {
        LED_NORTH = 7,
        LED_NORTHEAST = 6,
        LED_EAST = 0,
        LED_SOUTHEAST = 1,
        LED_SOUTH = 2,
        LED_SOUTHWEST = 3,
        LED_WEST = 4,
        LED_NORTHWEST = 5
    };
    #define LED_COUNT (8)
#else // NeoPixel/WS2812b Display LED ordering
    enum LEDBits {
        LED_NORTH = 1,
        LED_NORTHEAST = 15,
        LED_EAST = 13,
        LED_SOUTHEAST = 11,
        LED_SOUTH = 9,
        LED_SOUTHWEST = 7,
        LED_WEST = 5,
        LED_NORTHWEST = 3
    };
    #define LED_COUNT (8)
    #define PIXEL_COUNT (16)
    #warning    ---   Using WS2812b/NeoPixel Display Mode   ---
    #ifdef ENABLE_FLASH_LED
        #error Flash LED not currently supported with WS2812b display
    #endif
#endif


// CAMERA_MODEL_AI_THINKER pins
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22



// ***Preprocessor Checks & Updates***


//--Modify task priority if requested in analysis mode-- (breaks RM rules, but allows for analysis of the task in isolation)
#ifdef RM_ANALYSIS_MODE
    #warning    ---   Rate Monotonic Analysis Mode Enabled   ---
    #if RM_ANALYSIS_MODE == RM_FOCUS_SHIFT_REG && defined(RM_ANALYSIS_PRIORITY_OVERRIDE)
        #undef SHIFT_REG_SERVICE_PRIORITY
        #define SHIFT_REG_SERVICE_PRIORITY (RM_ANALYSIS_PRIORITY_OVERRIDE)
    #endif
    #if RM_ANALYSIS_MODE == RM_FOCUS_MOTOR && defined(RM_ANALYSIS_PRIORITY_OVERRIDE)
        #undef MOTOR_SERVICE_PRIORITY
        #define MOTOR_SERVICE_PRIORITY (RM_ANALYSIS_PRIORITY_OVERRIDE)
    #endif
    #if RM_ANALYSIS_MODE == RM_FOCUS_CAMERA && defined(RM_ANALYSIS_PRIORITY_OVERRIDE)
        #undef CAMERA_SERVICE_PRIORITY
        #define CAMERA_SERVICE_PRIORITY (RM_ANALYSIS_PRIORITY_OVERRIDE)
    #endif
    #if RM_ANALYSIS_MODE == RM_FOCUS_CONTROLLER && defined(RM_ANALYSIS_PRIORITY_OVERRIDE)
        #undef CONTROLLER_SERVICE_PRIORITY
        #define CONTROLLER_SERVICE_PRIORITY (RM_ANALYSIS_PRIORITY_OVERRIDE)
    #endif
#endif

//--Validate Motor Speed Configurations--
// Helper macro for range checking
#define CHECK_SPEED(val, min, max) static_assert((val) >= min && (val) <= max, "Motor speed " #val " out of range [" #min " - " #max "]")
// Check each speed definition
CHECK_SPEED(L_N_SPD, LEFT_MIN_SPEED, LEFT_MAX_SPEED);
CHECK_SPEED(R_N_SPD, RIGHT_MIN_SPEED, RIGHT_MAX_SPEED);
CHECK_SPEED(L_NE_SPD, LEFT_MIN_SPEED, LEFT_MAX_SPEED);
CHECK_SPEED(R_NE_SPD, RIGHT_MIN_SPEED, RIGHT_MAX_SPEED);
CHECK_SPEED(L_E_SPD, LEFT_MIN_SPEED, LEFT_MAX_SPEED);
CHECK_SPEED(R_E_SPD, RIGHT_MIN_SPEED, RIGHT_MAX_SPEED);
CHECK_SPEED(L_SE_SPD, LEFT_MIN_SPEED, LEFT_MAX_SPEED);
CHECK_SPEED(R_SE_SPD, RIGHT_MIN_SPEED, RIGHT_MAX_SPEED);
CHECK_SPEED(L_S_SPD, LEFT_MIN_SPEED, LEFT_MAX_SPEED);
CHECK_SPEED(R_S_SPD, RIGHT_MIN_SPEED, RIGHT_MAX_SPEED);
CHECK_SPEED(L_SW_SPD, LEFT_MIN_SPEED, LEFT_MAX_SPEED);
CHECK_SPEED(R_SW_SPD, RIGHT_MIN_SPEED, RIGHT_MAX_SPEED);
CHECK_SPEED(L_W_SPD, LEFT_MIN_SPEED, LEFT_MAX_SPEED);
CHECK_SPEED(R_W_SPD, RIGHT_MIN_SPEED, RIGHT_MAX_SPEED);
CHECK_SPEED(L_NW_SPD, LEFT_MIN_SPEED, LEFT_MAX_SPEED);
CHECK_SPEED(R_NW_SPD, RIGHT_MIN_SPEED, RIGHT_MAX_SPEED);


#endif // ROVER_CONFIG_H