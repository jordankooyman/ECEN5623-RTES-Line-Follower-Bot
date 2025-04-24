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
#include <Bluepad32.h>
#include "esp_camera.h"


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


#ifdef BLUETOOTH_CONTROLLER
// Global controller pointer
ControllerPtr activeController = nullptr;

// Controller connected callback
void onConnectedController(ControllerPtr ctl) {

    if (!activeController) {
        activeController = ctl;
        
    } else {
        ctl->disconnect();  // Force disconnect extra controllers
    }
}

// Controller disconnected callback
void onDisconnectedController(ControllerPtr ctl) {
    if (activeController == ctl) {
        activeController = nullptr;
    }
}
#endif

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
    pinMode(SHIFT_CLK, OUTPUT);   // This line appears to interfere with camera capture
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


    #ifdef BLUETOOTH_CONTROLLER
        BP32.setup(&onConnectedController, &onDisconnectedController);
        //BP32.forgetBluetoothKeys();  // Clear old pairings
    #endif

    /*
    #ifdef BLUETOOTH_CONTROLLER
        BP32.setup(&onConnectedController, &onDisconnectedController);

        while (!activeController || !activeController->isConnected()) {
            BP32.update();  
            delay(50);     
        }
        //BP32.forgetBluetoothKeys();  // Clear old pairings
    #endif
*/

/*
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = FRAMESIZE_96X96;
    config.jpeg_quality = 12;
    config.fb_count = 3;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;  

    // Init camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
       // Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }
    sensor_t *s = esp_camera_sensor_get();
*/
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
        if (
            !autonomousMode 
            #ifdef BLUETOOTH_CONTROLLER // But if we have an active bluetooth controller, let it drive
            && (!activeController || !activeController->isConnected())
          #endif
        ) {
            motorCommand.setMotorSpeed(buttonDirection, MANUAL_CONTROL_TIMEOUT);
        }
        

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
 * The function `get_line_angle_from_frame` is a helper function for vPrvCameraParse
 * It accepts a frame buffer and, using average brightness, detects a line
 * and returns an estimated angle at which the rover must turn for the line to 
 * become centered in the frame
 * Written by Eric Percin
 * 
 * @param fb A camera_fb_t frame buffer, currently grayscale and 96x96
 * @return A float angle in degrees representing the direction to turn.
 *         Positive values indicate turning right, negative values indicate
 *         turning left. Returns NAN if no line is detected, or frame is invalid.
 */
float get_line_angle_from_frame(camera_fb_t *fb) {
    if (!fb || fb->format != PIXFORMAT_GRAYSCALE || !fb->buf || fb->len == 0)
    return NAN; // Invalid frame

    const int w = fb->width;
    const int h = fb->height;
    const int scan_start = h - 20;
    const int scan_end = h - 10;

    uint32_t weighted_sum = 0;
    uint32_t count = 0;

    // Scan for dark pixels
    for (int y = scan_start; y <= scan_end; ++y) {
        for (int x = 0; x < w; ++x) {
            int idx = y * w + x;
            if (idx >= fb->len) break;
            if (fb->buf[idx] < 100) {
                weighted_sum += x;
                count++;
            }
        }
    }

    if (count == 0) 
    return NAN; // No line found
    
    // Compute angle from center
    float center_x = float(weighted_sum) / float(count);
    float error = (center_x - (w / 2.0f)) / (w / 2.0f);
    float angle = error * 45.0f;

    return angle;
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

/*

        // Camera parsing logic
        const float ANGLE_THRESHOLD = 20.0f;

        camera_fb_t *fb = esp_camera_fb_get();

        if (fb)
        {
            float angle = get_line_angle_from_frame(fb);

            byte_t direction = Stop;
            if (autonomousMode && !isnan(angle))
            {
                // Convert angle to direction 
                if (angle > ANGLE_THRESHOLD) direction = NorthEast;
                else if (angle < -ANGLE_THRESHOLD) direction = NorthWest;
                else direction = North;

                motorCommand.setMotorSpeed(direction, MANUAL_CONTROL_TIMEOUT);
            }

            // Return the frame buffer to the driver
            esp_camera_fb_return(fb);
        }
*/

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

        // Bluetooth controller parsing logic
        byte_t buttonDirection = Stop;
        static bool buttonStateChanged = false;

        BP32.update();

        if (activeController && activeController->isConnected())
        {
            uint16_t btn = activeController->buttons();
            int16_t  ax  = activeController->axisX();  // [–512, 512]
        
            if (btn & CONTROLLER_A) buttonDirection += North;     // A = Forward
            if (btn & CONTROLLER_B) buttonDirection += South;     // B = Reverse
            //if (btn & CONTROLLER_Y)                             // Y is currently unused
            if (ax < -CONTROLLER_DEADZONE) buttonDirection += West;
            if (ax > CONTROLLER_DEADZONE) buttonDirection += East;    

            // X toggles autonomous mode (once per press)
            if (btn & CONTROLLER_X)
            {
                if (!buttonStateChanged)
                {
                    buttonStateChanged = true;
                    autonomousMode = !autonomousMode;
                }
            }
            else
            {
                buttonStateChanged = false;
            }

            if(!autonomousMode)
            motorCommand.setMotorSpeed(buttonDirection, MANUAL_CONTROL_TIMEOUT);
        }

        #ifdef RM_ANALYSIS_MODE
            #if RM_ANALYSIS_MODE == RM_FOCUS_CONTROLLER
                digitalWrite(RM_OUTPUT_PIN, LOW); // Set the output pin high to indicate the start of the task
            #endif
        #endif

        // Delay until next period
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}
