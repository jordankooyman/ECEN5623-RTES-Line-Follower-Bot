// ESP32 IO Test Code
// Written by Jordan Kooyman
// Last modified on 4/17/2025

#include <Arduino.h>

// Pin definitions
#define MOTOR_LSPEED (12)
#define MOTOR_LDIR (13)
#define MOTOR_RSPEED (14)
#define MOTOR_RDIR (15)
#define DATA_IN (2)     // 74HC165 serial data input pin
#define DATA_OUT (4)    // 74HC595 serial data output pin
#define FLASH_LED (4)   // Onboard Camera Flash LED Pin
#define SHIFT_CLK (16)  // Shared clock for both shift registers
#define LATCH_PIN (3)   // Shared latch for both shift registers (RX pin)


// Constants
#define SHIFT_REG_BITS (8)


// Motor Speed Configurations
#define J_ROVER // Select Rover Configuration (J_ROVER or E_ROVER)
// Jordan's Rover
#ifdef J_ROVER
  #define LEFT_MAX_SPEED (200)
  #define LEFT_MIN_SPEED (100)
  #define RIGHT_MAX_SPEED (200)
  #define RIGHT_MIN_SPEED (100)
  #define LEFT_FORWARD (LOW)
  #define RIGHT_FORWARD (HIGH)
#elif defined(E_ROVER)
  #define LEFT_MAX_SPEED (255)
  #define LEFT_MIN_SPEED (70)
  #define RIGHT_MAX_SPEED (255)
  #define RIGHT_MIN_SPEED (70)
  #define LEFT_FORWARD (LOW)
  #define RIGHT_FORWARD (LOW)
#else
  #error No Rover Configuration Selected
#endif
#define MOTOR_STOP (0)
#define LEFT_REVERSE (!LEFT_FORWARD)
#define RIGHT_REVERSE (!RIGHT_FORWARD)


// Button bit positions in the shift register (tested, calibrated)
enum ButtonBits {
  BTN_UP = 7,
  BTN_DOWN = 6,
  BTN_LEFT = 5,
  BTN_RIGHT = 4,
  BTN_MODE = 3
};
#define BTN_COUNT (5)

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

// LED bit positions in the shift register (tested, calibrated)
#define SHIFT_LED_DISPLAY
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
#else
  #error No Output Display Mode Selected (NeoPixels NYI)
#endif

typedef unsigned char byte_t;


// Global Variables
byte_t ButtonBits;
#ifdef SHIFT_LED_DISPLAY
  byte_t LedBits; 
#endif



void setup() {
    // Initialize motor control pins
    pinMode(MOTOR_LSPEED, OUTPUT);
    pinMode(MOTOR_LDIR, OUTPUT);
    pinMode(MOTOR_RSPEED, OUTPUT);
    pinMode(MOTOR_RDIR, OUTPUT);

    // Initialize Camera Flash LED pin
    pinMode(FLASH_LED, OUTPUT);
    
    // Initialize shift register pins
    pinMode(DATA_IN, INPUT);
    pinMode(DATA_OUT, OUTPUT);
    pinMode(SHIFT_CLK, OUTPUT);
    pinMode(LATCH_PIN, OUTPUT);
    
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
    #ifdef SHIFT_LED_DISPLAY
      LedBits = 0; 
    #endif
}


void runShiftRegisters() {
    byte_t buttonStates = 0;
    #ifdef SHIFT_LED_DISPLAY
      byte_t ledStates = LedBits;
      byte_t prevPinState = digitalRead(FLASH_LED); // Preserve Flash Pin State afterwards
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
      digitalWrite(FLASH_LED, prevPinState);
    #endif

    ButtonBits = buttonStates;
}


byte_t parseButtons()
{
    byte_t buttonDirection = Stop;

    if (ButtonBits & (1 << BTN_UP))     buttonDirection += North;
    if (ButtonBits & (1 << BTN_DOWN))   buttonDirection += South;
    if (ButtonBits & (1 << BTN_LEFT))   buttonDirection += West;
    if (ButtonBits & (1 << BTN_RIGHT))  buttonDirection += East;
    if (ButtonBits & (1 << BTN_MODE))   buttonDirection += MotorsOff;

    return buttonDirection;
}


void controlMotors(byte_t dir)
{
    static bool RunMotors = true;
    static bool MotorStateChanged = false;

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
                leftSpeed = LEFT_MAX_SPEED;
                rightSpeed = RIGHT_MAX_SPEED;
                leftDir = LEFT_FORWARD;
                rightDir = RIGHT_FORWARD;
            }
            break;
        case East:
            LedBits |= (1 << LED_EAST);
            if (RunMotors)
            {
                leftSpeed = LEFT_MAX_SPEED;
                rightSpeed = RIGHT_MAX_SPEED;
                leftDir = LEFT_FORWARD;
                rightDir = RIGHT_REVERSE;
            }
            break;
        case South:
            LedBits |= (1 << LED_SOUTH);
            if (RunMotors)
            {
                leftSpeed = LEFT_MAX_SPEED;
                rightSpeed = RIGHT_MAX_SPEED;
                leftDir = LEFT_REVERSE;
                rightDir = RIGHT_REVERSE;
            }
            break;
        case West:
            LedBits |= (1 << LED_WEST);
            if (RunMotors)
            {
                leftSpeed = LEFT_MAX_SPEED;
                rightSpeed = RIGHT_MAX_SPEED;
                leftDir = LEFT_REVERSE;
                rightDir = RIGHT_FORWARD;
            }
            break;
        case NorthEast:
            LedBits |= (1 << LED_NORTHEAST);
            if (RunMotors)
            {
                leftSpeed = LEFT_MAX_SPEED;
                rightSpeed = RIGHT_MIN_SPEED;
                leftDir = LEFT_FORWARD;
                rightDir = RIGHT_FORWARD;
            }
            break;
        case SouthEast:
            LedBits |= (1 << LED_SOUTHEAST);
            if (RunMotors)
            {
                leftSpeed = LEFT_MAX_SPEED;
                rightSpeed = RIGHT_MIN_SPEED;
                leftDir = LEFT_REVERSE;
                rightDir = RIGHT_REVERSE;
            }
            break;
        case SouthWest:
            LedBits |= (1 << LED_SOUTHWEST);
            if (RunMotors)
            {
                leftSpeed = LEFT_MIN_SPEED;
                rightSpeed = RIGHT_MAX_SPEED;
                leftDir = LEFT_REVERSE;
                rightDir = RIGHT_REVERSE;
            }
            break;
        case NorthWest:
            LedBits |= (1 << LED_NORTHWEST);
            if (RunMotors)
            {
                leftSpeed = LEFT_MIN_SPEED;
                rightSpeed = RIGHT_MAX_SPEED;
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
}


void loop() {
    // Cycle Shift Register I/O
    runShiftRegisters();

    // Parse Button States
    controlMotors(parseButtons());
    
    // Small delay to debounce
    delay(50);
}
