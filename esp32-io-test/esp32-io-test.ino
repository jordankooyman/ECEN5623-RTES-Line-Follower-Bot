// ESP32 IO Test Code
// Written by Jordan Kooyman
// Last modified on 4/16/2025

#include <Arduino.h>
// Remaining todo for this test code:
// * Motor control

// Pin definitions
#define MOTOR_L1 (12)
#define MOTOR_L0 (13)
#define MOTOR_R1 (15)
#define MOTOR_R0 (14)
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
  #define LEFT_MAX_SPEED (255)
  #define LEFT_MIN_SPEED (0)
  #define RIGHT_MAX_SPEED (255)
  #define RIGHT_MIN_SPEED (0)
#elif defined(E_ROVER)
  #define LEFT_MAX_SPEED (255)
  #define LEFT_MIN_SPEED (0)
  #define RIGHT_MAX_SPEED (255)
  #define RIGHT_MIN_SPEED (0)
#else
  #error No Rover Configuration Selected
#endif
#define MOTOR_STOP (0)


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
    pinMode(MOTOR_L1, OUTPUT);
    pinMode(MOTOR_L0, OUTPUT);
    pinMode(MOTOR_R1, OUTPUT);
    pinMode(MOTOR_R0, OUTPUT);

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
    digitalWrite(MOTOR_L1, LOW);
    digitalWrite(MOTOR_L0, LOW);
    digitalWrite(MOTOR_R1, LOW);
    digitalWrite(MOTOR_R0, LOW);
    
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
  bool btnUp = ButtonBits & (1 << BTN_UP);
  bool btnDown = ButtonBits & (1 << BTN_DOWN);
  bool btnLeft = ButtonBits & (1 << BTN_LEFT);
  bool btnRight = ButtonBits & (1 << BTN_RIGHT);
  bool btnMode = ButtonBits & (1 << BTN_MODE);

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

  LedBits = 0;
  switch(dir)
  {
    case North:
      LedBits |= (1 << LED_NORTH);
      if (RunMotors)
      {
        analogWrite(MOTOR_L1, 255);
        analogWrite(MOTOR_L0, 0);
        analogWrite(MOTOR_R1, 255);
        analogWrite(MOTOR_R0, 0);
      }
      break;
    case East:
      LedBits |= (1 << LED_EAST);
      if (RunMotors)
      {
        analogWrite(MOTOR_L1, 255);
        analogWrite(MOTOR_L0, 0);
        analogWrite(MOTOR_R1, 255);
        analogWrite(MOTOR_R0, 0);
      }
      break;
    case South:
      LedBits |= (1 << LED_SOUTH);
      if (RunMotors)
      {
        analogWrite(MOTOR_L1, 255);
        analogWrite(MOTOR_L0, 0);
        analogWrite(MOTOR_R1, 255);
        analogWrite(MOTOR_R0, 0);
      }
      break;
    case West:
      LedBits |= (1 << LED_WEST);
      if (RunMotors)
      {
        analogWrite(MOTOR_L1, 255);
        analogWrite(MOTOR_L0, 0);
        analogWrite(MOTOR_R1, 255);
        analogWrite(MOTOR_R0, 0);
      }
      break;
    case NorthEast:
      LedBits |= (1 << LED_NORTHEAST);
      if (RunMotors)
      {
        analogWrite(MOTOR_L1, 255);
        analogWrite(MOTOR_L0, 0);
        analogWrite(MOTOR_R1, 255);
        analogWrite(MOTOR_R0, 0);
      }
      break;
    case SouthEast:
      LedBits |= (1 << LED_SOUTHEAST);
      if (RunMotors)
      {
        analogWrite(MOTOR_L1, 255);
        analogWrite(MOTOR_L0, 0);
        analogWrite(MOTOR_R1, 255);
        analogWrite(MOTOR_R0, 0);
      }
      break;
    case SouthWest:
      LedBits |= (1 << LED_SOUTHWEST);
      if (RunMotors)
      {
        analogWrite(MOTOR_L1, 255);
        analogWrite(MOTOR_L0, 0);
        analogWrite(MOTOR_R1, 255);
        analogWrite(MOTOR_R0, 0);
      }
      break;
    case NorthWest:
      LedBits |= (1 << LED_NORTHWEST);
      if (RunMotors)
      {
        analogWrite(MOTOR_L1, 255);
        analogWrite(MOTOR_L0, 0);
        analogWrite(MOTOR_R1, 255);
        analogWrite(MOTOR_R0, 0);
      }
      break;
    case Stop:
    default: // Conflicting Directions Requested, Stop
      analogWrite(MOTOR_L1, MOTOR_STOP);
      analogWrite(MOTOR_L0, MOTOR_STOP);
      analogWrite(MOTOR_R1, MOTOR_STOP);
      analogWrite(MOTOR_R0, MOTOR_STOP);
  }
}


void loop() {
    // Cycle Shift Register I/O
    runShiftRegisters();

    // Parse Button States
    controlMotors(parseButtons());
    
    // Small delay to debounce
    delay(50);
}
