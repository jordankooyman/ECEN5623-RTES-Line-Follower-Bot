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

// Motor Speed Configurations
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