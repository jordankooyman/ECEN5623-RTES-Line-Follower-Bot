// ESP32 Controller Auto Code
// Written by Jordan Kooyman and Eric Percin
// Last modified on 4/21/2025
// Connects to a bluetooth controller instead of the normal shift controller

#include <Arduino.h>
#include <math.h>
#include "esp_camera.h"
#include <Bluepad32.h>


#define DEBUG 1        // set to 0 to disable all debug printing
#if DEBUG
  #define DEBUG_BEGIN(baud)   Serial.begin(baud)
  #define DEBUG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define DEBUG_BEGIN(baud)
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif


// Pin definitions
#define MOTOR_LSPEED (12)
#define MOTOR_LDIR (13)
#define MOTOR_RSPEED (14)
#define MOTOR_RDIR (15)
//#define DATA_IN (2)     // 74HC165 serial data input pin
//#define DATA_OUT (4)    // 74HC595 serial data output pin
#define FLASH_LED (4)   // Onboard Camera Flash LED Pin
//#define SHIFT_CLK (16)  // Shared clock for both shift registers
//#define LATCH_PIN (3)   // Shared latch for both shift registers (RX pin)

// From camera_pins.h:
#define PWDN_GPIO_NUM    32
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27
#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22


// Constants
//#define SHIFT_REG_BITS (8)


// Bluetooth controller definitions and globals
#define DEADZONE 100
ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

// Motor Speed Configurations
#define E_ROVER // Select Rover Configuration (J_ROVER or E_ROVER)
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
    #define LEFT_FORWARD (LOW)
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



void initCamera() {
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
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);

  //Slow flash -> init failed. Rapid flash -> ok!
  if (err != ESP_OK) {
    // halt + blink only the flash LED
    while (true) {
      digitalWrite(FLASH_LED, HIGH);
      delay(900);
      digitalWrite(FLASH_LED, LOW);
      delay(100);
    }
  }
  else {
    for (int i = 0; i < 5; i++) {
      digitalWrite(FLASH_LED, HIGH);
      delay(100);
      digitalWrite(FLASH_LED, LOW);
      delay(100);
    }

  }
}


void onConnectedController(ControllerPtr ctl) {
  DEBUG_PRINTLN("Controller connected!");
    for (int i = 0; i < 5; i++) {
      digitalWrite(FLASH_LED, HIGH);
      delay(100);
      digitalWrite(FLASH_LED, LOW);
      delay(100);
    }
  for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
    if (!myControllers[i]) {
      myControllers[i] = ctl;
      DEBUG_PRINT("Controller slot "); Serial.print(i); Serial.println(" connected");
      break;
    }
  }
}
void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      DEBUG_PRINT("Controller slot "); Serial.print(i); Serial.println(" disconnected");
      break;
    }
  }
}

void setup() {

    DEBUG_BEGIN(9600);
    
    pinMode(FLASH_LED, OUTPUT);

    DEBUG_PRINTLN("Initializing camera...");
    initCamera();
    DEBUG_PRINTLN("Camera initialized.");

  
    // Initialize motor control pins
    pinMode(MOTOR_LSPEED, OUTPUT);
    pinMode(MOTOR_LDIR, OUTPUT);
    pinMode(MOTOR_RSPEED, OUTPUT);
    pinMode(MOTOR_RDIR, OUTPUT);

    // Initialize Camera Flash LED pin
    pinMode(FLASH_LED, OUTPUT);
    
    // Initialize shift register pins
    //pinMode(DATA_IN, INPUT);
    //pinMode(DATA_OUT, OUTPUT);
    //pinMode(SHIFT_CLK, OUTPUT);
    //pinMode(LATCH_PIN, OUTPUT);
    
    // Set default states
    //digitalWrite(SHIFT_CLK, HIGH);  // Clock idle state
    //digitalWrite(LATCH_PIN, HIGH);  // Latch idle state
    
    // Stop motors initially
    analogWrite(MOTOR_LSPEED, MOTOR_STOP);
    digitalWrite(MOTOR_LDIR, LEFT_FORWARD);
    analogWrite(MOTOR_RSPEED, MOTOR_STOP);
    digitalWrite(MOTOR_RDIR, RIGHT_FORWARD);

//    ButtonBits = 0;
//    #ifdef SHIFT_LED_DISPLAY
//        LedBits = 0; 
//    #endif

    DEBUG_PRINTLN("Hello world!");

    // Bluepad32 init
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();

}


float detectLine() {
    //DEBUG_PRINTLN("detectLine(): start");
    // grab frame
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        DEBUG_PRINTLN("  !fb (NULL)");
        return NAN;
    }
    //DEBUG_PRINT("  fb len=");    DEBUG_PRINT(fb->len);
    //DEBUG_PRINT(" w=");          DEBUG_PRINT(fb->width);
    //DEBUG_PRINT(" h=");          DEBUG_PRINTLN(fb->height);

    // check format
    if (fb->format != PIXFORMAT_GRAYSCALE || !fb->buf || fb->len == 0) {
        DEBUG_PRINTLN("  invalid format or empty buffer");
        esp_camera_fb_return(fb);
        return NAN;
    }

    int w = fb->width;
    int h = fb->height;
    // sanity check dimensions
    if (w <= 0 || h <= 0 || (size_t)w * h > fb->len) {
        //DEBUG_PRINT("  bad dims w=");         DEBUG_PRINT(w);
        //DEBUG_PRINT(" h=");                   DEBUG_PRINT(h);
        //DEBUG_PRINT(" buf_len=");             DEBUG_PRINTLN(fb->len);
        esp_camera_fb_return(fb);
        return NAN;
    }

    // scan for dark pixels
    int scan_start = h - 20, scan_end = h - 10;
    uint32_t weighted_sum = 0, count = 0;
    for (int y = scan_start; y <= scan_end; y++) {
        int row = y * w;
        for (int x = 0; x < w; x++) {
            int idx = row + x;
            if (idx >= fb->len) break;
            if (fb->buf[idx] < 100) {
                weighted_sum += x;
                count++;
            }
        }
    }
    //DEBUG_PRINT("  count=");        DEBUG_PRINT(count);
    //DEBUG_PRINT(" weighted_sum=");  DEBUG_PRINTLN(weighted_sum);

    // return fb and check if we found anything
    esp_camera_fb_return(fb);
    if (count == 0) {
        //DEBUG_PRINTLN("  no dark pixels");
        return NAN;
    }

    // compute angle
    float center_x = (float)weighted_sum / count;
    float error    = (center_x - (w / 2.0f)) / (w / 2.0f);
    float angle    = error * 45.0f;
    DEBUG_PRINT("  angle=");        DEBUG_PRINTLN(angle);
    return angle;
}



// Map the angle from detectLine to motor movement
void steerLineFollower(float angle) {
  // angle: –45…+45 degrees
  const int baseSpeed = 150;
  // convert ±45° → ±baseSpeed
  int correction = (int)(angle / 45.0f * baseSpeed);
  int ls = baseSpeed + correction;
  int rs = baseSpeed - correction;

  // directions
  if (ls >= 0) {
    digitalWrite(MOTOR_LDIR, LEFT_FORWARD);
  } else {
    digitalWrite(MOTOR_LDIR, LEFT_REVERSE);
    ls = -ls;
  }
  if (rs >= 0) {
    digitalWrite(MOTOR_RDIR, RIGHT_FORWARD);
  } else {
    digitalWrite(MOTOR_RDIR, RIGHT_REVERSE);
    rs = -rs;
  }

  // clamp and set
  analogWrite(MOTOR_LSPEED, constrain(ls, 0, 255));
  analogWrite(MOTOR_RSPEED, constrain(rs, 0, 255));
}

/*
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
*/

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


byte_t parseController(ControllerPtr gp) {
    byte_t buttonDirection = Stop;

    uint16_t btn = gp->buttons();
    int16_t  ax  = gp->axisX();  // –512…+512

    // A → forward
    if (btn & 0x0002) buttonDirection += North;  

    // B → backward
    if (btn & 0x0001) buttonDirection += South;  

    // X → MotorsOff (toggle manual/auto in your controlMotors logic)
    if (btn & 0x0004) buttonDirection += MotorsOff;

    // Stick left/right → steer
    if (ax <  -DEADZONE) buttonDirection += West;
    if (ax >   DEADZONE) buttonDirection += East;

    return buttonDirection;
}


void controlMotors(byte_t dir)
{
    // toggle manual <-> auto on MODE press
    static bool ManualControl   = true;
    static bool MotorStateChanged = false;
    

    if (dir >= MotorsOff) {
        if (!MotorStateChanged) {
            ManualControl = !ManualControl;
            MotorStateChanged = true;
        }
        dir -= MotorsOff;
    } else {
        MotorStateChanged = false;
    }

    digitalWrite(FLASH_LED, ManualControl ? HIGH : LOW);

    // AUTO mode: line‑following takes over
    if (!ManualControl) {
      float turn_angle = detectLine();
      //float turn_angle = 0.0f;
      if (isnan(turn_angle)) {
        LedBits = (1 << LED_SOUTH);
      }
      else if (turn_angle < -10.0f) {
        LedBits = (1 << LED_EAST);
      }
      else if (turn_angle > 10.0f) {
        LedBits = (1 << LED_WEST);
      }
      else {
        LedBits = (1 << LED_NORTH);
      }         
        //steerLineFollower(turn_angle);            
        return;
    }

    // MANUAL mode: original button‑based control
    byte_t leftSpeed  = MOTOR_STOP;
    byte_t rightSpeed = MOTOR_STOP;
    byte_t leftDir    = LEFT_FORWARD;
    byte_t rightDir   = RIGHT_FORWARD;

    LedBits = 0;
    switch(dir)
    {
        case North:
            LedBits |= (1 << LED_NORTH);
            leftSpeed  = L_N_SPD;
            rightSpeed = R_N_SPD;
            leftDir    = LEFT_FORWARD;
            rightDir   = RIGHT_FORWARD;
            break;

        case East:
            LedBits |= (1 << LED_EAST);
            leftSpeed  = L_E_SPD;
            rightSpeed = R_E_SPD;
            leftDir    = LEFT_FORWARD;
            rightDir   = RIGHT_REVERSE;
            break;

        case South:
            LedBits |= (1 << LED_SOUTH);
            leftSpeed  = L_S_SPD;
            rightSpeed = R_S_SPD;
            leftDir    = LEFT_REVERSE;
            rightDir   = RIGHT_REVERSE;
            break;

        case West:
            LedBits |= (1 << LED_WEST);
            leftSpeed  = L_W_SPD;
            rightSpeed = R_W_SPD;
            leftDir    = LEFT_REVERSE;
            rightDir   = RIGHT_FORWARD;
            break;

        case NorthEast:
            LedBits |= (1 << LED_NORTHEAST);
            leftSpeed  = L_NE_SPD;
            rightSpeed = R_NE_SPD;
            leftDir    = LEFT_FORWARD;
            rightDir   = RIGHT_FORWARD;
            break;

        case SouthEast:
            LedBits |= (1 << LED_SOUTHEAST);
            leftSpeed  = L_SE_SPD;
            rightSpeed = R_SE_SPD;
            leftDir    = LEFT_REVERSE;
            rightDir   = RIGHT_REVERSE;
            break;

        case SouthWest:
            LedBits |= (1 << LED_SOUTHWEST);
            leftSpeed  = L_SW_SPD;
            rightSpeed = R_SW_SPD;
            leftDir    = LEFT_REVERSE;
            rightDir   = RIGHT_REVERSE;
            break;

        case NorthWest:
            LedBits |= (1 << LED_NORTHWEST);
            leftSpeed  = L_NW_SPD;
            rightSpeed = R_NW_SPD;
            leftDir    = LEFT_FORWARD;
            rightDir   = RIGHT_FORWARD;
            break;

        case Stop:
        default:
            // leave speeds at 0
            break;
    }

    // apply to pins
    analogWrite(MOTOR_LSPEED, leftSpeed);
    digitalWrite(MOTOR_LDIR,   leftDir);
    analogWrite(MOTOR_RSPEED,  rightSpeed);
    digitalWrite(MOTOR_RDIR,   rightDir);
}



void loop() {
    //runShiftRegisters();

    BP32.update();

    for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
        ControllerPtr gp = myControllers[i];
        if (gp && gp->isConnected()) {
            byte_t dir = parseController(gp);
            controlMotors(dir);
            break; 
        }
    }

    // 4) small debounce delay
    delay(50);
}
