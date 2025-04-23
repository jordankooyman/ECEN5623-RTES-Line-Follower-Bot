#include "motor_control.h"
#include "line_algorithm.h"
#include <driver/timer.h>

// Motor pin definitions
#define MOTOR_LSPEED (12)
#define MOTOR_LDIR (13)
#define MOTOR_RSPEED (14)
#define MOTOR_RDIR (15)
#define FLASH_LED (4)

// Line following motor duty cycle
#define AUTO_MOVE_PERIOD_MS 50
#define AUTO_MOVE_ON_MS     50
#define AUTO_MOVE_OFF_MS    (AUTO_MOVE_PERIOD_MS - AUTO_MOVE_ON_MS)

#define E_ROVER
#ifdef E_ROVER
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

hw_timer_t* motorTimer = NULL;
volatile bool motorEnable = false;
volatile bool motorPhaseOn = true;

void IRAM_ATTR toggleMotorPWM() {
    motorPhaseOn = !motorPhaseOn;
    motorEnable = motorPhaseOn;

    uint32_t nextDuration = motorPhaseOn ? AUTO_MOVE_ON_MS : AUTO_MOVE_OFF_MS;
    timerAlarmWrite(motorTimer, nextDuration * 1000, true);
}


void motor_setup() {
    pinMode(MOTOR_LSPEED, OUTPUT);
    pinMode(MOTOR_LDIR, OUTPUT);
    pinMode(MOTOR_RSPEED, OUTPUT);
    pinMode(MOTOR_RDIR, OUTPUT);

    analogWrite(MOTOR_LSPEED, MOTOR_STOP);
    digitalWrite(MOTOR_LDIR, LEFT_FORWARD);
    analogWrite(MOTOR_RSPEED, MOTOR_STOP);
    digitalWrite(MOTOR_RDIR, RIGHT_FORWARD);

    pinMode(FLASH_LED, OUTPUT);

    motorTimer = timerBegin(0, 80, true);  // 80 MHz / 80 = 1 µs per tick
    timerAttachInterrupt(motorTimer, &toggleMotorPWM, true);
    timerAlarmWrite(motorTimer, AUTO_MOVE_ON_MS * 1000, true);  // Start with ON phase
    timerAlarmEnable(motorTimer);
}

void controlMotors(byte_t dir, float angle) {

    //Serial.printf("Motor control: %.2f)", angle);

    static bool ManualControl = true;
    static bool MotorStateChanged = false;

    /*
    if (dir >= MotorsOff) {
        if (!MotorStateChanged) {
            ManualControl = !ManualControl;
            MotorStateChanged = true;
        }
        dir -= MotorsOff;
    } else {
        MotorStateChanged = false;
    }
*/

    byte_t leftSpeed = MOTOR_STOP;
    byte_t rightSpeed = MOTOR_STOP;
    byte_t leftDir = LEFT_FORWARD;
    byte_t rightDir = RIGHT_FORWARD;

    //digitalWrite(FLASH_LED, ManualControl ? HIGH : LOW);

   // if (!ManualControl) {

        const float ANGLE_THRESHOLD = 20.0f;

        if (!motorEnable || isnan(angle)) {
            // Stop motors if PWM is off or no line detected
            leftSpeed = MOTOR_STOP;
            rightSpeed = MOTOR_STOP;
        } else {
            // Line detected and PWM is ON — choose direction based on angle
            if (angle > ANGLE_THRESHOLD) {
                leftSpeed  = L_NW_SPD;
                rightSpeed = R_NW_SPD;
            } else if (angle < -ANGLE_THRESHOLD) {
                leftSpeed  = L_NE_SPD;
                rightSpeed = R_NE_SPD;
            } else {
                leftSpeed  = L_N_SPD;
                rightSpeed = R_N_SPD;
            }
            leftDir  = LEFT_FORWARD;
            rightDir = RIGHT_FORWARD;
        }
 /*   } else {
        // Manual drive
        switch (dir) {
            case North:     leftSpeed = L_N_SPD; rightSpeed = R_N_SPD; leftDir = LEFT_FORWARD; rightDir = RIGHT_FORWARD; break;
            case East:      leftSpeed = L_E_SPD; rightSpeed = R_E_SPD; leftDir = LEFT_FORWARD; rightDir = RIGHT_REVERSE; break;
            case South:     leftSpeed = L_S_SPD; rightSpeed = R_S_SPD; leftDir = LEFT_REVERSE; rightDir = RIGHT_REVERSE; break;
            case West:      leftSpeed = L_W_SPD; rightSpeed = R_W_SPD; leftDir = LEFT_REVERSE; rightDir = RIGHT_FORWARD; break;
            case NorthEast: leftSpeed = L_NE_SPD; rightSpeed = R_NE_SPD; leftDir = LEFT_FORWARD; rightDir = RIGHT_FORWARD; break;
            case SouthEast: leftSpeed = L_SE_SPD; rightSpeed = R_SE_SPD; leftDir = LEFT_REVERSE; rightDir = RIGHT_REVERSE; break;
            case SouthWest: leftSpeed = L_SW_SPD; rightSpeed = R_SW_SPD; leftDir = LEFT_REVERSE; rightDir = RIGHT_REVERSE; break;
            case NorthWest: leftSpeed = L_NW_SPD; rightSpeed = R_NW_SPD; leftDir = LEFT_FORWARD; rightDir = RIGHT_FORWARD; break;
            case Stop:
            default: break;
        }
    }
*/
        //Serial.printf("%6.2f | L: %3d (%s) | R: %3d (%s)\n", angle, leftSpeed,  leftDir  == LEFT_FORWARD  ? "FWD" : "REV", rightSpeed, rightDir == RIGHT_FORWARD ? "FWD" : "REV");



    analogWrite(MOTOR_LSPEED, leftSpeed);
    digitalWrite(MOTOR_LDIR, leftDir);
    analogWrite(MOTOR_RSPEED, rightSpeed);
    digitalWrite(MOTOR_RDIR, rightDir);
}

byte_t parseController(ControllerPtr gp) {
    byte_t buttonDirection = Stop;
    uint16_t btn = gp->buttons();
    int16_t ax = gp->axisX();

    if (btn & 0x0002) buttonDirection += North;  
    if (btn & 0x0001) buttonDirection += South;  
    if (btn & 0x0004) buttonDirection += MotorsOff;
    if (ax < -100)    buttonDirection += West;
    if (ax >  100)    buttonDirection += East;

    return buttonDirection;
}
