#include <NewPing.h>
#include <Servo.h>
#include <Wire.h>
#include "MPU6050.h"

#define DEBUG_PRINT // Enable for sensor data prints

/*
 *  Pin Definitions 
 */
#define US_FRONT_POWER 23
#define US_FRONT_GROUND 29
#define US_FRONT_TRIGGER_PIN 25
#define US_FRONT_ECHO_PIN 27
#define US_LEFT_POWER 27
#define US_LEFT_GROUND 31
#define US_LEFT_TRIGGER_PIN 35
#define US_LEFT_ECHO_PIN 33
#define US_RIGHT_POWER 28
#define US_RIGHT_GROUND 22
#define US_RIGHT_TRIGGER_PIN 26
#define US_RIGHT_ECHO_PIN 24
#define MOTOR_LEFT_PWM_PIN 9
#define MOTOR_RIGHT_PWM_PIN 6
#define INT_PIN 19
#define SWITCH_PIN 53
#define SWITCH_POWER 51

/*
 *  Ultrasonic 
 */
#define US_MAX_DISTANCE 300 // Maximum distance (in cm) to ping.
#define US_NUM_SAMPLE 5
#define CM_WALL_TO_RAMP 34
#define CM_POLE_TO_WALL 19
#define DETECT_TOLERANCE 3

/*
 *  Preset Motor Speed 
 */
typedef enum 
{
    L_STOP = 92,
    L_FWD_SLOW = 84,
    L_FWD_50 = 55,
    L_FWD_75,
    L_FWD_MAX = 10,
    L_REV_25,
    L_REV_50,
    L_REV_75,
    L_REV_100
} LeftMotorSpeed;

typedef enum 
{
    R_STOP = 92,
    R_FWD_SLOW = 100,
    R_FWD_50 = 129,
    R_FWD_75,
    R_FWD_MAX = 141,
    R_REV_25,
    R_REV_50,
    R_REV_75,
    R_REV_100
} RightMotorSpeed;

/*
 *  Object Initialization
 */
NewPing pings[3] = 
{
    NewPing(US_FRONT_TRIGGER_PIN, US_FRONT_ECHO_PIN, US_MAX_DISTANCE),
    NewPing(US_LEFT_TRIGGER_PIN, US_LEFT_ECHO_PIN, US_MAX_DISTANCE),
    NewPing(US_RIGHT_TRIGGER_PIN, US_RIGHT_ECHO_PIN, US_MAX_DISTANCE)
};
Servo lMotor;
Servo rMotor;

/*
 *  Machine States
 */
typedef enum {
    ST_STOP = 0,
    ST_DRIVE_TO_WALL,
    ST_UP_WALL,
    ST_TOP_WALL,
    ST_DOWN_WALL_1,
    ST_DOWN_WALL_2,
    ST_POLE_DETECT,
    ST_DEBUG
} States;

/*
 *  Global Variables
 */
int switchReading;
States INITIAL_STATE = ST_DRIVE_TO_WALL;
States GLOBAL_STATE = INITIAL_STATE;
unsigned long STATE_START_TIME = 0;


/*
 *  Helper Functions
 */
void pinSetup()
{
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(INT_PIN, INPUT);
  digitalWrite(INT_PIN, LOW);

  pinMode(SWITCH_PIN, INPUT);
  digitalWrite(SWITCH_POWER, HIGH);

  digitalWrite(US_FRONT_GROUND, LOW);
  digitalWrite(US_LEFT_GROUND, LOW);
  digitalWrite(US_RIGHT_GROUND, LOW);
  digitalWrite(US_FRONT_POWER, HIGH);
  digitalWrite(US_LEFT_POWER, HIGH);
  digitalWrite(US_RIGHT_POWER, HIGH);

  lMotor.attach(MOTOR_LEFT_PWM_PIN);
  lMotor.write(L_STOP);
  rMotor.attach(MOTOR_RIGHT_PWM_PIN);
  rMotor.write(R_STOP);
}

// Call this whenever polling occurs
void update()
{
  switchReading = digitalRead(SWITCH_PIN);
  if (switchReading == HIGH)
  {
    GLOBAL_STATE = ST_STOP;
  } else 
  {
    if (GLOBAL_STATE == ST_STOP)
      GLOBAL_STATE = INITIAL_STATE;
  }
  
  imuUpdate();
  motorUpdate();
}

/*
 *  Main Loop */
void setup()
{
  Serial.begin(38400);
  Wire.begin();
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
  cbi(PORTC, 4);
  cbi(PORTC, 5);
#else
  cbi(PORTD, 0);
  cbi(PORTD, 1);
#endif  
  
  pinSetup();
  imuSetup();
}

void loop()
{  
  update();

  switch(GLOBAL_STATE)
  {
    case ST_STOP: 
    {
      stopBothMotors();
      break;
    } 
    case ST_DRIVE_TO_WALL:
      driveToWallState();
      break;
    case ST_UP_WALL:
      upWallState();
      break;
    case ST_TOP_WALL:
      topWallState();
      break;
    case ST_DOWN_WALL_1:
      downWall1State();
      break;
    case ST_DOWN_WALL_2:
      downWall2State();
      break;
    case ST_POLE_DETECT:
      break;
    case ST_DEBUG:
    {
      setLeftMotorSpeed(L_FWD_50);
      setRightMotorSpeed(R_FWD_50);
      break;
    } 
    default: 
      break;    
  }
}
