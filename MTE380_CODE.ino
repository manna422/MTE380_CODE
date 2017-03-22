#include <I2Cdev.h>
#include <MPU6050.h>
#include <Ultrasonic.h>
#include <Motor.h>
#include <NewPing.h>
#include <Servo.h>
#include <Wire.h>
#include <MadgwickAHRS.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

/*
 *  Ultrasonic Defines
 */
#define US_MAX_DISTANCE 300 // Maximum distance (in cm) to ping.

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

#define US_NUM_SAMPLE 5
#define CM_WALL_TO_RAMP 34
#define CM_POLE_TO_WALL 19
#define DETECT_TOLERANCE 3

/*
 *  Servo Defines
 */
#define MOTOR_LEFT_PWM_PIN 9
#define MOTOR_RIGHT_PWM_PIN 6

/*
 *  Machine States
 */
typedef enum {
    ST_STOP = 0,
    ST_DRIVE_TO_WALL,
    STATE2,
    STATE3,
    STATE4,
    STATE5,
    ST_DEBUG
} States;

/*
 *  Object initialization
 */
NewPing pings[3] = 
{
    NewPing(US_FRONT_TRIGGER_PIN, US_FRONT_ECHO_PIN, US_MAX_DISTANCE),
    NewPing(US_LEFT_TRIGGER_PIN, US_LEFT_ECHO_PIN, US_MAX_DISTANCE),
    NewPing(US_RIGHT_TRIGGER_PIN, US_RIGHT_ECHO_PIN, US_MAX_DISTANCE)
};
Servo lMotor;
Servo rMotor;
Motor motor(&lMotor, &rMotor);
Ultrasonic sonar(pings);
MPU6050 gyroIMU;
Madgwick filter;

/*
 *  Global Variables
 */
int16_t ax, ay, az;
int16_t gx, gy, gz;
uint32_t timeNow;
uint32_t timePrev = 0;
float deltat = 0.0f;
/*
 *  On/Off Switch
 */
int switchPin = 53;
int switchHigh = 51;
int switchReading;
/*
 *  Set Initial State 
 */
States GLOBAL_STATE = ST_DEBUG;

void initPins()
{
    pinMode(switchPin, INPUT);
    digitalWrite(switchHigh, HIGH);
    digitalWrite(US_FRONT_POWER, HIGH);
    digitalWrite(US_LEFT_POWER, HIGH);
    digitalWrite(US_RIGHT_POWER, HIGH);
    digitalWrite(US_FRONT_GROUND, LOW);
    digitalWrite(US_LEFT_GROUND, LOW);
    digitalWrite(US_RIGHT_GROUND, LOW);
}

void setup() 
{
    initPins();

    Wire.begin();
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
    // deactivate internal pull-ups for twi
    // as per note from atmega8 manual pg167
    cbi(PORTC, 4);
    cbi(PORTC, 5);
#else
    // deactivate internal pull-ups for twi
    // as per note from atmega128 manual pg204
    cbi(PORTD, 0);
    cbi(PORTD, 1);
#endif
    Serial.begin(38400);
    sonar.setSampleSize(US_NUM_SAMPLE);
    gyroIMU.initialize();
    filter.begin(25);
    
    lMotor.attach(MOTOR_LEFT_PWM_PIN);
    rMotor.attach(MOTOR_RIGHT_PWM_PIN);
}

void loop()
{   
    update();  
    switch(GLOBAL_STATE)
    {
        case ST_STOP:
            stopMotors();
            break;
        case ST_DRIVE_TO_WALL:
            break;
        case STATE2:
            break;
        case STATE3:
            break;
        case STATE4:
            break;
        case STATE5:
            break;
        case ST_DEBUG:
          calibrateMotors();
          break;
        default: ;    
    }
    
}

void update() {
    gyroIMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    timeNow = micros();
    deltat = ((timeNow - timePrev)/1000000.0f);
    timePrev = timeNow;

    filter.updateIMU(gx*PI/180.0f,gy*PI/180.0f,gz*PI/180.0f,ax,ay,az,deltat);
    motor.update();
    printIMU();
    
    switchReading = digitalRead(switchPin);
    if (switchReading == HIGH)
        GLOBAL_STATE = ST_STOP;
    else
        GLOBAL_STATE = ST_DEBUG;
}

void printIMU() {
    Serial.print(filter.getRoll()); Serial.print("\t");
    Serial.print(filter.getPitch()); Serial.print("\t");
    Serial.println(filter.getYaw()); 
}

