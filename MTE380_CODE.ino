#include <Ultrasonic.h>
#include <Motor.h>
#include <NewPing.h>
#include <Servo.h>

/*
 *  Ultrasonic Defines
 */
#define US_MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define US_FRONT_TRIGGER_PIN 12
#define US_FRONT_ECHO_PIN 11
#define US_LEFT_TRIGGER_PIN 8
#define US_LEFT_ECHO_PIN 7
#define US_RIGHT_TRIGGER_PIN 6
#define US_RIGHT_ECHO_PIN 5

/*
 * Servo Defines
 */
#define MOTOR_LEFT_PWM_PIN 10
#define MOTOR_RIGHT_PWM_PIN 9

NewPing pings[3] = {
    NewPing(US_FRONT_TRIGGER_PIN, US_FRONT_ECHO_PIN, US_MAX_DISTANCE),
    NewPing(US_LEFT_TRIGGER_PIN, US_LEFT_ECHO_PIN, US_MAX_DISTANCE),
    NewPing(US_RIGHT_TRIGGER_PIN, US_RIGHT_ECHO_PIN, US_MAX_DISTANCE)
};

Servo lMotor;
Servo rMotor;

Motor motor(&lMotor, &rMotor);
Ultrasonic sonar(pings);

void setup() 
{
    lMotor.attach(lMotorPWMPin);
    rMotor.attach(rMotorPWMPin);
    motor.stopBothMotors();
    motor.setLeftMotorSpeed(10f);
    motor.setRightMotorSpeed(10f);
}

void loop()
{
    motor.update();
    delay(30);
}
