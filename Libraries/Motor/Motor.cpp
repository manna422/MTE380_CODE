#include "Motor.h"
#include <Arduino.h>

Motor::Motor(Servo* left, Servo* right) {
    mLeft = left;
    mRight = right;
    rampStep = 10;
    currentLMotorSpeed = 92; //TODO verify motor stop
    currentRMotorSpeed = 92; //TODO verify motor stop
}

void Motor::update() {
    if (targetLMotorSpeed != currentLMotorSpeed) {
        if (targetLMotorSpeed > currentLMotorSpeed) {
            if (targetLMotorSpeed >= currentLMotorSpeed + rampStep) {
                currentLMotorSpeed += rampStep;
            } else {
                currentLMotorSpeed = targetLMotorSpeed;
            }
        } else {
            if (targetLMotorSpeed <= currentLMotorSpeed - rampStep) {
                currentLMotorSpeed -= rampStep;
            } else {
                currentLMotorSpeed = targetLMotorSpeed;
            }
        }

        mLeft->write(currentLMotorSpeed);
    }

    if (targetRMotorSpeed != currentRMotorSpeed) {
        if (targetRMotorSpeed > currentRMotorSpeed) {
            if (targetRMotorSpeed >= currentRMotorSpeed + rampStep) {
                currentRMotorSpeed += rampStep;
            } else {
                currentRMotorSpeed = targetRMotorSpeed;
            }
        } else {
            if (targetRMotorSpeed <= currentRMotorSpeed - rampStep) {
                currentRMotorSpeed -= rampStep;
            } else {
                currentLMotorSpeed = targetLMotorSpeed;
            }
        }

        mRight->write(currentRMotorSpeed);
    }
}

void Motor::setLeftMotorSpeed(int speed) {
    targetLMotorSpeed = speed;
}

void Motor::setRightMotorSpeed(int speed) {
    targetRMotorSpeed = speed;
}

void Motor::setRampStep(int step) {
    rampStep = step;
}
