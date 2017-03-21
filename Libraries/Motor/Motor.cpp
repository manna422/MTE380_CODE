#include "Motor.h"
#include <Arduino.h>

Motor::Motor(Servo* left, Servo* right) {
    mLeft = left;
    mRight = right;
    rampStep = 10;
    lMotorDeadPos = 92;
    rMotorDeadPos = 92;
    currentLMotorSpeed = lMotorDeadPos;
    currentRMotorSpeed = rMotorDeadPos;
    Motor::stopBothMotors();
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

void Motor::setLeftMotorSpeed(float speed) {
    if (speed >= 0) {
        targetLMotorSpeed = int(map(speed, 0.0f, 100.0f, lMotorDeadPos, 0));
    } else {
        targetLMotorSpeed = int(map(-1*speed, 0.0f, 100.0f, lMotorDeadPos, 180));
    }
}

void Motor::setRightMotorSpeed(float speed) {
    if (speed >= 0) {
        targetRMotorSpeed = int(map(speed, 0.0f, 100.0f, rMotorDeadPos, 180));
    } else {
        targetRMotorSpeed = int(map(-1*speed, 0.0f, 100.0f, rMotorDeadPos, 0));
    }
}

void Motor::stopBothMotors() {
    Motor::setLeftMotorSpeed(0.0f);
    Motor::setRightMotorSpeed(0.0f);
}

void Motor::setRampStep(int step) {
    rampStep = step;
}
