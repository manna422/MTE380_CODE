int targetLMotorSpeed;
int targetRMotorSpeed;
int currentLMotorSpeed;
int currentRMotorSpeed;
int lMotorDeadPos = 92;
int rMotorDeadPos = 92;
int rampStep = 10;

void setLeftMotorSpeed(int speed) {
    targetLMotorSpeed = speed;
}

void setRightMotorSpeed(int speed) {
    targetRMotorSpeed = speed;
}

void motorUpdate() {
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

        lMotor.write(currentLMotorSpeed);
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
                currentRMotorSpeed = targetRMotorSpeed;
            }
        }

        rMotor.write(currentRMotorSpeed);
    }
}

void stopBothMotors()
{
    targetLMotorSpeed = lMotorDeadPos;
    targetRMotorSpeed = rMotorDeadPos;
}