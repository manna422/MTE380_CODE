int lMotorDeadPos = 92;
int rMotorDeadPos = 92;
int targetLMotorSpeed = lMotorDeadPos;
int targetRMotorSpeed = rMotorDeadPos;
int currentLMotorSpeed = lMotorDeadPos;
int currentRMotorSpeed = rMotorDeadPos;
int rampStep = 10;

void setLeftMotorSpeed(int speed) {
    targetLMotorSpeed = speed;
}

void setRightMotorSpeed(int speed) {
    targetRMotorSpeed = speed;
}

int getCurrentLeftMotorSpeed() {
    return currentLMotorSpeed;
}

int getCurrentRightMotorSpeed() {
    return currentRMotorSpeed;
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
    
 
  Serial.print(" Left Motor:");
  printSpace(currentLMotorSpeed);
  Serial.print(currentLMotorSpeed);
  Serial.print(" Right Motor:");
  printSpace(currentRMotorSpeed);
  Serial.println(currentRMotorSpeed);
}

void stopBothMotors()
{
    targetLMotorSpeed = lMotorDeadPos;
    targetRMotorSpeed = rMotorDeadPos;
}
