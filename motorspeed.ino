typedef enum 
{
    L_STOP = 92,
    L_FWD_SLOW = 85,
    L_FWD_50,
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
    R_FWD_50,
    R_FWD_75,
    R_FWD_MAX = 141,
    R_REV_25,
    R_REV_50,
    R_REV_75,
    R_REV_100
} RightMotorSpeed;

void calibrateMotors()
{
    motor.setLeftMotorSpeed(L_FWD_SLOW);
    motor.setRightMotorSpeed(R_FWD_SLOW);
    while(switchReading == LOW) {
        delay(30);
        update();
    }
}

void stopMotors()
{
    motor.setLeftMotorSpeed(L_STOP);
    motor.setRightMotorSpeed(R_STOP);
    while(switchReading == HIGH) {
        delay(30);
        update();
    }
}

