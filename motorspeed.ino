typedef enum 
{
    L_STOP = 92,
    L_FWD_SLOW = 85,
    L_FWD_50,
    L_FWD_75,
    L_FWD_100 = 10,
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
    R_FWD_100 = 141,
    R_REV_25,
    R_REV_50,
    R_REV_75,
    R_REV_100
} RightMotorSpeed;

void calibrateMotors()
{
    motor.setLeftMotorSpeed(L_FWD_SLOW);
    motor.setRightMotorSpeed(R_FWD_SLOW);
    while(1) {
        delay(30);
        update();
    }
}

