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

void calibrateMotors()
{
    motor.setLeftMotorSpeed(L_FWD_50);
    motor.setRightMotorSpeed(R_FWD_50);
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

void positionAlignToWall()
{
    unsigned int dist = sonar.getFrontDist();
    if (dist > 30) {
        motor.setLeftMotorSpeed(50);
        motor.setRightMotorSpeed(50);
    } 
}

void turnLeft(int degrees) 
{

}

void turnRight(int degrees) 
{
    
}

void topOfWall()
{
    motor.setLeftMotorSpeed(L_FWD_MAX);
    motor.setRightMotorSpeed(R_FWD_MAX);
    
    while(filter.getPitch() > 70 && filter.getPitch() < 110 && switchReading == LOW)
    {
      update();
    }
    
    motor.setLeftMotorSpeed(L_FWD_SLOW);
    motor.setRightMotorSpeed(R_FWD_SLOW);
    
    while(filter.getPitch() < 70 || filter.getPitch() > 110 && switchReading == LOW)
    {
      update();
    }
}
