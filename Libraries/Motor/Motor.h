#ifndef MOTOR_H
#define MOTOR_H

#include <Servo.h>

class Motor {
    public:
        Motor(Servo* left, Servo* right);

        void update();
        void setLeftMotorSpeed(int speed);
        void setRightMotorSpeed(int speed);
        void setRampStep(int step);
        int getLTargetSpeed();
        int getRTargetSpeed();
        int getLCurrentSpeed();
        int getRCurrentSpeed();
        
    private:
        Servo* mLeft;
        Servo* mRight;
        int rampStep;
        int targetLMotorSpeed;
        int targetRMotorSpeed;
        int currentLMotorSpeed;
        int currentRMotorSpeed;
        int lMotorDeadPos;
        int rMotorDeadPos;
};

#endif