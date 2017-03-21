#ifndef MOTOR_H
#define MOTOR_H

#include <Servo.h>

class Motor {
    public:
        Motor(Servo* left, Servo* right);

        void update();
        void setLeftMotorSpeed(float speed);
        void setRightMotorSpeed(float speed);
        void stopBothMotors(); 
        void setRampStep(int step);

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