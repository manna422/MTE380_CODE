#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <NewPing.h>

/**
*   Ultrasonic utility class
*/
class Ultrasonic {
    public:
        // Insert US sensors in the order of 
        // front - left - right
        Ultrasonic(NewPing[] sensor);
        ~Ultrasonic();

        void getDistances(unsigned int* dFront, unsigned int* dLeft, unsigned int* dRight);     

    private:
        // TODO: remove unneeded consts
        const int MAX_DISTANCE = 200;
        const int FRONT_TRIGGER_PIN = 12;
        const int FRONT_ECHO_PIN = 11;
        const int LEFT_TRIGGER_PIN = 8;
        const int LEFT_ECHO_PIN = 7;
        const int RIGHT_TRIGGER_PIN = 6;
        const int RIGHT_ECHO_PIN = 5;
        const int NUM_US_SAMPLE = 3;
        const int CM_WALL_TO_RAMP = 34;
        const int CM_POLE_TO_WALL = 19;
        const int DETECT_TOLERANCE = 3;

        NewPing[] mSensor;
};

#endif