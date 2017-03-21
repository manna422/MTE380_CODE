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
        Ultrasonic(NewPing sensors[3]);

        unsigned int getFrontDist();
        unsigned int getLeftDist();
        unsigned int getRightDist();
        void getAllDist(unsigned int* dFront, unsigned int* dLeft, unsigned int* dRight);     
        void setSampleSize(int num);
        
    private:
        NewPing* mSensor;
        int mSampleSize;
};

#endif
    