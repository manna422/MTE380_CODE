#include "ultrasonic.h"

Ultrasonic::Ultrasonic(NewPings[] sensors) 
    : mSensor(sensors) 
{    
}

unsigned int Ultrasonic::getDistance(unsigned int* dFront, unsigned int* dLeft, unsigned int* dRight)
{
    *dFront = mSensors[0].convert_cm(mSensors[0].ping_median(NUM_US_SAMPLE));
    *dLeft = mSensors[1].convert_cm(mSensors[1].ping_median(NUM_US_SAMPLE));
    *dRight = mSensors[2].convert_cm(mSensors[2].ping_median(NUM_US_SAMPLE));
}