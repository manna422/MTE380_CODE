#include "ultrasonic.h"

Ultrasonic::Ultrasonic(NewPings sensors[3]) 
{
    mSensor = sensors;    
    mSampleSize = 3;
}

unsigned int Ultrasonic::getDistances(unsigned int* dFront, unsigned int* dLeft, unsigned int* dRight)
{
    *dFront = mSensors[0].convert_cm(mSensors[0].ping_median(mSampleSize));
    *dLeft = mSensors[1].convert_cm(mSensors[1].ping_median(mSampleSize));
    *dRight = mSensors[2].convert_cm(mSensors[2].ping_median(mSampleSize));
}

void Ultrasonic::setSampleSize(int num) 
{
    mSampleSize = num;
}