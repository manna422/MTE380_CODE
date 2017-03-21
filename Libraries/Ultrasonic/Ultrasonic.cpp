#include "Ultrasonic.h"

Ultrasonic::Ultrasonic(NewPing sensors[3]) 
{
    mSensor = sensors;    
    mSampleSize = 3;
}

unsigned int Ultrasonic::getFrontDist()
{
    return mSensor[0].convert_cm(mSensor[0].ping_median(mSampleSize));
}

unsigned int Ultrasonic::getLeftDist()
{
    return mSensor[1].convert_cm(mSensor[1].ping_median(mSampleSize));
}

unsigned int Ultrasonic::getRightDist()
{
    return mSensor[2].convert_cm(mSensor[2].ping_median(mSampleSize));
}

void Ultrasonic::getAllDist(unsigned int* dFront, unsigned int* dLeft, unsigned int* dRight)
{
    *dFront = mSensor[0].convert_cm(mSensor[0].ping_median(mSampleSize));
    *dLeft = mSensor[1].convert_cm(mSensor[1].ping_median(mSampleSize));
    *dRight = mSensor[2].convert_cm(mSensor[2].ping_median(mSampleSize));
}

void Ultrasonic::setSampleSize(int num) 
{
    mSampleSize = num;
}
