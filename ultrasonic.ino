unsigned int getFrontUSDist()
{
    return pings[0].convert_cm(pings[0].ping_median(US_NUM_SAMPLE));
}

unsigned int getLeftUSDist()
{
    return pings[1].convert_cm(pings[1].ping_median(US_NUM_SAMPLE));
}

unsigned int getRightUSDist()
{
    return pings[2].convert_cm(pings[2].ping_median(US_NUM_SAMPLE));
}