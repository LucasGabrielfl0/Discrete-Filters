#ifndef _RECURSIVE_FILTERS_H_
#define _RECURSIVE_FILTERS_H_

float CumulativeAvg();
float SimpleMovingAvg();
float ExpMovingAvg();
float LowPassFilter();


#endif

// .c File

/*Cumulative Average*/
float CumulativeAvg()
{

    return 1;
}
/* Simple Moving Average */
float SimpleMovingAvg()
{
    float SM_Avg;

    return SM_Avg;
}
/* Exponential Moving Average */
float ExpMovingAvg()
{
    int X_avg[2];
    X_avg[0] = 0;   // Current value for the average
    X_avg[1] = 0;   // Previous value for the average;

    int Xk = 0;    // Value that changes in discrete steps of K
    int k = 0;    //  Number os iterations

    // Average Filter is X_avg = (k-1/k)*X_avg[1] +(1/k)*Xk , or:
    float alpha = (k - 1)/k;
    float EM_Avg = alpha*X_avg[1] +(1-alpha)*Xk;

    return EM_Avg;
}

/* First Order Low Pass Filter */
float LowPassFilter()
{

    return 1;
}