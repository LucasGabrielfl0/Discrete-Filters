// Recursive Average
int X_avg[2];
X_avg[0] = 0;   // Current value for the average
X_avg[1] = 0;   // Previous value for the average;

int Xk = 0;    // Value that changes in discrete steps of K
int k = 0;    //  Number os iterations

// Average Filter is X_avg = (k-1/k)*X_avg[1] +(1/k)*Xk , or:
int alpha = (k=1)/k
int X_avg = alpha*X_avg[1] +(1-alpha)*Xk;
