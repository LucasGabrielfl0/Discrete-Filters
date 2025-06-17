#ifndef _PID_CONTROL_H_
#define _PID_CONTROL_H_



void PID_Tustin(int a, int b);
void PID_Euler(int a, int b);
void BilinearTransf(float ref, float fdback);
void BackwardsEuler(float ref, float fdback);

#endif