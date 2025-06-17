#include "PIDControl.h"

const float Kp = 1;  
const float Ki = 1;
const float Kd = 1;
const float Ts = 1;   // Sample Period (Seconds)

// Bilinear Transform
const float a0 = Kp + (Ts*Ki/2) + (2*Kd/Ts);
const float a1 = (Ki*Ts) - (4*Kd/Ts);
const float a2 = Kp + (Ts*Ki/2) + (2*Kd/Ts);

float Out[4];
float Error[4];

void BilinearTransf(float ref, float fdback)
{
  Error[0] = ref - fdback;
  Out[0] = Out[2] + Error[2]*a0 + Error[1]*a1 + Error[0]*a2;

  //Update Variables
  Out[3] = Out[2];
  Out[2] = Out[1];
  Out[1] = Out[0];

  Error[3] = Error[2];
  Error[2] = Error[1];
  Error[1] = Error[0];  
}


// Backwards Euler
const float b0 = Kd/Ts;
const float b1 = - Kp - (2*Kd/Ts);
const float b2 = Kp + (Ts*Ki) + (Kd/Ts);
float Out_e[3];
float Error_e[3];

void BackwardsEuler(float ref, float fdback)
{
  Error[0] = ref - fdback;
  Out_e[0] = Out_e[1] + Error_e[2]*b0 + Error_e[1]*b1 + Error_e[0]*b2;

  //Update Variables
  Out_e[3] = Out_e[2];
  Out_e[2] = Out_e[1];
  Out_e[1] = Out_e[0];

  Error_e[3] = Error_e[2];
  Error_e[2] = Error_e[1];
  Error_e[1] = Error_e[0];
}



/******************** */
// void PID_Tustin(float Error[3], float Out[3], float ref, float fdback);
void PID_Euler(int a, int b);


void PID_Tustin(float Error[3], float Out[3], float ref, float fdback){
  Error[0] = ref - fdback;
  Out[0] = Out[2] + Error[2]*a0 + Error[1]*a1 + Error[0]*a2;

  //Update Variables
  Out[3] = Out[2];
  Out[2] = Out[1];
  Out[1] = Out[0];

  Error[3] = Error[2];
  Error[2] = Error[1];
  Error[1] = Error[0];  


}