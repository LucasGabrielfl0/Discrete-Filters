// Bilinear Transform
const float Kp = 1;  
const float Ki = 1;
const float Kd = 1;
const float Ts = 1;   // Sample Period (Seconds)

const float a0 = Kp + (Ts*Ki/2) + (2*Kd/2);
const float a1 = (Ki*Ts) - (4*Kd/Ts);
const float a2 = Kp + (Ts*Ki/2) + (2*Kd/2);

float Out[3];
float Error[3];
float input, ref;


Error[0] = ref - input;
Out[0] = Out[2] + Error[2]*a0 + Error[1]*a1 + e[0]*a2;

//Update Variables
Out[3] = Out[2];
Out[2] = Out[1];
Out[1] = Out[0];

Error[3] = Error[2];
Error[2] = Error[1];
Error[1] = Error[0];
