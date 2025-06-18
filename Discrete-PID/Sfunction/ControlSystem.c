/* LEVEL 2 S FUNCTION:
*
*
*
* Sample Time is Inherited From the Simulation
*
* Author Lucas G.
*/

#define S_FUNCTION_NAME  ControlSystem
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"           //
#include "math.h"

// #include "PIDControl.h"         // PID, PI, PD

#ifndef MATLAB_MEX_FILE
#include <brtenv.h>
#endif


// Input and Output Width 
#define N_INPUTS    2       // Number of Inputs (2: Reference Signal, Controlled Variable Signal)
#define N_OUTPUTS   2       // Number of Outputs (2: Control Signal, Error Signal)
#define N_PARAMS    4       // Number of Parameters (Kp, Ki, Kd, Ts)

/*====================================  STATES ============================================== */
// Number of Discrete States [Number of values saved in memory between steps]
#define DSTATES 4           // [Elements in x_kn]
#define DEF_PARAM1(S)   ssGetSFcnParam(S, 0)
#define DEF_PARAM2(S)   ssGetSFcnParam(S, 1)
#define DEF_PARAM3(S)   ssGetSFcnParam(S, 2)
#define DEF_PARAM4(S)   ssGetSFcnParam(S, 3)

#define U(element) (*uPtrs[element])

// Previous Values [Memory Storage Array]
#define u_k1        x_k[0]     // u[k-1]
#define u_k2        x_k[1]     // u[k-2]
#define e_k1        x_k[2]     // e[k-1]
#define e_k2        x_k[3]     // e[k-2]

// Memory Initialization array [Used only as Initial values]
#define u0_k1       x0_k[0]     // u0[k-1]
#define u0_k2       x0_k[1]     // u0[k-2]
#define e0_k1       x0_k[2]     // e0[k-1]
#define e0_k2       x0_k[3]     // e0[k-2]

/*=======================================  CONTROL ============================================== */
// Control Variables:
// #define Kp  1       //
// #define Kd  1       //
// #define Ki  1       //
// #define Ts  1       // Sample Period in seconds

// Tustin
// #define a0  ( Kp+ (Ts*Ki) + (Kd/Ts) )
// #define a1  ( (-Kp) - (2*Kd/Ts)     )
// #define a2  ( Kd/Ts                 )


// Part 1: Define ports
static void mdlInitializeSizes(SimStruct *S)
{
    // Set Number of Expected paramters (
    ssSetNumSFcnParams(S, N_PARAMS);  
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) 
    {
        return;
    }

    //Set memory storage array (X_k)
    ssSetNumContStates(S, 0);               // No continous states (its a discre time control)
    ssSetNumDiscStates(S, DSTATES);         // How many elements in the state array x_k

    // Input Port Setup
    if (!ssSetNumInputPorts(S, 1)) return;              // Number Of Inputs [in this case, 1 array]
    ssSetInputPortWidth(S, 0, N_INPUTS);                // Number of elements in the input array
    ssSetInputPortDirectFeedThrough(S, 0, 1);           // If the current output depends on current input

    // Output Port Setup
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, N_OUTPUTS);

    // Number of different sample times [for 2 controllers running at different speed]
    ssSetNumSampleTimes(S, 1);
    
    // Internal Working Memory setup
    ssSetNumRWork(S, 0);                // Real Work
    ssSetNumIWork(S, 0);                // Interger Work
    ssSetNumPWork(S, 0);                // Pointer Work
    ssSetNumModes(S, 0);                // Modes switching ()
    ssSetNumNonsampledZCs(S, 0);        // Zero Crossings, not sure
    
    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

// Part 2: Set the sample time (Chosen or Inherited)
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);       // Uses same sample time as your simulink
    ssSetOffsetTime(S, 0, 0.0);                         // Set Controller Start time
}


// Part 3: Set the systems Initial conditions
#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S) 
{
    // Gets pointer to the Memory Storage Array
    real_T *x0_k = ssGetRealDiscStates(S);
    
    // Uses Pointer to Set Initial Values to the initial arrray [x0_k] that gets copied into the storage Array [X_k]
    u0_k1 = 0.0;
    u0_k2 = 0.0;
    e0_k1 = 0.0;
    e0_k2 = 0.0;
}


// Part 4: Function Call and Outputs
static void mdlOutputs(SimStruct *S, int_T tid)
{
    /*================================= EXTERNAL VARIABLES ==============================*/
    real_T            *y      = ssGetOutputPortRealSignal(S, 0);        // Pointer for the Block's Output
    real_T            *x_k    = ssGetRealDiscStates(S);                 // Pointer for the Discrete States [storage for variables]
    InputRealPtrsType uPtrs   = ssGetInputPortRealSignalPtrs(S, 0);     // Pointer for the Block's Input Signals

    // Control Settings
    real_T Kp 	              = *mxGetPr(DEF_PARAM1(S));
    real_T Ki                 = *mxGetPr(DEF_PARAM2(S));
    real_T Kd                 = *mxGetPr(DEF_PARAM3(S));
    real_T Ts                 = *mxGetPr(DEF_PARAM4(S));
    
    /*================================= INTERNAL VARIABLES ==============================*/
    // Control Algorithm [Euler]
    // real_T a0   = ( Kp + (Ts*Ki) + (Kd/Ts) );
    // real_T a1   = ( (-Kp) - (2*Kd/Ts)      );
    // real_T a2   = ( Kd/Ts                  );

    // PID Control Algorithm Parameters [Tustin] : 
    // real_T a0   = ( Kp + (Ts*Ki/2) + (2*Kd/Ts)  );
    // real_T a1   = ( (Ki*Ts) - (4*Kd/Ts)         );
    // real_T a2   = ( -Kp + (Ts*Ki/2) + (2*Kd/Ts) );

    // PI Control Algorithm Parameters [Tustin]: u_k = u_k1 + e_k1*a1 + e_k*a0;
    real_T a0   = ( Kp + (Ts*Ki/2) );
    real_T a1   = ( (-Kp) + (Ts*Ki/2) );


    // Other variables
    real_T e_k  = 0;  // Current Error
    real_T u_k  = 0;  // Current Control Signal
    
    /*================================= CONTROL SYSTEM ==============================*/
    // Read Inputs
    // real_T RPM_c     = *uPtrs[0];            // Current Plant Signal 
    // real_T RPM_Ref   = *uPtrs[1];            // Current Reference Signal

    real_T RPM_Ref   = U(0);            // Current Reference Signal
    real_T RPM_c     = U(1);            // Current Plant Signal 


    // Implement Control
    e_k = RPM_Ref - RPM_c;
    // u_k = u_k2 + e_k2*a2 + e_k1*a1 + e_k*a0;    // PID
    // u_k = u_k1 + e_k1*a1 + e_k*a0;    // PI
    u_k = u_k1 - e_k1*0.001028 + e_k*0.001028;    // PI
    // u_k = RPM_Ref/360;

    // Update Variables
    e_k2 = e_k1;        // Previous becomes before previous
    u_k1 = u_k;         // current becomes previous

    e_k2 = e_k1;        // Previous becomes before previous
    e_k1 = e_k;         // current becomes previous

    /*================================= Outputs ==============================*/
    y[0] = u_k;         // Current Control Signal 
    y[1] = e_k;         // Current Error Signal

}


// Part 5: 
#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid) {
    // Update function called after output, but not doing anything
    real_T            *x_k       = ssGetRealDiscStates(S);
    InputRealPtrsType uPtrs      = ssGetInputPortRealSignalPtrs(S, 0);
}


// Optional cleanup
static void mdlTerminate(SimStruct *S) {
    UNUSED_ARG(S); // unused input argument
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
