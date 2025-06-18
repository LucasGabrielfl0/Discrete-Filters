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
#include "PIDControl.h"         // PID, PI, PD

// Input and Output Width 
#define N_INPUTS    2
#define N_OUTPUTS   2
#define N_PARAMS    2

/*====================================  STATES ============================================== */
// Number of Discrete States [Number of values saved in memory between steps]
#define DSTATES 3

// Current Values
#define u_k     x_k[0]       // u[k]
#define e_k     x_k[1]       // e[k]

// Previous VALUES
#define u_k1    x_kn[0]     // u[k-1]
#define u_k1    x_kn[1]     // u[k-2]

#define e_k1    x_kn[2]     // e[k-1]
#define e_k2    x_kn[3]     // e[k-2]

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
    // Parameters Setup
    ssSetNumSFcnParams(S, N_PARAMS);  
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) 
    {
        return;
    }

    // Input Port Setup
    ssSetNumInputPorts(S, 1);                       // Number Of Inputs
    ssSetInputPortWidth(S, 0, 1);                   // Number of elements in the input array
    ssSetInputPortDirectFeedThrough(S, 0, 1);       // If the current output depends on current input

    // Output Port Setup
    ssSetNumOutputPorts(S, 1);
    ssSetOutputPortWidth(S, 0, 1);

    // Number of different sample times [for 2 controllers running at different speed]
    ssSetNumSampleTimes(S, 1);



    // ADD  ====================
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, DSTATES);
    
    if (!ssSetNumInputPorts(S, 1)) return; 
    ssSetInputPortWidth(S, 0, NINPUTS);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, NOUTPUTS);
    
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    
    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);

 // add end ===============================










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
    real_T *x0 = ssGetRealDiscStates(S);            // ?  
    u_k1 = 0.0;
    e_k1 = 0.0;
    e_k2 = 0.0;
}


// Part 4: Function Call and Outputs
static void mdlOutputs(SimStruct *S, int_T tid)
{
    /*================================= EXTERNAL VARIABLES ==============================*/
    real_T            *y      = ssGetOutputPortRealSignal(S, 0);        // Pointer for the Block's Output
    real_T            *x      = ssGetRealDiscStates(S);                 // Pointer for the Discrete States [storage for variables]
    InputRealPtrsType uPtrs   = ssGetInputPortRealSignalPtrs(S, 0);     // Pointer for the Block's Input Signals

    // Control Settings
    real_T Kp 	              = *mxGetPr(DEF_PARAM1(S));
    real_T Ki                 = *mxGetPr(DEF_PARAM2(S));
    real_T Kd                 = *mxGetPr(DEF_PARAM2(S));
    real_T Ts                 = *mxGetPr(DEF_PARAM3(S));
    
    /*================================= INTERNAL VARIABLES ==============================*/
    // Control Algorithm [Euler]
    // real_T a0   = ( Kp + (Ts*Ki) + (Kd/Ts) );
    // real_T a1   = ( (-Kp) - (2*Kd/Ts)      );
    // real_T a2   = ( Kd/Ts                  );

    // Control Algorithm Parameters [Tustin]
    real_T a0   = ( Kp + (Ts*Ki/2) + (2*Kd/Ts)  );
    real_T a1   = ( (Ki*Ts) - (4*Kd/Ts)         );
    real_T a2   = ( -Kp + (Ts*Ki/2) + (2*Kd/Ts) );

    // Inputs
    real_T RPM_c     = *uPtrs[0];            // Current Plant Signal 
    real_T RPM_Ref   = *uPtrs[1];            // Current Reference Signal

    // Other variables
    real_T Error_c  = 0;  // Current Error

    /*================================= CONTROL SYSTEM ==============================*/
    // Implement Control
    Error_c = RPM_Ref - RPM_c;
    u_k = u_k2 + Error_c[2]*a2 + Error_c[1]*a1 + Error_c[0]*a0;


    // Update Variables
    u_k1 = u_k;         // current becomes last

    e_k2 = e_k1;        // Last becomes before last
    e_k1 = Error_c;     // current becomes last

    /*================================= Outputs ==============================*/
    y[0] = u_k;           //
    y[1] = Error_c;         //

}


// Part 5: 
#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid) {
    real_T            *x       = ssGetRealDiscStates(S);                    //
    InputRealPtrsType uPtrs    = ssGetInputPortRealSignalPtrs(S, 0);        //
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
