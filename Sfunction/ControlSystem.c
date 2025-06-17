#define S_FUNCTION_NAME  ControlSystem
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"       //

#include "ControlSystem.h"         // Your Function's Header file



// Input and Output Width 
#define NINPUTS  2
#define NOUTPUTS 2

// Define Sample time


// Part 1: Define ports
static void mdlInitializeSizes(SimStruct *S)
{
    // Parameters Setup
    ssSetNumSFcnParams(S, NPARAMS);  
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) 
    {
        return;
    }

    //
    ssSetNumInputPorts(S, 1);
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    ssSetNumOutputPorts(S, 1);
    ssSetOutputPortWidth(S, 0, 1);

    ssSetNumSampleTimes(S, 1);
}

// Part 2: Set the sample time (can be chosen or Inherited)
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

// Part 3: Set the systems Initial conditions
static void mdlInitializeConditions(SimStruct *S) {
    real_T *x0 = ssGetRealDiscStates(S);
    
    e_10           = 0.0;
    u_10           = 0.0;
}


// Part 4: Function Call and Outputs
static void mdlOutputs(SimStruct *S, int_T tid)
{
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    real_T input = *uPtrs[0];

    real_T *y = ssGetOutputPortRealSignal(S, 0);
    my_function(&input, y);
}




/* Function: mdlUpdate ====================================================== */
static void mdlUpdate(SimStruct *S, int_T tid) {
    real_T            *x       = ssGetRealDiscStates(S);
    InputRealPtrsType uPtrs    = ssGetInputPortRealSignalPtrs(S, 0);
    
}


// Optional cleanup
static void mdlTerminate(SimStruct *S) {}

#include "simulink.c"
