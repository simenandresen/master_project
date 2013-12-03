/*  File    : pipi2inf.c
 *
 *  Author	: Morten Breivik
 *	Date	: 20/1-2004
 *
 *	Description:
 *      S-function that maps an angle from <-pi,pi> to <-inf,inf>
 *
 *  Modifications: 5/1-2005 ONS: Removed global variables, using simstruct instead.
 *
*/


#define S_FUNCTION_NAME pipi2inf
#define S_FUNCTION_LEVEL 2
#define PI 3.14159

#include "simstruc.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* Function: mdlInitializeSizes ===============================================
 *
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0); /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
	ssSetNumInputPorts(S, 1); // current_angle
	ssSetNumOutputPorts(S,1); // mapped_angle
	//Direct feedthrough: 1 = true, 0 = false
	ssSetInputPortDirectFeedThrough(S, 0, 1);

	ssSetInputPortWidth(S, 0, 1); // current_angle
    ssSetOutputPortWidth(S, 0, 1); // mapped_angle

    ssSetNumSampleTimes(S, 1);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);

    ssSetNumRWork(S, 2); // Previous angle and accumulation
	ssSetNumIWork(S, 1); // State
	ssSetNumPWork(S, 0);
}


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we inherit our sample time from the driving block.
 *    However, we don't execute in minor steps.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}


#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START)
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {

	// Initialize state to zero
	ssSetIWorkValue(S,0,0);

	// Initialize previous angle to zero
	ssSetRWorkValue(S,0,0);

	// Initialize accumulation to zero
	ssSetRWorkValue(S,1,0);

  }
#endif /*  MDL_START */


/* Function: mdlOutputs =======================================================
 * Abstract:
 *    y = rwork
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
	double current_angle, accumulate, mapped_angle;

	int state;
	double previous_angle, accumulation;

	InputRealPtrsType input = ssGetInputPortRealSignalPtrs(S,0);
	real_T *output = ssGetOutputPortRealSignal(S,0);

	// Get the variables from the working vectors
	state = (int) ssGetIWorkValue(S,0);
	previous_angle = (double) ssGetRWorkValue(S,0);
	accumulation = (double) ssGetRWorkValue(S,1);


	// Setting model parameters
	current_angle = *input[0];

	if( current_angle>0 && current_angle<(PI/2) ) {                // 1.quadrant
		if(state==1) { // Coming from the 1.quadrant
			accumulate = current_angle - previous_angle;
		}
		else if(state==2) { // Coming from the 2.quadrant
			accumulate = current_angle - previous_angle;
		}
		else if(state==3) { // Coming from the 3.quadrant
			if( (current_angle+abs(previous_angle))<=PI ) {
				accumulate = current_angle - previous_angle;
			}
			else {
				accumulate = previous_angle - current_angle + 2*PI;
			}
		}
		else if(state==4) { // Coming from the 4.quadrant
			accumulate = current_angle - previous_angle;
		}
		else if(state==5) { // Coming from pi/2
			accumulate = current_angle - previous_angle;
		}
		else if(state==6) { // Coming from 0
			accumulate = current_angle;
		}
		else if(state==7) { // Coming from -pi/2
			accumulate = current_angle - previous_angle;
		}
		else if(state==8) { // Coming from -pi/pi
			accumulate = current_angle - abs(previous_angle);
		}
		else { // Simulation has just started
			accumulate = current_angle - previous_angle;
		}
		state = 1;
	}
	else if( current_angle<0 && current_angle>(-PI/2) ) {           // 2.quadrant
		if(state==1) { // Coming from the 1.quadrant
			accumulate = current_angle - previous_angle;
		}
		else if(state==2) { // Coming from the 2.quadrant
			accumulate = current_angle - previous_angle;
		}
		else if(state==3) { // Coming from the 3.quadrant
			accumulate = current_angle - previous_angle;
		}
		else if(state==4) { // Coming from the 4.quadrant
			if( (abs(current_angle)+previous_angle)<=PI ) {
				accumulate = current_angle - previous_angle;
			}
			else {
				accumulate = current_angle - previous_angle + 2*PI;
			}
		}
		else if(state==5) { // Coming from pi/2
			accumulate = current_angle - previous_angle;
		}
		else if(state==6) { // Coming from 0
			accumulate = current_angle;
		}
		else if(state==7) { // Coming from -pi/2
			accumulate = current_angle - previous_angle;
		}
		else if(state==8) { // Coming from -pi/pi
			accumulate = current_angle + PI;
		}
		else { // Simulation has just started
			accumulate = current_angle - previous_angle;
		}
		state = 2;
	}
	else if( current_angle<(-PI/2) && current_angle>(-PI) ) {       // 3.quadrant
		if(state==1) { // Coming from the 1.quadrant
			if( (abs(current_angle)+previous_angle)<=PI ) {
				accumulate = current_angle - previous_angle;
			}
			else {
				accumulate = current_angle - previous_angle + 2*PI;
			}
		}
		else if(state==2) { // Coming from the 2.quadrant
			accumulate = current_angle - previous_angle;
		}
		else if(state==3) { // Coming from the 3.quadrant
			accumulate = current_angle - previous_angle;
		}
		else if(state==4) { // Coming from the 4.quadrant
			accumulate = current_angle - previous_angle + 2*PI;
		}
		else if(state==5) { // Coming from pi/2
			accumulate = 3*PI/2 - current_angle;
		}
		else if(state==6) { // Coming from 0
			accumulate = current_angle;
		}
		else if(state==7) { // Coming from -pi/2
			accumulate = current_angle - previous_angle;
		}
		else if(state==8) { // Coming from -pi/pi
			accumulate = current_angle + PI;
		}
		else { // Simulation has just started
			accumulate = current_angle - previous_angle;
		}
		state = 3;
	}
	else if( current_angle>(PI/2) && current_angle<(PI) ) {         // 4.quadrant
		if(state==1) { // Coming from the 1.quadrant
			accumulate = current_angle - previous_angle;
		}
		else if(state==2) { // Coming from the 2.quadrant
			if( (current_angle+abs(previous_angle))<=PI ) {
				accumulate = current_angle - previous_angle;
			}
			else {
				accumulate = previous_angle - current_angle + 2*PI;
			}
		}
		else if(state==3) { // Coming from the 3.quadrant
			accumulate = current_angle - previous_angle - 2*PI;
		}
		else if(state==4) { // Coming from the 4.quadrant
			accumulate = current_angle - previous_angle;
		}
		else if(state==5) { // Coming from pi/2
			accumulate = current_angle - previous_angle;
		}
		else if(state==6) { // Coming from 0
			accumulate = current_angle;
		}
		else if(state==7) { // Coming from -pi/2
			accumulate = current_angle -3*PI/2;
		}
		else if(state==8) { // Coming from -pi/pi
			accumulate = current_angle - PI;
		}
		else { // Simulation has just started
			accumulate = current_angle - previous_angle;
		}
		state = 4;
	}
	else if( current_angle==(PI/2) ) {         // pi/2 line
		if(state==1) { // Coming from the 1.quadrant
			accumulate = current_angle - previous_angle;
		}
		else if(state==2) { // Coming from the 2.quadrant
			accumulate = current_angle - previous_angle;
		}
		else if(state==3) { // Coming from the 3.quadrant
			accumulate = -3*PI/2 - previous_angle;
		}
		else if(state==4) { // Coming from the 4.quadrant
			accumulate = PI/2 - previous_angle;
		}
		else if(state==5) { // Coming from pi/2
			accumulate = 0;
		}
		else if(state==6) { // Coming from 0
			accumulate = current_angle;
		}
		else if(state==7) { // Coming from -pi/2
			accumulate = PI; // By choice (could also have subtracted pi, this would be equal from an assessment based on the Guide Rule)
		}
		else if(state==8) { // Coming from -pi/pi
			accumulate = -PI/2;
		}
		else { // Simulation has just started
			accumulate = current_angle - previous_angle;
		}
		state = 5;
	}
	else if( current_angle==0 ) {         // 0 line
		if(state==1) { // Coming from the 1.quadrant
			accumulate = -previous_angle;
		}
		else if(state==2) { // Coming from the 2.quadrant
			accumulate = -previous_angle;
		}
		else if(state==3) { // Coming from the 3.quadrant
			accumulate = -previous_angle;
		}
		else if(state==4) { // Coming from the 4.quadrant
			accumulate = -previous_angle;
		}
		else if(state==5) { // Coming from pi/2
			accumulate = -previous_angle;
		}
		else if(state==6) { // Coming from 0
			accumulate = 0;
		}
		else if(state==7) { // Coming from -pi/2
			accumulate = -previous_angle;
		}
		else if(state==8) { // Coming from -pi/pi
			accumulate = PI; // By choice (could also have subtracted pi, this would be equal from an assessment based on the Guide Rule)
		}
		else { // Simulation has just started
			accumulate = current_angle - previous_angle;
		}
		state = 6;
	}
	else if( current_angle==(-PI/2) ) {         // -PI/2 line
		if(state==1) { // Coming from the 1.quadrant
			accumulate = -previous_angle - PI/2;
		}
		else if(state==2) { // Coming from the 2.quadrant
			accumulate = -previous_angle - PI/2;
		}
		else if(state==3) { // Coming from the 3.quadrant
			accumulate = -previous_angle - PI/2;
		}
		else if(state==4) { // Coming from the 4.quadrant
			accumulate = 3*PI/2 - previous_angle;
		}
		else if(state==5) { // Coming from pi/2
			accumulate = PI; // By choice (could also have subtracted pi, this would be equal from an assessment based on the Guide Rule)
		}
		else if(state==6) { // Coming from 0
			accumulate = current_angle;
		}
		else if(state==7) { // Coming from -pi/2
			accumulate = 0;
		}
		else if(state==8) { // Coming from -pi/pi
			accumulate = PI/2;
		}
		else { // Simulation has just started
			accumulate = current_angle - previous_angle;
		}
		state = 7;
	}
	else { //( current_angle==-PI || current_angle==PI ) )         // -pi/pi line
		if(state==1) { // Coming from the 1.quadrant
			accumulate = PI - previous_angle;
		}
		else if(state==2) { // Coming from the 2.quadrant
			accumulate = -PI - previous_angle;
		}
		else if(state==3) { // Coming from the 3.quadrant
			accumulate = -PI - previous_angle;
		}
		else if(state==4) { // Coming from the 4.quadrant
			accumulate = PI - previous_angle;
		}
		else if(state==5) { // Coming from pi/2
			accumulate = PI/2;
		}
		else if(state==6) { // Coming from 0
			accumulate = PI; // By choice (could also have subtracted pi, this would be equal from an assessment based on the Guide Rule)
		}
		else if(state==7) { // Coming from -pi/2
			accumulate = PI/2;
		}
		else if(state==8) { // Coming from -pi/pi
			accumulate = 0;
		}
		else { // Simulation has just started
			accumulate = current_angle - previous_angle;
		}
		state = 8;
	}

	accumulation = accumulation + accumulate;
	mapped_angle = accumulation;
	previous_angle = current_angle;

	output[0] = mapped_angle;

	// Save the working variables
	ssSetIWorkValue(S,0,state);
	ssSetRWorkValue(S,0,previous_angle);
	ssSetRWorkValue(S,1,accumulation);

}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
    UNUSED_ARG(S); /* unused input argument */
}



#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
