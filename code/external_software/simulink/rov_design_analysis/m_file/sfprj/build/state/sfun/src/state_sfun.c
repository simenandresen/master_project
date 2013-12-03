/* Include files */
#include "state_sfun.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint8_T _sfEvent_;
uint32_T _stateMachineNumber_;
real_T _sfTime_;

/* Function Declarations */

/* Function Definitions */
void state_initializer(void)
{
  _sfEvent_ = CALL_EVENT;
}

void state_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_state_method_dispatcher(SimStruct *simstructPtr, const char
 *chartName, int_T method, void *data)
{
  return 0;
}
unsigned int sf_state_process_check_sum_call( int nlhs, mxArray * plhs[], int
 nrhs, const mxArray * prhs[] )
{
#ifdef MATLAB_MEX_FILE
  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) ) return 0;
  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if(strcmp(commandName,"sf_get_check_sum")) return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if(nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if(!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(291252440U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2302047904U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3313992679U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3421666653U);
    }else if(!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    }else if(!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(870376660U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(481385932U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2736352094U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2341651006U);
    }else if(nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch(chartFileNumber) {
       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    }else if(!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4125229215U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(365478573U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1351650418U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(619095777U);
    }else {
      return 0;
    }
  } else{
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2013371586U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(834230209U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1637885620U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2090454094U);
  }
  return 1;
#else
  return 0;
#endif
}

unsigned int sf_state_autoinheritance_info( int nlhs, mxArray * plhs[], int
 nrhs, const mxArray * prhs[] )
{
#ifdef MATLAB_MEX_FILE
  char commandName[32];
  if (nrhs<2 || !mxIsChar(prhs[0]) ) return 0;
  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if(strcmp(commandName,"get_autoinheritance_info")) return 0;
  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch(chartFileNumber) {
     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }
  return 1;
#else
  return 0;
#endif
}
void state_debug_initialize(void)
{
  _stateMachineNumber_ = sf_debug_initialize_machine("state","sfun",0,0,0,0,0);
  sf_debug_set_machine_event_thresholds(_stateMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(_stateMachineNumber_,0);
}

