/* Include files */
#include "rov_design_analysis_sfun.h"
#include "c1_rov_design_analysis.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _rov_design_analysisMachineNumber_;

/* Function Declarations */

/* Function Definitions */
void rov_design_analysis_initializer(void)
{
  _sfEvent_ = CALL_EVENT;
}

void rov_design_analysis_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_rov_design_analysis_method_dispatcher(SimStruct *simstructPtr,
 const char *chartName, int_T method, void *data)
{
  if(!strcmp_ignore_ws(chartName,"rov_design_analysis/ROV Controller /Switching- by stateflow/Chart/ SFunction "))
  {
    c1_rov_design_analysis_method_dispatcher(simstructPtr, method, data);
    return 1;
  }
  return 0;
}
unsigned int sf_rov_design_analysis_process_check_sum_call( int nlhs, mxArray *
 plhs[], int nrhs, const mxArray * prhs[] )
{
#ifdef MATLAB_MEX_FILE
  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) ) return 0;
  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if(strcmp(commandName,"sf_get_check_sum")) return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if(nrhs>2 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if(!strcmp(commandName,"library")) {
      char machineName[100];
      mxGetString(prhs[2], machineName,sizeof(machineName)/sizeof(char));
      machineName[(sizeof(machineName)/sizeof(char)-1)] = '\0';
      if(!strcmp(machineName,"rov_design_analysis")){
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1271294530U);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(487731934U);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3630882969U);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2509267412U);
      }else{
        return 0;
      }
    }
  }else {
    return 0;
  }

  return 1;
#else
  return 0;
#endif
}

unsigned int sf_rov_design_analysis_autoinheritance_info( int nlhs, mxArray *
 plhs[], int nrhs, const mxArray * prhs[] )
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
     case 1:
      {
        extern mxArray *sf_c1_rov_design_analysis_get_autoinheritance_info(void);
        plhs[0] = sf_c1_rov_design_analysis_get_autoinheritance_info();
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }
  return 1;
#else
  return 0;
#endif
}
void rov_design_analysis_debug_initialize(void)
{
  _rov_design_analysisMachineNumber_ =
    sf_debug_initialize_machine("rov_design_analysis","sfun",1,1,0,0,0);
  sf_debug_set_machine_event_thresholds(_rov_design_analysisMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(_rov_design_analysisMachineNumber_,0);
}

