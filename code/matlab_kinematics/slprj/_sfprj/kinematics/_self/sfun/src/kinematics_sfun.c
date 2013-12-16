/* Include files */

#include "kinematics_sfun.h"
#include "c1_kinematics.h"
#include "c2_kinematics.h"
#include "c3_kinematics.h"
#include "c4_kinematics.h"
#include "c5_kinematics.h"
#include "c6_kinematics.h"
#include "c7_kinematics.h"
#include "c8_kinematics.h"
#include "c11_kinematics.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _kinematicsMachineNumber_;
real_T _sfTime_;

/* Function Declarations */

/* Function Definitions */
void kinematics_initializer(void)
{
}

void kinematics_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_kinematics_method_dispatcher(SimStruct *simstructPtr, unsigned
  int chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==1) {
    c1_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==2) {
    c2_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==4) {
    c4_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==5) {
    c5_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==6) {
    c6_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==7) {
    c7_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==8) {
    c8_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==11) {
    c11_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_kinematics_process_check_sum_call( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1089022658U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2419393406U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(902931002U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3298671051U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1882155247U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(948396880U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3116724312U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(608862847U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 1:
        {
          extern void sf_c1_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c1_kinematics_get_check_sum(plhs);
          break;
        }

       case 2:
        {
          extern void sf_c2_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c2_kinematics_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_c3_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c3_kinematics_get_check_sum(plhs);
          break;
        }

       case 4:
        {
          extern void sf_c4_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c4_kinematics_get_check_sum(plhs);
          break;
        }

       case 5:
        {
          extern void sf_c5_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c5_kinematics_get_check_sum(plhs);
          break;
        }

       case 6:
        {
          extern void sf_c6_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c6_kinematics_get_check_sum(plhs);
          break;
        }

       case 7:
        {
          extern void sf_c7_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c7_kinematics_get_check_sum(plhs);
          break;
        }

       case 8:
        {
          extern void sf_c8_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c8_kinematics_get_check_sum(plhs);
          break;
        }

       case 11:
        {
          extern void sf_c11_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c11_kinematics_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3564696471U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(678668628U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1090454852U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3896867807U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3866530370U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1634129642U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3220441580U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1368295495U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_kinematics_autoinheritance_info( int nlhs, mxArray * plhs[], int
  nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(aiChksum, "s3hwsiQ8ZNu21O7VHsFrlD") == 0) {
          extern mxArray *sf_c1_kinematics_get_autoinheritance_info(void);
          plhs[0] = sf_c1_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 2:
      {
        if (strcmp(aiChksum, "nySmj9QsbElRecSpiNfse") == 0) {
          extern mxArray *sf_c2_kinematics_get_autoinheritance_info(void);
          plhs[0] = sf_c2_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "nuObfNF9CFP8h1dudASij") == 0) {
          extern mxArray *sf_c3_kinematics_get_autoinheritance_info(void);
          plhs[0] = sf_c3_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 4:
      {
        if (strcmp(aiChksum, "aWNKcVbgfliVKdLLh9TmZF") == 0) {
          extern mxArray *sf_c4_kinematics_get_autoinheritance_info(void);
          plhs[0] = sf_c4_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 5:
      {
        if (strcmp(aiChksum, "AHnNNsQvCG125rDlJch1FH") == 0) {
          extern mxArray *sf_c5_kinematics_get_autoinheritance_info(void);
          plhs[0] = sf_c5_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 6:
      {
        if (strcmp(aiChksum, "M0xOffBX25tpa22KWmoM0D") == 0) {
          extern mxArray *sf_c6_kinematics_get_autoinheritance_info(void);
          plhs[0] = sf_c6_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 7:
      {
        if (strcmp(aiChksum, "usafqUiVVQ0PIFgUb92QKF") == 0) {
          extern mxArray *sf_c7_kinematics_get_autoinheritance_info(void);
          plhs[0] = sf_c7_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 8:
      {
        if (strcmp(aiChksum, "vCJKyyuI5Z7XSCJy6wf4TE") == 0) {
          extern mxArray *sf_c8_kinematics_get_autoinheritance_info(void);
          plhs[0] = sf_c8_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 11:
      {
        if (strcmp(aiChksum, "cGyZKxApo1AphCNRHxsoxF") == 0) {
          extern mxArray *sf_c11_kinematics_get_autoinheritance_info(void);
          plhs[0] = sf_c11_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
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

unsigned int sf_kinematics_get_eml_resolved_functions_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        extern const mxArray *sf_c1_kinematics_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 2:
      {
        extern const mxArray *sf_c2_kinematics_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray *sf_c3_kinematics_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 4:
      {
        extern const mxArray *sf_c4_kinematics_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c4_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 5:
      {
        extern const mxArray *sf_c5_kinematics_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c5_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 6:
      {
        extern const mxArray *sf_c6_kinematics_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c6_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 7:
      {
        extern const mxArray *sf_c7_kinematics_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c7_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 8:
      {
        extern const mxArray *sf_c8_kinematics_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c8_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 11:
      {
        extern const mxArray *sf_c11_kinematics_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c11_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
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

void kinematics_debug_initialize(void)
{
  _kinematicsMachineNumber_ = sf_debug_initialize_machine("kinematics","sfun",0,
    9,0,0,0);
  sf_debug_set_machine_event_thresholds(_kinematicsMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(_kinematicsMachineNumber_,0);
}

void kinematics_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_kinematics_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info("kinematics",
      "kinematics");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_kinematics_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
