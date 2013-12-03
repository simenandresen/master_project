/* Include files */

#include "dynamics_kinematics_sfun.h"
#include "c1_dynamics_kinematics.h"
#include "c2_dynamics_kinematics.h"
#include "c3_dynamics_kinematics.h"
#include "c4_dynamics_kinematics.h"
#include "c5_dynamics_kinematics.h"
#include "c6_dynamics_kinematics.h"
#include "c7_dynamics_kinematics.h"
#include "c8_dynamics_kinematics.h"
#include "c9_dynamics_kinematics.h"
#include "c10_dynamics_kinematics.h"
#include "c11_dynamics_kinematics.h"
#include "c12_dynamics_kinematics.h"
#include "c13_dynamics_kinematics.h"
#include "c14_dynamics_kinematics.h"
#include "c15_dynamics_kinematics.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _dynamics_kinematicsMachineNumber_;
real_T _sfTime_;

/* Function Declarations */

/* Function Definitions */
void dynamics_kinematics_initializer(void)
{
}

void dynamics_kinematics_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_dynamics_kinematics_method_dispatcher(SimStruct *simstructPtr,
  unsigned int chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==1) {
    c1_dynamics_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==2) {
    c2_dynamics_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_dynamics_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==4) {
    c4_dynamics_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==5) {
    c5_dynamics_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==6) {
    c6_dynamics_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==7) {
    c7_dynamics_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==8) {
    c8_dynamics_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==9) {
    c9_dynamics_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==10) {
    c10_dynamics_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==11) {
    c11_dynamics_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==12) {
    c12_dynamics_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==13) {
    c13_dynamics_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==14) {
    c14_dynamics_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==15) {
    c15_dynamics_kinematics_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_dynamics_kinematics_process_check_sum_call( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
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
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(260117769U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(635458352U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2337646761U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(254122034U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3580786307U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1162537915U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1220619782U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1156872890U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 1:
        {
          extern void sf_c1_dynamics_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c1_dynamics_kinematics_get_check_sum(plhs);
          break;
        }

       case 2:
        {
          extern void sf_c2_dynamics_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c2_dynamics_kinematics_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_c3_dynamics_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c3_dynamics_kinematics_get_check_sum(plhs);
          break;
        }

       case 4:
        {
          extern void sf_c4_dynamics_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c4_dynamics_kinematics_get_check_sum(plhs);
          break;
        }

       case 5:
        {
          extern void sf_c5_dynamics_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c5_dynamics_kinematics_get_check_sum(plhs);
          break;
        }

       case 6:
        {
          extern void sf_c6_dynamics_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c6_dynamics_kinematics_get_check_sum(plhs);
          break;
        }

       case 7:
        {
          extern void sf_c7_dynamics_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c7_dynamics_kinematics_get_check_sum(plhs);
          break;
        }

       case 8:
        {
          extern void sf_c8_dynamics_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c8_dynamics_kinematics_get_check_sum(plhs);
          break;
        }

       case 9:
        {
          extern void sf_c9_dynamics_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c9_dynamics_kinematics_get_check_sum(plhs);
          break;
        }

       case 10:
        {
          extern void sf_c10_dynamics_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c10_dynamics_kinematics_get_check_sum(plhs);
          break;
        }

       case 11:
        {
          extern void sf_c11_dynamics_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c11_dynamics_kinematics_get_check_sum(plhs);
          break;
        }

       case 12:
        {
          extern void sf_c12_dynamics_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c12_dynamics_kinematics_get_check_sum(plhs);
          break;
        }

       case 13:
        {
          extern void sf_c13_dynamics_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c13_dynamics_kinematics_get_check_sum(plhs);
          break;
        }

       case 14:
        {
          extern void sf_c14_dynamics_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c14_dynamics_kinematics_get_check_sum(plhs);
          break;
        }

       case 15:
        {
          extern void sf_c15_dynamics_kinematics_get_check_sum(mxArray *plhs[]);
          sf_c15_dynamics_kinematics_get_check_sum(plhs);
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
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1886000320U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2938273892U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3109280255U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(469696138U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_dynamics_kinematics_autoinheritance_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
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
          extern mxArray *sf_c1_dynamics_kinematics_get_autoinheritance_info
            (void);
          plhs[0] = sf_c1_dynamics_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 2:
      {
        if (strcmp(aiChksum, "Z52LXaXMKTwxlEZtdeyNCC") == 0) {
          extern mxArray *sf_c2_dynamics_kinematics_get_autoinheritance_info
            (void);
          plhs[0] = sf_c2_dynamics_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "cGyZKxApo1AphCNRHxsoxF") == 0) {
          extern mxArray *sf_c3_dynamics_kinematics_get_autoinheritance_info
            (void);
          plhs[0] = sf_c3_dynamics_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 4:
      {
        if (strcmp(aiChksum, "dcs7TglugL3ge3wEJMCLMF") == 0) {
          extern mxArray *sf_c4_dynamics_kinematics_get_autoinheritance_info
            (void);
          plhs[0] = sf_c4_dynamics_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 5:
      {
        if (strcmp(aiChksum, "AHnNNsQvCG125rDlJch1FH") == 0) {
          extern mxArray *sf_c5_dynamics_kinematics_get_autoinheritance_info
            (void);
          plhs[0] = sf_c5_dynamics_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 6:
      {
        if (strcmp(aiChksum, "D0YaXTHH5AO1LocTE0Q2YB") == 0) {
          extern mxArray *sf_c6_dynamics_kinematics_get_autoinheritance_info
            (void);
          plhs[0] = sf_c6_dynamics_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 7:
      {
        if (strcmp(aiChksum, "usafqUiVVQ0PIFgUb92QKF") == 0) {
          extern mxArray *sf_c7_dynamics_kinematics_get_autoinheritance_info
            (void);
          plhs[0] = sf_c7_dynamics_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 8:
      {
        if (strcmp(aiChksum, "ZgQZBvOapeOh6z53wJcV6G") == 0) {
          extern mxArray *sf_c8_dynamics_kinematics_get_autoinheritance_info
            (void);
          plhs[0] = sf_c8_dynamics_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 9:
      {
        if (strcmp(aiChksum, "tNqq0strKLZiXTFsBKYR0D") == 0) {
          extern mxArray *sf_c9_dynamics_kinematics_get_autoinheritance_info
            (void);
          plhs[0] = sf_c9_dynamics_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 10:
      {
        if (strcmp(aiChksum, "V5PIoYZM2jISc0o8tzyTRG") == 0) {
          extern mxArray *sf_c10_dynamics_kinematics_get_autoinheritance_info
            (void);
          plhs[0] = sf_c10_dynamics_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 11:
      {
        if (strcmp(aiChksum, "cGyZKxApo1AphCNRHxsoxF") == 0) {
          extern mxArray *sf_c11_dynamics_kinematics_get_autoinheritance_info
            (void);
          plhs[0] = sf_c11_dynamics_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 12:
      {
        if (strcmp(aiChksum, "6wuFSAdD6CIv03SacLNImB") == 0) {
          extern mxArray *sf_c12_dynamics_kinematics_get_autoinheritance_info
            (void);
          plhs[0] = sf_c12_dynamics_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 13:
      {
        if (strcmp(aiChksum, "X4RdEazgMKVQUdTSAq5v9") == 0) {
          extern mxArray *sf_c13_dynamics_kinematics_get_autoinheritance_info
            (void);
          plhs[0] = sf_c13_dynamics_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 14:
      {
        if (strcmp(aiChksum, "JVrB8SRALQVbHH0sZndbRE") == 0) {
          extern mxArray *sf_c14_dynamics_kinematics_get_autoinheritance_info
            (void);
          plhs[0] = sf_c14_dynamics_kinematics_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 15:
      {
        if (strcmp(aiChksum, "pX9QiQyVKZmIrCTI0lSDdG") == 0) {
          extern mxArray *sf_c15_dynamics_kinematics_get_autoinheritance_info
            (void);
          plhs[0] = sf_c15_dynamics_kinematics_get_autoinheritance_info();
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

unsigned int sf_dynamics_kinematics_get_eml_resolved_functions_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
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
        extern const mxArray
          *sf_c1_dynamics_kinematics_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_dynamics_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 2:
      {
        extern const mxArray
          *sf_c2_dynamics_kinematics_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_dynamics_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray
          *sf_c3_dynamics_kinematics_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_dynamics_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 4:
      {
        extern const mxArray
          *sf_c4_dynamics_kinematics_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c4_dynamics_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 5:
      {
        extern const mxArray
          *sf_c5_dynamics_kinematics_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c5_dynamics_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 6:
      {
        extern const mxArray
          *sf_c6_dynamics_kinematics_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c6_dynamics_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 7:
      {
        extern const mxArray
          *sf_c7_dynamics_kinematics_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c7_dynamics_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 8:
      {
        extern const mxArray
          *sf_c8_dynamics_kinematics_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c8_dynamics_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 9:
      {
        extern const mxArray
          *sf_c9_dynamics_kinematics_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c9_dynamics_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 10:
      {
        extern const mxArray
          *sf_c10_dynamics_kinematics_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c10_dynamics_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 11:
      {
        extern const mxArray
          *sf_c11_dynamics_kinematics_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c11_dynamics_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 12:
      {
        extern const mxArray
          *sf_c12_dynamics_kinematics_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c12_dynamics_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 13:
      {
        extern const mxArray
          *sf_c13_dynamics_kinematics_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c13_dynamics_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 14:
      {
        extern const mxArray
          *sf_c14_dynamics_kinematics_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c14_dynamics_kinematics_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 15:
      {
        extern const mxArray
          *sf_c15_dynamics_kinematics_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c15_dynamics_kinematics_get_eml_resolved_functions_info();
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

void dynamics_kinematics_debug_initialize(void)
{
  _dynamics_kinematicsMachineNumber_ = sf_debug_initialize_machine(
    "dynamics_kinematics","sfun",0,15,0,0,0);
  sf_debug_set_machine_event_thresholds(_dynamics_kinematicsMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(_dynamics_kinematicsMachineNumber_,0);
}

void dynamics_kinematics_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_dynamics_kinematics_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info(
      "dynamics_kinematics", "dynamics_kinematics");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_dynamics_kinematics_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
