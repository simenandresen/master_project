#ifndef __c8_inverse_kinematics_h__
#define __c8_inverse_kinematics_h__

/* Include files */
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "rtwtypes.h"

/* Type Definitions */
typedef struct {
  int32_T c8_sfEvent;
  boolean_T c8_isStable;
  boolean_T c8_doneDoubleBufferReInit;
  uint8_T c8_is_active_c8_inverse_kinematics;
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
} SFc8_inverse_kinematicsInstanceStruct;

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c8_inverse_kinematics_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c8_inverse_kinematics_get_check_sum(mxArray *plhs[]);
extern void c8_inverse_kinematics_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
