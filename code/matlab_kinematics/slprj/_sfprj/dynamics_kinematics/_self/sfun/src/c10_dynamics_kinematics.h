#ifndef __c10_dynamics_kinematics_h__
#define __c10_dynamics_kinematics_h__

/* Include files */
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "rtwtypes.h"

/* Type Definitions */
typedef struct {
  int32_T c10_sfEvent;
  boolean_T c10_isStable;
  boolean_T c10_doneDoubleBufferReInit;
  uint8_T c10_is_active_c10_dynamics_kinematics;
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
} SFc10_dynamics_kinematicsInstanceStruct;

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c10_dynamics_kinematics_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c10_dynamics_kinematics_get_check_sum(mxArray *plhs[]);
extern void c10_dynamics_kinematics_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
