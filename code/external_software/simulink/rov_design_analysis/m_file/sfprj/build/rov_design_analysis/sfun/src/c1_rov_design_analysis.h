#ifndef __c1_rov_design_analysis_h__
#define __c1_rov_design_analysis_h__

/* Include files */
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "rtw_capi.h"
#include "rtw_modelmap.h"

/* Type Definitions */
typedef struct {
  SimStruct *S;
  void *c1_testPointAddrMap[2];
  uint32_T chartNumber;
  uint32_T instanceNumber;
  uint8_T c1_is_active_c1_rov_design_analysis;
  uint8_T c1_is_c1_rov_design_analysis;
  uint8_T c1_tp_off;
  uint8_T c1_tp_on;
  rtwCAPI_ModelMappingInfo c1_testPointMappingInfo;
  ChartInfoStruct chartInfo;
} SFc1_rov_design_analysisInstanceStruct;

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */

extern void sf_c1_rov_design_analysis_get_check_sum(mxArray *plhs[]);
extern void c1_rov_design_analysis_method_dispatcher(SimStruct *S, int_T method,
 void *data);

#endif

