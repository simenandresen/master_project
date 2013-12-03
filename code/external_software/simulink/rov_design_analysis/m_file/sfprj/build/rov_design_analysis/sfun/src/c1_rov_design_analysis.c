/* Include files */
#include "rov_design_analysis_sfun.h"
#include "c1_rov_design_analysis.h"
#define CHARTINSTANCE_CHARTNUMBER       (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER    (chartInstance->instanceNumber)
#include "rov_design_analysis_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define c1_IN_NO_ACTIVE_CHILD           (0)
#define c1_IN_off                       (1)
#define c1_IN_on                        (2)

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
static void
initialize_c1_rov_design_analysis(SFc1_rov_design_analysisInstanceStruct
 *chartInstance);
static void enable_c1_rov_design_analysis(SFc1_rov_design_analysisInstanceStruct
 *chartInstance);
static void
disable_c1_rov_design_analysis(SFc1_rov_design_analysisInstanceStruct
 *chartInstance);
static void
finalize_c1_rov_design_analysis(SFc1_rov_design_analysisInstanceStruct
 *chartInstance);
static void sf_c1_rov_design_analysis(SFc1_rov_design_analysisInstanceStruct
 *chartInstance);
static int8_T c1__s8_d_(SFc1_rov_design_analysisInstanceStruct *chartInstance,
 real_T c1_b);
static real_T *c1_L(SFc1_rov_design_analysisInstanceStruct *chartInstance);
static real_T *c1_V(SFc1_rov_design_analysisInstanceStruct *chartInstance);
static void init_test_point_addr_map(SFc1_rov_design_analysisInstanceStruct
 *chartInstance);
static void **get_test_point_address_map(SFc1_rov_design_analysisInstanceStruct
 *chartInstance);
static rtwCAPI_ModelMappingInfo
*get_test_point_mapping_info(SFc1_rov_design_analysisInstanceStruct
 *chartInstance);

/* Function Definitions */
static void
initialize_c1_rov_design_analysis(SFc1_rov_design_analysisInstanceStruct
 *chartInstance)
{
  chartInstance->c1_tp_off = 0;
  chartInstance->c1_tp_on = 0;
  chartInstance->c1_is_active_c1_rov_design_analysis = 0;
  chartInstance->c1_is_c1_rov_design_analysis = 0;
  if(!(cdrGetOutputPortReusable(chartInstance->S, 1) != 0)) {
    *c1_L(chartInstance) = (real_T)c1__s8_d_(chartInstance, 0.0);
  }
}

static void enable_c1_rov_design_analysis(SFc1_rov_design_analysisInstanceStruct
 *chartInstance)
{
}

static void
disable_c1_rov_design_analysis(SFc1_rov_design_analysisInstanceStruct
 *chartInstance)
{
}

static void
finalize_c1_rov_design_analysis(SFc1_rov_design_analysisInstanceStruct
 *chartInstance)
{
}

static void sf_c1_rov_design_analysis(SFc1_rov_design_analysisInstanceStruct
 *chartInstance)
{
  uint8_T c1_previousEvent;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_DATA_RANGE_CHECK(*c1_L(chartInstance), 0U);
  _SFD_DATA_RANGE_CHECK(*c1_V(chartInstance), 1U);
  c1_previousEvent = _sfEvent_;
  _sfEvent_ = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG,0);
  if(chartInstance->c1_is_active_c1_rov_design_analysis == 0) {
    _SFD_CC_CALL(CHART_ENTER_ENTRY_FUNCTION_TAG,0);
    chartInstance->c1_is_active_c1_rov_design_analysis = 1;
    _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
    _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,2);
    chartInstance->c1_is_c1_rov_design_analysis = (uint8_T)c1_IN_off;
    _SFD_CS_CALL(STATE_ACTIVE_TAG,0);
    chartInstance->c1_tp_off = 1;
    *c1_L(chartInstance) = 1.0;
    _SFD_DATA_RANGE_CHECK(*c1_L(chartInstance), 0U);
  } else {
    switch(chartInstance->c1_is_c1_rov_design_analysis) {
     case c1_IN_off:
      CV_CHART_EVAL(0,0,1);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,0);
      if(CV_TRANSITION_EVAL(0U, (int32_T)_SFD_CCP_CALL(0,0,(*c1_V(chartInstance)
          == 0.0))) != 0) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,0);
        chartInstance->c1_tp_off = 0;
        chartInstance->c1_is_c1_rov_design_analysis =
          (uint8_T)c1_IN_NO_ACTIVE_CHILD;
        _SFD_CS_CALL(STATE_INACTIVE_TAG,0);
        chartInstance->c1_is_c1_rov_design_analysis = (uint8_T)c1_IN_on;
        _SFD_CS_CALL(STATE_ACTIVE_TAG,1);
        chartInstance->c1_tp_on = 1;
        *c1_L(chartInstance) = 0.0;
        _SFD_DATA_RANGE_CHECK(*c1_L(chartInstance), 0U);
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
      break;
     case c1_IN_on:
      CV_CHART_EVAL(0,0,2);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG,1);
      if(CV_TRANSITION_EVAL(1U, (int32_T)_SFD_CCP_CALL(1,0,(*c1_V(chartInstance)
          > 0.0))) != 0) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG,1);
        chartInstance->c1_tp_on = 0;
        chartInstance->c1_is_c1_rov_design_analysis =
          (uint8_T)c1_IN_NO_ACTIVE_CHILD;
        _SFD_CS_CALL(STATE_INACTIVE_TAG,1);
        chartInstance->c1_is_c1_rov_design_analysis = (uint8_T)c1_IN_off;
        _SFD_CS_CALL(STATE_ACTIVE_TAG,0);
        chartInstance->c1_tp_off = 1;
        *c1_L(chartInstance) = 1.0;
        _SFD_DATA_RANGE_CHECK(*c1_L(chartInstance), 0U);
      }
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG,1);
      break;
     default:
      CV_CHART_EVAL(0,0,0);
      break;
    }
  }
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG,0);
  _sfEvent_ = c1_previousEvent;
}

static int8_T c1__s8_d_(SFc1_rov_design_analysisInstanceStruct *chartInstance,
 real_T c1_b)
{
  int8_T c1_a;
  c1_a = (int8_T)c1_b;
  if((real_T)c1_a != (c1_b < 0.0 ? ceil(c1_b) : floor(c1_b))) {
    sf_debug_overflow_detection(0);
  }
  return c1_a;
}

static real_T *c1_L(SFc1_rov_design_analysisInstanceStruct *chartInstance)
{
  return (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
}

static real_T *c1_V(SFc1_rov_design_analysisInstanceStruct *chartInstance)
{
  return (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
}

static void init_test_point_addr_map(SFc1_rov_design_analysisInstanceStruct
 *chartInstance)
{
  chartInstance->c1_testPointAddrMap[0] = &chartInstance->c1_tp_off;
  chartInstance->c1_testPointAddrMap[1] = &chartInstance->c1_tp_on;
}

static void **get_test_point_address_map(SFc1_rov_design_analysisInstanceStruct
 *chartInstance)
{
  return &chartInstance->c1_testPointAddrMap[0];
}

static rtwCAPI_ModelMappingInfo
*get_test_point_mapping_info(SFc1_rov_design_analysisInstanceStruct
 *chartInstance)
{
  return &chartInstance->c1_testPointMappingInfo;
}

/* SFunction Glue Code */
static void init_test_point_mapping_info(SimStruct *S);
void sf_c1_rov_design_analysis_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3019258681U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1215804055U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(446596295U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(794820937U);
}

mxArray *sf_c1_rov_design_analysis_get_autoinheritance_info(void)
{
  const char *autoinheritanceInfoStructFieldNames[] =
  {"checksum","inputTypes","outputSizes","outputTypes"};
  mxArray *mxAutoinheritanceInfo = NULL;
  mxArray *mxChecksum = NULL;
  mxArray *mxInputTypes = NULL;
  mxArray *mxOutputSizes = NULL;
  mxArray *mxOutputTypes = NULL;

  mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,
   sizeof(autoinheritanceInfoStructFieldNames)/sizeof(char *),
   autoinheritanceInfoStructFieldNames);

  mxChecksum = mxCreateDoubleMatrix(4,1,mxREAL);
  ((real_T *)mxGetPr((mxChecksum)))[0] = (real_T)(0U);
  ((real_T *)mxGetPr((mxChecksum)))[1] = (real_T)(0U);
  ((real_T *)mxGetPr((mxChecksum)))[2] = (real_T)(0U);
  ((real_T *)mxGetPr((mxChecksum)))[3] = (real_T)(0U);

  mxInputTypes = mxCreateDoubleMatrix(1,1,mxREAL);
  ((real_T *)mxGetPr((mxInputTypes)))[0] = (real_T)(10U);

  mxOutputSizes = mxCreateDoubleMatrix(2,1,mxREAL);
  mxOutputTypes = mxCreateDoubleMatrix(1,1,mxREAL);
  ((real_T *)mxGetPr((mxOutputSizes)))[0] = (real_T)(1U);
  ((real_T *)mxGetPr((mxOutputSizes)))[1] = (real_T)(1U);
  ((real_T *)mxGetPr((mxOutputTypes)))[0] = (real_T)(4U);

  mxSetFieldByNumber(mxAutoinheritanceInfo,0,0,mxChecksum);
  mxSetFieldByNumber(mxAutoinheritanceInfo,0,1,mxInputTypes);
  mxSetFieldByNumber(mxAutoinheritanceInfo,0,2,mxOutputSizes);
  mxSetFieldByNumber(mxAutoinheritanceInfo,0,3,mxOutputTypes);

  return(mxAutoinheritanceInfo);
}

static void chart_debug_initialization(SimStruct *S)
{
  SFc1_rov_design_analysisInstanceStruct *chartInstance;
  chartInstance = ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
  if(ssIsFirstInitCond(S)) {
    /* do this only if simulation is starting */
    if(!sim_mode_is_rtw_gen(S)) {
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent =
          sf_debug_initialize_chart(_rov_design_analysisMachineNumber_,
         1,
         2,
         3,
         2,
         0,
         0,
         0,
         0,
         &(chartInstance->chartNumber),
         &(chartInstance->instanceNumber),
         ssGetPath(S),
         (void *)S);
        if(chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting(_rov_design_analysisMachineNumber_,chartInstance->chartNumber,0);
          sf_debug_set_chart_event_thresholds(_rov_design_analysisMachineNumber_,
           chartInstance->chartNumber,
           0,
           0,
           0);

          _SFD_SET_DATA_PROPS(0,2,0,1,SF_INT8,0,NULL,0,0.0,1.0,0,"L",0);
          _SFD_SET_DATA_PROPS(1,1,1,0,SF_DOUBLE,0,NULL,0,0.0,1.0,0,"V",0);
          _SFD_STATE_INFO(0,0,0);
          _SFD_STATE_INFO(1,0,0);
          _SFD_CH_SUBSTATE_COUNT(2);
          _SFD_CH_SUBSTATE_DECOMP(0);
          _SFD_CH_SUBSTATE_INDEX(0,0);
          _SFD_CH_SUBSTATE_INDEX(1,1);
          _SFD_ST_SUBSTATE_COUNT(0,0);
          _SFD_ST_SUBSTATE_COUNT(1,0);
        }
        _SFD_CV_INIT_CHART(2,1,0,0);
        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }
        {
          _SFD_CV_INIT_STATE(1,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(2,0,NULL,NULL,0,NULL);

        {
          static unsigned int sStartGuardMap[] = {1};
          static unsigned int sEndGuardMap[] = {5};
          static int sPostFixPredicateTree[] = {0};
          _SFD_CV_INIT_TRANS(0,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
        }
        {
          static unsigned int sStartGuardMap[] = {1};
          static unsigned int sEndGuardMap[] = {4};
          static int sPostFixPredicateTree[] = {0};
          _SFD_CV_INIT_TRANS(1,1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),1,&(sPostFixPredicateTree[0]));
        }
        _SFD_TRANS_COV_WTS(2,0,0,0,0);
        if(chartAlreadyPresent==0)
        {
          _SFD_TRANS_COV_MAPS(2,
           0,NULL,NULL,
           0,NULL,NULL,
           0,NULL,NULL,
           0,NULL,NULL);
        }
        _SFD_TRANS_COV_WTS(0,0,1,0,0);
        if(chartAlreadyPresent==0)
        {
          static unsigned int sStartGuardMap[] = {1};
          static unsigned int sEndGuardMap[] = {5};
          _SFD_TRANS_COV_MAPS(0,
           0,NULL,NULL,
           1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
           0,NULL,NULL,
           0,NULL,NULL);
        }
        _SFD_TRANS_COV_WTS(1,0,1,0,0);
        if(chartAlreadyPresent==0)
        {
          static unsigned int sStartGuardMap[] = {1};
          static unsigned int sEndGuardMap[] = {4};
          _SFD_TRANS_COV_MAPS(1,
           0,NULL,NULL,
           1,&(sStartGuardMap[0]),&(sEndGuardMap[0]),
           0,NULL,NULL,
           0,NULL,NULL);
        }
        _SFD_SET_DATA_VALUE_PTR(0U, c1_L(chartInstance));
        _SFD_SET_DATA_VALUE_PTR(1U, c1_V(chartInstance));
      }
    }
  } else {
    sf_debug_reset_current_state_configuration(_rov_design_analysisMachineNumber_,chartInstance->chartNumber,chartInstance->instanceNumber);
  }
}

static void sf_opaque_initialize_c1_rov_design_analysis(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_rov_design_analysisInstanceStruct*)
    chartInstanceVar)->S);
  initialize_c1_rov_design_analysis((SFc1_rov_design_analysisInstanceStruct*)
   chartInstanceVar);
}

static void sf_opaque_enable_c1_rov_design_analysis(void *chartInstanceVar)
{
  enable_c1_rov_design_analysis((SFc1_rov_design_analysisInstanceStruct*)
   chartInstanceVar);
}

static void sf_opaque_disable_c1_rov_design_analysis(void *chartInstanceVar)
{
  disable_c1_rov_design_analysis((SFc1_rov_design_analysisInstanceStruct*)
   chartInstanceVar);
}

static void sf_opaque_gateway_c1_rov_design_analysis(void *chartInstanceVar)
{
  sf_c1_rov_design_analysis((SFc1_rov_design_analysisInstanceStruct*)
   chartInstanceVar);
}

static void sf_opaque_terminate_c1_rov_design_analysis(void *chartInstanceVar)
{
  if(chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_rov_design_analysisInstanceStruct*)
      chartInstanceVar)->S;
    sf_clear_rtw_identifier(S);
    finalize_c1_rov_design_analysis((SFc1_rov_design_analysisInstanceStruct*)
     chartInstanceVar);
    if(!sim_mode_is_rtw_gen(S)) {
      ssSetModelMappingInfoPtr(S, NULL);
    }
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void mdlSetWorkWidths_c1_rov_design_analysis(SimStruct *S)
{
  if(sim_mode_is_rtw_gen(S)) {
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable("rov_design_analysis",1);
    int_T chartIsMultiInstanced =
      (int_T)sf_is_chart_multi_instanced("rov_design_analysis",1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    if(chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,"rov_design_analysis",1,1);
      sf_mark_chart_reusable_outputs(S,"rov_design_analysis",1,1);
    }
    if (!sf_is_chart_instance_optimized_out("rov_design_analysis",1)) {
      int dtId;
      char *chartInstanceTypedefName =
        sf_chart_instance_typedef_name("rov_design_analysis",1);
      dtId = ssRegisterDataType(S, chartInstanceTypedefName);
      if (dtId == INVALID_DTYPE_ID ) return;
      /* Register the size of the udt */
      if (!ssSetDataTypeSize(S, dtId, 8)) return;
      if(!ssSetNumDWork(S,1)) return;
      ssSetDWorkDataType(S, 0, dtId);
      ssSetDWorkWidth(S, 0, 1);
      ssSetDWorkName(S, 0, "ChartInstance"); /*optional name, less than 16 chars*/
      sf_set_rtw_identifier(S);
    }
    ssSetHasSubFunctions(S,!(chartIsInlinable));
    ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  }

  ssSetChecksum0(S,(3019258681U));
  ssSetChecksum1(S,(1215804055U));
  ssSetChecksum2(S,(446596295U));
  ssSetChecksum3(S,(794820937U));

  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c1_rov_design_analysis(SimStruct *S)
{
  sf_write_symbol_mapping(S, "rov_design_analysis", 1);
  ssWriteRTWStrParam(S, "StateflowChartType", "Stateflow");
}

static void mdlStart_c1_rov_design_analysis(SimStruct *S)
{
  SFc1_rov_design_analysisInstanceStruct *chartInstance;
  chartInstance = (SFc1_rov_design_analysisInstanceStruct
    *)malloc(sizeof(SFc1_rov_design_analysisInstanceStruct));
  if(chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 0;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c1_rov_design_analysis;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c1_rov_design_analysis;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c1_rov_design_analysis;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c1_rov_design_analysis;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c1_rov_design_analysis;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_rov_design_analysis;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_rov_design_analysis;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c1_rov_design_analysis;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->S = S;
  ssSetUserData(S,(void *)(&(chartInstance->chartInfo))); /* register the chart instance with simstruct */

  if(!sim_mode_is_rtw_gen(S)) {
    init_test_point_mapping_info(S);
  }
}

void c1_rov_design_analysis_method_dispatcher(SimStruct *S, int_T method, void
 *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_rov_design_analysis(S);
    break;
   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_rov_design_analysis(S);
    break;
   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
     "Error calling c1_rov_design_analysis_method_dispatcher.\n"
     "Can't handle method %d.\n", method);
    break;
  }
}

static const rtwCAPI_DataTypeMap dataTypeMap[] = {
  /* cName, mwName, numElements, elemMapIndex, dataSize, slDataId, isComplex, isPointer */
  {"uint8_T", "uint8_T", 0, 0, sizeof(uint8_T), SS_UINT8, 0, 0}
};

static real_T fixPtSlopeBiasVals[2] = {
  0,
  1
};

static const rtwCAPI_FixPtMap fixedPointMap[] = {
  /* *fracSlope, *bias, scaleType, exponent, isSigned */
  {NULL, NULL, rtwCAPI_FIX_RESERVED, 0, 0}
};

static const rtwCAPI_DimensionMap dimensionMap[] = {
  /* dataOrientation, dimArrayIndex, numDims*/
  {rtwCAPI_SCALAR, 0, 2}
};

static const uint_T dimensionArray[] = {
  1, 1
};

static real_T sfCAPIsampleTimeZero = 0.0;
static const rtwCAPI_SampleTimeMap sampleTimeMap[] = {
  /* *period, *offset, taskId, contextTid, mode */
  {&sfCAPIsampleTimeZero, &sfCAPIsampleTimeZero, 0, 0, 0}
};

static const rtwCAPI_Signals testPointSignals[] = {
  /* addrMapIndex, sysNum, SFRelativePath, dataName, portNumber, dataTypeIndex, dimIndex, fixPtIdx, sTimeIndex */
  {0, 0, "StateflowChart/off", "off", 0, 0, 0, 0, 0},
  {1, 0, "StateflowChart/on", "on", 0, 0, 0, 0, 0}
};

static rtwCAPI_ModelMappingStaticInfo testPointMappingStaticInfo = {
  /* block signal monitoring */
  {
    testPointSignals,                   /* Block signals Array  */
    2                                   /* Num Block IO signals */
  },

  /* parameter tuning */
  {
    NULL,                               /* Block parameters Array    */
    0,                                  /* Num block parameters      */
    NULL,                               /* Variable parameters Array */
    0                                   /* Num variable parameters   */
  },

  /* block states */
  {
    NULL,                               /* Block States array        */
    0                                   /* Num Block States          */
  },

  /* Static maps */
  {
    dataTypeMap,                        /* Data Type Map            */
    dimensionMap,                       /* Data Dimension Map       */
    fixedPointMap,                      /* Fixed Point Map          */
    NULL,                               /* Structure Element map    */
    sampleTimeMap,                      /* Sample Times Map         */
    dimensionArray                      /* Dimension Array          */
  },

  /* Target type */
  "float"
};

static void init_test_point_mapping_info(SimStruct *S) {
  rtwCAPI_ModelMappingInfo *testPointMappingInfo;
  void **testPointAddrMap;
  SFc1_rov_design_analysisInstanceStruct *chartInstance;

  chartInstance = ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
  init_test_point_addr_map(chartInstance);
  testPointMappingInfo = get_test_point_mapping_info(chartInstance);
  testPointAddrMap = get_test_point_address_map(chartInstance);

  rtwCAPI_SetStaticMap(*testPointMappingInfo, &testPointMappingStaticInfo);
  rtwCAPI_SetPath(*testPointMappingInfo, "");
  rtwCAPI_SetFullPath(*testPointMappingInfo, NULL);
  rtwCAPI_SetDataAddressMap(*testPointMappingInfo, testPointAddrMap);
  rtwCAPI_SetChildMMIArray(*testPointMappingInfo, NULL);
  rtwCAPI_SetChildMMIArrayLen(*testPointMappingInfo, 0);

  ssSetModelMappingInfoPtr(S, testPointMappingInfo);
}

