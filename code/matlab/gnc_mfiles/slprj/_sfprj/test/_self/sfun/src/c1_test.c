/* Include files */

#include "blascompat32.h"
#include "test_sfun.h"
#include "c1_test.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "test_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c1_debug_family_names[6] = { "J", "nargin", "nargout",
  "ee_velocity", "euler_angles_ee", "ee_rates" };

/* Function Declarations */
static void initialize_c1_test(SFc1_testInstanceStruct *chartInstance);
static void initialize_params_c1_test(SFc1_testInstanceStruct *chartInstance);
static void enable_c1_test(SFc1_testInstanceStruct *chartInstance);
static void disable_c1_test(SFc1_testInstanceStruct *chartInstance);
static void c1_update_debugger_state_c1_test(SFc1_testInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c1_test(SFc1_testInstanceStruct
  *chartInstance);
static void set_sim_state_c1_test(SFc1_testInstanceStruct *chartInstance, const
  mxArray *c1_st);
static void finalize_c1_test(SFc1_testInstanceStruct *chartInstance);
static void sf_c1_test(SFc1_testInstanceStruct *chartInstance);
static void initSimStructsc1_test(SFc1_testInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_emlrt_marshallIn(SFc1_testInstanceStruct *chartInstance, const
  mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static void c1_b_emlrt_marshallIn(SFc1_testInstanceStruct *chartInstance, const
  mxArray *c1_mtimes, const char_T *c1_identifier, real_T c1_y[6]);
static void c1_c_emlrt_marshallIn(SFc1_testInstanceStruct *chartInstance, const
  mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[6]);
static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_d_emlrt_marshallIn(SFc1_testInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static uint8_T c1_e_emlrt_marshallIn(SFc1_testInstanceStruct *chartInstance,
  const mxArray *c1_b_is_active_c1_test, const char_T *c1_identifier);
static uint8_T c1_f_emlrt_marshallIn(SFc1_testInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void init_dsm_address_info(SFc1_testInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c1_test(SFc1_testInstanceStruct *chartInstance)
{
  chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c1_is_active_c1_test = 0U;
}

static void initialize_params_c1_test(SFc1_testInstanceStruct *chartInstance)
{
}

static void enable_c1_test(SFc1_testInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c1_test(SFc1_testInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c1_update_debugger_state_c1_test(SFc1_testInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c1_test(SFc1_testInstanceStruct
  *chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_y = NULL;
  int32_T c1_i0;
  real_T c1_u[6];
  const mxArray *c1_b_y = NULL;
  uint8_T c1_hoistedGlobal;
  uint8_T c1_b_u;
  const mxArray *c1_c_y = NULL;
  real_T (*c1_ee_rates)[6];
  c1_ee_rates = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c1_st = NULL;
  c1_st = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createcellarray(2), FALSE);
  for (c1_i0 = 0; c1_i0 < 6; c1_i0++) {
    c1_u[c1_i0] = (*c1_ee_rates)[c1_i0];
  }

  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_setcell(c1_y, 0, c1_b_y);
  c1_hoistedGlobal = chartInstance->c1_is_active_c1_test;
  c1_b_u = c1_hoistedGlobal;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 1, c1_c_y);
  sf_mex_assign(&c1_st, c1_y, FALSE);
  return c1_st;
}

static void set_sim_state_c1_test(SFc1_testInstanceStruct *chartInstance, const
  mxArray *c1_st)
{
  const mxArray *c1_u;
  real_T c1_dv0[6];
  int32_T c1_i1;
  real_T (*c1_ee_rates)[6];
  c1_ee_rates = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c1_doneDoubleBufferReInit = TRUE;
  c1_u = sf_mex_dup(c1_st);
  c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 0)),
                        "ee_rates", c1_dv0);
  for (c1_i1 = 0; c1_i1 < 6; c1_i1++) {
    (*c1_ee_rates)[c1_i1] = c1_dv0[c1_i1];
  }

  chartInstance->c1_is_active_c1_test = c1_e_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c1_u, 1)), "is_active_c1_test");
  sf_mex_destroy(&c1_u);
  c1_update_debugger_state_c1_test(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void finalize_c1_test(SFc1_testInstanceStruct *chartInstance)
{
}

static void sf_c1_test(SFc1_testInstanceStruct *chartInstance)
{
  int32_T c1_i2;
  int32_T c1_i3;
  int32_T c1_i4;
  int32_T c1_i5;
  real_T c1_ee_velocity[6];
  int32_T c1_i6;
  real_T c1_euler_angles_ee[3];
  uint32_T c1_debug_family_var_map[6];
  const mxArray *c1_J = NULL;
  real_T c1_nargin = 2.0;
  real_T c1_nargout = 1.0;
  real_T c1_ee_rates[6];
  int32_T c1_i7;
  real_T c1_u[3];
  const mxArray *c1_y = NULL;
  int32_T c1_i8;
  int32_T c1_i9;
  real_T c1_b_u[6];
  const mxArray *c1_b_y = NULL;
  real_T c1_dv1[6];
  int32_T c1_i10;
  int32_T c1_i11;
  real_T (*c1_b_ee_rates)[6];
  real_T (*c1_b_euler_angles_ee)[3];
  real_T (*c1_b_ee_velocity)[6];
  c1_b_euler_angles_ee = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c1_b_ee_rates = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c1_b_ee_velocity = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  for (c1_i2 = 0; c1_i2 < 6; c1_i2++) {
    _SFD_DATA_RANGE_CHECK((*c1_b_ee_velocity)[c1_i2], 0U);
  }

  for (c1_i3 = 0; c1_i3 < 6; c1_i3++) {
    _SFD_DATA_RANGE_CHECK((*c1_b_ee_rates)[c1_i3], 1U);
  }

  for (c1_i4 = 0; c1_i4 < 3; c1_i4++) {
    _SFD_DATA_RANGE_CHECK((*c1_b_euler_angles_ee)[c1_i4], 2U);
  }

  chartInstance->c1_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  for (c1_i5 = 0; c1_i5 < 6; c1_i5++) {
    c1_ee_velocity[c1_i5] = (*c1_b_ee_velocity)[c1_i5];
  }

  for (c1_i6 = 0; c1_i6 < 3; c1_i6++) {
    c1_euler_angles_ee[c1_i6] = (*c1_b_euler_angles_ee)[c1_i6];
  }

  sf_debug_symbol_scope_push_eml(0U, 6U, 6U, c1_debug_family_names,
    c1_debug_family_var_map);
  sf_debug_symbol_scope_add_eml(&c1_J, 0U, c1_d_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c1_nargin, 1U, c1_c_sf_marshallOut,
    c1_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_nargout, 2U, c1_c_sf_marshallOut,
    c1_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c1_ee_velocity, 3U, c1_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c1_euler_angles_ee, 4U, c1_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c1_ee_rates, 5U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 3);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 4);
  for (c1_i7 = 0; c1_i7 < 3; c1_i7++) {
    c1_u[c1_i7] = c1_euler_angles_ee[c1_i7];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c1_J, sf_mex_call_debug("analytical_jacobian", 1U, 1U, 14, c1_y),
                FALSE);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 6);
  for (c1_i8 = 0; c1_i8 < 6; c1_i8++) {
    c1_ee_rates[c1_i8] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 7);
  for (c1_i9 = 0; c1_i9 < 6; c1_i9++) {
    c1_b_u[c1_i9] = c1_ee_velocity[c1_i9];
  }

  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_b_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  c1_b_emlrt_marshallIn(chartInstance, sf_mex_call_debug("mtimes", 1U, 2U, 14,
    sf_mex_dup(c1_J), 14, c1_b_y), "mtimes", c1_dv1);
  for (c1_i10 = 0; c1_i10 < 6; c1_i10++) {
    c1_ee_rates[c1_i10] = c1_dv1[c1_i10];
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -7);
  sf_debug_symbol_scope_pop();
  sf_mex_destroy(&c1_J);
  for (c1_i11 = 0; c1_i11 < 6; c1_i11++) {
    (*c1_b_ee_rates)[c1_i11] = c1_ee_rates[c1_i11];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  sf_debug_check_for_state_inconsistency(_testMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc1_test(SFc1_testInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber)
{
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i12;
  real_T c1_b_inData[6];
  int32_T c1_i13;
  real_T c1_u[6];
  const mxArray *c1_y = NULL;
  SFc1_testInstanceStruct *chartInstance;
  chartInstance = (SFc1_testInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i12 = 0; c1_i12 < 6; c1_i12++) {
    c1_b_inData[c1_i12] = (*(real_T (*)[6])c1_inData)[c1_i12];
  }

  for (c1_i13 = 0; c1_i13 < 6; c1_i13++) {
    c1_u[c1_i13] = c1_b_inData[c1_i13];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_mtimes;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y[6];
  int32_T c1_i14;
  SFc1_testInstanceStruct *chartInstance;
  chartInstance = (SFc1_testInstanceStruct *)chartInstanceVoid;
  c1_mtimes = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_mtimes), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_mtimes);
  for (c1_i14 = 0; c1_i14 < 6; c1_i14++) {
    (*(real_T (*)[6])c1_outData)[c1_i14] = c1_y[c1_i14];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i15;
  real_T c1_b_inData[3];
  int32_T c1_i16;
  real_T c1_u[3];
  const mxArray *c1_y = NULL;
  SFc1_testInstanceStruct *chartInstance;
  chartInstance = (SFc1_testInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i15 = 0; c1_i15 < 3; c1_i15++) {
    c1_b_inData[c1_i15] = (*(real_T (*)[3])c1_inData)[c1_i15];
  }

  for (c1_i16 = 0; c1_i16 < 3; c1_i16++) {
    c1_u[c1_i16] = c1_b_inData[c1_i16];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_testInstanceStruct *chartInstance;
  chartInstance = (SFc1_testInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static real_T c1_emlrt_marshallIn(SFc1_testInstanceStruct *chartInstance, const
  mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d0;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d0, 1, 0, 0U, 0, 0U, 0);
  c1_y = c1_d0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_nargout;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_testInstanceStruct *chartInstance;
  chartInstance = (SFc1_testInstanceStruct *)chartInstanceVoid;
  c1_nargout = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_nargout), &c1_thisId);
  sf_mex_destroy(&c1_nargout);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_d_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  const mxArray *c1_u;
  const mxArray *c1_y = NULL;
  SFc1_testInstanceStruct *chartInstance;
  chartInstance = (SFc1_testInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = sf_mex_dup(*(const mxArray **)c1_inData);
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_duplicatearraysafe(&c1_u), FALSE);
  sf_mex_destroy(&c1_u);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

const mxArray *sf_c1_test_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo;
  c1_ResolvedFunctionInfo c1_info[1];
  c1_ResolvedFunctionInfo (*c1_b_info)[1];
  const mxArray *c1_m0 = NULL;
  int32_T c1_i17;
  c1_ResolvedFunctionInfo *c1_r0;
  c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  c1_b_info = (c1_ResolvedFunctionInfo (*)[1])c1_info;
  (*c1_b_info)[0].context = "";
  (*c1_b_info)[0].name = "mtimes";
  (*c1_b_info)[0].dominantType = "double";
  (*c1_b_info)[0].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  (*c1_b_info)[0].fileTimeLo = 1289519692U;
  (*c1_b_info)[0].fileTimeHi = 0U;
  (*c1_b_info)[0].mFileTimeLo = 0U;
  (*c1_b_info)[0].mFileTimeHi = 0U;
  sf_mex_assign(&c1_m0, sf_mex_createstruct("nameCaptureInfo", 1, 1), FALSE);
  for (c1_i17 = 0; c1_i17 < 1; c1_i17++) {
    c1_r0 = &c1_info[c1_i17];
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c1_r0->context)), "context", "nameCaptureInfo",
                    c1_i17);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c1_r0->name)), "name", "nameCaptureInfo", c1_i17);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c1_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c1_i17);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c1_r0->resolved)), "resolved", "nameCaptureInfo",
                    c1_i17);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c1_i17);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c1_i17);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c1_i17);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c1_i17);
  }

  sf_mex_assign(&c1_nameCaptureInfo, c1_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c1_nameCaptureInfo);
  return c1_nameCaptureInfo;
}

static void c1_b_emlrt_marshallIn(SFc1_testInstanceStruct *chartInstance, const
  mxArray *c1_mtimes, const char_T *c1_identifier, real_T c1_y[6])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_mtimes), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_mtimes);
}

static void c1_c_emlrt_marshallIn(SFc1_testInstanceStruct *chartInstance, const
  mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_y[6])
{
  real_T c1_dv2[6];
  int32_T c1_i18;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv2, 1, 0, 0U, 1, 0U, 1, 6);
  for (c1_i18 = 0; c1_i18 < 6; c1_i18++) {
    c1_y[c1_i18] = c1_dv2[c1_i18];
  }

  sf_mex_destroy(&c1_u);
}

static const mxArray *c1_e_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_testInstanceStruct *chartInstance;
  chartInstance = (SFc1_testInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(int32_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static int32_T c1_d_emlrt_marshallIn(SFc1_testInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_y;
  int32_T c1_i19;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i19, 1, 6, 0U, 0, 0U, 0);
  c1_y = c1_i19;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_sfEvent;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_y;
  SFc1_testInstanceStruct *chartInstance;
  chartInstance = (SFc1_testInstanceStruct *)chartInstanceVoid;
  c1_b_sfEvent = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_sfEvent),
    &c1_thisId);
  sf_mex_destroy(&c1_b_sfEvent);
  *(int32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static uint8_T c1_e_emlrt_marshallIn(SFc1_testInstanceStruct *chartInstance,
  const mxArray *c1_b_is_active_c1_test, const char_T *c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_is_active_c1_test),
    &c1_thisId);
  sf_mex_destroy(&c1_b_is_active_c1_test);
  return c1_y;
}

static uint8_T c1_f_emlrt_marshallIn(SFc1_testInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u0;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u0, 1, 3, 0U, 0, 0U, 0);
  c1_y = c1_u0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void init_dsm_address_info(SFc1_testInstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c1_test_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(218141713U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3660020262U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3410982697U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1061310790U);
}

mxArray *sf_c1_test_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("usafqUiVVQ0PIFgUb92QKF");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

static const mxArray *sf_get_sim_state_info_c1_test(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"ee_rates\",},{M[8],M[0],T\"is_active_c1_test\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_test_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_testInstanceStruct *chartInstance;
    chartInstance = (SFc1_testInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart(_testMachineNumber_,
          1,
          1,
          1,
          3,
          0,
          0,
          0,
          0,
          0,
          &(chartInstance->chartNumber),
          &(chartInstance->instanceNumber),
          ssGetPath(S),
          (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation(_testMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting(_testMachineNumber_,
            chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(_testMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"ee_velocity");
          _SFD_SET_DATA_PROPS(1,2,0,1,"ee_rates");
          _SFD_SET_DATA_PROPS(2,1,1,0,"euler_angles_ee");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,238);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)
            c1_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real_T (*c1_ee_velocity)[6];
          real_T (*c1_ee_rates)[6];
          real_T (*c1_euler_angles_ee)[3];
          c1_euler_angles_ee = (real_T (*)[3])ssGetInputPortSignal
            (chartInstance->S, 1);
          c1_ee_rates = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
          c1_ee_velocity = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S,
            0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c1_ee_velocity);
          _SFD_SET_DATA_VALUE_PTR(1U, *c1_ee_rates);
          _SFD_SET_DATA_VALUE_PTR(2U, *c1_euler_angles_ee);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(_testMachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization()
{
  return "V0wR3nMqfX8byC3lHIsJBE";
}

static void sf_opaque_initialize_c1_test(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_testInstanceStruct*) chartInstanceVar)->S,0);
  initialize_params_c1_test((SFc1_testInstanceStruct*) chartInstanceVar);
  initialize_c1_test((SFc1_testInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c1_test(void *chartInstanceVar)
{
  enable_c1_test((SFc1_testInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c1_test(void *chartInstanceVar)
{
  disable_c1_test((SFc1_testInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c1_test(void *chartInstanceVar)
{
  sf_c1_test((SFc1_testInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c1_test(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c1_test((SFc1_testInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c1_test();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c1_test(SimStruct* S, const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c1_test();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c1_test((SFc1_testInstanceStruct*)chartInfo->chartInstance,
                        mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c1_test(SimStruct* S)
{
  return sf_internal_get_sim_state_c1_test(S);
}

static void sf_opaque_set_sim_state_c1_test(SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c1_test(S, st);
}

static void sf_opaque_terminate_c1_test(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_testInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c1_test((SFc1_testInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_test_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_test((SFc1_testInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_test(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c1_test((SFc1_testInstanceStruct*)(((ChartInfoStruct *)
      ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c1_test(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_test_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,1,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,1,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,1);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3842407489U));
  ssSetChecksum1(S,(3239262986U));
  ssSetChecksum2(S,(1931821459U));
  ssSetChecksum3(S,(2626391849U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c1_test(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c1_test(SimStruct *S)
{
  SFc1_testInstanceStruct *chartInstance;
  chartInstance = (SFc1_testInstanceStruct *)malloc(sizeof
    (SFc1_testInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc1_testInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c1_test;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c1_test;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c1_test;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c1_test;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c1_test;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c1_test;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c1_test;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c1_test;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_test;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_test;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c1_test;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->S = S;
  ssSetUserData(S,(void *)(&(chartInstance->chartInfo)));/* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c1_test_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_test(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_test(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_test(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_test_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
