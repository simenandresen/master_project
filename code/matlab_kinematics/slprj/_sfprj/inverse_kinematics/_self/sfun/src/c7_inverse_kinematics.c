/* Include files */

#include "blascompat32.h"
#include "inverse_kinematics_sfun.h"
#include "c7_inverse_kinematics.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "inverse_kinematics_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c7_debug_family_names[6] = { "J", "nargin", "nargout",
  "ee_velocity", "euler_angles_ee", "ee_rates" };

/* Function Declarations */
static void initialize_c7_inverse_kinematics
  (SFc7_inverse_kinematicsInstanceStruct *chartInstance);
static void initialize_params_c7_inverse_kinematics
  (SFc7_inverse_kinematicsInstanceStruct *chartInstance);
static void enable_c7_inverse_kinematics(SFc7_inverse_kinematicsInstanceStruct
  *chartInstance);
static void disable_c7_inverse_kinematics(SFc7_inverse_kinematicsInstanceStruct *
  chartInstance);
static void c7_update_debugger_state_c7_inverse_kinematics
  (SFc7_inverse_kinematicsInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c7_inverse_kinematics
  (SFc7_inverse_kinematicsInstanceStruct *chartInstance);
static void set_sim_state_c7_inverse_kinematics
  (SFc7_inverse_kinematicsInstanceStruct *chartInstance, const mxArray *c7_st);
static void finalize_c7_inverse_kinematics(SFc7_inverse_kinematicsInstanceStruct
  *chartInstance);
static void sf_c7_inverse_kinematics(SFc7_inverse_kinematicsInstanceStruct
  *chartInstance);
static void initSimStructsc7_inverse_kinematics
  (SFc7_inverse_kinematicsInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c7_machineNumber, uint32_T
  c7_chartNumber);
static const mxArray *c7_sf_marshallOut(void *chartInstanceVoid, void *c7_inData);
static void c7_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_b_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static const mxArray *c7_c_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static real_T c7_emlrt_marshallIn(SFc7_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId);
static void c7_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_d_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_b_emlrt_marshallIn(SFc7_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c7_mtimes, const char_T *c7_identifier, real_T
  c7_y[6]);
static void c7_c_emlrt_marshallIn(SFc7_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[6]);
static const mxArray *c7_e_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static int32_T c7_d_emlrt_marshallIn(SFc7_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId);
static void c7_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static uint8_T c7_e_emlrt_marshallIn(SFc7_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c7_b_is_active_c7_inverse_kinematics, const
  char_T *c7_identifier);
static uint8_T c7_f_emlrt_marshallIn(SFc7_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId);
static void init_dsm_address_info(SFc7_inverse_kinematicsInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c7_inverse_kinematics
  (SFc7_inverse_kinematicsInstanceStruct *chartInstance)
{
  chartInstance->c7_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c7_is_active_c7_inverse_kinematics = 0U;
}

static void initialize_params_c7_inverse_kinematics
  (SFc7_inverse_kinematicsInstanceStruct *chartInstance)
{
}

static void enable_c7_inverse_kinematics(SFc7_inverse_kinematicsInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c7_inverse_kinematics(SFc7_inverse_kinematicsInstanceStruct *
  chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c7_update_debugger_state_c7_inverse_kinematics
  (SFc7_inverse_kinematicsInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c7_inverse_kinematics
  (SFc7_inverse_kinematicsInstanceStruct *chartInstance)
{
  const mxArray *c7_st;
  const mxArray *c7_y = NULL;
  int32_T c7_i0;
  real_T c7_u[6];
  const mxArray *c7_b_y = NULL;
  uint8_T c7_hoistedGlobal;
  uint8_T c7_b_u;
  const mxArray *c7_c_y = NULL;
  real_T (*c7_ee_rates)[6];
  c7_ee_rates = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c7_st = NULL;
  c7_st = NULL;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_createcellarray(2), FALSE);
  for (c7_i0 = 0; c7_i0 < 6; c7_i0++) {
    c7_u[c7_i0] = (*c7_ee_rates)[c7_i0];
  }

  c7_b_y = NULL;
  sf_mex_assign(&c7_b_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_setcell(c7_y, 0, c7_b_y);
  c7_hoistedGlobal = chartInstance->c7_is_active_c7_inverse_kinematics;
  c7_b_u = c7_hoistedGlobal;
  c7_c_y = NULL;
  sf_mex_assign(&c7_c_y, sf_mex_create("y", &c7_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c7_y, 1, c7_c_y);
  sf_mex_assign(&c7_st, c7_y, FALSE);
  return c7_st;
}

static void set_sim_state_c7_inverse_kinematics
  (SFc7_inverse_kinematicsInstanceStruct *chartInstance, const mxArray *c7_st)
{
  const mxArray *c7_u;
  real_T c7_dv0[6];
  int32_T c7_i1;
  real_T (*c7_ee_rates)[6];
  c7_ee_rates = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c7_doneDoubleBufferReInit = TRUE;
  c7_u = sf_mex_dup(c7_st);
  c7_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c7_u, 0)),
                        "ee_rates", c7_dv0);
  for (c7_i1 = 0; c7_i1 < 6; c7_i1++) {
    (*c7_ee_rates)[c7_i1] = c7_dv0[c7_i1];
  }

  chartInstance->c7_is_active_c7_inverse_kinematics = c7_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c7_u, 1)),
     "is_active_c7_inverse_kinematics");
  sf_mex_destroy(&c7_u);
  c7_update_debugger_state_c7_inverse_kinematics(chartInstance);
  sf_mex_destroy(&c7_st);
}

static void finalize_c7_inverse_kinematics(SFc7_inverse_kinematicsInstanceStruct
  *chartInstance)
{
}

static void sf_c7_inverse_kinematics(SFc7_inverse_kinematicsInstanceStruct
  *chartInstance)
{
  int32_T c7_i2;
  int32_T c7_i3;
  int32_T c7_i4;
  int32_T c7_i5;
  real_T c7_ee_velocity[6];
  int32_T c7_i6;
  real_T c7_euler_angles_ee[3];
  uint32_T c7_debug_family_var_map[6];
  const mxArray *c7_J = NULL;
  real_T c7_nargin = 2.0;
  real_T c7_nargout = 1.0;
  real_T c7_ee_rates[6];
  int32_T c7_i7;
  real_T c7_u[3];
  const mxArray *c7_y = NULL;
  int32_T c7_i8;
  int32_T c7_i9;
  real_T c7_b_u[6];
  const mxArray *c7_b_y = NULL;
  real_T c7_dv1[6];
  int32_T c7_i10;
  int32_T c7_i11;
  real_T (*c7_b_ee_rates)[6];
  real_T (*c7_b_euler_angles_ee)[3];
  real_T (*c7_b_ee_velocity)[6];
  c7_b_euler_angles_ee = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c7_b_ee_rates = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c7_b_ee_velocity = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 6U, chartInstance->c7_sfEvent);
  for (c7_i2 = 0; c7_i2 < 6; c7_i2++) {
    _SFD_DATA_RANGE_CHECK((*c7_b_ee_velocity)[c7_i2], 0U);
  }

  for (c7_i3 = 0; c7_i3 < 6; c7_i3++) {
    _SFD_DATA_RANGE_CHECK((*c7_b_ee_rates)[c7_i3], 1U);
  }

  for (c7_i4 = 0; c7_i4 < 3; c7_i4++) {
    _SFD_DATA_RANGE_CHECK((*c7_b_euler_angles_ee)[c7_i4], 2U);
  }

  chartInstance->c7_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 6U, chartInstance->c7_sfEvent);
  for (c7_i5 = 0; c7_i5 < 6; c7_i5++) {
    c7_ee_velocity[c7_i5] = (*c7_b_ee_velocity)[c7_i5];
  }

  for (c7_i6 = 0; c7_i6 < 3; c7_i6++) {
    c7_euler_angles_ee[c7_i6] = (*c7_b_euler_angles_ee)[c7_i6];
  }

  sf_debug_symbol_scope_push_eml(0U, 6U, 6U, c7_debug_family_names,
    c7_debug_family_var_map);
  sf_debug_symbol_scope_add_eml(&c7_J, 0U, c7_d_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c7_nargin, 1U, c7_c_sf_marshallOut,
    c7_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c7_nargout, 2U, c7_c_sf_marshallOut,
    c7_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c7_ee_velocity, 3U, c7_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c7_euler_angles_ee, 4U, c7_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c7_ee_rates, 5U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 3);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 4);
  for (c7_i7 = 0; c7_i7 < 3; c7_i7++) {
    c7_u[c7_i7] = c7_euler_angles_ee[c7_i7];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c7_J, sf_mex_call_debug("analytical_jacobian", 1U, 1U, 14, c7_y),
                FALSE);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 6);
  for (c7_i8 = 0; c7_i8 < 6; c7_i8++) {
    c7_ee_rates[c7_i8] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 7);
  for (c7_i9 = 0; c7_i9 < 6; c7_i9++) {
    c7_b_u[c7_i9] = c7_ee_velocity[c7_i9];
  }

  c7_b_y = NULL;
  sf_mex_assign(&c7_b_y, sf_mex_create("y", c7_b_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  c7_b_emlrt_marshallIn(chartInstance, sf_mex_call_debug("mtimes", 1U, 2U, 14,
    sf_mex_dup(c7_J), 14, c7_b_y), "mtimes", c7_dv1);
  for (c7_i10 = 0; c7_i10 < 6; c7_i10++) {
    c7_ee_rates[c7_i10] = c7_dv1[c7_i10];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, -7);
  sf_debug_symbol_scope_pop();
  sf_mex_destroy(&c7_J);
  for (c7_i11 = 0; c7_i11 < 6; c7_i11++) {
    (*c7_b_ee_rates)[c7_i11] = c7_ee_rates[c7_i11];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 6U, chartInstance->c7_sfEvent);
  sf_debug_check_for_state_inconsistency(_inverse_kinematicsMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc7_inverse_kinematics
  (SFc7_inverse_kinematicsInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c7_machineNumber, uint32_T
  c7_chartNumber)
{
}

static const mxArray *c7_sf_marshallOut(void *chartInstanceVoid, void *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i12;
  real_T c7_b_inData[6];
  int32_T c7_i13;
  real_T c7_u[6];
  const mxArray *c7_y = NULL;
  SFc7_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc7_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i12 = 0; c7_i12 < 6; c7_i12++) {
    c7_b_inData[c7_i12] = (*(real_T (*)[6])c7_inData)[c7_i12];
  }

  for (c7_i13 = 0; c7_i13 < 6; c7_i13++) {
    c7_u[c7_i13] = c7_b_inData[c7_i13];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static void c7_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_mtimes;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_y[6];
  int32_T c7_i14;
  SFc7_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc7_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c7_mtimes = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_mtimes), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_mtimes);
  for (c7_i14 = 0; c7_i14 < 6; c7_i14++) {
    (*(real_T (*)[6])c7_outData)[c7_i14] = c7_y[c7_i14];
  }

  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_b_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i15;
  real_T c7_b_inData[3];
  int32_T c7_i16;
  real_T c7_u[3];
  const mxArray *c7_y = NULL;
  SFc7_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc7_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i15 = 0; c7_i15 < 3; c7_i15++) {
    c7_b_inData[c7_i15] = (*(real_T (*)[3])c7_inData)[c7_i15];
  }

  for (c7_i16 = 0; c7_i16 < 3; c7_i16++) {
    c7_u[c7_i16] = c7_b_inData[c7_i16];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static const mxArray *c7_c_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  real_T c7_u;
  const mxArray *c7_y = NULL;
  SFc7_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc7_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_u = *(real_T *)c7_inData;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", &c7_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static real_T c7_emlrt_marshallIn(SFc7_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId)
{
  real_T c7_y;
  real_T c7_d0;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), &c7_d0, 1, 0, 0U, 0, 0U, 0);
  c7_y = c7_d0;
  sf_mex_destroy(&c7_u);
  return c7_y;
}

static void c7_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_nargout;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_y;
  SFc7_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc7_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c7_nargout = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_nargout), &c7_thisId);
  sf_mex_destroy(&c7_nargout);
  *(real_T *)c7_outData = c7_y;
  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_d_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  const mxArray *c7_u;
  const mxArray *c7_y = NULL;
  SFc7_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc7_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_u = sf_mex_dup(*(const mxArray **)c7_inData);
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_duplicatearraysafe(&c7_u), FALSE);
  sf_mex_destroy(&c7_u);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

const mxArray *sf_c7_inverse_kinematics_get_eml_resolved_functions_info(void)
{
  const mxArray *c7_nameCaptureInfo;
  c7_ResolvedFunctionInfo c7_info[1];
  c7_ResolvedFunctionInfo (*c7_b_info)[1];
  const mxArray *c7_m0 = NULL;
  int32_T c7_i17;
  c7_ResolvedFunctionInfo *c7_r0;
  c7_nameCaptureInfo = NULL;
  c7_nameCaptureInfo = NULL;
  c7_b_info = (c7_ResolvedFunctionInfo (*)[1])c7_info;
  (*c7_b_info)[0].context = "";
  (*c7_b_info)[0].name = "mtimes";
  (*c7_b_info)[0].dominantType = "double";
  (*c7_b_info)[0].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  (*c7_b_info)[0].fileTimeLo = 1289519692U;
  (*c7_b_info)[0].fileTimeHi = 0U;
  (*c7_b_info)[0].mFileTimeLo = 0U;
  (*c7_b_info)[0].mFileTimeHi = 0U;
  sf_mex_assign(&c7_m0, sf_mex_createstruct("nameCaptureInfo", 1, 1), FALSE);
  for (c7_i17 = 0; c7_i17 < 1; c7_i17++) {
    c7_r0 = &c7_info[c7_i17];
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", c7_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c7_r0->context)), "context", "nameCaptureInfo",
                    c7_i17);
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", c7_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c7_r0->name)), "name", "nameCaptureInfo", c7_i17);
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", c7_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c7_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c7_i17);
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", c7_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c7_r0->resolved)), "resolved", "nameCaptureInfo",
                    c7_i17);
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", &c7_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c7_i17);
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", &c7_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c7_i17);
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", &c7_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c7_i17);
    sf_mex_addfield(c7_m0, sf_mex_create("nameCaptureInfo", &c7_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c7_i17);
  }

  sf_mex_assign(&c7_nameCaptureInfo, c7_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c7_nameCaptureInfo);
  return c7_nameCaptureInfo;
}

static void c7_b_emlrt_marshallIn(SFc7_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c7_mtimes, const char_T *c7_identifier, real_T
  c7_y[6])
{
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_mtimes), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_mtimes);
}

static void c7_c_emlrt_marshallIn(SFc7_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[6])
{
  real_T c7_dv2[6];
  int32_T c7_i18;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), c7_dv2, 1, 0, 0U, 1, 0U, 1, 6);
  for (c7_i18 = 0; c7_i18 < 6; c7_i18++) {
    c7_y[c7_i18] = c7_dv2[c7_i18];
  }

  sf_mex_destroy(&c7_u);
}

static const mxArray *c7_e_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_u;
  const mxArray *c7_y = NULL;
  SFc7_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc7_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_u = *(int32_T *)c7_inData;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", &c7_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, FALSE);
  return c7_mxArrayOutData;
}

static int32_T c7_d_emlrt_marshallIn(SFc7_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId)
{
  int32_T c7_y;
  int32_T c7_i19;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), &c7_i19, 1, 6, 0U, 0, 0U, 0);
  c7_y = c7_i19;
  sf_mex_destroy(&c7_u);
  return c7_y;
}

static void c7_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_b_sfEvent;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  int32_T c7_y;
  SFc7_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc7_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c7_b_sfEvent = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_b_sfEvent),
    &c7_thisId);
  sf_mex_destroy(&c7_b_sfEvent);
  *(int32_T *)c7_outData = c7_y;
  sf_mex_destroy(&c7_mxArrayInData);
}

static uint8_T c7_e_emlrt_marshallIn(SFc7_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c7_b_is_active_c7_inverse_kinematics, const
  char_T *c7_identifier)
{
  uint8_T c7_y;
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c7_b_is_active_c7_inverse_kinematics), &c7_thisId);
  sf_mex_destroy(&c7_b_is_active_c7_inverse_kinematics);
  return c7_y;
}

static uint8_T c7_f_emlrt_marshallIn(SFc7_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId)
{
  uint8_T c7_y;
  uint8_T c7_u0;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), &c7_u0, 1, 3, 0U, 0, 0U, 0);
  c7_y = c7_u0;
  sf_mex_destroy(&c7_u);
  return c7_y;
}

static void init_dsm_address_info(SFc7_inverse_kinematicsInstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c7_inverse_kinematics_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(218141713U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3660020262U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3410982697U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1061310790U);
}

mxArray *sf_c7_inverse_kinematics_get_autoinheritance_info(void)
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

static const mxArray *sf_get_sim_state_info_c7_inverse_kinematics(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"ee_rates\",},{M[8],M[0],T\"is_active_c7_inverse_kinematics\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c7_inverse_kinematics_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc7_inverse_kinematicsInstanceStruct *chartInstance;
    chartInstance = (SFc7_inverse_kinematicsInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_inverse_kinematicsMachineNumber_,
           7,
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
          init_script_number_translation(_inverse_kinematicsMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (_inverse_kinematicsMachineNumber_,chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(_inverse_kinematicsMachineNumber_,
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
            1.0,0,0,(MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)
            c7_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c7_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real_T (*c7_ee_velocity)[6];
          real_T (*c7_ee_rates)[6];
          real_T (*c7_euler_angles_ee)[3];
          c7_euler_angles_ee = (real_T (*)[3])ssGetInputPortSignal
            (chartInstance->S, 1);
          c7_ee_rates = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
          c7_ee_velocity = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S,
            0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c7_ee_velocity);
          _SFD_SET_DATA_VALUE_PTR(1U, *c7_ee_rates);
          _SFD_SET_DATA_VALUE_PTR(2U, *c7_euler_angles_ee);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration
        (_inverse_kinematicsMachineNumber_,chartInstance->chartNumber,
         chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization()
{
  return "V0wR3nMqfX8byC3lHIsJBE";
}

static void sf_opaque_initialize_c7_inverse_kinematics(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc7_inverse_kinematicsInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c7_inverse_kinematics((SFc7_inverse_kinematicsInstanceStruct*)
    chartInstanceVar);
  initialize_c7_inverse_kinematics((SFc7_inverse_kinematicsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c7_inverse_kinematics(void *chartInstanceVar)
{
  enable_c7_inverse_kinematics((SFc7_inverse_kinematicsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c7_inverse_kinematics(void *chartInstanceVar)
{
  disable_c7_inverse_kinematics((SFc7_inverse_kinematicsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c7_inverse_kinematics(void *chartInstanceVar)
{
  sf_c7_inverse_kinematics((SFc7_inverse_kinematicsInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c7_inverse_kinematics(SimStruct*
  S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c7_inverse_kinematics
    ((SFc7_inverse_kinematicsInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c7_inverse_kinematics();/* state var info */
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

extern void sf_internal_set_sim_state_c7_inverse_kinematics(SimStruct* S, const
  mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c7_inverse_kinematics();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c7_inverse_kinematics((SFc7_inverse_kinematicsInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c7_inverse_kinematics(SimStruct* S)
{
  return sf_internal_get_sim_state_c7_inverse_kinematics(S);
}

static void sf_opaque_set_sim_state_c7_inverse_kinematics(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c7_inverse_kinematics(S, st);
}

static void sf_opaque_terminate_c7_inverse_kinematics(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc7_inverse_kinematicsInstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c7_inverse_kinematics((SFc7_inverse_kinematicsInstanceStruct*)
      chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_inverse_kinematics_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc7_inverse_kinematics((SFc7_inverse_kinematicsInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c7_inverse_kinematics(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c7_inverse_kinematics
      ((SFc7_inverse_kinematicsInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c7_inverse_kinematics(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_inverse_kinematics_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      7);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,7,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,7,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,7,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,7,1);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,7);
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

static void mdlRTW_c7_inverse_kinematics(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c7_inverse_kinematics(SimStruct *S)
{
  SFc7_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc7_inverse_kinematicsInstanceStruct *)malloc(sizeof
    (SFc7_inverse_kinematicsInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc7_inverse_kinematicsInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c7_inverse_kinematics;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c7_inverse_kinematics;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c7_inverse_kinematics;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c7_inverse_kinematics;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c7_inverse_kinematics;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c7_inverse_kinematics;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c7_inverse_kinematics;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c7_inverse_kinematics;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c7_inverse_kinematics;
  chartInstance->chartInfo.mdlStart = mdlStart_c7_inverse_kinematics;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c7_inverse_kinematics;
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

void c7_inverse_kinematics_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c7_inverse_kinematics(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c7_inverse_kinematics(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c7_inverse_kinematics(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c7_inverse_kinematics_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
