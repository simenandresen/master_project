/* Include files */

#include "blascompat32.h"
#include "dynamics_kinematics_sfun.h"
#include "c1_dynamics_kinematics.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "dynamics_kinematics_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c1_debug_family_names[3] = { "nargin", "nargout", "I" };

/* Function Declarations */
static void initialize_c1_dynamics_kinematics
  (SFc1_dynamics_kinematicsInstanceStruct *chartInstance);
static void initialize_params_c1_dynamics_kinematics
  (SFc1_dynamics_kinematicsInstanceStruct *chartInstance);
static void enable_c1_dynamics_kinematics(SFc1_dynamics_kinematicsInstanceStruct
  *chartInstance);
static void disable_c1_dynamics_kinematics
  (SFc1_dynamics_kinematicsInstanceStruct *chartInstance);
static void c1_update_debugger_state_c1_dynamics_kinematics
  (SFc1_dynamics_kinematicsInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c1_dynamics_kinematics
  (SFc1_dynamics_kinematicsInstanceStruct *chartInstance);
static void set_sim_state_c1_dynamics_kinematics
  (SFc1_dynamics_kinematicsInstanceStruct *chartInstance, const mxArray *c1_st);
static void finalize_c1_dynamics_kinematics
  (SFc1_dynamics_kinematicsInstanceStruct *chartInstance);
static void sf_c1_dynamics_kinematics(SFc1_dynamics_kinematicsInstanceStruct
  *chartInstance);
static void initSimStructsc1_dynamics_kinematics
  (SFc1_dynamics_kinematicsInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static void c1_emlrt_marshallIn(SFc1_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c1_I, const char_T *c1_identifier, real_T c1_y
  [144]);
static void c1_b_emlrt_marshallIn(SFc1_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y[144]);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_c_emlrt_marshallIn(SFc1_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static void c1_info_helper(c1_ResolvedFunctionInfo c1_info[16]);
static void c1_eml_int_forloop_overflow_check
  (SFc1_dynamics_kinematicsInstanceStruct *chartInstance);
static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_d_emlrt_marshallIn(SFc1_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static uint8_T c1_e_emlrt_marshallIn(SFc1_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c1_b_is_active_c1_dynamics_kinematics, const
  char_T *c1_identifier);
static uint8_T c1_f_emlrt_marshallIn(SFc1_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void init_dsm_address_info(SFc1_dynamics_kinematicsInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c1_dynamics_kinematics
  (SFc1_dynamics_kinematicsInstanceStruct *chartInstance)
{
  chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c1_is_active_c1_dynamics_kinematics = 0U;
}

static void initialize_params_c1_dynamics_kinematics
  (SFc1_dynamics_kinematicsInstanceStruct *chartInstance)
{
}

static void enable_c1_dynamics_kinematics(SFc1_dynamics_kinematicsInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c1_dynamics_kinematics
  (SFc1_dynamics_kinematicsInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c1_update_debugger_state_c1_dynamics_kinematics
  (SFc1_dynamics_kinematicsInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c1_dynamics_kinematics
  (SFc1_dynamics_kinematicsInstanceStruct *chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_y = NULL;
  int32_T c1_i0;
  real_T c1_u[144];
  const mxArray *c1_b_y = NULL;
  uint8_T c1_hoistedGlobal;
  uint8_T c1_b_u;
  const mxArray *c1_c_y = NULL;
  real_T (*c1_I)[144];
  c1_I = (real_T (*)[144])ssGetOutputPortSignal(chartInstance->S, 1);
  c1_st = NULL;
  c1_st = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createcellarray(2), FALSE);
  for (c1_i0 = 0; c1_i0 < 144; c1_i0++) {
    c1_u[c1_i0] = (*c1_I)[c1_i0];
  }

  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 2, 12, 12),
                FALSE);
  sf_mex_setcell(c1_y, 0, c1_b_y);
  c1_hoistedGlobal = chartInstance->c1_is_active_c1_dynamics_kinematics;
  c1_b_u = c1_hoistedGlobal;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c1_y, 1, c1_c_y);
  sf_mex_assign(&c1_st, c1_y, FALSE);
  return c1_st;
}

static void set_sim_state_c1_dynamics_kinematics
  (SFc1_dynamics_kinematicsInstanceStruct *chartInstance, const mxArray *c1_st)
{
  const mxArray *c1_u;
  real_T c1_dv0[144];
  int32_T c1_i1;
  real_T (*c1_I)[144];
  c1_I = (real_T (*)[144])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c1_doneDoubleBufferReInit = TRUE;
  c1_u = sf_mex_dup(c1_st);
  c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 0)), "I",
                      c1_dv0);
  for (c1_i1 = 0; c1_i1 < 144; c1_i1++) {
    (*c1_I)[c1_i1] = c1_dv0[c1_i1];
  }

  chartInstance->c1_is_active_c1_dynamics_kinematics = c1_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c1_u, 1)),
     "is_active_c1_dynamics_kinematics");
  sf_mex_destroy(&c1_u);
  c1_update_debugger_state_c1_dynamics_kinematics(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void finalize_c1_dynamics_kinematics
  (SFc1_dynamics_kinematicsInstanceStruct *chartInstance)
{
}

static void sf_c1_dynamics_kinematics(SFc1_dynamics_kinematicsInstanceStruct
  *chartInstance)
{
  int32_T c1_i2;
  uint32_T c1_debug_family_var_map[3];
  real_T c1_nargin = 0.0;
  real_T c1_nargout = 1.0;
  real_T c1_I[144];
  int32_T c1_i3;
  int32_T c1_i;
  int32_T c1_b_i;
  int32_T c1_i4;
  real_T (*c1_b_I)[144];
  c1_b_I = (real_T (*)[144])ssGetOutputPortSignal(chartInstance->S, 1);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  for (c1_i2 = 0; c1_i2 < 144; c1_i2++) {
    _SFD_DATA_RANGE_CHECK((*c1_b_I)[c1_i2], 0U);
  }

  chartInstance->c1_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  sf_debug_symbol_scope_push_eml(0U, 3U, 3U, c1_debug_family_names,
    c1_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(&c1_nargin, 0U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c1_nargout, 1U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c1_I, 2U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 3);
  for (c1_i3 = 0; c1_i3 < 144; c1_i3++) {
    c1_I[c1_i3] = 0.0;
  }

  c1_eml_int_forloop_overflow_check(chartInstance);
  for (c1_i = 1; c1_i < 13; c1_i++) {
    c1_b_i = c1_i;
    c1_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c1_b_i), 1, 12, 1, 0) + 12 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c1_b_i), 1, 12, 2, 0) - 1))
      - 1] = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -3);
  sf_debug_symbol_scope_pop();
  for (c1_i4 = 0; c1_i4 < 144; c1_i4++) {
    (*c1_b_I)[c1_i4] = c1_I[c1_i4];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  sf_debug_check_for_state_inconsistency(_dynamics_kinematicsMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc1_dynamics_kinematics
  (SFc1_dynamics_kinematicsInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber)
{
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i5;
  int32_T c1_i6;
  int32_T c1_i7;
  real_T c1_b_inData[144];
  int32_T c1_i8;
  int32_T c1_i9;
  int32_T c1_i10;
  real_T c1_u[144];
  const mxArray *c1_y = NULL;
  SFc1_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc1_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_i5 = 0;
  for (c1_i6 = 0; c1_i6 < 12; c1_i6++) {
    for (c1_i7 = 0; c1_i7 < 12; c1_i7++) {
      c1_b_inData[c1_i7 + c1_i5] = (*(real_T (*)[144])c1_inData)[c1_i7 + c1_i5];
    }

    c1_i5 += 12;
  }

  c1_i8 = 0;
  for (c1_i9 = 0; c1_i9 < 12; c1_i9++) {
    for (c1_i10 = 0; c1_i10 < 12; c1_i10++) {
      c1_u[c1_i10 + c1_i8] = c1_b_inData[c1_i10 + c1_i8];
    }

    c1_i8 += 12;
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 2, 12, 12), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static void c1_emlrt_marshallIn(SFc1_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c1_I, const char_T *c1_identifier, real_T c1_y
  [144])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_I), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_I);
}

static void c1_b_emlrt_marshallIn(SFc1_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId,
  real_T c1_y[144])
{
  real_T c1_dv1[144];
  int32_T c1_i11;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv1, 1, 0, 0U, 1, 0U, 2, 12,
                12);
  for (c1_i11 = 0; c1_i11 < 144; c1_i11++) {
    c1_y[c1_i11] = c1_dv1[c1_i11];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_I;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y[144];
  int32_T c1_i12;
  int32_T c1_i13;
  int32_T c1_i14;
  SFc1_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc1_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c1_I = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_I), &c1_thisId, c1_y);
  sf_mex_destroy(&c1_I);
  c1_i12 = 0;
  for (c1_i13 = 0; c1_i13 < 12; c1_i13++) {
    for (c1_i14 = 0; c1_i14 < 12; c1_i14++) {
      (*(real_T (*)[144])c1_outData)[c1_i14 + c1_i12] = c1_y[c1_i14 + c1_i12];
    }

    c1_i12 += 12;
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc1_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static real_T c1_c_emlrt_marshallIn(SFc1_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
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
  SFc1_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc1_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c1_nargout = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_nargout), &c1_thisId);
  sf_mex_destroy(&c1_nargout);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

const mxArray *sf_c1_dynamics_kinematics_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo;
  c1_ResolvedFunctionInfo c1_info[16];
  const mxArray *c1_m0 = NULL;
  int32_T c1_i15;
  c1_ResolvedFunctionInfo *c1_r0;
  c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  c1_info_helper(c1_info);
  sf_mex_assign(&c1_m0, sf_mex_createstruct("nameCaptureInfo", 1, 16), FALSE);
  for (c1_i15 = 0; c1_i15 < 16; c1_i15++) {
    c1_r0 = &c1_info[c1_i15];
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c1_r0->context)), "context", "nameCaptureInfo",
                    c1_i15);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c1_r0->name)), "name", "nameCaptureInfo", c1_i15);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c1_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c1_i15);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", c1_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c1_r0->resolved)), "resolved", "nameCaptureInfo",
                    c1_i15);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c1_i15);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c1_i15);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c1_i15);
    sf_mex_addfield(c1_m0, sf_mex_create("nameCaptureInfo", &c1_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c1_i15);
  }

  sf_mex_assign(&c1_nameCaptureInfo, c1_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c1_nameCaptureInfo);
  return c1_nameCaptureInfo;
}

static void c1_info_helper(c1_ResolvedFunctionInfo c1_info[16])
{
  c1_info[0].context = "";
  c1_info[0].name = "eye";
  c1_info[0].dominantType = "double";
  c1_info[0].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m";
  c1_info[0].fileTimeLo = 1286818688U;
  c1_info[0].fileTimeHi = 0U;
  c1_info[0].mFileTimeLo = 0U;
  c1_info[0].mFileTimeHi = 0U;
  c1_info[1].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c1_info[1].name = "eml_assert_valid_size_arg";
  c1_info[1].dominantType = "double";
  c1_info[1].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c1_info[1].fileTimeLo = 1286818694U;
  c1_info[1].fileTimeHi = 0U;
  c1_info[1].mFileTimeLo = 0U;
  c1_info[1].mFileTimeHi = 0U;
  c1_info[2].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral";
  c1_info[2].name = "isinf";
  c1_info[2].dominantType = "double";
  c1_info[2].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isinf.m";
  c1_info[2].fileTimeLo = 1286818760U;
  c1_info[2].fileTimeHi = 0U;
  c1_info[2].mFileTimeLo = 0U;
  c1_info[2].mFileTimeHi = 0U;
  c1_info[3].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!numel_for_size";
  c1_info[3].name = "mtimes";
  c1_info[3].dominantType = "double";
  c1_info[3].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c1_info[3].fileTimeLo = 1289519692U;
  c1_info[3].fileTimeHi = 0U;
  c1_info[3].mFileTimeLo = 0U;
  c1_info[3].mFileTimeHi = 0U;
  c1_info[4].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c1_info[4].name = "eml_index_class";
  c1_info[4].dominantType = "";
  c1_info[4].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[4].fileTimeLo = 1286818778U;
  c1_info[4].fileTimeHi = 0U;
  c1_info[4].mFileTimeLo = 0U;
  c1_info[4].mFileTimeHi = 0U;
  c1_info[5].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c1_info[5].name = "intmax";
  c1_info[5].dominantType = "char";
  c1_info[5].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/intmax.m";
  c1_info[5].fileTimeLo = 1311255316U;
  c1_info[5].fileTimeHi = 0U;
  c1_info[5].mFileTimeLo = 0U;
  c1_info[5].mFileTimeHi = 0U;
  c1_info[6].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c1_info[6].name = "eml_is_float_class";
  c1_info[6].dominantType = "char";
  c1_info[6].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c1_info[6].fileTimeLo = 1286818782U;
  c1_info[6].fileTimeHi = 0U;
  c1_info[6].mFileTimeLo = 0U;
  c1_info[6].mFileTimeHi = 0U;
  c1_info[7].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c1_info[7].name = "min";
  c1_info[7].dominantType = "double";
  c1_info[7].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/min.m";
  c1_info[7].fileTimeLo = 1311255318U;
  c1_info[7].fileTimeHi = 0U;
  c1_info[7].mFileTimeLo = 0U;
  c1_info[7].mFileTimeHi = 0U;
  c1_info[8].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/min.m";
  c1_info[8].name = "eml_min_or_max";
  c1_info[8].dominantType = "char";
  c1_info[8].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c1_info[8].fileTimeLo = 1303146212U;
  c1_info[8].fileTimeHi = 0U;
  c1_info[8].mFileTimeLo = 0U;
  c1_info[8].mFileTimeHi = 0U;
  c1_info[9].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c1_info[9].name = "eml_scalar_eg";
  c1_info[9].dominantType = "double";
  c1_info[9].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[9].fileTimeLo = 1286818796U;
  c1_info[9].fileTimeHi = 0U;
  c1_info[9].mFileTimeLo = 0U;
  c1_info[9].mFileTimeHi = 0U;
  c1_info[10].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c1_info[10].name = "eml_scalexp_alloc";
  c1_info[10].dominantType = "double";
  c1_info[10].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c1_info[10].fileTimeLo = 1286818796U;
  c1_info[10].fileTimeHi = 0U;
  c1_info[10].mFileTimeLo = 0U;
  c1_info[10].mFileTimeHi = 0U;
  c1_info[11].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c1_info[11].name = "eml_index_class";
  c1_info[11].dominantType = "";
  c1_info[11].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[11].fileTimeLo = 1286818778U;
  c1_info[11].fileTimeHi = 0U;
  c1_info[11].mFileTimeLo = 0U;
  c1_info[11].mFileTimeHi = 0U;
  c1_info[12].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c1_info[12].name = "eml_scalar_eg";
  c1_info[12].dominantType = "double";
  c1_info[12].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c1_info[12].fileTimeLo = 1286818796U;
  c1_info[12].fileTimeHi = 0U;
  c1_info[12].mFileTimeLo = 0U;
  c1_info[12].mFileTimeHi = 0U;
  c1_info[13].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c1_info[13].name = "eml_index_class";
  c1_info[13].dominantType = "";
  c1_info[13].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c1_info[13].fileTimeLo = 1286818778U;
  c1_info[13].fileTimeHi = 0U;
  c1_info[13].mFileTimeLo = 0U;
  c1_info[13].mFileTimeHi = 0U;
  c1_info[14].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c1_info[14].name = "eml_int_forloop_overflow_check";
  c1_info[14].dominantType = "";
  c1_info[14].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c1_info[14].fileTimeLo = 1311255316U;
  c1_info[14].fileTimeHi = 0U;
  c1_info[14].mFileTimeLo = 0U;
  c1_info[14].mFileTimeHi = 0U;
  c1_info[15].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c1_info[15].name = "intmax";
  c1_info[15].dominantType = "char";
  c1_info[15].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/intmax.m";
  c1_info[15].fileTimeLo = 1311255316U;
  c1_info[15].fileTimeHi = 0U;
  c1_info[15].mFileTimeLo = 0U;
  c1_info[15].mFileTimeHi = 0U;
}

static void c1_eml_int_forloop_overflow_check
  (SFc1_dynamics_kinematicsInstanceStruct *chartInstance)
{
}

static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc1_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(int32_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, FALSE);
  return c1_mxArrayOutData;
}

static int32_T c1_d_emlrt_marshallIn(SFc1_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_y;
  int32_T c1_i16;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i16, 1, 6, 0U, 0, 0U, 0);
  c1_y = c1_i16;
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
  SFc1_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc1_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
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

static uint8_T c1_e_emlrt_marshallIn(SFc1_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c1_b_is_active_c1_dynamics_kinematics, const
  char_T *c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_y = c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_b_is_active_c1_dynamics_kinematics), &c1_thisId);
  sf_mex_destroy(&c1_b_is_active_c1_dynamics_kinematics);
  return c1_y;
}

static uint8_T c1_f_emlrt_marshallIn(SFc1_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u0;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u0, 1, 3, 0U, 0, 0U, 0);
  c1_y = c1_u0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void init_dsm_address_info(SFc1_dynamics_kinematicsInstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c1_dynamics_kinematics_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1686148029U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3329909720U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3399718523U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(4243308297U);
}

mxArray *sf_c1_dynamics_kinematics_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("s3hwsiQ8ZNu21O7VHsFrlD");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxCreateDoubleMatrix(0,0,mxREAL));
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
      pr[0] = (double)(12);
      pr[1] = (double)(12);
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

static const mxArray *sf_get_sim_state_info_c1_dynamics_kinematics(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"I\",},{M[8],M[0],T\"is_active_c1_dynamics_kinematics\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_dynamics_kinematics_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_dynamics_kinematicsInstanceStruct *chartInstance;
    chartInstance = (SFc1_dynamics_kinematicsInstanceStruct *) ((ChartInfoStruct
      *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_dynamics_kinematicsMachineNumber_,
           1,
           1,
           1,
           1,
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
          init_script_number_translation(_dynamics_kinematicsMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (_dynamics_kinematicsMachineNumber_,chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(_dynamics_kinematicsMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,2,0,1,"I");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,60);
        _SFD_TRANS_COV_WTS(0,0,0,1,0);
        if (chartAlreadyPresent==0) {
          _SFD_TRANS_COV_MAPS(0,
                              0,NULL,NULL,
                              0,NULL,NULL,
                              1,NULL,NULL,
                              0,NULL,NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 12;
          dimVector[1]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)
            c1_sf_marshallIn);
        }

        {
          real_T (*c1_I)[144];
          c1_I = (real_T (*)[144])ssGetOutputPortSignal(chartInstance->S, 1);
          _SFD_SET_DATA_VALUE_PTR(0U, *c1_I);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration
        (_dynamics_kinematicsMachineNumber_,chartInstance->chartNumber,
         chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization()
{
  return "KgC6BeanPvWEr9foRu7QUF";
}

static void sf_opaque_initialize_c1_dynamics_kinematics(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c1_dynamics_kinematics
    ((SFc1_dynamics_kinematicsInstanceStruct*) chartInstanceVar);
  initialize_c1_dynamics_kinematics((SFc1_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c1_dynamics_kinematics(void *chartInstanceVar)
{
  enable_c1_dynamics_kinematics((SFc1_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c1_dynamics_kinematics(void *chartInstanceVar)
{
  disable_c1_dynamics_kinematics((SFc1_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c1_dynamics_kinematics(void *chartInstanceVar)
{
  sf_c1_dynamics_kinematics((SFc1_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c1_dynamics_kinematics(SimStruct*
  S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c1_dynamics_kinematics
    ((SFc1_dynamics_kinematicsInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c1_dynamics_kinematics();/* state var info */
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

extern void sf_internal_set_sim_state_c1_dynamics_kinematics(SimStruct* S, const
  mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c1_dynamics_kinematics();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c1_dynamics_kinematics((SFc1_dynamics_kinematicsInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c1_dynamics_kinematics(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c1_dynamics_kinematics(S);
}

static void sf_opaque_set_sim_state_c1_dynamics_kinematics(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c1_dynamics_kinematics(S, st);
}

static void sf_opaque_terminate_c1_dynamics_kinematics(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_dynamics_kinematicsInstanceStruct*) chartInstanceVar
      )->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c1_dynamics_kinematics((SFc1_dynamics_kinematicsInstanceStruct*)
      chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_dynamics_kinematics_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_dynamics_kinematics((SFc1_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_dynamics_kinematics(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c1_dynamics_kinematics
      ((SFc1_dynamics_kinematicsInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c1_dynamics_kinematics(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_dynamics_kinematics_optimization_info();
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
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,1);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3038804422U));
  ssSetChecksum1(S,(1280235970U));
  ssSetChecksum2(S,(3329604171U));
  ssSetChecksum3(S,(4105755726U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c1_dynamics_kinematics(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c1_dynamics_kinematics(SimStruct *S)
{
  SFc1_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc1_dynamics_kinematicsInstanceStruct *)malloc(sizeof
    (SFc1_dynamics_kinematicsInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc1_dynamics_kinematicsInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c1_dynamics_kinematics;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c1_dynamics_kinematics;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c1_dynamics_kinematics;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c1_dynamics_kinematics;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c1_dynamics_kinematics;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c1_dynamics_kinematics;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c1_dynamics_kinematics;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c1_dynamics_kinematics;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_dynamics_kinematics;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_dynamics_kinematics;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c1_dynamics_kinematics;
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

void c1_dynamics_kinematics_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_dynamics_kinematics(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_dynamics_kinematics(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_dynamics_kinematics(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_dynamics_kinematics_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
