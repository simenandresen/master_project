/* Include files */

#include "blascompat32.h"
#include "dynamics_kinematics_sfun.h"
#include "c4_dynamics_kinematics.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "dynamics_kinematics_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c4_debug_family_names[4] = { "nargin", "nargout", "J",
  "J_inv" };

/* Function Declarations */
static void initialize_c4_dynamics_kinematics
  (SFc4_dynamics_kinematicsInstanceStruct *chartInstance);
static void initialize_params_c4_dynamics_kinematics
  (SFc4_dynamics_kinematicsInstanceStruct *chartInstance);
static void enable_c4_dynamics_kinematics(SFc4_dynamics_kinematicsInstanceStruct
  *chartInstance);
static void disable_c4_dynamics_kinematics
  (SFc4_dynamics_kinematicsInstanceStruct *chartInstance);
static void c4_update_debugger_state_c4_dynamics_kinematics
  (SFc4_dynamics_kinematicsInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c4_dynamics_kinematics
  (SFc4_dynamics_kinematicsInstanceStruct *chartInstance);
static void set_sim_state_c4_dynamics_kinematics
  (SFc4_dynamics_kinematicsInstanceStruct *chartInstance, const mxArray *c4_st);
static void finalize_c4_dynamics_kinematics
  (SFc4_dynamics_kinematicsInstanceStruct *chartInstance);
static void sf_c4_dynamics_kinematics(SFc4_dynamics_kinematicsInstanceStruct
  *chartInstance);
static void initSimStructsc4_dynamics_kinematics
  (SFc4_dynamics_kinematicsInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber);
static const mxArray *c4_sf_marshallOut(void *chartInstanceVoid, void *c4_inData);
static void c4_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_b_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static const mxArray *c4_c_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static real_T c4_emlrt_marshallIn(SFc4_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static void c4_b_emlrt_marshallIn(SFc4_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c4_mp_inv, const char_T *c4_identifier, real_T
  c4_y[72]);
static void c4_c_emlrt_marshallIn(SFc4_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[72]);
static const mxArray *c4_d_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static int32_T c4_d_emlrt_marshallIn(SFc4_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static uint8_T c4_e_emlrt_marshallIn(SFc4_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c4_b_is_active_c4_dynamics_kinematics, const
  char_T *c4_identifier);
static uint8_T c4_f_emlrt_marshallIn(SFc4_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void init_dsm_address_info(SFc4_dynamics_kinematicsInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c4_dynamics_kinematics
  (SFc4_dynamics_kinematicsInstanceStruct *chartInstance)
{
  chartInstance->c4_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c4_is_active_c4_dynamics_kinematics = 0U;
}

static void initialize_params_c4_dynamics_kinematics
  (SFc4_dynamics_kinematicsInstanceStruct *chartInstance)
{
}

static void enable_c4_dynamics_kinematics(SFc4_dynamics_kinematicsInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c4_dynamics_kinematics
  (SFc4_dynamics_kinematicsInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c4_update_debugger_state_c4_dynamics_kinematics
  (SFc4_dynamics_kinematicsInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c4_dynamics_kinematics
  (SFc4_dynamics_kinematicsInstanceStruct *chartInstance)
{
  const mxArray *c4_st;
  const mxArray *c4_y = NULL;
  int32_T c4_i0;
  real_T c4_u[72];
  const mxArray *c4_b_y = NULL;
  uint8_T c4_hoistedGlobal;
  uint8_T c4_b_u;
  const mxArray *c4_c_y = NULL;
  real_T (*c4_J_inv)[72];
  c4_J_inv = (real_T (*)[72])ssGetOutputPortSignal(chartInstance->S, 1);
  c4_st = NULL;
  c4_st = NULL;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_createcellarray(2), FALSE);
  for (c4_i0 = 0; c4_i0 < 72; c4_i0++) {
    c4_u[c4_i0] = (*c4_J_inv)[c4_i0];
  }

  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 12, 6),
                FALSE);
  sf_mex_setcell(c4_y, 0, c4_b_y);
  c4_hoistedGlobal = chartInstance->c4_is_active_c4_dynamics_kinematics;
  c4_b_u = c4_hoistedGlobal;
  c4_c_y = NULL;
  sf_mex_assign(&c4_c_y, sf_mex_create("y", &c4_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c4_y, 1, c4_c_y);
  sf_mex_assign(&c4_st, c4_y, FALSE);
  return c4_st;
}

static void set_sim_state_c4_dynamics_kinematics
  (SFc4_dynamics_kinematicsInstanceStruct *chartInstance, const mxArray *c4_st)
{
  const mxArray *c4_u;
  real_T c4_dv0[72];
  int32_T c4_i1;
  real_T (*c4_J_inv)[72];
  c4_J_inv = (real_T (*)[72])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c4_doneDoubleBufferReInit = TRUE;
  c4_u = sf_mex_dup(c4_st);
  c4_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 0)),
                        "J_inv", c4_dv0);
  for (c4_i1 = 0; c4_i1 < 72; c4_i1++) {
    (*c4_J_inv)[c4_i1] = c4_dv0[c4_i1];
  }

  chartInstance->c4_is_active_c4_dynamics_kinematics = c4_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 1)),
     "is_active_c4_dynamics_kinematics");
  sf_mex_destroy(&c4_u);
  c4_update_debugger_state_c4_dynamics_kinematics(chartInstance);
  sf_mex_destroy(&c4_st);
}

static void finalize_c4_dynamics_kinematics
  (SFc4_dynamics_kinematicsInstanceStruct *chartInstance)
{
}

static void sf_c4_dynamics_kinematics(SFc4_dynamics_kinematicsInstanceStruct
  *chartInstance)
{
  int32_T c4_i2;
  int32_T c4_i3;
  int32_T c4_i4;
  real_T c4_J[72];
  uint32_T c4_debug_family_var_map[4];
  real_T c4_nargin = 1.0;
  real_T c4_nargout = 1.0;
  real_T c4_J_inv[72];
  int32_T c4_i5;
  int32_T c4_i6;
  real_T c4_u[72];
  const mxArray *c4_y = NULL;
  real_T c4_dv1[72];
  int32_T c4_i7;
  int32_T c4_i8;
  real_T (*c4_b_J_inv)[72];
  real_T (*c4_b_J)[72];
  c4_b_J_inv = (real_T (*)[72])ssGetOutputPortSignal(chartInstance->S, 1);
  c4_b_J = (real_T (*)[72])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 3U, chartInstance->c4_sfEvent);
  for (c4_i2 = 0; c4_i2 < 72; c4_i2++) {
    _SFD_DATA_RANGE_CHECK((*c4_b_J)[c4_i2], 0U);
  }

  for (c4_i3 = 0; c4_i3 < 72; c4_i3++) {
    _SFD_DATA_RANGE_CHECK((*c4_b_J_inv)[c4_i3], 1U);
  }

  chartInstance->c4_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 3U, chartInstance->c4_sfEvent);
  for (c4_i4 = 0; c4_i4 < 72; c4_i4++) {
    c4_J[c4_i4] = (*c4_b_J)[c4_i4];
  }

  sf_debug_symbol_scope_push_eml(0U, 4U, 4U, c4_debug_family_names,
    c4_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(&c4_nargin, 0U, c4_c_sf_marshallOut,
    c4_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c4_nargout, 1U, c4_c_sf_marshallOut,
    c4_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c4_J, 2U, c4_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c4_J_inv, 3U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 3);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 5);
  for (c4_i5 = 0; c4_i5 < 72; c4_i5++) {
    c4_J_inv[c4_i5] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 6);
  for (c4_i6 = 0; c4_i6 < 72; c4_i6++) {
    c4_u[c4_i6] = c4_J[c4_i6];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 6, 12), FALSE);
  c4_b_emlrt_marshallIn(chartInstance, sf_mex_call_debug("mp_inv", 1U, 1U, 14,
    c4_y), "mp_inv", c4_dv1);
  for (c4_i7 = 0; c4_i7 < 72; c4_i7++) {
    c4_J_inv[c4_i7] = c4_dv1[c4_i7];
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, -6);
  sf_debug_symbol_scope_pop();
  for (c4_i8 = 0; c4_i8 < 72; c4_i8++) {
    (*c4_b_J_inv)[c4_i8] = c4_J_inv[c4_i8];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 3U, chartInstance->c4_sfEvent);
  sf_debug_check_for_state_inconsistency(_dynamics_kinematicsMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc4_dynamics_kinematics
  (SFc4_dynamics_kinematicsInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber)
{
}

static const mxArray *c4_sf_marshallOut(void *chartInstanceVoid, void *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i9;
  int32_T c4_i10;
  int32_T c4_i11;
  real_T c4_b_inData[72];
  int32_T c4_i12;
  int32_T c4_i13;
  int32_T c4_i14;
  real_T c4_u[72];
  const mxArray *c4_y = NULL;
  SFc4_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc4_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_i9 = 0;
  for (c4_i10 = 0; c4_i10 < 6; c4_i10++) {
    for (c4_i11 = 0; c4_i11 < 12; c4_i11++) {
      c4_b_inData[c4_i11 + c4_i9] = (*(real_T (*)[72])c4_inData)[c4_i11 + c4_i9];
    }

    c4_i9 += 12;
  }

  c4_i12 = 0;
  for (c4_i13 = 0; c4_i13 < 6; c4_i13++) {
    for (c4_i14 = 0; c4_i14 < 12; c4_i14++) {
      c4_u[c4_i14 + c4_i12] = c4_b_inData[c4_i14 + c4_i12];
    }

    c4_i12 += 12;
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 12, 6), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static void c4_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_mp_inv;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y[72];
  int32_T c4_i15;
  int32_T c4_i16;
  int32_T c4_i17;
  SFc4_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc4_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c4_mp_inv = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_mp_inv), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_mp_inv);
  c4_i15 = 0;
  for (c4_i16 = 0; c4_i16 < 6; c4_i16++) {
    for (c4_i17 = 0; c4_i17 < 12; c4_i17++) {
      (*(real_T (*)[72])c4_outData)[c4_i17 + c4_i15] = c4_y[c4_i17 + c4_i15];
    }

    c4_i15 += 12;
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_b_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i18;
  int32_T c4_i19;
  int32_T c4_i20;
  real_T c4_b_inData[72];
  int32_T c4_i21;
  int32_T c4_i22;
  int32_T c4_i23;
  real_T c4_u[72];
  const mxArray *c4_y = NULL;
  SFc4_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc4_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_i18 = 0;
  for (c4_i19 = 0; c4_i19 < 12; c4_i19++) {
    for (c4_i20 = 0; c4_i20 < 6; c4_i20++) {
      c4_b_inData[c4_i20 + c4_i18] = (*(real_T (*)[72])c4_inData)[c4_i20 +
        c4_i18];
    }

    c4_i18 += 6;
  }

  c4_i21 = 0;
  for (c4_i22 = 0; c4_i22 < 12; c4_i22++) {
    for (c4_i23 = 0; c4_i23 < 6; c4_i23++) {
      c4_u[c4_i23 + c4_i21] = c4_b_inData[c4_i23 + c4_i21];
    }

    c4_i21 += 6;
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 6, 12), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static const mxArray *c4_c_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  real_T c4_u;
  const mxArray *c4_y = NULL;
  SFc4_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc4_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(real_T *)c4_inData;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static real_T c4_emlrt_marshallIn(SFc4_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  real_T c4_y;
  real_T c4_d0;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_d0, 1, 0, 0U, 0, 0U, 0);
  c4_y = c4_d0;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void c4_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_nargout;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y;
  SFc4_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc4_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c4_nargout = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_nargout), &c4_thisId);
  sf_mex_destroy(&c4_nargout);
  *(real_T *)c4_outData = c4_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

const mxArray *sf_c4_dynamics_kinematics_get_eml_resolved_functions_info(void)
{
  const mxArray *c4_nameCaptureInfo = NULL;
  c4_nameCaptureInfo = NULL;
  sf_mex_assign(&c4_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), FALSE);
  return c4_nameCaptureInfo;
}

static void c4_b_emlrt_marshallIn(SFc4_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c4_mp_inv, const char_T *c4_identifier, real_T
  c4_y[72])
{
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_mp_inv), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_mp_inv);
}

static void c4_c_emlrt_marshallIn(SFc4_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real_T c4_y[72])
{
  real_T c4_dv2[72];
  int32_T c4_i24;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv2, 1, 0, 0U, 1, 0U, 2, 12, 6);
  for (c4_i24 = 0; c4_i24 < 72; c4_i24++) {
    c4_y[c4_i24] = c4_dv2[c4_i24];
  }

  sf_mex_destroy(&c4_u);
}

static const mxArray *c4_d_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_u;
  const mxArray *c4_y = NULL;
  SFc4_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc4_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(int32_T *)c4_inData;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static int32_T c4_d_emlrt_marshallIn(SFc4_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  int32_T c4_y;
  int32_T c4_i25;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_i25, 1, 6, 0U, 0, 0U, 0);
  c4_y = c4_i25;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void c4_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_sfEvent;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  int32_T c4_y;
  SFc4_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc4_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c4_b_sfEvent = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_sfEvent),
    &c4_thisId);
  sf_mex_destroy(&c4_b_sfEvent);
  *(int32_T *)c4_outData = c4_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

static uint8_T c4_e_emlrt_marshallIn(SFc4_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c4_b_is_active_c4_dynamics_kinematics, const
  char_T *c4_identifier)
{
  uint8_T c4_y;
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c4_b_is_active_c4_dynamics_kinematics), &c4_thisId);
  sf_mex_destroy(&c4_b_is_active_c4_dynamics_kinematics);
  return c4_y;
}

static uint8_T c4_f_emlrt_marshallIn(SFc4_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  uint8_T c4_y;
  uint8_T c4_u0;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_u0, 1, 3, 0U, 0, 0U, 0);
  c4_y = c4_u0;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void init_dsm_address_info(SFc4_dynamics_kinematicsInstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c4_dynamics_kinematics_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1892860005U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2829136283U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3396495545U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(386242389U);
}

mxArray *sf_c4_dynamics_kinematics_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("dcs7TglugL3ge3wEJMCLMF");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
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
      pr[0] = (double)(12);
      pr[1] = (double)(6);
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

static const mxArray *sf_get_sim_state_info_c4_dynamics_kinematics(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"J_inv\",},{M[8],M[0],T\"is_active_c4_dynamics_kinematics\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c4_dynamics_kinematics_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc4_dynamics_kinematicsInstanceStruct *chartInstance;
    chartInstance = (SFc4_dynamics_kinematicsInstanceStruct *) ((ChartInfoStruct
      *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_dynamics_kinematicsMachineNumber_,
           4,
           1,
           1,
           2,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"J");
          _SFD_SET_DATA_PROPS(1,2,0,1,"J_inv");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,142);
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
          dimVector[0]= 6;
          dimVector[1]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 12;
          dimVector[1]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_sf_marshallOut,(MexInFcnForType)
            c4_sf_marshallIn);
        }

        {
          real_T (*c4_J)[72];
          real_T (*c4_J_inv)[72];
          c4_J_inv = (real_T (*)[72])ssGetOutputPortSignal(chartInstance->S, 1);
          c4_J = (real_T (*)[72])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c4_J);
          _SFD_SET_DATA_VALUE_PTR(1U, *c4_J_inv);
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
  return "Gz0ZW5p2hl3cEtry7RZWCE";
}

static void sf_opaque_initialize_c4_dynamics_kinematics(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc4_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c4_dynamics_kinematics
    ((SFc4_dynamics_kinematicsInstanceStruct*) chartInstanceVar);
  initialize_c4_dynamics_kinematics((SFc4_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c4_dynamics_kinematics(void *chartInstanceVar)
{
  enable_c4_dynamics_kinematics((SFc4_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c4_dynamics_kinematics(void *chartInstanceVar)
{
  disable_c4_dynamics_kinematics((SFc4_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c4_dynamics_kinematics(void *chartInstanceVar)
{
  sf_c4_dynamics_kinematics((SFc4_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c4_dynamics_kinematics(SimStruct*
  S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c4_dynamics_kinematics
    ((SFc4_dynamics_kinematicsInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c4_dynamics_kinematics();/* state var info */
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

extern void sf_internal_set_sim_state_c4_dynamics_kinematics(SimStruct* S, const
  mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c4_dynamics_kinematics();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c4_dynamics_kinematics((SFc4_dynamics_kinematicsInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c4_dynamics_kinematics(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c4_dynamics_kinematics(S);
}

static void sf_opaque_set_sim_state_c4_dynamics_kinematics(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c4_dynamics_kinematics(S, st);
}

static void sf_opaque_terminate_c4_dynamics_kinematics(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc4_dynamics_kinematicsInstanceStruct*) chartInstanceVar
      )->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c4_dynamics_kinematics((SFc4_dynamics_kinematicsInstanceStruct*)
      chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_dynamics_kinematics_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc4_dynamics_kinematics((SFc4_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c4_dynamics_kinematics(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c4_dynamics_kinematics
      ((SFc4_dynamics_kinematicsInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c4_dynamics_kinematics(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_dynamics_kinematics_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      4);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,4,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,4,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,4,1);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,4,1);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,4);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3990823221U));
  ssSetChecksum1(S,(4189505759U));
  ssSetChecksum2(S,(3700343109U));
  ssSetChecksum3(S,(225783912U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c4_dynamics_kinematics(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c4_dynamics_kinematics(SimStruct *S)
{
  SFc4_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc4_dynamics_kinematicsInstanceStruct *)malloc(sizeof
    (SFc4_dynamics_kinematicsInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc4_dynamics_kinematicsInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c4_dynamics_kinematics;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c4_dynamics_kinematics;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c4_dynamics_kinematics;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c4_dynamics_kinematics;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c4_dynamics_kinematics;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c4_dynamics_kinematics;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c4_dynamics_kinematics;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c4_dynamics_kinematics;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c4_dynamics_kinematics;
  chartInstance->chartInfo.mdlStart = mdlStart_c4_dynamics_kinematics;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c4_dynamics_kinematics;
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

void c4_dynamics_kinematics_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c4_dynamics_kinematics(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c4_dynamics_kinematics(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c4_dynamics_kinematics(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c4_dynamics_kinematics_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
