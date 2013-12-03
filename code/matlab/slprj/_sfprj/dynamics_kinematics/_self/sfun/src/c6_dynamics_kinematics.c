/* Include files */

#include "blascompat32.h"
#include "dynamics_kinematics_sfun.h"
#include "c6_dynamics_kinematics.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "dynamics_kinematics_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c6_debug_family_names[8] = { "J_inv", "nargin", "nargout",
  "J", "vee", "q", "eta2", "zeta" };

/* Function Declarations */
static void initialize_c6_dynamics_kinematics
  (SFc6_dynamics_kinematicsInstanceStruct *chartInstance);
static void initialize_params_c6_dynamics_kinematics
  (SFc6_dynamics_kinematicsInstanceStruct *chartInstance);
static void enable_c6_dynamics_kinematics(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance);
static void disable_c6_dynamics_kinematics
  (SFc6_dynamics_kinematicsInstanceStruct *chartInstance);
static void c6_update_debugger_state_c6_dynamics_kinematics
  (SFc6_dynamics_kinematicsInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c6_dynamics_kinematics
  (SFc6_dynamics_kinematicsInstanceStruct *chartInstance);
static void set_sim_state_c6_dynamics_kinematics
  (SFc6_dynamics_kinematicsInstanceStruct *chartInstance, const mxArray *c6_st);
static void finalize_c6_dynamics_kinematics
  (SFc6_dynamics_kinematicsInstanceStruct *chartInstance);
static void sf_c6_dynamics_kinematics(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance);
static void c6_chartstep_c6_dynamics_kinematics
  (SFc6_dynamics_kinematicsInstanceStruct *chartInstance);
static void initSimStructsc6_dynamics_kinematics
  (SFc6_dynamics_kinematicsInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber);
static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData);
static void c6_emlrt_marshallIn(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_zeta, const char_T *c6_identifier, real_T
  c6_y[12]);
static void c6_b_emlrt_marshallIn(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[12]);
static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static const mxArray *c6_c_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static const mxArray *c6_e_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static real_T c6_c_emlrt_marshallIn(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_f_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static void c6_eml_scalar_eg(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance);
static void c6_d_emlrt_marshallIn(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_wln_m, const char_T *c6_identifier, real_T
  c6_y[72]);
static void c6_e_emlrt_marshallIn(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[72]);
static const mxArray *c6_g_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static int32_T c6_f_emlrt_marshallIn(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static uint8_T c6_g_emlrt_marshallIn(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_b_is_active_c6_dynamics_kinematics, const
  char_T *c6_identifier);
static uint8_T c6_h_emlrt_marshallIn(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void init_dsm_address_info(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c6_dynamics_kinematics
  (SFc6_dynamics_kinematicsInstanceStruct *chartInstance)
{
  chartInstance->c6_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c6_is_active_c6_dynamics_kinematics = 0U;
}

static void initialize_params_c6_dynamics_kinematics
  (SFc6_dynamics_kinematicsInstanceStruct *chartInstance)
{
}

static void enable_c6_dynamics_kinematics(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c6_dynamics_kinematics
  (SFc6_dynamics_kinematicsInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c6_update_debugger_state_c6_dynamics_kinematics
  (SFc6_dynamics_kinematicsInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c6_dynamics_kinematics
  (SFc6_dynamics_kinematicsInstanceStruct *chartInstance)
{
  const mxArray *c6_st;
  const mxArray *c6_y = NULL;
  int32_T c6_i0;
  real_T c6_u[12];
  const mxArray *c6_b_y = NULL;
  uint8_T c6_hoistedGlobal;
  uint8_T c6_b_u;
  const mxArray *c6_c_y = NULL;
  real_T (*c6_zeta)[12];
  c6_zeta = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 1);
  c6_st = NULL;
  c6_st = NULL;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_createcellarray(2), FALSE);
  for (c6_i0 = 0; c6_i0 < 12; c6_i0++) {
    c6_u[c6_i0] = (*c6_zeta)[c6_i0];
  }

  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 12), FALSE);
  sf_mex_setcell(c6_y, 0, c6_b_y);
  c6_hoistedGlobal = chartInstance->c6_is_active_c6_dynamics_kinematics;
  c6_b_u = c6_hoistedGlobal;
  c6_c_y = NULL;
  sf_mex_assign(&c6_c_y, sf_mex_create("y", &c6_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c6_y, 1, c6_c_y);
  sf_mex_assign(&c6_st, c6_y, FALSE);
  return c6_st;
}

static void set_sim_state_c6_dynamics_kinematics
  (SFc6_dynamics_kinematicsInstanceStruct *chartInstance, const mxArray *c6_st)
{
  const mxArray *c6_u;
  real_T c6_dv0[12];
  int32_T c6_i1;
  real_T (*c6_zeta)[12];
  c6_zeta = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c6_doneDoubleBufferReInit = TRUE;
  c6_u = sf_mex_dup(c6_st);
  c6_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 0)), "zeta",
                      c6_dv0);
  for (c6_i1 = 0; c6_i1 < 12; c6_i1++) {
    (*c6_zeta)[c6_i1] = c6_dv0[c6_i1];
  }

  chartInstance->c6_is_active_c6_dynamics_kinematics = c6_g_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 1)),
     "is_active_c6_dynamics_kinematics");
  sf_mex_destroy(&c6_u);
  c6_update_debugger_state_c6_dynamics_kinematics(chartInstance);
  sf_mex_destroy(&c6_st);
}

static void finalize_c6_dynamics_kinematics
  (SFc6_dynamics_kinematicsInstanceStruct *chartInstance)
{
}

static void sf_c6_dynamics_kinematics(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance)
{
  int32_T c6_i2;
  int32_T c6_i3;
  int32_T c6_i4;
  int32_T c6_i5;
  int32_T c6_i6;
  real_T (*c6_eta2)[3];
  real_T (*c6_q)[6];
  real_T (*c6_vee)[6];
  real_T (*c6_zeta)[12];
  real_T (*c6_J)[72];
  c6_eta2 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 3);
  c6_q = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 2);
  c6_vee = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 1);
  c6_zeta = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 1);
  c6_J = (real_T (*)[72])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 5U, chartInstance->c6_sfEvent);
  for (c6_i2 = 0; c6_i2 < 72; c6_i2++) {
    _SFD_DATA_RANGE_CHECK((*c6_J)[c6_i2], 0U);
  }

  for (c6_i3 = 0; c6_i3 < 12; c6_i3++) {
    _SFD_DATA_RANGE_CHECK((*c6_zeta)[c6_i3], 1U);
  }

  for (c6_i4 = 0; c6_i4 < 6; c6_i4++) {
    _SFD_DATA_RANGE_CHECK((*c6_vee)[c6_i4], 2U);
  }

  for (c6_i5 = 0; c6_i5 < 6; c6_i5++) {
    _SFD_DATA_RANGE_CHECK((*c6_q)[c6_i5], 3U);
  }

  for (c6_i6 = 0; c6_i6 < 3; c6_i6++) {
    _SFD_DATA_RANGE_CHECK((*c6_eta2)[c6_i6], 4U);
  }

  chartInstance->c6_sfEvent = CALL_EVENT;
  c6_chartstep_c6_dynamics_kinematics(chartInstance);
  sf_debug_check_for_state_inconsistency(_dynamics_kinematicsMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c6_chartstep_c6_dynamics_kinematics
  (SFc6_dynamics_kinematicsInstanceStruct *chartInstance)
{
  int32_T c6_i7;
  real_T c6_J[72];
  int32_T c6_i8;
  real_T c6_vee[6];
  int32_T c6_i9;
  real_T c6_q[6];
  int32_T c6_i10;
  real_T c6_eta2[3];
  uint32_T c6_debug_family_var_map[8];
  real_T c6_J_inv[72];
  real_T c6_nargin = 4.0;
  real_T c6_nargout = 1.0;
  real_T c6_zeta[12];
  int32_T c6_i11;
  int32_T c6_i12;
  real_T c6_u[72];
  const mxArray *c6_y = NULL;
  int32_T c6_i13;
  real_T c6_b_u[6];
  const mxArray *c6_b_y = NULL;
  int32_T c6_i14;
  real_T c6_c_u[3];
  const mxArray *c6_c_y = NULL;
  real_T c6_dv1[72];
  int32_T c6_i15;
  int32_T c6_i16;
  real_T c6_a[72];
  int32_T c6_i17;
  real_T c6_b[6];
  int32_T c6_i18;
  int32_T c6_i19;
  int32_T c6_i20;
  real_T c6_C[12];
  int32_T c6_i21;
  int32_T c6_i22;
  int32_T c6_i23;
  int32_T c6_i24;
  int32_T c6_i25;
  int32_T c6_i26;
  int32_T c6_i27;
  real_T (*c6_b_zeta)[12];
  real_T (*c6_b_eta2)[3];
  real_T (*c6_b_q)[6];
  real_T (*c6_b_vee)[6];
  real_T (*c6_b_J)[72];
  c6_b_eta2 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 3);
  c6_b_q = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 2);
  c6_b_vee = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 1);
  c6_b_zeta = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 1);
  c6_b_J = (real_T (*)[72])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 5U, chartInstance->c6_sfEvent);
  for (c6_i7 = 0; c6_i7 < 72; c6_i7++) {
    c6_J[c6_i7] = (*c6_b_J)[c6_i7];
  }

  for (c6_i8 = 0; c6_i8 < 6; c6_i8++) {
    c6_vee[c6_i8] = (*c6_b_vee)[c6_i8];
  }

  for (c6_i9 = 0; c6_i9 < 6; c6_i9++) {
    c6_q[c6_i9] = (*c6_b_q)[c6_i9];
  }

  for (c6_i10 = 0; c6_i10 < 3; c6_i10++) {
    c6_eta2[c6_i10] = (*c6_b_eta2)[c6_i10];
  }

  sf_debug_symbol_scope_push_eml(0U, 8U, 8U, c6_debug_family_names,
    c6_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c6_J_inv, 0U, c6_f_sf_marshallOut,
    c6_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_nargin, 1U, c6_e_sf_marshallOut,
    c6_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_nargout, 2U, c6_e_sf_marshallOut,
    c6_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c6_J, 3U, c6_d_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c6_vee, 4U, c6_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c6_q, 5U, c6_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c6_eta2, 6U, c6_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c6_zeta, 7U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 3);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 4);
  for (c6_i11 = 0; c6_i11 < 72; c6_i11++) {
    c6_J_inv[c6_i11] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 5);
  for (c6_i12 = 0; c6_i12 < 72; c6_i12++) {
    c6_u[c6_i12] = c6_J[c6_i12];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 6, 12), FALSE);
  for (c6_i13 = 0; c6_i13 < 6; c6_i13++) {
    c6_b_u[c6_i13] = c6_q[c6_i13];
  }

  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", c6_b_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  for (c6_i14 = 0; c6_i14 < 3; c6_i14++) {
    c6_c_u[c6_i14] = c6_eta2[c6_i14];
  }

  c6_c_y = NULL;
  sf_mex_assign(&c6_c_y, sf_mex_create("y", c6_c_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  c6_d_emlrt_marshallIn(chartInstance, sf_mex_call_debug("wln_m", 1U, 3U, 14,
    c6_y, 14, c6_b_y, 14, c6_c_y), "wln_m", c6_dv1);
  for (c6_i15 = 0; c6_i15 < 72; c6_i15++) {
    c6_J_inv[c6_i15] = c6_dv1[c6_i15];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 7);
  for (c6_i16 = 0; c6_i16 < 72; c6_i16++) {
    c6_a[c6_i16] = c6_J_inv[c6_i16];
  }

  for (c6_i17 = 0; c6_i17 < 6; c6_i17++) {
    c6_b[c6_i17] = c6_vee[c6_i17];
  }

  c6_eml_scalar_eg(chartInstance);
  c6_eml_scalar_eg(chartInstance);
  for (c6_i18 = 0; c6_i18 < 12; c6_i18++) {
    c6_zeta[c6_i18] = 0.0;
  }

  for (c6_i19 = 0; c6_i19 < 12; c6_i19++) {
    c6_zeta[c6_i19] = 0.0;
  }

  for (c6_i20 = 0; c6_i20 < 12; c6_i20++) {
    c6_C[c6_i20] = c6_zeta[c6_i20];
  }

  for (c6_i21 = 0; c6_i21 < 12; c6_i21++) {
    c6_zeta[c6_i21] = c6_C[c6_i21];
  }

  for (c6_i22 = 0; c6_i22 < 12; c6_i22++) {
    c6_C[c6_i22] = c6_zeta[c6_i22];
  }

  for (c6_i23 = 0; c6_i23 < 12; c6_i23++) {
    c6_zeta[c6_i23] = c6_C[c6_i23];
  }

  for (c6_i24 = 0; c6_i24 < 12; c6_i24++) {
    c6_zeta[c6_i24] = 0.0;
    c6_i25 = 0;
    for (c6_i26 = 0; c6_i26 < 6; c6_i26++) {
      c6_zeta[c6_i24] += c6_a[c6_i25 + c6_i24] * c6_b[c6_i26];
      c6_i25 += 12;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, -7);
  sf_debug_symbol_scope_pop();
  for (c6_i27 = 0; c6_i27 < 12; c6_i27++) {
    (*c6_b_zeta)[c6_i27] = c6_zeta[c6_i27];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 5U, chartInstance->c6_sfEvent);
}

static void initSimStructsc6_dynamics_kinematics
  (SFc6_dynamics_kinematicsInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber)
{
}

static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i28;
  real_T c6_b_inData[12];
  int32_T c6_i29;
  real_T c6_u[12];
  const mxArray *c6_y = NULL;
  SFc6_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i28 = 0; c6_i28 < 12; c6_i28++) {
    c6_b_inData[c6_i28] = (*(real_T (*)[12])c6_inData)[c6_i28];
  }

  for (c6_i29 = 0; c6_i29 < 12; c6_i29++) {
    c6_u[c6_i29] = c6_b_inData[c6_i29];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 12), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_emlrt_marshallIn(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_zeta, const char_T *c6_identifier, real_T
  c6_y[12])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_zeta), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_zeta);
}

static void c6_b_emlrt_marshallIn(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[12])
{
  real_T c6_dv2[12];
  int32_T c6_i30;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv2, 1, 0, 0U, 1, 0U, 1, 12);
  for (c6_i30 = 0; c6_i30 < 12; c6_i30++) {
    c6_y[c6_i30] = c6_dv2[c6_i30];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_zeta;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[12];
  int32_T c6_i31;
  SFc6_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_zeta = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_zeta), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_zeta);
  for (c6_i31 = 0; c6_i31 < 12; c6_i31++) {
    (*(real_T (*)[12])c6_outData)[c6_i31] = c6_y[c6_i31];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i32;
  real_T c6_b_inData[3];
  int32_T c6_i33;
  real_T c6_u[3];
  const mxArray *c6_y = NULL;
  SFc6_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i32 = 0; c6_i32 < 3; c6_i32++) {
    c6_b_inData[c6_i32] = (*(real_T (*)[3])c6_inData)[c6_i32];
  }

  for (c6_i33 = 0; c6_i33 < 3; c6_i33++) {
    c6_u[c6_i33] = c6_b_inData[c6_i33];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static const mxArray *c6_c_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i34;
  real_T c6_b_inData[6];
  int32_T c6_i35;
  real_T c6_u[6];
  const mxArray *c6_y = NULL;
  SFc6_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i34 = 0; c6_i34 < 6; c6_i34++) {
    c6_b_inData[c6_i34] = (*(real_T (*)[6])c6_inData)[c6_i34];
  }

  for (c6_i35 = 0; c6_i35 < 6; c6_i35++) {
    c6_u[c6_i35] = c6_b_inData[c6_i35];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i36;
  int32_T c6_i37;
  int32_T c6_i38;
  real_T c6_b_inData[72];
  int32_T c6_i39;
  int32_T c6_i40;
  int32_T c6_i41;
  real_T c6_u[72];
  const mxArray *c6_y = NULL;
  SFc6_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i36 = 0;
  for (c6_i37 = 0; c6_i37 < 12; c6_i37++) {
    for (c6_i38 = 0; c6_i38 < 6; c6_i38++) {
      c6_b_inData[c6_i38 + c6_i36] = (*(real_T (*)[72])c6_inData)[c6_i38 +
        c6_i36];
    }

    c6_i36 += 6;
  }

  c6_i39 = 0;
  for (c6_i40 = 0; c6_i40 < 12; c6_i40++) {
    for (c6_i41 = 0; c6_i41 < 6; c6_i41++) {
      c6_u[c6_i41 + c6_i39] = c6_b_inData[c6_i41 + c6_i39];
    }

    c6_i39 += 6;
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 6, 12), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static const mxArray *c6_e_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  real_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(real_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static real_T c6_c_emlrt_marshallIn(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  real_T c6_y;
  real_T c6_d0;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_d0, 1, 0, 0U, 0, 0U, 0);
  c6_y = c6_d0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_nargout;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y;
  SFc6_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_nargout = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_nargout), &c6_thisId);
  sf_mex_destroy(&c6_nargout);
  *(real_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_f_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i42;
  int32_T c6_i43;
  int32_T c6_i44;
  real_T c6_b_inData[72];
  int32_T c6_i45;
  int32_T c6_i46;
  int32_T c6_i47;
  real_T c6_u[72];
  const mxArray *c6_y = NULL;
  SFc6_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i42 = 0;
  for (c6_i43 = 0; c6_i43 < 6; c6_i43++) {
    for (c6_i44 = 0; c6_i44 < 12; c6_i44++) {
      c6_b_inData[c6_i44 + c6_i42] = (*(real_T (*)[72])c6_inData)[c6_i44 +
        c6_i42];
    }

    c6_i42 += 12;
  }

  c6_i45 = 0;
  for (c6_i46 = 0; c6_i46 < 6; c6_i46++) {
    for (c6_i47 = 0; c6_i47 < 12; c6_i47++) {
      c6_u[c6_i47 + c6_i45] = c6_b_inData[c6_i47 + c6_i45];
    }

    c6_i45 += 12;
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 12, 6), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_wln_m;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[72];
  int32_T c6_i48;
  int32_T c6_i49;
  int32_T c6_i50;
  SFc6_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_wln_m = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_wln_m), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_wln_m);
  c6_i48 = 0;
  for (c6_i49 = 0; c6_i49 < 6; c6_i49++) {
    for (c6_i50 = 0; c6_i50 < 12; c6_i50++) {
      (*(real_T (*)[72])c6_outData)[c6_i50 + c6_i48] = c6_y[c6_i50 + c6_i48];
    }

    c6_i48 += 12;
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

const mxArray *sf_c6_dynamics_kinematics_get_eml_resolved_functions_info(void)
{
  const mxArray *c6_nameCaptureInfo;
  c6_ResolvedFunctionInfo c6_info[8];
  c6_ResolvedFunctionInfo (*c6_b_info)[8];
  const mxArray *c6_m0 = NULL;
  int32_T c6_i51;
  c6_ResolvedFunctionInfo *c6_r0;
  c6_nameCaptureInfo = NULL;
  c6_nameCaptureInfo = NULL;
  c6_b_info = (c6_ResolvedFunctionInfo (*)[8])c6_info;
  (*c6_b_info)[0].context = "";
  (*c6_b_info)[0].name = "mtimes";
  (*c6_b_info)[0].dominantType = "double";
  (*c6_b_info)[0].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  (*c6_b_info)[0].fileTimeLo = 1289519692U;
  (*c6_b_info)[0].fileTimeHi = 0U;
  (*c6_b_info)[0].mFileTimeLo = 0U;
  (*c6_b_info)[0].mFileTimeHi = 0U;
  (*c6_b_info)[1].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  (*c6_b_info)[1].name = "eml_index_class";
  (*c6_b_info)[1].dominantType = "";
  (*c6_b_info)[1].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  (*c6_b_info)[1].fileTimeLo = 1286818778U;
  (*c6_b_info)[1].fileTimeHi = 0U;
  (*c6_b_info)[1].mFileTimeLo = 0U;
  (*c6_b_info)[1].mFileTimeHi = 0U;
  (*c6_b_info)[2].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  (*c6_b_info)[2].name = "eml_scalar_eg";
  (*c6_b_info)[2].dominantType = "double";
  (*c6_b_info)[2].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  (*c6_b_info)[2].fileTimeLo = 1286818796U;
  (*c6_b_info)[2].fileTimeHi = 0U;
  (*c6_b_info)[2].mFileTimeLo = 0U;
  (*c6_b_info)[2].mFileTimeHi = 0U;
  (*c6_b_info)[3].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  (*c6_b_info)[3].name = "eml_xgemm";
  (*c6_b_info)[3].dominantType = "int32";
  (*c6_b_info)[3].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  (*c6_b_info)[3].fileTimeLo = 1299076772U;
  (*c6_b_info)[3].fileTimeHi = 0U;
  (*c6_b_info)[3].mFileTimeLo = 0U;
  (*c6_b_info)[3].mFileTimeHi = 0U;
  (*c6_b_info)[4].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  (*c6_b_info)[4].name = "eml_blas_inline";
  (*c6_b_info)[4].dominantType = "";
  (*c6_b_info)[4].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  (*c6_b_info)[4].fileTimeLo = 1299076768U;
  (*c6_b_info)[4].fileTimeHi = 0U;
  (*c6_b_info)[4].mFileTimeLo = 0U;
  (*c6_b_info)[4].mFileTimeHi = 0U;
  (*c6_b_info)[5].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  (*c6_b_info)[5].name = "mtimes";
  (*c6_b_info)[5].dominantType = "double";
  (*c6_b_info)[5].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  (*c6_b_info)[5].fileTimeLo = 1289519692U;
  (*c6_b_info)[5].fileTimeHi = 0U;
  (*c6_b_info)[5].mFileTimeLo = 0U;
  (*c6_b_info)[5].mFileTimeHi = 0U;
  (*c6_b_info)[6].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  (*c6_b_info)[6].name = "eml_scalar_eg";
  (*c6_b_info)[6].dominantType = "double";
  (*c6_b_info)[6].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  (*c6_b_info)[6].fileTimeLo = 1286818796U;
  (*c6_b_info)[6].fileTimeHi = 0U;
  (*c6_b_info)[6].mFileTimeLo = 0U;
  (*c6_b_info)[6].mFileTimeHi = 0U;
  (*c6_b_info)[7].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  (*c6_b_info)[7].name = "eml_refblas_xgemm";
  (*c6_b_info)[7].dominantType = "int32";
  (*c6_b_info)[7].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  (*c6_b_info)[7].fileTimeLo = 1299076774U;
  (*c6_b_info)[7].fileTimeHi = 0U;
  (*c6_b_info)[7].mFileTimeLo = 0U;
  (*c6_b_info)[7].mFileTimeHi = 0U;
  sf_mex_assign(&c6_m0, sf_mex_createstruct("nameCaptureInfo", 1, 8), FALSE);
  for (c6_i51 = 0; c6_i51 < 8; c6_i51++) {
    c6_r0 = &c6_info[c6_i51];
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c6_r0->context)), "context", "nameCaptureInfo",
                    c6_i51);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c6_r0->name)), "name", "nameCaptureInfo", c6_i51);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c6_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c6_i51);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c6_r0->resolved)), "resolved", "nameCaptureInfo",
                    c6_i51);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c6_i51);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c6_i51);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c6_i51);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c6_i51);
  }

  sf_mex_assign(&c6_nameCaptureInfo, c6_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c6_nameCaptureInfo);
  return c6_nameCaptureInfo;
}

static void c6_eml_scalar_eg(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance)
{
}

static void c6_d_emlrt_marshallIn(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_wln_m, const char_T *c6_identifier, real_T
  c6_y[72])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_wln_m), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_wln_m);
}

static void c6_e_emlrt_marshallIn(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[72])
{
  real_T c6_dv3[72];
  int32_T c6_i52;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv3, 1, 0, 0U, 1, 0U, 2, 12, 6);
  for (c6_i52 = 0; c6_i52 < 72; c6_i52++) {
    c6_y[c6_i52] = c6_dv3[c6_i52];
  }

  sf_mex_destroy(&c6_u);
}

static const mxArray *c6_g_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(int32_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static int32_T c6_f_emlrt_marshallIn(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  int32_T c6_y;
  int32_T c6_i53;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_i53, 1, 6, 0U, 0, 0U, 0);
  c6_y = c6_i53;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_sfEvent;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  int32_T c6_y;
  SFc6_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_b_sfEvent = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_sfEvent),
    &c6_thisId);
  sf_mex_destroy(&c6_b_sfEvent);
  *(int32_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static uint8_T c6_g_emlrt_marshallIn(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_b_is_active_c6_dynamics_kinematics, const
  char_T *c6_identifier)
{
  uint8_T c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c6_b_is_active_c6_dynamics_kinematics), &c6_thisId);
  sf_mex_destroy(&c6_b_is_active_c6_dynamics_kinematics);
  return c6_y;
}

static uint8_T c6_h_emlrt_marshallIn(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  uint8_T c6_y;
  uint8_T c6_u0;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_u0, 1, 3, 0U, 0, 0U, 0);
  c6_y = c6_u0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void init_dsm_address_info(SFc6_dynamics_kinematicsInstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c6_dynamics_kinematics_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4214271632U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(236513190U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3860240470U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(83592104U);
}

mxArray *sf_c6_dynamics_kinematics_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("D0YaXTHH5AO1LocTE0Q2YB");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
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

static const mxArray *sf_get_sim_state_info_c6_dynamics_kinematics(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"zeta\",},{M[8],M[0],T\"is_active_c6_dynamics_kinematics\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c6_dynamics_kinematics_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc6_dynamics_kinematicsInstanceStruct *chartInstance;
    chartInstance = (SFc6_dynamics_kinematicsInstanceStruct *) ((ChartInfoStruct
      *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_dynamics_kinematicsMachineNumber_,
           6,
           1,
           1,
           5,
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
          _SFD_SET_DATA_PROPS(1,2,0,1,"zeta");
          _SFD_SET_DATA_PROPS(2,1,1,0,"vee");
          _SFD_SET_DATA_PROPS(3,1,1,0,"q");
          _SFD_SET_DATA_PROPS(4,1,1,0,"eta2");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,169);
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
            1.0,0,0,(MexFcnForType)c6_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)
            c6_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real_T (*c6_J)[72];
          real_T (*c6_zeta)[12];
          real_T (*c6_vee)[6];
          real_T (*c6_q)[6];
          real_T (*c6_eta2)[3];
          c6_eta2 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 3);
          c6_q = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 2);
          c6_vee = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 1);
          c6_zeta = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 1);
          c6_J = (real_T (*)[72])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c6_J);
          _SFD_SET_DATA_VALUE_PTR(1U, *c6_zeta);
          _SFD_SET_DATA_VALUE_PTR(2U, *c6_vee);
          _SFD_SET_DATA_VALUE_PTR(3U, *c6_q);
          _SFD_SET_DATA_VALUE_PTR(4U, *c6_eta2);
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
  return "ZMBMZLcBWjmf6T3Hb0LfW";
}

static void sf_opaque_initialize_c6_dynamics_kinematics(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc6_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c6_dynamics_kinematics
    ((SFc6_dynamics_kinematicsInstanceStruct*) chartInstanceVar);
  initialize_c6_dynamics_kinematics((SFc6_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c6_dynamics_kinematics(void *chartInstanceVar)
{
  enable_c6_dynamics_kinematics((SFc6_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c6_dynamics_kinematics(void *chartInstanceVar)
{
  disable_c6_dynamics_kinematics((SFc6_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c6_dynamics_kinematics(void *chartInstanceVar)
{
  sf_c6_dynamics_kinematics((SFc6_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c6_dynamics_kinematics(SimStruct*
  S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c6_dynamics_kinematics
    ((SFc6_dynamics_kinematicsInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c6_dynamics_kinematics();/* state var info */
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

extern void sf_internal_set_sim_state_c6_dynamics_kinematics(SimStruct* S, const
  mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c6_dynamics_kinematics();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c6_dynamics_kinematics((SFc6_dynamics_kinematicsInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c6_dynamics_kinematics(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c6_dynamics_kinematics(S);
}

static void sf_opaque_set_sim_state_c6_dynamics_kinematics(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c6_dynamics_kinematics(S, st);
}

static void sf_opaque_terminate_c6_dynamics_kinematics(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc6_dynamics_kinematicsInstanceStruct*) chartInstanceVar
      )->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c6_dynamics_kinematics((SFc6_dynamics_kinematicsInstanceStruct*)
      chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_dynamics_kinematics_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc6_dynamics_kinematics((SFc6_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c6_dynamics_kinematics(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c6_dynamics_kinematics
      ((SFc6_dynamics_kinematicsInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c6_dynamics_kinematics(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_dynamics_kinematics_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      6);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,6,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,6,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,6,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,6,1);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,6);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1110599110U));
  ssSetChecksum1(S,(169939312U));
  ssSetChecksum2(S,(517561538U));
  ssSetChecksum3(S,(2692302841U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c6_dynamics_kinematics(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c6_dynamics_kinematics(SimStruct *S)
{
  SFc6_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_dynamics_kinematicsInstanceStruct *)malloc(sizeof
    (SFc6_dynamics_kinematicsInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc6_dynamics_kinematicsInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c6_dynamics_kinematics;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c6_dynamics_kinematics;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c6_dynamics_kinematics;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c6_dynamics_kinematics;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c6_dynamics_kinematics;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c6_dynamics_kinematics;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c6_dynamics_kinematics;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c6_dynamics_kinematics;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c6_dynamics_kinematics;
  chartInstance->chartInfo.mdlStart = mdlStart_c6_dynamics_kinematics;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c6_dynamics_kinematics;
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

void c6_dynamics_kinematics_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c6_dynamics_kinematics(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c6_dynamics_kinematics(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c6_dynamics_kinematics(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c6_dynamics_kinematics_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
