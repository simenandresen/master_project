/* Include files */

#include "blascompat32.h"
#include "kinematics_sfun.h"
#include "c8_kinematics.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "kinematics_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c8_debug_family_names[26] = { "ee_pos", "h_q", "h_phi",
  "h_theta", "h_x", "h_z", "I", "x", "x_b_vel", "k", "nargin", "nargout", "v_ee",
  "J", "J_inv", "q", "eta1", "eta2", "ee_eta1", "qmin", "qmax", "d", "a",
  "alpha", "vec", "x_ee_in_body" };

static const char * c8_b_debug_family_names[7] = { "nargin", "nargout", "theta",
  "d", "a", "alpha", "mat" };

static const char * c8_c_debug_family_names[12] = { "cphi", "sphi", "cth", "sth",
  "cpsi", "spsi", "nargin", "nargout", "phi", "theta", "psi", "R" };

static const char * c8_d_debug_family_names[10] = { "g", "Rtemp", "tool_hom_mat",
  "nargin", "nargout", "q", "d", "a", "alpha", "vec" };

/* Function Declarations */
static void initialize_c8_kinematics(SFc8_kinematicsInstanceStruct
  *chartInstance);
static void initialize_params_c8_kinematics(SFc8_kinematicsInstanceStruct
  *chartInstance);
static void enable_c8_kinematics(SFc8_kinematicsInstanceStruct *chartInstance);
static void disable_c8_kinematics(SFc8_kinematicsInstanceStruct *chartInstance);
static void c8_update_debugger_state_c8_kinematics(SFc8_kinematicsInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c8_kinematics(SFc8_kinematicsInstanceStruct *
  chartInstance);
static void set_sim_state_c8_kinematics(SFc8_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_st);
static void finalize_c8_kinematics(SFc8_kinematicsInstanceStruct *chartInstance);
static void sf_c8_kinematics(SFc8_kinematicsInstanceStruct *chartInstance);
static void c8_chartstep_c8_kinematics(SFc8_kinematicsInstanceStruct
  *chartInstance);
static void initSimStructsc8_kinematics(SFc8_kinematicsInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c8_machineNumber, uint32_T
  c8_chartNumber);
static const mxArray *c8_sf_marshallOut(void *chartInstanceVoid, void *c8_inData);
static real_T c8_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_x_ee_in_body, const char_T *c8_identifier);
static real_T c8_b_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void c8_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_b_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static void c8_c_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_vec, const char_T *c8_identifier, real_T c8_y[12]);
static void c8_d_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[12]);
static void c8_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_c_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static void c8_e_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[6]);
static void c8_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_d_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static const mxArray *c8_e_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static const mxArray *c8_f_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static const mxArray *c8_g_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static void c8_f_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[144]);
static void c8_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static void c8_g_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[3]);
static void c8_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_h_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static void c8_h_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[16]);
static void c8_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_i_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static void c8_i_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[9]);
static void c8_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static void c8_info_helper(c8_ResolvedFunctionInfo c8_info[45]);
static void c8_ee_in_vehicle_frame(SFc8_kinematicsInstanceStruct *chartInstance,
  real_T c8_q[6], real_T c8_b_d[6], real_T c8_b_a[6], real_T c8_b_alpha[6],
  real_T c8_vec[3]);
static void c8_isVariableSizing(SFc8_kinematicsInstanceStruct *chartInstance);
static void c8_eye(SFc8_kinematicsInstanceStruct *chartInstance, real_T c8_I[16]);
static void c8_eml_scalar_eg(SFc8_kinematicsInstanceStruct *chartInstance);
static void c8_eml_int_forloop_overflow_check(SFc8_kinematicsInstanceStruct
  *chartInstance);
static void c8_dh_homogenous(SFc8_kinematicsInstanceStruct *chartInstance,
  real_T c8_theta, real_T c8_b_d, real_T c8_b_a, real_T c8_b_alpha, real_T
  c8_mat[16]);
static void c8_b_eml_scalar_eg(SFc8_kinematicsInstanceStruct *chartInstance);
static real_T c8_power(SFc8_kinematicsInstanceStruct *chartInstance, real_T
  c8_b_a);
static void c8_b_eml_int_forloop_overflow_check(SFc8_kinematicsInstanceStruct
  *chartInstance);
static const mxArray *c8_j_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static int32_T c8_j_emlrt_marshallIn(SFc8_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void c8_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static uint8_T c8_k_emlrt_marshallIn(SFc8_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_b_is_active_c8_kinematics, const char_T
  *c8_identifier);
static uint8_T c8_l_emlrt_marshallIn(SFc8_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void init_dsm_address_info(SFc8_kinematicsInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c8_kinematics(SFc8_kinematicsInstanceStruct
  *chartInstance)
{
  chartInstance->c8_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c8_is_active_c8_kinematics = 0U;
}

static void initialize_params_c8_kinematics(SFc8_kinematicsInstanceStruct
  *chartInstance)
{
  real_T c8_dv0[6];
  int32_T c8_i0;
  real_T c8_dv1[6];
  int32_T c8_i1;
  real_T c8_dv2[6];
  int32_T c8_i2;
  real_T c8_dv3[6];
  int32_T c8_i3;
  real_T c8_dv4[6];
  int32_T c8_i4;
  sf_set_error_prefix_string(
    "Error evaluating data 'qmin' in the parent workspace.\n");
  sf_mex_import_named("qmin", sf_mex_get_sfun_param(chartInstance->S, 4, 0),
                      c8_dv0, 0, 0, 0U, 1, 0U, 1, 6);
  for (c8_i0 = 0; c8_i0 < 6; c8_i0++) {
    chartInstance->c8_qmin[c8_i0] = c8_dv0[c8_i0];
  }

  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
  sf_set_error_prefix_string(
    "Error evaluating data 'qmax' in the parent workspace.\n");
  sf_mex_import_named("qmax", sf_mex_get_sfun_param(chartInstance->S, 3, 0),
                      c8_dv1, 0, 0, 0U, 1, 0U, 1, 6);
  for (c8_i1 = 0; c8_i1 < 6; c8_i1++) {
    chartInstance->c8_qmax[c8_i1] = c8_dv1[c8_i1];
  }

  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
  sf_set_error_prefix_string(
    "Error evaluating data 'd' in the parent workspace.\n");
  sf_mex_import_named("d", sf_mex_get_sfun_param(chartInstance->S, 2, 0), c8_dv2,
                      0, 0, 0U, 1, 0U, 1, 6);
  for (c8_i2 = 0; c8_i2 < 6; c8_i2++) {
    chartInstance->c8_d[c8_i2] = c8_dv2[c8_i2];
  }

  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
  sf_set_error_prefix_string(
    "Error evaluating data 'a' in the parent workspace.\n");
  sf_mex_import_named("a", sf_mex_get_sfun_param(chartInstance->S, 0, 0), c8_dv3,
                      0, 0, 0U, 1, 0U, 1, 6);
  for (c8_i3 = 0; c8_i3 < 6; c8_i3++) {
    chartInstance->c8_a[c8_i3] = c8_dv3[c8_i3];
  }

  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
  sf_set_error_prefix_string(
    "Error evaluating data 'alpha' in the parent workspace.\n");
  sf_mex_import_named("alpha", sf_mex_get_sfun_param(chartInstance->S, 1, 0),
                      c8_dv4, 0, 0, 0U, 1, 0U, 1, 6);
  for (c8_i4 = 0; c8_i4 < 6; c8_i4++) {
    chartInstance->c8_alpha[c8_i4] = c8_dv4[c8_i4];
  }

  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
}

static void enable_c8_kinematics(SFc8_kinematicsInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c8_kinematics(SFc8_kinematicsInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c8_update_debugger_state_c8_kinematics(SFc8_kinematicsInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c8_kinematics(SFc8_kinematicsInstanceStruct *
  chartInstance)
{
  const mxArray *c8_st;
  const mxArray *c8_y = NULL;
  int32_T c8_i5;
  real_T c8_u[12];
  const mxArray *c8_b_y = NULL;
  real_T c8_hoistedGlobal;
  real_T c8_b_u;
  const mxArray *c8_c_y = NULL;
  uint8_T c8_b_hoistedGlobal;
  uint8_T c8_c_u;
  const mxArray *c8_d_y = NULL;
  real_T *c8_x_ee_in_body;
  real_T (*c8_vec)[12];
  c8_x_ee_in_body = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c8_vec = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 1);
  c8_st = NULL;
  c8_st = NULL;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_createcellarray(3), FALSE);
  for (c8_i5 = 0; c8_i5 < 12; c8_i5++) {
    c8_u[c8_i5] = (*c8_vec)[c8_i5];
  }

  c8_b_y = NULL;
  sf_mex_assign(&c8_b_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 1, 12), FALSE);
  sf_mex_setcell(c8_y, 0, c8_b_y);
  c8_hoistedGlobal = *c8_x_ee_in_body;
  c8_b_u = c8_hoistedGlobal;
  c8_c_y = NULL;
  sf_mex_assign(&c8_c_y, sf_mex_create("y", &c8_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c8_y, 1, c8_c_y);
  c8_b_hoistedGlobal = chartInstance->c8_is_active_c8_kinematics;
  c8_c_u = c8_b_hoistedGlobal;
  c8_d_y = NULL;
  sf_mex_assign(&c8_d_y, sf_mex_create("y", &c8_c_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c8_y, 2, c8_d_y);
  sf_mex_assign(&c8_st, c8_y, FALSE);
  return c8_st;
}

static void set_sim_state_c8_kinematics(SFc8_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_st)
{
  const mxArray *c8_u;
  real_T c8_dv5[12];
  int32_T c8_i6;
  real_T *c8_x_ee_in_body;
  real_T (*c8_vec)[12];
  c8_x_ee_in_body = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c8_vec = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c8_doneDoubleBufferReInit = TRUE;
  c8_u = sf_mex_dup(c8_st);
  c8_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c8_u, 0)),
                        "vec", c8_dv5);
  for (c8_i6 = 0; c8_i6 < 12; c8_i6++) {
    (*c8_vec)[c8_i6] = c8_dv5[c8_i6];
  }

  *c8_x_ee_in_body = c8_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c8_u, 1)), "x_ee_in_body");
  chartInstance->c8_is_active_c8_kinematics = c8_k_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c8_u, 2)),
     "is_active_c8_kinematics");
  sf_mex_destroy(&c8_u);
  c8_update_debugger_state_c8_kinematics(chartInstance);
  sf_mex_destroy(&c8_st);
}

static void finalize_c8_kinematics(SFc8_kinematicsInstanceStruct *chartInstance)
{
}

static void sf_c8_kinematics(SFc8_kinematicsInstanceStruct *chartInstance)
{
  int32_T c8_i7;
  int32_T c8_i8;
  int32_T c8_i9;
  int32_T c8_i10;
  int32_T c8_i11;
  int32_T c8_i12;
  int32_T c8_i13;
  int32_T c8_i14;
  int32_T c8_i15;
  int32_T c8_i16;
  int32_T c8_i17;
  int32_T c8_i18;
  int32_T c8_i19;
  real_T *c8_x_ee_in_body;
  real_T (*c8_ee_eta1)[3];
  real_T (*c8_eta2)[3];
  real_T (*c8_eta1)[3];
  real_T (*c8_q)[6];
  real_T (*c8_vec)[12];
  real_T (*c8_J_inv)[72];
  real_T (*c8_J)[72];
  real_T (*c8_v_ee)[6];
  c8_x_ee_in_body = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c8_ee_eta1 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 6);
  c8_eta2 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 5);
  c8_eta1 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 4);
  c8_q = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 3);
  c8_vec = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 1);
  c8_J_inv = (real_T (*)[72])ssGetInputPortSignal(chartInstance->S, 2);
  c8_J = (real_T (*)[72])ssGetInputPortSignal(chartInstance->S, 1);
  c8_v_ee = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 7U, chartInstance->c8_sfEvent);
  for (c8_i7 = 0; c8_i7 < 6; c8_i7++) {
    _SFD_DATA_RANGE_CHECK((*c8_v_ee)[c8_i7], 0U);
  }

  for (c8_i8 = 0; c8_i8 < 72; c8_i8++) {
    _SFD_DATA_RANGE_CHECK((*c8_J)[c8_i8], 1U);
  }

  for (c8_i9 = 0; c8_i9 < 72; c8_i9++) {
    _SFD_DATA_RANGE_CHECK((*c8_J_inv)[c8_i9], 2U);
  }

  for (c8_i10 = 0; c8_i10 < 12; c8_i10++) {
    _SFD_DATA_RANGE_CHECK((*c8_vec)[c8_i10], 3U);
  }

  for (c8_i11 = 0; c8_i11 < 6; c8_i11++) {
    _SFD_DATA_RANGE_CHECK((*c8_q)[c8_i11], 4U);
  }

  for (c8_i12 = 0; c8_i12 < 3; c8_i12++) {
    _SFD_DATA_RANGE_CHECK((*c8_eta1)[c8_i12], 5U);
  }

  for (c8_i13 = 0; c8_i13 < 3; c8_i13++) {
    _SFD_DATA_RANGE_CHECK((*c8_eta2)[c8_i13], 6U);
  }

  for (c8_i14 = 0; c8_i14 < 3; c8_i14++) {
    _SFD_DATA_RANGE_CHECK((*c8_ee_eta1)[c8_i14], 7U);
  }

  for (c8_i15 = 0; c8_i15 < 6; c8_i15++) {
    _SFD_DATA_RANGE_CHECK(chartInstance->c8_qmin[c8_i15], 8U);
  }

  for (c8_i16 = 0; c8_i16 < 6; c8_i16++) {
    _SFD_DATA_RANGE_CHECK(chartInstance->c8_qmax[c8_i16], 9U);
  }

  for (c8_i17 = 0; c8_i17 < 6; c8_i17++) {
    _SFD_DATA_RANGE_CHECK(chartInstance->c8_d[c8_i17], 10U);
  }

  for (c8_i18 = 0; c8_i18 < 6; c8_i18++) {
    _SFD_DATA_RANGE_CHECK(chartInstance->c8_a[c8_i18], 11U);
  }

  for (c8_i19 = 0; c8_i19 < 6; c8_i19++) {
    _SFD_DATA_RANGE_CHECK(chartInstance->c8_alpha[c8_i19], 12U);
  }

  _SFD_DATA_RANGE_CHECK(*c8_x_ee_in_body, 13U);
  chartInstance->c8_sfEvent = CALL_EVENT;
  c8_chartstep_c8_kinematics(chartInstance);
  sf_debug_check_for_state_inconsistency(_kinematicsMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c8_chartstep_c8_kinematics(SFc8_kinematicsInstanceStruct
  *chartInstance)
{
  int32_T c8_i20;
  real_T c8_v_ee[6];
  int32_T c8_i21;
  real_T c8_J[72];
  int32_T c8_i22;
  real_T c8_J_inv[72];
  int32_T c8_i23;
  real_T c8_q[6];
  int32_T c8_i24;
  real_T c8_eta1[3];
  int32_T c8_i25;
  real_T c8_eta2[3];
  int32_T c8_i26;
  real_T c8_ee_eta1[3];
  int32_T c8_i27;
  real_T c8_b_qmin[6];
  int32_T c8_i28;
  real_T c8_b_qmax[6];
  int32_T c8_i29;
  real_T c8_b_d[6];
  int32_T c8_i30;
  real_T c8_b_a[6];
  int32_T c8_i31;
  real_T c8_b_alpha[6];
  uint32_T c8_debug_family_var_map[26];
  real_T c8_ee_pos[3];
  real_T c8_h_q[6];
  real_T c8_h_phi;
  real_T c8_h_theta;
  real_T c8_h_x;
  real_T c8_h_z;
  real_T c8_I[144];
  real_T c8_x;
  real_T c8_x_b_vel;
  real_T c8_k;
  real_T c8_nargin = 12.0;
  real_T c8_nargout = 2.0;
  real_T c8_vec[12];
  real_T c8_x_ee_in_body;
  int32_T c8_i32;
  real_T c8_b_q[6];
  int32_T c8_i33;
  real_T c8_c_d[6];
  int32_T c8_i34;
  real_T c8_c_a[6];
  int32_T c8_i35;
  real_T c8_c_alpha[6];
  real_T c8_dv6[3];
  int32_T c8_i36;
  int32_T c8_i37;
  real_T c8_b;
  real_T c8_y;
  real_T c8_d_a;
  real_T c8_b_b;
  real_T c8_b_y;
  real_T c8_e_a;
  real_T c8_c_b;
  real_T c8_c_y;
  real_T c8_d_b;
  real_T c8_d_y;
  real_T c8_e_b;
  real_T c8_e_y;
  int32_T c8_i;
  real_T c8_b_i;
  real_T c8_f_b;
  real_T c8_f_y;
  real_T c8_f_a;
  real_T c8_g_b;
  real_T c8_g_y;
  real_T c8_A;
  real_T c8_B;
  real_T c8_b_x;
  real_T c8_h_y;
  real_T c8_c_x;
  real_T c8_i_y;
  real_T c8_j_y;
  real_T c8_g_a;
  real_T c8_h_b;
  real_T c8_k_y;
  real_T c8_b_A;
  real_T c8_b_B;
  real_T c8_d_x;
  real_T c8_l_y;
  real_T c8_e_x;
  real_T c8_m_y;
  real_T c8_n_y;
  int32_T c8_i38;
  int32_T c8_c_i;
  int32_T c8_d_i;
  int32_T c8_i39;
  real_T c8_c_q[6];
  int32_T c8_i40;
  real_T c8_d_d[6];
  int32_T c8_i41;
  real_T c8_h_a[6];
  int32_T c8_i42;
  real_T c8_d_alpha[6];
  real_T c8_dv7[3];
  int32_T c8_i43;
  real_T c8_i_b;
  real_T c8_j_b;
  int32_T c8_i44;
  int32_T c8_i45;
  real_T c8_k_b[12];
  int32_T c8_i46;
  int32_T c8_i47;
  real_T *c8_b_x_ee_in_body;
  real_T (*c8_b_vec)[12];
  real_T (*c8_b_ee_eta1)[3];
  real_T (*c8_b_eta2)[3];
  real_T (*c8_b_eta1)[3];
  real_T (*c8_d_q)[6];
  real_T (*c8_b_J_inv)[72];
  real_T (*c8_b_J)[72];
  real_T (*c8_b_v_ee)[6];
  c8_b_x_ee_in_body = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c8_b_ee_eta1 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 6);
  c8_b_eta2 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 5);
  c8_b_eta1 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 4);
  c8_d_q = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 3);
  c8_b_vec = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 1);
  c8_b_J_inv = (real_T (*)[72])ssGetInputPortSignal(chartInstance->S, 2);
  c8_b_J = (real_T (*)[72])ssGetInputPortSignal(chartInstance->S, 1);
  c8_b_v_ee = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 7U, chartInstance->c8_sfEvent);
  for (c8_i20 = 0; c8_i20 < 6; c8_i20++) {
    c8_v_ee[c8_i20] = (*c8_b_v_ee)[c8_i20];
  }

  for (c8_i21 = 0; c8_i21 < 72; c8_i21++) {
    c8_J[c8_i21] = (*c8_b_J)[c8_i21];
  }

  for (c8_i22 = 0; c8_i22 < 72; c8_i22++) {
    c8_J_inv[c8_i22] = (*c8_b_J_inv)[c8_i22];
  }

  for (c8_i23 = 0; c8_i23 < 6; c8_i23++) {
    c8_q[c8_i23] = (*c8_d_q)[c8_i23];
  }

  for (c8_i24 = 0; c8_i24 < 3; c8_i24++) {
    c8_eta1[c8_i24] = (*c8_b_eta1)[c8_i24];
  }

  for (c8_i25 = 0; c8_i25 < 3; c8_i25++) {
    c8_eta2[c8_i25] = (*c8_b_eta2)[c8_i25];
  }

  for (c8_i26 = 0; c8_i26 < 3; c8_i26++) {
    c8_ee_eta1[c8_i26] = (*c8_b_ee_eta1)[c8_i26];
  }

  for (c8_i27 = 0; c8_i27 < 6; c8_i27++) {
    c8_b_qmin[c8_i27] = chartInstance->c8_qmin[c8_i27];
  }

  for (c8_i28 = 0; c8_i28 < 6; c8_i28++) {
    c8_b_qmax[c8_i28] = chartInstance->c8_qmax[c8_i28];
  }

  for (c8_i29 = 0; c8_i29 < 6; c8_i29++) {
    c8_b_d[c8_i29] = chartInstance->c8_d[c8_i29];
  }

  for (c8_i30 = 0; c8_i30 < 6; c8_i30++) {
    c8_b_a[c8_i30] = chartInstance->c8_a[c8_i30];
  }

  for (c8_i31 = 0; c8_i31 < 6; c8_i31++) {
    c8_b_alpha[c8_i31] = chartInstance->c8_alpha[c8_i31];
  }

  sf_debug_symbol_scope_push_eml(0U, 26U, 26U, c8_debug_family_names,
    c8_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c8_ee_pos, 0U, c8_d_sf_marshallOut,
    c8_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_h_q, 1U, c8_c_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_h_phi, 2U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_h_theta, 3U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_h_x, 4U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_h_z, 5U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_I, 6U, c8_g_sf_marshallOut,
    c8_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_x, 7U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_x_b_vel, 8U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c8_k, 9U, c8_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c8_nargin, 10U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_nargout, 11U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c8_v_ee, 12U, c8_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c8_J, 13U, c8_f_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c8_J_inv, 14U, c8_e_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c8_q, 15U, c8_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c8_eta1, 16U, c8_d_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c8_eta2, 17U, c8_d_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c8_ee_eta1, 18U, c8_d_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c8_b_qmin, 19U, c8_c_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_b_qmax, 20U, c8_c_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_b_d, 21U, c8_c_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_b_a, 22U, c8_c_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_b_alpha, 23U, c8_c_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_vec, 24U, c8_b_sf_marshallOut,
    c8_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_x_ee_in_body, 25U,
    c8_sf_marshallOut, c8_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 5);
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 6);
  for (c8_i32 = 0; c8_i32 < 6; c8_i32++) {
    c8_b_q[c8_i32] = c8_q[c8_i32];
  }

  for (c8_i33 = 0; c8_i33 < 6; c8_i33++) {
    c8_c_d[c8_i33] = c8_b_d[c8_i33];
  }

  for (c8_i34 = 0; c8_i34 < 6; c8_i34++) {
    c8_c_a[c8_i34] = c8_b_a[c8_i34];
  }

  for (c8_i35 = 0; c8_i35 < 6; c8_i35++) {
    c8_c_alpha[c8_i35] = c8_b_alpha[c8_i35];
  }

  c8_ee_in_vehicle_frame(chartInstance, c8_b_q, c8_c_d, c8_c_a, c8_c_alpha,
    c8_dv6);
  for (c8_i36 = 0; c8_i36 < 3; c8_i36++) {
    c8_ee_pos[c8_i36] = c8_dv6[c8_i36];
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 10);
  for (c8_i37 = 0; c8_i37 < 6; c8_i37++) {
    c8_h_q[c8_i37] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 11);
  c8_b = c8_eta2[0];
  c8_y = 2.0 * c8_b;
  c8_d_a = c8_y;
  c8_h_phi = c8_d_a * 100.0;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 12);
  c8_b_b = c8_eta2[1];
  c8_b_y = 2.0 * c8_b_b;
  c8_e_a = c8_b_y;
  c8_h_theta = c8_e_a * 50.0;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 13);
  c8_c_b = c8_ee_pos[0];
  c8_c_y = c8_c_b;
  c8_d_b = c8_c_y - 1.3;
  c8_d_y = 5.0 * c8_d_b;
  c8_h_x = -c8_d_y;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 14);
  c8_e_b = c8_ee_pos[2];
  c8_e_y = 20.0 * c8_e_b;
  c8_h_z = -c8_e_y;
  c8_i = 0;
  while (c8_i < 6) {
    c8_b_i = 1.0 + (real_T)c8_i;
    CV_EML_FOR(0, 1, 0, 1);
    _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 17);
    c8_f_b = -c8_power(chartInstance, c8_b_qmax[_SFD_EML_ARRAY_BOUNDS_CHECK(
      "qmax", (int32_T)_SFD_INTEGER_CHECK("i", c8_b_i), 1, 6, 1, 0) - 1] -
                       c8_b_qmin[_SFD_EML_ARRAY_BOUNDS_CHECK("qmin", (int32_T)
      _SFD_INTEGER_CHECK("i", c8_b_i), 1, 6, 1, 0) - 1]);
    c8_f_y = 0.25 * c8_f_b;
    c8_f_a = -c8_q[_SFD_EML_ARRAY_BOUNDS_CHECK("q", (int32_T)_SFD_INTEGER_CHECK(
      "i", c8_b_i), 1, 6, 1, 0) - 1] + c8_b_qmax[_SFD_EML_ARRAY_BOUNDS_CHECK(
      "qmax", (int32_T)_SFD_INTEGER_CHECK("i", c8_b_i), 1, 6, 1, 0) - 1];
    c8_g_b = c8_power(chartInstance, c8_q[_SFD_EML_ARRAY_BOUNDS_CHECK("q",
      (int32_T)_SFD_INTEGER_CHECK("i", c8_b_i), 1, 6, 1, 0) - 1] -
                      c8_b_qmin[_SFD_EML_ARRAY_BOUNDS_CHECK("qmin", (int32_T)
      _SFD_INTEGER_CHECK("i", c8_b_i), 1, 6, 1, 0) - 1]);
    c8_g_y = c8_f_a * c8_g_b;
    c8_A = c8_f_y;
    c8_B = c8_g_y;
    c8_b_x = c8_A;
    c8_h_y = c8_B;
    c8_c_x = c8_b_x;
    c8_i_y = c8_h_y;
    c8_j_y = c8_c_x / c8_i_y;
    c8_g_a = c8_power(chartInstance, -c8_q[_SFD_EML_ARRAY_BOUNDS_CHECK("q",
      (int32_T)_SFD_INTEGER_CHECK("i", c8_b_i), 1, 6, 1, 0) - 1] +
                      c8_b_qmax[_SFD_EML_ARRAY_BOUNDS_CHECK("qmax", (int32_T)
      _SFD_INTEGER_CHECK("i", c8_b_i), 1, 6, 1, 0) - 1]);
    c8_h_b = c8_q[_SFD_EML_ARRAY_BOUNDS_CHECK("q", (int32_T)_SFD_INTEGER_CHECK(
      "i", c8_b_i), 1, 6, 1, 0) - 1] - c8_b_qmin[_SFD_EML_ARRAY_BOUNDS_CHECK(
      "qmin", (int32_T)_SFD_INTEGER_CHECK("i", c8_b_i), 1, 6, 1, 0) - 1];
    c8_k_y = c8_g_a * c8_h_b;
    c8_b_A = c8_power(chartInstance, c8_b_qmax[_SFD_EML_ARRAY_BOUNDS_CHECK(
      "qmax", (int32_T)_SFD_INTEGER_CHECK("i", c8_b_i), 1, 6, 1, 0) - 1] -
                      c8_b_qmin[_SFD_EML_ARRAY_BOUNDS_CHECK("qmin", (int32_T)
      _SFD_INTEGER_CHECK("i", c8_b_i), 1, 6, 1, 0) - 1]);
    c8_b_B = c8_k_y;
    c8_d_x = c8_b_A;
    c8_l_y = c8_b_B;
    c8_e_x = c8_d_x;
    c8_m_y = c8_l_y;
    c8_n_y = c8_e_x / c8_m_y;
    c8_h_q[_SFD_EML_ARRAY_BOUNDS_CHECK("h_q", (int32_T)_SFD_INTEGER_CHECK("i",
      c8_b_i), 1, 6, 1, 0) - 1] = c8_j_y + c8_n_y;
    c8_i++;
    sf_mex_listen_for_ctrl_c(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 0, 0);
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 19);
  c8_isVariableSizing(chartInstance);
  for (c8_i38 = 0; c8_i38 < 144; c8_i38++) {
    c8_I[c8_i38] = 0.0;
  }

  c8_b_eml_int_forloop_overflow_check(chartInstance);
  for (c8_c_i = 1; c8_c_i < 13; c8_c_i++) {
    c8_d_i = c8_c_i;
    c8_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c8_d_i), 1, 12, 1, 0) + 12 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_d_i), 1, 12, 2, 0) - 1))
      - 1] = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 24);
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 26);
  for (c8_i39 = 0; c8_i39 < 6; c8_i39++) {
    c8_c_q[c8_i39] = c8_q[c8_i39];
  }

  for (c8_i40 = 0; c8_i40 < 6; c8_i40++) {
    c8_d_d[c8_i40] = c8_b_d[c8_i40];
  }

  for (c8_i41 = 0; c8_i41 < 6; c8_i41++) {
    c8_h_a[c8_i41] = c8_b_a[c8_i41];
  }

  for (c8_i42 = 0; c8_i42 < 6; c8_i42++) {
    c8_d_alpha[c8_i42] = c8_b_alpha[c8_i42];
  }

  c8_ee_in_vehicle_frame(chartInstance, c8_c_q, c8_d_d, c8_h_a, c8_d_alpha,
    c8_dv7);
  for (c8_i43 = 0; c8_i43 < 3; c8_i43++) {
    c8_ee_pos[c8_i43] = c8_dv7[c8_i43];
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 27);
  c8_x = c8_ee_pos[0];
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 29);
  if (CV_EML_IF(0, 1, 0, c8_x > 2.1)) {
    _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 30);
    c8_i_b = c8_power(chartInstance, c8_x - 2.1);
    c8_x_b_vel = 2.0 * c8_i_b;
  } else {
    _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 31);
    if (CV_EML_IF(0, 1, 1, c8_x < 1.5)) {
      _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 32);
      c8_j_b = c8_power(chartInstance, c8_x - 1.5);
      c8_x_b_vel = -2.0 * c8_j_b;
    } else {
      _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 34);
      c8_x_b_vel = 0.0;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 39);
  c8_x_ee_in_body = c8_x;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 43);
  for (c8_i44 = 0; c8_i44 < 12; c8_i44++) {
    c8_vec[c8_i44] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 46);
  c8_vec[0] = c8_x_b_vel;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 49);
  c8_k = 0.4;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 50);
  for (c8_i45 = 0; c8_i45 < 12; c8_i45++) {
    c8_k_b[c8_i45] = c8_vec[c8_i45];
  }

  for (c8_i46 = 0; c8_i46 < 12; c8_i46++) {
    c8_vec[c8_i46] = 0.4 * c8_k_b[c8_i46];
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, -50);
  sf_debug_symbol_scope_pop();
  for (c8_i47 = 0; c8_i47 < 12; c8_i47++) {
    (*c8_b_vec)[c8_i47] = c8_vec[c8_i47];
  }

  *c8_b_x_ee_in_body = c8_x_ee_in_body;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 7U, chartInstance->c8_sfEvent);
}

static void initSimStructsc8_kinematics(SFc8_kinematicsInstanceStruct
  *chartInstance)
{
}

static void init_script_number_translation(uint32_T c8_machineNumber, uint32_T
  c8_chartNumber)
{
  _SFD_SCRIPT_TRANSLATION(c8_chartNumber, 0U, sf_debug_get_script_id(
    "/home/simena/Dropbox/master_project/code/matlab_kinematics/ee_in_vehicle_frame.m"));
  _SFD_SCRIPT_TRANSLATION(c8_chartNumber, 1U, sf_debug_get_script_id(
    "/home/simena/Dropbox/master_project/code/matlab/gnc_mfiles/Rzyx.m"));
}

static const mxArray *c8_sf_marshallOut(void *chartInstanceVoid, void *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  real_T c8_u;
  const mxArray *c8_y = NULL;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_u = *(real_T *)c8_inData;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", &c8_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static real_T c8_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_x_ee_in_body, const char_T *c8_identifier)
{
  real_T c8_y;
  emlrtMsgIdentifier c8_thisId;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_x_ee_in_body),
    &c8_thisId);
  sf_mex_destroy(&c8_x_ee_in_body);
  return c8_y;
}

static real_T c8_b_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  real_T c8_y;
  real_T c8_d0;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_d0, 1, 0, 0U, 0, 0U, 0);
  c8_y = c8_d0;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void c8_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_x_ee_in_body;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_x_ee_in_body = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_x_ee_in_body),
    &c8_thisId);
  sf_mex_destroy(&c8_x_ee_in_body);
  *(real_T *)c8_outData = c8_y;
  sf_mex_destroy(&c8_mxArrayInData);
}

static const mxArray *c8_b_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i48;
  real_T c8_b_inData[12];
  int32_T c8_i49;
  real_T c8_u[12];
  const mxArray *c8_y = NULL;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  for (c8_i48 = 0; c8_i48 < 12; c8_i48++) {
    c8_b_inData[c8_i48] = (*(real_T (*)[12])c8_inData)[c8_i48];
  }

  for (c8_i49 = 0; c8_i49 < 12; c8_i49++) {
    c8_u[c8_i49] = c8_b_inData[c8_i49];
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 1, 12), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static void c8_c_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_vec, const char_T *c8_identifier, real_T c8_y[12])
{
  emlrtMsgIdentifier c8_thisId;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_vec), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_vec);
}

static void c8_d_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[12])
{
  real_T c8_dv8[12];
  int32_T c8_i50;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv8, 1, 0, 0U, 1, 0U, 1, 12);
  for (c8_i50 = 0; c8_i50 < 12; c8_i50++) {
    c8_y[c8_i50] = c8_dv8[c8_i50];
  }

  sf_mex_destroy(&c8_u);
}

static void c8_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_vec;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y[12];
  int32_T c8_i51;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_vec = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_vec), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_vec);
  for (c8_i51 = 0; c8_i51 < 12; c8_i51++) {
    (*(real_T (*)[12])c8_outData)[c8_i51] = c8_y[c8_i51];
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

static const mxArray *c8_c_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i52;
  real_T c8_b_inData[6];
  int32_T c8_i53;
  real_T c8_u[6];
  const mxArray *c8_y = NULL;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  for (c8_i52 = 0; c8_i52 < 6; c8_i52++) {
    c8_b_inData[c8_i52] = (*(real_T (*)[6])c8_inData)[c8_i52];
  }

  for (c8_i53 = 0; c8_i53 < 6; c8_i53++) {
    c8_u[c8_i53] = c8_b_inData[c8_i53];
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static void c8_e_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[6])
{
  real_T c8_dv9[6];
  int32_T c8_i54;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv9, 1, 0, 0U, 1, 0U, 1, 6);
  for (c8_i54 = 0; c8_i54 < 6; c8_i54++) {
    c8_y[c8_i54] = c8_dv9[c8_i54];
  }

  sf_mex_destroy(&c8_u);
}

static void c8_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_b_alpha;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y[6];
  int32_T c8_i55;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_b_alpha = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_b_alpha), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_b_alpha);
  for (c8_i55 = 0; c8_i55 < 6; c8_i55++) {
    (*(real_T (*)[6])c8_outData)[c8_i55] = c8_y[c8_i55];
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

static const mxArray *c8_d_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i56;
  real_T c8_b_inData[3];
  int32_T c8_i57;
  real_T c8_u[3];
  const mxArray *c8_y = NULL;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  for (c8_i56 = 0; c8_i56 < 3; c8_i56++) {
    c8_b_inData[c8_i56] = (*(real_T (*)[3])c8_inData)[c8_i56];
  }

  for (c8_i57 = 0; c8_i57 < 3; c8_i57++) {
    c8_u[c8_i57] = c8_b_inData[c8_i57];
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static const mxArray *c8_e_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i58;
  int32_T c8_i59;
  int32_T c8_i60;
  real_T c8_b_inData[72];
  int32_T c8_i61;
  int32_T c8_i62;
  int32_T c8_i63;
  real_T c8_u[72];
  const mxArray *c8_y = NULL;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_i58 = 0;
  for (c8_i59 = 0; c8_i59 < 6; c8_i59++) {
    for (c8_i60 = 0; c8_i60 < 12; c8_i60++) {
      c8_b_inData[c8_i60 + c8_i58] = (*(real_T (*)[72])c8_inData)[c8_i60 +
        c8_i58];
    }

    c8_i58 += 12;
  }

  c8_i61 = 0;
  for (c8_i62 = 0; c8_i62 < 6; c8_i62++) {
    for (c8_i63 = 0; c8_i63 < 12; c8_i63++) {
      c8_u[c8_i63 + c8_i61] = c8_b_inData[c8_i63 + c8_i61];
    }

    c8_i61 += 12;
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 2, 12, 6), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static const mxArray *c8_f_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i64;
  int32_T c8_i65;
  int32_T c8_i66;
  real_T c8_b_inData[72];
  int32_T c8_i67;
  int32_T c8_i68;
  int32_T c8_i69;
  real_T c8_u[72];
  const mxArray *c8_y = NULL;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_i64 = 0;
  for (c8_i65 = 0; c8_i65 < 12; c8_i65++) {
    for (c8_i66 = 0; c8_i66 < 6; c8_i66++) {
      c8_b_inData[c8_i66 + c8_i64] = (*(real_T (*)[72])c8_inData)[c8_i66 +
        c8_i64];
    }

    c8_i64 += 6;
  }

  c8_i67 = 0;
  for (c8_i68 = 0; c8_i68 < 12; c8_i68++) {
    for (c8_i69 = 0; c8_i69 < 6; c8_i69++) {
      c8_u[c8_i69 + c8_i67] = c8_b_inData[c8_i69 + c8_i67];
    }

    c8_i67 += 6;
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 2, 6, 12), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static const mxArray *c8_g_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i70;
  int32_T c8_i71;
  int32_T c8_i72;
  real_T c8_b_inData[144];
  int32_T c8_i73;
  int32_T c8_i74;
  int32_T c8_i75;
  real_T c8_u[144];
  const mxArray *c8_y = NULL;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_i70 = 0;
  for (c8_i71 = 0; c8_i71 < 12; c8_i71++) {
    for (c8_i72 = 0; c8_i72 < 12; c8_i72++) {
      c8_b_inData[c8_i72 + c8_i70] = (*(real_T (*)[144])c8_inData)[c8_i72 +
        c8_i70];
    }

    c8_i70 += 12;
  }

  c8_i73 = 0;
  for (c8_i74 = 0; c8_i74 < 12; c8_i74++) {
    for (c8_i75 = 0; c8_i75 < 12; c8_i75++) {
      c8_u[c8_i75 + c8_i73] = c8_b_inData[c8_i75 + c8_i73];
    }

    c8_i73 += 12;
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 2, 12, 12), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static void c8_f_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[144])
{
  real_T c8_dv10[144];
  int32_T c8_i76;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv10, 1, 0, 0U, 1, 0U, 2, 12,
                12);
  for (c8_i76 = 0; c8_i76 < 144; c8_i76++) {
    c8_y[c8_i76] = c8_dv10[c8_i76];
  }

  sf_mex_destroy(&c8_u);
}

static void c8_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_I;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y[144];
  int32_T c8_i77;
  int32_T c8_i78;
  int32_T c8_i79;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_I = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_I), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_I);
  c8_i77 = 0;
  for (c8_i78 = 0; c8_i78 < 12; c8_i78++) {
    for (c8_i79 = 0; c8_i79 < 12; c8_i79++) {
      (*(real_T (*)[144])c8_outData)[c8_i79 + c8_i77] = c8_y[c8_i79 + c8_i77];
    }

    c8_i77 += 12;
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

static void c8_g_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[3])
{
  real_T c8_dv11[3];
  int32_T c8_i80;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv11, 1, 0, 0U, 1, 0U, 1, 3);
  for (c8_i80 = 0; c8_i80 < 3; c8_i80++) {
    c8_y[c8_i80] = c8_dv11[c8_i80];
  }

  sf_mex_destroy(&c8_u);
}

static void c8_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_ee_pos;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y[3];
  int32_T c8_i81;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_ee_pos = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_ee_pos), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_ee_pos);
  for (c8_i81 = 0; c8_i81 < 3; c8_i81++) {
    (*(real_T (*)[3])c8_outData)[c8_i81] = c8_y[c8_i81];
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

static const mxArray *c8_h_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i82;
  int32_T c8_i83;
  int32_T c8_i84;
  real_T c8_b_inData[16];
  int32_T c8_i85;
  int32_T c8_i86;
  int32_T c8_i87;
  real_T c8_u[16];
  const mxArray *c8_y = NULL;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_i82 = 0;
  for (c8_i83 = 0; c8_i83 < 4; c8_i83++) {
    for (c8_i84 = 0; c8_i84 < 4; c8_i84++) {
      c8_b_inData[c8_i84 + c8_i82] = (*(real_T (*)[16])c8_inData)[c8_i84 +
        c8_i82];
    }

    c8_i82 += 4;
  }

  c8_i85 = 0;
  for (c8_i86 = 0; c8_i86 < 4; c8_i86++) {
    for (c8_i87 = 0; c8_i87 < 4; c8_i87++) {
      c8_u[c8_i87 + c8_i85] = c8_b_inData[c8_i87 + c8_i85];
    }

    c8_i85 += 4;
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static void c8_h_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[16])
{
  real_T c8_dv12[16];
  int32_T c8_i88;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv12, 1, 0, 0U, 1, 0U, 2, 4, 4);
  for (c8_i88 = 0; c8_i88 < 16; c8_i88++) {
    c8_y[c8_i88] = c8_dv12[c8_i88];
  }

  sf_mex_destroy(&c8_u);
}

static void c8_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_mat;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y[16];
  int32_T c8_i89;
  int32_T c8_i90;
  int32_T c8_i91;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mat = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_mat), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_mat);
  c8_i89 = 0;
  for (c8_i90 = 0; c8_i90 < 4; c8_i90++) {
    for (c8_i91 = 0; c8_i91 < 4; c8_i91++) {
      (*(real_T (*)[16])c8_outData)[c8_i91 + c8_i89] = c8_y[c8_i91 + c8_i89];
    }

    c8_i89 += 4;
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

static const mxArray *c8_i_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i92;
  int32_T c8_i93;
  int32_T c8_i94;
  real_T c8_b_inData[9];
  int32_T c8_i95;
  int32_T c8_i96;
  int32_T c8_i97;
  real_T c8_u[9];
  const mxArray *c8_y = NULL;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_i92 = 0;
  for (c8_i93 = 0; c8_i93 < 3; c8_i93++) {
    for (c8_i94 = 0; c8_i94 < 3; c8_i94++) {
      c8_b_inData[c8_i94 + c8_i92] = (*(real_T (*)[9])c8_inData)[c8_i94 + c8_i92];
    }

    c8_i92 += 3;
  }

  c8_i95 = 0;
  for (c8_i96 = 0; c8_i96 < 3; c8_i96++) {
    for (c8_i97 = 0; c8_i97 < 3; c8_i97++) {
      c8_u[c8_i97 + c8_i95] = c8_b_inData[c8_i97 + c8_i95];
    }

    c8_i95 += 3;
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 2, 3, 3), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static void c8_i_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[9])
{
  real_T c8_dv13[9];
  int32_T c8_i98;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv13, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c8_i98 = 0; c8_i98 < 9; c8_i98++) {
    c8_y[c8_i98] = c8_dv13[c8_i98];
  }

  sf_mex_destroy(&c8_u);
}

static void c8_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_R;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y[9];
  int32_T c8_i99;
  int32_T c8_i100;
  int32_T c8_i101;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_R = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_R), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_R);
  c8_i99 = 0;
  for (c8_i100 = 0; c8_i100 < 3; c8_i100++) {
    for (c8_i101 = 0; c8_i101 < 3; c8_i101++) {
      (*(real_T (*)[9])c8_outData)[c8_i101 + c8_i99] = c8_y[c8_i101 + c8_i99];
    }

    c8_i99 += 3;
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

const mxArray *sf_c8_kinematics_get_eml_resolved_functions_info(void)
{
  const mxArray *c8_nameCaptureInfo;
  c8_ResolvedFunctionInfo c8_info[45];
  const mxArray *c8_m0 = NULL;
  int32_T c8_i102;
  c8_ResolvedFunctionInfo *c8_r0;
  c8_nameCaptureInfo = NULL;
  c8_nameCaptureInfo = NULL;
  c8_info_helper(c8_info);
  sf_mex_assign(&c8_m0, sf_mex_createstruct("nameCaptureInfo", 1, 45), FALSE);
  for (c8_i102 = 0; c8_i102 < 45; c8_i102++) {
    c8_r0 = &c8_info[c8_i102];
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", c8_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c8_r0->context)), "context", "nameCaptureInfo",
                    c8_i102);
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", c8_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c8_r0->name)), "name", "nameCaptureInfo", c8_i102);
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", c8_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c8_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c8_i102);
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", c8_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c8_r0->resolved)), "resolved", "nameCaptureInfo",
                    c8_i102);
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", &c8_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c8_i102);
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", &c8_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c8_i102);
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", &c8_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c8_i102);
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", &c8_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c8_i102);
  }

  sf_mex_assign(&c8_nameCaptureInfo, c8_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c8_nameCaptureInfo);
  return c8_nameCaptureInfo;
}

static void c8_info_helper(c8_ResolvedFunctionInfo c8_info[45])
{
  c8_info[0].context = "";
  c8_info[0].name = "ee_in_vehicle_frame";
  c8_info[0].dominantType = "double";
  c8_info[0].resolved =
    "[E]/home/simena/Dropbox/master_project/code/matlab_kinematics/ee_in_vehicle_frame.m";
  c8_info[0].fileTimeLo = 1385744105U;
  c8_info[0].fileTimeHi = 0U;
  c8_info[0].mFileTimeLo = 0U;
  c8_info[0].mFileTimeHi = 0U;
  c8_info[1].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab_kinematics/ee_in_vehicle_frame.m";
  c8_info[1].name = "eye";
  c8_info[1].dominantType = "double";
  c8_info[1].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m";
  c8_info[1].fileTimeLo = 1286818688U;
  c8_info[1].fileTimeHi = 0U;
  c8_info[1].mFileTimeLo = 0U;
  c8_info[1].mFileTimeHi = 0U;
  c8_info[2].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c8_info[2].name = "eml_assert_valid_size_arg";
  c8_info[2].dominantType = "double";
  c8_info[2].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c8_info[2].fileTimeLo = 1286818694U;
  c8_info[2].fileTimeHi = 0U;
  c8_info[2].mFileTimeLo = 0U;
  c8_info[2].mFileTimeHi = 0U;
  c8_info[3].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral";
  c8_info[3].name = "isinf";
  c8_info[3].dominantType = "double";
  c8_info[3].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isinf.m";
  c8_info[3].fileTimeLo = 1286818760U;
  c8_info[3].fileTimeHi = 0U;
  c8_info[3].mFileTimeLo = 0U;
  c8_info[3].mFileTimeHi = 0U;
  c8_info[4].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!numel_for_size";
  c8_info[4].name = "mtimes";
  c8_info[4].dominantType = "double";
  c8_info[4].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c8_info[4].fileTimeLo = 1289519692U;
  c8_info[4].fileTimeHi = 0U;
  c8_info[4].mFileTimeLo = 0U;
  c8_info[4].mFileTimeHi = 0U;
  c8_info[5].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c8_info[5].name = "eml_index_class";
  c8_info[5].dominantType = "";
  c8_info[5].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[5].fileTimeLo = 1286818778U;
  c8_info[5].fileTimeHi = 0U;
  c8_info[5].mFileTimeLo = 0U;
  c8_info[5].mFileTimeHi = 0U;
  c8_info[6].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c8_info[6].name = "intmax";
  c8_info[6].dominantType = "char";
  c8_info[6].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/intmax.m";
  c8_info[6].fileTimeLo = 1311255316U;
  c8_info[6].fileTimeHi = 0U;
  c8_info[6].mFileTimeLo = 0U;
  c8_info[6].mFileTimeHi = 0U;
  c8_info[7].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c8_info[7].name = "eml_is_float_class";
  c8_info[7].dominantType = "char";
  c8_info[7].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c8_info[7].fileTimeLo = 1286818782U;
  c8_info[7].fileTimeHi = 0U;
  c8_info[7].mFileTimeLo = 0U;
  c8_info[7].mFileTimeHi = 0U;
  c8_info[8].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c8_info[8].name = "min";
  c8_info[8].dominantType = "double";
  c8_info[8].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/min.m";
  c8_info[8].fileTimeLo = 1311255318U;
  c8_info[8].fileTimeHi = 0U;
  c8_info[8].mFileTimeLo = 0U;
  c8_info[8].mFileTimeHi = 0U;
  c8_info[9].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/min.m";
  c8_info[9].name = "eml_min_or_max";
  c8_info[9].dominantType = "char";
  c8_info[9].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c8_info[9].fileTimeLo = 1303146212U;
  c8_info[9].fileTimeHi = 0U;
  c8_info[9].mFileTimeLo = 0U;
  c8_info[9].mFileTimeHi = 0U;
  c8_info[10].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c8_info[10].name = "eml_scalar_eg";
  c8_info[10].dominantType = "double";
  c8_info[10].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c8_info[10].fileTimeLo = 1286818796U;
  c8_info[10].fileTimeHi = 0U;
  c8_info[10].mFileTimeLo = 0U;
  c8_info[10].mFileTimeHi = 0U;
  c8_info[11].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c8_info[11].name = "eml_scalexp_alloc";
  c8_info[11].dominantType = "double";
  c8_info[11].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c8_info[11].fileTimeLo = 1286818796U;
  c8_info[11].fileTimeHi = 0U;
  c8_info[11].mFileTimeLo = 0U;
  c8_info[11].mFileTimeHi = 0U;
  c8_info[12].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c8_info[12].name = "eml_index_class";
  c8_info[12].dominantType = "";
  c8_info[12].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[12].fileTimeLo = 1286818778U;
  c8_info[12].fileTimeHi = 0U;
  c8_info[12].mFileTimeLo = 0U;
  c8_info[12].mFileTimeHi = 0U;
  c8_info[13].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c8_info[13].name = "eml_scalar_eg";
  c8_info[13].dominantType = "double";
  c8_info[13].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c8_info[13].fileTimeLo = 1286818796U;
  c8_info[13].fileTimeHi = 0U;
  c8_info[13].mFileTimeLo = 0U;
  c8_info[13].mFileTimeHi = 0U;
  c8_info[14].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c8_info[14].name = "eml_index_class";
  c8_info[14].dominantType = "";
  c8_info[14].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[14].fileTimeLo = 1286818778U;
  c8_info[14].fileTimeHi = 0U;
  c8_info[14].mFileTimeLo = 0U;
  c8_info[14].mFileTimeHi = 0U;
  c8_info[15].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c8_info[15].name = "eml_int_forloop_overflow_check";
  c8_info[15].dominantType = "";
  c8_info[15].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c8_info[15].fileTimeLo = 1311255316U;
  c8_info[15].fileTimeHi = 0U;
  c8_info[15].mFileTimeLo = 0U;
  c8_info[15].mFileTimeHi = 0U;
  c8_info[16].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c8_info[16].name = "intmax";
  c8_info[16].dominantType = "char";
  c8_info[16].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/intmax.m";
  c8_info[16].fileTimeLo = 1311255316U;
  c8_info[16].fileTimeHi = 0U;
  c8_info[16].mFileTimeLo = 0U;
  c8_info[16].mFileTimeHi = 0U;
  c8_info[17].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab_kinematics/ee_in_vehicle_frame.m!dh_homogenous";
  c8_info[17].name = "eye";
  c8_info[17].dominantType = "double";
  c8_info[17].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m";
  c8_info[17].fileTimeLo = 1286818688U;
  c8_info[17].fileTimeHi = 0U;
  c8_info[17].mFileTimeLo = 0U;
  c8_info[17].mFileTimeHi = 0U;
  c8_info[18].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab_kinematics/ee_in_vehicle_frame.m!dh_homogenous";
  c8_info[18].name = "cos";
  c8_info[18].dominantType = "double";
  c8_info[18].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/cos.m";
  c8_info[18].fileTimeLo = 1286818706U;
  c8_info[18].fileTimeHi = 0U;
  c8_info[18].mFileTimeLo = 0U;
  c8_info[18].mFileTimeHi = 0U;
  c8_info[19].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/cos.m";
  c8_info[19].name = "eml_scalar_cos";
  c8_info[19].dominantType = "double";
  c8_info[19].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m";
  c8_info[19].fileTimeLo = 1286818722U;
  c8_info[19].fileTimeHi = 0U;
  c8_info[19].mFileTimeLo = 0U;
  c8_info[19].mFileTimeHi = 0U;
  c8_info[20].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab_kinematics/ee_in_vehicle_frame.m!dh_homogenous";
  c8_info[20].name = "sin";
  c8_info[20].dominantType = "double";
  c8_info[20].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/sin.m";
  c8_info[20].fileTimeLo = 1286818750U;
  c8_info[20].fileTimeHi = 0U;
  c8_info[20].mFileTimeLo = 0U;
  c8_info[20].mFileTimeHi = 0U;
  c8_info[21].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/sin.m";
  c8_info[21].name = "eml_scalar_sin";
  c8_info[21].dominantType = "double";
  c8_info[21].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m";
  c8_info[21].fileTimeLo = 1286818736U;
  c8_info[21].fileTimeHi = 0U;
  c8_info[21].mFileTimeLo = 0U;
  c8_info[21].mFileTimeHi = 0U;
  c8_info[22].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab_kinematics/ee_in_vehicle_frame.m!dh_homogenous";
  c8_info[22].name = "mtimes";
  c8_info[22].dominantType = "double";
  c8_info[22].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c8_info[22].fileTimeLo = 1289519692U;
  c8_info[22].fileTimeHi = 0U;
  c8_info[22].mFileTimeLo = 0U;
  c8_info[22].mFileTimeHi = 0U;
  c8_info[23].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab_kinematics/ee_in_vehicle_frame.m";
  c8_info[23].name = "mtimes";
  c8_info[23].dominantType = "double";
  c8_info[23].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c8_info[23].fileTimeLo = 1289519692U;
  c8_info[23].fileTimeHi = 0U;
  c8_info[23].mFileTimeLo = 0U;
  c8_info[23].mFileTimeHi = 0U;
  c8_info[24].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c8_info[24].name = "eml_index_class";
  c8_info[24].dominantType = "";
  c8_info[24].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[24].fileTimeLo = 1286818778U;
  c8_info[24].fileTimeHi = 0U;
  c8_info[24].mFileTimeLo = 0U;
  c8_info[24].mFileTimeHi = 0U;
  c8_info[25].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c8_info[25].name = "eml_scalar_eg";
  c8_info[25].dominantType = "double";
  c8_info[25].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c8_info[25].fileTimeLo = 1286818796U;
  c8_info[25].fileTimeHi = 0U;
  c8_info[25].mFileTimeLo = 0U;
  c8_info[25].mFileTimeHi = 0U;
  c8_info[26].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c8_info[26].name = "eml_xgemm";
  c8_info[26].dominantType = "int32";
  c8_info[26].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c8_info[26].fileTimeLo = 1299076772U;
  c8_info[26].fileTimeHi = 0U;
  c8_info[26].mFileTimeLo = 0U;
  c8_info[26].mFileTimeHi = 0U;
  c8_info[27].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c8_info[27].name = "eml_blas_inline";
  c8_info[27].dominantType = "";
  c8_info[27].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c8_info[27].fileTimeLo = 1299076768U;
  c8_info[27].fileTimeHi = 0U;
  c8_info[27].mFileTimeLo = 0U;
  c8_info[27].mFileTimeHi = 0U;
  c8_info[28].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c8_info[28].name = "mtimes";
  c8_info[28].dominantType = "double";
  c8_info[28].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c8_info[28].fileTimeLo = 1289519692U;
  c8_info[28].fileTimeHi = 0U;
  c8_info[28].mFileTimeLo = 0U;
  c8_info[28].mFileTimeHi = 0U;
  c8_info[29].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c8_info[29].name = "eml_scalar_eg";
  c8_info[29].dominantType = "double";
  c8_info[29].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c8_info[29].fileTimeLo = 1286818796U;
  c8_info[29].fileTimeHi = 0U;
  c8_info[29].mFileTimeLo = 0U;
  c8_info[29].mFileTimeHi = 0U;
  c8_info[30].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c8_info[30].name = "eml_refblas_xgemm";
  c8_info[30].dominantType = "int32";
  c8_info[30].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c8_info[30].fileTimeLo = 1299076774U;
  c8_info[30].fileTimeHi = 0U;
  c8_info[30].mFileTimeLo = 0U;
  c8_info[30].mFileTimeHi = 0U;
  c8_info[31].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab_kinematics/ee_in_vehicle_frame.m";
  c8_info[31].name = "mrdivide";
  c8_info[31].dominantType = "double";
  c8_info[31].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c8_info[31].fileTimeLo = 1325124138U;
  c8_info[31].fileTimeHi = 0U;
  c8_info[31].mFileTimeLo = 1319729966U;
  c8_info[31].mFileTimeHi = 0U;
  c8_info[32].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c8_info[32].name = "rdivide";
  c8_info[32].dominantType = "double";
  c8_info[32].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/rdivide.m";
  c8_info[32].fileTimeLo = 1286818844U;
  c8_info[32].fileTimeHi = 0U;
  c8_info[32].mFileTimeLo = 0U;
  c8_info[32].mFileTimeHi = 0U;
  c8_info[33].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/rdivide.m";
  c8_info[33].name = "eml_div";
  c8_info[33].dominantType = "double";
  c8_info[33].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_div.m";
  c8_info[33].fileTimeLo = 1313347810U;
  c8_info[33].fileTimeHi = 0U;
  c8_info[33].mFileTimeLo = 0U;
  c8_info[33].mFileTimeHi = 0U;
  c8_info[34].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab_kinematics/ee_in_vehicle_frame.m";
  c8_info[34].name = "Rzyx";
  c8_info[34].dominantType = "double";
  c8_info[34].resolved =
    "[E]/home/simena/Dropbox/master_project/code/matlab/gnc_mfiles/Rzyx.m";
  c8_info[34].fileTimeLo = 1206483734U;
  c8_info[34].fileTimeHi = 0U;
  c8_info[34].mFileTimeLo = 0U;
  c8_info[34].mFileTimeHi = 0U;
  c8_info[35].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab/gnc_mfiles/Rzyx.m";
  c8_info[35].name = "cos";
  c8_info[35].dominantType = "double";
  c8_info[35].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/cos.m";
  c8_info[35].fileTimeLo = 1286818706U;
  c8_info[35].fileTimeHi = 0U;
  c8_info[35].mFileTimeLo = 0U;
  c8_info[35].mFileTimeHi = 0U;
  c8_info[36].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab/gnc_mfiles/Rzyx.m";
  c8_info[36].name = "sin";
  c8_info[36].dominantType = "double";
  c8_info[36].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/sin.m";
  c8_info[36].fileTimeLo = 1286818750U;
  c8_info[36].fileTimeHi = 0U;
  c8_info[36].mFileTimeLo = 0U;
  c8_info[36].mFileTimeHi = 0U;
  c8_info[37].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab/gnc_mfiles/Rzyx.m";
  c8_info[37].name = "mtimes";
  c8_info[37].dominantType = "double";
  c8_info[37].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c8_info[37].fileTimeLo = 1289519692U;
  c8_info[37].fileTimeHi = 0U;
  c8_info[37].mFileTimeLo = 0U;
  c8_info[37].mFileTimeHi = 0U;
  c8_info[38].context = "";
  c8_info[38].name = "mtimes";
  c8_info[38].dominantType = "double";
  c8_info[38].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c8_info[38].fileTimeLo = 1289519692U;
  c8_info[38].fileTimeHi = 0U;
  c8_info[38].mFileTimeLo = 0U;
  c8_info[38].mFileTimeHi = 0U;
  c8_info[39].context = "";
  c8_info[39].name = "mrdivide";
  c8_info[39].dominantType = "double";
  c8_info[39].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c8_info[39].fileTimeLo = 1325124138U;
  c8_info[39].fileTimeHi = 0U;
  c8_info[39].mFileTimeLo = 1319729966U;
  c8_info[39].mFileTimeHi = 0U;
  c8_info[40].context = "";
  c8_info[40].name = "power";
  c8_info[40].dominantType = "double";
  c8_info[40].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/power.m";
  c8_info[40].fileTimeLo = 1307651240U;
  c8_info[40].fileTimeHi = 0U;
  c8_info[40].mFileTimeLo = 0U;
  c8_info[40].mFileTimeHi = 0U;
  c8_info[41].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/power.m";
  c8_info[41].name = "eml_scalar_eg";
  c8_info[41].dominantType = "double";
  c8_info[41].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c8_info[41].fileTimeLo = 1286818796U;
  c8_info[41].fileTimeHi = 0U;
  c8_info[41].mFileTimeLo = 0U;
  c8_info[41].mFileTimeHi = 0U;
  c8_info[42].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/power.m";
  c8_info[42].name = "eml_scalexp_alloc";
  c8_info[42].dominantType = "double";
  c8_info[42].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c8_info[42].fileTimeLo = 1286818796U;
  c8_info[42].fileTimeHi = 0U;
  c8_info[42].mFileTimeLo = 0U;
  c8_info[42].mFileTimeHi = 0U;
  c8_info[43].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/power.m";
  c8_info[43].name = "eml_scalar_floor";
  c8_info[43].dominantType = "double";
  c8_info[43].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c8_info[43].fileTimeLo = 1286818726U;
  c8_info[43].fileTimeHi = 0U;
  c8_info[43].mFileTimeLo = 0U;
  c8_info[43].mFileTimeHi = 0U;
  c8_info[44].context = "";
  c8_info[44].name = "eye";
  c8_info[44].dominantType = "double";
  c8_info[44].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m";
  c8_info[44].fileTimeLo = 1286818688U;
  c8_info[44].fileTimeHi = 0U;
  c8_info[44].mFileTimeLo = 0U;
  c8_info[44].mFileTimeHi = 0U;
}

static void c8_ee_in_vehicle_frame(SFc8_kinematicsInstanceStruct *chartInstance,
  real_T c8_q[6], real_T c8_b_d[6], real_T c8_b_a[6], real_T c8_b_alpha[6],
  real_T c8_vec[3])
{
  uint32_T c8_debug_family_var_map[10];
  real_T c8_g[16];
  real_T c8_Rtemp[9];
  real_T c8_tool_hom_mat[16];
  real_T c8_nargin = 4.0;
  real_T c8_nargout = 1.0;
  real_T c8_dv14[16];
  int32_T c8_i103;
  int32_T c8_i;
  real_T c8_b_i;
  int32_T c8_i104;
  real_T c8_c_a[16];
  real_T c8_b[16];
  int32_T c8_i105;
  int32_T c8_i106;
  int32_T c8_i107;
  real_T c8_C[16];
  int32_T c8_i108;
  int32_T c8_i109;
  int32_T c8_i110;
  int32_T c8_i111;
  int32_T c8_i112;
  int32_T c8_i113;
  int32_T c8_i114;
  int32_T c8_i115;
  int32_T c8_i116;
  static real_T c8_dv15[9] = { 3.749399456654644E-33, 6.123233995736766E-17, 1.0,
    -1.0, 6.123233995736766E-17, 0.0, -6.123233995736766E-17, -1.0,
    6.123233995736766E-17 };

  int32_T c8_i117;
  static real_T c8_b_b[16] = { 3.749399456654644E-33, 6.123233995736766E-17, 1.0,
    0.0, -1.0, 6.123233995736766E-17, 0.0, 0.0, -6.123233995736766E-17, -1.0,
    6.123233995736766E-17, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T c8_i118;
  int32_T c8_i119;
  int32_T c8_i120;
  int32_T c8_i121;
  int32_T c8_i122;
  int32_T c8_i123;
  int32_T c8_i124;
  int32_T c8_i125;
  int32_T c8_i126;
  int32_T c8_i127;
  int32_T c8_i128;
  int32_T c8_i129;
  int32_T c8_i130;
  int32_T c8_i131;
  sf_debug_symbol_scope_push_eml(0U, 10U, 10U, c8_d_debug_family_names,
    c8_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c8_g, 0U, c8_h_sf_marshallOut,
    c8_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c8_Rtemp, 1U, c8_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c8_tool_hom_mat, 2U, c8_h_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c8_nargin, 3U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_nargout, 4U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_q, 5U, c8_c_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_b_d, 6U, c8_c_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_b_a, 7U, c8_c_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_b_alpha, 8U, c8_c_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_vec, 9U, c8_d_sf_marshallOut,
    c8_e_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c8_sfEvent, 3);
  c8_eye(chartInstance, c8_dv14);
  for (c8_i103 = 0; c8_i103 < 16; c8_i103++) {
    c8_g[c8_i103] = c8_dv14[c8_i103];
  }

  c8_i = 0;
  while (c8_i < 6) {
    c8_b_i = 1.0 + (real_T)c8_i;
    CV_SCRIPT_FOR(0, 0, 1);
    _SFD_SCRIPT_CALL(0U, chartInstance->c8_sfEvent, 5);
    for (c8_i104 = 0; c8_i104 < 16; c8_i104++) {
      c8_c_a[c8_i104] = c8_g[c8_i104];
    }

    c8_dh_homogenous(chartInstance, c8_q[_SFD_EML_ARRAY_BOUNDS_CHECK("q",
      (int32_T)_SFD_INTEGER_CHECK("i", c8_b_i), 1, 6, 1, 0) - 1],
                     c8_b_d[_SFD_EML_ARRAY_BOUNDS_CHECK("d", (int32_T)
      _SFD_INTEGER_CHECK("i", c8_b_i), 1, 6, 1, 0) - 1],
                     c8_b_a[_SFD_EML_ARRAY_BOUNDS_CHECK("a", (int32_T)
      _SFD_INTEGER_CHECK("i", c8_b_i), 1, 6, 1, 0) - 1],
                     c8_b_alpha[_SFD_EML_ARRAY_BOUNDS_CHECK("alpha", (int32_T)
      _SFD_INTEGER_CHECK("i", c8_b_i), 1, 6, 1, 0) - 1], c8_b);
    c8_b_eml_scalar_eg(chartInstance);
    c8_b_eml_scalar_eg(chartInstance);
    for (c8_i105 = 0; c8_i105 < 16; c8_i105++) {
      c8_g[c8_i105] = 0.0;
    }

    for (c8_i106 = 0; c8_i106 < 16; c8_i106++) {
      c8_g[c8_i106] = 0.0;
    }

    for (c8_i107 = 0; c8_i107 < 16; c8_i107++) {
      c8_C[c8_i107] = c8_g[c8_i107];
    }

    for (c8_i108 = 0; c8_i108 < 16; c8_i108++) {
      c8_g[c8_i108] = c8_C[c8_i108];
    }

    for (c8_i109 = 0; c8_i109 < 16; c8_i109++) {
      c8_C[c8_i109] = c8_g[c8_i109];
    }

    for (c8_i110 = 0; c8_i110 < 16; c8_i110++) {
      c8_g[c8_i110] = c8_C[c8_i110];
    }

    for (c8_i111 = 0; c8_i111 < 4; c8_i111++) {
      c8_i112 = 0;
      for (c8_i113 = 0; c8_i113 < 4; c8_i113++) {
        c8_g[c8_i112 + c8_i111] = 0.0;
        c8_i114 = 0;
        for (c8_i115 = 0; c8_i115 < 4; c8_i115++) {
          c8_g[c8_i112 + c8_i111] += c8_c_a[c8_i114 + c8_i111] * c8_b[c8_i115 +
            c8_i112];
          c8_i114 += 4;
        }

        c8_i112 += 4;
      }
    }

    c8_i++;
    sf_mex_listen_for_ctrl_c(chartInstance->S);
  }

  CV_SCRIPT_FOR(0, 0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c8_sfEvent, 8);
  for (c8_i116 = 0; c8_i116 < 9; c8_i116++) {
    c8_Rtemp[c8_i116] = c8_dv15[c8_i116];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c8_sfEvent, 9);
  for (c8_i117 = 0; c8_i117 < 16; c8_i117++) {
    c8_tool_hom_mat[c8_i117] = c8_b_b[c8_i117];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c8_sfEvent, 10);
  for (c8_i118 = 0; c8_i118 < 16; c8_i118++) {
    c8_c_a[c8_i118] = c8_g[c8_i118];
  }

  c8_b_eml_scalar_eg(chartInstance);
  c8_b_eml_scalar_eg(chartInstance);
  for (c8_i119 = 0; c8_i119 < 16; c8_i119++) {
    c8_g[c8_i119] = 0.0;
  }

  for (c8_i120 = 0; c8_i120 < 16; c8_i120++) {
    c8_g[c8_i120] = 0.0;
  }

  for (c8_i121 = 0; c8_i121 < 16; c8_i121++) {
    c8_C[c8_i121] = c8_g[c8_i121];
  }

  for (c8_i122 = 0; c8_i122 < 16; c8_i122++) {
    c8_g[c8_i122] = c8_C[c8_i122];
  }

  for (c8_i123 = 0; c8_i123 < 16; c8_i123++) {
    c8_C[c8_i123] = c8_g[c8_i123];
  }

  for (c8_i124 = 0; c8_i124 < 16; c8_i124++) {
    c8_g[c8_i124] = c8_C[c8_i124];
  }

  for (c8_i125 = 0; c8_i125 < 4; c8_i125++) {
    c8_i126 = 0;
    for (c8_i127 = 0; c8_i127 < 4; c8_i127++) {
      c8_g[c8_i126 + c8_i125] = 0.0;
      c8_i128 = 0;
      for (c8_i129 = 0; c8_i129 < 4; c8_i129++) {
        c8_g[c8_i126 + c8_i125] += c8_c_a[c8_i128 + c8_i125] * c8_b_b[c8_i129 +
          c8_i126];
        c8_i128 += 4;
      }

      c8_i126 += 4;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c8_sfEvent, 11);
  for (c8_i130 = 0; c8_i130 < 3; c8_i130++) {
    c8_vec[c8_i130] = 0.0;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c8_sfEvent, 12);
  for (c8_i131 = 0; c8_i131 < 3; c8_i131++) {
    c8_vec[c8_i131] = c8_g[c8_i131 + 12];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c8_sfEvent, -12);
  sf_debug_symbol_scope_pop();
}

static void c8_isVariableSizing(SFc8_kinematicsInstanceStruct *chartInstance)
{
}

static void c8_eye(SFc8_kinematicsInstanceStruct *chartInstance, real_T c8_I[16])
{
  int32_T c8_i132;
  int32_T c8_i;
  int32_T c8_b_i;
  c8_isVariableSizing(chartInstance);
  for (c8_i132 = 0; c8_i132 < 16; c8_i132++) {
    c8_I[c8_i132] = 0.0;
  }

  c8_eml_int_forloop_overflow_check(chartInstance);
  for (c8_i = 1; c8_i < 5; c8_i++) {
    c8_b_i = c8_i;
    c8_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c8_b_i), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_i), 1, 4, 2, 0) - 1) <<
           2)) - 1] = 1.0;
  }
}

static void c8_eml_scalar_eg(SFc8_kinematicsInstanceStruct *chartInstance)
{
}

static void c8_eml_int_forloop_overflow_check(SFc8_kinematicsInstanceStruct
  *chartInstance)
{
}

static void c8_dh_homogenous(SFc8_kinematicsInstanceStruct *chartInstance,
  real_T c8_theta, real_T c8_b_d, real_T c8_b_a, real_T c8_b_alpha, real_T
  c8_mat[16])
{
  uint32_T c8_debug_family_var_map[7];
  real_T c8_nargin = 4.0;
  real_T c8_nargout = 1.0;
  real_T c8_dv16[16];
  int32_T c8_i133;
  real_T c8_x;
  real_T c8_b_x;
  real_T c8_c_x;
  real_T c8_d_x;
  real_T c8_e_x;
  real_T c8_f_x;
  real_T c8_c_a;
  real_T c8_b;
  real_T c8_y;
  real_T c8_g_x;
  real_T c8_h_x;
  real_T c8_i_x;
  real_T c8_j_x;
  real_T c8_d_a;
  real_T c8_b_b;
  real_T c8_b_y;
  real_T c8_k_x;
  real_T c8_l_x;
  real_T c8_e_a;
  real_T c8_c_b;
  real_T c8_c_y;
  real_T c8_m_x;
  real_T c8_n_x;
  real_T c8_o_x;
  real_T c8_p_x;
  real_T c8_q_x;
  real_T c8_r_x;
  real_T c8_f_a;
  real_T c8_d_b;
  real_T c8_d_y;
  real_T c8_s_x;
  real_T c8_t_x;
  real_T c8_u_x;
  real_T c8_v_x;
  real_T c8_g_a;
  real_T c8_e_b;
  real_T c8_e_y;
  real_T c8_w_x;
  real_T c8_x_x;
  real_T c8_h_a;
  real_T c8_f_b;
  real_T c8_f_y;
  real_T c8_y_x;
  real_T c8_ab_x;
  real_T c8_bb_x;
  real_T c8_cb_x;
  int32_T c8_i134;
  int32_T c8_i135;
  static real_T c8_dv17[4] = { 0.0, 0.0, 0.0, 1.0 };

  sf_debug_symbol_scope_push_eml(0U, 7U, 7U, c8_b_debug_family_names,
    c8_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(&c8_nargin, 0U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_nargout, 1U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_theta, 2U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_b_d, 3U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_b_a, 4U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_b_alpha, 5U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_mat, 6U, c8_h_sf_marshallOut,
    c8_f_sf_marshallIn);
  CV_SCRIPT_FCN(0, 1);
  _SFD_SCRIPT_CALL(0U, chartInstance->c8_sfEvent, 36);
  c8_eye(chartInstance, c8_dv16);
  for (c8_i133 = 0; c8_i133 < 16; c8_i133++) {
    c8_mat[c8_i133] = c8_dv16[c8_i133];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c8_sfEvent, 37);
  c8_x = c8_theta;
  c8_b_x = c8_x;
  c8_b_x = muDoubleScalarCos(c8_b_x);
  c8_c_x = c8_theta;
  c8_d_x = c8_c_x;
  c8_d_x = muDoubleScalarSin(c8_d_x);
  c8_e_x = c8_b_alpha;
  c8_f_x = c8_e_x;
  c8_f_x = muDoubleScalarCos(c8_f_x);
  c8_c_a = -c8_d_x;
  c8_b = c8_f_x;
  c8_y = c8_c_a * c8_b;
  c8_g_x = c8_theta;
  c8_h_x = c8_g_x;
  c8_h_x = muDoubleScalarSin(c8_h_x);
  c8_i_x = c8_b_alpha;
  c8_j_x = c8_i_x;
  c8_j_x = muDoubleScalarSin(c8_j_x);
  c8_d_a = c8_h_x;
  c8_b_b = c8_j_x;
  c8_b_y = c8_d_a * c8_b_b;
  c8_k_x = c8_theta;
  c8_l_x = c8_k_x;
  c8_l_x = muDoubleScalarCos(c8_l_x);
  c8_e_a = c8_b_a;
  c8_c_b = c8_l_x;
  c8_c_y = c8_e_a * c8_c_b;
  c8_m_x = c8_theta;
  c8_n_x = c8_m_x;
  c8_n_x = muDoubleScalarSin(c8_n_x);
  c8_o_x = c8_theta;
  c8_p_x = c8_o_x;
  c8_p_x = muDoubleScalarCos(c8_p_x);
  c8_q_x = c8_b_alpha;
  c8_r_x = c8_q_x;
  c8_r_x = muDoubleScalarCos(c8_r_x);
  c8_f_a = c8_p_x;
  c8_d_b = c8_r_x;
  c8_d_y = c8_f_a * c8_d_b;
  c8_s_x = c8_theta;
  c8_t_x = c8_s_x;
  c8_t_x = muDoubleScalarCos(c8_t_x);
  c8_u_x = c8_b_alpha;
  c8_v_x = c8_u_x;
  c8_v_x = muDoubleScalarSin(c8_v_x);
  c8_g_a = -c8_t_x;
  c8_e_b = c8_v_x;
  c8_e_y = c8_g_a * c8_e_b;
  c8_w_x = c8_theta;
  c8_x_x = c8_w_x;
  c8_x_x = muDoubleScalarSin(c8_x_x);
  c8_h_a = c8_b_a;
  c8_f_b = c8_x_x;
  c8_f_y = c8_h_a * c8_f_b;
  c8_y_x = c8_b_alpha;
  c8_ab_x = c8_y_x;
  c8_ab_x = muDoubleScalarSin(c8_ab_x);
  c8_bb_x = c8_b_alpha;
  c8_cb_x = c8_bb_x;
  c8_cb_x = muDoubleScalarCos(c8_cb_x);
  c8_mat[0] = c8_b_x;
  c8_mat[4] = c8_y;
  c8_mat[8] = c8_b_y;
  c8_mat[12] = c8_c_y;
  c8_mat[1] = c8_n_x;
  c8_mat[5] = c8_d_y;
  c8_mat[9] = c8_e_y;
  c8_mat[13] = c8_f_y;
  c8_mat[2] = 0.0;
  c8_mat[6] = c8_ab_x;
  c8_mat[10] = c8_cb_x;
  c8_mat[14] = c8_b_d;
  c8_i134 = 0;
  for (c8_i135 = 0; c8_i135 < 4; c8_i135++) {
    c8_mat[c8_i134 + 3] = c8_dv17[c8_i135];
    c8_i134 += 4;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c8_sfEvent, -37);
  sf_debug_symbol_scope_pop();
}

static void c8_b_eml_scalar_eg(SFc8_kinematicsInstanceStruct *chartInstance)
{
}

static real_T c8_power(SFc8_kinematicsInstanceStruct *chartInstance, real_T
  c8_b_a)
{
  real_T c8_ak;
  c8_eml_scalar_eg(chartInstance);
  c8_ak = c8_b_a;
  return muDoubleScalarPower(c8_ak, 2.0);
}

static void c8_b_eml_int_forloop_overflow_check(SFc8_kinematicsInstanceStruct
  *chartInstance)
{
}

static const mxArray *c8_j_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_u;
  const mxArray *c8_y = NULL;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_u = *(int32_T *)c8_inData;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", &c8_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static int32_T c8_j_emlrt_marshallIn(SFc8_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  int32_T c8_y;
  int32_T c8_i136;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_i136, 1, 6, 0U, 0, 0U, 0);
  c8_y = c8_i136;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void c8_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_b_sfEvent;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  int32_T c8_y;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_b_sfEvent = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_b_sfEvent),
    &c8_thisId);
  sf_mex_destroy(&c8_b_sfEvent);
  *(int32_T *)c8_outData = c8_y;
  sf_mex_destroy(&c8_mxArrayInData);
}

static uint8_T c8_k_emlrt_marshallIn(SFc8_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_b_is_active_c8_kinematics, const char_T
  *c8_identifier)
{
  uint8_T c8_y;
  emlrtMsgIdentifier c8_thisId;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_l_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c8_b_is_active_c8_kinematics), &c8_thisId);
  sf_mex_destroy(&c8_b_is_active_c8_kinematics);
  return c8_y;
}

static uint8_T c8_l_emlrt_marshallIn(SFc8_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  uint8_T c8_y;
  uint8_T c8_u0;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_u0, 1, 3, 0U, 0, 0U, 0);
  c8_y = c8_u0;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void init_dsm_address_info(SFc8_kinematicsInstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c8_kinematics_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4225379507U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3422549489U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(150467233U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(354676489U);
}

mxArray *sf_c8_kinematics_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("vCJKyyuI5Z7XSCJy6wf4TE");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,7,3,dataFields);

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
      pr[0] = (double)(6);
      pr[1] = (double)(12);
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
      pr[0] = (double)(12);
      pr[1] = (double)(6);
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
      pr[0] = (double)(6);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,6,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,5,3,dataFields);

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
      pr[0] = (double)(6);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

static const mxArray *sf_get_sim_state_info_c8_kinematics(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x3'type','srcId','name','auxInfo'{{M[1],M[5],T\"vec\",},{M[1],M[18],T\"x_ee_in_body\",},{M[8],M[0],T\"is_active_c8_kinematics\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 3, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c8_kinematics_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc8_kinematicsInstanceStruct *chartInstance;
    chartInstance = (SFc8_kinematicsInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_kinematicsMachineNumber_,
           8,
           1,
           1,
           14,
           0,
           0,
           0,
           0,
           2,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           ssGetPath(S),
           (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation(_kinematicsMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting(_kinematicsMachineNumber_,
            chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(_kinematicsMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"v_ee");
          _SFD_SET_DATA_PROPS(1,1,1,0,"J");
          _SFD_SET_DATA_PROPS(2,1,1,0,"J_inv");
          _SFD_SET_DATA_PROPS(3,2,0,1,"vec");
          _SFD_SET_DATA_PROPS(4,1,1,0,"q");
          _SFD_SET_DATA_PROPS(5,1,1,0,"eta1");
          _SFD_SET_DATA_PROPS(6,1,1,0,"eta2");
          _SFD_SET_DATA_PROPS(7,1,1,0,"ee_eta1");
          _SFD_SET_DATA_PROPS(8,10,0,0,"qmin");
          _SFD_SET_DATA_PROPS(9,10,0,0,"qmax");
          _SFD_SET_DATA_PROPS(10,10,0,0,"d");
          _SFD_SET_DATA_PROPS(11,10,0,0,"a");
          _SFD_SET_DATA_PROPS(12,10,0,0,"alpha");
          _SFD_SET_DATA_PROPS(13,2,0,1,"x_ee_in_body");
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
        _SFD_CV_INIT_EML(0,1,1,2,0,0,1,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1269);
        _SFD_CV_INIT_EML_IF(0,1,0,755,765,800,884);
        _SFD_CV_INIT_EML_IF(0,1,1,800,814,850,884);
        _SFD_CV_INIT_EML_FOR(0,1,0,406,416,584);
        _SFD_CV_INIT_SCRIPT(0,2,0,0,0,1,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"ee_in_vehicle_frame",0,-1,284);
        _SFD_CV_INIT_SCRIPT_FCN(0,1,"dh_homogenous",813,-1,1104);
        _SFD_CV_INIT_SCRIPT_FOR(0,0,71,81,140);
        _SFD_CV_INIT_SCRIPT(1,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(1,0,"Rzyx",0,-1,1478);
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
            1.0,0,0,(MexFcnForType)c8_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 6;
          dimVector[1]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_f_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 12;
          dimVector[1]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_b_sf_marshallOut,(MexInFcnForType)
            c8_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_c_sf_marshallOut,(MexInFcnForType)
            c8_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_c_sf_marshallOut,(MexInFcnForType)
            c8_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_c_sf_marshallOut,(MexInFcnForType)
            c8_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_c_sf_marshallOut,(MexInFcnForType)
            c8_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(12,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_c_sf_marshallOut,(MexInFcnForType)
            c8_c_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(13,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c8_sf_marshallOut,(MexInFcnForType)c8_sf_marshallIn);

        {
          real_T *c8_x_ee_in_body;
          real_T (*c8_v_ee)[6];
          real_T (*c8_J)[72];
          real_T (*c8_J_inv)[72];
          real_T (*c8_vec)[12];
          real_T (*c8_q)[6];
          real_T (*c8_eta1)[3];
          real_T (*c8_eta2)[3];
          real_T (*c8_ee_eta1)[3];
          c8_x_ee_in_body = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c8_ee_eta1 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 6);
          c8_eta2 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 5);
          c8_eta1 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 4);
          c8_q = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 3);
          c8_vec = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 1);
          c8_J_inv = (real_T (*)[72])ssGetInputPortSignal(chartInstance->S, 2);
          c8_J = (real_T (*)[72])ssGetInputPortSignal(chartInstance->S, 1);
          c8_v_ee = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c8_v_ee);
          _SFD_SET_DATA_VALUE_PTR(1U, *c8_J);
          _SFD_SET_DATA_VALUE_PTR(2U, *c8_J_inv);
          _SFD_SET_DATA_VALUE_PTR(3U, *c8_vec);
          _SFD_SET_DATA_VALUE_PTR(4U, *c8_q);
          _SFD_SET_DATA_VALUE_PTR(5U, *c8_eta1);
          _SFD_SET_DATA_VALUE_PTR(6U, *c8_eta2);
          _SFD_SET_DATA_VALUE_PTR(7U, *c8_ee_eta1);
          _SFD_SET_DATA_VALUE_PTR(8U, chartInstance->c8_qmin);
          _SFD_SET_DATA_VALUE_PTR(9U, chartInstance->c8_qmax);
          _SFD_SET_DATA_VALUE_PTR(10U, chartInstance->c8_d);
          _SFD_SET_DATA_VALUE_PTR(11U, chartInstance->c8_a);
          _SFD_SET_DATA_VALUE_PTR(12U, chartInstance->c8_alpha);
          _SFD_SET_DATA_VALUE_PTR(13U, c8_x_ee_in_body);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(_kinematicsMachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization()
{
  return "sUWKNu1cIA16V79mHs84jD";
}

static void sf_opaque_initialize_c8_kinematics(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc8_kinematicsInstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c8_kinematics((SFc8_kinematicsInstanceStruct*)
    chartInstanceVar);
  initialize_c8_kinematics((SFc8_kinematicsInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c8_kinematics(void *chartInstanceVar)
{
  enable_c8_kinematics((SFc8_kinematicsInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c8_kinematics(void *chartInstanceVar)
{
  disable_c8_kinematics((SFc8_kinematicsInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c8_kinematics(void *chartInstanceVar)
{
  sf_c8_kinematics((SFc8_kinematicsInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c8_kinematics(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c8_kinematics
    ((SFc8_kinematicsInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c8_kinematics();/* state var info */
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

extern void sf_internal_set_sim_state_c8_kinematics(SimStruct* S, const mxArray *
  st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c8_kinematics();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c8_kinematics((SFc8_kinematicsInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c8_kinematics(SimStruct* S)
{
  return sf_internal_get_sim_state_c8_kinematics(S);
}

static void sf_opaque_set_sim_state_c8_kinematics(SimStruct* S, const mxArray
  *st)
{
  sf_internal_set_sim_state_c8_kinematics(S, st);
}

static void sf_opaque_terminate_c8_kinematics(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc8_kinematicsInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c8_kinematics((SFc8_kinematicsInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_kinematics_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc8_kinematics((SFc8_kinematicsInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c8_kinematics(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c8_kinematics((SFc8_kinematicsInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c8_kinematics(SimStruct *S)
{
  /* Actual parameters from chart:
     a alpha d qmax qmin
   */
  const char_T *rtParamNames[] = { "p1", "p2", "p3", "p4", "p5" };

  ssSetNumRunTimeParams(S,ssGetSFcnParamsCount(S));

  /* registration for a*/
  ssRegDlgParamAsRunTimeParam(S, 0, 0, rtParamNames[0], SS_DOUBLE);

  /* registration for alpha*/
  ssRegDlgParamAsRunTimeParam(S, 1, 1, rtParamNames[1], SS_DOUBLE);

  /* registration for d*/
  ssRegDlgParamAsRunTimeParam(S, 2, 2, rtParamNames[2], SS_DOUBLE);

  /* registration for qmax*/
  ssRegDlgParamAsRunTimeParam(S, 3, 3, rtParamNames[3], SS_DOUBLE);

  /* registration for qmin*/
  ssRegDlgParamAsRunTimeParam(S, 4, 4, rtParamNames[4], SS_DOUBLE);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_kinematics_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      8);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,8,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,8,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 6, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,8,7);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,8,2);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,8);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2957720918U));
  ssSetChecksum1(S,(3585508017U));
  ssSetChecksum2(S,(4181344032U));
  ssSetChecksum3(S,(2590505887U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c8_kinematics(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c8_kinematics(SimStruct *S)
{
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)malloc(sizeof
    (SFc8_kinematicsInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc8_kinematicsInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c8_kinematics;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c8_kinematics;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c8_kinematics;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c8_kinematics;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c8_kinematics;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c8_kinematics;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c8_kinematics;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c8_kinematics;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c8_kinematics;
  chartInstance->chartInfo.mdlStart = mdlStart_c8_kinematics;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c8_kinematics;
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

void c8_kinematics_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c8_kinematics(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c8_kinematics(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c8_kinematics(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c8_kinematics_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
