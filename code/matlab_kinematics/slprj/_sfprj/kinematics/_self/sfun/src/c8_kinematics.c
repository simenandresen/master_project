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
static const char * c8_debug_family_names[23] = { "ee_pos", "h_q", "h_phi",
  "h_theta", "h_x", "h_z", "I", "k", "nargin", "nargout", "v_ee", "J", "J_inv",
  "q", "eta1", "eta2", "ee_eta1", "qmin", "qmax", "d", "a", "alpha", "vec" };

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
static void c8_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_vec, const char_T *c8_identifier, real_T c8_y[12]);
static void c8_b_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[12]);
static void c8_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_b_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static void c8_c_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[6]);
static void c8_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_c_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static const mxArray *c8_d_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static const mxArray *c8_e_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static const mxArray *c8_f_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static real_T c8_d_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void c8_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_g_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static void c8_e_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[144]);
static void c8_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static void c8_f_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[3]);
static void c8_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_h_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static void c8_g_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[16]);
static void c8_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_i_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static void c8_h_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[9]);
static void c8_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static void c8_info_helper(c8_ResolvedFunctionInfo c8_info[153]);
static void c8_b_info_helper(c8_ResolvedFunctionInfo c8_info[153]);
static void c8_c_info_helper(c8_ResolvedFunctionInfo c8_info[153]);
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
static real_T c8_eml_div(SFc8_kinematicsInstanceStruct *chartInstance, real_T
  c8_x, real_T c8_y);
static real_T c8_power(SFc8_kinematicsInstanceStruct *chartInstance, real_T
  c8_b_a);
static void c8_b_eml_int_forloop_overflow_check(SFc8_kinematicsInstanceStruct
  *chartInstance);
static void c8_c_eml_scalar_eg(SFc8_kinematicsInstanceStruct *chartInstance);
static real_T c8_norm(SFc8_kinematicsInstanceStruct *chartInstance, real_T c8_x
                      [12]);
static void c8_realmin(SFc8_kinematicsInstanceStruct *chartInstance);
static real_T c8_abs(SFc8_kinematicsInstanceStruct *chartInstance, real_T c8_x);
static void c8_d_eml_scalar_eg(SFc8_kinematicsInstanceStruct *chartInstance);
static void c8_eml_xgemm(SFc8_kinematicsInstanceStruct *chartInstance, real_T
  c8_A[72], real_T c8_B[72], real_T c8_C[144], real_T c8_b_C[144]);
static void c8_e_eml_scalar_eg(SFc8_kinematicsInstanceStruct *chartInstance);
static real_T c8_b_norm(SFc8_kinematicsInstanceStruct *chartInstance, real_T
  c8_x[144]);
static void c8_c_eml_int_forloop_overflow_check(SFc8_kinematicsInstanceStruct
  *chartInstance);
static void c8_eml_error(SFc8_kinematicsInstanceStruct *chartInstance);
static void c8_eml_matlab_zsvdc(SFc8_kinematicsInstanceStruct *chartInstance,
  real_T c8_A[144], real_T c8_S[12]);
static void c8_d_eml_int_forloop_overflow_check(SFc8_kinematicsInstanceStruct
  *chartInstance);
static real_T c8_eml_xnrm2(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_x[144], int32_T c8_ix0);
static void c8_e_eml_int_forloop_overflow_check(SFc8_kinematicsInstanceStruct
  *chartInstance, int32_T c8_b_a, int32_T c8_b);
static void c8_eml_xscal(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_b_a, real_T c8_x[144], int32_T c8_ix0, real_T c8_b_x[144]);
static real_T c8_eml_xdotc(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_x[144], int32_T c8_ix0, real_T c8_y[144], int32_T c8_iy0);
static void c8_eml_xaxpy(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_b_a, int32_T c8_ix0, real_T c8_y[144], int32_T c8_iy0, real_T
  c8_b_y[144]);
static real_T c8_b_eml_xnrm2(SFc8_kinematicsInstanceStruct *chartInstance,
  int32_T c8_n, real_T c8_x[12], int32_T c8_ix0);
static void c8_b_eml_xscal(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_b_a, real_T c8_x[12], int32_T c8_ix0, real_T c8_b_x[12]);
static void c8_b_eml_xaxpy(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_b_a, real_T c8_x[144], int32_T c8_ix0, real_T c8_y[12],
  int32_T c8_iy0, real_T c8_b_y[12]);
static void c8_c_eml_xaxpy(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_b_a, real_T c8_x[12], int32_T c8_ix0, real_T c8_y[144],
  int32_T c8_iy0, real_T c8_b_y[144]);
static void c8_eps(SFc8_kinematicsInstanceStruct *chartInstance);
static void c8_b_eml_error(SFc8_kinematicsInstanceStruct *chartInstance);
static void c8_f_eml_int_forloop_overflow_check(SFc8_kinematicsInstanceStruct
  *chartInstance, int32_T c8_b_a);
static void c8_g_eml_int_forloop_overflow_check(SFc8_kinematicsInstanceStruct
  *chartInstance, int32_T c8_b_a, int32_T c8_b);
static real_T c8_sqrt(SFc8_kinematicsInstanceStruct *chartInstance, real_T c8_x);
static void c8_c_eml_error(SFc8_kinematicsInstanceStruct *chartInstance);
static void c8_eml_xrotg(SFc8_kinematicsInstanceStruct *chartInstance, real_T
  c8_b_a, real_T c8_b, real_T *c8_c_a, real_T *c8_b_b, real_T *c8_c, real_T
  *c8_s);
static void c8_f_eml_scalar_eg(SFc8_kinematicsInstanceStruct *chartInstance);
static const mxArray *c8_j_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static int32_T c8_i_emlrt_marshallIn(SFc8_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void c8_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static uint8_T c8_j_emlrt_marshallIn(SFc8_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_b_is_active_c8_kinematics, const char_T
  *c8_identifier);
static uint8_T c8_k_emlrt_marshallIn(SFc8_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void c8_b_eml_xgemm(SFc8_kinematicsInstanceStruct *chartInstance, real_T
  c8_A[72], real_T c8_B[72], real_T c8_C[144]);
static void c8_c_eml_xscal(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_b_a, real_T c8_x[144], int32_T c8_ix0);
static void c8_d_eml_xaxpy(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_b_a, int32_T c8_ix0, real_T c8_y[144], int32_T c8_iy0);
static void c8_d_eml_xscal(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_b_a, real_T c8_x[12], int32_T c8_ix0);
static void c8_e_eml_xaxpy(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_b_a, real_T c8_x[144], int32_T c8_ix0, real_T c8_y[12],
  int32_T c8_iy0);
static void c8_f_eml_xaxpy(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_b_a, real_T c8_x[12], int32_T c8_ix0, real_T c8_y[144],
  int32_T c8_iy0);
static void c8_b_sqrt(SFc8_kinematicsInstanceStruct *chartInstance, real_T *c8_x);
static void c8_b_eml_xrotg(SFc8_kinematicsInstanceStruct *chartInstance, real_T *
  c8_b_a, real_T *c8_b, real_T *c8_c, real_T *c8_s);
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
  uint8_T c8_hoistedGlobal;
  uint8_T c8_b_u;
  const mxArray *c8_c_y = NULL;
  real_T (*c8_vec)[12];
  c8_vec = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 1);
  c8_st = NULL;
  c8_st = NULL;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_createcellarray(2), FALSE);
  for (c8_i5 = 0; c8_i5 < 12; c8_i5++) {
    c8_u[c8_i5] = (*c8_vec)[c8_i5];
  }

  c8_b_y = NULL;
  sf_mex_assign(&c8_b_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 1, 12), FALSE);
  sf_mex_setcell(c8_y, 0, c8_b_y);
  c8_hoistedGlobal = chartInstance->c8_is_active_c8_kinematics;
  c8_b_u = c8_hoistedGlobal;
  c8_c_y = NULL;
  sf_mex_assign(&c8_c_y, sf_mex_create("y", &c8_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c8_y, 1, c8_c_y);
  sf_mex_assign(&c8_st, c8_y, FALSE);
  return c8_st;
}

static void set_sim_state_c8_kinematics(SFc8_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_st)
{
  const mxArray *c8_u;
  real_T c8_dv5[12];
  int32_T c8_i6;
  real_T (*c8_vec)[12];
  c8_vec = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c8_doneDoubleBufferReInit = TRUE;
  c8_u = sf_mex_dup(c8_st);
  c8_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c8_u, 0)), "vec",
                      c8_dv5);
  for (c8_i6 = 0; c8_i6 < 12; c8_i6++) {
    (*c8_vec)[c8_i6] = c8_dv5[c8_i6];
  }

  chartInstance->c8_is_active_c8_kinematics = c8_j_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c8_u, 1)),
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
  real_T (*c8_ee_eta1)[3];
  real_T (*c8_eta2)[3];
  real_T (*c8_eta1)[3];
  real_T (*c8_q)[6];
  real_T (*c8_vec)[12];
  real_T (*c8_J_inv)[72];
  real_T (*c8_J)[72];
  real_T (*c8_v_ee)[6];
  c8_ee_eta1 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 6);
  c8_eta2 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 5);
  c8_eta1 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 4);
  c8_q = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 3);
  c8_vec = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 1);
  c8_J_inv = (real_T (*)[72])ssGetInputPortSignal(chartInstance->S, 2);
  c8_J = (real_T (*)[72])ssGetInputPortSignal(chartInstance->S, 1);
  c8_v_ee = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 6U, chartInstance->c8_sfEvent);
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
  uint32_T c8_debug_family_var_map[23];
  real_T c8_ee_pos[3];
  real_T c8_h_q[6];
  real_T c8_h_phi;
  real_T c8_h_theta;
  real_T c8_h_x;
  real_T c8_h_z;
  real_T c8_I[144];
  real_T c8_k;
  real_T c8_nargin = 12.0;
  real_T c8_nargout = 1.0;
  real_T c8_vec[12];
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
  real_T c8_x;
  real_T c8_h_y;
  real_T c8_b_x;
  real_T c8_i_y;
  real_T c8_j_y;
  real_T c8_g_a;
  real_T c8_h_b;
  real_T c8_k_y;
  real_T c8_b_A;
  real_T c8_b_B;
  real_T c8_c_x;
  real_T c8_l_y;
  real_T c8_d_x;
  real_T c8_m_y;
  real_T c8_n_y;
  int32_T c8_i38;
  static real_T c8_dv7[144] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T c8_i39;
  int32_T c8_i40;
  int32_T c8_i41;
  real_T c8_h_a[72];
  int32_T c8_i42;
  real_T c8_i_b[6];
  int32_T c8_i43;
  real_T c8_o_y[12];
  int32_T c8_i44;
  int32_T c8_i45;
  int32_T c8_i46;
  int32_T c8_i47;
  real_T c8_j_b[72];
  int32_T c8_i48;
  real_T c8_p_y[144];
  int32_T c8_i49;
  real_T c8_i_a[72];
  int32_T c8_i50;
  real_T c8_k_b[72];
  int32_T c8_i51;
  int32_T c8_i52;
  int32_T c8_i53;
  real_T c8_q_y[144];
  int32_T c8_i54;
  real_T c8_j_a[72];
  int32_T c8_i55;
  real_T c8_l_b[72];
  int32_T c8_i56;
  int32_T c8_i57;
  real_T c8_m_b[12];
  int32_T c8_i58;
  real_T c8_r_y[12];
  int32_T c8_i59;
  int32_T c8_i60;
  int32_T c8_i61;
  real_T c8_b_I[144];
  real_T c8_k_a;
  int32_T c8_i62;
  real_T c8_s_y[12];
  real_T c8_n_b;
  real_T c8_t_y;
  int32_T c8_i63;
  real_T c8_u_y[12];
  real_T c8_c_A;
  real_T c8_c_B;
  real_T c8_e_x;
  real_T c8_v_y;
  real_T c8_f_x;
  real_T c8_w_y;
  int32_T c8_i64;
  int32_T c8_i65;
  real_T (*c8_b_vec)[12];
  real_T (*c8_b_ee_eta1)[3];
  real_T (*c8_b_eta2)[3];
  real_T (*c8_b_eta1)[3];
  real_T (*c8_c_q)[6];
  real_T (*c8_b_J_inv)[72];
  real_T (*c8_b_J)[72];
  real_T (*c8_b_v_ee)[6];
  c8_b_ee_eta1 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 6);
  c8_b_eta2 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 5);
  c8_b_eta1 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 4);
  c8_c_q = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 3);
  c8_b_vec = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 1);
  c8_b_J_inv = (real_T (*)[72])ssGetInputPortSignal(chartInstance->S, 2);
  c8_b_J = (real_T (*)[72])ssGetInputPortSignal(chartInstance->S, 1);
  c8_b_v_ee = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 6U, chartInstance->c8_sfEvent);
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
    c8_q[c8_i23] = (*c8_c_q)[c8_i23];
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

  sf_debug_symbol_scope_push_eml(0U, 23U, 23U, c8_debug_family_names,
    c8_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c8_ee_pos, 0U, c8_c_sf_marshallOut,
    c8_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_h_q, 1U, c8_b_sf_marshallOut,
    c8_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_h_phi, 2U, c8_f_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_h_theta, 3U, c8_f_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_h_x, 4U, c8_f_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_h_z, 5U, c8_f_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_I, 6U, c8_g_sf_marshallOut,
    c8_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_k, 7U, c8_f_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_nargin, 8U, c8_f_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_nargout, 9U, c8_f_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c8_v_ee, 10U, c8_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c8_J, 11U, c8_e_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c8_J_inv, 12U, c8_d_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c8_q, 13U, c8_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c8_eta1, 14U, c8_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c8_eta2, 15U, c8_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c8_ee_eta1, 16U, c8_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c8_b_qmin, 17U, c8_b_sf_marshallOut,
    c8_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_b_qmax, 18U, c8_b_sf_marshallOut,
    c8_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_b_d, 19U, c8_b_sf_marshallOut,
    c8_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_b_a, 20U, c8_b_sf_marshallOut,
    c8_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_b_alpha, 21U, c8_b_sf_marshallOut,
    c8_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_vec, 22U, c8_sf_marshallOut,
    c8_sf_marshallIn);
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
    c8_x = c8_A;
    c8_h_y = c8_B;
    c8_b_x = c8_x;
    c8_i_y = c8_h_y;
    c8_j_y = c8_b_x / c8_i_y;
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
    c8_c_x = c8_b_A;
    c8_l_y = c8_b_B;
    c8_d_x = c8_c_x;
    c8_m_y = c8_l_y;
    c8_n_y = c8_d_x / c8_m_y;
    c8_h_q[_SFD_EML_ARRAY_BOUNDS_CHECK("h_q", (int32_T)_SFD_INTEGER_CHECK("i",
      c8_b_i), 1, 6, 1, 0) - 1] = c8_j_y + c8_n_y;
    c8_i++;
    sf_mex_listen_for_ctrl_c(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 0, 0);
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 19);
  for (c8_i38 = 0; c8_i38 < 144; c8_i38++) {
    c8_I[c8_i38] = c8_dv7[c8_i38];
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 21);
  for (c8_i39 = 0; c8_i39 < 12; c8_i39++) {
    c8_vec[c8_i39] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 22);
  for (c8_i40 = 0; c8_i40 < 6; c8_i40++) {
    c8_vec[c8_i40 + 6] = c8_h_q[c8_i40];
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 23);
  c8_vec[3] = c8_h_phi;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 24);
  c8_vec[0] = c8_h_x;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 25);
  c8_vec[4] = c8_h_theta;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 26);
  for (c8_i41 = 0; c8_i41 < 72; c8_i41++) {
    c8_h_a[c8_i41] = c8_J_inv[c8_i41];
  }

  for (c8_i42 = 0; c8_i42 < 6; c8_i42++) {
    c8_i_b[c8_i42] = c8_v_ee[c8_i42];
  }

  c8_c_eml_scalar_eg(chartInstance);
  c8_c_eml_scalar_eg(chartInstance);
  for (c8_i43 = 0; c8_i43 < 12; c8_i43++) {
    c8_o_y[c8_i43] = 0.0;
    c8_i44 = 0;
    for (c8_i45 = 0; c8_i45 < 6; c8_i45++) {
      c8_o_y[c8_i43] += c8_h_a[c8_i44 + c8_i43] * c8_i_b[c8_i45];
      c8_i44 += 12;
    }
  }

  for (c8_i46 = 0; c8_i46 < 72; c8_i46++) {
    c8_h_a[c8_i46] = c8_J_inv[c8_i46];
  }

  for (c8_i47 = 0; c8_i47 < 72; c8_i47++) {
    c8_j_b[c8_i47] = c8_J[c8_i47];
  }

  c8_d_eml_scalar_eg(chartInstance);
  c8_d_eml_scalar_eg(chartInstance);
  for (c8_i48 = 0; c8_i48 < 144; c8_i48++) {
    c8_p_y[c8_i48] = 0.0;
  }

  for (c8_i49 = 0; c8_i49 < 72; c8_i49++) {
    c8_i_a[c8_i49] = c8_h_a[c8_i49];
  }

  for (c8_i50 = 0; c8_i50 < 72; c8_i50++) {
    c8_k_b[c8_i50] = c8_j_b[c8_i50];
  }

  c8_b_eml_xgemm(chartInstance, c8_i_a, c8_k_b, c8_p_y);
  for (c8_i51 = 0; c8_i51 < 72; c8_i51++) {
    c8_h_a[c8_i51] = c8_J_inv[c8_i51];
  }

  for (c8_i52 = 0; c8_i52 < 72; c8_i52++) {
    c8_j_b[c8_i52] = c8_J[c8_i52];
  }

  c8_d_eml_scalar_eg(chartInstance);
  c8_d_eml_scalar_eg(chartInstance);
  for (c8_i53 = 0; c8_i53 < 144; c8_i53++) {
    c8_q_y[c8_i53] = 0.0;
  }

  for (c8_i54 = 0; c8_i54 < 72; c8_i54++) {
    c8_j_a[c8_i54] = c8_h_a[c8_i54];
  }

  for (c8_i55 = 0; c8_i55 < 72; c8_i55++) {
    c8_l_b[c8_i55] = c8_j_b[c8_i55];
  }

  c8_b_eml_xgemm(chartInstance, c8_j_a, c8_l_b, c8_q_y);
  for (c8_i56 = 0; c8_i56 < 144; c8_i56++) {
    c8_q_y[c8_i56] = c8_I[c8_i56] - c8_q_y[c8_i56];
  }

  for (c8_i57 = 0; c8_i57 < 12; c8_i57++) {
    c8_m_b[c8_i57] = c8_vec[c8_i57];
  }

  c8_f_eml_scalar_eg(chartInstance);
  c8_f_eml_scalar_eg(chartInstance);
  for (c8_i58 = 0; c8_i58 < 12; c8_i58++) {
    c8_r_y[c8_i58] = 0.0;
    c8_i59 = 0;
    for (c8_i60 = 0; c8_i60 < 12; c8_i60++) {
      c8_r_y[c8_i58] += c8_q_y[c8_i59 + c8_i58] * c8_m_b[c8_i60];
      c8_i59 += 12;
    }
  }

  for (c8_i61 = 0; c8_i61 < 144; c8_i61++) {
    c8_b_I[c8_i61] = c8_I[c8_i61] - c8_p_y[c8_i61];
  }

  c8_k_a = c8_b_norm(chartInstance, c8_b_I);
  for (c8_i62 = 0; c8_i62 < 12; c8_i62++) {
    c8_s_y[c8_i62] = c8_r_y[c8_i62];
  }

  c8_n_b = c8_norm(chartInstance, c8_s_y);
  c8_t_y = c8_k_a * c8_n_b;
  for (c8_i63 = 0; c8_i63 < 12; c8_i63++) {
    c8_u_y[c8_i63] = c8_o_y[c8_i63];
  }

  c8_c_A = -(2.0 - c8_norm(chartInstance, c8_u_y));
  c8_c_B = c8_t_y;
  c8_e_x = c8_c_A;
  c8_v_y = c8_c_B;
  c8_f_x = c8_e_x;
  c8_w_y = c8_v_y;
  c8_k = c8_f_x / c8_w_y;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 27);
  for (c8_i64 = 0; c8_i64 < 12; c8_i64++) {
    c8_vec[c8_i64] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 28);
  c8_vec[0] = 0.4;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, -28);
  sf_debug_symbol_scope_pop();
  for (c8_i65 = 0; c8_i65 < 12; c8_i65++) {
    (*c8_b_vec)[c8_i65] = c8_vec[c8_i65];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 6U, chartInstance->c8_sfEvent);
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
  int32_T c8_i66;
  real_T c8_b_inData[12];
  int32_T c8_i67;
  real_T c8_u[12];
  const mxArray *c8_y = NULL;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  for (c8_i66 = 0; c8_i66 < 12; c8_i66++) {
    c8_b_inData[c8_i66] = (*(real_T (*)[12])c8_inData)[c8_i66];
  }

  for (c8_i67 = 0; c8_i67 < 12; c8_i67++) {
    c8_u[c8_i67] = c8_b_inData[c8_i67];
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 1, 12), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static void c8_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_vec, const char_T *c8_identifier, real_T c8_y[12])
{
  emlrtMsgIdentifier c8_thisId;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_vec), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_vec);
}

static void c8_b_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[12])
{
  real_T c8_dv8[12];
  int32_T c8_i68;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv8, 1, 0, 0U, 1, 0U, 1, 12);
  for (c8_i68 = 0; c8_i68 < 12; c8_i68++) {
    c8_y[c8_i68] = c8_dv8[c8_i68];
  }

  sf_mex_destroy(&c8_u);
}

static void c8_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_vec;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y[12];
  int32_T c8_i69;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_vec = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_vec), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_vec);
  for (c8_i69 = 0; c8_i69 < 12; c8_i69++) {
    (*(real_T (*)[12])c8_outData)[c8_i69] = c8_y[c8_i69];
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

static const mxArray *c8_b_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i70;
  real_T c8_b_inData[6];
  int32_T c8_i71;
  real_T c8_u[6];
  const mxArray *c8_y = NULL;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  for (c8_i70 = 0; c8_i70 < 6; c8_i70++) {
    c8_b_inData[c8_i70] = (*(real_T (*)[6])c8_inData)[c8_i70];
  }

  for (c8_i71 = 0; c8_i71 < 6; c8_i71++) {
    c8_u[c8_i71] = c8_b_inData[c8_i71];
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static void c8_c_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[6])
{
  real_T c8_dv9[6];
  int32_T c8_i72;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv9, 1, 0, 0U, 1, 0U, 1, 6);
  for (c8_i72 = 0; c8_i72 < 6; c8_i72++) {
    c8_y[c8_i72] = c8_dv9[c8_i72];
  }

  sf_mex_destroy(&c8_u);
}

static void c8_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_b_alpha;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y[6];
  int32_T c8_i73;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_b_alpha = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_b_alpha), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_b_alpha);
  for (c8_i73 = 0; c8_i73 < 6; c8_i73++) {
    (*(real_T (*)[6])c8_outData)[c8_i73] = c8_y[c8_i73];
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

static const mxArray *c8_c_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i74;
  real_T c8_b_inData[3];
  int32_T c8_i75;
  real_T c8_u[3];
  const mxArray *c8_y = NULL;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  for (c8_i74 = 0; c8_i74 < 3; c8_i74++) {
    c8_b_inData[c8_i74] = (*(real_T (*)[3])c8_inData)[c8_i74];
  }

  for (c8_i75 = 0; c8_i75 < 3; c8_i75++) {
    c8_u[c8_i75] = c8_b_inData[c8_i75];
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static const mxArray *c8_d_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i76;
  int32_T c8_i77;
  int32_T c8_i78;
  real_T c8_b_inData[72];
  int32_T c8_i79;
  int32_T c8_i80;
  int32_T c8_i81;
  real_T c8_u[72];
  const mxArray *c8_y = NULL;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_i76 = 0;
  for (c8_i77 = 0; c8_i77 < 6; c8_i77++) {
    for (c8_i78 = 0; c8_i78 < 12; c8_i78++) {
      c8_b_inData[c8_i78 + c8_i76] = (*(real_T (*)[72])c8_inData)[c8_i78 +
        c8_i76];
    }

    c8_i76 += 12;
  }

  c8_i79 = 0;
  for (c8_i80 = 0; c8_i80 < 6; c8_i80++) {
    for (c8_i81 = 0; c8_i81 < 12; c8_i81++) {
      c8_u[c8_i81 + c8_i79] = c8_b_inData[c8_i81 + c8_i79];
    }

    c8_i79 += 12;
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 2, 12, 6), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static const mxArray *c8_e_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i82;
  int32_T c8_i83;
  int32_T c8_i84;
  real_T c8_b_inData[72];
  int32_T c8_i85;
  int32_T c8_i86;
  int32_T c8_i87;
  real_T c8_u[72];
  const mxArray *c8_y = NULL;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_i82 = 0;
  for (c8_i83 = 0; c8_i83 < 12; c8_i83++) {
    for (c8_i84 = 0; c8_i84 < 6; c8_i84++) {
      c8_b_inData[c8_i84 + c8_i82] = (*(real_T (*)[72])c8_inData)[c8_i84 +
        c8_i82];
    }

    c8_i82 += 6;
  }

  c8_i85 = 0;
  for (c8_i86 = 0; c8_i86 < 12; c8_i86++) {
    for (c8_i87 = 0; c8_i87 < 6; c8_i87++) {
      c8_u[c8_i87 + c8_i85] = c8_b_inData[c8_i87 + c8_i85];
    }

    c8_i85 += 6;
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 2, 6, 12), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static const mxArray *c8_f_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
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

static real_T c8_d_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  real_T c8_y;
  real_T c8_d0;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_d0, 1, 0, 0U, 0, 0U, 0);
  c8_y = c8_d0;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void c8_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_nargout;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_nargout = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_nargout), &c8_thisId);
  sf_mex_destroy(&c8_nargout);
  *(real_T *)c8_outData = c8_y;
  sf_mex_destroy(&c8_mxArrayInData);
}

static const mxArray *c8_g_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i88;
  int32_T c8_i89;
  int32_T c8_i90;
  real_T c8_b_inData[144];
  int32_T c8_i91;
  int32_T c8_i92;
  int32_T c8_i93;
  real_T c8_u[144];
  const mxArray *c8_y = NULL;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_i88 = 0;
  for (c8_i89 = 0; c8_i89 < 12; c8_i89++) {
    for (c8_i90 = 0; c8_i90 < 12; c8_i90++) {
      c8_b_inData[c8_i90 + c8_i88] = (*(real_T (*)[144])c8_inData)[c8_i90 +
        c8_i88];
    }

    c8_i88 += 12;
  }

  c8_i91 = 0;
  for (c8_i92 = 0; c8_i92 < 12; c8_i92++) {
    for (c8_i93 = 0; c8_i93 < 12; c8_i93++) {
      c8_u[c8_i93 + c8_i91] = c8_b_inData[c8_i93 + c8_i91];
    }

    c8_i91 += 12;
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 2, 12, 12), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static void c8_e_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[144])
{
  real_T c8_dv10[144];
  int32_T c8_i94;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv10, 1, 0, 0U, 1, 0U, 2, 12,
                12);
  for (c8_i94 = 0; c8_i94 < 144; c8_i94++) {
    c8_y[c8_i94] = c8_dv10[c8_i94];
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
  int32_T c8_i95;
  int32_T c8_i96;
  int32_T c8_i97;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_I = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_I), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_I);
  c8_i95 = 0;
  for (c8_i96 = 0; c8_i96 < 12; c8_i96++) {
    for (c8_i97 = 0; c8_i97 < 12; c8_i97++) {
      (*(real_T (*)[144])c8_outData)[c8_i97 + c8_i95] = c8_y[c8_i97 + c8_i95];
    }

    c8_i95 += 12;
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

static void c8_f_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[3])
{
  real_T c8_dv11[3];
  int32_T c8_i98;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv11, 1, 0, 0U, 1, 0U, 1, 3);
  for (c8_i98 = 0; c8_i98 < 3; c8_i98++) {
    c8_y[c8_i98] = c8_dv11[c8_i98];
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
  int32_T c8_i99;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_ee_pos = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_ee_pos), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_ee_pos);
  for (c8_i99 = 0; c8_i99 < 3; c8_i99++) {
    (*(real_T (*)[3])c8_outData)[c8_i99] = c8_y[c8_i99];
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

static const mxArray *c8_h_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i100;
  int32_T c8_i101;
  int32_T c8_i102;
  real_T c8_b_inData[16];
  int32_T c8_i103;
  int32_T c8_i104;
  int32_T c8_i105;
  real_T c8_u[16];
  const mxArray *c8_y = NULL;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_i100 = 0;
  for (c8_i101 = 0; c8_i101 < 4; c8_i101++) {
    for (c8_i102 = 0; c8_i102 < 4; c8_i102++) {
      c8_b_inData[c8_i102 + c8_i100] = (*(real_T (*)[16])c8_inData)[c8_i102 +
        c8_i100];
    }

    c8_i100 += 4;
  }

  c8_i103 = 0;
  for (c8_i104 = 0; c8_i104 < 4; c8_i104++) {
    for (c8_i105 = 0; c8_i105 < 4; c8_i105++) {
      c8_u[c8_i105 + c8_i103] = c8_b_inData[c8_i105 + c8_i103];
    }

    c8_i103 += 4;
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static void c8_g_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[16])
{
  real_T c8_dv12[16];
  int32_T c8_i106;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv12, 1, 0, 0U, 1, 0U, 2, 4, 4);
  for (c8_i106 = 0; c8_i106 < 16; c8_i106++) {
    c8_y[c8_i106] = c8_dv12[c8_i106];
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
  int32_T c8_i107;
  int32_T c8_i108;
  int32_T c8_i109;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mat = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_mat), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_mat);
  c8_i107 = 0;
  for (c8_i108 = 0; c8_i108 < 4; c8_i108++) {
    for (c8_i109 = 0; c8_i109 < 4; c8_i109++) {
      (*(real_T (*)[16])c8_outData)[c8_i109 + c8_i107] = c8_y[c8_i109 + c8_i107];
    }

    c8_i107 += 4;
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

static const mxArray *c8_i_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i110;
  int32_T c8_i111;
  int32_T c8_i112;
  real_T c8_b_inData[9];
  int32_T c8_i113;
  int32_T c8_i114;
  int32_T c8_i115;
  real_T c8_u[9];
  const mxArray *c8_y = NULL;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_i110 = 0;
  for (c8_i111 = 0; c8_i111 < 3; c8_i111++) {
    for (c8_i112 = 0; c8_i112 < 3; c8_i112++) {
      c8_b_inData[c8_i112 + c8_i110] = (*(real_T (*)[9])c8_inData)[c8_i112 +
        c8_i110];
    }

    c8_i110 += 3;
  }

  c8_i113 = 0;
  for (c8_i114 = 0; c8_i114 < 3; c8_i114++) {
    for (c8_i115 = 0; c8_i115 < 3; c8_i115++) {
      c8_u[c8_i115 + c8_i113] = c8_b_inData[c8_i115 + c8_i113];
    }

    c8_i113 += 3;
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 2, 3, 3), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static void c8_h_emlrt_marshallIn(SFc8_kinematicsInstanceStruct *chartInstance,
  const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId, real_T c8_y[9])
{
  real_T c8_dv13[9];
  int32_T c8_i116;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv13, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c8_i116 = 0; c8_i116 < 9; c8_i116++) {
    c8_y[c8_i116] = c8_dv13[c8_i116];
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
  int32_T c8_i117;
  int32_T c8_i118;
  int32_T c8_i119;
  SFc8_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_R = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_R), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_R);
  c8_i117 = 0;
  for (c8_i118 = 0; c8_i118 < 3; c8_i118++) {
    for (c8_i119 = 0; c8_i119 < 3; c8_i119++) {
      (*(real_T (*)[9])c8_outData)[c8_i119 + c8_i117] = c8_y[c8_i119 + c8_i117];
    }

    c8_i117 += 3;
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

const mxArray *sf_c8_kinematics_get_eml_resolved_functions_info(void)
{
  const mxArray *c8_nameCaptureInfo;
  c8_ResolvedFunctionInfo c8_info[153];
  const mxArray *c8_m0 = NULL;
  int32_T c8_i120;
  c8_ResolvedFunctionInfo *c8_r0;
  c8_nameCaptureInfo = NULL;
  c8_nameCaptureInfo = NULL;
  c8_info_helper(c8_info);
  c8_b_info_helper(c8_info);
  c8_c_info_helper(c8_info);
  sf_mex_assign(&c8_m0, sf_mex_createstruct("nameCaptureInfo", 1, 153), FALSE);
  for (c8_i120 = 0; c8_i120 < 153; c8_i120++) {
    c8_r0 = &c8_info[c8_i120];
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", c8_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c8_r0->context)), "context", "nameCaptureInfo",
                    c8_i120);
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", c8_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c8_r0->name)), "name", "nameCaptureInfo", c8_i120);
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", c8_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c8_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c8_i120);
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", c8_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c8_r0->resolved)), "resolved", "nameCaptureInfo",
                    c8_i120);
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", &c8_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c8_i120);
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", &c8_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c8_i120);
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", &c8_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c8_i120);
    sf_mex_addfield(c8_m0, sf_mex_create("nameCaptureInfo", &c8_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c8_i120);
  }

  sf_mex_assign(&c8_nameCaptureInfo, c8_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c8_nameCaptureInfo);
  return c8_nameCaptureInfo;
}

static void c8_info_helper(c8_ResolvedFunctionInfo c8_info[153])
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
  c8_info[45].context = "";
  c8_info[45].name = "norm";
  c8_info[45].dominantType = "double";
  c8_info[45].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/norm.m";
  c8_info[45].fileTimeLo = 1286818826U;
  c8_info[45].fileTimeHi = 0U;
  c8_info[45].mFileTimeLo = 0U;
  c8_info[45].mFileTimeHi = 0U;
  c8_info[46].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm";
  c8_info[46].name = "eml_index_class";
  c8_info[46].dominantType = "";
  c8_info[46].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[46].fileTimeLo = 1286818778U;
  c8_info[46].fileTimeHi = 0U;
  c8_info[46].mFileTimeLo = 0U;
  c8_info[46].mFileTimeHi = 0U;
  c8_info[47].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm";
  c8_info[47].name = "eml_xnrm2";
  c8_info[47].dominantType = "int32";
  c8_info[47].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c8_info[47].fileTimeLo = 1299076776U;
  c8_info[47].fileTimeHi = 0U;
  c8_info[47].mFileTimeLo = 0U;
  c8_info[47].mFileTimeHi = 0U;
  c8_info[48].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c8_info[48].name = "eml_blas_inline";
  c8_info[48].dominantType = "";
  c8_info[48].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c8_info[48].fileTimeLo = 1299076768U;
  c8_info[48].fileTimeHi = 0U;
  c8_info[48].mFileTimeLo = 0U;
  c8_info[48].mFileTimeHi = 0U;
  c8_info[49].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m";
  c8_info[49].name = "eml_refblas_xnrm2";
  c8_info[49].dominantType = "int32";
  c8_info[49].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c8_info[49].fileTimeLo = 1299076784U;
  c8_info[49].fileTimeHi = 0U;
  c8_info[49].mFileTimeLo = 0U;
  c8_info[49].mFileTimeHi = 0U;
  c8_info[50].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c8_info[50].name = "realmin";
  c8_info[50].dominantType = "char";
  c8_info[50].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/realmin.m";
  c8_info[50].fileTimeLo = 1307651242U;
  c8_info[50].fileTimeHi = 0U;
  c8_info[50].mFileTimeLo = 0U;
  c8_info[50].mFileTimeHi = 0U;
  c8_info[51].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/realmin.m";
  c8_info[51].name = "eml_realmin";
  c8_info[51].dominantType = "char";
  c8_info[51].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c8_info[51].fileTimeLo = 1307651244U;
  c8_info[51].fileTimeHi = 0U;
  c8_info[51].mFileTimeLo = 0U;
  c8_info[51].mFileTimeHi = 0U;
  c8_info[52].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c8_info[52].name = "eml_float_model";
  c8_info[52].dominantType = "char";
  c8_info[52].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c8_info[52].fileTimeLo = 1307651242U;
  c8_info[52].fileTimeHi = 0U;
  c8_info[52].mFileTimeLo = 0U;
  c8_info[52].mFileTimeHi = 0U;
  c8_info[53].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c8_info[53].name = "eml_index_class";
  c8_info[53].dominantType = "";
  c8_info[53].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[53].fileTimeLo = 1286818778U;
  c8_info[53].fileTimeHi = 0U;
  c8_info[53].mFileTimeLo = 0U;
  c8_info[53].mFileTimeHi = 0U;
  c8_info[54].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c8_info[54].name = "eml_index_minus";
  c8_info[54].dominantType = "int32";
  c8_info[54].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c8_info[54].fileTimeLo = 1286818778U;
  c8_info[54].fileTimeHi = 0U;
  c8_info[54].mFileTimeLo = 0U;
  c8_info[54].mFileTimeHi = 0U;
  c8_info[55].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c8_info[55].name = "eml_index_class";
  c8_info[55].dominantType = "";
  c8_info[55].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[55].fileTimeLo = 1286818778U;
  c8_info[55].fileTimeHi = 0U;
  c8_info[55].mFileTimeLo = 0U;
  c8_info[55].mFileTimeHi = 0U;
  c8_info[56].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c8_info[56].name = "eml_index_times";
  c8_info[56].dominantType = "int32";
  c8_info[56].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c8_info[56].fileTimeLo = 1286818780U;
  c8_info[56].fileTimeHi = 0U;
  c8_info[56].mFileTimeLo = 0U;
  c8_info[56].mFileTimeHi = 0U;
  c8_info[57].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c8_info[57].name = "eml_index_class";
  c8_info[57].dominantType = "";
  c8_info[57].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[57].fileTimeLo = 1286818778U;
  c8_info[57].fileTimeHi = 0U;
  c8_info[57].mFileTimeLo = 0U;
  c8_info[57].mFileTimeHi = 0U;
  c8_info[58].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c8_info[58].name = "eml_index_plus";
  c8_info[58].dominantType = "int32";
  c8_info[58].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c8_info[58].fileTimeLo = 1286818778U;
  c8_info[58].fileTimeHi = 0U;
  c8_info[58].mFileTimeLo = 0U;
  c8_info[58].mFileTimeHi = 0U;
  c8_info[59].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c8_info[59].name = "eml_index_class";
  c8_info[59].dominantType = "";
  c8_info[59].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[59].fileTimeLo = 1286818778U;
  c8_info[59].fileTimeHi = 0U;
  c8_info[59].mFileTimeLo = 0U;
  c8_info[59].mFileTimeHi = 0U;
  c8_info[60].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c8_info[60].name = "eml_int_forloop_overflow_check";
  c8_info[60].dominantType = "";
  c8_info[60].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c8_info[60].fileTimeLo = 1311255316U;
  c8_info[60].fileTimeHi = 0U;
  c8_info[60].mFileTimeLo = 0U;
  c8_info[60].mFileTimeHi = 0U;
  c8_info[61].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c8_info[61].name = "abs";
  c8_info[61].dominantType = "double";
  c8_info[61].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c8_info[61].fileTimeLo = 1286818694U;
  c8_info[61].fileTimeHi = 0U;
  c8_info[61].mFileTimeLo = 0U;
  c8_info[61].mFileTimeHi = 0U;
  c8_info[62].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c8_info[62].name = "eml_scalar_abs";
  c8_info[62].dominantType = "double";
  c8_info[62].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c8_info[62].fileTimeLo = 1286818712U;
  c8_info[62].fileTimeHi = 0U;
  c8_info[62].mFileTimeLo = 0U;
  c8_info[62].mFileTimeHi = 0U;
  c8_info[63].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c8_info[63].name = "eml_index_class";
  c8_info[63].dominantType = "";
  c8_info[63].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[63].fileTimeLo = 1286818778U;
  c8_info[63].fileTimeHi = 0U;
  c8_info[63].mFileTimeLo = 0U;
  c8_info[63].mFileTimeHi = 0U;
}

static void c8_b_info_helper(c8_ResolvedFunctionInfo c8_info[153])
{
  c8_info[64].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/norm.m";
  c8_info[64].name = "svd";
  c8_info[64].dominantType = "double";
  c8_info[64].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/svd.m";
  c8_info[64].fileTimeLo = 1286818832U;
  c8_info[64].fileTimeHi = 0U;
  c8_info[64].mFileTimeLo = 0U;
  c8_info[64].mFileTimeHi = 0U;
  c8_info[65].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/svd.m";
  c8_info[65].name = "eml_index_class";
  c8_info[65].dominantType = "";
  c8_info[65].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[65].fileTimeLo = 1286818778U;
  c8_info[65].fileTimeHi = 0U;
  c8_info[65].mFileTimeLo = 0U;
  c8_info[65].mFileTimeHi = 0U;
  c8_info[66].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/svd.m";
  c8_info[66].name = "eml_int_forloop_overflow_check";
  c8_info[66].dominantType = "";
  c8_info[66].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c8_info[66].fileTimeLo = 1311255316U;
  c8_info[66].fileTimeHi = 0U;
  c8_info[66].mFileTimeLo = 0U;
  c8_info[66].mFileTimeHi = 0U;
  c8_info[67].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/svd.m";
  c8_info[67].name = "isfinite";
  c8_info[67].dominantType = "double";
  c8_info[67].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c8_info[67].fileTimeLo = 1286818758U;
  c8_info[67].fileTimeHi = 0U;
  c8_info[67].mFileTimeLo = 0U;
  c8_info[67].mFileTimeHi = 0U;
  c8_info[68].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c8_info[68].name = "isinf";
  c8_info[68].dominantType = "double";
  c8_info[68].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isinf.m";
  c8_info[68].fileTimeLo = 1286818760U;
  c8_info[68].fileTimeHi = 0U;
  c8_info[68].mFileTimeLo = 0U;
  c8_info[68].mFileTimeHi = 0U;
  c8_info[69].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c8_info[69].name = "isnan";
  c8_info[69].dominantType = "double";
  c8_info[69].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isnan.m";
  c8_info[69].fileTimeLo = 1286818760U;
  c8_info[69].fileTimeHi = 0U;
  c8_info[69].mFileTimeLo = 0U;
  c8_info[69].mFileTimeHi = 0U;
  c8_info[70].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/svd.m";
  c8_info[70].name = "eml_error";
  c8_info[70].dominantType = "char";
  c8_info[70].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_error.m";
  c8_info[70].fileTimeLo = 1305318000U;
  c8_info[70].fileTimeHi = 0U;
  c8_info[70].mFileTimeLo = 0U;
  c8_info[70].mFileTimeHi = 0U;
  c8_info[71].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/svd.m";
  c8_info[71].name = "eml_xgesvd";
  c8_info[71].dominantType = "double";
  c8_info[71].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/eml_xgesvd.m";
  c8_info[71].fileTimeLo = 1286818806U;
  c8_info[71].fileTimeHi = 0U;
  c8_info[71].mFileTimeLo = 0U;
  c8_info[71].mFileTimeHi = 0U;
  c8_info[72].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/eml_xgesvd.m";
  c8_info[72].name = "eml_lapack_xgesvd";
  c8_info[72].dominantType = "double";
  c8_info[72].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgesvd.m";
  c8_info[72].fileTimeLo = 1286818810U;
  c8_info[72].fileTimeHi = 0U;
  c8_info[72].mFileTimeLo = 0U;
  c8_info[72].mFileTimeHi = 0U;
  c8_info[73].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgesvd.m";
  c8_info[73].name = "eml_matlab_zsvdc";
  c8_info[73].dominantType = "double";
  c8_info[73].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[73].fileTimeLo = 1295284866U;
  c8_info[73].fileTimeHi = 0U;
  c8_info[73].mFileTimeLo = 0U;
  c8_info[73].mFileTimeHi = 0U;
  c8_info[74].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[74].name = "eml_index_class";
  c8_info[74].dominantType = "";
  c8_info[74].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[74].fileTimeLo = 1286818778U;
  c8_info[74].fileTimeHi = 0U;
  c8_info[74].mFileTimeLo = 0U;
  c8_info[74].mFileTimeHi = 0U;
  c8_info[75].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[75].name = "eml_scalar_eg";
  c8_info[75].dominantType = "double";
  c8_info[75].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c8_info[75].fileTimeLo = 1286818796U;
  c8_info[75].fileTimeHi = 0U;
  c8_info[75].mFileTimeLo = 0U;
  c8_info[75].mFileTimeHi = 0U;
  c8_info[76].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[76].name = "eml_index_plus";
  c8_info[76].dominantType = "int32";
  c8_info[76].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c8_info[76].fileTimeLo = 1286818778U;
  c8_info[76].fileTimeHi = 0U;
  c8_info[76].mFileTimeLo = 0U;
  c8_info[76].mFileTimeHi = 0U;
  c8_info[77].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[77].name = "min";
  c8_info[77].dominantType = "int32";
  c8_info[77].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/min.m";
  c8_info[77].fileTimeLo = 1311255318U;
  c8_info[77].fileTimeHi = 0U;
  c8_info[77].mFileTimeLo = 0U;
  c8_info[77].mFileTimeHi = 0U;
  c8_info[78].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/min.m";
  c8_info[78].name = "eml_min_or_max";
  c8_info[78].dominantType = "int32";
  c8_info[78].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c8_info[78].fileTimeLo = 1303146212U;
  c8_info[78].fileTimeHi = 0U;
  c8_info[78].mFileTimeLo = 0U;
  c8_info[78].mFileTimeHi = 0U;
  c8_info[79].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c8_info[79].name = "eml_scalar_eg";
  c8_info[79].dominantType = "int32";
  c8_info[79].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c8_info[79].fileTimeLo = 1286818796U;
  c8_info[79].fileTimeHi = 0U;
  c8_info[79].mFileTimeLo = 0U;
  c8_info[79].mFileTimeHi = 0U;
  c8_info[80].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c8_info[80].name = "eml_scalexp_alloc";
  c8_info[80].dominantType = "int32";
  c8_info[80].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c8_info[80].fileTimeLo = 1286818796U;
  c8_info[80].fileTimeHi = 0U;
  c8_info[80].mFileTimeLo = 0U;
  c8_info[80].mFileTimeHi = 0U;
  c8_info[81].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c8_info[81].name = "eml_scalar_eg";
  c8_info[81].dominantType = "int32";
  c8_info[81].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c8_info[81].fileTimeLo = 1286818796U;
  c8_info[81].fileTimeHi = 0U;
  c8_info[81].mFileTimeLo = 0U;
  c8_info[81].mFileTimeHi = 0U;
  c8_info[82].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[82].name = "max";
  c8_info[82].dominantType = "int32";
  c8_info[82].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/max.m";
  c8_info[82].fileTimeLo = 1311255316U;
  c8_info[82].fileTimeHi = 0U;
  c8_info[82].mFileTimeLo = 0U;
  c8_info[82].mFileTimeHi = 0U;
  c8_info[83].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/max.m";
  c8_info[83].name = "eml_min_or_max";
  c8_info[83].dominantType = "int32";
  c8_info[83].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c8_info[83].fileTimeLo = 1303146212U;
  c8_info[83].fileTimeHi = 0U;
  c8_info[83].mFileTimeLo = 0U;
  c8_info[83].mFileTimeHi = 0U;
  c8_info[84].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c8_info[84].name = "eml_relop";
  c8_info[84].dominantType = "function_handle";
  c8_info[84].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_relop.m";
  c8_info[84].fileTimeLo = 1292190510U;
  c8_info[84].fileTimeHi = 0U;
  c8_info[84].mFileTimeLo = 0U;
  c8_info[84].mFileTimeHi = 0U;
  c8_info[85].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c8_info[85].name = "isnan";
  c8_info[85].dominantType = "int32";
  c8_info[85].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isnan.m";
  c8_info[85].fileTimeLo = 1286818760U;
  c8_info[85].fileTimeHi = 0U;
  c8_info[85].mFileTimeLo = 0U;
  c8_info[85].mFileTimeHi = 0U;
  c8_info[86].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[86].name = "eml_index_minus";
  c8_info[86].dominantType = "int32";
  c8_info[86].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c8_info[86].fileTimeLo = 1286818778U;
  c8_info[86].fileTimeHi = 0U;
  c8_info[86].mFileTimeLo = 0U;
  c8_info[86].mFileTimeHi = 0U;
  c8_info[87].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[87].name = "eml_int_forloop_overflow_check";
  c8_info[87].dominantType = "";
  c8_info[87].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c8_info[87].fileTimeLo = 1311255316U;
  c8_info[87].fileTimeHi = 0U;
  c8_info[87].mFileTimeLo = 0U;
  c8_info[87].mFileTimeHi = 0U;
  c8_info[88].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[88].name = "eml_index_times";
  c8_info[88].dominantType = "int32";
  c8_info[88].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c8_info[88].fileTimeLo = 1286818780U;
  c8_info[88].fileTimeHi = 0U;
  c8_info[88].mFileTimeLo = 0U;
  c8_info[88].mFileTimeHi = 0U;
  c8_info[89].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[89].name = "eml_xnrm2";
  c8_info[89].dominantType = "int32";
  c8_info[89].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c8_info[89].fileTimeLo = 1299076776U;
  c8_info[89].fileTimeHi = 0U;
  c8_info[89].mFileTimeLo = 0U;
  c8_info[89].mFileTimeHi = 0U;
  c8_info[90].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m!below_threshold";
  c8_info[90].name = "length";
  c8_info[90].dominantType = "double";
  c8_info[90].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/length.m";
  c8_info[90].fileTimeLo = 1303146206U;
  c8_info[90].fileTimeHi = 0U;
  c8_info[90].mFileTimeLo = 0U;
  c8_info[90].mFileTimeHi = 0U;
  c8_info[91].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/length.m!intlength";
  c8_info[91].name = "eml_index_class";
  c8_info[91].dominantType = "";
  c8_info[91].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[91].fileTimeLo = 1286818778U;
  c8_info[91].fileTimeHi = 0U;
  c8_info[91].mFileTimeLo = 0U;
  c8_info[91].mFileTimeHi = 0U;
  c8_info[92].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[92].name = "eml_div";
  c8_info[92].dominantType = "double";
  c8_info[92].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_div.m";
  c8_info[92].fileTimeLo = 1313347810U;
  c8_info[92].fileTimeHi = 0U;
  c8_info[92].mFileTimeLo = 0U;
  c8_info[92].mFileTimeHi = 0U;
  c8_info[93].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[93].name = "eml_xscal";
  c8_info[93].dominantType = "int32";
  c8_info[93].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m";
  c8_info[93].fileTimeLo = 1299076776U;
  c8_info[93].fileTimeHi = 0U;
  c8_info[93].mFileTimeLo = 0U;
  c8_info[93].mFileTimeHi = 0U;
  c8_info[94].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m";
  c8_info[94].name = "eml_blas_inline";
  c8_info[94].dominantType = "";
  c8_info[94].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c8_info[94].fileTimeLo = 1299076768U;
  c8_info[94].fileTimeHi = 0U;
  c8_info[94].mFileTimeLo = 0U;
  c8_info[94].mFileTimeHi = 0U;
  c8_info[95].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m!below_threshold";
  c8_info[95].name = "length";
  c8_info[95].dominantType = "double";
  c8_info[95].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/length.m";
  c8_info[95].fileTimeLo = 1303146206U;
  c8_info[95].fileTimeHi = 0U;
  c8_info[95].mFileTimeLo = 0U;
  c8_info[95].mFileTimeHi = 0U;
  c8_info[96].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m";
  c8_info[96].name = "eml_scalar_eg";
  c8_info[96].dominantType = "double";
  c8_info[96].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c8_info[96].fileTimeLo = 1286818796U;
  c8_info[96].fileTimeHi = 0U;
  c8_info[96].mFileTimeLo = 0U;
  c8_info[96].mFileTimeHi = 0U;
  c8_info[97].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m";
  c8_info[97].name = "eml_refblas_xscal";
  c8_info[97].dominantType = "int32";
  c8_info[97].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c8_info[97].fileTimeLo = 1299076784U;
  c8_info[97].fileTimeHi = 0U;
  c8_info[97].mFileTimeLo = 0U;
  c8_info[97].mFileTimeHi = 0U;
  c8_info[98].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c8_info[98].name = "eml_index_class";
  c8_info[98].dominantType = "";
  c8_info[98].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[98].fileTimeLo = 1286818778U;
  c8_info[98].fileTimeHi = 0U;
  c8_info[98].mFileTimeLo = 0U;
  c8_info[98].mFileTimeHi = 0U;
  c8_info[99].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c8_info[99].name = "eml_index_minus";
  c8_info[99].dominantType = "int32";
  c8_info[99].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c8_info[99].fileTimeLo = 1286818778U;
  c8_info[99].fileTimeHi = 0U;
  c8_info[99].mFileTimeLo = 0U;
  c8_info[99].mFileTimeHi = 0U;
  c8_info[100].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c8_info[100].name = "eml_index_times";
  c8_info[100].dominantType = "int32";
  c8_info[100].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c8_info[100].fileTimeLo = 1286818780U;
  c8_info[100].fileTimeHi = 0U;
  c8_info[100].mFileTimeLo = 0U;
  c8_info[100].mFileTimeHi = 0U;
  c8_info[101].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c8_info[101].name = "eml_index_plus";
  c8_info[101].dominantType = "int32";
  c8_info[101].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c8_info[101].fileTimeLo = 1286818778U;
  c8_info[101].fileTimeHi = 0U;
  c8_info[101].mFileTimeLo = 0U;
  c8_info[101].mFileTimeHi = 0U;
  c8_info[102].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c8_info[102].name = "eml_int_forloop_overflow_check";
  c8_info[102].dominantType = "";
  c8_info[102].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c8_info[102].fileTimeLo = 1311255316U;
  c8_info[102].fileTimeHi = 0U;
  c8_info[102].mFileTimeLo = 0U;
  c8_info[102].mFileTimeHi = 0U;
  c8_info[103].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[103].name = "eml_xdotc";
  c8_info[103].dominantType = "int32";
  c8_info[103].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m";
  c8_info[103].fileTimeLo = 1299076772U;
  c8_info[103].fileTimeHi = 0U;
  c8_info[103].mFileTimeLo = 0U;
  c8_info[103].mFileTimeHi = 0U;
  c8_info[104].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m";
  c8_info[104].name = "eml_blas_inline";
  c8_info[104].dominantType = "";
  c8_info[104].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c8_info[104].fileTimeLo = 1299076768U;
  c8_info[104].fileTimeHi = 0U;
  c8_info[104].mFileTimeLo = 0U;
  c8_info[104].mFileTimeHi = 0U;
  c8_info[105].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m";
  c8_info[105].name = "eml_xdot";
  c8_info[105].dominantType = "int32";
  c8_info[105].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m";
  c8_info[105].fileTimeLo = 1299076772U;
  c8_info[105].fileTimeHi = 0U;
  c8_info[105].mFileTimeLo = 0U;
  c8_info[105].mFileTimeHi = 0U;
  c8_info[106].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m";
  c8_info[106].name = "eml_blas_inline";
  c8_info[106].dominantType = "";
  c8_info[106].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c8_info[106].fileTimeLo = 1299076768U;
  c8_info[106].fileTimeHi = 0U;
  c8_info[106].mFileTimeLo = 0U;
  c8_info[106].mFileTimeHi = 0U;
  c8_info[107].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m!below_threshold";
  c8_info[107].name = "length";
  c8_info[107].dominantType = "double";
  c8_info[107].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/length.m";
  c8_info[107].fileTimeLo = 1303146206U;
  c8_info[107].fileTimeHi = 0U;
  c8_info[107].mFileTimeLo = 0U;
  c8_info[107].mFileTimeHi = 0U;
  c8_info[108].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m";
  c8_info[108].name = "eml_refblas_xdot";
  c8_info[108].dominantType = "int32";
  c8_info[108].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m";
  c8_info[108].fileTimeLo = 1299076772U;
  c8_info[108].fileTimeHi = 0U;
  c8_info[108].mFileTimeLo = 0U;
  c8_info[108].mFileTimeHi = 0U;
  c8_info[109].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m";
  c8_info[109].name = "eml_refblas_xdotx";
  c8_info[109].dominantType = "int32";
  c8_info[109].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c8_info[109].fileTimeLo = 1299076774U;
  c8_info[109].fileTimeHi = 0U;
  c8_info[109].mFileTimeLo = 0U;
  c8_info[109].mFileTimeHi = 0U;
  c8_info[110].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c8_info[110].name = "eml_scalar_eg";
  c8_info[110].dominantType = "double";
  c8_info[110].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c8_info[110].fileTimeLo = 1286818796U;
  c8_info[110].fileTimeHi = 0U;
  c8_info[110].mFileTimeLo = 0U;
  c8_info[110].mFileTimeHi = 0U;
  c8_info[111].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c8_info[111].name = "eml_index_class";
  c8_info[111].dominantType = "";
  c8_info[111].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[111].fileTimeLo = 1286818778U;
  c8_info[111].fileTimeHi = 0U;
  c8_info[111].mFileTimeLo = 0U;
  c8_info[111].mFileTimeHi = 0U;
  c8_info[112].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c8_info[112].name = "eml_int_forloop_overflow_check";
  c8_info[112].dominantType = "";
  c8_info[112].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c8_info[112].fileTimeLo = 1311255316U;
  c8_info[112].fileTimeHi = 0U;
  c8_info[112].mFileTimeLo = 0U;
  c8_info[112].mFileTimeHi = 0U;
  c8_info[113].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c8_info[113].name = "eml_index_plus";
  c8_info[113].dominantType = "int32";
  c8_info[113].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c8_info[113].fileTimeLo = 1286818778U;
  c8_info[113].fileTimeHi = 0U;
  c8_info[113].mFileTimeLo = 0U;
  c8_info[113].mFileTimeHi = 0U;
  c8_info[114].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[114].name = "eml_xaxpy";
  c8_info[114].dominantType = "int32";
  c8_info[114].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m";
  c8_info[114].fileTimeLo = 1299076770U;
  c8_info[114].fileTimeHi = 0U;
  c8_info[114].mFileTimeLo = 0U;
  c8_info[114].mFileTimeHi = 0U;
  c8_info[115].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m";
  c8_info[115].name = "eml_blas_inline";
  c8_info[115].dominantType = "";
  c8_info[115].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c8_info[115].fileTimeLo = 1299076768U;
  c8_info[115].fileTimeHi = 0U;
  c8_info[115].mFileTimeLo = 0U;
  c8_info[115].mFileTimeHi = 0U;
  c8_info[116].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xaxpy.m!below_threshold";
  c8_info[116].name = "length";
  c8_info[116].dominantType = "double";
  c8_info[116].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/length.m";
  c8_info[116].fileTimeLo = 1303146206U;
  c8_info[116].fileTimeHi = 0U;
  c8_info[116].mFileTimeLo = 0U;
  c8_info[116].mFileTimeHi = 0U;
  c8_info[117].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xaxpy.m";
  c8_info[117].name = "eml_scalar_eg";
  c8_info[117].dominantType = "double";
  c8_info[117].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c8_info[117].fileTimeLo = 1286818796U;
  c8_info[117].fileTimeHi = 0U;
  c8_info[117].mFileTimeLo = 0U;
  c8_info[117].mFileTimeHi = 0U;
  c8_info[118].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xaxpy.m";
  c8_info[118].name = "eml_refblas_xaxpy";
  c8_info[118].dominantType = "int32";
  c8_info[118].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c8_info[118].fileTimeLo = 1299076772U;
  c8_info[118].fileTimeHi = 0U;
  c8_info[118].mFileTimeLo = 0U;
  c8_info[118].mFileTimeHi = 0U;
  c8_info[119].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c8_info[119].name = "eml_index_class";
  c8_info[119].dominantType = "";
  c8_info[119].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[119].fileTimeLo = 1286818778U;
  c8_info[119].fileTimeHi = 0U;
  c8_info[119].mFileTimeLo = 0U;
  c8_info[119].mFileTimeHi = 0U;
  c8_info[120].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c8_info[120].name = "eml_isa_uint";
  c8_info[120].dominantType = "int32";
  c8_info[120].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c8_info[120].fileTimeLo = 1286818784U;
  c8_info[120].fileTimeHi = 0U;
  c8_info[120].mFileTimeLo = 0U;
  c8_info[120].mFileTimeHi = 0U;
  c8_info[121].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c8_info[121].name = "eml_index_minus";
  c8_info[121].dominantType = "int32";
  c8_info[121].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c8_info[121].fileTimeLo = 1286818778U;
  c8_info[121].fileTimeHi = 0U;
  c8_info[121].mFileTimeLo = 0U;
  c8_info[121].mFileTimeHi = 0U;
  c8_info[122].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c8_info[122].name = "eml_int_forloop_overflow_check";
  c8_info[122].dominantType = "";
  c8_info[122].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c8_info[122].fileTimeLo = 1311255316U;
  c8_info[122].fileTimeHi = 0U;
  c8_info[122].mFileTimeLo = 0U;
  c8_info[122].mFileTimeHi = 0U;
  c8_info[123].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c8_info[123].name = "eml_index_plus";
  c8_info[123].dominantType = "int32";
  c8_info[123].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c8_info[123].fileTimeLo = 1286818778U;
  c8_info[123].fileTimeHi = 0U;
  c8_info[123].mFileTimeLo = 0U;
  c8_info[123].mFileTimeHi = 0U;
  c8_info[124].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[124].name = "abs";
  c8_info[124].dominantType = "double";
  c8_info[124].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c8_info[124].fileTimeLo = 1286818694U;
  c8_info[124].fileTimeHi = 0U;
  c8_info[124].mFileTimeLo = 0U;
  c8_info[124].mFileTimeHi = 0U;
  c8_info[125].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[125].name = "mtimes";
  c8_info[125].dominantType = "double";
  c8_info[125].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c8_info[125].fileTimeLo = 1289519692U;
  c8_info[125].fileTimeHi = 0U;
  c8_info[125].mFileTimeLo = 0U;
  c8_info[125].mFileTimeHi = 0U;
  c8_info[126].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[126].name = "realmin";
  c8_info[126].dominantType = "char";
  c8_info[126].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/realmin.m";
  c8_info[126].fileTimeLo = 1307651242U;
  c8_info[126].fileTimeHi = 0U;
  c8_info[126].mFileTimeLo = 0U;
  c8_info[126].mFileTimeHi = 0U;
  c8_info[127].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[127].name = "eps";
  c8_info[127].dominantType = "char";
  c8_info[127].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eps.m";
  c8_info[127].fileTimeLo = 1307651240U;
  c8_info[127].fileTimeHi = 0U;
  c8_info[127].mFileTimeLo = 0U;
  c8_info[127].mFileTimeHi = 0U;
}

static void c8_c_info_helper(c8_ResolvedFunctionInfo c8_info[153])
{
  c8_info[128].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eps.m";
  c8_info[128].name = "eml_is_float_class";
  c8_info[128].dominantType = "char";
  c8_info[128].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c8_info[128].fileTimeLo = 1286818782U;
  c8_info[128].fileTimeHi = 0U;
  c8_info[128].mFileTimeLo = 0U;
  c8_info[128].mFileTimeHi = 0U;
  c8_info[129].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eps.m";
  c8_info[129].name = "eml_eps";
  c8_info[129].dominantType = "char";
  c8_info[129].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c8_info[129].fileTimeLo = 1307651240U;
  c8_info[129].fileTimeHi = 0U;
  c8_info[129].mFileTimeLo = 0U;
  c8_info[129].mFileTimeHi = 0U;
  c8_info[130].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c8_info[130].name = "eml_float_model";
  c8_info[130].dominantType = "char";
  c8_info[130].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c8_info[130].fileTimeLo = 1307651242U;
  c8_info[130].fileTimeHi = 0U;
  c8_info[130].mFileTimeLo = 0U;
  c8_info[130].mFileTimeHi = 0U;
  c8_info[131].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[131].name = "max";
  c8_info[131].dominantType = "double";
  c8_info[131].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/max.m";
  c8_info[131].fileTimeLo = 1311255316U;
  c8_info[131].fileTimeHi = 0U;
  c8_info[131].mFileTimeLo = 0U;
  c8_info[131].mFileTimeHi = 0U;
  c8_info[132].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/max.m";
  c8_info[132].name = "eml_min_or_max";
  c8_info[132].dominantType = "char";
  c8_info[132].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c8_info[132].fileTimeLo = 1303146212U;
  c8_info[132].fileTimeHi = 0U;
  c8_info[132].mFileTimeLo = 0U;
  c8_info[132].mFileTimeHi = 0U;
  c8_info[133].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[133].name = "eml_error";
  c8_info[133].dominantType = "char";
  c8_info[133].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_error.m";
  c8_info[133].fileTimeLo = 1305318000U;
  c8_info[133].fileTimeHi = 0U;
  c8_info[133].mFileTimeLo = 0U;
  c8_info[133].mFileTimeHi = 0U;
  c8_info[134].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c8_info[134].name = "intmin";
  c8_info[134].dominantType = "char";
  c8_info[134].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/intmin.m";
  c8_info[134].fileTimeLo = 1311255318U;
  c8_info[134].fileTimeHi = 0U;
  c8_info[134].mFileTimeLo = 0U;
  c8_info[134].mFileTimeHi = 0U;
  c8_info[135].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum";
  c8_info[135].name = "eml_const_nonsingleton_dim";
  c8_info[135].dominantType = "double";
  c8_info[135].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m";
  c8_info[135].fileTimeLo = 1286818696U;
  c8_info[135].fileTimeHi = 0U;
  c8_info[135].mFileTimeLo = 0U;
  c8_info[135].mFileTimeHi = 0U;
  c8_info[136].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum";
  c8_info[136].name = "eml_scalar_eg";
  c8_info[136].dominantType = "double";
  c8_info[136].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c8_info[136].fileTimeLo = 1286818796U;
  c8_info[136].fileTimeHi = 0U;
  c8_info[136].mFileTimeLo = 0U;
  c8_info[136].mFileTimeHi = 0U;
  c8_info[137].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum";
  c8_info[137].name = "eml_index_class";
  c8_info[137].dominantType = "";
  c8_info[137].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[137].fileTimeLo = 1286818778U;
  c8_info[137].fileTimeHi = 0U;
  c8_info[137].mFileTimeLo = 0U;
  c8_info[137].mFileTimeHi = 0U;
  c8_info[138].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c8_info[138].name = "eml_index_class";
  c8_info[138].dominantType = "";
  c8_info[138].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c8_info[138].fileTimeLo = 1286818778U;
  c8_info[138].fileTimeHi = 0U;
  c8_info[138].mFileTimeLo = 0U;
  c8_info[138].mFileTimeHi = 0U;
  c8_info[139].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c8_info[139].name = "isnan";
  c8_info[139].dominantType = "double";
  c8_info[139].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isnan.m";
  c8_info[139].fileTimeLo = 1286818760U;
  c8_info[139].fileTimeHi = 0U;
  c8_info[139].mFileTimeLo = 0U;
  c8_info[139].mFileTimeHi = 0U;
  c8_info[140].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c8_info[140].name = "eml_index_plus";
  c8_info[140].dominantType = "int32";
  c8_info[140].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c8_info[140].fileTimeLo = 1286818778U;
  c8_info[140].fileTimeHi = 0U;
  c8_info[140].mFileTimeLo = 0U;
  c8_info[140].mFileTimeHi = 0U;
  c8_info[141].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c8_info[141].name = "eml_int_forloop_overflow_check";
  c8_info[141].dominantType = "";
  c8_info[141].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c8_info[141].fileTimeLo = 1311255316U;
  c8_info[141].fileTimeHi = 0U;
  c8_info[141].mFileTimeLo = 0U;
  c8_info[141].mFileTimeHi = 0U;
  c8_info[142].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c8_info[142].name = "eml_relop";
  c8_info[142].dominantType = "function_handle";
  c8_info[142].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_relop.m";
  c8_info[142].fileTimeLo = 1292190510U;
  c8_info[142].fileTimeHi = 0U;
  c8_info[142].mFileTimeLo = 0U;
  c8_info[142].mFileTimeHi = 0U;
  c8_info[143].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[143].name = "sqrt";
  c8_info[143].dominantType = "double";
  c8_info[143].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c8_info[143].fileTimeLo = 1286818752U;
  c8_info[143].fileTimeHi = 0U;
  c8_info[143].mFileTimeLo = 0U;
  c8_info[143].mFileTimeHi = 0U;
  c8_info[144].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c8_info[144].name = "eml_error";
  c8_info[144].dominantType = "char";
  c8_info[144].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_error.m";
  c8_info[144].fileTimeLo = 1305318000U;
  c8_info[144].fileTimeHi = 0U;
  c8_info[144].mFileTimeLo = 0U;
  c8_info[144].mFileTimeHi = 0U;
  c8_info[145].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c8_info[145].name = "eml_scalar_sqrt";
  c8_info[145].dominantType = "double";
  c8_info[145].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m";
  c8_info[145].fileTimeLo = 1286818738U;
  c8_info[145].fileTimeHi = 0U;
  c8_info[145].mFileTimeLo = 0U;
  c8_info[145].mFileTimeHi = 0U;
  c8_info[146].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c8_info[146].name = "eml_xrotg";
  c8_info[146].dominantType = "double";
  c8_info[146].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m";
  c8_info[146].fileTimeLo = 1299076776U;
  c8_info[146].fileTimeHi = 0U;
  c8_info[146].mFileTimeLo = 0U;
  c8_info[146].mFileTimeHi = 0U;
  c8_info[147].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m";
  c8_info[147].name = "eml_blas_inline";
  c8_info[147].dominantType = "";
  c8_info[147].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c8_info[147].fileTimeLo = 1299076768U;
  c8_info[147].fileTimeHi = 0U;
  c8_info[147].mFileTimeLo = 0U;
  c8_info[147].mFileTimeHi = 0U;
  c8_info[148].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xrotg.m";
  c8_info[148].name = "eml_refblas_xrotg";
  c8_info[148].dominantType = "double";
  c8_info[148].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrotg.m";
  c8_info[148].fileTimeLo = 1299076784U;
  c8_info[148].fileTimeHi = 0U;
  c8_info[148].mFileTimeLo = 0U;
  c8_info[148].mFileTimeHi = 0U;
  c8_info[149].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrotg.m";
  c8_info[149].name = "abs";
  c8_info[149].dominantType = "double";
  c8_info[149].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c8_info[149].fileTimeLo = 1286818694U;
  c8_info[149].fileTimeHi = 0U;
  c8_info[149].mFileTimeLo = 0U;
  c8_info[149].mFileTimeHi = 0U;
  c8_info[150].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrotg.m";
  c8_info[150].name = "mrdivide";
  c8_info[150].dominantType = "double";
  c8_info[150].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c8_info[150].fileTimeLo = 1325124138U;
  c8_info[150].fileTimeHi = 0U;
  c8_info[150].mFileTimeLo = 1319729966U;
  c8_info[150].mFileTimeHi = 0U;
  c8_info[151].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrotg.m";
  c8_info[151].name = "sqrt";
  c8_info[151].dominantType = "double";
  c8_info[151].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c8_info[151].fileTimeLo = 1286818752U;
  c8_info[151].fileTimeHi = 0U;
  c8_info[151].mFileTimeLo = 0U;
  c8_info[151].mFileTimeHi = 0U;
  c8_info[152].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xrotg.m!eml_ceval_xrotg";
  c8_info[152].name = "eml_scalar_eg";
  c8_info[152].dominantType = "double";
  c8_info[152].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c8_info[152].fileTimeLo = 1286818796U;
  c8_info[152].fileTimeHi = 0U;
  c8_info[152].mFileTimeLo = 0U;
  c8_info[152].mFileTimeHi = 0U;
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
  int32_T c8_i121;
  int32_T c8_i;
  real_T c8_b_i;
  int32_T c8_i122;
  real_T c8_c_a[16];
  real_T c8_b[16];
  int32_T c8_i123;
  int32_T c8_i124;
  int32_T c8_i125;
  real_T c8_C[16];
  int32_T c8_i126;
  int32_T c8_i127;
  int32_T c8_i128;
  int32_T c8_i129;
  int32_T c8_i130;
  int32_T c8_i131;
  int32_T c8_i132;
  int32_T c8_i133;
  int32_T c8_i134;
  static real_T c8_dv15[9] = { 3.749399456654644E-33, 6.123233995736766E-17, 1.0,
    -1.0, 6.123233995736766E-17, 0.0, -6.123233995736766E-17, -1.0,
    6.123233995736766E-17 };

  int32_T c8_i135;
  static real_T c8_b_b[16] = { 3.749399456654644E-33, 6.123233995736766E-17, 1.0,
    0.0, -1.0, 6.123233995736766E-17, 0.0, 0.0, -6.123233995736766E-17, -1.0,
    6.123233995736766E-17, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T c8_i136;
  int32_T c8_i137;
  int32_T c8_i138;
  int32_T c8_i139;
  int32_T c8_i140;
  int32_T c8_i141;
  int32_T c8_i142;
  int32_T c8_i143;
  int32_T c8_i144;
  int32_T c8_i145;
  int32_T c8_i146;
  int32_T c8_i147;
  int32_T c8_i148;
  int32_T c8_i149;
  sf_debug_symbol_scope_push_eml(0U, 10U, 10U, c8_d_debug_family_names,
    c8_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c8_g, 0U, c8_h_sf_marshallOut,
    c8_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c8_Rtemp, 1U, c8_i_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c8_tool_hom_mat, 2U, c8_h_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c8_nargin, 3U, c8_f_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_nargout, 4U, c8_f_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_q, 5U, c8_b_sf_marshallOut,
    c8_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_b_d, 6U, c8_b_sf_marshallOut,
    c8_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_b_a, 7U, c8_b_sf_marshallOut,
    c8_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_b_alpha, 8U, c8_b_sf_marshallOut,
    c8_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_vec, 9U, c8_c_sf_marshallOut,
    c8_e_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c8_sfEvent, 3);
  c8_eye(chartInstance, c8_dv14);
  for (c8_i121 = 0; c8_i121 < 16; c8_i121++) {
    c8_g[c8_i121] = c8_dv14[c8_i121];
  }

  c8_i = 0;
  while (c8_i < 6) {
    c8_b_i = 1.0 + (real_T)c8_i;
    CV_SCRIPT_FOR(0, 0, 1);
    _SFD_SCRIPT_CALL(0U, chartInstance->c8_sfEvent, 5);
    for (c8_i122 = 0; c8_i122 < 16; c8_i122++) {
      c8_c_a[c8_i122] = c8_g[c8_i122];
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
    for (c8_i123 = 0; c8_i123 < 16; c8_i123++) {
      c8_g[c8_i123] = 0.0;
    }

    for (c8_i124 = 0; c8_i124 < 16; c8_i124++) {
      c8_g[c8_i124] = 0.0;
    }

    for (c8_i125 = 0; c8_i125 < 16; c8_i125++) {
      c8_C[c8_i125] = c8_g[c8_i125];
    }

    for (c8_i126 = 0; c8_i126 < 16; c8_i126++) {
      c8_g[c8_i126] = c8_C[c8_i126];
    }

    for (c8_i127 = 0; c8_i127 < 16; c8_i127++) {
      c8_C[c8_i127] = c8_g[c8_i127];
    }

    for (c8_i128 = 0; c8_i128 < 16; c8_i128++) {
      c8_g[c8_i128] = c8_C[c8_i128];
    }

    for (c8_i129 = 0; c8_i129 < 4; c8_i129++) {
      c8_i130 = 0;
      for (c8_i131 = 0; c8_i131 < 4; c8_i131++) {
        c8_g[c8_i130 + c8_i129] = 0.0;
        c8_i132 = 0;
        for (c8_i133 = 0; c8_i133 < 4; c8_i133++) {
          c8_g[c8_i130 + c8_i129] += c8_c_a[c8_i132 + c8_i129] * c8_b[c8_i133 +
            c8_i130];
          c8_i132 += 4;
        }

        c8_i130 += 4;
      }
    }

    c8_i++;
    sf_mex_listen_for_ctrl_c(chartInstance->S);
  }

  CV_SCRIPT_FOR(0, 0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c8_sfEvent, 8);
  for (c8_i134 = 0; c8_i134 < 9; c8_i134++) {
    c8_Rtemp[c8_i134] = c8_dv15[c8_i134];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c8_sfEvent, 9);
  for (c8_i135 = 0; c8_i135 < 16; c8_i135++) {
    c8_tool_hom_mat[c8_i135] = c8_b_b[c8_i135];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c8_sfEvent, 10);
  for (c8_i136 = 0; c8_i136 < 16; c8_i136++) {
    c8_c_a[c8_i136] = c8_g[c8_i136];
  }

  c8_b_eml_scalar_eg(chartInstance);
  c8_b_eml_scalar_eg(chartInstance);
  for (c8_i137 = 0; c8_i137 < 16; c8_i137++) {
    c8_g[c8_i137] = 0.0;
  }

  for (c8_i138 = 0; c8_i138 < 16; c8_i138++) {
    c8_g[c8_i138] = 0.0;
  }

  for (c8_i139 = 0; c8_i139 < 16; c8_i139++) {
    c8_C[c8_i139] = c8_g[c8_i139];
  }

  for (c8_i140 = 0; c8_i140 < 16; c8_i140++) {
    c8_g[c8_i140] = c8_C[c8_i140];
  }

  for (c8_i141 = 0; c8_i141 < 16; c8_i141++) {
    c8_C[c8_i141] = c8_g[c8_i141];
  }

  for (c8_i142 = 0; c8_i142 < 16; c8_i142++) {
    c8_g[c8_i142] = c8_C[c8_i142];
  }

  for (c8_i143 = 0; c8_i143 < 4; c8_i143++) {
    c8_i144 = 0;
    for (c8_i145 = 0; c8_i145 < 4; c8_i145++) {
      c8_g[c8_i144 + c8_i143] = 0.0;
      c8_i146 = 0;
      for (c8_i147 = 0; c8_i147 < 4; c8_i147++) {
        c8_g[c8_i144 + c8_i143] += c8_c_a[c8_i146 + c8_i143] * c8_b_b[c8_i147 +
          c8_i144];
        c8_i146 += 4;
      }

      c8_i144 += 4;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c8_sfEvent, 11);
  for (c8_i148 = 0; c8_i148 < 3; c8_i148++) {
    c8_vec[c8_i148] = 0.0;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c8_sfEvent, 12);
  for (c8_i149 = 0; c8_i149 < 3; c8_i149++) {
    c8_vec[c8_i149] = c8_g[c8_i149 + 12];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c8_sfEvent, -12);
  sf_debug_symbol_scope_pop();
}

static void c8_isVariableSizing(SFc8_kinematicsInstanceStruct *chartInstance)
{
}

static void c8_eye(SFc8_kinematicsInstanceStruct *chartInstance, real_T c8_I[16])
{
  int32_T c8_i150;
  int32_T c8_i;
  int32_T c8_b_i;
  c8_isVariableSizing(chartInstance);
  for (c8_i150 = 0; c8_i150 < 16; c8_i150++) {
    c8_I[c8_i150] = 0.0;
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
  int32_T c8_i151;
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
  int32_T c8_i152;
  int32_T c8_i153;
  static real_T c8_dv17[4] = { 0.0, 0.0, 0.0, 1.0 };

  sf_debug_symbol_scope_push_eml(0U, 7U, 7U, c8_b_debug_family_names,
    c8_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(&c8_nargin, 0U, c8_f_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_nargout, 1U, c8_f_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_theta, 2U, c8_f_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_b_d, 3U, c8_f_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_b_a, 4U, c8_f_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_b_alpha, 5U, c8_f_sf_marshallOut,
    c8_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c8_mat, 6U, c8_h_sf_marshallOut,
    c8_f_sf_marshallIn);
  CV_SCRIPT_FCN(0, 1);
  _SFD_SCRIPT_CALL(0U, chartInstance->c8_sfEvent, 36);
  c8_eye(chartInstance, c8_dv16);
  for (c8_i151 = 0; c8_i151 < 16; c8_i151++) {
    c8_mat[c8_i151] = c8_dv16[c8_i151];
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
  c8_i152 = 0;
  for (c8_i153 = 0; c8_i153 < 4; c8_i153++) {
    c8_mat[c8_i152 + 3] = c8_dv17[c8_i153];
    c8_i152 += 4;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c8_sfEvent, -37);
  sf_debug_symbol_scope_pop();
}

static void c8_b_eml_scalar_eg(SFc8_kinematicsInstanceStruct *chartInstance)
{
}

static real_T c8_eml_div(SFc8_kinematicsInstanceStruct *chartInstance, real_T
  c8_x, real_T c8_y)
{
  return c8_x / c8_y;
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

static void c8_c_eml_scalar_eg(SFc8_kinematicsInstanceStruct *chartInstance)
{
}

static real_T c8_norm(SFc8_kinematicsInstanceStruct *chartInstance, real_T c8_x
                      [12])
{
  real_T c8_y;
  real_T c8_scale;
  int32_T c8_k;
  int32_T c8_b_k;
  real_T c8_b_x;
  real_T c8_c_x;
  real_T c8_absxk;
  real_T c8_t;
  c8_y = 0.0;
  c8_realmin(chartInstance);
  c8_scale = 2.2250738585072014E-308;
  c8_b_eml_int_forloop_overflow_check(chartInstance);
  for (c8_k = 1; c8_k < 13; c8_k++) {
    c8_b_k = c8_k;
    c8_b_x = c8_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c8_b_k), 1, 12, 1, 0) - 1];
    c8_c_x = c8_b_x;
    c8_absxk = muDoubleScalarAbs(c8_c_x);
    if (c8_absxk > c8_scale) {
      c8_t = c8_scale / c8_absxk;
      c8_y = 1.0 + c8_y * c8_t * c8_t;
      c8_scale = c8_absxk;
    } else {
      c8_t = c8_absxk / c8_scale;
      c8_y += c8_t * c8_t;
    }
  }

  return c8_scale * muDoubleScalarSqrt(c8_y);
}

static void c8_realmin(SFc8_kinematicsInstanceStruct *chartInstance)
{
}

static real_T c8_abs(SFc8_kinematicsInstanceStruct *chartInstance, real_T c8_x)
{
  real_T c8_b_x;
  c8_b_x = c8_x;
  return muDoubleScalarAbs(c8_b_x);
}

static void c8_d_eml_scalar_eg(SFc8_kinematicsInstanceStruct *chartInstance)
{
}

static void c8_eml_xgemm(SFc8_kinematicsInstanceStruct *chartInstance, real_T
  c8_A[72], real_T c8_B[72], real_T c8_C[144], real_T c8_b_C[144])
{
  int32_T c8_i154;
  int32_T c8_i155;
  real_T c8_b_A[72];
  int32_T c8_i156;
  real_T c8_b_B[72];
  for (c8_i154 = 0; c8_i154 < 144; c8_i154++) {
    c8_b_C[c8_i154] = c8_C[c8_i154];
  }

  for (c8_i155 = 0; c8_i155 < 72; c8_i155++) {
    c8_b_A[c8_i155] = c8_A[c8_i155];
  }

  for (c8_i156 = 0; c8_i156 < 72; c8_i156++) {
    c8_b_B[c8_i156] = c8_B[c8_i156];
  }

  c8_b_eml_xgemm(chartInstance, c8_b_A, c8_b_B, c8_b_C);
}

static void c8_e_eml_scalar_eg(SFc8_kinematicsInstanceStruct *chartInstance)
{
}

static real_T c8_b_norm(SFc8_kinematicsInstanceStruct *chartInstance, real_T
  c8_x[144])
{
  int32_T c8_k;
  int32_T c8_b_k;
  real_T c8_b_x;
  real_T c8_c_x;
  boolean_T c8_b;
  boolean_T c8_b0;
  real_T c8_d_x;
  boolean_T c8_b_b;
  boolean_T c8_b1;
  boolean_T c8_c_b;
  int32_T c8_i157;
  real_T c8_e_x[144];
  real_T c8_s[12];
  c8_c_eml_int_forloop_overflow_check(chartInstance);
  for (c8_k = 1; c8_k < 145; c8_k++) {
    c8_b_k = c8_k;
    c8_b_x = c8_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c8_b_k), 1, 144, 1, 0) - 1];
    c8_c_x = c8_b_x;
    c8_b = muDoubleScalarIsInf(c8_c_x);
    c8_b0 = !c8_b;
    c8_d_x = c8_b_x;
    c8_b_b = muDoubleScalarIsNaN(c8_d_x);
    c8_b1 = !c8_b_b;
    c8_c_b = (c8_b0 && c8_b1);
    if (!c8_c_b) {
      c8_eml_error(chartInstance);
    }
  }

  for (c8_i157 = 0; c8_i157 < 144; c8_i157++) {
    c8_e_x[c8_i157] = c8_x[c8_i157];
  }

  c8_eml_matlab_zsvdc(chartInstance, c8_e_x, c8_s);
  return c8_s[0];
}

static void c8_c_eml_int_forloop_overflow_check(SFc8_kinematicsInstanceStruct
  *chartInstance)
{
}

static void c8_eml_error(SFc8_kinematicsInstanceStruct *chartInstance)
{
  int32_T c8_i158;
  static char_T c8_varargin_1[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'v', 'd', '_', 'm', 'a', 't', 'r', 'i', 'x',
    'W', 'i', 't', 'h', 'N', 'a', 'N', 'I', 'n', 'f' };

  char_T c8_u[33];
  const mxArray *c8_y = NULL;
  for (c8_i158 = 0; c8_i158 < 33; c8_i158++) {
    c8_u[c8_i158] = c8_varargin_1[c8_i158];
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 10, 0U, 1U, 0U, 2, 1, 33), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c8_y));
}

static void c8_eml_matlab_zsvdc(SFc8_kinematicsInstanceStruct *chartInstance,
  real_T c8_A[144], real_T c8_S[12])
{
  int32_T c8_i159;
  real_T c8_s[12];
  int32_T c8_i160;
  real_T c8_e[12];
  int32_T c8_i161;
  real_T c8_work[12];
  int32_T c8_q;
  int32_T c8_b_q;
  int32_T c8_b_a;
  int32_T c8_qp1;
  int32_T c8_c_a;
  int32_T c8_qm1;
  int32_T c8_b;
  int32_T c8_c;
  int32_T c8_d_a;
  int32_T c8_b_b;
  int32_T c8_qq;
  int32_T c8_c_b;
  int32_T c8_nmq;
  int32_T c8_e_a;
  int32_T c8_nmqp1;
  int32_T c8_i162;
  real_T c8_b_A[144];
  real_T c8_nrm;
  real_T c8_absx;
  real_T c8_b_d;
  real_T c8_y;
  real_T c8_d1;
  int32_T c8_b_qp1;
  int32_T c8_jj;
  int32_T c8_b_jj;
  int32_T c8_f_a;
  int32_T c8_b_c;
  int32_T c8_d_b;
  int32_T c8_c_c;
  int32_T c8_g_a;
  int32_T c8_e_b;
  int32_T c8_qjj;
  int32_T c8_i163;
  real_T c8_c_A[144];
  int32_T c8_i164;
  real_T c8_d_A[144];
  real_T c8_t;
  int32_T c8_f_b;
  int32_T c8_pmq;
  int32_T c8_i165;
  real_T c8_b_e[12];
  real_T c8_b_absx;
  real_T c8_c_d;
  real_T c8_b_y;
  real_T c8_d2;
  int32_T c8_c_qp1;
  int32_T c8_ii;
  int32_T c8_b_ii;
  int32_T c8_d_qp1;
  int32_T c8_c_jj;
  int32_T c8_h_a;
  int32_T c8_d_c;
  int32_T c8_g_b;
  int32_T c8_e_c;
  int32_T c8_i_a;
  int32_T c8_h_b;
  int32_T c8_qp1jj;
  int32_T c8_i166;
  real_T c8_e_A[144];
  int32_T c8_e_qp1;
  int32_T c8_d_jj;
  int32_T c8_j_a;
  int32_T c8_f_c;
  int32_T c8_i_b;
  int32_T c8_g_c;
  int32_T c8_k_a;
  int32_T c8_j_b;
  int32_T c8_i167;
  real_T c8_b_work[12];
  int32_T c8_m;
  int32_T c8_c_q;
  real_T c8_rt;
  real_T c8_r;
  int32_T c8_l_a;
  int32_T c8_h_c;
  int32_T c8_m_a;
  int32_T c8_i_c;
  real_T c8_n_a;
  real_T c8_k_b;
  real_T c8_c_y;
  real_T c8_iter;
  real_T c8_tiny;
  real_T c8_snorm;
  int32_T c8_c_ii;
  real_T c8_varargin_1;
  real_T c8_varargin_2;
  real_T c8_b_varargin_2;
  real_T c8_varargin_3;
  real_T c8_x;
  real_T c8_d_y;
  real_T c8_b_x;
  real_T c8_e_y;
  real_T c8_xk;
  real_T c8_yk;
  real_T c8_c_x;
  real_T c8_f_y;
  real_T c8_maxval;
  real_T c8_b_varargin_1;
  real_T c8_c_varargin_2;
  real_T c8_d_varargin_2;
  real_T c8_b_varargin_3;
  real_T c8_d_x;
  real_T c8_g_y;
  real_T c8_e_x;
  real_T c8_h_y;
  real_T c8_b_xk;
  real_T c8_b_yk;
  real_T c8_f_x;
  real_T c8_i_y;
  int32_T c8_o_a;
  int32_T c8_p_a;
  int32_T c8_i168;
  int32_T c8_d_ii;
  int32_T c8_q_a;
  int32_T c8_j_c;
  real_T c8_test0;
  real_T c8_ztest0;
  real_T c8_l_b;
  real_T c8_j_y;
  real_T c8_m_b;
  real_T c8_k_y;
  int32_T c8_r_a;
  int32_T c8_k_c;
  real_T c8_kase;
  int32_T c8_qs;
  int32_T c8_b_m;
  int32_T c8_e_ii;
  real_T c8_test;
  int32_T c8_s_a;
  int32_T c8_l_c;
  int32_T c8_t_a;
  int32_T c8_m_c;
  real_T c8_ztest;
  real_T c8_n_b;
  real_T c8_l_y;
  int32_T c8_u_a;
  int32_T c8_v_a;
  int32_T c8_n_c;
  real_T c8_f;
  int32_T c8_w_a;
  int32_T c8_o_c;
  int32_T c8_x_a;
  int32_T c8_i169;
  int32_T c8_k;
  int32_T c8_b_k;
  real_T c8_t1;
  real_T c8_b_t1;
  real_T c8_b_f;
  real_T c8_sn;
  real_T c8_cs;
  real_T c8_b_cs;
  real_T c8_b_sn;
  int32_T c8_y_a;
  int32_T c8_km1;
  real_T c8_ab_a;
  real_T c8_o_b;
  real_T c8_bb_a;
  real_T c8_p_b;
  real_T c8_m_y;
  int32_T c8_cb_a;
  int32_T c8_d_q;
  int32_T c8_c_k;
  real_T c8_c_t1;
  real_T c8_unusedU0;
  real_T c8_c_sn;
  real_T c8_c_cs;
  real_T c8_db_a;
  real_T c8_q_b;
  real_T c8_eb_a;
  real_T c8_r_b;
  real_T c8_n_y;
  int32_T c8_fb_a;
  int32_T c8_mm1;
  real_T c8_d3;
  real_T c8_d4;
  real_T c8_d5;
  real_T c8_d6;
  real_T c8_d7;
  real_T c8_c_varargin_1[5];
  int32_T c8_ixstart;
  real_T c8_mtmp;
  real_T c8_g_x;
  boolean_T c8_s_b;
  int32_T c8_ix;
  int32_T c8_b_ix;
  real_T c8_h_x;
  boolean_T c8_t_b;
  int32_T c8_gb_a;
  int32_T c8_i170;
  int32_T c8_c_ix;
  real_T c8_hb_a;
  real_T c8_u_b;
  boolean_T c8_p;
  real_T c8_b_mtmp;
  real_T c8_scale;
  real_T c8_sm;
  real_T c8_smm1;
  real_T c8_emm1;
  real_T c8_sqds;
  real_T c8_eqds;
  real_T c8_ib_a;
  real_T c8_v_b;
  real_T c8_o_y;
  real_T c8_jb_a;
  real_T c8_w_b;
  real_T c8_p_y;
  real_T c8_x_b;
  real_T c8_kb_a;
  real_T c8_y_b;
  real_T c8_p_c;
  real_T c8_lb_a;
  real_T c8_ab_b;
  real_T c8_shift;
  real_T c8_mb_a;
  real_T c8_bb_b;
  real_T c8_q_y;
  real_T c8_nb_a;
  real_T c8_cb_b;
  real_T c8_r_y;
  real_T c8_ob_a;
  real_T c8_db_b;
  real_T c8_g;
  int32_T c8_e_q;
  int32_T c8_d_k;
  int32_T c8_pb_a;
  int32_T c8_qb_a;
  int32_T c8_kp1;
  real_T c8_c_f;
  real_T c8_unusedU1;
  real_T c8_d_sn;
  real_T c8_d_cs;
  real_T c8_rb_a;
  real_T c8_eb_b;
  real_T c8_s_y;
  real_T c8_sb_a;
  real_T c8_fb_b;
  real_T c8_t_y;
  real_T c8_tb_a;
  real_T c8_gb_b;
  real_T c8_u_y;
  real_T c8_ub_a;
  real_T c8_hb_b;
  real_T c8_v_y;
  real_T c8_vb_a;
  real_T c8_ib_b;
  real_T c8_wb_a;
  real_T c8_jb_b;
  real_T c8_w_y;
  real_T c8_d_f;
  real_T c8_unusedU2;
  real_T c8_e_sn;
  real_T c8_e_cs;
  real_T c8_xb_a;
  real_T c8_kb_b;
  real_T c8_x_y;
  real_T c8_yb_a;
  real_T c8_lb_b;
  real_T c8_y_y;
  real_T c8_ac_a;
  real_T c8_mb_b;
  real_T c8_ab_y;
  real_T c8_bc_a;
  real_T c8_nb_b;
  real_T c8_bb_y;
  real_T c8_cc_a;
  real_T c8_ob_b;
  real_T c8_dc_a;
  real_T c8_pb_b;
  real_T c8_cb_y;
  int32_T c8_ec_a;
  int32_T c8_q_c;
  int32_T c8_e_k;
  int32_T c8_fc_a;
  int32_T c8_gc_a;
  int32_T c8_hc_a;
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  boolean_T guard3 = FALSE;
  boolean_T guard4 = FALSE;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  boolean_T exitg4;
  boolean_T exitg5;
  boolean_T guard11 = FALSE;
  c8_e_eml_scalar_eg(chartInstance);
  for (c8_i159 = 0; c8_i159 < 12; c8_i159++) {
    c8_s[c8_i159] = 0.0;
  }

  for (c8_i160 = 0; c8_i160 < 12; c8_i160++) {
    c8_e[c8_i160] = 0.0;
  }

  for (c8_i161 = 0; c8_i161 < 12; c8_i161++) {
    c8_work[c8_i161] = 0.0;
  }

  c8_d_eml_int_forloop_overflow_check(chartInstance);
  for (c8_q = 1; c8_q < 12; c8_q++) {
    c8_b_q = c8_q;
    c8_b_a = c8_b_q + 1;
    c8_qp1 = c8_b_a;
    c8_c_a = c8_b_q - 1;
    c8_qm1 = c8_c_a;
    c8_b = c8_qm1;
    c8_c = 12 * c8_b;
    c8_d_a = c8_b_q;
    c8_b_b = c8_c;
    c8_qq = c8_d_a + c8_b_b;
    c8_c_b = c8_b_q;
    c8_nmq = 12 - c8_c_b;
    c8_e_a = c8_nmq + 1;
    c8_nmqp1 = c8_e_a;
    if (c8_b_q <= 11) {
      for (c8_i162 = 0; c8_i162 < 144; c8_i162++) {
        c8_b_A[c8_i162] = c8_A[c8_i162];
      }

      c8_nrm = c8_eml_xnrm2(chartInstance, c8_nmqp1, c8_b_A, c8_qq);
      if (c8_nrm > 0.0) {
        c8_absx = c8_nrm;
        c8_b_d = c8_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c8_qq), 1, 144, 1, 0) - 1];
        if (c8_b_d < 0.0) {
          c8_y = -c8_absx;
        } else {
          c8_y = c8_absx;
        }

        c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c8_b_q), 1, 12, 1, 0) - 1] = c8_y;
        c8_d1 = c8_eml_div(chartInstance, 1.0, c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_q), 1, 12, 1, 0) - 1]);
        c8_c_eml_xscal(chartInstance, c8_nmqp1, c8_d1, c8_A, c8_qq);
        c8_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c8_qq), 1, 144, 1, 0) - 1] = c8_A[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_qq), 1, 144, 1, 0) - 1]
          + 1.0;
        c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c8_b_q), 1, 12, 1, 0) - 1] = -c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK
          ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_q), 1, 12, 1, 0) - 1];
      } else {
        c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c8_b_q), 1, 12, 1, 0) - 1] = 0.0;
      }
    }

    c8_b_qp1 = c8_qp1;
    c8_e_eml_int_forloop_overflow_check(chartInstance, c8_b_qp1, 12);
    for (c8_jj = c8_b_qp1; c8_jj < 13; c8_jj++) {
      c8_b_jj = c8_jj;
      c8_f_a = c8_b_jj - 1;
      c8_b_c = c8_f_a;
      c8_d_b = c8_b_c;
      c8_c_c = 12 * c8_d_b;
      c8_g_a = c8_b_q;
      c8_e_b = c8_c_c;
      c8_qjj = c8_g_a + c8_e_b;
      if (c8_b_q <= 11) {
        if (c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c8_b_q), 1, 12, 1, 0) - 1] != 0.0) {
          for (c8_i163 = 0; c8_i163 < 144; c8_i163++) {
            c8_c_A[c8_i163] = c8_A[c8_i163];
          }

          for (c8_i164 = 0; c8_i164 < 144; c8_i164++) {
            c8_d_A[c8_i164] = c8_A[c8_i164];
          }

          c8_t = c8_eml_xdotc(chartInstance, c8_nmqp1, c8_c_A, c8_qq, c8_d_A,
                              c8_qjj);
          c8_t = -c8_eml_div(chartInstance, c8_t, c8_A
                             [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c8_b_q), 1, 12, 1, 0) + 12 *
                               (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c8_b_q), 1, 12, 2, 0) - 1)) - 1]);
          c8_d_eml_xaxpy(chartInstance, c8_nmqp1, c8_t, c8_qq, c8_A, c8_qjj);
        }
      }

      c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c8_b_jj), 1, 12, 1, 0) - 1] = c8_A[_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_qjj), 1, 144, 1, 0) - 1];
    }

    if (c8_b_q <= 10) {
      c8_f_b = c8_b_q;
      c8_pmq = 12 - c8_f_b;
      for (c8_i165 = 0; c8_i165 < 12; c8_i165++) {
        c8_b_e[c8_i165] = c8_e[c8_i165];
      }

      c8_nrm = c8_b_eml_xnrm2(chartInstance, c8_pmq, c8_b_e, c8_qp1);
      if (c8_nrm == 0.0) {
        c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c8_b_q), 1, 12, 1, 0) - 1] = 0.0;
      } else {
        c8_b_absx = c8_nrm;
        c8_c_d = c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c8_qp1), 1, 12, 1, 0) - 1];
        if (c8_c_d < 0.0) {
          c8_b_y = -c8_b_absx;
        } else {
          c8_b_y = c8_b_absx;
        }

        c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c8_b_q), 1, 12, 1, 0) - 1] = c8_b_y;
        c8_d2 = c8_eml_div(chartInstance, 1.0, c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_q), 1, 12, 1, 0) - 1]);
        c8_d_eml_xscal(chartInstance, c8_pmq, c8_d2, c8_e, c8_qp1);
        c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c8_qp1), 1, 12, 1, 0) - 1] = c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_qp1), 1, 12, 1, 0) - 1]
          + 1.0;
      }

      c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c8_b_q), 1, 12, 1, 0) - 1] = -c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_q), 1, 12, 1, 0) - 1];
      if (c8_qp1 <= 12) {
        if (c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c8_b_q), 1, 12, 1, 0) - 1] != 0.0) {
          c8_c_qp1 = c8_qp1;
          c8_e_eml_int_forloop_overflow_check(chartInstance, c8_c_qp1, 12);
          for (c8_ii = c8_c_qp1; c8_ii < 13; c8_ii++) {
            c8_b_ii = c8_ii;
            c8_work[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c8_b_ii), 1, 12, 1, 0) - 1] = 0.0;
          }

          c8_d_qp1 = c8_qp1;
          c8_e_eml_int_forloop_overflow_check(chartInstance, c8_d_qp1, 12);
          for (c8_c_jj = c8_d_qp1; c8_c_jj < 13; c8_c_jj++) {
            c8_b_jj = c8_c_jj;
            c8_h_a = c8_b_jj - 1;
            c8_d_c = c8_h_a;
            c8_g_b = c8_d_c;
            c8_e_c = 12 * c8_g_b;
            c8_i_a = c8_qp1;
            c8_h_b = c8_e_c;
            c8_qp1jj = c8_i_a + c8_h_b;
            for (c8_i166 = 0; c8_i166 < 144; c8_i166++) {
              c8_e_A[c8_i166] = c8_A[c8_i166];
            }

            c8_e_eml_xaxpy(chartInstance, c8_nmq,
                           c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c8_b_jj), 1, 12, 1, 0) - 1], c8_e_A,
                           c8_qp1jj, c8_work, c8_qp1);
          }

          c8_e_qp1 = c8_qp1;
          c8_e_eml_int_forloop_overflow_check(chartInstance, c8_e_qp1, 12);
          for (c8_d_jj = c8_e_qp1; c8_d_jj < 13; c8_d_jj++) {
            c8_b_jj = c8_d_jj;
            c8_t = c8_eml_div(chartInstance, -c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
              "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_jj), 1, 12, 1, 0)
                              - 1], c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_qp1), 1, 12, 1, 0) - 1]);
            c8_j_a = c8_b_jj - 1;
            c8_f_c = c8_j_a;
            c8_i_b = c8_f_c;
            c8_g_c = 12 * c8_i_b;
            c8_k_a = c8_qp1;
            c8_j_b = c8_g_c;
            c8_qp1jj = c8_k_a + c8_j_b;
            for (c8_i167 = 0; c8_i167 < 12; c8_i167++) {
              c8_b_work[c8_i167] = c8_work[c8_i167];
            }

            c8_f_eml_xaxpy(chartInstance, c8_nmq, c8_t, c8_b_work, c8_qp1, c8_A,
                           c8_qp1jj);
          }
        }
      }
    }
  }

  c8_m = 12;
  c8_s[11] = c8_A[143];
  c8_e[10] = c8_A[142];
  c8_e[11] = 0.0;
  c8_b_eml_int_forloop_overflow_check(chartInstance);
  for (c8_c_q = 1; c8_c_q < 13; c8_c_q++) {
    c8_b_q = c8_c_q;
    if (c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c8_b_q), 1, 12, 1, 0) - 1] != 0.0) {
      c8_rt = c8_abs(chartInstance, c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_q), 1, 12, 1, 0) - 1]);
      c8_r = c8_eml_div(chartInstance, c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_q), 1, 12, 1, 0) - 1],
                        c8_rt);
      c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c8_b_q), 1, 12, 1, 0) - 1] = c8_rt;
      if (c8_b_q < 12) {
        c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c8_b_q), 1, 12, 1, 0) - 1] = c8_eml_div(chartInstance,
          c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c8_b_q), 1, 12, 1, 0) - 1], c8_r);
      }
    }

    if (c8_b_q < 12) {
      if (c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c8_b_q), 1, 12, 1, 0) - 1] != 0.0) {
        c8_rt = c8_abs(chartInstance, c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_q), 1, 12, 1, 0) - 1]);
        c8_r = c8_eml_div(chartInstance, c8_rt, c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK
                          ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_q),
                           1, 12, 1, 0) - 1]);
        c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c8_b_q), 1, 12, 1, 0) - 1] = c8_rt;
        c8_l_a = c8_b_q + 1;
        c8_h_c = c8_l_a;
        c8_m_a = c8_b_q + 1;
        c8_i_c = c8_m_a;
        c8_n_a = c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c8_i_c), 1, 12, 1, 0) - 1];
        c8_k_b = c8_r;
        c8_c_y = c8_n_a * c8_k_b;
        c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c8_h_c), 1, 12, 1, 0) - 1] = c8_c_y;
      }
    }
  }

  c8_iter = 0.0;
  c8_realmin(chartInstance);
  c8_eps(chartInstance);
  c8_tiny = c8_eml_div(chartInstance, 2.2250738585072014E-308,
                       2.2204460492503131E-16);
  c8_snorm = 0.0;
  c8_b_eml_int_forloop_overflow_check(chartInstance);
  for (c8_c_ii = 1; c8_c_ii < 13; c8_c_ii++) {
    c8_b_ii = c8_c_ii;
    c8_varargin_1 = c8_abs(chartInstance, c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_ii), 1, 12, 1, 0) - 1]);
    c8_varargin_2 = c8_abs(chartInstance, c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_ii), 1, 12, 1, 0) - 1]);
    c8_b_varargin_2 = c8_varargin_1;
    c8_varargin_3 = c8_varargin_2;
    c8_x = c8_b_varargin_2;
    c8_d_y = c8_varargin_3;
    c8_b_x = c8_x;
    c8_e_y = c8_d_y;
    c8_eml_scalar_eg(chartInstance);
    c8_xk = c8_b_x;
    c8_yk = c8_e_y;
    c8_c_x = c8_xk;
    c8_f_y = c8_yk;
    c8_eml_scalar_eg(chartInstance);
    c8_maxval = muDoubleScalarMax(c8_c_x, c8_f_y);
    c8_b_varargin_1 = c8_snorm;
    c8_c_varargin_2 = c8_maxval;
    c8_d_varargin_2 = c8_b_varargin_1;
    c8_b_varargin_3 = c8_c_varargin_2;
    c8_d_x = c8_d_varargin_2;
    c8_g_y = c8_b_varargin_3;
    c8_e_x = c8_d_x;
    c8_h_y = c8_g_y;
    c8_eml_scalar_eg(chartInstance);
    c8_b_xk = c8_e_x;
    c8_b_yk = c8_h_y;
    c8_f_x = c8_b_xk;
    c8_i_y = c8_b_yk;
    c8_eml_scalar_eg(chartInstance);
    c8_snorm = muDoubleScalarMax(c8_f_x, c8_i_y);
  }

  exitg1 = FALSE;
  while ((exitg1 == 0U) && ((real_T)c8_m > 0.0)) {
    if (c8_iter >= 75.0) {
      c8_b_eml_error(chartInstance);
      exitg1 = TRUE;
    } else {
      c8_o_a = c8_m - 1;
      c8_b_q = c8_o_a;
      c8_p_a = c8_m - 1;
      c8_i168 = c8_p_a;
      c8_f_eml_int_forloop_overflow_check(chartInstance, c8_i168);
      c8_d_ii = c8_i168;
      guard3 = FALSE;
      guard4 = FALSE;
      exitg5 = FALSE;
      while ((exitg5 == 0U) && (c8_d_ii > -1)) {
        c8_b_ii = c8_d_ii;
        c8_b_q = c8_b_ii;
        if ((real_T)c8_b_ii == 0.0) {
          exitg5 = TRUE;
        } else {
          c8_q_a = c8_b_ii + 1;
          c8_j_c = c8_q_a;
          c8_test0 = c8_abs(chartInstance, c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_ii), 1,
            12, 1, 0) - 1]) + c8_abs(chartInstance,
            c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                                       (real_T)c8_j_c), 1, 12, 1, 0) - 1]);
          c8_ztest0 = c8_abs(chartInstance, c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_ii),
            1, 12, 1, 0) - 1]);
          c8_eps(chartInstance);
          c8_l_b = c8_test0;
          c8_j_y = 2.2204460492503131E-16 * c8_l_b;
          if (c8_ztest0 <= c8_j_y) {
            guard4 = TRUE;
            exitg5 = TRUE;
          } else if (c8_ztest0 <= c8_tiny) {
            guard4 = TRUE;
            exitg5 = TRUE;
          } else {
            guard11 = FALSE;
            if (c8_iter > 20.0) {
              c8_eps(chartInstance);
              c8_m_b = c8_snorm;
              c8_k_y = 2.2204460492503131E-16 * c8_m_b;
              if (c8_ztest0 <= c8_k_y) {
                guard3 = TRUE;
                exitg5 = TRUE;
              } else {
                guard11 = TRUE;
              }
            } else {
              guard11 = TRUE;
            }

            if (guard11 == TRUE) {
              c8_d_ii--;
              guard3 = FALSE;
              guard4 = FALSE;
            }
          }
        }
      }

      if (guard4 == TRUE) {
        guard3 = TRUE;
      }

      if (guard3 == TRUE) {
        c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c8_b_ii), 1, 12, 1, 0) - 1] = 0.0;
      }

      c8_r_a = c8_m - 1;
      c8_k_c = c8_r_a;
      if (c8_b_q == c8_k_c) {
        c8_kase = 4.0;
      } else {
        c8_qs = c8_m;
        c8_b_m = c8_m;
        c8_g_eml_int_forloop_overflow_check(chartInstance, c8_b_m, c8_b_q);
        c8_e_ii = c8_b_m;
        guard2 = FALSE;
        exitg4 = FALSE;
        while ((exitg4 == 0U) && (c8_e_ii >= c8_b_q)) {
          c8_b_ii = c8_e_ii;
          c8_qs = c8_b_ii;
          if (c8_b_ii == c8_b_q) {
            exitg4 = TRUE;
          } else {
            c8_test = 0.0;
            if (c8_b_ii < c8_m) {
              c8_test = c8_abs(chartInstance, c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
                "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_ii), 1, 12, 1,
                0) - 1]);
            }

            c8_s_a = c8_b_q + 1;
            c8_l_c = c8_s_a;
            if (c8_b_ii > c8_l_c) {
              c8_t_a = c8_b_ii - 1;
              c8_m_c = c8_t_a;
              c8_test += c8_abs(chartInstance, c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
                "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_m_c), 1, 12, 1, 0)
                                - 1]);
            }

            c8_ztest = c8_abs(chartInstance, c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                               (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_ii),
              1, 12, 1, 0) - 1]);
            c8_eps(chartInstance);
            c8_n_b = c8_test;
            c8_l_y = 2.2204460492503131E-16 * c8_n_b;
            if (c8_ztest <= c8_l_y) {
              guard2 = TRUE;
              exitg4 = TRUE;
            } else if (c8_ztest <= c8_tiny) {
              guard2 = TRUE;
              exitg4 = TRUE;
            } else {
              c8_e_ii--;
              guard2 = FALSE;
            }
          }
        }

        if (guard2 == TRUE) {
          c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c8_b_ii), 1, 12, 1, 0) - 1] = 0.0;
        }

        if (c8_qs == c8_b_q) {
          c8_kase = 3.0;
        } else if (c8_qs == c8_m) {
          c8_kase = 1.0;
        } else {
          c8_kase = 2.0;
          c8_b_q = c8_qs;
        }
      }

      c8_u_a = c8_b_q + 1;
      c8_b_q = c8_u_a;
      switch ((int32_T)_SFD_INTEGER_CHECK("", c8_kase)) {
       case 1:
        c8_v_a = c8_m - 1;
        c8_n_c = c8_v_a;
        c8_f = c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c8_n_c), 1, 12, 1, 0) - 1];
        c8_w_a = c8_m - 1;
        c8_o_c = c8_w_a;
        c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c8_o_c), 1, 12, 1, 0) - 1] = 0.0;
        c8_x_a = c8_m - 1;
        c8_i169 = c8_x_a;
        c8_g_eml_int_forloop_overflow_check(chartInstance, c8_i169, c8_b_q);
        for (c8_k = c8_i169; c8_k >= c8_b_q; c8_k--) {
          c8_b_k = c8_k;
          c8_t1 = c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c8_b_k), 1, 12, 1, 0) - 1];
          c8_b_t1 = c8_t1;
          c8_b_f = c8_f;
          c8_b_eml_xrotg(chartInstance, &c8_b_t1, &c8_b_f, &c8_cs, &c8_sn);
          c8_t1 = c8_b_t1;
          c8_f = c8_b_f;
          c8_b_cs = c8_cs;
          c8_b_sn = c8_sn;
          c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c8_b_k), 1, 12, 1, 0) - 1] = c8_t1;
          if (c8_b_k > c8_b_q) {
            c8_y_a = c8_b_k - 1;
            c8_km1 = c8_y_a;
            c8_ab_a = -c8_b_sn;
            c8_o_b = c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c8_km1), 1, 12, 1, 0) - 1];
            c8_f = c8_ab_a * c8_o_b;
            c8_bb_a = c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c8_km1), 1, 12, 1, 0) - 1];
            c8_p_b = c8_b_cs;
            c8_m_y = c8_bb_a * c8_p_b;
            c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c8_km1), 1, 12, 1, 0) - 1] = c8_m_y;
          }
        }
        break;

       case 2:
        c8_cb_a = c8_b_q - 1;
        c8_qm1 = c8_cb_a;
        c8_f = c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c8_qm1), 1, 12, 1, 0) - 1];
        c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c8_qm1), 1, 12, 1, 0) - 1] = 0.0;
        c8_d_q = c8_b_q;
        c8_e_eml_int_forloop_overflow_check(chartInstance, c8_d_q, c8_m);
        for (c8_c_k = c8_d_q; c8_c_k <= c8_m; c8_c_k++) {
          c8_b_k = c8_c_k;
          c8_t1 = c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c8_b_k), 1, 12, 1, 0) - 1];
          c8_c_t1 = c8_t1;
          c8_unusedU0 = c8_f;
          c8_b_eml_xrotg(chartInstance, &c8_c_t1, &c8_unusedU0, &c8_c_cs,
                         &c8_c_sn);
          c8_t1 = c8_c_t1;
          c8_b_cs = c8_c_cs;
          c8_b_sn = c8_c_sn;
          c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c8_b_k), 1, 12, 1, 0) - 1] = c8_t1;
          c8_db_a = -c8_b_sn;
          c8_q_b = c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c8_b_k), 1, 12, 1, 0) - 1];
          c8_f = c8_db_a * c8_q_b;
          c8_eb_a = c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c8_b_k), 1, 12, 1, 0) - 1];
          c8_r_b = c8_b_cs;
          c8_n_y = c8_eb_a * c8_r_b;
          c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c8_b_k), 1, 12, 1, 0) - 1] = c8_n_y;
        }
        break;

       case 3:
        c8_fb_a = c8_m - 1;
        c8_mm1 = c8_fb_a;
        c8_d3 = c8_abs(chartInstance, c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_m), 1, 12, 1, 0) - 1]);
        c8_d4 = c8_abs(chartInstance, c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_mm1), 1, 12, 1, 0) - 1]);
        c8_d5 = c8_abs(chartInstance, c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_mm1), 1, 12, 1, 0) - 1]);
        c8_d6 = c8_abs(chartInstance, c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_q), 1, 12, 1, 0) - 1]);
        c8_d7 = c8_abs(chartInstance, c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_q), 1, 12, 1, 0) - 1]);
        c8_c_varargin_1[0] = c8_d3;
        c8_c_varargin_1[1] = c8_d4;
        c8_c_varargin_1[2] = c8_d5;
        c8_c_varargin_1[3] = c8_d6;
        c8_c_varargin_1[4] = c8_d7;
        c8_ixstart = 1;
        c8_mtmp = c8_c_varargin_1[0];
        c8_g_x = c8_mtmp;
        c8_s_b = muDoubleScalarIsNaN(c8_g_x);
        if (c8_s_b) {
          c8_e_eml_int_forloop_overflow_check(chartInstance, 2, 5);
          c8_ix = 2;
          exitg2 = FALSE;
          while ((exitg2 == 0U) && (c8_ix < 6)) {
            c8_b_ix = c8_ix;
            c8_ixstart = c8_b_ix;
            c8_h_x = c8_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c8_b_ix), 1, 5, 1, 0) - 1];
            c8_t_b = muDoubleScalarIsNaN(c8_h_x);
            if (!c8_t_b) {
              c8_mtmp = c8_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)c8_b_ix), 1, 5, 1, 0) - 1];
              exitg2 = TRUE;
            } else {
              c8_ix++;
            }
          }
        }

        if (c8_ixstart < 5) {
          c8_gb_a = c8_ixstart + 1;
          c8_i170 = c8_gb_a;
          c8_e_eml_int_forloop_overflow_check(chartInstance, c8_i170, 5);
          for (c8_c_ix = c8_i170; c8_c_ix < 6; c8_c_ix++) {
            c8_b_ix = c8_c_ix;
            c8_hb_a = c8_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c8_b_ix), 1, 5, 1, 0) - 1];
            c8_u_b = c8_mtmp;
            c8_p = (c8_hb_a > c8_u_b);
            if (c8_p) {
              c8_mtmp = c8_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)c8_b_ix), 1, 5, 1, 0) - 1];
            }
          }
        }

        c8_b_mtmp = c8_mtmp;
        c8_scale = c8_b_mtmp;
        c8_sm = c8_eml_div(chartInstance, c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_m), 1, 12, 1, 0) - 1],
                           c8_scale);
        c8_smm1 = c8_eml_div(chartInstance, c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_mm1), 1,
          12, 1, 0) - 1], c8_scale);
        c8_emm1 = c8_eml_div(chartInstance, c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_mm1), 1,
          12, 1, 0) - 1], c8_scale);
        c8_sqds = c8_eml_div(chartInstance, c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_q), 1,
          12, 1, 0) - 1], c8_scale);
        c8_eqds = c8_eml_div(chartInstance, c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_q), 1,
          12, 1, 0) - 1], c8_scale);
        c8_ib_a = c8_smm1 + c8_sm;
        c8_v_b = c8_smm1 - c8_sm;
        c8_o_y = c8_ib_a * c8_v_b;
        c8_jb_a = c8_emm1;
        c8_w_b = c8_emm1;
        c8_p_y = c8_jb_a * c8_w_b;
        c8_x_b = c8_eml_div(chartInstance, c8_o_y + c8_p_y, 2.0);
        c8_kb_a = c8_sm;
        c8_y_b = c8_emm1;
        c8_p_c = c8_kb_a * c8_y_b;
        c8_lb_a = c8_p_c;
        c8_ab_b = c8_p_c;
        c8_p_c = c8_lb_a * c8_ab_b;
        c8_shift = 0.0;
        guard1 = FALSE;
        if (c8_x_b != 0.0) {
          guard1 = TRUE;
        } else {
          if (c8_p_c != 0.0) {
            guard1 = TRUE;
          }
        }

        if (guard1 == TRUE) {
          c8_mb_a = c8_x_b;
          c8_bb_b = c8_x_b;
          c8_q_y = c8_mb_a * c8_bb_b;
          c8_shift = c8_q_y + c8_p_c;
          c8_b_sqrt(chartInstance, &c8_shift);
          if (c8_x_b < 0.0) {
            c8_shift = -c8_shift;
          }

          c8_shift = c8_eml_div(chartInstance, c8_p_c, c8_x_b + c8_shift);
        }

        c8_nb_a = c8_sqds + c8_sm;
        c8_cb_b = c8_sqds - c8_sm;
        c8_r_y = c8_nb_a * c8_cb_b;
        c8_f = c8_r_y + c8_shift;
        c8_ob_a = c8_sqds;
        c8_db_b = c8_eqds;
        c8_g = c8_ob_a * c8_db_b;
        c8_e_q = c8_b_q;
        c8_e_eml_int_forloop_overflow_check(chartInstance, c8_e_q, c8_mm1);
        for (c8_d_k = c8_e_q; c8_d_k <= c8_mm1; c8_d_k++) {
          c8_b_k = c8_d_k;
          c8_pb_a = c8_b_k - 1;
          c8_km1 = c8_pb_a;
          c8_qb_a = c8_b_k + 1;
          c8_kp1 = c8_qb_a;
          c8_c_f = c8_f;
          c8_unusedU1 = c8_g;
          c8_b_eml_xrotg(chartInstance, &c8_c_f, &c8_unusedU1, &c8_d_cs,
                         &c8_d_sn);
          c8_f = c8_c_f;
          c8_b_cs = c8_d_cs;
          c8_b_sn = c8_d_sn;
          if (c8_b_k > c8_b_q) {
            c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c8_km1), 1, 12, 1, 0) - 1] = c8_f;
          }

          c8_rb_a = c8_b_cs;
          c8_eb_b = c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c8_b_k), 1, 12, 1, 0) - 1];
          c8_s_y = c8_rb_a * c8_eb_b;
          c8_sb_a = c8_b_sn;
          c8_fb_b = c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c8_b_k), 1, 12, 1, 0) - 1];
          c8_t_y = c8_sb_a * c8_fb_b;
          c8_f = c8_s_y + c8_t_y;
          c8_tb_a = c8_b_cs;
          c8_gb_b = c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c8_b_k), 1, 12, 1, 0) - 1];
          c8_u_y = c8_tb_a * c8_gb_b;
          c8_ub_a = c8_b_sn;
          c8_hb_b = c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c8_b_k), 1, 12, 1, 0) - 1];
          c8_v_y = c8_ub_a * c8_hb_b;
          c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c8_b_k), 1, 12, 1, 0) - 1] = c8_u_y - c8_v_y;
          c8_vb_a = c8_b_sn;
          c8_ib_b = c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c8_kp1), 1, 12, 1, 0) - 1];
          c8_g = c8_vb_a * c8_ib_b;
          c8_wb_a = c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c8_kp1), 1, 12, 1, 0) - 1];
          c8_jb_b = c8_b_cs;
          c8_w_y = c8_wb_a * c8_jb_b;
          c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c8_kp1), 1, 12, 1, 0) - 1] = c8_w_y;
          c8_d_f = c8_f;
          c8_unusedU2 = c8_g;
          c8_b_eml_xrotg(chartInstance, &c8_d_f, &c8_unusedU2, &c8_e_cs,
                         &c8_e_sn);
          c8_f = c8_d_f;
          c8_b_cs = c8_e_cs;
          c8_b_sn = c8_e_sn;
          c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c8_b_k), 1, 12, 1, 0) - 1] = c8_f;
          c8_xb_a = c8_b_cs;
          c8_kb_b = c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c8_b_k), 1, 12, 1, 0) - 1];
          c8_x_y = c8_xb_a * c8_kb_b;
          c8_yb_a = c8_b_sn;
          c8_lb_b = c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c8_kp1), 1, 12, 1, 0) - 1];
          c8_y_y = c8_yb_a * c8_lb_b;
          c8_f = c8_x_y + c8_y_y;
          c8_ac_a = -c8_b_sn;
          c8_mb_b = c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c8_b_k), 1, 12, 1, 0) - 1];
          c8_ab_y = c8_ac_a * c8_mb_b;
          c8_bc_a = c8_b_cs;
          c8_nb_b = c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c8_kp1), 1, 12, 1, 0) - 1];
          c8_bb_y = c8_bc_a * c8_nb_b;
          c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c8_kp1), 1, 12, 1, 0) - 1] = c8_ab_y + c8_bb_y;
          c8_cc_a = c8_b_sn;
          c8_ob_b = c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c8_kp1), 1, 12, 1, 0) - 1];
          c8_g = c8_cc_a * c8_ob_b;
          c8_dc_a = c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c8_kp1), 1, 12, 1, 0) - 1];
          c8_pb_b = c8_b_cs;
          c8_cb_y = c8_dc_a * c8_pb_b;
          c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c8_kp1), 1, 12, 1, 0) - 1] = c8_cb_y;
        }

        c8_ec_a = c8_m - 1;
        c8_q_c = c8_ec_a;
        c8_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c8_q_c), 1, 12, 1, 0) - 1] = c8_f;
        c8_iter++;
        break;

       default:
        if (c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c8_b_q), 1, 12, 1, 0) - 1] < 0.0) {
          c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c8_b_q), 1, 12, 1, 0) - 1] =
            -c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c8_b_q), 1, 12, 1, 0) - 1];
        }

        c8_fc_a = c8_b_q + 1;
        c8_qp1 = c8_fc_a;
        exitg3 = FALSE;
        while ((exitg3 == 0U) && (c8_b_q < 12)) {
          if (c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
                "", (real_T)c8_b_q), 1, 12, 1, 0) - 1] <
              c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
                "", (real_T)c8_qp1), 1, 12, 1, 0) - 1]) {
            c8_rt = c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c8_b_q), 1, 12, 1, 0) - 1];
            c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c8_b_q), 1, 12, 1, 0) - 1] =
              c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c8_qp1), 1, 12, 1, 0) - 1];
            c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c8_qp1), 1, 12, 1, 0) - 1] = c8_rt;
            c8_b_q = c8_qp1;
            c8_hc_a = c8_b_q + 1;
            c8_qp1 = c8_hc_a;
          } else {
            exitg3 = TRUE;
          }
        }

        c8_iter = 0.0;
        c8_gc_a = c8_m - 1;
        c8_m = c8_gc_a;
        break;
      }
    }
  }

  c8_b_eml_int_forloop_overflow_check(chartInstance);
  for (c8_e_k = 1; c8_e_k < 13; c8_e_k++) {
    c8_b_k = c8_e_k;
    c8_S[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c8_b_k), 1, 12, 1, 0) - 1] = c8_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c8_b_k), 1, 12, 1, 0) - 1];
  }
}

static void c8_d_eml_int_forloop_overflow_check(SFc8_kinematicsInstanceStruct
  *chartInstance)
{
}

static real_T c8_eml_xnrm2(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_x[144], int32_T c8_ix0)
{
  real_T c8_y;
  int32_T c8_b_n;
  int32_T c8_b_ix0;
  int32_T c8_c_n;
  int32_T c8_c_ix0;
  real_T c8_b_x;
  real_T c8_c_x;
  real_T c8_scale;
  int32_T c8_kstart;
  int32_T c8_b_a;
  int32_T c8_c;
  int32_T c8_c_a;
  int32_T c8_b_c;
  int32_T c8_d_a;
  int32_T c8_b;
  int32_T c8_kend;
  int32_T c8_b_kstart;
  int32_T c8_k;
  int32_T c8_b_k;
  real_T c8_d_x;
  real_T c8_e_x;
  real_T c8_absxk;
  real_T c8_t;
  c8_b_n = c8_n;
  c8_b_ix0 = c8_ix0;
  c8_c_n = c8_b_n;
  c8_c_ix0 = c8_b_ix0;
  c8_y = 0.0;
  if ((real_T)c8_c_n < 1.0) {
  } else if ((real_T)c8_c_n == 1.0) {
    c8_b_x = c8_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c8_c_ix0), 1, 144, 1, 0) - 1];
    c8_c_x = c8_b_x;
    c8_y = muDoubleScalarAbs(c8_c_x);
  } else {
    c8_realmin(chartInstance);
    c8_scale = 2.2250738585072014E-308;
    c8_kstart = c8_c_ix0;
    c8_b_a = c8_c_n;
    c8_c = c8_b_a;
    c8_c_a = c8_c - 1;
    c8_b_c = c8_c_a;
    c8_d_a = c8_kstart;
    c8_b = c8_b_c;
    c8_kend = c8_d_a + c8_b;
    c8_b_kstart = c8_kstart;
    c8_e_eml_int_forloop_overflow_check(chartInstance, c8_b_kstart, c8_kend);
    for (c8_k = c8_b_kstart; c8_k <= c8_kend; c8_k++) {
      c8_b_k = c8_k;
      c8_d_x = c8_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c8_b_k), 1, 144, 1, 0) - 1];
      c8_e_x = c8_d_x;
      c8_absxk = muDoubleScalarAbs(c8_e_x);
      if (c8_absxk > c8_scale) {
        c8_t = c8_scale / c8_absxk;
        c8_y = 1.0 + c8_y * c8_t * c8_t;
        c8_scale = c8_absxk;
      } else {
        c8_t = c8_absxk / c8_scale;
        c8_y += c8_t * c8_t;
      }
    }

    c8_y = c8_scale * muDoubleScalarSqrt(c8_y);
  }

  return c8_y;
}

static void c8_e_eml_int_forloop_overflow_check(SFc8_kinematicsInstanceStruct
  *chartInstance, int32_T c8_b_a, int32_T c8_b)
{
  int32_T c8_c_a;
  int32_T c8_b_b;
  boolean_T c8_overflow;
  boolean_T c8_safe;
  int32_T c8_i171;
  static char_T c8_cv0[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c8_u[34];
  const mxArray *c8_y = NULL;
  int32_T c8_i172;
  static char_T c8_cv1[5] = { 'i', 'n', 't', '3', '2' };

  char_T c8_b_u[5];
  const mxArray *c8_b_y = NULL;
  c8_c_a = c8_b_a;
  c8_b_b = c8_b;
  if (c8_c_a > c8_b_b) {
    c8_overflow = FALSE;
  } else {
    c8_overflow = (c8_b_b > 2147483646);
  }

  c8_safe = !c8_overflow;
  if (c8_safe) {
  } else {
    for (c8_i171 = 0; c8_i171 < 34; c8_i171++) {
      c8_u[c8_i171] = c8_cv0[c8_i171];
    }

    c8_y = NULL;
    sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c8_i172 = 0; c8_i172 < 5; c8_i172++) {
      c8_b_u[c8_i172] = c8_cv1[c8_i172];
    }

    c8_b_y = NULL;
    sf_mex_assign(&c8_b_y, sf_mex_create("y", c8_b_u, 10, 0U, 1U, 0U, 2, 1, 5),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c8_y, 14, c8_b_y));
  }
}

static void c8_eml_xscal(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_b_a, real_T c8_x[144], int32_T c8_ix0, real_T c8_b_x[144])
{
  int32_T c8_i173;
  for (c8_i173 = 0; c8_i173 < 144; c8_i173++) {
    c8_b_x[c8_i173] = c8_x[c8_i173];
  }

  c8_c_eml_xscal(chartInstance, c8_n, c8_b_a, c8_b_x, c8_ix0);
}

static real_T c8_eml_xdotc(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_x[144], int32_T c8_ix0, real_T c8_y[144], int32_T c8_iy0)
{
  real_T c8_b_d;
  int32_T c8_b_n;
  int32_T c8_b_ix0;
  int32_T c8_b_iy0;
  int32_T c8_c_n;
  int32_T c8_c_ix0;
  int32_T c8_c_iy0;
  int32_T c8_d_n;
  int32_T c8_d_ix0;
  int32_T c8_d_iy0;
  int32_T c8_e_n;
  int32_T c8_e_ix0;
  int32_T c8_e_iy0;
  int32_T c8_ix;
  int32_T c8_iy;
  int32_T c8_loop_ub;
  int32_T c8_k;
  int32_T c8_b_a;
  int32_T c8_c_a;
  c8_b_n = c8_n;
  c8_b_ix0 = c8_ix0;
  c8_b_iy0 = c8_iy0;
  c8_c_n = c8_b_n;
  c8_c_ix0 = c8_b_ix0;
  c8_c_iy0 = c8_b_iy0;
  c8_d_n = c8_c_n;
  c8_d_ix0 = c8_c_ix0;
  c8_d_iy0 = c8_c_iy0;
  c8_e_n = c8_d_n;
  c8_e_ix0 = c8_d_ix0;
  c8_e_iy0 = c8_d_iy0;
  c8_b_d = 0.0;
  if ((real_T)c8_e_n < 1.0) {
  } else {
    c8_ix = c8_e_ix0;
    c8_iy = c8_e_iy0;
    c8_e_eml_int_forloop_overflow_check(chartInstance, 1, c8_e_n);
    c8_loop_ub = c8_e_n;
    for (c8_k = 1; c8_k <= c8_loop_ub; c8_k++) {
      c8_b_d += c8_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
        ("", (real_T)c8_ix), 1, 144, 1, 0) - 1] *
        c8_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c8_iy), 1, 144, 1, 0) - 1];
      c8_b_a = c8_ix + 1;
      c8_ix = c8_b_a;
      c8_c_a = c8_iy + 1;
      c8_iy = c8_c_a;
    }
  }

  return c8_b_d;
}

static void c8_eml_xaxpy(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_b_a, int32_T c8_ix0, real_T c8_y[144], int32_T c8_iy0, real_T
  c8_b_y[144])
{
  int32_T c8_i174;
  for (c8_i174 = 0; c8_i174 < 144; c8_i174++) {
    c8_b_y[c8_i174] = c8_y[c8_i174];
  }

  c8_d_eml_xaxpy(chartInstance, c8_n, c8_b_a, c8_ix0, c8_b_y, c8_iy0);
}

static real_T c8_b_eml_xnrm2(SFc8_kinematicsInstanceStruct *chartInstance,
  int32_T c8_n, real_T c8_x[12], int32_T c8_ix0)
{
  real_T c8_y;
  int32_T c8_b_n;
  int32_T c8_b_ix0;
  int32_T c8_c_n;
  int32_T c8_c_ix0;
  real_T c8_b_x;
  real_T c8_c_x;
  real_T c8_scale;
  int32_T c8_kstart;
  int32_T c8_b_a;
  int32_T c8_c;
  int32_T c8_c_a;
  int32_T c8_b_c;
  int32_T c8_d_a;
  int32_T c8_b;
  int32_T c8_kend;
  int32_T c8_b_kstart;
  int32_T c8_k;
  int32_T c8_b_k;
  real_T c8_d_x;
  real_T c8_e_x;
  real_T c8_absxk;
  real_T c8_t;
  c8_b_n = c8_n;
  c8_b_ix0 = c8_ix0;
  c8_c_n = c8_b_n;
  c8_c_ix0 = c8_b_ix0;
  c8_y = 0.0;
  if ((real_T)c8_c_n < 1.0) {
  } else if ((real_T)c8_c_n == 1.0) {
    c8_b_x = c8_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c8_c_ix0), 1, 12, 1, 0) - 1];
    c8_c_x = c8_b_x;
    c8_y = muDoubleScalarAbs(c8_c_x);
  } else {
    c8_realmin(chartInstance);
    c8_scale = 2.2250738585072014E-308;
    c8_kstart = c8_c_ix0;
    c8_b_a = c8_c_n;
    c8_c = c8_b_a;
    c8_c_a = c8_c - 1;
    c8_b_c = c8_c_a;
    c8_d_a = c8_kstart;
    c8_b = c8_b_c;
    c8_kend = c8_d_a + c8_b;
    c8_b_kstart = c8_kstart;
    c8_e_eml_int_forloop_overflow_check(chartInstance, c8_b_kstart, c8_kend);
    for (c8_k = c8_b_kstart; c8_k <= c8_kend; c8_k++) {
      c8_b_k = c8_k;
      c8_d_x = c8_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c8_b_k), 1, 12, 1, 0) - 1];
      c8_e_x = c8_d_x;
      c8_absxk = muDoubleScalarAbs(c8_e_x);
      if (c8_absxk > c8_scale) {
        c8_t = c8_scale / c8_absxk;
        c8_y = 1.0 + c8_y * c8_t * c8_t;
        c8_scale = c8_absxk;
      } else {
        c8_t = c8_absxk / c8_scale;
        c8_y += c8_t * c8_t;
      }
    }

    c8_y = c8_scale * muDoubleScalarSqrt(c8_y);
  }

  return c8_y;
}

static void c8_b_eml_xscal(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_b_a, real_T c8_x[12], int32_T c8_ix0, real_T c8_b_x[12])
{
  int32_T c8_i175;
  for (c8_i175 = 0; c8_i175 < 12; c8_i175++) {
    c8_b_x[c8_i175] = c8_x[c8_i175];
  }

  c8_d_eml_xscal(chartInstance, c8_n, c8_b_a, c8_b_x, c8_ix0);
}

static void c8_b_eml_xaxpy(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_b_a, real_T c8_x[144], int32_T c8_ix0, real_T c8_y[12],
  int32_T c8_iy0, real_T c8_b_y[12])
{
  int32_T c8_i176;
  int32_T c8_i177;
  real_T c8_b_x[144];
  for (c8_i176 = 0; c8_i176 < 12; c8_i176++) {
    c8_b_y[c8_i176] = c8_y[c8_i176];
  }

  for (c8_i177 = 0; c8_i177 < 144; c8_i177++) {
    c8_b_x[c8_i177] = c8_x[c8_i177];
  }

  c8_e_eml_xaxpy(chartInstance, c8_n, c8_b_a, c8_b_x, c8_ix0, c8_b_y, c8_iy0);
}

static void c8_c_eml_xaxpy(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_b_a, real_T c8_x[12], int32_T c8_ix0, real_T c8_y[144],
  int32_T c8_iy0, real_T c8_b_y[144])
{
  int32_T c8_i178;
  int32_T c8_i179;
  real_T c8_b_x[12];
  for (c8_i178 = 0; c8_i178 < 144; c8_i178++) {
    c8_b_y[c8_i178] = c8_y[c8_i178];
  }

  for (c8_i179 = 0; c8_i179 < 12; c8_i179++) {
    c8_b_x[c8_i179] = c8_x[c8_i179];
  }

  c8_f_eml_xaxpy(chartInstance, c8_n, c8_b_a, c8_b_x, c8_ix0, c8_b_y, c8_iy0);
}

static void c8_eps(SFc8_kinematicsInstanceStruct *chartInstance)
{
}

static void c8_b_eml_error(SFc8_kinematicsInstanceStruct *chartInstance)
{
  int32_T c8_i180;
  static char_T c8_varargin_1[30] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'v', 'd', '_', 'N', 'o', 'C', 'o', 'n', 'v',
    'e', 'r', 'g', 'e', 'n', 'c', 'e' };

  char_T c8_u[30];
  const mxArray *c8_y = NULL;
  for (c8_i180 = 0; c8_i180 < 30; c8_i180++) {
    c8_u[c8_i180] = c8_varargin_1[c8_i180];
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c8_y));
}

static void c8_f_eml_int_forloop_overflow_check(SFc8_kinematicsInstanceStruct
  *chartInstance, int32_T c8_b_a)
{
  boolean_T c8_overflow;
  boolean_T c8_safe;
  int32_T c8_i181;
  static char_T c8_cv2[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c8_u[34];
  const mxArray *c8_y = NULL;
  int32_T c8_i182;
  static char_T c8_cv3[5] = { 'i', 'n', 't', '3', '2' };

  char_T c8_b_u[5];
  const mxArray *c8_b_y = NULL;
  c8_overflow = FALSE;
  c8_safe = !c8_overflow;
  if (c8_safe) {
  } else {
    for (c8_i181 = 0; c8_i181 < 34; c8_i181++) {
      c8_u[c8_i181] = c8_cv2[c8_i181];
    }

    c8_y = NULL;
    sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c8_i182 = 0; c8_i182 < 5; c8_i182++) {
      c8_b_u[c8_i182] = c8_cv3[c8_i182];
    }

    c8_b_y = NULL;
    sf_mex_assign(&c8_b_y, sf_mex_create("y", c8_b_u, 10, 0U, 1U, 0U, 2, 1, 5),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c8_y, 14, c8_b_y));
  }
}

static void c8_g_eml_int_forloop_overflow_check(SFc8_kinematicsInstanceStruct
  *chartInstance, int32_T c8_b_a, int32_T c8_b)
{
  int32_T c8_c_a;
  int32_T c8_b_b;
  boolean_T c8_overflow;
  boolean_T c8_safe;
  int32_T c8_i183;
  static char_T c8_cv4[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c8_u[34];
  const mxArray *c8_y = NULL;
  int32_T c8_i184;
  static char_T c8_cv5[5] = { 'i', 'n', 't', '3', '2' };

  char_T c8_b_u[5];
  const mxArray *c8_b_y = NULL;
  c8_c_a = c8_b_a;
  c8_b_b = c8_b;
  if (c8_c_a < c8_b_b) {
    c8_overflow = FALSE;
  } else {
    c8_overflow = (c8_b_b < -2147483647);
  }

  c8_safe = !c8_overflow;
  if (c8_safe) {
  } else {
    for (c8_i183 = 0; c8_i183 < 34; c8_i183++) {
      c8_u[c8_i183] = c8_cv4[c8_i183];
    }

    c8_y = NULL;
    sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c8_i184 = 0; c8_i184 < 5; c8_i184++) {
      c8_b_u[c8_i184] = c8_cv5[c8_i184];
    }

    c8_b_y = NULL;
    sf_mex_assign(&c8_b_y, sf_mex_create("y", c8_b_u, 10, 0U, 1U, 0U, 2, 1, 5),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c8_y, 14, c8_b_y));
  }
}

static real_T c8_sqrt(SFc8_kinematicsInstanceStruct *chartInstance, real_T c8_x)
{
  real_T c8_b_x;
  c8_b_x = c8_x;
  c8_b_sqrt(chartInstance, &c8_b_x);
  return c8_b_x;
}

static void c8_c_eml_error(SFc8_kinematicsInstanceStruct *chartInstance)
{
  int32_T c8_i185;
  static char_T c8_varargin_1[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o',
    'o', 'l', 'b', 'o', 'x', ':', 's', 'q', 'r', 't', '_', 'd', 'o', 'm', 'a',
    'i', 'n', 'E', 'r', 'r', 'o', 'r' };

  char_T c8_u[30];
  const mxArray *c8_y = NULL;
  for (c8_i185 = 0; c8_i185 < 30; c8_i185++) {
    c8_u[c8_i185] = c8_varargin_1[c8_i185];
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c8_y));
}

static void c8_eml_xrotg(SFc8_kinematicsInstanceStruct *chartInstance, real_T
  c8_b_a, real_T c8_b, real_T *c8_c_a, real_T *c8_b_b, real_T *c8_c, real_T
  *c8_s)
{
  *c8_c_a = c8_b_a;
  *c8_b_b = c8_b;
  c8_b_eml_xrotg(chartInstance, c8_c_a, c8_b_b, c8_c, c8_s);
}

static void c8_f_eml_scalar_eg(SFc8_kinematicsInstanceStruct *chartInstance)
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

static int32_T c8_i_emlrt_marshallIn(SFc8_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  int32_T c8_y;
  int32_T c8_i186;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_i186, 1, 6, 0U, 0, 0U, 0);
  c8_y = c8_i186;
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
  c8_y = c8_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_b_sfEvent),
    &c8_thisId);
  sf_mex_destroy(&c8_b_sfEvent);
  *(int32_T *)c8_outData = c8_y;
  sf_mex_destroy(&c8_mxArrayInData);
}

static uint8_T c8_j_emlrt_marshallIn(SFc8_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_b_is_active_c8_kinematics, const char_T
  *c8_identifier)
{
  uint8_T c8_y;
  emlrtMsgIdentifier c8_thisId;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_k_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c8_b_is_active_c8_kinematics), &c8_thisId);
  sf_mex_destroy(&c8_b_is_active_c8_kinematics);
  return c8_y;
}

static uint8_T c8_k_emlrt_marshallIn(SFc8_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  uint8_T c8_y;
  uint8_T c8_u0;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_u0, 1, 3, 0U, 0, 0U, 0);
  c8_y = c8_u0;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void c8_b_eml_xgemm(SFc8_kinematicsInstanceStruct *chartInstance, real_T
  c8_A[72], real_T c8_B[72], real_T c8_C[144])
{
  int32_T c8_m;
  int32_T c8_n;
  int32_T c8_k;
  real_T c8_alpha1;
  int32_T c8_lda;
  int32_T c8_ldb;
  real_T c8_beta1;
  int32_T c8_ldc;
  char_T c8_TRANSA;
  char_T c8_TRANSB;
  c8_m = 12;
  c8_n = 12;
  c8_k = 6;
  c8_alpha1 = 1.0;
  c8_lda = 12;
  c8_ldb = 6;
  c8_beta1 = 0.0;
  c8_ldc = 12;
  c8_TRANSA = 'N';
  c8_TRANSB = 'N';
  dgemm32(&c8_TRANSA, &c8_TRANSB, &c8_m, &c8_n, &c8_k, &c8_alpha1, &c8_A[0],
          &c8_lda, &c8_B[0], &c8_ldb, &c8_beta1, &c8_C[0], &c8_ldc);
}

static void c8_c_eml_xscal(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_b_a, real_T c8_x[144], int32_T c8_ix0)
{
  int32_T c8_b_n;
  real_T c8_c_a;
  int32_T c8_b_ix0;
  int32_T c8_c_n;
  real_T c8_d_a;
  int32_T c8_c_ix0;
  int32_T c8_d_ix0;
  int32_T c8_e_a;
  int32_T c8_c;
  int32_T c8_b;
  int32_T c8_b_c;
  int32_T c8_f_a;
  int32_T c8_b_b;
  int32_T c8_i187;
  int32_T c8_k;
  int32_T c8_b_k;
  c8_b_n = c8_n;
  c8_c_a = c8_b_a;
  c8_b_ix0 = c8_ix0;
  c8_c_n = c8_b_n;
  c8_d_a = c8_c_a;
  c8_c_ix0 = c8_b_ix0;
  c8_d_ix0 = c8_c_ix0;
  c8_e_a = c8_c_n;
  c8_c = c8_e_a;
  c8_b = c8_c - 1;
  c8_b_c = c8_b;
  c8_f_a = c8_c_ix0;
  c8_b_b = c8_b_c;
  c8_i187 = c8_f_a + c8_b_b;
  c8_e_eml_int_forloop_overflow_check(chartInstance, c8_d_ix0, c8_i187);
  for (c8_k = c8_d_ix0; c8_k <= c8_i187; c8_k++) {
    c8_b_k = c8_k;
    c8_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c8_b_k), 1, 144, 1, 0) - 1] = c8_d_a * c8_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_k), 1, 144, 1, 0) - 1];
  }
}

static void c8_d_eml_xaxpy(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_b_a, int32_T c8_ix0, real_T c8_y[144], int32_T c8_iy0)
{
  int32_T c8_b_n;
  real_T c8_c_a;
  int32_T c8_b_ix0;
  int32_T c8_b_iy0;
  int32_T c8_c_n;
  real_T c8_d_a;
  int32_T c8_c_ix0;
  int32_T c8_c_iy0;
  int32_T c8_e_a;
  int32_T c8_ix;
  int32_T c8_f_a;
  int32_T c8_iy;
  int32_T c8_g_a;
  int32_T c8_i188;
  int32_T c8_k;
  int32_T c8_h_a;
  int32_T c8_c;
  int32_T c8_i_a;
  int32_T c8_b_c;
  int32_T c8_j_a;
  int32_T c8_c_c;
  int32_T c8_k_a;
  int32_T c8_l_a;
  c8_b_n = c8_n;
  c8_c_a = c8_b_a;
  c8_b_ix0 = c8_ix0;
  c8_b_iy0 = c8_iy0;
  c8_c_n = c8_b_n;
  c8_d_a = c8_c_a;
  c8_c_ix0 = c8_b_ix0;
  c8_c_iy0 = c8_b_iy0;
  if ((real_T)c8_c_n < 1.0) {
  } else if (c8_d_a == 0.0) {
  } else {
    c8_e_a = c8_c_ix0 - 1;
    c8_ix = c8_e_a;
    c8_f_a = c8_c_iy0 - 1;
    c8_iy = c8_f_a;
    c8_g_a = c8_c_n - 1;
    c8_i188 = c8_g_a;
    c8_e_eml_int_forloop_overflow_check(chartInstance, 0, c8_i188);
    for (c8_k = 0; c8_k <= c8_i188; c8_k++) {
      c8_h_a = c8_iy;
      c8_c = c8_h_a;
      c8_i_a = c8_iy;
      c8_b_c = c8_i_a;
      c8_j_a = c8_ix;
      c8_c_c = c8_j_a;
      c8_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c8_c + 1)), 1, 144, 1, 0) - 1] =
        c8_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c8_b_c + 1)), 1, 144, 1, 0) - 1] + c8_d_a *
        c8_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c8_c_c + 1)), 1, 144, 1, 0) - 1];
      c8_k_a = c8_ix + 1;
      c8_ix = c8_k_a;
      c8_l_a = c8_iy + 1;
      c8_iy = c8_l_a;
    }
  }
}

static void c8_d_eml_xscal(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_b_a, real_T c8_x[12], int32_T c8_ix0)
{
  int32_T c8_b_n;
  real_T c8_c_a;
  int32_T c8_b_ix0;
  int32_T c8_c_n;
  real_T c8_d_a;
  int32_T c8_c_ix0;
  int32_T c8_d_ix0;
  int32_T c8_e_a;
  int32_T c8_c;
  int32_T c8_b;
  int32_T c8_b_c;
  int32_T c8_f_a;
  int32_T c8_b_b;
  int32_T c8_i189;
  int32_T c8_k;
  int32_T c8_b_k;
  c8_b_n = c8_n;
  c8_c_a = c8_b_a;
  c8_b_ix0 = c8_ix0;
  c8_c_n = c8_b_n;
  c8_d_a = c8_c_a;
  c8_c_ix0 = c8_b_ix0;
  c8_d_ix0 = c8_c_ix0;
  c8_e_a = c8_c_n;
  c8_c = c8_e_a;
  c8_b = c8_c - 1;
  c8_b_c = c8_b;
  c8_f_a = c8_c_ix0;
  c8_b_b = c8_b_c;
  c8_i189 = c8_f_a + c8_b_b;
  c8_e_eml_int_forloop_overflow_check(chartInstance, c8_d_ix0, c8_i189);
  for (c8_k = c8_d_ix0; c8_k <= c8_i189; c8_k++) {
    c8_b_k = c8_k;
    c8_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c8_b_k), 1, 12, 1, 0) - 1] = c8_d_a * c8_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c8_b_k), 1, 12, 1, 0) - 1];
  }
}

static void c8_e_eml_xaxpy(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_b_a, real_T c8_x[144], int32_T c8_ix0, real_T c8_y[12],
  int32_T c8_iy0)
{
  int32_T c8_b_n;
  real_T c8_c_a;
  int32_T c8_b_ix0;
  int32_T c8_b_iy0;
  int32_T c8_c_n;
  real_T c8_d_a;
  int32_T c8_c_ix0;
  int32_T c8_c_iy0;
  int32_T c8_e_a;
  int32_T c8_ix;
  int32_T c8_f_a;
  int32_T c8_iy;
  int32_T c8_g_a;
  int32_T c8_i190;
  int32_T c8_k;
  int32_T c8_h_a;
  int32_T c8_c;
  int32_T c8_i_a;
  int32_T c8_b_c;
  int32_T c8_j_a;
  int32_T c8_c_c;
  int32_T c8_k_a;
  int32_T c8_l_a;
  c8_b_n = c8_n;
  c8_c_a = c8_b_a;
  c8_b_ix0 = c8_ix0;
  c8_b_iy0 = c8_iy0;
  c8_c_n = c8_b_n;
  c8_d_a = c8_c_a;
  c8_c_ix0 = c8_b_ix0;
  c8_c_iy0 = c8_b_iy0;
  if ((real_T)c8_c_n < 1.0) {
  } else if (c8_d_a == 0.0) {
  } else {
    c8_e_a = c8_c_ix0 - 1;
    c8_ix = c8_e_a;
    c8_f_a = c8_c_iy0 - 1;
    c8_iy = c8_f_a;
    c8_g_a = c8_c_n - 1;
    c8_i190 = c8_g_a;
    c8_e_eml_int_forloop_overflow_check(chartInstance, 0, c8_i190);
    for (c8_k = 0; c8_k <= c8_i190; c8_k++) {
      c8_h_a = c8_iy;
      c8_c = c8_h_a;
      c8_i_a = c8_iy;
      c8_b_c = c8_i_a;
      c8_j_a = c8_ix;
      c8_c_c = c8_j_a;
      c8_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c8_c + 1)), 1, 12, 1, 0) - 1] =
        c8_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c8_b_c + 1)), 1, 12, 1, 0) - 1] + c8_d_a *
        c8_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c8_c_c + 1)), 1, 144, 1, 0) - 1];
      c8_k_a = c8_ix + 1;
      c8_ix = c8_k_a;
      c8_l_a = c8_iy + 1;
      c8_iy = c8_l_a;
    }
  }
}

static void c8_f_eml_xaxpy(SFc8_kinematicsInstanceStruct *chartInstance, int32_T
  c8_n, real_T c8_b_a, real_T c8_x[12], int32_T c8_ix0, real_T c8_y[144],
  int32_T c8_iy0)
{
  int32_T c8_b_n;
  real_T c8_c_a;
  int32_T c8_b_ix0;
  int32_T c8_b_iy0;
  int32_T c8_c_n;
  real_T c8_d_a;
  int32_T c8_c_ix0;
  int32_T c8_c_iy0;
  int32_T c8_e_a;
  int32_T c8_ix;
  int32_T c8_f_a;
  int32_T c8_iy;
  int32_T c8_g_a;
  int32_T c8_i191;
  int32_T c8_k;
  int32_T c8_h_a;
  int32_T c8_c;
  int32_T c8_i_a;
  int32_T c8_b_c;
  int32_T c8_j_a;
  int32_T c8_c_c;
  int32_T c8_k_a;
  int32_T c8_l_a;
  c8_b_n = c8_n;
  c8_c_a = c8_b_a;
  c8_b_ix0 = c8_ix0;
  c8_b_iy0 = c8_iy0;
  c8_c_n = c8_b_n;
  c8_d_a = c8_c_a;
  c8_c_ix0 = c8_b_ix0;
  c8_c_iy0 = c8_b_iy0;
  if ((real_T)c8_c_n < 1.0) {
  } else if (c8_d_a == 0.0) {
  } else {
    c8_e_a = c8_c_ix0 - 1;
    c8_ix = c8_e_a;
    c8_f_a = c8_c_iy0 - 1;
    c8_iy = c8_f_a;
    c8_g_a = c8_c_n - 1;
    c8_i191 = c8_g_a;
    c8_e_eml_int_forloop_overflow_check(chartInstance, 0, c8_i191);
    for (c8_k = 0; c8_k <= c8_i191; c8_k++) {
      c8_h_a = c8_iy;
      c8_c = c8_h_a;
      c8_i_a = c8_iy;
      c8_b_c = c8_i_a;
      c8_j_a = c8_ix;
      c8_c_c = c8_j_a;
      c8_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c8_c + 1)), 1, 144, 1, 0) - 1] =
        c8_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c8_b_c + 1)), 1, 144, 1, 0) - 1] + c8_d_a *
        c8_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c8_c_c + 1)), 1, 12, 1, 0) - 1];
      c8_k_a = c8_ix + 1;
      c8_ix = c8_k_a;
      c8_l_a = c8_iy + 1;
      c8_iy = c8_l_a;
    }
  }
}

static void c8_b_sqrt(SFc8_kinematicsInstanceStruct *chartInstance, real_T *c8_x)
{
  if (*c8_x < 0.0) {
    c8_c_eml_error(chartInstance);
  }

  *c8_x = muDoubleScalarSqrt(*c8_x);
}

static void c8_b_eml_xrotg(SFc8_kinematicsInstanceStruct *chartInstance, real_T *
  c8_b_a, real_T *c8_b, real_T *c8_c, real_T *c8_s)
{
  real_T c8_c_a;
  real_T c8_b_b;
  real_T c8_c_b;
  real_T c8_d_a;
  real_T c8_e_a;
  real_T c8_d_b;
  real_T c8_e_b;
  real_T c8_f_a;
  real_T c8_b_c;
  real_T c8_b_s;
  real_T c8_c_c;
  real_T c8_c_s;
  c8_c_a = *c8_b_a;
  c8_b_b = *c8_b;
  c8_c_b = c8_b_b;
  c8_d_a = c8_c_a;
  c8_e_a = c8_d_a;
  c8_d_b = c8_c_b;
  c8_e_b = c8_d_b;
  c8_f_a = c8_e_a;
  c8_b_c = 0.0;
  c8_b_s = 0.0;
  drotg32(&c8_f_a, &c8_e_b, &c8_b_c, &c8_b_s);
  c8_d_a = c8_f_a;
  c8_c_b = c8_e_b;
  c8_c_c = c8_b_c;
  c8_c_s = c8_b_s;
  *c8_b_a = c8_d_a;
  *c8_b = c8_c_b;
  *c8_c = c8_c_c;
  *c8_s = c8_c_s;
}

static void init_dsm_address_info(SFc8_kinematicsInstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c8_kinematics_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(27946887U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(978737085U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(4274359881U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(695887609U);
}

mxArray *sf_c8_kinematics_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("Va7dqc0NgWW8PFcIAnJCkD");
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

static const mxArray *sf_get_sim_state_info_c8_kinematics(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"vec\",},{M[8],M[0],T\"is_active_c8_kinematics\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
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
           13,
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
        _SFD_CV_INIT_EML(0,1,1,0,0,0,1,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,808);
        _SFD_CV_INIT_EML_FOR(0,1,0,391,401,569);
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
            1.0,0,0,(MexFcnForType)c8_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 6;
          dimVector[1]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 12;
          dimVector[1]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_sf_marshallOut,(MexInFcnForType)
            c8_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_b_sf_marshallOut,(MexInFcnForType)
            c8_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_b_sf_marshallOut,(MexInFcnForType)
            c8_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_b_sf_marshallOut,(MexInFcnForType)
            c8_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_b_sf_marshallOut,(MexInFcnForType)
            c8_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(12,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_b_sf_marshallOut,(MexInFcnForType)
            c8_b_sf_marshallIn);
        }

        {
          real_T (*c8_v_ee)[6];
          real_T (*c8_J)[72];
          real_T (*c8_J_inv)[72];
          real_T (*c8_vec)[12];
          real_T (*c8_q)[6];
          real_T (*c8_eta1)[3];
          real_T (*c8_eta2)[3];
          real_T (*c8_ee_eta1)[3];
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
  return "Zy4wrVTauiLD9o190PlrqF";
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
        infoStruct,8,1);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,8);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2940152842U));
  ssSetChecksum1(S,(2411877342U));
  ssSetChecksum2(S,(1106706919U));
  ssSetChecksum3(S,(1167430259U));
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
