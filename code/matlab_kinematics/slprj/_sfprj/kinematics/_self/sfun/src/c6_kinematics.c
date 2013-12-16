/* Include files */

#include "blascompat32.h"
#include "kinematics_sfun.h"
#include "c6_kinematics.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "kinematics_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c6_debug_family_names[36] = { "dH_prev", "eta2_w_factors",
  "roll_max", "roll_min", "temp", "pitch_max", "pitch_min", "ee_pos", "xmax",
  "xmin", "x", "dev_x", "stddev_x", "W", "Wi", "Jt", "inv_prod", "J_inv",
  "nargin", "nargout", "J", "vee", "q", "eta1", "eta2", "ee_eta1", "qmin",
  "qmax", "d", "a", "alpha", "previousDH", "zeta", "Jw_inv", "dH_trace", "dH" };

static const char * c6_b_debug_family_names[7] = { "nargin", "nargout", "theta",
  "d", "a", "alpha", "mat" };

static const char * c6_c_debug_family_names[12] = { "cphi", "sphi", "cth", "sth",
  "cpsi", "spsi", "nargin", "nargout", "phi", "theta", "psi", "R" };

static const char * c6_d_debug_family_names[10] = { "g", "Rtemp", "tool_hom_mat",
  "nargin", "nargout", "q", "d", "a", "alpha", "vec" };

/* Function Declarations */
static void initialize_c6_kinematics(SFc6_kinematicsInstanceStruct
  *chartInstance);
static void initialize_params_c6_kinematics(SFc6_kinematicsInstanceStruct
  *chartInstance);
static void enable_c6_kinematics(SFc6_kinematicsInstanceStruct *chartInstance);
static void disable_c6_kinematics(SFc6_kinematicsInstanceStruct *chartInstance);
static void c6_update_debugger_state_c6_kinematics(SFc6_kinematicsInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c6_kinematics(SFc6_kinematicsInstanceStruct *
  chartInstance);
static void set_sim_state_c6_kinematics(SFc6_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_st);
static void finalize_c6_kinematics(SFc6_kinematicsInstanceStruct *chartInstance);
static void sf_c6_kinematics(SFc6_kinematicsInstanceStruct *chartInstance);
static void c6_chartstep_c6_kinematics(SFc6_kinematicsInstanceStruct
  *chartInstance);
static void initSimStructsc6_kinematics(SFc6_kinematicsInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber);
static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData);
static void c6_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_dH, const char_T *c6_identifier, real_T c6_y[144]);
static void c6_b_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[144]);
static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_c_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_dH_trace, const char_T *c6_identifier, real_T c6_y[12]);
static void c6_d_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[12]);
static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_c_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_e_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_Jw_inv, const char_T *c6_identifier, real_T c6_y[72]);
static void c6_f_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[72]);
static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_g_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[6]);
static void c6_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_e_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static const mxArray *c6_f_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static const mxArray *c6_g_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static real_T c6_h_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_h_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_i_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[36]);
static void c6_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static void c6_j_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[3]);
static void c6_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_i_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_k_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[3]);
static void c6_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_j_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_l_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[16]);
static void c6_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_k_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_m_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[9]);
static void c6_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static void c6_info_helper(c6_ResolvedFunctionInfo c6_info[156]);
static void c6_b_info_helper(c6_ResolvedFunctionInfo c6_info[156]);
static void c6_c_info_helper(c6_ResolvedFunctionInfo c6_info[156]);
static void c6_isVariableSizing(SFc6_kinematicsInstanceStruct *chartInstance);
static void c6_eye(SFc6_kinematicsInstanceStruct *chartInstance, real_T c6_I[144]);
static void c6_eml_scalar_eg(SFc6_kinematicsInstanceStruct *chartInstance);
static void c6_eml_int_forloop_overflow_check(SFc6_kinematicsInstanceStruct
  *chartInstance);
static real_T c6_rdivide(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_x, real_T c6_y);
static real_T c6_power(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_b_a);
static real_T c6_abs(SFc6_kinematicsInstanceStruct *chartInstance, real_T c6_x);
static real_T c6_b_power(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_b_a);
static void c6_ee_in_vehicle_frame(SFc6_kinematicsInstanceStruct *chartInstance,
  real_T c6_q[6], real_T c6_b_d[6], real_T c6_b_a[6], real_T c6_b_alpha[6],
  real_T c6_vec[3]);
static void c6_b_eye(SFc6_kinematicsInstanceStruct *chartInstance, real_T c6_I
                     [16]);
static void c6_b_eml_int_forloop_overflow_check(SFc6_kinematicsInstanceStruct
  *chartInstance);
static void c6_dh_homogenous(SFc6_kinematicsInstanceStruct *chartInstance,
  real_T c6_theta, real_T c6_b_d, real_T c6_b_a, real_T c6_b_alpha, real_T
  c6_mat[16]);
static void c6_b_eml_scalar_eg(SFc6_kinematicsInstanceStruct *chartInstance);
static void c6_inv(SFc6_kinematicsInstanceStruct *chartInstance, real_T c6_x[144],
                   real_T c6_y[144]);
static void c6_invNxN(SFc6_kinematicsInstanceStruct *chartInstance, real_T c6_x
                      [144], real_T c6_y[144]);
static void c6_realmin(SFc6_kinematicsInstanceStruct *chartInstance);
static void c6_eps(SFc6_kinematicsInstanceStruct *chartInstance);
static void c6_eml_matlab_zgetrf(SFc6_kinematicsInstanceStruct *chartInstance,
  real_T c6_A[144], real_T c6_b_A[144], int32_T c6_ipiv[12], int32_T *c6_info);
static void c6_c_eml_int_forloop_overflow_check(SFc6_kinematicsInstanceStruct
  *chartInstance);
static int32_T c6_eml_ixamax(SFc6_kinematicsInstanceStruct *chartInstance,
  int32_T c6_n, real_T c6_x[144], int32_T c6_ix0);
static void c6_d_eml_int_forloop_overflow_check(SFc6_kinematicsInstanceStruct
  *chartInstance, int32_T c6_b);
static void c6_e_eml_int_forloop_overflow_check(SFc6_kinematicsInstanceStruct
  *chartInstance, int32_T c6_b_a, int32_T c6_b);
static void c6_eml_xtrsm(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_A[144], real_T c6_B[144], real_T c6_b_B[144]);
static real_T c6_norm(SFc6_kinematicsInstanceStruct *chartInstance, real_T c6_x
                      [144]);
static void c6_eml_warning(SFc6_kinematicsInstanceStruct *chartInstance);
static void c6_eml_flt2str(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_x, char_T c6_str[14]);
static void c6_b_eml_warning(SFc6_kinematicsInstanceStruct *chartInstance,
  char_T c6_varargin_2[14]);
static void c6_c_eml_scalar_eg(SFc6_kinematicsInstanceStruct *chartInstance);
static void c6_eml_xgemm(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_A[72], real_T c6_B[144], real_T c6_C[72], real_T c6_b_C[72]);
static void c6_d_eml_scalar_eg(SFc6_kinematicsInstanceStruct *chartInstance);
static void c6_b_inv(SFc6_kinematicsInstanceStruct *chartInstance, real_T c6_x
                     [36], real_T c6_y[36]);
static void c6_b_invNxN(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_x[36], real_T c6_y[36]);
static void c6_b_eml_matlab_zgetrf(SFc6_kinematicsInstanceStruct *chartInstance,
  real_T c6_A[36], real_T c6_b_A[36], int32_T c6_ipiv[6], int32_T *c6_info);
static int32_T c6_b_eml_ixamax(SFc6_kinematicsInstanceStruct *chartInstance,
  int32_T c6_n, real_T c6_x[36], int32_T c6_ix0);
static void c6_b_eml_xtrsm(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_A[36], real_T c6_B[36], real_T c6_b_B[36]);
static void c6_f_eml_int_forloop_overflow_check(SFc6_kinematicsInstanceStruct
  *chartInstance);
static real_T c6_b_norm(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_x[36]);
static void c6_e_eml_scalar_eg(SFc6_kinematicsInstanceStruct *chartInstance);
static void c6_b_eml_xgemm(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_A[144], real_T c6_B[72], real_T c6_C[72], real_T c6_b_C[72]);
static void c6_f_eml_scalar_eg(SFc6_kinematicsInstanceStruct *chartInstance);
static void c6_g_eml_scalar_eg(SFc6_kinematicsInstanceStruct *chartInstance);
static void c6_n_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_sprintf, const char_T *c6_identifier, char_T c6_y[14]);
static void c6_o_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, char_T c6_y[14]);
static const mxArray *c6_l_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static int32_T c6_p_emlrt_marshallIn(SFc6_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static uint8_T c6_q_emlrt_marshallIn(SFc6_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_b_is_active_c6_kinematics, const char_T
  *c6_identifier);
static uint8_T c6_r_emlrt_marshallIn(SFc6_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_c_eml_matlab_zgetrf(SFc6_kinematicsInstanceStruct *chartInstance,
  real_T c6_A[144], int32_T c6_ipiv[12], int32_T *c6_info);
static void c6_c_eml_xtrsm(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_A[144], real_T c6_B[144]);
static void c6_c_eml_xgemm(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_A[72], real_T c6_B[144], real_T c6_C[72]);
static void c6_d_eml_matlab_zgetrf(SFc6_kinematicsInstanceStruct *chartInstance,
  real_T c6_A[36], int32_T c6_ipiv[6], int32_T *c6_info);
static void c6_d_eml_xtrsm(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_A[36], real_T c6_B[36]);
static void c6_d_eml_xgemm(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_A[144], real_T c6_B[72], real_T c6_C[72]);
static void init_dsm_address_info(SFc6_kinematicsInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c6_kinematics(SFc6_kinematicsInstanceStruct
  *chartInstance)
{
  chartInstance->c6_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c6_is_active_c6_kinematics = 0U;
}

static void initialize_params_c6_kinematics(SFc6_kinematicsInstanceStruct
  *chartInstance)
{
  real_T c6_dv0[6];
  int32_T c6_i0;
  real_T c6_dv1[6];
  int32_T c6_i1;
  real_T c6_dv2[6];
  int32_T c6_i2;
  real_T c6_dv3[6];
  int32_T c6_i3;
  real_T c6_dv4[6];
  int32_T c6_i4;
  sf_set_error_prefix_string(
    "Error evaluating data 'qmin' in the parent workspace.\n");
  sf_mex_import_named("qmin", sf_mex_get_sfun_param(chartInstance->S, 4, 0),
                      c6_dv0, 0, 0, 0U, 1, 0U, 1, 6);
  for (c6_i0 = 0; c6_i0 < 6; c6_i0++) {
    chartInstance->c6_qmin[c6_i0] = c6_dv0[c6_i0];
  }

  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
  sf_set_error_prefix_string(
    "Error evaluating data 'qmax' in the parent workspace.\n");
  sf_mex_import_named("qmax", sf_mex_get_sfun_param(chartInstance->S, 3, 0),
                      c6_dv1, 0, 0, 0U, 1, 0U, 1, 6);
  for (c6_i1 = 0; c6_i1 < 6; c6_i1++) {
    chartInstance->c6_qmax[c6_i1] = c6_dv1[c6_i1];
  }

  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
  sf_set_error_prefix_string(
    "Error evaluating data 'd' in the parent workspace.\n");
  sf_mex_import_named("d", sf_mex_get_sfun_param(chartInstance->S, 2, 0), c6_dv2,
                      0, 0, 0U, 1, 0U, 1, 6);
  for (c6_i2 = 0; c6_i2 < 6; c6_i2++) {
    chartInstance->c6_d[c6_i2] = c6_dv2[c6_i2];
  }

  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
  sf_set_error_prefix_string(
    "Error evaluating data 'a' in the parent workspace.\n");
  sf_mex_import_named("a", sf_mex_get_sfun_param(chartInstance->S, 0, 0), c6_dv3,
                      0, 0, 0U, 1, 0U, 1, 6);
  for (c6_i3 = 0; c6_i3 < 6; c6_i3++) {
    chartInstance->c6_a[c6_i3] = c6_dv3[c6_i3];
  }

  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
  sf_set_error_prefix_string(
    "Error evaluating data 'alpha' in the parent workspace.\n");
  sf_mex_import_named("alpha", sf_mex_get_sfun_param(chartInstance->S, 1, 0),
                      c6_dv4, 0, 0, 0U, 1, 0U, 1, 6);
  for (c6_i4 = 0; c6_i4 < 6; c6_i4++) {
    chartInstance->c6_alpha[c6_i4] = c6_dv4[c6_i4];
  }

  sf_set_error_prefix_string("Stateflow Runtime Error (chart): ");
}

static void enable_c6_kinematics(SFc6_kinematicsInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c6_kinematics(SFc6_kinematicsInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c6_update_debugger_state_c6_kinematics(SFc6_kinematicsInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c6_kinematics(SFc6_kinematicsInstanceStruct *
  chartInstance)
{
  const mxArray *c6_st;
  const mxArray *c6_y = NULL;
  int32_T c6_i5;
  real_T c6_u[72];
  const mxArray *c6_b_y = NULL;
  int32_T c6_i6;
  real_T c6_b_u[144];
  const mxArray *c6_c_y = NULL;
  int32_T c6_i7;
  real_T c6_c_u[12];
  const mxArray *c6_d_y = NULL;
  int32_T c6_i8;
  real_T c6_d_u[12];
  const mxArray *c6_e_y = NULL;
  uint8_T c6_hoistedGlobal;
  uint8_T c6_e_u;
  const mxArray *c6_f_y = NULL;
  real_T (*c6_zeta)[12];
  real_T (*c6_dH_trace)[12];
  real_T (*c6_dH)[144];
  real_T (*c6_Jw_inv)[72];
  c6_dH = (real_T (*)[144])ssGetOutputPortSignal(chartInstance->S, 4);
  c6_dH_trace = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 3);
  c6_Jw_inv = (real_T (*)[72])ssGetOutputPortSignal(chartInstance->S, 2);
  c6_zeta = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 1);
  c6_st = NULL;
  c6_st = NULL;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_createcellarray(5), FALSE);
  for (c6_i5 = 0; c6_i5 < 72; c6_i5++) {
    c6_u[c6_i5] = (*c6_Jw_inv)[c6_i5];
  }

  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 12, 6),
                FALSE);
  sf_mex_setcell(c6_y, 0, c6_b_y);
  for (c6_i6 = 0; c6_i6 < 144; c6_i6++) {
    c6_b_u[c6_i6] = (*c6_dH)[c6_i6];
  }

  c6_c_y = NULL;
  sf_mex_assign(&c6_c_y, sf_mex_create("y", c6_b_u, 0, 0U, 1U, 0U, 2, 12, 12),
                FALSE);
  sf_mex_setcell(c6_y, 1, c6_c_y);
  for (c6_i7 = 0; c6_i7 < 12; c6_i7++) {
    c6_c_u[c6_i7] = (*c6_dH_trace)[c6_i7];
  }

  c6_d_y = NULL;
  sf_mex_assign(&c6_d_y, sf_mex_create("y", c6_c_u, 0, 0U, 1U, 0U, 1, 12), FALSE);
  sf_mex_setcell(c6_y, 2, c6_d_y);
  for (c6_i8 = 0; c6_i8 < 12; c6_i8++) {
    c6_d_u[c6_i8] = (*c6_zeta)[c6_i8];
  }

  c6_e_y = NULL;
  sf_mex_assign(&c6_e_y, sf_mex_create("y", c6_d_u, 0, 0U, 1U, 0U, 1, 12), FALSE);
  sf_mex_setcell(c6_y, 3, c6_e_y);
  c6_hoistedGlobal = chartInstance->c6_is_active_c6_kinematics;
  c6_e_u = c6_hoistedGlobal;
  c6_f_y = NULL;
  sf_mex_assign(&c6_f_y, sf_mex_create("y", &c6_e_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c6_y, 4, c6_f_y);
  sf_mex_assign(&c6_st, c6_y, FALSE);
  return c6_st;
}

static void set_sim_state_c6_kinematics(SFc6_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_st)
{
  const mxArray *c6_u;
  real_T c6_dv5[72];
  int32_T c6_i9;
  real_T c6_dv6[144];
  int32_T c6_i10;
  real_T c6_dv7[12];
  int32_T c6_i11;
  real_T c6_dv8[12];
  int32_T c6_i12;
  real_T (*c6_Jw_inv)[72];
  real_T (*c6_dH)[144];
  real_T (*c6_dH_trace)[12];
  real_T (*c6_zeta)[12];
  c6_dH = (real_T (*)[144])ssGetOutputPortSignal(chartInstance->S, 4);
  c6_dH_trace = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 3);
  c6_Jw_inv = (real_T (*)[72])ssGetOutputPortSignal(chartInstance->S, 2);
  c6_zeta = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c6_doneDoubleBufferReInit = TRUE;
  c6_u = sf_mex_dup(c6_st);
  c6_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 0)),
                        "Jw_inv", c6_dv5);
  for (c6_i9 = 0; c6_i9 < 72; c6_i9++) {
    (*c6_Jw_inv)[c6_i9] = c6_dv5[c6_i9];
  }

  c6_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 1)), "dH",
                      c6_dv6);
  for (c6_i10 = 0; c6_i10 < 144; c6_i10++) {
    (*c6_dH)[c6_i10] = c6_dv6[c6_i10];
  }

  c6_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 2)),
                        "dH_trace", c6_dv7);
  for (c6_i11 = 0; c6_i11 < 12; c6_i11++) {
    (*c6_dH_trace)[c6_i11] = c6_dv7[c6_i11];
  }

  c6_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 3)),
                        "zeta", c6_dv8);
  for (c6_i12 = 0; c6_i12 < 12; c6_i12++) {
    (*c6_zeta)[c6_i12] = c6_dv8[c6_i12];
  }

  chartInstance->c6_is_active_c6_kinematics = c6_q_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 4)),
     "is_active_c6_kinematics");
  sf_mex_destroy(&c6_u);
  c6_update_debugger_state_c6_kinematics(chartInstance);
  sf_mex_destroy(&c6_st);
}

static void finalize_c6_kinematics(SFc6_kinematicsInstanceStruct *chartInstance)
{
}

static void sf_c6_kinematics(SFc6_kinematicsInstanceStruct *chartInstance)
{
  int32_T c6_i13;
  int32_T c6_i14;
  int32_T c6_i15;
  int32_T c6_i16;
  int32_T c6_i17;
  int32_T c6_i18;
  int32_T c6_i19;
  int32_T c6_i20;
  int32_T c6_i21;
  int32_T c6_i22;
  int32_T c6_i23;
  int32_T c6_i24;
  int32_T c6_i25;
  int32_T c6_i26;
  int32_T c6_i27;
  int32_T c6_i28;
  real_T (*c6_previousDH)[144];
  real_T (*c6_dH)[144];
  real_T (*c6_dH_trace)[12];
  real_T (*c6_Jw_inv)[72];
  real_T (*c6_ee_eta1)[3];
  real_T (*c6_eta2)[3];
  real_T (*c6_eta1)[3];
  real_T (*c6_q)[6];
  real_T (*c6_vee)[6];
  real_T (*c6_zeta)[12];
  real_T (*c6_J)[72];
  c6_previousDH = (real_T (*)[144])ssGetInputPortSignal(chartInstance->S, 6);
  c6_dH = (real_T (*)[144])ssGetOutputPortSignal(chartInstance->S, 4);
  c6_dH_trace = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 3);
  c6_Jw_inv = (real_T (*)[72])ssGetOutputPortSignal(chartInstance->S, 2);
  c6_ee_eta1 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 5);
  c6_eta2 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 4);
  c6_eta1 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 3);
  c6_q = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 2);
  c6_vee = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 1);
  c6_zeta = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 1);
  c6_J = (real_T (*)[72])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 5U, chartInstance->c6_sfEvent);
  for (c6_i13 = 0; c6_i13 < 72; c6_i13++) {
    _SFD_DATA_RANGE_CHECK((*c6_J)[c6_i13], 0U);
  }

  for (c6_i14 = 0; c6_i14 < 12; c6_i14++) {
    _SFD_DATA_RANGE_CHECK((*c6_zeta)[c6_i14], 1U);
  }

  for (c6_i15 = 0; c6_i15 < 6; c6_i15++) {
    _SFD_DATA_RANGE_CHECK((*c6_vee)[c6_i15], 2U);
  }

  for (c6_i16 = 0; c6_i16 < 6; c6_i16++) {
    _SFD_DATA_RANGE_CHECK((*c6_q)[c6_i16], 3U);
  }

  for (c6_i17 = 0; c6_i17 < 3; c6_i17++) {
    _SFD_DATA_RANGE_CHECK((*c6_eta1)[c6_i17], 4U);
  }

  for (c6_i18 = 0; c6_i18 < 3; c6_i18++) {
    _SFD_DATA_RANGE_CHECK((*c6_eta2)[c6_i18], 5U);
  }

  for (c6_i19 = 0; c6_i19 < 3; c6_i19++) {
    _SFD_DATA_RANGE_CHECK((*c6_ee_eta1)[c6_i19], 6U);
  }

  for (c6_i20 = 0; c6_i20 < 6; c6_i20++) {
    _SFD_DATA_RANGE_CHECK(chartInstance->c6_qmin[c6_i20], 7U);
  }

  for (c6_i21 = 0; c6_i21 < 6; c6_i21++) {
    _SFD_DATA_RANGE_CHECK(chartInstance->c6_qmax[c6_i21], 8U);
  }

  for (c6_i22 = 0; c6_i22 < 6; c6_i22++) {
    _SFD_DATA_RANGE_CHECK(chartInstance->c6_d[c6_i22], 9U);
  }

  for (c6_i23 = 0; c6_i23 < 6; c6_i23++) {
    _SFD_DATA_RANGE_CHECK(chartInstance->c6_a[c6_i23], 10U);
  }

  for (c6_i24 = 0; c6_i24 < 6; c6_i24++) {
    _SFD_DATA_RANGE_CHECK(chartInstance->c6_alpha[c6_i24], 11U);
  }

  for (c6_i25 = 0; c6_i25 < 72; c6_i25++) {
    _SFD_DATA_RANGE_CHECK((*c6_Jw_inv)[c6_i25], 12U);
  }

  for (c6_i26 = 0; c6_i26 < 12; c6_i26++) {
    _SFD_DATA_RANGE_CHECK((*c6_dH_trace)[c6_i26], 13U);
  }

  for (c6_i27 = 0; c6_i27 < 144; c6_i27++) {
    _SFD_DATA_RANGE_CHECK((*c6_dH)[c6_i27], 14U);
  }

  for (c6_i28 = 0; c6_i28 < 144; c6_i28++) {
    _SFD_DATA_RANGE_CHECK((*c6_previousDH)[c6_i28], 15U);
  }

  chartInstance->c6_sfEvent = CALL_EVENT;
  c6_chartstep_c6_kinematics(chartInstance);
  sf_debug_check_for_state_inconsistency(_kinematicsMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c6_chartstep_c6_kinematics(SFc6_kinematicsInstanceStruct
  *chartInstance)
{
  int32_T c6_i29;
  real_T c6_J[72];
  int32_T c6_i30;
  real_T c6_vee[6];
  int32_T c6_i31;
  real_T c6_q[6];
  int32_T c6_i32;
  real_T c6_eta1[3];
  int32_T c6_i33;
  real_T c6_eta2[3];
  int32_T c6_i34;
  real_T c6_ee_eta1[3];
  int32_T c6_i35;
  real_T c6_b_qmin[6];
  int32_T c6_i36;
  real_T c6_b_qmax[6];
  int32_T c6_i37;
  real_T c6_b_d[6];
  int32_T c6_i38;
  real_T c6_b_a[6];
  int32_T c6_i39;
  real_T c6_b_alpha[6];
  int32_T c6_i40;
  real_T c6_previousDH[144];
  uint32_T c6_debug_family_var_map[36];
  real_T c6_dH_prev[144];
  real_T c6_eta2_w_factors[3];
  real_T c6_roll_max;
  real_T c6_roll_min;
  real_T c6_temp;
  real_T c6_pitch_max;
  real_T c6_pitch_min;
  real_T c6_ee_pos[3];
  real_T c6_xmax;
  real_T c6_xmin;
  real_T c6_x;
  real_T c6_dev_x;
  real_T c6_stddev_x;
  real_T c6_W[144];
  real_T c6_Wi[144];
  real_T c6_Jt[72];
  real_T c6_inv_prod[36];
  real_T c6_J_inv[72];
  real_T c6_nargin = 12.0;
  real_T c6_nargout = 4.0;
  real_T c6_zeta[12];
  real_T c6_Jw_inv[72];
  real_T c6_dH_trace[12];
  real_T c6_dH[144];
  int32_T c6_i41;
  int32_T c6_i42;
  real_T c6_dv9[144];
  int32_T c6_i43;
  int32_T c6_i44;
  static real_T c6_dv10[3] = { 4000.0, 4000.0, 10.0 };

  real_T c6_b;
  real_T c6_y;
  real_T c6_b_b;
  real_T c6_b_y;
  real_T c6_c_b;
  real_T c6_c_y;
  real_T c6_c_a;
  real_T c6_d_b;
  real_T c6_d_y;
  real_T c6_A;
  real_T c6_B;
  real_T c6_b_x;
  real_T c6_e_y;
  real_T c6_c_x;
  real_T c6_f_y;
  real_T c6_g_y;
  real_T c6_e_b;
  real_T c6_h_y;
  real_T c6_f_b;
  real_T c6_i_y;
  real_T c6_g_b;
  real_T c6_j_y;
  real_T c6_h_b;
  real_T c6_k_y;
  real_T c6_d_a;
  real_T c6_i_b;
  real_T c6_l_y;
  real_T c6_b_A;
  real_T c6_b_B;
  real_T c6_d_x;
  real_T c6_m_y;
  real_T c6_e_x;
  real_T c6_n_y;
  real_T c6_o_y;
  real_T c6_j_b;
  real_T c6_p_y;
  int32_T c6_i45;
  real_T c6_b_q[6];
  int32_T c6_i46;
  real_T c6_c_d[6];
  int32_T c6_i47;
  real_T c6_e_a[6];
  int32_T c6_i48;
  real_T c6_c_alpha[6];
  real_T c6_dv11[3];
  int32_T c6_i49;
  real_T c6_f_x;
  real_T c6_g_x;
  real_T c6_k_b;
  real_T c6_q_y;
  int32_T c6_i;
  real_T c6_b_i;
  int32_T c6_c_i;
  int32_T c6_d_i;
  real_T c6_l_b;
  real_T c6_r_y;
  real_T c6_f_a;
  real_T c6_m_b;
  real_T c6_s_y;
  real_T c6_n_b;
  real_T c6_t_y;
  real_T c6_g_a;
  real_T c6_o_b;
  real_T c6_u_y;
  real_T c6_c_A;
  real_T c6_c_B;
  real_T c6_h_x;
  real_T c6_v_y;
  real_T c6_i_x;
  real_T c6_w_y;
  real_T c6_x_y;
  int32_T c6_i50;
  static real_T c6_dv12[144] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T c6_i51;
  int32_T c6_i52;
  real_T c6_b_W[144];
  real_T c6_dv13[144];
  int32_T c6_i53;
  int32_T c6_i54;
  int32_T c6_i55;
  int32_T c6_i56;
  int32_T c6_i57;
  int32_T c6_i58;
  real_T c6_h_a[72];
  int32_T c6_i59;
  real_T c6_p_b[144];
  int32_T c6_i60;
  real_T c6_y_y[72];
  int32_T c6_i61;
  real_T c6_i_a[72];
  int32_T c6_i62;
  real_T c6_q_b[144];
  int32_T c6_i63;
  real_T c6_r_b[72];
  int32_T c6_i64;
  int32_T c6_i65;
  int32_T c6_i66;
  int32_T c6_i67;
  real_T c6_ab_y[36];
  int32_T c6_i68;
  int32_T c6_i69;
  int32_T c6_i70;
  real_T c6_bb_y[36];
  real_T c6_dv14[36];
  int32_T c6_i71;
  int32_T c6_i72;
  int32_T c6_i73;
  int32_T c6_i74;
  int32_T c6_i75;
  real_T c6_cb_y[72];
  int32_T c6_i76;
  real_T c6_s_b[144];
  int32_T c6_i77;
  real_T c6_t_b[72];
  int32_T c6_i78;
  int32_T c6_i79;
  int32_T c6_i80;
  int32_T c6_i81;
  int32_T c6_i82;
  int32_T c6_i83;
  int32_T c6_i84;
  int32_T c6_i85;
  int32_T c6_i86;
  int32_T c6_i87;
  int32_T c6_i88;
  int32_T c6_i89;
  int32_T c6_i90;
  int32_T c6_i91;
  int32_T c6_i92;
  int32_T c6_i93;
  real_T c6_u_b[6];
  int32_T c6_i94;
  int32_T c6_i95;
  int32_T c6_i96;
  real_T c6_C[12];
  int32_T c6_i97;
  int32_T c6_i98;
  int32_T c6_i99;
  int32_T c6_i100;
  int32_T c6_i101;
  int32_T c6_i102;
  int32_T c6_e_i;
  int32_T c6_i103;
  int32_T c6_i104;
  int32_T c6_i105;
  int32_T c6_i106;
  real_T (*c6_b_zeta)[12];
  real_T (*c6_b_Jw_inv)[72];
  real_T (*c6_b_dH_trace)[12];
  real_T (*c6_b_dH)[144];
  real_T (*c6_b_previousDH)[144];
  real_T (*c6_b_ee_eta1)[3];
  real_T (*c6_b_eta2)[3];
  real_T (*c6_b_eta1)[3];
  real_T (*c6_c_q)[6];
  real_T (*c6_b_vee)[6];
  real_T (*c6_b_J)[72];
  c6_b_previousDH = (real_T (*)[144])ssGetInputPortSignal(chartInstance->S, 6);
  c6_b_dH = (real_T (*)[144])ssGetOutputPortSignal(chartInstance->S, 4);
  c6_b_dH_trace = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 3);
  c6_b_Jw_inv = (real_T (*)[72])ssGetOutputPortSignal(chartInstance->S, 2);
  c6_b_ee_eta1 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 5);
  c6_b_eta2 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 4);
  c6_b_eta1 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 3);
  c6_c_q = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 2);
  c6_b_vee = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 1);
  c6_b_zeta = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 1);
  c6_b_J = (real_T (*)[72])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 5U, chartInstance->c6_sfEvent);
  for (c6_i29 = 0; c6_i29 < 72; c6_i29++) {
    c6_J[c6_i29] = (*c6_b_J)[c6_i29];
  }

  for (c6_i30 = 0; c6_i30 < 6; c6_i30++) {
    c6_vee[c6_i30] = (*c6_b_vee)[c6_i30];
  }

  for (c6_i31 = 0; c6_i31 < 6; c6_i31++) {
    c6_q[c6_i31] = (*c6_c_q)[c6_i31];
  }

  for (c6_i32 = 0; c6_i32 < 3; c6_i32++) {
    c6_eta1[c6_i32] = (*c6_b_eta1)[c6_i32];
  }

  for (c6_i33 = 0; c6_i33 < 3; c6_i33++) {
    c6_eta2[c6_i33] = (*c6_b_eta2)[c6_i33];
  }

  for (c6_i34 = 0; c6_i34 < 3; c6_i34++) {
    c6_ee_eta1[c6_i34] = (*c6_b_ee_eta1)[c6_i34];
  }

  for (c6_i35 = 0; c6_i35 < 6; c6_i35++) {
    c6_b_qmin[c6_i35] = chartInstance->c6_qmin[c6_i35];
  }

  for (c6_i36 = 0; c6_i36 < 6; c6_i36++) {
    c6_b_qmax[c6_i36] = chartInstance->c6_qmax[c6_i36];
  }

  for (c6_i37 = 0; c6_i37 < 6; c6_i37++) {
    c6_b_d[c6_i37] = chartInstance->c6_d[c6_i37];
  }

  for (c6_i38 = 0; c6_i38 < 6; c6_i38++) {
    c6_b_a[c6_i38] = chartInstance->c6_a[c6_i38];
  }

  for (c6_i39 = 0; c6_i39 < 6; c6_i39++) {
    c6_b_alpha[c6_i39] = chartInstance->c6_alpha[c6_i39];
  }

  for (c6_i40 = 0; c6_i40 < 144; c6_i40++) {
    c6_previousDH[c6_i40] = (*c6_b_previousDH)[c6_i40];
  }

  sf_debug_symbol_scope_push_eml(0U, 36U, 36U, c6_debug_family_names,
    c6_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c6_dH_prev, 0U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_eta2_w_factors, 1U,
    c6_i_sf_marshallOut, c6_h_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c6_roll_max, 2U, c6_g_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c6_roll_min, 3U, c6_g_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c6_temp, 4U, c6_g_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c6_pitch_max, 5U, c6_g_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c6_pitch_min, 6U, c6_g_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c6_ee_pos, 7U, c6_e_sf_marshallOut,
    c6_g_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_xmax, 8U, c6_g_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_xmin, 9U, c6_g_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_x, 10U, c6_g_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_dev_x, 11U, c6_g_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c6_stddev_x, 12U, c6_g_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c6_W, 13U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_Wi, 14U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_Jt, 15U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_inv_prod, 16U, c6_h_sf_marshallOut,
    c6_f_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_J_inv, 17U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_nargin, 18U, c6_g_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_nargout, 19U, c6_g_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c6_J, 20U, c6_f_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c6_vee, 21U, c6_d_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c6_q, 22U, c6_d_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c6_eta1, 23U, c6_e_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c6_eta2, 24U, c6_e_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c6_ee_eta1, 25U, c6_e_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c6_b_qmin, 26U, c6_d_sf_marshallOut,
    c6_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_b_qmax, 27U, c6_d_sf_marshallOut,
    c6_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_b_d, 28U, c6_d_sf_marshallOut,
    c6_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_b_a, 29U, c6_d_sf_marshallOut,
    c6_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_b_alpha, 30U, c6_d_sf_marshallOut,
    c6_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c6_previousDH, 31U, c6_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c6_zeta, 32U, c6_b_sf_marshallOut,
    c6_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_Jw_inv, 33U, c6_c_sf_marshallOut,
    c6_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_dH_trace, 34U, c6_b_sf_marshallOut,
    c6_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_dH, 35U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 4);
  for (c6_i41 = 0; c6_i41 < 12; c6_i41++) {
    c6_dH_trace[c6_i41] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 5);
  for (c6_i42 = 0; c6_i42 < 144; c6_i42++) {
    c6_dH_prev[c6_i42] = c6_previousDH[c6_i42];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 7);
  c6_eye(chartInstance, c6_dv9);
  for (c6_i43 = 0; c6_i43 < 144; c6_i43++) {
    c6_dH[c6_i43] = c6_dv9[c6_i43];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 8);
  for (c6_i44 = 0; c6_i44 < 3; c6_i44++) {
    c6_eta2_w_factors[c6_i44] = c6_dv10[c6_i44];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 12);
  c6_roll_max = 1.3962634015954636;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 13);
  c6_roll_min = -1.3962634015954636;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 14);
  c6_b = c6_eta2[0];
  c6_y = 2.0 * c6_b;
  c6_b_b = (c6_y - c6_roll_max) - c6_roll_min;
  c6_b_y = 7.7982059465397393 * c6_b_b;
  c6_c_b = c6_power(chartInstance, c6_roll_max - c6_eta2[0]);
  c6_c_y = 4.0 * c6_c_b;
  c6_c_a = c6_c_y;
  c6_d_b = c6_power(chartInstance, c6_eta2[0] - c6_roll_min);
  c6_d_y = c6_c_a * c6_d_b;
  c6_A = c6_b_y;
  c6_B = c6_d_y;
  c6_b_x = c6_A;
  c6_e_y = c6_B;
  c6_c_x = c6_b_x;
  c6_f_y = c6_e_y;
  c6_g_y = c6_c_x / c6_f_y;
  c6_e_b = c6_eta2[0];
  c6_h_y = 300.0 * c6_e_b;
  c6_temp = (1.0 + c6_abs(chartInstance, c6_g_y)) + c6_b_power(chartInstance,
    c6_h_y);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 16);
  c6_dH[39] = 10.0 + c6_temp;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 20);
  c6_pitch_max = 1.3962634015954636;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 21);
  c6_pitch_min = -1.3962634015954636;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 22);
  c6_f_b = c6_eta2[1];
  c6_i_y = 2.0 * c6_f_b;
  c6_g_b = (c6_i_y - c6_pitch_max) - c6_pitch_min;
  c6_j_y = 7.7982059465397393 * c6_g_b;
  c6_h_b = c6_power(chartInstance, c6_pitch_max - c6_eta2[1]);
  c6_k_y = 4.0 * c6_h_b;
  c6_d_a = c6_k_y;
  c6_i_b = c6_power(chartInstance, c6_eta2[1] - c6_pitch_min);
  c6_l_y = c6_d_a * c6_i_b;
  c6_b_A = c6_j_y;
  c6_b_B = c6_l_y;
  c6_d_x = c6_b_A;
  c6_m_y = c6_b_B;
  c6_e_x = c6_d_x;
  c6_n_y = c6_m_y;
  c6_o_y = c6_e_x / c6_n_y;
  c6_j_b = c6_eta2[1];
  c6_p_y = 50.0 * c6_j_b;
  c6_temp = (1.0 + c6_abs(chartInstance, c6_o_y)) + c6_power(chartInstance,
    c6_p_y);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 24);
  c6_dH[52] = 10.0 + c6_temp;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 27);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 29);
  for (c6_i45 = 0; c6_i45 < 6; c6_i45++) {
    c6_b_q[c6_i45] = c6_q[c6_i45];
  }

  for (c6_i46 = 0; c6_i46 < 6; c6_i46++) {
    c6_c_d[c6_i46] = c6_b_d[c6_i46];
  }

  for (c6_i47 = 0; c6_i47 < 6; c6_i47++) {
    c6_e_a[c6_i47] = c6_b_a[c6_i47];
  }

  for (c6_i48 = 0; c6_i48 < 6; c6_i48++) {
    c6_c_alpha[c6_i48] = c6_b_alpha[c6_i48];
  }

  c6_ee_in_vehicle_frame(chartInstance, c6_b_q, c6_c_d, c6_e_a, c6_c_alpha,
    c6_dv11);
  for (c6_i49 = 0; c6_i49 < 3; c6_i49++) {
    c6_ee_pos[c6_i49] = c6_dv11[c6_i49];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 31);
  c6_xmax = 1.9;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 32);
  c6_xmin = 0.4;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 33);
  c6_x = c6_ee_pos[0];
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 34);
  c6_dev_x = c6_abs(chartInstance, c6_x - 2.2);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 35);
  c6_stddev_x = 0.2;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 36);
  c6_f_x = c6_rdivide(chartInstance, -c6_power(chartInstance, c6_dev_x),
                      0.080000000000000016);
  c6_g_x = c6_f_x;
  c6_g_x = muDoubleScalarExp(c6_g_x);
  c6_k_b = c6_g_x;
  c6_q_y = 2000.0 * c6_k_b;
  c6_temp = 200.0 + c6_q_y;
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 38);
  c6_dH[0] = 760.0;
  c6_i = 0;
  while (c6_i < 3) {
    c6_b_i = 4.0 + (real_T)c6_i;
    CV_EML_FOR(0, 1, 0, 1);
    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 41);
    c6_dH[(_SFD_EML_ARRAY_BOUNDS_CHECK("dH", (int32_T)_SFD_INTEGER_CHECK("i",
             c6_b_i), 1, 12, 1, 0) + 12 * (_SFD_EML_ARRAY_BOUNDS_CHECK("dH",
             (int32_T)_SFD_INTEGER_CHECK("i", c6_b_i), 1, 12, 2, 0) - 1)) - 1] =
      470.0;
    c6_i++;
    sf_mex_listen_for_ctrl_c(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 0, 0);
  c6_c_i = 0;
  while (c6_c_i < 2) {
    c6_b_i = 2.0 + (real_T)c6_c_i;
    CV_EML_FOR(0, 1, 1, 1);
    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 51);
    c6_dH[(_SFD_EML_ARRAY_BOUNDS_CHECK("dH", (int32_T)_SFD_INTEGER_CHECK("i",
             c6_b_i), 1, 12, 1, 0) + 12 * (_SFD_EML_ARRAY_BOUNDS_CHECK("dH",
             (int32_T)_SFD_INTEGER_CHECK("i", c6_b_i), 1, 12, 2, 0) - 1)) - 1] =
      400.0;
    c6_c_i++;
    sf_mex_listen_for_ctrl_c(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 1, 0);
  c6_d_i = 0;
  while (c6_d_i < 6) {
    c6_b_i = 1.0 + (real_T)c6_d_i;
    CV_EML_FOR(0, 1, 2, 1);
    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 57);
    c6_l_b = c6_q[_SFD_EML_ARRAY_BOUNDS_CHECK("q", (int32_T)_SFD_INTEGER_CHECK(
      "i", c6_b_i), 1, 6, 1, 0) - 1];
    c6_r_y = 2.0 * c6_l_b;
    c6_f_a = c6_power(chartInstance, c6_b_qmax[_SFD_EML_ARRAY_BOUNDS_CHECK(
      "qmax", (int32_T)_SFD_INTEGER_CHECK("i", c6_b_i), 1, 6, 1, 0) - 1] -
                      c6_b_qmin[_SFD_EML_ARRAY_BOUNDS_CHECK("qmin", (int32_T)
      _SFD_INTEGER_CHECK("i", c6_b_i), 1, 6, 1, 0) - 1]);
    c6_m_b = (c6_r_y - c6_b_qmax[_SFD_EML_ARRAY_BOUNDS_CHECK("qmax", (int32_T)
               _SFD_INTEGER_CHECK("i", c6_b_i), 1, 6, 1, 0) - 1]) -
      c6_b_qmin[_SFD_EML_ARRAY_BOUNDS_CHECK("qmin", (int32_T)_SFD_INTEGER_CHECK(
      "i", c6_b_i), 1, 6, 1, 0) - 1];
    c6_s_y = c6_f_a * c6_m_b;
    c6_n_b = c6_power(chartInstance, c6_b_qmax[_SFD_EML_ARRAY_BOUNDS_CHECK(
      "qmax", (int32_T)_SFD_INTEGER_CHECK("i", c6_b_i), 1, 6, 1, 0) - 1] -
                      c6_q[_SFD_EML_ARRAY_BOUNDS_CHECK("q", (int32_T)
      _SFD_INTEGER_CHECK("i", c6_b_i), 1, 6, 1, 0) - 1]);
    c6_t_y = 4.0 * c6_n_b;
    c6_g_a = c6_t_y;
    c6_o_b = c6_power(chartInstance, c6_q[_SFD_EML_ARRAY_BOUNDS_CHECK("q",
      (int32_T)_SFD_INTEGER_CHECK("i", c6_b_i), 1, 6, 1, 0) - 1] -
                      c6_b_qmin[_SFD_EML_ARRAY_BOUNDS_CHECK("qmin", (int32_T)
      _SFD_INTEGER_CHECK("i", c6_b_i), 1, 6, 1, 0) - 1]);
    c6_u_y = c6_g_a * c6_o_b;
    c6_c_A = c6_s_y;
    c6_c_B = c6_u_y;
    c6_h_x = c6_c_A;
    c6_v_y = c6_c_B;
    c6_i_x = c6_h_x;
    c6_w_y = c6_v_y;
    c6_x_y = c6_i_x / c6_w_y;
    c6_temp = c6_abs(chartInstance, c6_x_y);
    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 58);
    if (CV_EML_IF(0, 1, 0, c6_temp - c6_previousDH[(_SFD_EML_ARRAY_BOUNDS_CHECK(
           "previousDH", (int32_T)_SFD_INTEGER_CHECK("i+6", c6_b_i + 6.0), 1, 12,
           1, 0) + 12 * (_SFD_EML_ARRAY_BOUNDS_CHECK("previousDH", (int32_T)
            _SFD_INTEGER_CHECK("i+6", c6_b_i + 6.0), 1, 12, 2, 0) - 1)) - 1] <
                  0.0)) {
      _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 59);
      c6_dH[(_SFD_EML_ARRAY_BOUNDS_CHECK("dH", (int32_T)_SFD_INTEGER_CHECK("i+6",
               c6_b_i + 6.0), 1, 12, 1, 0) + 12 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
               "dH", (int32_T)_SFD_INTEGER_CHECK("i+6", c6_b_i + 6.0), 1, 12, 2,
               0) - 1)) - 1] = 0.0;
    } else {
      _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 61);
      c6_dH[(_SFD_EML_ARRAY_BOUNDS_CHECK("dH", (int32_T)_SFD_INTEGER_CHECK("i+6",
               c6_b_i + 6.0), 1, 12, 1, 0) + 12 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
               "dH", (int32_T)_SFD_INTEGER_CHECK("i+6", c6_b_i + 6.0), 1, 12, 2,
               0) - 1)) - 1] = c6_temp;
    }

    c6_d_i++;
    sf_mex_listen_for_ctrl_c(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 2, 0);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 78);
  for (c6_i50 = 0; c6_i50 < 144; c6_i50++) {
    c6_W[c6_i50] = c6_dv12[c6_i50];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 79);
  for (c6_i51 = 0; c6_i51 < 144; c6_i51++) {
    c6_W[c6_i51] += c6_dH[c6_i51];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 81);
  for (c6_i52 = 0; c6_i52 < 144; c6_i52++) {
    c6_b_W[c6_i52] = c6_W[c6_i52];
  }

  c6_inv(chartInstance, c6_b_W, c6_dv13);
  for (c6_i53 = 0; c6_i53 < 144; c6_i53++) {
    c6_Wi[c6_i53] = c6_dv13[c6_i53];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 82);
  c6_i54 = 0;
  for (c6_i55 = 0; c6_i55 < 6; c6_i55++) {
    c6_i56 = 0;
    for (c6_i57 = 0; c6_i57 < 12; c6_i57++) {
      c6_Jt[c6_i57 + c6_i54] = c6_J[c6_i56 + c6_i55];
      c6_i56 += 6;
    }

    c6_i54 += 12;
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 83);
  for (c6_i58 = 0; c6_i58 < 72; c6_i58++) {
    c6_h_a[c6_i58] = c6_J[c6_i58];
  }

  for (c6_i59 = 0; c6_i59 < 144; c6_i59++) {
    c6_p_b[c6_i59] = c6_Wi[c6_i59];
  }

  c6_c_eml_scalar_eg(chartInstance);
  c6_c_eml_scalar_eg(chartInstance);
  for (c6_i60 = 0; c6_i60 < 72; c6_i60++) {
    c6_y_y[c6_i60] = 0.0;
  }

  for (c6_i61 = 0; c6_i61 < 72; c6_i61++) {
    c6_i_a[c6_i61] = c6_h_a[c6_i61];
  }

  for (c6_i62 = 0; c6_i62 < 144; c6_i62++) {
    c6_q_b[c6_i62] = c6_p_b[c6_i62];
  }

  c6_c_eml_xgemm(chartInstance, c6_i_a, c6_q_b, c6_y_y);
  for (c6_i63 = 0; c6_i63 < 72; c6_i63++) {
    c6_r_b[c6_i63] = c6_Jt[c6_i63];
  }

  c6_d_eml_scalar_eg(chartInstance);
  c6_d_eml_scalar_eg(chartInstance);
  for (c6_i64 = 0; c6_i64 < 6; c6_i64++) {
    c6_i65 = 0;
    c6_i66 = 0;
    for (c6_i67 = 0; c6_i67 < 6; c6_i67++) {
      c6_ab_y[c6_i65 + c6_i64] = 0.0;
      c6_i68 = 0;
      for (c6_i69 = 0; c6_i69 < 12; c6_i69++) {
        c6_ab_y[c6_i65 + c6_i64] += c6_y_y[c6_i68 + c6_i64] * c6_r_b[c6_i69 +
          c6_i66];
        c6_i68 += 6;
      }

      c6_i65 += 6;
      c6_i66 += 12;
    }
  }

  for (c6_i70 = 0; c6_i70 < 36; c6_i70++) {
    c6_bb_y[c6_i70] = c6_ab_y[c6_i70];
  }

  c6_b_inv(chartInstance, c6_bb_y, c6_dv14);
  for (c6_i71 = 0; c6_i71 < 36; c6_i71++) {
    c6_inv_prod[c6_i71] = c6_dv14[c6_i71];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 84);
  for (c6_i72 = 0; c6_i72 < 72; c6_i72++) {
    c6_J_inv[c6_i72] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 85);
  for (c6_i73 = 0; c6_i73 < 144; c6_i73++) {
    c6_p_b[c6_i73] = c6_Wi[c6_i73];
  }

  for (c6_i74 = 0; c6_i74 < 72; c6_i74++) {
    c6_r_b[c6_i74] = c6_Jt[c6_i74];
  }

  c6_e_eml_scalar_eg(chartInstance);
  c6_e_eml_scalar_eg(chartInstance);
  for (c6_i75 = 0; c6_i75 < 72; c6_i75++) {
    c6_cb_y[c6_i75] = 0.0;
  }

  for (c6_i76 = 0; c6_i76 < 144; c6_i76++) {
    c6_s_b[c6_i76] = c6_p_b[c6_i76];
  }

  for (c6_i77 = 0; c6_i77 < 72; c6_i77++) {
    c6_t_b[c6_i77] = c6_r_b[c6_i77];
  }

  c6_d_eml_xgemm(chartInstance, c6_s_b, c6_t_b, c6_cb_y);
  for (c6_i78 = 0; c6_i78 < 36; c6_i78++) {
    c6_ab_y[c6_i78] = c6_inv_prod[c6_i78];
  }

  c6_f_eml_scalar_eg(chartInstance);
  c6_f_eml_scalar_eg(chartInstance);
  for (c6_i79 = 0; c6_i79 < 72; c6_i79++) {
    c6_J_inv[c6_i79] = 0.0;
  }

  for (c6_i80 = 0; c6_i80 < 72; c6_i80++) {
    c6_J_inv[c6_i80] = 0.0;
  }

  for (c6_i81 = 0; c6_i81 < 72; c6_i81++) {
    c6_r_b[c6_i81] = c6_J_inv[c6_i81];
  }

  for (c6_i82 = 0; c6_i82 < 72; c6_i82++) {
    c6_J_inv[c6_i82] = c6_r_b[c6_i82];
  }

  for (c6_i83 = 0; c6_i83 < 72; c6_i83++) {
    c6_r_b[c6_i83] = c6_J_inv[c6_i83];
  }

  for (c6_i84 = 0; c6_i84 < 72; c6_i84++) {
    c6_J_inv[c6_i84] = c6_r_b[c6_i84];
  }

  for (c6_i85 = 0; c6_i85 < 12; c6_i85++) {
    c6_i86 = 0;
    c6_i87 = 0;
    for (c6_i88 = 0; c6_i88 < 6; c6_i88++) {
      c6_J_inv[c6_i86 + c6_i85] = 0.0;
      c6_i89 = 0;
      for (c6_i90 = 0; c6_i90 < 6; c6_i90++) {
        c6_J_inv[c6_i86 + c6_i85] += c6_cb_y[c6_i89 + c6_i85] * c6_ab_y[c6_i90 +
          c6_i87];
        c6_i89 += 12;
      }

      c6_i86 += 12;
      c6_i87 += 6;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 86);
  for (c6_i91 = 0; c6_i91 < 72; c6_i91++) {
    c6_Jw_inv[c6_i91] = c6_J_inv[c6_i91];
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 87);
  for (c6_i92 = 0; c6_i92 < 72; c6_i92++) {
    c6_r_b[c6_i92] = c6_J_inv[c6_i92];
  }

  for (c6_i93 = 0; c6_i93 < 6; c6_i93++) {
    c6_u_b[c6_i93] = c6_vee[c6_i93];
  }

  c6_g_eml_scalar_eg(chartInstance);
  c6_g_eml_scalar_eg(chartInstance);
  for (c6_i94 = 0; c6_i94 < 12; c6_i94++) {
    c6_zeta[c6_i94] = 0.0;
  }

  for (c6_i95 = 0; c6_i95 < 12; c6_i95++) {
    c6_zeta[c6_i95] = 0.0;
  }

  for (c6_i96 = 0; c6_i96 < 12; c6_i96++) {
    c6_C[c6_i96] = c6_zeta[c6_i96];
  }

  for (c6_i97 = 0; c6_i97 < 12; c6_i97++) {
    c6_zeta[c6_i97] = c6_C[c6_i97];
  }

  for (c6_i98 = 0; c6_i98 < 12; c6_i98++) {
    c6_C[c6_i98] = c6_zeta[c6_i98];
  }

  for (c6_i99 = 0; c6_i99 < 12; c6_i99++) {
    c6_zeta[c6_i99] = c6_C[c6_i99];
  }

  for (c6_i100 = 0; c6_i100 < 12; c6_i100++) {
    c6_zeta[c6_i100] = 0.0;
    c6_i101 = 0;
    for (c6_i102 = 0; c6_i102 < 6; c6_i102++) {
      c6_zeta[c6_i100] += c6_r_b[c6_i101 + c6_i100] * c6_u_b[c6_i102];
      c6_i101 += 12;
    }
  }

  c6_e_i = 0;
  while (c6_e_i < 12) {
    c6_b_i = 1.0 + (real_T)c6_e_i;
    CV_EML_FOR(0, 1, 3, 1);
    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 91);
    c6_dH_trace[_SFD_EML_ARRAY_BOUNDS_CHECK("dH_trace", (int32_T)
      _SFD_INTEGER_CHECK("i", c6_b_i), 1, 12, 1, 0) - 1] = c6_dH
      [(_SFD_EML_ARRAY_BOUNDS_CHECK("dH", (int32_T)_SFD_INTEGER_CHECK("i",
          c6_b_i), 1, 12, 1, 0) + 12 * (_SFD_EML_ARRAY_BOUNDS_CHECK("dH",
          (int32_T)_SFD_INTEGER_CHECK("i", c6_b_i), 1, 12, 2, 0) - 1)) - 1];
    c6_e_i++;
    sf_mex_listen_for_ctrl_c(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 3, 0);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, -91);
  sf_debug_symbol_scope_pop();
  for (c6_i103 = 0; c6_i103 < 12; c6_i103++) {
    (*c6_b_zeta)[c6_i103] = c6_zeta[c6_i103];
  }

  for (c6_i104 = 0; c6_i104 < 72; c6_i104++) {
    (*c6_b_Jw_inv)[c6_i104] = c6_Jw_inv[c6_i104];
  }

  for (c6_i105 = 0; c6_i105 < 12; c6_i105++) {
    (*c6_b_dH_trace)[c6_i105] = c6_dH_trace[c6_i105];
  }

  for (c6_i106 = 0; c6_i106 < 144; c6_i106++) {
    (*c6_b_dH)[c6_i106] = c6_dH[c6_i106];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 5U, chartInstance->c6_sfEvent);
}

static void initSimStructsc6_kinematics(SFc6_kinematicsInstanceStruct
  *chartInstance)
{
}

static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber)
{
  _SFD_SCRIPT_TRANSLATION(c6_chartNumber, 0U, sf_debug_get_script_id(
    "/home/simena/Dropbox/master_project/code/matlab_kinematics/ee_in_vehicle_frame.m"));
  _SFD_SCRIPT_TRANSLATION(c6_chartNumber, 1U, sf_debug_get_script_id(
    "/home/simena/Dropbox/master_project/code/matlab/gnc_mfiles/Rzyx.m"));
}

static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i107;
  int32_T c6_i108;
  int32_T c6_i109;
  real_T c6_b_inData[144];
  int32_T c6_i110;
  int32_T c6_i111;
  int32_T c6_i112;
  real_T c6_u[144];
  const mxArray *c6_y = NULL;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i107 = 0;
  for (c6_i108 = 0; c6_i108 < 12; c6_i108++) {
    for (c6_i109 = 0; c6_i109 < 12; c6_i109++) {
      c6_b_inData[c6_i109 + c6_i107] = (*(real_T (*)[144])c6_inData)[c6_i109 +
        c6_i107];
    }

    c6_i107 += 12;
  }

  c6_i110 = 0;
  for (c6_i111 = 0; c6_i111 < 12; c6_i111++) {
    for (c6_i112 = 0; c6_i112 < 12; c6_i112++) {
      c6_u[c6_i112 + c6_i110] = c6_b_inData[c6_i112 + c6_i110];
    }

    c6_i110 += 12;
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 12, 12), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_dH, const char_T *c6_identifier, real_T c6_y[144])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_dH), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_dH);
}

static void c6_b_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[144])
{
  real_T c6_dv15[144];
  int32_T c6_i113;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv15, 1, 0, 0U, 1, 0U, 2, 12,
                12);
  for (c6_i113 = 0; c6_i113 < 144; c6_i113++) {
    c6_y[c6_i113] = c6_dv15[c6_i113];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_dH;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[144];
  int32_T c6_i114;
  int32_T c6_i115;
  int32_T c6_i116;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_dH = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_dH), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_dH);
  c6_i114 = 0;
  for (c6_i115 = 0; c6_i115 < 12; c6_i115++) {
    for (c6_i116 = 0; c6_i116 < 12; c6_i116++) {
      (*(real_T (*)[144])c6_outData)[c6_i116 + c6_i114] = c6_y[c6_i116 + c6_i114];
    }

    c6_i114 += 12;
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i117;
  real_T c6_b_inData[12];
  int32_T c6_i118;
  real_T c6_u[12];
  const mxArray *c6_y = NULL;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i117 = 0; c6_i117 < 12; c6_i117++) {
    c6_b_inData[c6_i117] = (*(real_T (*)[12])c6_inData)[c6_i117];
  }

  for (c6_i118 = 0; c6_i118 < 12; c6_i118++) {
    c6_u[c6_i118] = c6_b_inData[c6_i118];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 12), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_c_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_dH_trace, const char_T *c6_identifier, real_T c6_y[12])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_dH_trace), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_dH_trace);
}

static void c6_d_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[12])
{
  real_T c6_dv16[12];
  int32_T c6_i119;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv16, 1, 0, 0U, 1, 0U, 1, 12);
  for (c6_i119 = 0; c6_i119 < 12; c6_i119++) {
    c6_y[c6_i119] = c6_dv16[c6_i119];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_dH_trace;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[12];
  int32_T c6_i120;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_dH_trace = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_dH_trace), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_dH_trace);
  for (c6_i120 = 0; c6_i120 < 12; c6_i120++) {
    (*(real_T (*)[12])c6_outData)[c6_i120] = c6_y[c6_i120];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_c_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i121;
  int32_T c6_i122;
  int32_T c6_i123;
  real_T c6_b_inData[72];
  int32_T c6_i124;
  int32_T c6_i125;
  int32_T c6_i126;
  real_T c6_u[72];
  const mxArray *c6_y = NULL;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i121 = 0;
  for (c6_i122 = 0; c6_i122 < 6; c6_i122++) {
    for (c6_i123 = 0; c6_i123 < 12; c6_i123++) {
      c6_b_inData[c6_i123 + c6_i121] = (*(real_T (*)[72])c6_inData)[c6_i123 +
        c6_i121];
    }

    c6_i121 += 12;
  }

  c6_i124 = 0;
  for (c6_i125 = 0; c6_i125 < 6; c6_i125++) {
    for (c6_i126 = 0; c6_i126 < 12; c6_i126++) {
      c6_u[c6_i126 + c6_i124] = c6_b_inData[c6_i126 + c6_i124];
    }

    c6_i124 += 12;
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 12, 6), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_e_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_Jw_inv, const char_T *c6_identifier, real_T c6_y[72])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_Jw_inv), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_Jw_inv);
}

static void c6_f_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[72])
{
  real_T c6_dv17[72];
  int32_T c6_i127;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv17, 1, 0, 0U, 1, 0U, 2, 12,
                6);
  for (c6_i127 = 0; c6_i127 < 72; c6_i127++) {
    c6_y[c6_i127] = c6_dv17[c6_i127];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_Jw_inv;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[72];
  int32_T c6_i128;
  int32_T c6_i129;
  int32_T c6_i130;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_Jw_inv = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_Jw_inv), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_Jw_inv);
  c6_i128 = 0;
  for (c6_i129 = 0; c6_i129 < 6; c6_i129++) {
    for (c6_i130 = 0; c6_i130 < 12; c6_i130++) {
      (*(real_T (*)[72])c6_outData)[c6_i130 + c6_i128] = c6_y[c6_i130 + c6_i128];
    }

    c6_i128 += 12;
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i131;
  real_T c6_b_inData[6];
  int32_T c6_i132;
  real_T c6_u[6];
  const mxArray *c6_y = NULL;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i131 = 0; c6_i131 < 6; c6_i131++) {
    c6_b_inData[c6_i131] = (*(real_T (*)[6])c6_inData)[c6_i131];
  }

  for (c6_i132 = 0; c6_i132 < 6; c6_i132++) {
    c6_u[c6_i132] = c6_b_inData[c6_i132];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_g_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[6])
{
  real_T c6_dv18[6];
  int32_T c6_i133;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv18, 1, 0, 0U, 1, 0U, 1, 6);
  for (c6_i133 = 0; c6_i133 < 6; c6_i133++) {
    c6_y[c6_i133] = c6_dv18[c6_i133];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_alpha;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[6];
  int32_T c6_i134;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_b_alpha = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_alpha), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_b_alpha);
  for (c6_i134 = 0; c6_i134 < 6; c6_i134++) {
    (*(real_T (*)[6])c6_outData)[c6_i134] = c6_y[c6_i134];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_e_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i135;
  real_T c6_b_inData[3];
  int32_T c6_i136;
  real_T c6_u[3];
  const mxArray *c6_y = NULL;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i135 = 0; c6_i135 < 3; c6_i135++) {
    c6_b_inData[c6_i135] = (*(real_T (*)[3])c6_inData)[c6_i135];
  }

  for (c6_i136 = 0; c6_i136 < 3; c6_i136++) {
    c6_u[c6_i136] = c6_b_inData[c6_i136];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static const mxArray *c6_f_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i137;
  int32_T c6_i138;
  int32_T c6_i139;
  real_T c6_b_inData[72];
  int32_T c6_i140;
  int32_T c6_i141;
  int32_T c6_i142;
  real_T c6_u[72];
  const mxArray *c6_y = NULL;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i137 = 0;
  for (c6_i138 = 0; c6_i138 < 12; c6_i138++) {
    for (c6_i139 = 0; c6_i139 < 6; c6_i139++) {
      c6_b_inData[c6_i139 + c6_i137] = (*(real_T (*)[72])c6_inData)[c6_i139 +
        c6_i137];
    }

    c6_i137 += 6;
  }

  c6_i140 = 0;
  for (c6_i141 = 0; c6_i141 < 12; c6_i141++) {
    for (c6_i142 = 0; c6_i142 < 6; c6_i142++) {
      c6_u[c6_i142 + c6_i140] = c6_b_inData[c6_i142 + c6_i140];
    }

    c6_i140 += 6;
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 6, 12), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static const mxArray *c6_g_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  real_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(real_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static real_T c6_h_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  real_T c6_y;
  real_T c6_d0;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_d0, 1, 0, 0U, 0, 0U, 0);
  c6_y = c6_d0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_nargout;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_nargout = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_nargout), &c6_thisId);
  sf_mex_destroy(&c6_nargout);
  *(real_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_h_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i143;
  int32_T c6_i144;
  int32_T c6_i145;
  real_T c6_b_inData[36];
  int32_T c6_i146;
  int32_T c6_i147;
  int32_T c6_i148;
  real_T c6_u[36];
  const mxArray *c6_y = NULL;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i143 = 0;
  for (c6_i144 = 0; c6_i144 < 6; c6_i144++) {
    for (c6_i145 = 0; c6_i145 < 6; c6_i145++) {
      c6_b_inData[c6_i145 + c6_i143] = (*(real_T (*)[36])c6_inData)[c6_i145 +
        c6_i143];
    }

    c6_i143 += 6;
  }

  c6_i146 = 0;
  for (c6_i147 = 0; c6_i147 < 6; c6_i147++) {
    for (c6_i148 = 0; c6_i148 < 6; c6_i148++) {
      c6_u[c6_i148 + c6_i146] = c6_b_inData[c6_i148 + c6_i146];
    }

    c6_i146 += 6;
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 6, 6), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_i_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[36])
{
  real_T c6_dv19[36];
  int32_T c6_i149;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv19, 1, 0, 0U, 1, 0U, 2, 6, 6);
  for (c6_i149 = 0; c6_i149 < 36; c6_i149++) {
    c6_y[c6_i149] = c6_dv19[c6_i149];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_inv_prod;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[36];
  int32_T c6_i150;
  int32_T c6_i151;
  int32_T c6_i152;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_inv_prod = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_inv_prod), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_inv_prod);
  c6_i150 = 0;
  for (c6_i151 = 0; c6_i151 < 6; c6_i151++) {
    for (c6_i152 = 0; c6_i152 < 6; c6_i152++) {
      (*(real_T (*)[36])c6_outData)[c6_i152 + c6_i150] = c6_y[c6_i152 + c6_i150];
    }

    c6_i150 += 6;
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static void c6_j_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[3])
{
  real_T c6_dv20[3];
  int32_T c6_i153;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv20, 1, 0, 0U, 1, 0U, 1, 3);
  for (c6_i153 = 0; c6_i153 < 3; c6_i153++) {
    c6_y[c6_i153] = c6_dv20[c6_i153];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_ee_pos;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[3];
  int32_T c6_i154;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_ee_pos = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_ee_pos), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_ee_pos);
  for (c6_i154 = 0; c6_i154 < 3; c6_i154++) {
    (*(real_T (*)[3])c6_outData)[c6_i154] = c6_y[c6_i154];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_i_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i155;
  real_T c6_b_inData[3];
  int32_T c6_i156;
  real_T c6_u[3];
  const mxArray *c6_y = NULL;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i155 = 0; c6_i155 < 3; c6_i155++) {
    c6_b_inData[c6_i155] = (*(real_T (*)[3])c6_inData)[c6_i155];
  }

  for (c6_i156 = 0; c6_i156 < 3; c6_i156++) {
    c6_u[c6_i156] = c6_b_inData[c6_i156];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 1, 3), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_k_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[3])
{
  real_T c6_dv21[3];
  int32_T c6_i157;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv21, 1, 0, 0U, 1, 0U, 2, 1, 3);
  for (c6_i157 = 0; c6_i157 < 3; c6_i157++) {
    c6_y[c6_i157] = c6_dv21[c6_i157];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_eta2_w_factors;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[3];
  int32_T c6_i158;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_eta2_w_factors = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_eta2_w_factors), &c6_thisId,
                        c6_y);
  sf_mex_destroy(&c6_eta2_w_factors);
  for (c6_i158 = 0; c6_i158 < 3; c6_i158++) {
    (*(real_T (*)[3])c6_outData)[c6_i158] = c6_y[c6_i158];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_j_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i159;
  int32_T c6_i160;
  int32_T c6_i161;
  real_T c6_b_inData[16];
  int32_T c6_i162;
  int32_T c6_i163;
  int32_T c6_i164;
  real_T c6_u[16];
  const mxArray *c6_y = NULL;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i159 = 0;
  for (c6_i160 = 0; c6_i160 < 4; c6_i160++) {
    for (c6_i161 = 0; c6_i161 < 4; c6_i161++) {
      c6_b_inData[c6_i161 + c6_i159] = (*(real_T (*)[16])c6_inData)[c6_i161 +
        c6_i159];
    }

    c6_i159 += 4;
  }

  c6_i162 = 0;
  for (c6_i163 = 0; c6_i163 < 4; c6_i163++) {
    for (c6_i164 = 0; c6_i164 < 4; c6_i164++) {
      c6_u[c6_i164 + c6_i162] = c6_b_inData[c6_i164 + c6_i162];
    }

    c6_i162 += 4;
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_l_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[16])
{
  real_T c6_dv22[16];
  int32_T c6_i165;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv22, 1, 0, 0U, 1, 0U, 2, 4, 4);
  for (c6_i165 = 0; c6_i165 < 16; c6_i165++) {
    c6_y[c6_i165] = c6_dv22[c6_i165];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_mat;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[16];
  int32_T c6_i166;
  int32_T c6_i167;
  int32_T c6_i168;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_mat = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_mat), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_mat);
  c6_i166 = 0;
  for (c6_i167 = 0; c6_i167 < 4; c6_i167++) {
    for (c6_i168 = 0; c6_i168 < 4; c6_i168++) {
      (*(real_T (*)[16])c6_outData)[c6_i168 + c6_i166] = c6_y[c6_i168 + c6_i166];
    }

    c6_i166 += 4;
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_k_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i169;
  int32_T c6_i170;
  int32_T c6_i171;
  real_T c6_b_inData[9];
  int32_T c6_i172;
  int32_T c6_i173;
  int32_T c6_i174;
  real_T c6_u[9];
  const mxArray *c6_y = NULL;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i169 = 0;
  for (c6_i170 = 0; c6_i170 < 3; c6_i170++) {
    for (c6_i171 = 0; c6_i171 < 3; c6_i171++) {
      c6_b_inData[c6_i171 + c6_i169] = (*(real_T (*)[9])c6_inData)[c6_i171 +
        c6_i169];
    }

    c6_i169 += 3;
  }

  c6_i172 = 0;
  for (c6_i173 = 0; c6_i173 < 3; c6_i173++) {
    for (c6_i174 = 0; c6_i174 < 3; c6_i174++) {
      c6_u[c6_i174 + c6_i172] = c6_b_inData[c6_i174 + c6_i172];
    }

    c6_i172 += 3;
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 3, 3), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static void c6_m_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, real_T c6_y[9])
{
  real_T c6_dv23[9];
  int32_T c6_i175;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv23, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c6_i175 = 0; c6_i175 < 9; c6_i175++) {
    c6_y[c6_i175] = c6_dv23[c6_i175];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_R;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[9];
  int32_T c6_i176;
  int32_T c6_i177;
  int32_T c6_i178;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_R = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_m_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_R), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_R);
  c6_i176 = 0;
  for (c6_i177 = 0; c6_i177 < 3; c6_i177++) {
    for (c6_i178 = 0; c6_i178 < 3; c6_i178++) {
      (*(real_T (*)[9])c6_outData)[c6_i178 + c6_i176] = c6_y[c6_i178 + c6_i176];
    }

    c6_i176 += 3;
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

const mxArray *sf_c6_kinematics_get_eml_resolved_functions_info(void)
{
  const mxArray *c6_nameCaptureInfo;
  c6_ResolvedFunctionInfo c6_info[156];
  const mxArray *c6_m0 = NULL;
  int32_T c6_i179;
  c6_ResolvedFunctionInfo *c6_r0;
  c6_nameCaptureInfo = NULL;
  c6_nameCaptureInfo = NULL;
  c6_info_helper(c6_info);
  c6_b_info_helper(c6_info);
  c6_c_info_helper(c6_info);
  sf_mex_assign(&c6_m0, sf_mex_createstruct("nameCaptureInfo", 1, 156), FALSE);
  for (c6_i179 = 0; c6_i179 < 156; c6_i179++) {
    c6_r0 = &c6_info[c6_i179];
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c6_r0->context)), "context", "nameCaptureInfo",
                    c6_i179);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c6_r0->name)), "name", "nameCaptureInfo", c6_i179);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c6_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c6_i179);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", c6_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c6_r0->resolved)), "resolved", "nameCaptureInfo",
                    c6_i179);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c6_i179);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c6_i179);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c6_i179);
    sf_mex_addfield(c6_m0, sf_mex_create("nameCaptureInfo", &c6_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c6_i179);
  }

  sf_mex_assign(&c6_nameCaptureInfo, c6_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c6_nameCaptureInfo);
  return c6_nameCaptureInfo;
}

static void c6_info_helper(c6_ResolvedFunctionInfo c6_info[156])
{
  c6_info[0].context = "";
  c6_info[0].name = "eye";
  c6_info[0].dominantType = "double";
  c6_info[0].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m";
  c6_info[0].fileTimeLo = 1286818688U;
  c6_info[0].fileTimeHi = 0U;
  c6_info[0].mFileTimeLo = 0U;
  c6_info[0].mFileTimeHi = 0U;
  c6_info[1].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c6_info[1].name = "eml_assert_valid_size_arg";
  c6_info[1].dominantType = "double";
  c6_info[1].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c6_info[1].fileTimeLo = 1286818694U;
  c6_info[1].fileTimeHi = 0U;
  c6_info[1].mFileTimeLo = 0U;
  c6_info[1].mFileTimeHi = 0U;
  c6_info[2].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral";
  c6_info[2].name = "isinf";
  c6_info[2].dominantType = "double";
  c6_info[2].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isinf.m";
  c6_info[2].fileTimeLo = 1286818760U;
  c6_info[2].fileTimeHi = 0U;
  c6_info[2].mFileTimeLo = 0U;
  c6_info[2].mFileTimeHi = 0U;
  c6_info[3].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!numel_for_size";
  c6_info[3].name = "mtimes";
  c6_info[3].dominantType = "double";
  c6_info[3].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[3].fileTimeLo = 1289519692U;
  c6_info[3].fileTimeHi = 0U;
  c6_info[3].mFileTimeLo = 0U;
  c6_info[3].mFileTimeHi = 0U;
  c6_info[4].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c6_info[4].name = "eml_index_class";
  c6_info[4].dominantType = "";
  c6_info[4].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[4].fileTimeLo = 1286818778U;
  c6_info[4].fileTimeHi = 0U;
  c6_info[4].mFileTimeLo = 0U;
  c6_info[4].mFileTimeHi = 0U;
  c6_info[5].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m";
  c6_info[5].name = "intmax";
  c6_info[5].dominantType = "char";
  c6_info[5].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/intmax.m";
  c6_info[5].fileTimeLo = 1311255316U;
  c6_info[5].fileTimeHi = 0U;
  c6_info[5].mFileTimeLo = 0U;
  c6_info[5].mFileTimeHi = 0U;
  c6_info[6].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c6_info[6].name = "eml_is_float_class";
  c6_info[6].dominantType = "char";
  c6_info[6].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c6_info[6].fileTimeLo = 1286818782U;
  c6_info[6].fileTimeHi = 0U;
  c6_info[6].mFileTimeLo = 0U;
  c6_info[6].mFileTimeHi = 0U;
  c6_info[7].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c6_info[7].name = "min";
  c6_info[7].dominantType = "double";
  c6_info[7].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/min.m";
  c6_info[7].fileTimeLo = 1311255318U;
  c6_info[7].fileTimeHi = 0U;
  c6_info[7].mFileTimeLo = 0U;
  c6_info[7].mFileTimeHi = 0U;
  c6_info[8].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/min.m";
  c6_info[8].name = "eml_min_or_max";
  c6_info[8].dominantType = "char";
  c6_info[8].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c6_info[8].fileTimeLo = 1303146212U;
  c6_info[8].fileTimeHi = 0U;
  c6_info[8].mFileTimeLo = 0U;
  c6_info[8].mFileTimeHi = 0U;
  c6_info[9].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c6_info[9].name = "eml_scalar_eg";
  c6_info[9].dominantType = "double";
  c6_info[9].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[9].fileTimeLo = 1286818796U;
  c6_info[9].fileTimeHi = 0U;
  c6_info[9].mFileTimeLo = 0U;
  c6_info[9].mFileTimeHi = 0U;
  c6_info[10].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c6_info[10].name = "eml_scalexp_alloc";
  c6_info[10].dominantType = "double";
  c6_info[10].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c6_info[10].fileTimeLo = 1286818796U;
  c6_info[10].fileTimeHi = 0U;
  c6_info[10].mFileTimeLo = 0U;
  c6_info[10].mFileTimeHi = 0U;
  c6_info[11].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c6_info[11].name = "eml_index_class";
  c6_info[11].dominantType = "";
  c6_info[11].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[11].fileTimeLo = 1286818778U;
  c6_info[11].fileTimeHi = 0U;
  c6_info[11].mFileTimeLo = 0U;
  c6_info[11].mFileTimeHi = 0U;
  c6_info[12].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c6_info[12].name = "eml_scalar_eg";
  c6_info[12].dominantType = "double";
  c6_info[12].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[12].fileTimeLo = 1286818796U;
  c6_info[12].fileTimeHi = 0U;
  c6_info[12].mFileTimeLo = 0U;
  c6_info[12].mFileTimeHi = 0U;
  c6_info[13].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c6_info[13].name = "eml_index_class";
  c6_info[13].dominantType = "";
  c6_info[13].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[13].fileTimeLo = 1286818778U;
  c6_info[13].fileTimeHi = 0U;
  c6_info[13].mFileTimeLo = 0U;
  c6_info[13].mFileTimeHi = 0U;
  c6_info[14].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m!eye_internal";
  c6_info[14].name = "eml_int_forloop_overflow_check";
  c6_info[14].dominantType = "";
  c6_info[14].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c6_info[14].fileTimeLo = 1311255316U;
  c6_info[14].fileTimeHi = 0U;
  c6_info[14].mFileTimeLo = 0U;
  c6_info[14].mFileTimeHi = 0U;
  c6_info[15].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c6_info[15].name = "intmax";
  c6_info[15].dominantType = "char";
  c6_info[15].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/intmax.m";
  c6_info[15].fileTimeLo = 1311255316U;
  c6_info[15].fileTimeHi = 0U;
  c6_info[15].mFileTimeLo = 0U;
  c6_info[15].mFileTimeHi = 0U;
  c6_info[16].context = "";
  c6_info[16].name = "mrdivide";
  c6_info[16].dominantType = "double";
  c6_info[16].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c6_info[16].fileTimeLo = 1325124138U;
  c6_info[16].fileTimeHi = 0U;
  c6_info[16].mFileTimeLo = 1319729966U;
  c6_info[16].mFileTimeHi = 0U;
  c6_info[17].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c6_info[17].name = "rdivide";
  c6_info[17].dominantType = "double";
  c6_info[17].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/rdivide.m";
  c6_info[17].fileTimeLo = 1286818844U;
  c6_info[17].fileTimeHi = 0U;
  c6_info[17].mFileTimeLo = 0U;
  c6_info[17].mFileTimeHi = 0U;
  c6_info[18].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/rdivide.m";
  c6_info[18].name = "eml_div";
  c6_info[18].dominantType = "double";
  c6_info[18].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_div.m";
  c6_info[18].fileTimeLo = 1313347810U;
  c6_info[18].fileTimeHi = 0U;
  c6_info[18].mFileTimeLo = 0U;
  c6_info[18].mFileTimeHi = 0U;
  c6_info[19].context = "";
  c6_info[19].name = "mtimes";
  c6_info[19].dominantType = "double";
  c6_info[19].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[19].fileTimeLo = 1289519692U;
  c6_info[19].fileTimeHi = 0U;
  c6_info[19].mFileTimeLo = 0U;
  c6_info[19].mFileTimeHi = 0U;
  c6_info[20].context = "";
  c6_info[20].name = "power";
  c6_info[20].dominantType = "double";
  c6_info[20].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/power.m";
  c6_info[20].fileTimeLo = 1307651240U;
  c6_info[20].fileTimeHi = 0U;
  c6_info[20].mFileTimeLo = 0U;
  c6_info[20].mFileTimeHi = 0U;
  c6_info[21].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/power.m";
  c6_info[21].name = "eml_scalar_eg";
  c6_info[21].dominantType = "double";
  c6_info[21].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[21].fileTimeLo = 1286818796U;
  c6_info[21].fileTimeHi = 0U;
  c6_info[21].mFileTimeLo = 0U;
  c6_info[21].mFileTimeHi = 0U;
  c6_info[22].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/power.m";
  c6_info[22].name = "eml_scalexp_alloc";
  c6_info[22].dominantType = "double";
  c6_info[22].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c6_info[22].fileTimeLo = 1286818796U;
  c6_info[22].fileTimeHi = 0U;
  c6_info[22].mFileTimeLo = 0U;
  c6_info[22].mFileTimeHi = 0U;
  c6_info[23].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/power.m";
  c6_info[23].name = "eml_scalar_floor";
  c6_info[23].dominantType = "double";
  c6_info[23].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c6_info[23].fileTimeLo = 1286818726U;
  c6_info[23].fileTimeHi = 0U;
  c6_info[23].mFileTimeLo = 0U;
  c6_info[23].mFileTimeHi = 0U;
  c6_info[24].context = "";
  c6_info[24].name = "abs";
  c6_info[24].dominantType = "double";
  c6_info[24].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c6_info[24].fileTimeLo = 1286818694U;
  c6_info[24].fileTimeHi = 0U;
  c6_info[24].mFileTimeLo = 0U;
  c6_info[24].mFileTimeHi = 0U;
  c6_info[25].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c6_info[25].name = "eml_scalar_abs";
  c6_info[25].dominantType = "double";
  c6_info[25].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c6_info[25].fileTimeLo = 1286818712U;
  c6_info[25].fileTimeHi = 0U;
  c6_info[25].mFileTimeLo = 0U;
  c6_info[25].mFileTimeHi = 0U;
  c6_info[26].context = "";
  c6_info[26].name = "ee_in_vehicle_frame";
  c6_info[26].dominantType = "double";
  c6_info[26].resolved =
    "[E]/home/simena/Dropbox/master_project/code/matlab_kinematics/ee_in_vehicle_frame.m";
  c6_info[26].fileTimeLo = 1385744105U;
  c6_info[26].fileTimeHi = 0U;
  c6_info[26].mFileTimeLo = 0U;
  c6_info[26].mFileTimeHi = 0U;
  c6_info[27].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab_kinematics/ee_in_vehicle_frame.m";
  c6_info[27].name = "eye";
  c6_info[27].dominantType = "double";
  c6_info[27].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m";
  c6_info[27].fileTimeLo = 1286818688U;
  c6_info[27].fileTimeHi = 0U;
  c6_info[27].mFileTimeLo = 0U;
  c6_info[27].mFileTimeHi = 0U;
  c6_info[28].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab_kinematics/ee_in_vehicle_frame.m!dh_homogenous";
  c6_info[28].name = "eye";
  c6_info[28].dominantType = "double";
  c6_info[28].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eye.m";
  c6_info[28].fileTimeLo = 1286818688U;
  c6_info[28].fileTimeHi = 0U;
  c6_info[28].mFileTimeLo = 0U;
  c6_info[28].mFileTimeHi = 0U;
  c6_info[29].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab_kinematics/ee_in_vehicle_frame.m!dh_homogenous";
  c6_info[29].name = "cos";
  c6_info[29].dominantType = "double";
  c6_info[29].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/cos.m";
  c6_info[29].fileTimeLo = 1286818706U;
  c6_info[29].fileTimeHi = 0U;
  c6_info[29].mFileTimeLo = 0U;
  c6_info[29].mFileTimeHi = 0U;
  c6_info[30].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/cos.m";
  c6_info[30].name = "eml_scalar_cos";
  c6_info[30].dominantType = "double";
  c6_info[30].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m";
  c6_info[30].fileTimeLo = 1286818722U;
  c6_info[30].fileTimeHi = 0U;
  c6_info[30].mFileTimeLo = 0U;
  c6_info[30].mFileTimeHi = 0U;
  c6_info[31].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab_kinematics/ee_in_vehicle_frame.m!dh_homogenous";
  c6_info[31].name = "sin";
  c6_info[31].dominantType = "double";
  c6_info[31].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/sin.m";
  c6_info[31].fileTimeLo = 1286818750U;
  c6_info[31].fileTimeHi = 0U;
  c6_info[31].mFileTimeLo = 0U;
  c6_info[31].mFileTimeHi = 0U;
  c6_info[32].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/sin.m";
  c6_info[32].name = "eml_scalar_sin";
  c6_info[32].dominantType = "double";
  c6_info[32].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m";
  c6_info[32].fileTimeLo = 1286818736U;
  c6_info[32].fileTimeHi = 0U;
  c6_info[32].mFileTimeLo = 0U;
  c6_info[32].mFileTimeHi = 0U;
  c6_info[33].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab_kinematics/ee_in_vehicle_frame.m!dh_homogenous";
  c6_info[33].name = "mtimes";
  c6_info[33].dominantType = "double";
  c6_info[33].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[33].fileTimeLo = 1289519692U;
  c6_info[33].fileTimeHi = 0U;
  c6_info[33].mFileTimeLo = 0U;
  c6_info[33].mFileTimeHi = 0U;
  c6_info[34].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab_kinematics/ee_in_vehicle_frame.m";
  c6_info[34].name = "mtimes";
  c6_info[34].dominantType = "double";
  c6_info[34].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[34].fileTimeLo = 1289519692U;
  c6_info[34].fileTimeHi = 0U;
  c6_info[34].mFileTimeLo = 0U;
  c6_info[34].mFileTimeHi = 0U;
  c6_info[35].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[35].name = "eml_index_class";
  c6_info[35].dominantType = "";
  c6_info[35].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[35].fileTimeLo = 1286818778U;
  c6_info[35].fileTimeHi = 0U;
  c6_info[35].mFileTimeLo = 0U;
  c6_info[35].mFileTimeHi = 0U;
  c6_info[36].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[36].name = "eml_scalar_eg";
  c6_info[36].dominantType = "double";
  c6_info[36].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[36].fileTimeLo = 1286818796U;
  c6_info[36].fileTimeHi = 0U;
  c6_info[36].mFileTimeLo = 0U;
  c6_info[36].mFileTimeHi = 0U;
  c6_info[37].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[37].name = "eml_xgemm";
  c6_info[37].dominantType = "int32";
  c6_info[37].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c6_info[37].fileTimeLo = 1299076772U;
  c6_info[37].fileTimeHi = 0U;
  c6_info[37].mFileTimeLo = 0U;
  c6_info[37].mFileTimeHi = 0U;
  c6_info[38].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c6_info[38].name = "eml_blas_inline";
  c6_info[38].dominantType = "";
  c6_info[38].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c6_info[38].fileTimeLo = 1299076768U;
  c6_info[38].fileTimeHi = 0U;
  c6_info[38].mFileTimeLo = 0U;
  c6_info[38].mFileTimeHi = 0U;
  c6_info[39].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c6_info[39].name = "mtimes";
  c6_info[39].dominantType = "double";
  c6_info[39].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[39].fileTimeLo = 1289519692U;
  c6_info[39].fileTimeHi = 0U;
  c6_info[39].mFileTimeLo = 0U;
  c6_info[39].mFileTimeHi = 0U;
  c6_info[40].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c6_info[40].name = "eml_scalar_eg";
  c6_info[40].dominantType = "double";
  c6_info[40].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[40].fileTimeLo = 1286818796U;
  c6_info[40].fileTimeHi = 0U;
  c6_info[40].mFileTimeLo = 0U;
  c6_info[40].mFileTimeHi = 0U;
  c6_info[41].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c6_info[41].name = "eml_refblas_xgemm";
  c6_info[41].dominantType = "int32";
  c6_info[41].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c6_info[41].fileTimeLo = 1299076774U;
  c6_info[41].fileTimeHi = 0U;
  c6_info[41].mFileTimeLo = 0U;
  c6_info[41].mFileTimeHi = 0U;
  c6_info[42].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab_kinematics/ee_in_vehicle_frame.m";
  c6_info[42].name = "mrdivide";
  c6_info[42].dominantType = "double";
  c6_info[42].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c6_info[42].fileTimeLo = 1325124138U;
  c6_info[42].fileTimeHi = 0U;
  c6_info[42].mFileTimeLo = 1319729966U;
  c6_info[42].mFileTimeHi = 0U;
  c6_info[43].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab_kinematics/ee_in_vehicle_frame.m";
  c6_info[43].name = "Rzyx";
  c6_info[43].dominantType = "double";
  c6_info[43].resolved =
    "[E]/home/simena/Dropbox/master_project/code/matlab/gnc_mfiles/Rzyx.m";
  c6_info[43].fileTimeLo = 1206483734U;
  c6_info[43].fileTimeHi = 0U;
  c6_info[43].mFileTimeLo = 0U;
  c6_info[43].mFileTimeHi = 0U;
  c6_info[44].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab/gnc_mfiles/Rzyx.m";
  c6_info[44].name = "cos";
  c6_info[44].dominantType = "double";
  c6_info[44].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/cos.m";
  c6_info[44].fileTimeLo = 1286818706U;
  c6_info[44].fileTimeHi = 0U;
  c6_info[44].mFileTimeLo = 0U;
  c6_info[44].mFileTimeHi = 0U;
  c6_info[45].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab/gnc_mfiles/Rzyx.m";
  c6_info[45].name = "sin";
  c6_info[45].dominantType = "double";
  c6_info[45].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/sin.m";
  c6_info[45].fileTimeLo = 1286818750U;
  c6_info[45].fileTimeHi = 0U;
  c6_info[45].mFileTimeLo = 0U;
  c6_info[45].mFileTimeHi = 0U;
  c6_info[46].context =
    "[E]/home/simena/Dropbox/master_project/code/matlab/gnc_mfiles/Rzyx.m";
  c6_info[46].name = "mtimes";
  c6_info[46].dominantType = "double";
  c6_info[46].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[46].fileTimeLo = 1289519692U;
  c6_info[46].fileTimeHi = 0U;
  c6_info[46].mFileTimeLo = 0U;
  c6_info[46].mFileTimeHi = 0U;
  c6_info[47].context = "";
  c6_info[47].name = "rdivide";
  c6_info[47].dominantType = "double";
  c6_info[47].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/rdivide.m";
  c6_info[47].fileTimeLo = 1286818844U;
  c6_info[47].fileTimeHi = 0U;
  c6_info[47].mFileTimeLo = 0U;
  c6_info[47].mFileTimeHi = 0U;
  c6_info[48].context = "";
  c6_info[48].name = "exp";
  c6_info[48].dominantType = "double";
  c6_info[48].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/exp.m";
  c6_info[48].fileTimeLo = 1286818740U;
  c6_info[48].fileTimeHi = 0U;
  c6_info[48].mFileTimeLo = 0U;
  c6_info[48].mFileTimeHi = 0U;
  c6_info[49].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/exp.m";
  c6_info[49].name = "eml_scalar_exp";
  c6_info[49].dominantType = "double";
  c6_info[49].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_exp.m";
  c6_info[49].fileTimeLo = 1301328464U;
  c6_info[49].fileTimeHi = 0U;
  c6_info[49].mFileTimeLo = 0U;
  c6_info[49].mFileTimeHi = 0U;
  c6_info[50].context = "";
  c6_info[50].name = "inv";
  c6_info[50].dominantType = "double";
  c6_info[50].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m";
  c6_info[50].fileTimeLo = 1305318000U;
  c6_info[50].fileTimeHi = 0U;
  c6_info[50].mFileTimeLo = 0U;
  c6_info[50].mFileTimeHi = 0U;
  c6_info[51].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c6_info[51].name = "eml_index_class";
  c6_info[51].dominantType = "";
  c6_info[51].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[51].fileTimeLo = 1286818778U;
  c6_info[51].fileTimeHi = 0U;
  c6_info[51].mFileTimeLo = 0U;
  c6_info[51].mFileTimeHi = 0U;
  c6_info[52].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c6_info[52].name = "eml_xgetrf";
  c6_info[52].dominantType = "int32";
  c6_info[52].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c6_info[52].fileTimeLo = 1286818806U;
  c6_info[52].fileTimeHi = 0U;
  c6_info[52].mFileTimeLo = 0U;
  c6_info[52].mFileTimeHi = 0U;
  c6_info[53].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/eml_xgetrf.m";
  c6_info[53].name = "eml_lapack_xgetrf";
  c6_info[53].dominantType = "int32";
  c6_info[53].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c6_info[53].fileTimeLo = 1286818810U;
  c6_info[53].fileTimeHi = 0U;
  c6_info[53].mFileTimeLo = 0U;
  c6_info[53].mFileTimeHi = 0U;
  c6_info[54].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgetrf.m";
  c6_info[54].name = "eml_matlab_zgetrf";
  c6_info[54].dominantType = "int32";
  c6_info[54].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[54].fileTimeLo = 1302688994U;
  c6_info[54].fileTimeHi = 0U;
  c6_info[54].mFileTimeLo = 0U;
  c6_info[54].mFileTimeHi = 0U;
  c6_info[55].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[55].name = "realmin";
  c6_info[55].dominantType = "char";
  c6_info[55].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/realmin.m";
  c6_info[55].fileTimeLo = 1307651242U;
  c6_info[55].fileTimeHi = 0U;
  c6_info[55].mFileTimeLo = 0U;
  c6_info[55].mFileTimeHi = 0U;
  c6_info[56].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/realmin.m";
  c6_info[56].name = "eml_realmin";
  c6_info[56].dominantType = "char";
  c6_info[56].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c6_info[56].fileTimeLo = 1307651244U;
  c6_info[56].fileTimeHi = 0U;
  c6_info[56].mFileTimeLo = 0U;
  c6_info[56].mFileTimeHi = 0U;
  c6_info[57].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c6_info[57].name = "eml_float_model";
  c6_info[57].dominantType = "char";
  c6_info[57].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c6_info[57].fileTimeLo = 1307651242U;
  c6_info[57].fileTimeHi = 0U;
  c6_info[57].mFileTimeLo = 0U;
  c6_info[57].mFileTimeHi = 0U;
  c6_info[58].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[58].name = "eps";
  c6_info[58].dominantType = "char";
  c6_info[58].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eps.m";
  c6_info[58].fileTimeLo = 1307651240U;
  c6_info[58].fileTimeHi = 0U;
  c6_info[58].mFileTimeLo = 0U;
  c6_info[58].mFileTimeHi = 0U;
  c6_info[59].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eps.m";
  c6_info[59].name = "eml_is_float_class";
  c6_info[59].dominantType = "char";
  c6_info[59].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c6_info[59].fileTimeLo = 1286818782U;
  c6_info[59].fileTimeHi = 0U;
  c6_info[59].mFileTimeLo = 0U;
  c6_info[59].mFileTimeHi = 0U;
  c6_info[60].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eps.m";
  c6_info[60].name = "eml_eps";
  c6_info[60].dominantType = "char";
  c6_info[60].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c6_info[60].fileTimeLo = 1307651240U;
  c6_info[60].fileTimeHi = 0U;
  c6_info[60].mFileTimeLo = 0U;
  c6_info[60].mFileTimeHi = 0U;
  c6_info[61].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c6_info[61].name = "eml_float_model";
  c6_info[61].dominantType = "char";
  c6_info[61].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c6_info[61].fileTimeLo = 1307651242U;
  c6_info[61].fileTimeHi = 0U;
  c6_info[61].mFileTimeLo = 0U;
  c6_info[61].mFileTimeHi = 0U;
  c6_info[62].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[62].name = "min";
  c6_info[62].dominantType = "int32";
  c6_info[62].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/min.m";
  c6_info[62].fileTimeLo = 1311255318U;
  c6_info[62].fileTimeHi = 0U;
  c6_info[62].mFileTimeLo = 0U;
  c6_info[62].mFileTimeHi = 0U;
  c6_info[63].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/min.m";
  c6_info[63].name = "eml_min_or_max";
  c6_info[63].dominantType = "int32";
  c6_info[63].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c6_info[63].fileTimeLo = 1303146212U;
  c6_info[63].fileTimeHi = 0U;
  c6_info[63].mFileTimeLo = 0U;
  c6_info[63].mFileTimeHi = 0U;
}

static void c6_b_info_helper(c6_ResolvedFunctionInfo c6_info[156])
{
  c6_info[64].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c6_info[64].name = "eml_scalar_eg";
  c6_info[64].dominantType = "int32";
  c6_info[64].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[64].fileTimeLo = 1286818796U;
  c6_info[64].fileTimeHi = 0U;
  c6_info[64].mFileTimeLo = 0U;
  c6_info[64].mFileTimeHi = 0U;
  c6_info[65].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c6_info[65].name = "eml_scalexp_alloc";
  c6_info[65].dominantType = "int32";
  c6_info[65].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c6_info[65].fileTimeLo = 1286818796U;
  c6_info[65].fileTimeHi = 0U;
  c6_info[65].mFileTimeLo = 0U;
  c6_info[65].mFileTimeHi = 0U;
  c6_info[66].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c6_info[66].name = "eml_scalar_eg";
  c6_info[66].dominantType = "int32";
  c6_info[66].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[66].fileTimeLo = 1286818796U;
  c6_info[66].fileTimeHi = 0U;
  c6_info[66].mFileTimeLo = 0U;
  c6_info[66].mFileTimeHi = 0U;
  c6_info[67].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[67].name = "colon";
  c6_info[67].dominantType = "int32";
  c6_info[67].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/colon.m";
  c6_info[67].fileTimeLo = 1311255318U;
  c6_info[67].fileTimeHi = 0U;
  c6_info[67].mFileTimeLo = 0U;
  c6_info[67].mFileTimeHi = 0U;
  c6_info[68].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/colon.m";
  c6_info[68].name = "colon";
  c6_info[68].dominantType = "int32";
  c6_info[68].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/colon.m";
  c6_info[68].fileTimeLo = 1311255318U;
  c6_info[68].fileTimeHi = 0U;
  c6_info[68].mFileTimeLo = 0U;
  c6_info[68].mFileTimeHi = 0U;
  c6_info[69].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/colon.m";
  c6_info[69].name = "floor";
  c6_info[69].dominantType = "double";
  c6_info[69].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/floor.m";
  c6_info[69].fileTimeLo = 1286818742U;
  c6_info[69].fileTimeHi = 0U;
  c6_info[69].mFileTimeLo = 0U;
  c6_info[69].mFileTimeHi = 0U;
  c6_info[70].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/floor.m";
  c6_info[70].name = "eml_scalar_floor";
  c6_info[70].dominantType = "double";
  c6_info[70].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m";
  c6_info[70].fileTimeLo = 1286818726U;
  c6_info[70].fileTimeHi = 0U;
  c6_info[70].mFileTimeLo = 0U;
  c6_info[70].mFileTimeHi = 0U;
  c6_info[71].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c6_info[71].name = "intmin";
  c6_info[71].dominantType = "char";
  c6_info[71].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/intmin.m";
  c6_info[71].fileTimeLo = 1311255318U;
  c6_info[71].fileTimeHi = 0U;
  c6_info[71].mFileTimeLo = 0U;
  c6_info[71].mFileTimeHi = 0U;
  c6_info[72].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/colon.m!checkrange";
  c6_info[72].name = "intmax";
  c6_info[72].dominantType = "char";
  c6_info[72].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/intmax.m";
  c6_info[72].fileTimeLo = 1311255316U;
  c6_info[72].fileTimeHi = 0U;
  c6_info[72].mFileTimeLo = 0U;
  c6_info[72].mFileTimeHi = 0U;
  c6_info[73].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c6_info[73].name = "intmin";
  c6_info[73].dominantType = "char";
  c6_info[73].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/intmin.m";
  c6_info[73].fileTimeLo = 1311255318U;
  c6_info[73].fileTimeHi = 0U;
  c6_info[73].mFileTimeLo = 0U;
  c6_info[73].mFileTimeHi = 0U;
  c6_info[74].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c6_info[74].name = "intmax";
  c6_info[74].dominantType = "char";
  c6_info[74].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/intmax.m";
  c6_info[74].fileTimeLo = 1311255316U;
  c6_info[74].fileTimeHi = 0U;
  c6_info[74].mFileTimeLo = 0U;
  c6_info[74].mFileTimeHi = 0U;
  c6_info[75].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher";
  c6_info[75].name = "eml_isa_uint";
  c6_info[75].dominantType = "int32";
  c6_info[75].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c6_info[75].fileTimeLo = 1286818784U;
  c6_info[75].fileTimeHi = 0U;
  c6_info[75].mFileTimeLo = 0U;
  c6_info[75].mFileTimeHi = 0U;
  c6_info[76].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c6_info[76].name = "eml_unsigned_class";
  c6_info[76].dominantType = "char";
  c6_info[76].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m";
  c6_info[76].fileTimeLo = 1286818800U;
  c6_info[76].fileTimeHi = 0U;
  c6_info[76].mFileTimeLo = 0U;
  c6_info[76].mFileTimeHi = 0U;
  c6_info[77].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c6_info[77].name = "eml_index_class";
  c6_info[77].dominantType = "";
  c6_info[77].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[77].fileTimeLo = 1286818778U;
  c6_info[77].fileTimeHi = 0U;
  c6_info[77].mFileTimeLo = 0U;
  c6_info[77].mFileTimeHi = 0U;
  c6_info[78].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c6_info[78].name = "intmax";
  c6_info[78].dominantType = "char";
  c6_info[78].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/intmax.m";
  c6_info[78].fileTimeLo = 1311255316U;
  c6_info[78].fileTimeHi = 0U;
  c6_info[78].mFileTimeLo = 0U;
  c6_info[78].mFileTimeHi = 0U;
  c6_info[79].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c6_info[79].name = "eml_isa_uint";
  c6_info[79].dominantType = "int32";
  c6_info[79].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c6_info[79].fileTimeLo = 1286818784U;
  c6_info[79].fileTimeHi = 0U;
  c6_info[79].mFileTimeLo = 0U;
  c6_info[79].mFileTimeHi = 0U;
  c6_info[80].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd";
  c6_info[80].name = "eml_index_plus";
  c6_info[80].dominantType = "int32";
  c6_info[80].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[80].fileTimeLo = 1286818778U;
  c6_info[80].fileTimeHi = 0U;
  c6_info[80].mFileTimeLo = 0U;
  c6_info[80].mFileTimeHi = 0U;
  c6_info[81].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[81].name = "eml_index_class";
  c6_info[81].dominantType = "";
  c6_info[81].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[81].fileTimeLo = 1286818778U;
  c6_info[81].fileTimeHi = 0U;
  c6_info[81].mFileTimeLo = 0U;
  c6_info[81].mFileTimeHi = 0U;
  c6_info[82].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/colon.m!eml_signed_integer_colon";
  c6_info[82].name = "eml_int_forloop_overflow_check";
  c6_info[82].dominantType = "";
  c6_info[82].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c6_info[82].fileTimeLo = 1311255316U;
  c6_info[82].fileTimeHi = 0U;
  c6_info[82].mFileTimeLo = 0U;
  c6_info[82].mFileTimeHi = 0U;
  c6_info[83].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[83].name = "eml_index_class";
  c6_info[83].dominantType = "";
  c6_info[83].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[83].fileTimeLo = 1286818778U;
  c6_info[83].fileTimeHi = 0U;
  c6_info[83].mFileTimeLo = 0U;
  c6_info[83].mFileTimeHi = 0U;
  c6_info[84].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[84].name = "eml_index_plus";
  c6_info[84].dominantType = "int32";
  c6_info[84].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[84].fileTimeLo = 1286818778U;
  c6_info[84].fileTimeHi = 0U;
  c6_info[84].mFileTimeLo = 0U;
  c6_info[84].mFileTimeHi = 0U;
  c6_info[85].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[85].name = "eml_int_forloop_overflow_check";
  c6_info[85].dominantType = "";
  c6_info[85].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c6_info[85].fileTimeLo = 1311255316U;
  c6_info[85].fileTimeHi = 0U;
  c6_info[85].mFileTimeLo = 0U;
  c6_info[85].mFileTimeHi = 0U;
  c6_info[86].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[86].name = "eml_index_minus";
  c6_info[86].dominantType = "int32";
  c6_info[86].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c6_info[86].fileTimeLo = 1286818778U;
  c6_info[86].fileTimeHi = 0U;
  c6_info[86].mFileTimeLo = 0U;
  c6_info[86].mFileTimeHi = 0U;
  c6_info[87].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c6_info[87].name = "eml_index_class";
  c6_info[87].dominantType = "";
  c6_info[87].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[87].fileTimeLo = 1286818778U;
  c6_info[87].fileTimeHi = 0U;
  c6_info[87].mFileTimeLo = 0U;
  c6_info[87].mFileTimeHi = 0U;
  c6_info[88].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[88].name = "eml_index_times";
  c6_info[88].dominantType = "int32";
  c6_info[88].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c6_info[88].fileTimeLo = 1286818780U;
  c6_info[88].fileTimeHi = 0U;
  c6_info[88].mFileTimeLo = 0U;
  c6_info[88].mFileTimeHi = 0U;
  c6_info[89].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c6_info[89].name = "eml_index_class";
  c6_info[89].dominantType = "";
  c6_info[89].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[89].fileTimeLo = 1286818778U;
  c6_info[89].fileTimeHi = 0U;
  c6_info[89].mFileTimeLo = 0U;
  c6_info[89].mFileTimeHi = 0U;
  c6_info[90].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[90].name = "eml_ixamax";
  c6_info[90].dominantType = "int32";
  c6_info[90].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c6_info[90].fileTimeLo = 1299076770U;
  c6_info[90].fileTimeHi = 0U;
  c6_info[90].mFileTimeLo = 0U;
  c6_info[90].mFileTimeHi = 0U;
  c6_info[91].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m";
  c6_info[91].name = "eml_blas_inline";
  c6_info[91].dominantType = "";
  c6_info[91].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c6_info[91].fileTimeLo = 1299076768U;
  c6_info[91].fileTimeHi = 0U;
  c6_info[91].mFileTimeLo = 0U;
  c6_info[91].mFileTimeHi = 0U;
  c6_info[92].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m!below_threshold";
  c6_info[92].name = "length";
  c6_info[92].dominantType = "double";
  c6_info[92].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/length.m";
  c6_info[92].fileTimeLo = 1303146206U;
  c6_info[92].fileTimeHi = 0U;
  c6_info[92].mFileTimeLo = 0U;
  c6_info[92].mFileTimeHi = 0U;
  c6_info[93].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/length.m!intlength";
  c6_info[93].name = "eml_index_class";
  c6_info[93].dominantType = "";
  c6_info[93].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[93].fileTimeLo = 1286818778U;
  c6_info[93].fileTimeHi = 0U;
  c6_info[93].mFileTimeLo = 0U;
  c6_info[93].mFileTimeHi = 0U;
  c6_info[94].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_ixamax.m";
  c6_info[94].name = "eml_refblas_ixamax";
  c6_info[94].dominantType = "int32";
  c6_info[94].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c6_info[94].fileTimeLo = 1299076770U;
  c6_info[94].fileTimeHi = 0U;
  c6_info[94].mFileTimeLo = 0U;
  c6_info[94].mFileTimeHi = 0U;
  c6_info[95].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c6_info[95].name = "eml_index_class";
  c6_info[95].dominantType = "";
  c6_info[95].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[95].fileTimeLo = 1286818778U;
  c6_info[95].fileTimeHi = 0U;
  c6_info[95].mFileTimeLo = 0U;
  c6_info[95].mFileTimeHi = 0U;
  c6_info[96].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c6_info[96].name = "eml_xcabs1";
  c6_info[96].dominantType = "double";
  c6_info[96].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c6_info[96].fileTimeLo = 1286818706U;
  c6_info[96].fileTimeHi = 0U;
  c6_info[96].mFileTimeLo = 0U;
  c6_info[96].mFileTimeHi = 0U;
  c6_info[97].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m";
  c6_info[97].name = "abs";
  c6_info[97].dominantType = "double";
  c6_info[97].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c6_info[97].fileTimeLo = 1286818694U;
  c6_info[97].fileTimeHi = 0U;
  c6_info[97].mFileTimeLo = 0U;
  c6_info[97].mFileTimeHi = 0U;
  c6_info[98].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c6_info[98].name = "eml_int_forloop_overflow_check";
  c6_info[98].dominantType = "";
  c6_info[98].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c6_info[98].fileTimeLo = 1311255316U;
  c6_info[98].fileTimeHi = 0U;
  c6_info[98].mFileTimeLo = 0U;
  c6_info[98].mFileTimeHi = 0U;
  c6_info[99].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_ixamax.m";
  c6_info[99].name = "eml_index_plus";
  c6_info[99].dominantType = "int32";
  c6_info[99].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[99].fileTimeLo = 1286818778U;
  c6_info[99].fileTimeHi = 0U;
  c6_info[99].mFileTimeLo = 0U;
  c6_info[99].mFileTimeHi = 0U;
  c6_info[100].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[100].name = "eml_xswap";
  c6_info[100].dominantType = "int32";
  c6_info[100].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c6_info[100].fileTimeLo = 1299076778U;
  c6_info[100].fileTimeHi = 0U;
  c6_info[100].mFileTimeLo = 0U;
  c6_info[100].mFileTimeHi = 0U;
  c6_info[101].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c6_info[101].name = "eml_blas_inline";
  c6_info[101].dominantType = "";
  c6_info[101].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c6_info[101].fileTimeLo = 1299076768U;
  c6_info[101].fileTimeHi = 0U;
  c6_info[101].mFileTimeLo = 0U;
  c6_info[101].mFileTimeHi = 0U;
  c6_info[102].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c6_info[102].name = "eml_refblas_xswap";
  c6_info[102].dominantType = "int32";
  c6_info[102].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c6_info[102].fileTimeLo = 1299076786U;
  c6_info[102].fileTimeHi = 0U;
  c6_info[102].mFileTimeLo = 0U;
  c6_info[102].mFileTimeHi = 0U;
  c6_info[103].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c6_info[103].name = "eml_index_class";
  c6_info[103].dominantType = "";
  c6_info[103].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[103].fileTimeLo = 1286818778U;
  c6_info[103].fileTimeHi = 0U;
  c6_info[103].mFileTimeLo = 0U;
  c6_info[103].mFileTimeHi = 0U;
  c6_info[104].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c6_info[104].name = "abs";
  c6_info[104].dominantType = "int32";
  c6_info[104].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c6_info[104].fileTimeLo = 1286818694U;
  c6_info[104].fileTimeHi = 0U;
  c6_info[104].mFileTimeLo = 0U;
  c6_info[104].mFileTimeHi = 0U;
  c6_info[105].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c6_info[105].name = "eml_scalar_abs";
  c6_info[105].dominantType = "int32";
  c6_info[105].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c6_info[105].fileTimeLo = 1286818712U;
  c6_info[105].fileTimeHi = 0U;
  c6_info[105].mFileTimeLo = 0U;
  c6_info[105].mFileTimeHi = 0U;
  c6_info[106].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c6_info[106].name = "eml_int_forloop_overflow_check";
  c6_info[106].dominantType = "";
  c6_info[106].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c6_info[106].fileTimeLo = 1311255316U;
  c6_info[106].fileTimeHi = 0U;
  c6_info[106].mFileTimeLo = 0U;
  c6_info[106].mFileTimeHi = 0U;
  c6_info[107].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c6_info[107].name = "eml_index_plus";
  c6_info[107].dominantType = "int32";
  c6_info[107].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[107].fileTimeLo = 1286818778U;
  c6_info[107].fileTimeHi = 0U;
  c6_info[107].mFileTimeLo = 0U;
  c6_info[107].mFileTimeHi = 0U;
  c6_info[108].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[108].name = "eml_div";
  c6_info[108].dominantType = "double";
  c6_info[108].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_div.m";
  c6_info[108].fileTimeLo = 1313347810U;
  c6_info[108].fileTimeHi = 0U;
  c6_info[108].mFileTimeLo = 0U;
  c6_info[108].mFileTimeHi = 0U;
  c6_info[109].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgetrf.m";
  c6_info[109].name = "eml_xgeru";
  c6_info[109].dominantType = "int32";
  c6_info[109].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c6_info[109].fileTimeLo = 1299076774U;
  c6_info[109].fileTimeHi = 0U;
  c6_info[109].mFileTimeLo = 0U;
  c6_info[109].mFileTimeHi = 0U;
  c6_info[110].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c6_info[110].name = "eml_blas_inline";
  c6_info[110].dominantType = "";
  c6_info[110].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c6_info[110].fileTimeLo = 1299076768U;
  c6_info[110].fileTimeHi = 0U;
  c6_info[110].mFileTimeLo = 0U;
  c6_info[110].mFileTimeHi = 0U;
  c6_info[111].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xgeru.m";
  c6_info[111].name = "eml_xger";
  c6_info[111].dominantType = "int32";
  c6_info[111].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c6_info[111].fileTimeLo = 1299076774U;
  c6_info[111].fileTimeHi = 0U;
  c6_info[111].mFileTimeLo = 0U;
  c6_info[111].mFileTimeHi = 0U;
  c6_info[112].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xger.m";
  c6_info[112].name = "eml_blas_inline";
  c6_info[112].dominantType = "";
  c6_info[112].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c6_info[112].fileTimeLo = 1299076768U;
  c6_info[112].fileTimeHi = 0U;
  c6_info[112].mFileTimeLo = 0U;
  c6_info[112].mFileTimeHi = 0U;
  c6_info[113].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c6_info[113].name = "intmax";
  c6_info[113].dominantType = "char";
  c6_info[113].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/intmax.m";
  c6_info[113].fileTimeLo = 1311255316U;
  c6_info[113].fileTimeHi = 0U;
  c6_info[113].mFileTimeLo = 0U;
  c6_info[113].mFileTimeHi = 0U;
  c6_info[114].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c6_info[114].name = "min";
  c6_info[114].dominantType = "double";
  c6_info[114].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/min.m";
  c6_info[114].fileTimeLo = 1311255318U;
  c6_info[114].fileTimeHi = 0U;
  c6_info[114].mFileTimeLo = 0U;
  c6_info[114].mFileTimeHi = 0U;
  c6_info[115].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m!below_threshold";
  c6_info[115].name = "mtimes";
  c6_info[115].dominantType = "double";
  c6_info[115].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[115].fileTimeLo = 1289519692U;
  c6_info[115].fileTimeHi = 0U;
  c6_info[115].mFileTimeLo = 0U;
  c6_info[115].mFileTimeHi = 0U;
  c6_info[116].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xger.m";
  c6_info[116].name = "eml_refblas_xger";
  c6_info[116].dominantType = "int32";
  c6_info[116].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c6_info[116].fileTimeLo = 1299076776U;
  c6_info[116].fileTimeHi = 0U;
  c6_info[116].mFileTimeLo = 0U;
  c6_info[116].mFileTimeHi = 0U;
  c6_info[117].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xger.m";
  c6_info[117].name = "eml_refblas_xgerx";
  c6_info[117].dominantType = "int32";
  c6_info[117].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c6_info[117].fileTimeLo = 1299076778U;
  c6_info[117].fileTimeHi = 0U;
  c6_info[117].mFileTimeLo = 0U;
  c6_info[117].mFileTimeHi = 0U;
  c6_info[118].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c6_info[118].name = "eml_index_class";
  c6_info[118].dominantType = "";
  c6_info[118].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[118].fileTimeLo = 1286818778U;
  c6_info[118].fileTimeHi = 0U;
  c6_info[118].mFileTimeLo = 0U;
  c6_info[118].mFileTimeHi = 0U;
  c6_info[119].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c6_info[119].name = "abs";
  c6_info[119].dominantType = "int32";
  c6_info[119].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c6_info[119].fileTimeLo = 1286818694U;
  c6_info[119].fileTimeHi = 0U;
  c6_info[119].mFileTimeLo = 0U;
  c6_info[119].mFileTimeHi = 0U;
  c6_info[120].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c6_info[120].name = "eml_index_minus";
  c6_info[120].dominantType = "int32";
  c6_info[120].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c6_info[120].fileTimeLo = 1286818778U;
  c6_info[120].fileTimeHi = 0U;
  c6_info[120].mFileTimeLo = 0U;
  c6_info[120].mFileTimeHi = 0U;
  c6_info[121].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c6_info[121].name = "eml_int_forloop_overflow_check";
  c6_info[121].dominantType = "";
  c6_info[121].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c6_info[121].fileTimeLo = 1311255316U;
  c6_info[121].fileTimeHi = 0U;
  c6_info[121].mFileTimeLo = 0U;
  c6_info[121].mFileTimeHi = 0U;
  c6_info[122].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgerx.m";
  c6_info[122].name = "eml_index_plus";
  c6_info[122].dominantType = "int32";
  c6_info[122].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[122].fileTimeLo = 1286818778U;
  c6_info[122].fileTimeHi = 0U;
  c6_info[122].mFileTimeLo = 0U;
  c6_info[122].mFileTimeHi = 0U;
  c6_info[123].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c6_info[123].name = "eml_ipiv2perm";
  c6_info[123].dominantType = "int32";
  c6_info[123].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_ipiv2perm.m";
  c6_info[123].fileTimeLo = 1286818782U;
  c6_info[123].fileTimeHi = 0U;
  c6_info[123].mFileTimeLo = 0U;
  c6_info[123].mFileTimeHi = 0U;
  c6_info[124].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_ipiv2perm.m";
  c6_info[124].name = "colon";
  c6_info[124].dominantType = "int32";
  c6_info[124].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/colon.m";
  c6_info[124].fileTimeLo = 1311255318U;
  c6_info[124].fileTimeHi = 0U;
  c6_info[124].mFileTimeLo = 0U;
  c6_info[124].mFileTimeHi = 0U;
  c6_info[125].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_ipiv2perm.m";
  c6_info[125].name = "eml_index_class";
  c6_info[125].dominantType = "";
  c6_info[125].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[125].fileTimeLo = 1286818778U;
  c6_info[125].fileTimeHi = 0U;
  c6_info[125].mFileTimeLo = 0U;
  c6_info[125].mFileTimeHi = 0U;
  c6_info[126].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c6_info[126].name = "eml_int_forloop_overflow_check";
  c6_info[126].dominantType = "";
  c6_info[126].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c6_info[126].fileTimeLo = 1311255316U;
  c6_info[126].fileTimeHi = 0U;
  c6_info[126].mFileTimeLo = 0U;
  c6_info[126].mFileTimeHi = 0U;
  c6_info[127].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c6_info[127].name = "eml_index_plus";
  c6_info[127].dominantType = "int32";
  c6_info[127].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[127].fileTimeLo = 1286818778U;
  c6_info[127].fileTimeHi = 0U;
  c6_info[127].mFileTimeLo = 0U;
  c6_info[127].mFileTimeHi = 0U;
}

static void c6_c_info_helper(c6_ResolvedFunctionInfo c6_info[156])
{
  c6_info[128].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c6_info[128].name = "mtimes";
  c6_info[128].dominantType = "double";
  c6_info[128].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[128].fileTimeLo = 1289519692U;
  c6_info[128].fileTimeHi = 0U;
  c6_info[128].mFileTimeLo = 0U;
  c6_info[128].mFileTimeHi = 0U;
  c6_info[129].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c6_info[129].name = "eml_scalar_eg";
  c6_info[129].dominantType = "double";
  c6_info[129].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[129].fileTimeLo = 1286818796U;
  c6_info[129].fileTimeHi = 0U;
  c6_info[129].mFileTimeLo = 0U;
  c6_info[129].mFileTimeHi = 0U;
  c6_info[130].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!invNxN";
  c6_info[130].name = "eml_xtrsm";
  c6_info[130].dominantType = "int32";
  c6_info[130].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c6_info[130].fileTimeLo = 1299076778U;
  c6_info[130].fileTimeHi = 0U;
  c6_info[130].mFileTimeLo = 0U;
  c6_info[130].mFileTimeHi = 0U;
  c6_info[131].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xtrsm.m";
  c6_info[131].name = "eml_blas_inline";
  c6_info[131].dominantType = "";
  c6_info[131].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c6_info[131].fileTimeLo = 1299076768U;
  c6_info[131].fileTimeHi = 0U;
  c6_info[131].mFileTimeLo = 0U;
  c6_info[131].mFileTimeHi = 0U;
  c6_info[132].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m!below_threshold";
  c6_info[132].name = "mtimes";
  c6_info[132].dominantType = "double";
  c6_info[132].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[132].fileTimeLo = 1289519692U;
  c6_info[132].fileTimeHi = 0U;
  c6_info[132].mFileTimeLo = 0U;
  c6_info[132].mFileTimeHi = 0U;
  c6_info[133].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c6_info[133].name = "eml_scalar_eg";
  c6_info[133].dominantType = "double";
  c6_info[133].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[133].fileTimeLo = 1286818796U;
  c6_info[133].fileTimeHi = 0U;
  c6_info[133].mFileTimeLo = 0U;
  c6_info[133].mFileTimeHi = 0U;
  c6_info[134].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c6_info[134].name = "eml_refblas_xtrsm";
  c6_info[134].dominantType = "int32";
  c6_info[134].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c6_info[134].fileTimeLo = 1299076786U;
  c6_info[134].fileTimeHi = 0U;
  c6_info[134].mFileTimeLo = 0U;
  c6_info[134].mFileTimeHi = 0U;
  c6_info[135].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c6_info[135].name = "eml_scalar_eg";
  c6_info[135].dominantType = "double";
  c6_info[135].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c6_info[135].fileTimeLo = 1286818796U;
  c6_info[135].fileTimeHi = 0U;
  c6_info[135].mFileTimeLo = 0U;
  c6_info[135].mFileTimeHi = 0U;
  c6_info[136].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c6_info[136].name = "eml_index_minus";
  c6_info[136].dominantType = "int32";
  c6_info[136].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c6_info[136].fileTimeLo = 1286818778U;
  c6_info[136].fileTimeHi = 0U;
  c6_info[136].mFileTimeLo = 0U;
  c6_info[136].mFileTimeHi = 0U;
  c6_info[137].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c6_info[137].name = "eml_index_class";
  c6_info[137].dominantType = "";
  c6_info[137].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[137].fileTimeLo = 1286818778U;
  c6_info[137].fileTimeHi = 0U;
  c6_info[137].mFileTimeLo = 0U;
  c6_info[137].mFileTimeHi = 0U;
  c6_info[138].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c6_info[138].name = "eml_int_forloop_overflow_check";
  c6_info[138].dominantType = "";
  c6_info[138].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c6_info[138].fileTimeLo = 1311255316U;
  c6_info[138].fileTimeHi = 0U;
  c6_info[138].mFileTimeLo = 0U;
  c6_info[138].mFileTimeHi = 0U;
  c6_info[139].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c6_info[139].name = "eml_index_times";
  c6_info[139].dominantType = "int32";
  c6_info[139].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c6_info[139].fileTimeLo = 1286818780U;
  c6_info[139].fileTimeHi = 0U;
  c6_info[139].mFileTimeLo = 0U;
  c6_info[139].mFileTimeHi = 0U;
  c6_info[140].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c6_info[140].name = "eml_index_plus";
  c6_info[140].dominantType = "int32";
  c6_info[140].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c6_info[140].fileTimeLo = 1286818778U;
  c6_info[140].fileTimeHi = 0U;
  c6_info[140].mFileTimeLo = 0U;
  c6_info[140].mFileTimeHi = 0U;
  c6_info[141].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c6_info[141].name = "intmin";
  c6_info[141].dominantType = "char";
  c6_info[141].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/intmin.m";
  c6_info[141].fileTimeLo = 1311255318U;
  c6_info[141].fileTimeHi = 0U;
  c6_info[141].mFileTimeLo = 0U;
  c6_info[141].mFileTimeHi = 0U;
  c6_info[142].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xtrsm.m";
  c6_info[142].name = "eml_div";
  c6_info[142].dominantType = "double";
  c6_info[142].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_div.m";
  c6_info[142].fileTimeLo = 1313347810U;
  c6_info[142].fileTimeHi = 0U;
  c6_info[142].mFileTimeLo = 0U;
  c6_info[142].mFileTimeHi = 0U;
  c6_info[143].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xtrsm.m";
  c6_info[143].name = "eml_index_class";
  c6_info[143].dominantType = "";
  c6_info[143].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[143].fileTimeLo = 1286818778U;
  c6_info[143].fileTimeHi = 0U;
  c6_info[143].mFileTimeLo = 0U;
  c6_info[143].mFileTimeHi = 0U;
  c6_info[144].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c6_info[144].name = "norm";
  c6_info[144].dominantType = "double";
  c6_info[144].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/norm.m";
  c6_info[144].fileTimeLo = 1286818826U;
  c6_info[144].fileTimeHi = 0U;
  c6_info[144].mFileTimeLo = 0U;
  c6_info[144].mFileTimeHi = 0U;
  c6_info[145].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm";
  c6_info[145].name = "abs";
  c6_info[145].dominantType = "double";
  c6_info[145].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c6_info[145].fileTimeLo = 1286818694U;
  c6_info[145].fileTimeHi = 0U;
  c6_info[145].mFileTimeLo = 0U;
  c6_info[145].mFileTimeHi = 0U;
  c6_info[146].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm";
  c6_info[146].name = "isnan";
  c6_info[146].dominantType = "double";
  c6_info[146].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isnan.m";
  c6_info[146].fileTimeLo = 1286818760U;
  c6_info[146].fileTimeHi = 0U;
  c6_info[146].mFileTimeLo = 0U;
  c6_info[146].mFileTimeHi = 0U;
  c6_info[147].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/norm.m!mat1norm";
  c6_info[147].name = "eml_guarded_nan";
  c6_info[147].dominantType = "char";
  c6_info[147].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c6_info[147].fileTimeLo = 1286818776U;
  c6_info[147].fileTimeHi = 0U;
  c6_info[147].mFileTimeLo = 0U;
  c6_info[147].mFileTimeHi = 0U;
  c6_info[148].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m";
  c6_info[148].name = "eml_is_float_class";
  c6_info[148].dominantType = "char";
  c6_info[148].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c6_info[148].fileTimeLo = 1286818782U;
  c6_info[148].fileTimeHi = 0U;
  c6_info[148].mFileTimeLo = 0U;
  c6_info[148].mFileTimeHi = 0U;
  c6_info[149].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c6_info[149].name = "mtimes";
  c6_info[149].dominantType = "double";
  c6_info[149].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c6_info[149].fileTimeLo = 1289519692U;
  c6_info[149].fileTimeHi = 0U;
  c6_info[149].mFileTimeLo = 0U;
  c6_info[149].mFileTimeHi = 0U;
  c6_info[150].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c6_info[150].name = "eml_warning";
  c6_info[150].dominantType = "char";
  c6_info[150].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_warning.m";
  c6_info[150].fileTimeLo = 1286818802U;
  c6_info[150].fileTimeHi = 0U;
  c6_info[150].mFileTimeLo = 0U;
  c6_info[150].mFileTimeHi = 0U;
  c6_info[151].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c6_info[151].name = "isnan";
  c6_info[151].dominantType = "double";
  c6_info[151].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isnan.m";
  c6_info[151].fileTimeLo = 1286818760U;
  c6_info[151].fileTimeHi = 0U;
  c6_info[151].mFileTimeLo = 0U;
  c6_info[151].mFileTimeHi = 0U;
  c6_info[152].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c6_info[152].name = "eps";
  c6_info[152].dominantType = "char";
  c6_info[152].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eps.m";
  c6_info[152].fileTimeLo = 1307651240U;
  c6_info[152].fileTimeHi = 0U;
  c6_info[152].mFileTimeLo = 0U;
  c6_info[152].mFileTimeHi = 0U;
  c6_info[153].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/inv.m!checkcond";
  c6_info[153].name = "eml_flt2str";
  c6_info[153].dominantType = "double";
  c6_info[153].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_flt2str.m";
  c6_info[153].fileTimeLo = 1309451196U;
  c6_info[153].fileTimeHi = 0U;
  c6_info[153].mFileTimeLo = 0U;
  c6_info[153].mFileTimeHi = 0U;
  c6_info[154].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_flt2str.m";
  c6_info[154].name = "char";
  c6_info[154].dominantType = "double";
  c6_info[154].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/strfun/char.m";
  c6_info[154].fileTimeLo = 1319729968U;
  c6_info[154].fileTimeHi = 0U;
  c6_info[154].mFileTimeLo = 0U;
  c6_info[154].mFileTimeHi = 0U;
  c6_info[155].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c6_info[155].name = "eml_index_class";
  c6_info[155].dominantType = "";
  c6_info[155].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c6_info[155].fileTimeLo = 1286818778U;
  c6_info[155].fileTimeHi = 0U;
  c6_info[155].mFileTimeLo = 0U;
  c6_info[155].mFileTimeHi = 0U;
}

static void c6_isVariableSizing(SFc6_kinematicsInstanceStruct *chartInstance)
{
}

static void c6_eye(SFc6_kinematicsInstanceStruct *chartInstance, real_T c6_I[144])
{
  int32_T c6_i180;
  int32_T c6_i;
  int32_T c6_b_i;
  c6_isVariableSizing(chartInstance);
  for (c6_i180 = 0; c6_i180 < 144; c6_i180++) {
    c6_I[c6_i180] = 0.0;
  }

  c6_eml_int_forloop_overflow_check(chartInstance);
  for (c6_i = 1; c6_i < 13; c6_i++) {
    c6_b_i = c6_i;
    c6_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_i), 1, 12, 1, 0) + 12 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_i), 1, 12, 2, 0) - 1))
      - 1] = 1.0;
  }
}

static void c6_eml_scalar_eg(SFc6_kinematicsInstanceStruct *chartInstance)
{
}

static void c6_eml_int_forloop_overflow_check(SFc6_kinematicsInstanceStruct
  *chartInstance)
{
}

static real_T c6_rdivide(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_x, real_T c6_y)
{
  real_T c6_b_x;
  real_T c6_b_y;
  c6_b_x = c6_x;
  c6_b_y = c6_y;
  return c6_b_x / c6_b_y;
}

static real_T c6_power(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_b_a)
{
  real_T c6_ak;
  c6_eml_scalar_eg(chartInstance);
  c6_ak = c6_b_a;
  return muDoubleScalarPower(c6_ak, 2.0);
}

static real_T c6_abs(SFc6_kinematicsInstanceStruct *chartInstance, real_T c6_x)
{
  real_T c6_b_x;
  c6_b_x = c6_x;
  return muDoubleScalarAbs(c6_b_x);
}

static real_T c6_b_power(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_b_a)
{
  real_T c6_ak;
  c6_eml_scalar_eg(chartInstance);
  c6_ak = c6_b_a;
  return muDoubleScalarPower(c6_ak, 4.0);
}

static void c6_ee_in_vehicle_frame(SFc6_kinematicsInstanceStruct *chartInstance,
  real_T c6_q[6], real_T c6_b_d[6], real_T c6_b_a[6], real_T c6_b_alpha[6],
  real_T c6_vec[3])
{
  uint32_T c6_debug_family_var_map[10];
  real_T c6_g[16];
  real_T c6_Rtemp[9];
  real_T c6_tool_hom_mat[16];
  real_T c6_nargin = 4.0;
  real_T c6_nargout = 1.0;
  real_T c6_dv24[16];
  int32_T c6_i181;
  int32_T c6_i;
  real_T c6_b_i;
  int32_T c6_i182;
  real_T c6_c_a[16];
  real_T c6_b[16];
  int32_T c6_i183;
  int32_T c6_i184;
  int32_T c6_i185;
  real_T c6_C[16];
  int32_T c6_i186;
  int32_T c6_i187;
  int32_T c6_i188;
  int32_T c6_i189;
  int32_T c6_i190;
  int32_T c6_i191;
  int32_T c6_i192;
  int32_T c6_i193;
  int32_T c6_i194;
  static real_T c6_dv25[9] = { 3.749399456654644E-33, 6.123233995736766E-17, 1.0,
    -1.0, 6.123233995736766E-17, 0.0, -6.123233995736766E-17, -1.0,
    6.123233995736766E-17 };

  int32_T c6_i195;
  static real_T c6_b_b[16] = { 3.749399456654644E-33, 6.123233995736766E-17, 1.0,
    0.0, -1.0, 6.123233995736766E-17, 0.0, 0.0, -6.123233995736766E-17, -1.0,
    6.123233995736766E-17, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T c6_i196;
  int32_T c6_i197;
  int32_T c6_i198;
  int32_T c6_i199;
  int32_T c6_i200;
  int32_T c6_i201;
  int32_T c6_i202;
  int32_T c6_i203;
  int32_T c6_i204;
  int32_T c6_i205;
  int32_T c6_i206;
  int32_T c6_i207;
  int32_T c6_i208;
  int32_T c6_i209;
  sf_debug_symbol_scope_push_eml(0U, 10U, 10U, c6_d_debug_family_names,
    c6_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c6_g, 0U, c6_j_sf_marshallOut,
    c6_i_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c6_Rtemp, 1U, c6_k_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c6_tool_hom_mat, 2U, c6_j_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c6_nargin, 3U, c6_g_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_nargout, 4U, c6_g_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_q, 5U, c6_d_sf_marshallOut,
    c6_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_b_d, 6U, c6_d_sf_marshallOut,
    c6_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_b_a, 7U, c6_d_sf_marshallOut,
    c6_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_b_alpha, 8U, c6_d_sf_marshallOut,
    c6_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_vec, 9U, c6_e_sf_marshallOut,
    c6_g_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 3);
  c6_b_eye(chartInstance, c6_dv24);
  for (c6_i181 = 0; c6_i181 < 16; c6_i181++) {
    c6_g[c6_i181] = c6_dv24[c6_i181];
  }

  c6_i = 0;
  while (c6_i < 6) {
    c6_b_i = 1.0 + (real_T)c6_i;
    CV_SCRIPT_FOR(0, 0, 1);
    _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 5);
    for (c6_i182 = 0; c6_i182 < 16; c6_i182++) {
      c6_c_a[c6_i182] = c6_g[c6_i182];
    }

    c6_dh_homogenous(chartInstance, c6_q[_SFD_EML_ARRAY_BOUNDS_CHECK("q",
      (int32_T)_SFD_INTEGER_CHECK("i", c6_b_i), 1, 6, 1, 0) - 1],
                     c6_b_d[_SFD_EML_ARRAY_BOUNDS_CHECK("d", (int32_T)
      _SFD_INTEGER_CHECK("i", c6_b_i), 1, 6, 1, 0) - 1],
                     c6_b_a[_SFD_EML_ARRAY_BOUNDS_CHECK("a", (int32_T)
      _SFD_INTEGER_CHECK("i", c6_b_i), 1, 6, 1, 0) - 1],
                     c6_b_alpha[_SFD_EML_ARRAY_BOUNDS_CHECK("alpha", (int32_T)
      _SFD_INTEGER_CHECK("i", c6_b_i), 1, 6, 1, 0) - 1], c6_b);
    c6_b_eml_scalar_eg(chartInstance);
    c6_b_eml_scalar_eg(chartInstance);
    for (c6_i183 = 0; c6_i183 < 16; c6_i183++) {
      c6_g[c6_i183] = 0.0;
    }

    for (c6_i184 = 0; c6_i184 < 16; c6_i184++) {
      c6_g[c6_i184] = 0.0;
    }

    for (c6_i185 = 0; c6_i185 < 16; c6_i185++) {
      c6_C[c6_i185] = c6_g[c6_i185];
    }

    for (c6_i186 = 0; c6_i186 < 16; c6_i186++) {
      c6_g[c6_i186] = c6_C[c6_i186];
    }

    for (c6_i187 = 0; c6_i187 < 16; c6_i187++) {
      c6_C[c6_i187] = c6_g[c6_i187];
    }

    for (c6_i188 = 0; c6_i188 < 16; c6_i188++) {
      c6_g[c6_i188] = c6_C[c6_i188];
    }

    for (c6_i189 = 0; c6_i189 < 4; c6_i189++) {
      c6_i190 = 0;
      for (c6_i191 = 0; c6_i191 < 4; c6_i191++) {
        c6_g[c6_i190 + c6_i189] = 0.0;
        c6_i192 = 0;
        for (c6_i193 = 0; c6_i193 < 4; c6_i193++) {
          c6_g[c6_i190 + c6_i189] += c6_c_a[c6_i192 + c6_i189] * c6_b[c6_i193 +
            c6_i190];
          c6_i192 += 4;
        }

        c6_i190 += 4;
      }
    }

    c6_i++;
    sf_mex_listen_for_ctrl_c(chartInstance->S);
  }

  CV_SCRIPT_FOR(0, 0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 8);
  for (c6_i194 = 0; c6_i194 < 9; c6_i194++) {
    c6_Rtemp[c6_i194] = c6_dv25[c6_i194];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 9);
  for (c6_i195 = 0; c6_i195 < 16; c6_i195++) {
    c6_tool_hom_mat[c6_i195] = c6_b_b[c6_i195];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 10);
  for (c6_i196 = 0; c6_i196 < 16; c6_i196++) {
    c6_c_a[c6_i196] = c6_g[c6_i196];
  }

  c6_b_eml_scalar_eg(chartInstance);
  c6_b_eml_scalar_eg(chartInstance);
  for (c6_i197 = 0; c6_i197 < 16; c6_i197++) {
    c6_g[c6_i197] = 0.0;
  }

  for (c6_i198 = 0; c6_i198 < 16; c6_i198++) {
    c6_g[c6_i198] = 0.0;
  }

  for (c6_i199 = 0; c6_i199 < 16; c6_i199++) {
    c6_C[c6_i199] = c6_g[c6_i199];
  }

  for (c6_i200 = 0; c6_i200 < 16; c6_i200++) {
    c6_g[c6_i200] = c6_C[c6_i200];
  }

  for (c6_i201 = 0; c6_i201 < 16; c6_i201++) {
    c6_C[c6_i201] = c6_g[c6_i201];
  }

  for (c6_i202 = 0; c6_i202 < 16; c6_i202++) {
    c6_g[c6_i202] = c6_C[c6_i202];
  }

  for (c6_i203 = 0; c6_i203 < 4; c6_i203++) {
    c6_i204 = 0;
    for (c6_i205 = 0; c6_i205 < 4; c6_i205++) {
      c6_g[c6_i204 + c6_i203] = 0.0;
      c6_i206 = 0;
      for (c6_i207 = 0; c6_i207 < 4; c6_i207++) {
        c6_g[c6_i204 + c6_i203] += c6_c_a[c6_i206 + c6_i203] * c6_b_b[c6_i207 +
          c6_i204];
        c6_i206 += 4;
      }

      c6_i204 += 4;
    }
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 11);
  for (c6_i208 = 0; c6_i208 < 3; c6_i208++) {
    c6_vec[c6_i208] = 0.0;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 12);
  for (c6_i209 = 0; c6_i209 < 3; c6_i209++) {
    c6_vec[c6_i209] = c6_g[c6_i209 + 12];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, -12);
  sf_debug_symbol_scope_pop();
}

static void c6_b_eye(SFc6_kinematicsInstanceStruct *chartInstance, real_T c6_I
                     [16])
{
  int32_T c6_i210;
  int32_T c6_i;
  int32_T c6_b_i;
  c6_isVariableSizing(chartInstance);
  for (c6_i210 = 0; c6_i210 < 16; c6_i210++) {
    c6_I[c6_i210] = 0.0;
  }

  c6_b_eml_int_forloop_overflow_check(chartInstance);
  for (c6_i = 1; c6_i < 5; c6_i++) {
    c6_b_i = c6_i;
    c6_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_i), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_b_i), 1, 4, 2, 0) - 1) <<
           2)) - 1] = 1.0;
  }
}

static void c6_b_eml_int_forloop_overflow_check(SFc6_kinematicsInstanceStruct
  *chartInstance)
{
}

static void c6_dh_homogenous(SFc6_kinematicsInstanceStruct *chartInstance,
  real_T c6_theta, real_T c6_b_d, real_T c6_b_a, real_T c6_b_alpha, real_T
  c6_mat[16])
{
  uint32_T c6_debug_family_var_map[7];
  real_T c6_nargin = 4.0;
  real_T c6_nargout = 1.0;
  real_T c6_dv26[16];
  int32_T c6_i211;
  real_T c6_x;
  real_T c6_b_x;
  real_T c6_c_x;
  real_T c6_d_x;
  real_T c6_e_x;
  real_T c6_f_x;
  real_T c6_c_a;
  real_T c6_b;
  real_T c6_y;
  real_T c6_g_x;
  real_T c6_h_x;
  real_T c6_i_x;
  real_T c6_j_x;
  real_T c6_d_a;
  real_T c6_b_b;
  real_T c6_b_y;
  real_T c6_k_x;
  real_T c6_l_x;
  real_T c6_e_a;
  real_T c6_c_b;
  real_T c6_c_y;
  real_T c6_m_x;
  real_T c6_n_x;
  real_T c6_o_x;
  real_T c6_p_x;
  real_T c6_q_x;
  real_T c6_r_x;
  real_T c6_f_a;
  real_T c6_d_b;
  real_T c6_d_y;
  real_T c6_s_x;
  real_T c6_t_x;
  real_T c6_u_x;
  real_T c6_v_x;
  real_T c6_g_a;
  real_T c6_e_b;
  real_T c6_e_y;
  real_T c6_w_x;
  real_T c6_x_x;
  real_T c6_h_a;
  real_T c6_f_b;
  real_T c6_f_y;
  real_T c6_y_x;
  real_T c6_ab_x;
  real_T c6_bb_x;
  real_T c6_cb_x;
  int32_T c6_i212;
  int32_T c6_i213;
  static real_T c6_dv27[4] = { 0.0, 0.0, 0.0, 1.0 };

  sf_debug_symbol_scope_push_eml(0U, 7U, 7U, c6_b_debug_family_names,
    c6_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(&c6_nargin, 0U, c6_g_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_nargout, 1U, c6_g_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_theta, 2U, c6_g_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_b_d, 3U, c6_g_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_b_a, 4U, c6_g_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c6_b_alpha, 5U, c6_g_sf_marshallOut,
    c6_e_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c6_mat, 6U, c6_j_sf_marshallOut,
    c6_i_sf_marshallIn);
  CV_SCRIPT_FCN(0, 1);
  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 36);
  c6_b_eye(chartInstance, c6_dv26);
  for (c6_i211 = 0; c6_i211 < 16; c6_i211++) {
    c6_mat[c6_i211] = c6_dv26[c6_i211];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, 37);
  c6_x = c6_theta;
  c6_b_x = c6_x;
  c6_b_x = muDoubleScalarCos(c6_b_x);
  c6_c_x = c6_theta;
  c6_d_x = c6_c_x;
  c6_d_x = muDoubleScalarSin(c6_d_x);
  c6_e_x = c6_b_alpha;
  c6_f_x = c6_e_x;
  c6_f_x = muDoubleScalarCos(c6_f_x);
  c6_c_a = -c6_d_x;
  c6_b = c6_f_x;
  c6_y = c6_c_a * c6_b;
  c6_g_x = c6_theta;
  c6_h_x = c6_g_x;
  c6_h_x = muDoubleScalarSin(c6_h_x);
  c6_i_x = c6_b_alpha;
  c6_j_x = c6_i_x;
  c6_j_x = muDoubleScalarSin(c6_j_x);
  c6_d_a = c6_h_x;
  c6_b_b = c6_j_x;
  c6_b_y = c6_d_a * c6_b_b;
  c6_k_x = c6_theta;
  c6_l_x = c6_k_x;
  c6_l_x = muDoubleScalarCos(c6_l_x);
  c6_e_a = c6_b_a;
  c6_c_b = c6_l_x;
  c6_c_y = c6_e_a * c6_c_b;
  c6_m_x = c6_theta;
  c6_n_x = c6_m_x;
  c6_n_x = muDoubleScalarSin(c6_n_x);
  c6_o_x = c6_theta;
  c6_p_x = c6_o_x;
  c6_p_x = muDoubleScalarCos(c6_p_x);
  c6_q_x = c6_b_alpha;
  c6_r_x = c6_q_x;
  c6_r_x = muDoubleScalarCos(c6_r_x);
  c6_f_a = c6_p_x;
  c6_d_b = c6_r_x;
  c6_d_y = c6_f_a * c6_d_b;
  c6_s_x = c6_theta;
  c6_t_x = c6_s_x;
  c6_t_x = muDoubleScalarCos(c6_t_x);
  c6_u_x = c6_b_alpha;
  c6_v_x = c6_u_x;
  c6_v_x = muDoubleScalarSin(c6_v_x);
  c6_g_a = -c6_t_x;
  c6_e_b = c6_v_x;
  c6_e_y = c6_g_a * c6_e_b;
  c6_w_x = c6_theta;
  c6_x_x = c6_w_x;
  c6_x_x = muDoubleScalarSin(c6_x_x);
  c6_h_a = c6_b_a;
  c6_f_b = c6_x_x;
  c6_f_y = c6_h_a * c6_f_b;
  c6_y_x = c6_b_alpha;
  c6_ab_x = c6_y_x;
  c6_ab_x = muDoubleScalarSin(c6_ab_x);
  c6_bb_x = c6_b_alpha;
  c6_cb_x = c6_bb_x;
  c6_cb_x = muDoubleScalarCos(c6_cb_x);
  c6_mat[0] = c6_b_x;
  c6_mat[4] = c6_y;
  c6_mat[8] = c6_b_y;
  c6_mat[12] = c6_c_y;
  c6_mat[1] = c6_n_x;
  c6_mat[5] = c6_d_y;
  c6_mat[9] = c6_e_y;
  c6_mat[13] = c6_f_y;
  c6_mat[2] = 0.0;
  c6_mat[6] = c6_ab_x;
  c6_mat[10] = c6_cb_x;
  c6_mat[14] = c6_b_d;
  c6_i212 = 0;
  for (c6_i213 = 0; c6_i213 < 4; c6_i213++) {
    c6_mat[c6_i212 + 3] = c6_dv27[c6_i213];
    c6_i212 += 4;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c6_sfEvent, -37);
  sf_debug_symbol_scope_pop();
}

static void c6_b_eml_scalar_eg(SFc6_kinematicsInstanceStruct *chartInstance)
{
}

static void c6_inv(SFc6_kinematicsInstanceStruct *chartInstance, real_T c6_x[144],
                   real_T c6_y[144])
{
  int32_T c6_i214;
  real_T c6_b_x[144];
  int32_T c6_i215;
  real_T c6_c_x[144];
  real_T c6_n1x;
  int32_T c6_i216;
  real_T c6_b_y[144];
  real_T c6_n1xinv;
  real_T c6_b_a;
  real_T c6_b;
  real_T c6_c_y;
  real_T c6_rc;
  real_T c6_d_x;
  boolean_T c6_b_b;
  char_T c6_cv0[14];
  int32_T c6_i217;
  char_T c6_cv1[14];
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  boolean_T guard3 = FALSE;
  for (c6_i214 = 0; c6_i214 < 144; c6_i214++) {
    c6_b_x[c6_i214] = c6_x[c6_i214];
  }

  c6_invNxN(chartInstance, c6_b_x, c6_y);
  for (c6_i215 = 0; c6_i215 < 144; c6_i215++) {
    c6_c_x[c6_i215] = c6_x[c6_i215];
  }

  c6_n1x = c6_norm(chartInstance, c6_c_x);
  for (c6_i216 = 0; c6_i216 < 144; c6_i216++) {
    c6_b_y[c6_i216] = c6_y[c6_i216];
  }

  c6_n1xinv = c6_norm(chartInstance, c6_b_y);
  c6_b_a = c6_n1x;
  c6_b = c6_n1xinv;
  c6_c_y = c6_b_a * c6_b;
  c6_rc = 1.0 / c6_c_y;
  guard1 = FALSE;
  guard2 = FALSE;
  if (c6_n1x == 0.0) {
    guard2 = TRUE;
  } else if (c6_n1xinv == 0.0) {
    guard2 = TRUE;
  } else if (c6_rc == 0.0) {
    guard1 = TRUE;
  } else {
    c6_d_x = c6_rc;
    c6_b_b = muDoubleScalarIsNaN(c6_d_x);
    guard3 = FALSE;
    if (c6_b_b) {
      guard3 = TRUE;
    } else {
      c6_eps(chartInstance);
      if (c6_rc < 2.2204460492503131E-16) {
        guard3 = TRUE;
      }
    }

    if (guard3 == TRUE) {
      c6_eml_flt2str(chartInstance, c6_rc, c6_cv0);
      for (c6_i217 = 0; c6_i217 < 14; c6_i217++) {
        c6_cv1[c6_i217] = c6_cv0[c6_i217];
      }

      c6_b_eml_warning(chartInstance, c6_cv1);
    }
  }

  if (guard2 == TRUE) {
    guard1 = TRUE;
  }

  if (guard1 == TRUE) {
    c6_eml_warning(chartInstance);
  }
}

static void c6_invNxN(SFc6_kinematicsInstanceStruct *chartInstance, real_T c6_x
                      [144], real_T c6_y[144])
{
  int32_T c6_i218;
  int32_T c6_info;
  int32_T c6_ipiv[12];
  int32_T c6_i219;
  int32_T c6_p[12];
  int32_T c6_k;
  real_T c6_b_k;
  int32_T c6_ipk;
  int32_T c6_pipk;
  int32_T c6_c_k;
  int32_T c6_d_k;
  int32_T c6_c;
  int32_T c6_e_k;
  int32_T c6_j;
  int32_T c6_b_j;
  int32_T c6_b_a;
  int32_T c6_i220;
  int32_T c6_i;
  int32_T c6_b_i;
  real_T c6_c_a;
  real_T c6_b;
  real_T c6_b_y;
  int32_T c6_i221;
  real_T c6_b_x[144];
  for (c6_i218 = 0; c6_i218 < 144; c6_i218++) {
    c6_y[c6_i218] = 0.0;
  }

  c6_c_eml_matlab_zgetrf(chartInstance, c6_x, c6_ipiv, &c6_info);
  for (c6_i219 = 0; c6_i219 < 12; c6_i219++) {
    c6_p[c6_i219] = 1 + c6_i219;
  }

  for (c6_k = 0; c6_k < 11; c6_k++) {
    c6_b_k = 1.0 + (real_T)c6_k;
    c6_ipk = c6_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
      ("", c6_b_k), 1, 12, 1, 0) - 1];
    if ((real_T)c6_ipk > c6_b_k) {
      c6_pipk = c6_p[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
        ("", (real_T)c6_ipk), 1, 12, 1, 0) - 1];
      c6_p[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c6_ipk), 1, 12, 1, 0) - 1] = c6_p[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", c6_b_k), 1, 12, 1, 0) - 1];
      c6_p[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        c6_b_k), 1, 12, 1, 0) - 1] = c6_pipk;
    }
  }

  c6_eml_int_forloop_overflow_check(chartInstance);
  for (c6_c_k = 1; c6_c_k < 13; c6_c_k++) {
    c6_d_k = c6_c_k;
    c6_c = c6_p[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c6_d_k), 1, 12, 1, 0) - 1];
    c6_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_d_k), 1, 12, 1, 0) + 12 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_c), 1, 12, 2, 0) - 1)) -
      1] = 1.0;
    c6_e_k = c6_d_k;
    c6_e_eml_int_forloop_overflow_check(chartInstance, c6_e_k, 12);
    for (c6_j = c6_e_k; c6_j < 13; c6_j++) {
      c6_b_j = c6_j;
      if (c6_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c6_b_j), 1, 12, 1, 0) + 12 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
             "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_c), 1, 12, 2, 0) - 1))
          - 1] != 0.0) {
        c6_b_a = c6_b_j + 1;
        c6_i220 = c6_b_a;
        c6_e_eml_int_forloop_overflow_check(chartInstance, c6_i220, 12);
        for (c6_i = c6_i220; c6_i < 13; c6_i++) {
          c6_b_i = c6_i;
          c6_c_a = c6_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_b_j), 1, 12, 1, 0) + 12 *
                         (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_c), 1, 12, 2, 0) - 1)) - 1];
          c6_b = c6_x[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_b_i), 1, 12, 1, 0) + 12 *
                       (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_b_j), 1, 12, 2, 0) - 1)) - 1];
          c6_b_y = c6_c_a * c6_b;
          c6_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c6_b_i), 1, 12, 1, 0) + 12 *
                (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c6_c), 1, 12, 2, 0) - 1)) - 1] = c6_y
            [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c6_b_i), 1, 12, 1, 0) + 12 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c6_c), 1, 12, 2, 0) - 1)) - 1] - c6_b_y;
        }
      }
    }
  }

  for (c6_i221 = 0; c6_i221 < 144; c6_i221++) {
    c6_b_x[c6_i221] = c6_x[c6_i221];
  }

  c6_c_eml_xtrsm(chartInstance, c6_b_x, c6_y);
}

static void c6_realmin(SFc6_kinematicsInstanceStruct *chartInstance)
{
}

static void c6_eps(SFc6_kinematicsInstanceStruct *chartInstance)
{
}

static void c6_eml_matlab_zgetrf(SFc6_kinematicsInstanceStruct *chartInstance,
  real_T c6_A[144], real_T c6_b_A[144], int32_T c6_ipiv[12], int32_T *c6_info)
{
  int32_T c6_i222;
  for (c6_i222 = 0; c6_i222 < 144; c6_i222++) {
    c6_b_A[c6_i222] = c6_A[c6_i222];
  }

  c6_c_eml_matlab_zgetrf(chartInstance, c6_b_A, c6_ipiv, c6_info);
}

static void c6_c_eml_int_forloop_overflow_check(SFc6_kinematicsInstanceStruct
  *chartInstance)
{
}

static int32_T c6_eml_ixamax(SFc6_kinematicsInstanceStruct *chartInstance,
  int32_T c6_n, real_T c6_x[144], int32_T c6_ix0)
{
  int32_T c6_idxmax;
  int32_T c6_b_n;
  int32_T c6_b_ix0;
  int32_T c6_c_n;
  int32_T c6_c_ix0;
  int32_T c6_ix;
  real_T c6_b_x;
  real_T c6_c_x;
  real_T c6_d_x;
  real_T c6_y;
  real_T c6_e_x;
  real_T c6_f_x;
  real_T c6_b_y;
  real_T c6_smax;
  int32_T c6_loop_ub;
  int32_T c6_k;
  int32_T c6_b_k;
  int32_T c6_b_a;
  real_T c6_g_x;
  real_T c6_h_x;
  real_T c6_i_x;
  real_T c6_c_y;
  real_T c6_j_x;
  real_T c6_k_x;
  real_T c6_d_y;
  real_T c6_s;
  c6_b_n = c6_n;
  c6_b_ix0 = c6_ix0;
  c6_c_n = c6_b_n;
  c6_c_ix0 = c6_b_ix0;
  if ((real_T)c6_c_n < 1.0) {
    c6_idxmax = 0;
  } else {
    c6_idxmax = 1;
    if ((real_T)c6_c_n > 1.0) {
      c6_ix = c6_c_ix0;
      c6_b_x = c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c6_ix), 1, 144, 1, 0) - 1];
      c6_c_x = c6_b_x;
      c6_d_x = c6_c_x;
      c6_y = muDoubleScalarAbs(c6_d_x);
      c6_e_x = 0.0;
      c6_f_x = c6_e_x;
      c6_b_y = muDoubleScalarAbs(c6_f_x);
      c6_smax = c6_y + c6_b_y;
      c6_d_eml_int_forloop_overflow_check(chartInstance, c6_c_n);
      c6_loop_ub = c6_c_n;
      for (c6_k = 2; c6_k <= c6_loop_ub; c6_k++) {
        c6_b_k = c6_k;
        c6_b_a = c6_ix + 1;
        c6_ix = c6_b_a;
        c6_g_x = c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c6_ix), 1, 144, 1, 0) - 1];
        c6_h_x = c6_g_x;
        c6_i_x = c6_h_x;
        c6_c_y = muDoubleScalarAbs(c6_i_x);
        c6_j_x = 0.0;
        c6_k_x = c6_j_x;
        c6_d_y = muDoubleScalarAbs(c6_k_x);
        c6_s = c6_c_y + c6_d_y;
        if (c6_s > c6_smax) {
          c6_idxmax = c6_b_k;
          c6_smax = c6_s;
        }
      }
    }
  }

  return c6_idxmax;
}

static void c6_d_eml_int_forloop_overflow_check(SFc6_kinematicsInstanceStruct
  *chartInstance, int32_T c6_b)
{
  int32_T c6_b_b;
  boolean_T c6_overflow;
  boolean_T c6_safe;
  int32_T c6_i223;
  static char_T c6_cv2[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c6_u[34];
  const mxArray *c6_y = NULL;
  int32_T c6_i224;
  static char_T c6_cv3[5] = { 'i', 'n', 't', '3', '2' };

  char_T c6_b_u[5];
  const mxArray *c6_b_y = NULL;
  c6_b_b = c6_b;
  if (2 > c6_b_b) {
    c6_overflow = FALSE;
  } else {
    c6_overflow = (c6_b_b > 2147483646);
  }

  c6_safe = !c6_overflow;
  if (c6_safe) {
  } else {
    for (c6_i223 = 0; c6_i223 < 34; c6_i223++) {
      c6_u[c6_i223] = c6_cv2[c6_i223];
    }

    c6_y = NULL;
    sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c6_i224 = 0; c6_i224 < 5; c6_i224++) {
      c6_b_u[c6_i224] = c6_cv3[c6_i224];
    }

    c6_b_y = NULL;
    sf_mex_assign(&c6_b_y, sf_mex_create("y", c6_b_u, 10, 0U, 1U, 0U, 2, 1, 5),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c6_y, 14, c6_b_y));
  }
}

static void c6_e_eml_int_forloop_overflow_check(SFc6_kinematicsInstanceStruct
  *chartInstance, int32_T c6_b_a, int32_T c6_b)
{
  int32_T c6_c_a;
  int32_T c6_b_b;
  boolean_T c6_overflow;
  boolean_T c6_safe;
  int32_T c6_i225;
  static char_T c6_cv4[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c6_u[34];
  const mxArray *c6_y = NULL;
  int32_T c6_i226;
  static char_T c6_cv5[5] = { 'i', 'n', 't', '3', '2' };

  char_T c6_b_u[5];
  const mxArray *c6_b_y = NULL;
  c6_c_a = c6_b_a;
  c6_b_b = c6_b;
  if (c6_c_a > c6_b_b) {
    c6_overflow = FALSE;
  } else {
    c6_overflow = (c6_b_b > 2147483646);
  }

  c6_safe = !c6_overflow;
  if (c6_safe) {
  } else {
    for (c6_i225 = 0; c6_i225 < 34; c6_i225++) {
      c6_u[c6_i225] = c6_cv4[c6_i225];
    }

    c6_y = NULL;
    sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c6_i226 = 0; c6_i226 < 5; c6_i226++) {
      c6_b_u[c6_i226] = c6_cv5[c6_i226];
    }

    c6_b_y = NULL;
    sf_mex_assign(&c6_b_y, sf_mex_create("y", c6_b_u, 10, 0U, 1U, 0U, 2, 1, 5),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c6_y, 14, c6_b_y));
  }
}

static void c6_eml_xtrsm(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_A[144], real_T c6_B[144], real_T c6_b_B[144])
{
  int32_T c6_i227;
  int32_T c6_i228;
  real_T c6_b_A[144];
  for (c6_i227 = 0; c6_i227 < 144; c6_i227++) {
    c6_b_B[c6_i227] = c6_B[c6_i227];
  }

  for (c6_i228 = 0; c6_i228 < 144; c6_i228++) {
    c6_b_A[c6_i228] = c6_A[c6_i228];
  }

  c6_c_eml_xtrsm(chartInstance, c6_b_A, c6_b_B);
}

static real_T c6_norm(SFc6_kinematicsInstanceStruct *chartInstance, real_T c6_x
                      [144])
{
  real_T c6_y;
  int32_T c6_j;
  real_T c6_b_j;
  real_T c6_s;
  int32_T c6_i;
  real_T c6_b_i;
  real_T c6_b_x;
  real_T c6_c_x;
  real_T c6_b_y;
  real_T c6_d_x;
  boolean_T c6_b;
  boolean_T exitg1;
  c6_y = 0.0;
  c6_j = 0;
  exitg1 = FALSE;
  while ((exitg1 == 0U) && (c6_j < 12)) {
    c6_b_j = 1.0 + (real_T)c6_j;
    c6_s = 0.0;
    for (c6_i = 0; c6_i < 12; c6_i++) {
      c6_b_i = 1.0 + (real_T)c6_i;
      c6_b_x = c6_x[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
                      ("", c6_b_i), 1, 12, 1, 0) + 12 *
                     (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", c6_b_j), 1, 12, 2, 0) - 1)) - 1];
      c6_c_x = c6_b_x;
      c6_b_y = muDoubleScalarAbs(c6_c_x);
      c6_s += c6_b_y;
    }

    c6_d_x = c6_s;
    c6_b = muDoubleScalarIsNaN(c6_d_x);
    if (c6_b) {
      c6_y = rtNaN;
      exitg1 = TRUE;
    } else {
      if (c6_s > c6_y) {
        c6_y = c6_s;
      }

      c6_j++;
    }
  }

  return c6_y;
}

static void c6_eml_warning(SFc6_kinematicsInstanceStruct *chartInstance)
{
  int32_T c6_i229;
  static char_T c6_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  char_T c6_u[27];
  const mxArray *c6_y = NULL;
  for (c6_i229 = 0; c6_i229 < 27; c6_i229++) {
    c6_u[c6_i229] = c6_varargin_1[c6_i229];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 27), FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U,
    14, c6_y));
}

static void c6_eml_flt2str(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_x, char_T c6_str[14])
{
  int32_T c6_i230;
  static char_T c6_cv6[8] = { '%', '%', '%', 'd', '.', '%', 'd', 'e' };

  char_T c6_u[8];
  const mxArray *c6_y = NULL;
  real_T c6_b_u;
  const mxArray *c6_b_y = NULL;
  real_T c6_c_u;
  const mxArray *c6_c_y = NULL;
  real_T c6_d_u;
  const mxArray *c6_d_y = NULL;
  for (c6_i230 = 0; c6_i230 < 8; c6_i230++) {
    c6_u[c6_i230] = c6_cv6[c6_i230];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 8), FALSE);
  c6_b_u = 14.0;
  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", &c6_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  c6_c_u = 6.0;
  c6_c_y = NULL;
  sf_mex_assign(&c6_c_y, sf_mex_create("y", &c6_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  c6_d_u = c6_x;
  c6_d_y = NULL;
  sf_mex_assign(&c6_d_y, sf_mex_create("y", &c6_d_u, 0, 0U, 0U, 0U, 0), FALSE);
  c6_n_emlrt_marshallIn(chartInstance, sf_mex_call_debug("sprintf", 1U, 2U, 14,
    sf_mex_call_debug("sprintf", 1U, 3U, 14, c6_y, 14, c6_b_y, 14, c6_c_y), 14,
    c6_d_y), "sprintf", c6_str);
}

static void c6_b_eml_warning(SFc6_kinematicsInstanceStruct *chartInstance,
  char_T c6_varargin_2[14])
{
  int32_T c6_i231;
  static char_T c6_varargin_1[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'i', 'l', 'l', 'C', 'o', 'n', 'd', 'i', 't', 'i',
    'o', 'n', 'e', 'd', 'M', 'a', 't', 'r', 'i', 'x' };

  char_T c6_u[33];
  const mxArray *c6_y = NULL;
  int32_T c6_i232;
  char_T c6_b_u[14];
  const mxArray *c6_b_y = NULL;
  for (c6_i231 = 0; c6_i231 < 33; c6_i231++) {
    c6_u[c6_i231] = c6_varargin_1[c6_i231];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 10, 0U, 1U, 0U, 2, 1, 33), FALSE);
  for (c6_i232 = 0; c6_i232 < 14; c6_i232++) {
    c6_b_u[c6_i232] = c6_varargin_2[c6_i232];
  }

  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", c6_b_u, 10, 0U, 1U, 0U, 2, 1, 14),
                FALSE);
  sf_mex_call_debug("warning", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
    14, c6_y, 14, c6_b_y));
}

static void c6_c_eml_scalar_eg(SFc6_kinematicsInstanceStruct *chartInstance)
{
}

static void c6_eml_xgemm(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_A[72], real_T c6_B[144], real_T c6_C[72], real_T c6_b_C[72])
{
  int32_T c6_i233;
  int32_T c6_i234;
  real_T c6_b_A[72];
  int32_T c6_i235;
  real_T c6_b_B[144];
  for (c6_i233 = 0; c6_i233 < 72; c6_i233++) {
    c6_b_C[c6_i233] = c6_C[c6_i233];
  }

  for (c6_i234 = 0; c6_i234 < 72; c6_i234++) {
    c6_b_A[c6_i234] = c6_A[c6_i234];
  }

  for (c6_i235 = 0; c6_i235 < 144; c6_i235++) {
    c6_b_B[c6_i235] = c6_B[c6_i235];
  }

  c6_c_eml_xgemm(chartInstance, c6_b_A, c6_b_B, c6_b_C);
}

static void c6_d_eml_scalar_eg(SFc6_kinematicsInstanceStruct *chartInstance)
{
}

static void c6_b_inv(SFc6_kinematicsInstanceStruct *chartInstance, real_T c6_x
                     [36], real_T c6_y[36])
{
  int32_T c6_i236;
  real_T c6_b_x[36];
  int32_T c6_i237;
  real_T c6_c_x[36];
  real_T c6_n1x;
  int32_T c6_i238;
  real_T c6_b_y[36];
  real_T c6_n1xinv;
  real_T c6_b_a;
  real_T c6_b;
  real_T c6_c_y;
  real_T c6_rc;
  real_T c6_d_x;
  boolean_T c6_b_b;
  char_T c6_cv7[14];
  int32_T c6_i239;
  char_T c6_cv8[14];
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  boolean_T guard3 = FALSE;
  for (c6_i236 = 0; c6_i236 < 36; c6_i236++) {
    c6_b_x[c6_i236] = c6_x[c6_i236];
  }

  c6_b_invNxN(chartInstance, c6_b_x, c6_y);
  for (c6_i237 = 0; c6_i237 < 36; c6_i237++) {
    c6_c_x[c6_i237] = c6_x[c6_i237];
  }

  c6_n1x = c6_b_norm(chartInstance, c6_c_x);
  for (c6_i238 = 0; c6_i238 < 36; c6_i238++) {
    c6_b_y[c6_i238] = c6_y[c6_i238];
  }

  c6_n1xinv = c6_b_norm(chartInstance, c6_b_y);
  c6_b_a = c6_n1x;
  c6_b = c6_n1xinv;
  c6_c_y = c6_b_a * c6_b;
  c6_rc = 1.0 / c6_c_y;
  guard1 = FALSE;
  guard2 = FALSE;
  if (c6_n1x == 0.0) {
    guard2 = TRUE;
  } else if (c6_n1xinv == 0.0) {
    guard2 = TRUE;
  } else if (c6_rc == 0.0) {
    guard1 = TRUE;
  } else {
    c6_d_x = c6_rc;
    c6_b_b = muDoubleScalarIsNaN(c6_d_x);
    guard3 = FALSE;
    if (c6_b_b) {
      guard3 = TRUE;
    } else {
      c6_eps(chartInstance);
      if (c6_rc < 2.2204460492503131E-16) {
        guard3 = TRUE;
      }
    }

    if (guard3 == TRUE) {
      c6_eml_flt2str(chartInstance, c6_rc, c6_cv7);
      for (c6_i239 = 0; c6_i239 < 14; c6_i239++) {
        c6_cv8[c6_i239] = c6_cv7[c6_i239];
      }

      c6_b_eml_warning(chartInstance, c6_cv8);
    }
  }

  if (guard2 == TRUE) {
    guard1 = TRUE;
  }

  if (guard1 == TRUE) {
    c6_eml_warning(chartInstance);
  }
}

static void c6_b_invNxN(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_x[36], real_T c6_y[36])
{
  int32_T c6_i240;
  int32_T c6_info;
  int32_T c6_ipiv[6];
  int32_T c6_i241;
  int32_T c6_p[6];
  int32_T c6_k;
  real_T c6_b_k;
  int32_T c6_ipk;
  int32_T c6_pipk;
  int32_T c6_c_k;
  int32_T c6_d_k;
  int32_T c6_c;
  int32_T c6_e_k;
  int32_T c6_j;
  int32_T c6_b_j;
  int32_T c6_b_a;
  int32_T c6_i242;
  int32_T c6_i;
  int32_T c6_b_i;
  real_T c6_c_a;
  real_T c6_b;
  real_T c6_b_y;
  int32_T c6_i243;
  real_T c6_b_x[36];
  for (c6_i240 = 0; c6_i240 < 36; c6_i240++) {
    c6_y[c6_i240] = 0.0;
  }

  c6_d_eml_matlab_zgetrf(chartInstance, c6_x, c6_ipiv, &c6_info);
  for (c6_i241 = 0; c6_i241 < 6; c6_i241++) {
    c6_p[c6_i241] = 1 + c6_i241;
  }

  for (c6_k = 0; c6_k < 5; c6_k++) {
    c6_b_k = 1.0 + (real_T)c6_k;
    c6_ipk = c6_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
      ("", c6_b_k), 1, 6, 1, 0) - 1];
    if ((real_T)c6_ipk > c6_b_k) {
      c6_pipk = c6_p[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
        ("", (real_T)c6_ipk), 1, 6, 1, 0) - 1];
      c6_p[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c6_ipk), 1, 6, 1, 0) - 1] = c6_p[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", c6_b_k), 1, 6, 1, 0) - 1];
      c6_p[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        c6_b_k), 1, 6, 1, 0) - 1] = c6_pipk;
    }
  }

  c6_e_eml_int_forloop_overflow_check(chartInstance, 1, 6);
  for (c6_c_k = 1; c6_c_k < 7; c6_c_k++) {
    c6_d_k = c6_c_k;
    c6_c = c6_p[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c6_d_k), 1, 6, 1, 0) - 1];
    c6_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_d_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_c), 1, 6, 2, 0) - 1)) - 1]
      = 1.0;
    c6_e_k = c6_d_k;
    c6_e_eml_int_forloop_overflow_check(chartInstance, c6_e_k, 6);
    for (c6_j = c6_e_k; c6_j < 7; c6_j++) {
      c6_b_j = c6_j;
      if (c6_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c6_b_j), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_c), 1, 6, 2, 0) - 1)) -
          1] != 0.0) {
        c6_b_a = c6_b_j + 1;
        c6_i242 = c6_b_a;
        c6_e_eml_int_forloop_overflow_check(chartInstance, c6_i242, 6);
        for (c6_i = c6_i242; c6_i < 7; c6_i++) {
          c6_b_i = c6_i;
          c6_c_a = c6_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_b_j), 1, 6, 1, 0) + 6 *
                         (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_c), 1, 6, 2, 0) - 1)) - 1];
          c6_b = c6_x[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_b_i), 1, 6, 1, 0) + 6 *
                       (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_b_j), 1, 6, 2, 0) - 1)) - 1];
          c6_b_y = c6_c_a * c6_b;
          c6_y[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c6_b_i), 1, 6, 1, 0) + 6 *
                (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                   (real_T)c6_c), 1, 6, 2, 0) - 1)) - 1] = c6_y
            [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c6_b_i), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
                "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_c), 1, 6, 2, 0) -
               1)) - 1] - c6_b_y;
        }
      }
    }
  }

  for (c6_i243 = 0; c6_i243 < 36; c6_i243++) {
    c6_b_x[c6_i243] = c6_x[c6_i243];
  }

  c6_d_eml_xtrsm(chartInstance, c6_b_x, c6_y);
}

static void c6_b_eml_matlab_zgetrf(SFc6_kinematicsInstanceStruct *chartInstance,
  real_T c6_A[36], real_T c6_b_A[36], int32_T c6_ipiv[6], int32_T *c6_info)
{
  int32_T c6_i244;
  for (c6_i244 = 0; c6_i244 < 36; c6_i244++) {
    c6_b_A[c6_i244] = c6_A[c6_i244];
  }

  c6_d_eml_matlab_zgetrf(chartInstance, c6_b_A, c6_ipiv, c6_info);
}

static int32_T c6_b_eml_ixamax(SFc6_kinematicsInstanceStruct *chartInstance,
  int32_T c6_n, real_T c6_x[36], int32_T c6_ix0)
{
  int32_T c6_idxmax;
  int32_T c6_b_n;
  int32_T c6_b_ix0;
  int32_T c6_c_n;
  int32_T c6_c_ix0;
  int32_T c6_ix;
  real_T c6_b_x;
  real_T c6_c_x;
  real_T c6_d_x;
  real_T c6_y;
  real_T c6_e_x;
  real_T c6_f_x;
  real_T c6_b_y;
  real_T c6_smax;
  int32_T c6_loop_ub;
  int32_T c6_k;
  int32_T c6_b_k;
  int32_T c6_b_a;
  real_T c6_g_x;
  real_T c6_h_x;
  real_T c6_i_x;
  real_T c6_c_y;
  real_T c6_j_x;
  real_T c6_k_x;
  real_T c6_d_y;
  real_T c6_s;
  c6_b_n = c6_n;
  c6_b_ix0 = c6_ix0;
  c6_c_n = c6_b_n;
  c6_c_ix0 = c6_b_ix0;
  if ((real_T)c6_c_n < 1.0) {
    c6_idxmax = 0;
  } else {
    c6_idxmax = 1;
    if ((real_T)c6_c_n > 1.0) {
      c6_ix = c6_c_ix0;
      c6_b_x = c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c6_ix), 1, 36, 1, 0) - 1];
      c6_c_x = c6_b_x;
      c6_d_x = c6_c_x;
      c6_y = muDoubleScalarAbs(c6_d_x);
      c6_e_x = 0.0;
      c6_f_x = c6_e_x;
      c6_b_y = muDoubleScalarAbs(c6_f_x);
      c6_smax = c6_y + c6_b_y;
      c6_d_eml_int_forloop_overflow_check(chartInstance, c6_c_n);
      c6_loop_ub = c6_c_n;
      for (c6_k = 2; c6_k <= c6_loop_ub; c6_k++) {
        c6_b_k = c6_k;
        c6_b_a = c6_ix + 1;
        c6_ix = c6_b_a;
        c6_g_x = c6_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c6_ix), 1, 36, 1, 0) - 1];
        c6_h_x = c6_g_x;
        c6_i_x = c6_h_x;
        c6_c_y = muDoubleScalarAbs(c6_i_x);
        c6_j_x = 0.0;
        c6_k_x = c6_j_x;
        c6_d_y = muDoubleScalarAbs(c6_k_x);
        c6_s = c6_c_y + c6_d_y;
        if (c6_s > c6_smax) {
          c6_idxmax = c6_b_k;
          c6_smax = c6_s;
        }
      }
    }
  }

  return c6_idxmax;
}

static void c6_b_eml_xtrsm(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_A[36], real_T c6_B[36], real_T c6_b_B[36])
{
  int32_T c6_i245;
  int32_T c6_i246;
  real_T c6_b_A[36];
  for (c6_i245 = 0; c6_i245 < 36; c6_i245++) {
    c6_b_B[c6_i245] = c6_B[c6_i245];
  }

  for (c6_i246 = 0; c6_i246 < 36; c6_i246++) {
    c6_b_A[c6_i246] = c6_A[c6_i246];
  }

  c6_d_eml_xtrsm(chartInstance, c6_b_A, c6_b_B);
}

static void c6_f_eml_int_forloop_overflow_check(SFc6_kinematicsInstanceStruct
  *chartInstance)
{
}

static real_T c6_b_norm(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_x[36])
{
  real_T c6_y;
  int32_T c6_j;
  real_T c6_b_j;
  real_T c6_s;
  int32_T c6_i;
  real_T c6_b_i;
  real_T c6_b_x;
  real_T c6_c_x;
  real_T c6_b_y;
  real_T c6_d_x;
  boolean_T c6_b;
  boolean_T exitg1;
  c6_y = 0.0;
  c6_j = 0;
  exitg1 = FALSE;
  while ((exitg1 == 0U) && (c6_j < 6)) {
    c6_b_j = 1.0 + (real_T)c6_j;
    c6_s = 0.0;
    for (c6_i = 0; c6_i < 6; c6_i++) {
      c6_b_i = 1.0 + (real_T)c6_i;
      c6_b_x = c6_x[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
                      ("", c6_b_i), 1, 6, 1, 0) + 6 *
                     (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", c6_b_j), 1, 6, 2, 0) - 1)) - 1];
      c6_c_x = c6_b_x;
      c6_b_y = muDoubleScalarAbs(c6_c_x);
      c6_s += c6_b_y;
    }

    c6_d_x = c6_s;
    c6_b = muDoubleScalarIsNaN(c6_d_x);
    if (c6_b) {
      c6_y = rtNaN;
      exitg1 = TRUE;
    } else {
      if (c6_s > c6_y) {
        c6_y = c6_s;
      }

      c6_j++;
    }
  }

  return c6_y;
}

static void c6_e_eml_scalar_eg(SFc6_kinematicsInstanceStruct *chartInstance)
{
}

static void c6_b_eml_xgemm(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_A[144], real_T c6_B[72], real_T c6_C[72], real_T c6_b_C[72])
{
  int32_T c6_i247;
  int32_T c6_i248;
  real_T c6_b_A[144];
  int32_T c6_i249;
  real_T c6_b_B[72];
  for (c6_i247 = 0; c6_i247 < 72; c6_i247++) {
    c6_b_C[c6_i247] = c6_C[c6_i247];
  }

  for (c6_i248 = 0; c6_i248 < 144; c6_i248++) {
    c6_b_A[c6_i248] = c6_A[c6_i248];
  }

  for (c6_i249 = 0; c6_i249 < 72; c6_i249++) {
    c6_b_B[c6_i249] = c6_B[c6_i249];
  }

  c6_d_eml_xgemm(chartInstance, c6_b_A, c6_b_B, c6_b_C);
}

static void c6_f_eml_scalar_eg(SFc6_kinematicsInstanceStruct *chartInstance)
{
}

static void c6_g_eml_scalar_eg(SFc6_kinematicsInstanceStruct *chartInstance)
{
}

static void c6_n_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_sprintf, const char_T *c6_identifier, char_T c6_y[14])
{
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_o_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_sprintf), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_sprintf);
}

static void c6_o_emlrt_marshallIn(SFc6_kinematicsInstanceStruct *chartInstance,
  const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId, char_T c6_y[14])
{
  char_T c6_cv9[14];
  int32_T c6_i250;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_cv9, 1, 10, 0U, 1, 0U, 2, 1,
                14);
  for (c6_i250 = 0; c6_i250 < 14; c6_i250++) {
    c6_y[c6_i250] = c6_cv9[c6_i250];
  }

  sf_mex_destroy(&c6_u);
}

static const mxArray *c6_l_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(int32_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static int32_T c6_p_emlrt_marshallIn(SFc6_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  int32_T c6_y;
  int32_T c6_i251;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_i251, 1, 6, 0U, 0, 0U, 0);
  c6_y = c6_i251;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_sfEvent;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  int32_T c6_y;
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)chartInstanceVoid;
  c6_b_sfEvent = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_sfEvent),
    &c6_thisId);
  sf_mex_destroy(&c6_b_sfEvent);
  *(int32_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static uint8_T c6_q_emlrt_marshallIn(SFc6_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_b_is_active_c6_kinematics, const char_T
  *c6_identifier)
{
  uint8_T c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_r_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c6_b_is_active_c6_kinematics), &c6_thisId);
  sf_mex_destroy(&c6_b_is_active_c6_kinematics);
  return c6_y;
}

static uint8_T c6_r_emlrt_marshallIn(SFc6_kinematicsInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  uint8_T c6_y;
  uint8_T c6_u0;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_u0, 1, 3, 0U, 0, 0U, 0);
  c6_y = c6_u0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_c_eml_matlab_zgetrf(SFc6_kinematicsInstanceStruct *chartInstance,
  real_T c6_A[144], int32_T c6_ipiv[12], int32_T *c6_info)
{
  int32_T c6_i252;
  int32_T c6_j;
  int32_T c6_b_j;
  int32_T c6_b_a;
  int32_T c6_jm1;
  int32_T c6_b;
  int32_T c6_mmj;
  int32_T c6_c_a;
  int32_T c6_c;
  int32_T c6_b_b;
  int32_T c6_jj;
  int32_T c6_d_a;
  int32_T c6_jp1j;
  int32_T c6_e_a;
  int32_T c6_b_c;
  int32_T c6_i253;
  int32_T c6_i254;
  int32_T c6_i255;
  real_T c6_b_A[144];
  int32_T c6_f_a;
  int32_T c6_jpiv_offset;
  int32_T c6_g_a;
  int32_T c6_c_b;
  int32_T c6_jpiv;
  int32_T c6_h_a;
  int32_T c6_d_b;
  int32_T c6_c_c;
  int32_T c6_e_b;
  int32_T c6_jrow;
  int32_T c6_i_a;
  int32_T c6_f_b;
  int32_T c6_jprow;
  int32_T c6_ix0;
  int32_T c6_iy0;
  int32_T c6_b_ix0;
  int32_T c6_b_iy0;
  int32_T c6_c_ix0;
  int32_T c6_c_iy0;
  int32_T c6_ix;
  int32_T c6_iy;
  int32_T c6_k;
  real_T c6_temp;
  int32_T c6_j_a;
  int32_T c6_k_a;
  int32_T c6_b_jp1j;
  int32_T c6_l_a;
  int32_T c6_d_c;
  int32_T c6_m_a;
  int32_T c6_g_b;
  int32_T c6_i256;
  int32_T c6_i;
  int32_T c6_b_i;
  real_T c6_x;
  real_T c6_y;
  real_T c6_z;
  int32_T c6_h_b;
  int32_T c6_e_c;
  int32_T c6_n_a;
  int32_T c6_f_c;
  int32_T c6_o_a;
  int32_T c6_g_c;
  int32_T c6_m;
  int32_T c6_n;
  int32_T c6_d_ix0;
  int32_T c6_d_iy0;
  int32_T c6_ia0;
  int32_T c6_b_m;
  int32_T c6_b_n;
  int32_T c6_e_ix0;
  int32_T c6_e_iy0;
  int32_T c6_b_ia0;
  int32_T c6_c_m;
  int32_T c6_c_n;
  int32_T c6_f_ix0;
  int32_T c6_f_iy0;
  int32_T c6_c_ia0;
  int32_T c6_d_m;
  int32_T c6_d_n;
  int32_T c6_g_ix0;
  int32_T c6_g_iy0;
  int32_T c6_d_ia0;
  int32_T c6_e_m;
  int32_T c6_e_n;
  int32_T c6_h_ix0;
  int32_T c6_h_iy0;
  int32_T c6_e_ia0;
  int32_T c6_ixstart;
  int32_T c6_p_a;
  int32_T c6_jA;
  int32_T c6_jy;
  int32_T c6_loop_ub;
  int32_T c6_c_j;
  real_T c6_yjy;
  real_T c6_b_temp;
  int32_T c6_b_ix;
  int32_T c6_i_b;
  int32_T c6_i257;
  int32_T c6_q_a;
  int32_T c6_j_b;
  int32_T c6_i258;
  int32_T c6_ijA;
  int32_T c6_b_ijA;
  int32_T c6_r_a;
  int32_T c6_s_a;
  int32_T c6_t_a;
  c6_realmin(chartInstance);
  c6_eps(chartInstance);
  for (c6_i252 = 0; c6_i252 < 12; c6_i252++) {
    c6_ipiv[c6_i252] = 1 + c6_i252;
  }

  *c6_info = 0;
  c6_c_eml_int_forloop_overflow_check(chartInstance);
  for (c6_j = 1; c6_j < 12; c6_j++) {
    c6_b_j = c6_j;
    c6_b_a = c6_b_j - 1;
    c6_jm1 = c6_b_a;
    c6_b = c6_b_j;
    c6_mmj = 12 - c6_b;
    c6_c_a = c6_jm1;
    c6_c = c6_c_a * 13;
    c6_b_b = c6_c + 1;
    c6_jj = c6_b_b;
    c6_d_a = c6_jj + 1;
    c6_jp1j = c6_d_a;
    c6_e_a = c6_mmj;
    c6_b_c = c6_e_a;
    c6_i253 = 0;
    for (c6_i254 = 0; c6_i254 < 12; c6_i254++) {
      for (c6_i255 = 0; c6_i255 < 12; c6_i255++) {
        c6_b_A[c6_i255 + c6_i253] = c6_A[c6_i255 + c6_i253];
      }

      c6_i253 += 12;
    }

    c6_f_a = c6_eml_ixamax(chartInstance, c6_b_c + 1, c6_b_A, c6_jj) - 1;
    c6_jpiv_offset = c6_f_a;
    c6_g_a = c6_jj;
    c6_c_b = c6_jpiv_offset;
    c6_jpiv = c6_g_a + c6_c_b;
    if (c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_jpiv), 1, 144, 1, 0) - 1] != 0.0) {
      if ((real_T)c6_jpiv_offset != 0.0) {
        c6_h_a = c6_b_j;
        c6_d_b = c6_jpiv_offset;
        c6_c_c = c6_h_a + c6_d_b;
        c6_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_b_j), 1, 12, 1, 0) - 1] = c6_c_c;
        c6_e_b = c6_jm1 + 1;
        c6_jrow = c6_e_b;
        c6_i_a = c6_jrow;
        c6_f_b = c6_jpiv_offset;
        c6_jprow = c6_i_a + c6_f_b;
        c6_ix0 = c6_jrow;
        c6_iy0 = c6_jprow;
        c6_b_ix0 = c6_ix0;
        c6_b_iy0 = c6_iy0;
        c6_c_ix0 = c6_b_ix0;
        c6_c_iy0 = c6_b_iy0;
        c6_ix = c6_c_ix0;
        c6_iy = c6_c_iy0;
        c6_eml_int_forloop_overflow_check(chartInstance);
        for (c6_k = 1; c6_k < 13; c6_k++) {
          c6_temp = c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_ix), 1, 144, 1, 0) - 1];
          c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_ix), 1, 144, 1, 0) - 1] =
            c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_iy), 1, 144, 1, 0) - 1];
          c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_iy), 1, 144, 1, 0) - 1] = c6_temp;
          c6_j_a = c6_ix + 12;
          c6_ix = c6_j_a;
          c6_k_a = c6_iy + 12;
          c6_iy = c6_k_a;
        }
      }

      c6_b_jp1j = c6_jp1j;
      c6_l_a = c6_mmj;
      c6_d_c = c6_l_a;
      c6_m_a = c6_jp1j;
      c6_g_b = c6_d_c - 1;
      c6_i256 = c6_m_a + c6_g_b;
      c6_e_eml_int_forloop_overflow_check(chartInstance, c6_b_jp1j, c6_i256);
      for (c6_i = c6_b_jp1j; c6_i <= c6_i256; c6_i++) {
        c6_b_i = c6_i;
        c6_x = c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c6_b_i), 1, 144, 1, 0) - 1];
        c6_y = c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c6_jj), 1, 144, 1, 0) - 1];
        c6_z = c6_x / c6_y;
        c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_b_i), 1, 144, 1, 0) - 1] = c6_z;
      }
    } else {
      *c6_info = c6_b_j;
    }

    c6_h_b = c6_b_j;
    c6_e_c = 12 - c6_h_b;
    c6_n_a = c6_jj;
    c6_f_c = c6_n_a;
    c6_o_a = c6_jj;
    c6_g_c = c6_o_a;
    c6_m = c6_mmj;
    c6_n = c6_e_c;
    c6_d_ix0 = c6_jp1j;
    c6_d_iy0 = c6_f_c + 12;
    c6_ia0 = c6_g_c + 13;
    c6_b_m = c6_m;
    c6_b_n = c6_n;
    c6_e_ix0 = c6_d_ix0;
    c6_e_iy0 = c6_d_iy0;
    c6_b_ia0 = c6_ia0;
    c6_c_m = c6_b_m;
    c6_c_n = c6_b_n;
    c6_f_ix0 = c6_e_ix0;
    c6_f_iy0 = c6_e_iy0;
    c6_c_ia0 = c6_b_ia0;
    c6_d_m = c6_c_m;
    c6_d_n = c6_c_n;
    c6_g_ix0 = c6_f_ix0;
    c6_g_iy0 = c6_f_iy0;
    c6_d_ia0 = c6_c_ia0;
    c6_e_m = c6_d_m;
    c6_e_n = c6_d_n;
    c6_h_ix0 = c6_g_ix0;
    c6_h_iy0 = c6_g_iy0;
    c6_e_ia0 = c6_d_ia0;
    c6_ixstart = c6_h_ix0;
    c6_p_a = c6_e_ia0 - 1;
    c6_jA = c6_p_a;
    c6_jy = c6_h_iy0;
    c6_e_eml_int_forloop_overflow_check(chartInstance, 1, c6_e_n);
    c6_loop_ub = c6_e_n;
    for (c6_c_j = 1; c6_c_j <= c6_loop_ub; c6_c_j++) {
      c6_yjy = c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c6_jy), 1, 144, 1, 0) - 1];
      if (c6_yjy != 0.0) {
        c6_b_temp = -c6_yjy;
        c6_b_ix = c6_ixstart;
        c6_i_b = c6_jA + 1;
        c6_i257 = c6_i_b;
        c6_q_a = c6_e_m;
        c6_j_b = c6_jA;
        c6_i258 = c6_q_a + c6_j_b;
        c6_e_eml_int_forloop_overflow_check(chartInstance, c6_i257, c6_i258);
        for (c6_ijA = c6_i257; c6_ijA <= c6_i258; c6_ijA++) {
          c6_b_ijA = c6_ijA;
          c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_ijA), 1, 144, 1, 0) - 1] =
            c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_ijA), 1, 144, 1, 0) - 1] +
            c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_ix), 1, 144, 1, 0) - 1] * c6_b_temp;
          c6_r_a = c6_b_ix + 1;
          c6_b_ix = c6_r_a;
        }
      }

      c6_s_a = c6_jy + 12;
      c6_jy = c6_s_a;
      c6_t_a = c6_jA + 12;
      c6_jA = c6_t_a;
    }
  }

  if ((real_T)*c6_info == 0.0) {
    if (!(c6_A[143] != 0.0)) {
      *c6_info = 12;
    }
  }
}

static void c6_c_eml_xtrsm(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_A[144], real_T c6_B[144])
{
  int32_T c6_m;
  int32_T c6_n;
  real_T c6_alpha1;
  int32_T c6_lda;
  int32_T c6_ldb;
  char_T c6_SIDE;
  char_T c6_UPLO;
  char_T c6_TRANSA;
  char_T c6_DIAGA;
  c6_m = 12;
  c6_n = 12;
  c6_alpha1 = 1.0;
  c6_lda = 12;
  c6_ldb = 12;
  c6_SIDE = 'L';
  c6_UPLO = 'U';
  c6_TRANSA = 'N';
  c6_DIAGA = 'N';
  dtrsm32(&c6_SIDE, &c6_UPLO, &c6_TRANSA, &c6_DIAGA, &c6_m, &c6_n, &c6_alpha1,
          &c6_A[0], &c6_lda, &c6_B[0], &c6_ldb);
}

static void c6_c_eml_xgemm(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_A[72], real_T c6_B[144], real_T c6_C[72])
{
  int32_T c6_m;
  int32_T c6_n;
  int32_T c6_k;
  real_T c6_alpha1;
  int32_T c6_lda;
  int32_T c6_ldb;
  real_T c6_beta1;
  int32_T c6_ldc;
  char_T c6_TRANSA;
  char_T c6_TRANSB;
  c6_m = 6;
  c6_n = 12;
  c6_k = 12;
  c6_alpha1 = 1.0;
  c6_lda = 6;
  c6_ldb = 12;
  c6_beta1 = 0.0;
  c6_ldc = 6;
  c6_TRANSA = 'N';
  c6_TRANSB = 'N';
  dgemm32(&c6_TRANSA, &c6_TRANSB, &c6_m, &c6_n, &c6_k, &c6_alpha1, &c6_A[0],
          &c6_lda, &c6_B[0], &c6_ldb, &c6_beta1, &c6_C[0], &c6_ldc);
}

static void c6_d_eml_matlab_zgetrf(SFc6_kinematicsInstanceStruct *chartInstance,
  real_T c6_A[36], int32_T c6_ipiv[6], int32_T *c6_info)
{
  int32_T c6_i259;
  int32_T c6_j;
  int32_T c6_b_j;
  int32_T c6_b_a;
  int32_T c6_jm1;
  int32_T c6_b;
  int32_T c6_mmj;
  int32_T c6_c_a;
  int32_T c6_c;
  int32_T c6_b_b;
  int32_T c6_jj;
  int32_T c6_d_a;
  int32_T c6_jp1j;
  int32_T c6_e_a;
  int32_T c6_b_c;
  int32_T c6_i260;
  int32_T c6_i261;
  int32_T c6_i262;
  real_T c6_b_A[36];
  int32_T c6_f_a;
  int32_T c6_jpiv_offset;
  int32_T c6_g_a;
  int32_T c6_c_b;
  int32_T c6_jpiv;
  int32_T c6_h_a;
  int32_T c6_d_b;
  int32_T c6_c_c;
  int32_T c6_e_b;
  int32_T c6_jrow;
  int32_T c6_i_a;
  int32_T c6_f_b;
  int32_T c6_jprow;
  int32_T c6_ix0;
  int32_T c6_iy0;
  int32_T c6_b_ix0;
  int32_T c6_b_iy0;
  int32_T c6_c_ix0;
  int32_T c6_c_iy0;
  int32_T c6_ix;
  int32_T c6_iy;
  int32_T c6_k;
  real_T c6_temp;
  int32_T c6_j_a;
  int32_T c6_k_a;
  int32_T c6_b_jp1j;
  int32_T c6_l_a;
  int32_T c6_d_c;
  int32_T c6_m_a;
  int32_T c6_g_b;
  int32_T c6_i263;
  int32_T c6_i;
  int32_T c6_b_i;
  real_T c6_x;
  real_T c6_y;
  real_T c6_z;
  int32_T c6_h_b;
  int32_T c6_e_c;
  int32_T c6_n_a;
  int32_T c6_f_c;
  int32_T c6_o_a;
  int32_T c6_g_c;
  int32_T c6_m;
  int32_T c6_n;
  int32_T c6_d_ix0;
  int32_T c6_d_iy0;
  int32_T c6_ia0;
  int32_T c6_b_m;
  int32_T c6_b_n;
  int32_T c6_e_ix0;
  int32_T c6_e_iy0;
  int32_T c6_b_ia0;
  int32_T c6_c_m;
  int32_T c6_c_n;
  int32_T c6_f_ix0;
  int32_T c6_f_iy0;
  int32_T c6_c_ia0;
  int32_T c6_d_m;
  int32_T c6_d_n;
  int32_T c6_g_ix0;
  int32_T c6_g_iy0;
  int32_T c6_d_ia0;
  int32_T c6_e_m;
  int32_T c6_e_n;
  int32_T c6_h_ix0;
  int32_T c6_h_iy0;
  int32_T c6_e_ia0;
  int32_T c6_ixstart;
  int32_T c6_p_a;
  int32_T c6_jA;
  int32_T c6_jy;
  int32_T c6_loop_ub;
  int32_T c6_c_j;
  real_T c6_yjy;
  real_T c6_b_temp;
  int32_T c6_b_ix;
  int32_T c6_i_b;
  int32_T c6_i264;
  int32_T c6_q_a;
  int32_T c6_j_b;
  int32_T c6_i265;
  int32_T c6_ijA;
  int32_T c6_b_ijA;
  int32_T c6_r_a;
  int32_T c6_s_a;
  int32_T c6_t_a;
  c6_realmin(chartInstance);
  c6_eps(chartInstance);
  for (c6_i259 = 0; c6_i259 < 6; c6_i259++) {
    c6_ipiv[c6_i259] = 1 + c6_i259;
  }

  *c6_info = 0;
  c6_e_eml_int_forloop_overflow_check(chartInstance, 1, 5);
  for (c6_j = 1; c6_j < 6; c6_j++) {
    c6_b_j = c6_j;
    c6_b_a = c6_b_j - 1;
    c6_jm1 = c6_b_a;
    c6_b = c6_b_j;
    c6_mmj = 6 - c6_b;
    c6_c_a = c6_jm1;
    c6_c = c6_c_a * 7;
    c6_b_b = c6_c + 1;
    c6_jj = c6_b_b;
    c6_d_a = c6_jj + 1;
    c6_jp1j = c6_d_a;
    c6_e_a = c6_mmj;
    c6_b_c = c6_e_a;
    c6_i260 = 0;
    for (c6_i261 = 0; c6_i261 < 6; c6_i261++) {
      for (c6_i262 = 0; c6_i262 < 6; c6_i262++) {
        c6_b_A[c6_i262 + c6_i260] = c6_A[c6_i262 + c6_i260];
      }

      c6_i260 += 6;
    }

    c6_f_a = c6_b_eml_ixamax(chartInstance, c6_b_c + 1, c6_b_A, c6_jj) - 1;
    c6_jpiv_offset = c6_f_a;
    c6_g_a = c6_jj;
    c6_c_b = c6_jpiv_offset;
    c6_jpiv = c6_g_a + c6_c_b;
    if (c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_jpiv), 1, 36, 1, 0) - 1] != 0.0) {
      if ((real_T)c6_jpiv_offset != 0.0) {
        c6_h_a = c6_b_j;
        c6_d_b = c6_jpiv_offset;
        c6_c_c = c6_h_a + c6_d_b;
        c6_ipiv[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_b_j), 1, 6, 1, 0) - 1] = c6_c_c;
        c6_e_b = c6_jm1 + 1;
        c6_jrow = c6_e_b;
        c6_i_a = c6_jrow;
        c6_f_b = c6_jpiv_offset;
        c6_jprow = c6_i_a + c6_f_b;
        c6_ix0 = c6_jrow;
        c6_iy0 = c6_jprow;
        c6_b_ix0 = c6_ix0;
        c6_b_iy0 = c6_iy0;
        c6_c_ix0 = c6_b_ix0;
        c6_c_iy0 = c6_b_iy0;
        c6_ix = c6_c_ix0;
        c6_iy = c6_c_iy0;
        c6_e_eml_int_forloop_overflow_check(chartInstance, 1, 6);
        for (c6_k = 1; c6_k < 7; c6_k++) {
          c6_temp = c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c6_ix), 1, 36, 1, 0) - 1];
          c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_ix), 1, 36, 1, 0) - 1] = c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK
            ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c6_iy), 1, 36, 1, 0) -
            1];
          c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_iy), 1, 36, 1, 0) - 1] = c6_temp;
          c6_j_a = c6_ix + 6;
          c6_ix = c6_j_a;
          c6_k_a = c6_iy + 6;
          c6_iy = c6_k_a;
        }
      }

      c6_b_jp1j = c6_jp1j;
      c6_l_a = c6_mmj;
      c6_d_c = c6_l_a;
      c6_m_a = c6_jp1j;
      c6_g_b = c6_d_c - 1;
      c6_i263 = c6_m_a + c6_g_b;
      c6_e_eml_int_forloop_overflow_check(chartInstance, c6_b_jp1j, c6_i263);
      for (c6_i = c6_b_jp1j; c6_i <= c6_i263; c6_i++) {
        c6_b_i = c6_i;
        c6_x = c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c6_b_i), 1, 36, 1, 0) - 1];
        c6_y = c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c6_jj), 1, 36, 1, 0) - 1];
        c6_z = c6_x / c6_y;
        c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_b_i), 1, 36, 1, 0) - 1] = c6_z;
      }
    } else {
      *c6_info = c6_b_j;
    }

    c6_h_b = c6_b_j;
    c6_e_c = 6 - c6_h_b;
    c6_n_a = c6_jj;
    c6_f_c = c6_n_a;
    c6_o_a = c6_jj;
    c6_g_c = c6_o_a;
    c6_m = c6_mmj;
    c6_n = c6_e_c;
    c6_d_ix0 = c6_jp1j;
    c6_d_iy0 = c6_f_c + 6;
    c6_ia0 = c6_g_c + 7;
    c6_b_m = c6_m;
    c6_b_n = c6_n;
    c6_e_ix0 = c6_d_ix0;
    c6_e_iy0 = c6_d_iy0;
    c6_b_ia0 = c6_ia0;
    c6_c_m = c6_b_m;
    c6_c_n = c6_b_n;
    c6_f_ix0 = c6_e_ix0;
    c6_f_iy0 = c6_e_iy0;
    c6_c_ia0 = c6_b_ia0;
    c6_d_m = c6_c_m;
    c6_d_n = c6_c_n;
    c6_g_ix0 = c6_f_ix0;
    c6_g_iy0 = c6_f_iy0;
    c6_d_ia0 = c6_c_ia0;
    c6_e_m = c6_d_m;
    c6_e_n = c6_d_n;
    c6_h_ix0 = c6_g_ix0;
    c6_h_iy0 = c6_g_iy0;
    c6_e_ia0 = c6_d_ia0;
    c6_ixstart = c6_h_ix0;
    c6_p_a = c6_e_ia0 - 1;
    c6_jA = c6_p_a;
    c6_jy = c6_h_iy0;
    c6_e_eml_int_forloop_overflow_check(chartInstance, 1, c6_e_n);
    c6_loop_ub = c6_e_n;
    for (c6_c_j = 1; c6_c_j <= c6_loop_ub; c6_c_j++) {
      c6_yjy = c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c6_jy), 1, 36, 1, 0) - 1];
      if (c6_yjy != 0.0) {
        c6_b_temp = -c6_yjy;
        c6_b_ix = c6_ixstart;
        c6_i_b = c6_jA + 1;
        c6_i264 = c6_i_b;
        c6_q_a = c6_e_m;
        c6_j_b = c6_jA;
        c6_i265 = c6_q_a + c6_j_b;
        c6_e_eml_int_forloop_overflow_check(chartInstance, c6_i264, c6_i265);
        for (c6_ijA = c6_i264; c6_ijA <= c6_i265; c6_ijA++) {
          c6_b_ijA = c6_ijA;
          c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_ijA), 1, 36, 1, 0) - 1] =
            c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_ijA), 1, 36, 1, 0) - 1] +
            c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_b_ix), 1, 36, 1, 0) - 1] * c6_b_temp;
          c6_r_a = c6_b_ix + 1;
          c6_b_ix = c6_r_a;
        }
      }

      c6_s_a = c6_jy + 6;
      c6_jy = c6_s_a;
      c6_t_a = c6_jA + 6;
      c6_jA = c6_t_a;
    }
  }

  if ((real_T)*c6_info == 0.0) {
    if (!(c6_A[35] != 0.0)) {
      *c6_info = 6;
    }
  }
}

static void c6_d_eml_xtrsm(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_A[36], real_T c6_B[36])
{
  int32_T c6_j;
  int32_T c6_b_j;
  int32_T c6_b_a;
  int32_T c6_c;
  int32_T c6_b;
  int32_T c6_b_c;
  int32_T c6_b_b;
  int32_T c6_jBcol;
  int32_T c6_k;
  int32_T c6_b_k;
  int32_T c6_c_a;
  int32_T c6_c_c;
  int32_T c6_c_b;
  int32_T c6_d_c;
  int32_T c6_d_b;
  int32_T c6_kAcol;
  int32_T c6_d_a;
  int32_T c6_e_b;
  int32_T c6_e_c;
  int32_T c6_e_a;
  int32_T c6_f_b;
  int32_T c6_f_c;
  int32_T c6_f_a;
  int32_T c6_g_b;
  int32_T c6_g_c;
  int32_T c6_g_a;
  int32_T c6_h_b;
  int32_T c6_h_c;
  real_T c6_x;
  real_T c6_y;
  real_T c6_z;
  int32_T c6_h_a;
  int32_T c6_i266;
  int32_T c6_i;
  int32_T c6_b_i;
  int32_T c6_i_a;
  int32_T c6_i_b;
  int32_T c6_i_c;
  int32_T c6_j_a;
  int32_T c6_j_b;
  int32_T c6_j_c;
  int32_T c6_k_a;
  int32_T c6_k_b;
  int32_T c6_k_c;
  int32_T c6_l_a;
  int32_T c6_l_b;
  int32_T c6_l_c;
  c6_e_eml_int_forloop_overflow_check(chartInstance, 1, 6);
  for (c6_j = 1; c6_j < 7; c6_j++) {
    c6_b_j = c6_j;
    c6_b_a = c6_b_j;
    c6_c = c6_b_a;
    c6_b = c6_c - 1;
    c6_b_c = 6 * c6_b;
    c6_b_b = c6_b_c;
    c6_jBcol = c6_b_b;
    c6_f_eml_int_forloop_overflow_check(chartInstance);
    for (c6_k = 6; c6_k > 0; c6_k--) {
      c6_b_k = c6_k;
      c6_c_a = c6_b_k;
      c6_c_c = c6_c_a;
      c6_c_b = c6_c_c - 1;
      c6_d_c = 6 * c6_c_b;
      c6_d_b = c6_d_c;
      c6_kAcol = c6_d_b;
      c6_d_a = c6_b_k;
      c6_e_b = c6_jBcol;
      c6_e_c = c6_d_a + c6_e_b;
      if (c6_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_e_c), 1, 36, 1, 0) - 1] != 0.0) {
        c6_e_a = c6_b_k;
        c6_f_b = c6_jBcol;
        c6_f_c = c6_e_a + c6_f_b;
        c6_f_a = c6_b_k;
        c6_g_b = c6_jBcol;
        c6_g_c = c6_f_a + c6_g_b;
        c6_g_a = c6_b_k;
        c6_h_b = c6_kAcol;
        c6_h_c = c6_g_a + c6_h_b;
        c6_x = c6_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c6_g_c), 1, 36, 1, 0) - 1];
        c6_y = c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c6_h_c), 1, 36, 1, 0) - 1];
        c6_z = c6_x / c6_y;
        c6_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c6_f_c), 1, 36, 1, 0) - 1] = c6_z;
        c6_h_a = c6_b_k - 1;
        c6_i266 = c6_h_a;
        c6_e_eml_int_forloop_overflow_check(chartInstance, 1, c6_i266);
        for (c6_i = 1; c6_i <= c6_i266; c6_i++) {
          c6_b_i = c6_i;
          c6_i_a = c6_b_i;
          c6_i_b = c6_jBcol;
          c6_i_c = c6_i_a + c6_i_b;
          c6_j_a = c6_b_i;
          c6_j_b = c6_jBcol;
          c6_j_c = c6_j_a + c6_j_b;
          c6_k_a = c6_b_k;
          c6_k_b = c6_jBcol;
          c6_k_c = c6_k_a + c6_k_b;
          c6_l_a = c6_b_i;
          c6_l_b = c6_kAcol;
          c6_l_c = c6_l_a + c6_l_b;
          c6_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_i_c), 1, 36, 1, 0) - 1] =
            c6_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_j_c), 1, 36, 1, 0) - 1] -
            c6_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_k_c), 1, 36, 1, 0) - 1] *
            c6_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c6_l_c), 1, 36, 1, 0) - 1];
        }
      }
    }
  }
}

static void c6_d_eml_xgemm(SFc6_kinematicsInstanceStruct *chartInstance, real_T
  c6_A[144], real_T c6_B[72], real_T c6_C[72])
{
  int32_T c6_m;
  int32_T c6_n;
  int32_T c6_k;
  real_T c6_alpha1;
  int32_T c6_lda;
  int32_T c6_ldb;
  real_T c6_beta1;
  int32_T c6_ldc;
  char_T c6_TRANSA;
  char_T c6_TRANSB;
  c6_m = 12;
  c6_n = 6;
  c6_k = 12;
  c6_alpha1 = 1.0;
  c6_lda = 12;
  c6_ldb = 12;
  c6_beta1 = 0.0;
  c6_ldc = 12;
  c6_TRANSA = 'N';
  c6_TRANSB = 'N';
  dgemm32(&c6_TRANSA, &c6_TRANSB, &c6_m, &c6_n, &c6_k, &c6_alpha1, &c6_A[0],
          &c6_lda, &c6_B[0], &c6_ldb, &c6_beta1, &c6_C[0], &c6_ldc);
}

static void init_dsm_address_info(SFc6_kinematicsInstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c6_kinematics_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3844064264U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(747537945U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2191798392U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(683014552U);
}

mxArray *sf_c6_kinematics_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("M0xOffBX25tpa22KWmoM0D");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,7,3,dataFields);

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
      pr[0] = (double)(12);
      pr[1] = (double)(12);
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

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

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
      pr[0] = (double)(12);
      pr[1] = (double)(6);
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
      pr[0] = (double)(12);
      pr[1] = (double)(12);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

static const mxArray *sf_get_sim_state_info_c6_kinematics(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x5'type','srcId','name','auxInfo'{{M[1],M[16],T\"Jw_inv\",},{M[1],M[18],T\"dH\",},{M[1],M[17],T\"dH_trace\",},{M[1],M[5],T\"zeta\",},{M[8],M[0],T\"is_active_c6_kinematics\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 5, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c6_kinematics_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc6_kinematicsInstanceStruct *chartInstance;
    chartInstance = (SFc6_kinematicsInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_kinematicsMachineNumber_,
           6,
           1,
           1,
           16,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"J");
          _SFD_SET_DATA_PROPS(1,2,0,1,"zeta");
          _SFD_SET_DATA_PROPS(2,1,1,0,"vee");
          _SFD_SET_DATA_PROPS(3,1,1,0,"q");
          _SFD_SET_DATA_PROPS(4,1,1,0,"eta1");
          _SFD_SET_DATA_PROPS(5,1,1,0,"eta2");
          _SFD_SET_DATA_PROPS(6,1,1,0,"ee_eta1");
          _SFD_SET_DATA_PROPS(7,10,0,0,"qmin");
          _SFD_SET_DATA_PROPS(8,10,0,0,"qmax");
          _SFD_SET_DATA_PROPS(9,10,0,0,"d");
          _SFD_SET_DATA_PROPS(10,10,0,0,"a");
          _SFD_SET_DATA_PROPS(11,10,0,0,"alpha");
          _SFD_SET_DATA_PROPS(12,2,0,1,"Jw_inv");
          _SFD_SET_DATA_PROPS(13,2,0,1,"dH_trace");
          _SFD_SET_DATA_PROPS(14,2,0,1,"dH");
          _SFD_SET_DATA_PROPS(15,1,1,0,"previousDH");
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
        _SFD_CV_INIT_EML(0,1,2,1,0,0,4,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1970);
        _SFD_CV_INIT_EML_FCN(0,1,"generateDHMatrix",2020,-1,2109);
        _SFD_CV_INIT_EML_IF(0,1,0,1469,1500,1537,1583);
        _SFD_CV_INIT_EML_FOR(0,1,0,1162,1172,1200);
        _SFD_CV_INIT_EML_FOR(0,1,1,1261,1271,1299);
        _SFD_CV_INIT_EML_FOR(0,1,2,1332,1342,1608);
        _SFD_CV_INIT_EML_FOR(0,1,3,1914,1925,1961);
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
          unsigned int dimVector[2];
          dimVector[0]= 6;
          dimVector[1]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_f_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_b_sf_marshallOut,(MexInFcnForType)
            c6_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_d_sf_marshallOut,(MexInFcnForType)
            c6_d_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_d_sf_marshallOut,(MexInFcnForType)
            c6_d_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_d_sf_marshallOut,(MexInFcnForType)
            c6_d_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_d_sf_marshallOut,(MexInFcnForType)
            c6_d_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_d_sf_marshallOut,(MexInFcnForType)
            c6_d_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 12;
          dimVector[1]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(12,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_c_sf_marshallOut,(MexInFcnForType)
            c6_c_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(13,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_b_sf_marshallOut,(MexInFcnForType)
            c6_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 12;
          dimVector[1]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(14,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)
            c6_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 12;
          dimVector[1]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(15,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real_T (*c6_J)[72];
          real_T (*c6_zeta)[12];
          real_T (*c6_vee)[6];
          real_T (*c6_q)[6];
          real_T (*c6_eta1)[3];
          real_T (*c6_eta2)[3];
          real_T (*c6_ee_eta1)[3];
          real_T (*c6_Jw_inv)[72];
          real_T (*c6_dH_trace)[12];
          real_T (*c6_dH)[144];
          real_T (*c6_previousDH)[144];
          c6_previousDH = (real_T (*)[144])ssGetInputPortSignal(chartInstance->S,
            6);
          c6_dH = (real_T (*)[144])ssGetOutputPortSignal(chartInstance->S, 4);
          c6_dH_trace = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S,
            3);
          c6_Jw_inv = (real_T (*)[72])ssGetOutputPortSignal(chartInstance->S, 2);
          c6_ee_eta1 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 5);
          c6_eta2 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 4);
          c6_eta1 = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 3);
          c6_q = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 2);
          c6_vee = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 1);
          c6_zeta = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 1);
          c6_J = (real_T (*)[72])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c6_J);
          _SFD_SET_DATA_VALUE_PTR(1U, *c6_zeta);
          _SFD_SET_DATA_VALUE_PTR(2U, *c6_vee);
          _SFD_SET_DATA_VALUE_PTR(3U, *c6_q);
          _SFD_SET_DATA_VALUE_PTR(4U, *c6_eta1);
          _SFD_SET_DATA_VALUE_PTR(5U, *c6_eta2);
          _SFD_SET_DATA_VALUE_PTR(6U, *c6_ee_eta1);
          _SFD_SET_DATA_VALUE_PTR(7U, chartInstance->c6_qmin);
          _SFD_SET_DATA_VALUE_PTR(8U, chartInstance->c6_qmax);
          _SFD_SET_DATA_VALUE_PTR(9U, chartInstance->c6_d);
          _SFD_SET_DATA_VALUE_PTR(10U, chartInstance->c6_a);
          _SFD_SET_DATA_VALUE_PTR(11U, chartInstance->c6_alpha);
          _SFD_SET_DATA_VALUE_PTR(12U, *c6_Jw_inv);
          _SFD_SET_DATA_VALUE_PTR(13U, *c6_dH_trace);
          _SFD_SET_DATA_VALUE_PTR(14U, *c6_dH);
          _SFD_SET_DATA_VALUE_PTR(15U, *c6_previousDH);
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
  return "FVBLfkWFIV01gAJ2OAeHM";
}

static void sf_opaque_initialize_c6_kinematics(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc6_kinematicsInstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c6_kinematics((SFc6_kinematicsInstanceStruct*)
    chartInstanceVar);
  initialize_c6_kinematics((SFc6_kinematicsInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c6_kinematics(void *chartInstanceVar)
{
  enable_c6_kinematics((SFc6_kinematicsInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c6_kinematics(void *chartInstanceVar)
{
  disable_c6_kinematics((SFc6_kinematicsInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c6_kinematics(void *chartInstanceVar)
{
  sf_c6_kinematics((SFc6_kinematicsInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c6_kinematics(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c6_kinematics
    ((SFc6_kinematicsInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c6_kinematics();/* state var info */
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

extern void sf_internal_set_sim_state_c6_kinematics(SimStruct* S, const mxArray *
  st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c6_kinematics();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c6_kinematics((SFc6_kinematicsInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c6_kinematics(SimStruct* S)
{
  return sf_internal_get_sim_state_c6_kinematics(S);
}

static void sf_opaque_set_sim_state_c6_kinematics(SimStruct* S, const mxArray
  *st)
{
  sf_internal_set_sim_state_c6_kinematics(S, st);
}

static void sf_opaque_terminate_c6_kinematics(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc6_kinematicsInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c6_kinematics((SFc6_kinematicsInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_kinematics_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc6_kinematics((SFc6_kinematicsInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c6_kinematics(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c6_kinematics((SFc6_kinematicsInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c6_kinematics(SimStruct *S)
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
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 6, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,6,7);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,6,4);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,6);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1337621658U));
  ssSetChecksum1(S,(2776643509U));
  ssSetChecksum2(S,(2530355448U));
  ssSetChecksum3(S,(3634983122U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c6_kinematics(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c6_kinematics(SimStruct *S)
{
  SFc6_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc6_kinematicsInstanceStruct *)malloc(sizeof
    (SFc6_kinematicsInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc6_kinematicsInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c6_kinematics;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c6_kinematics;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c6_kinematics;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c6_kinematics;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c6_kinematics;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c6_kinematics;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c6_kinematics;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c6_kinematics;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c6_kinematics;
  chartInstance->chartInfo.mdlStart = mdlStart_c6_kinematics;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c6_kinematics;
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

void c6_kinematics_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c6_kinematics(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c6_kinematics(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c6_kinematics(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c6_kinematics_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
