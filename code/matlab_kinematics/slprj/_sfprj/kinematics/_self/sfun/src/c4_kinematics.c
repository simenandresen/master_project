/* Include files */

#include "blascompat32.h"
#include "kinematics_sfun.h"
#include "c4_kinematics.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "kinematics_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c4_debug_family_names[4] = { "nargin", "nargout", "J",
  "J_inv" };

/* Function Declarations */
static void initialize_c4_kinematics(SFc4_kinematicsInstanceStruct
  *chartInstance);
static void initialize_params_c4_kinematics(SFc4_kinematicsInstanceStruct
  *chartInstance);
static void enable_c4_kinematics(SFc4_kinematicsInstanceStruct *chartInstance);
static void disable_c4_kinematics(SFc4_kinematicsInstanceStruct *chartInstance);
static void c4_update_debugger_state_c4_kinematics(SFc4_kinematicsInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c4_kinematics(SFc4_kinematicsInstanceStruct *
  chartInstance);
static void set_sim_state_c4_kinematics(SFc4_kinematicsInstanceStruct
  *chartInstance, const mxArray *c4_st);
static void finalize_c4_kinematics(SFc4_kinematicsInstanceStruct *chartInstance);
static void sf_c4_kinematics(SFc4_kinematicsInstanceStruct *chartInstance);
static void initSimStructsc4_kinematics(SFc4_kinematicsInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber);
static const mxArray *c4_sf_marshallOut(void *chartInstanceVoid, void *c4_inData);
static void c4_emlrt_marshallIn(SFc4_kinematicsInstanceStruct *chartInstance,
  const mxArray *c4_J_inv, const char_T *c4_identifier, real_T c4_y[72]);
static void c4_b_emlrt_marshallIn(SFc4_kinematicsInstanceStruct *chartInstance,
  const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId, real_T c4_y[72]);
static void c4_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_b_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static const mxArray *c4_c_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static real_T c4_c_emlrt_marshallIn(SFc4_kinematicsInstanceStruct *chartInstance,
  const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static void c4_info_helper(c4_ResolvedFunctionInfo c4_info[149]);
static void c4_b_info_helper(c4_ResolvedFunctionInfo c4_info[149]);
static void c4_c_info_helper(c4_ResolvedFunctionInfo c4_info[149]);
static void c4_eml_scalar_eg(SFc4_kinematicsInstanceStruct *chartInstance);
static void c4_eml_int_forloop_overflow_check(SFc4_kinematicsInstanceStruct
  *chartInstance);
static void c4_eml_error(SFc4_kinematicsInstanceStruct *chartInstance);
static void c4_eml_xgesvd(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_A[72], real_T c4_U[72], real_T c4_S[6], real_T c4_V[36]);
static void c4_b_eml_int_forloop_overflow_check(SFc4_kinematicsInstanceStruct
  *chartInstance);
static real_T c4_eml_xnrm2(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_x[72], int32_T c4_ix0);
static real_T c4_abs(SFc4_kinematicsInstanceStruct *chartInstance, real_T c4_x);
static void c4_realmin(SFc4_kinematicsInstanceStruct *chartInstance);
static void c4_c_eml_int_forloop_overflow_check(SFc4_kinematicsInstanceStruct
  *chartInstance, int32_T c4_a, int32_T c4_b);
static real_T c4_eml_div(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_x, real_T c4_y);
static void c4_eml_xscal(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, real_T c4_x[72], int32_T c4_ix0, real_T c4_b_x[72]);
static real_T c4_eml_xdotc(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_x[72], int32_T c4_ix0, real_T c4_y[72], int32_T c4_iy0);
static void c4_eml_xaxpy(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, int32_T c4_ix0, real_T c4_y[72], int32_T c4_iy0, real_T
  c4_b_y[72]);
static real_T c4_b_eml_xnrm2(SFc4_kinematicsInstanceStruct *chartInstance,
  int32_T c4_n, real_T c4_x[6], int32_T c4_ix0);
static void c4_b_eml_xscal(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, real_T c4_x[6], int32_T c4_ix0, real_T c4_b_x[6]);
static void c4_b_eml_xaxpy(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, real_T c4_x[72], int32_T c4_ix0, real_T c4_y[12], int32_T
  c4_iy0, real_T c4_b_y[12]);
static void c4_c_eml_xaxpy(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, real_T c4_x[12], int32_T c4_ix0, real_T c4_y[72], int32_T
  c4_iy0, real_T c4_b_y[72]);
static void c4_d_eml_int_forloop_overflow_check(SFc4_kinematicsInstanceStruct
  *chartInstance);
static real_T c4_b_eml_xdotc(SFc4_kinematicsInstanceStruct *chartInstance,
  int32_T c4_n, real_T c4_x[36], int32_T c4_ix0, real_T c4_y[36], int32_T c4_iy0);
static void c4_d_eml_xaxpy(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, int32_T c4_ix0, real_T c4_y[36], int32_T c4_iy0, real_T
  c4_b_y[36]);
static void c4_b_eml_scalar_eg(SFc4_kinematicsInstanceStruct *chartInstance);
static void c4_c_eml_xscal(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_a, real_T c4_x[36], int32_T c4_ix0, real_T c4_b_x[36]);
static void c4_eps(SFc4_kinematicsInstanceStruct *chartInstance);
static void c4_c_eml_scalar_eg(SFc4_kinematicsInstanceStruct *chartInstance);
static void c4_b_eml_error(SFc4_kinematicsInstanceStruct *chartInstance);
static void c4_e_eml_int_forloop_overflow_check(SFc4_kinematicsInstanceStruct
  *chartInstance, int32_T c4_a);
static void c4_f_eml_int_forloop_overflow_check(SFc4_kinematicsInstanceStruct
  *chartInstance, int32_T c4_a, int32_T c4_b);
static real_T c4_sqrt(SFc4_kinematicsInstanceStruct *chartInstance, real_T c4_x);
static void c4_c_eml_error(SFc4_kinematicsInstanceStruct *chartInstance);
static void c4_eml_xrotg(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_a, real_T c4_b, real_T *c4_b_a, real_T *c4_b_b, real_T *c4_c, real_T *c4_s);
static void c4_eml_xrot(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_x[36], int32_T c4_ix0, int32_T c4_iy0, real_T c4_c, real_T c4_s, real_T
  c4_b_x[36]);
static void c4_b_eml_xrot(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_x[72], int32_T c4_ix0, int32_T c4_iy0, real_T c4_c, real_T c4_s, real_T
  c4_b_x[72]);
static void c4_eml_xswap(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_x[36], int32_T c4_ix0, int32_T c4_iy0, real_T c4_b_x[36]);
static void c4_b_eml_xswap(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_x[72], int32_T c4_ix0, int32_T c4_iy0, real_T c4_b_x[72]);
static void c4_g_eml_int_forloop_overflow_check(SFc4_kinematicsInstanceStruct
  *chartInstance);
static void c4_eml_xgemm(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_k, real_T c4_A[36], real_T c4_B[72], real_T c4_C[72]);
static void c4_h_eml_int_forloop_overflow_check(SFc4_kinematicsInstanceStruct
  *chartInstance, int32_T c4_a, int32_T c4_b);
static const mxArray *c4_d_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static int32_T c4_d_emlrt_marshallIn(SFc4_kinematicsInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static uint8_T c4_e_emlrt_marshallIn(SFc4_kinematicsInstanceStruct
  *chartInstance, const mxArray *c4_b_is_active_c4_kinematics, const char_T
  *c4_identifier);
static uint8_T c4_f_emlrt_marshallIn(SFc4_kinematicsInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_d_eml_xscal(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, real_T c4_x[72], int32_T c4_ix0);
static void c4_e_eml_xaxpy(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, int32_T c4_ix0, real_T c4_y[72], int32_T c4_iy0);
static void c4_e_eml_xscal(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, real_T c4_x[6], int32_T c4_ix0);
static void c4_f_eml_xaxpy(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, real_T c4_x[72], int32_T c4_ix0, real_T c4_y[12], int32_T
  c4_iy0);
static void c4_g_eml_xaxpy(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, real_T c4_x[12], int32_T c4_ix0, real_T c4_y[72], int32_T
  c4_iy0);
static void c4_h_eml_xaxpy(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, int32_T c4_ix0, real_T c4_y[36], int32_T c4_iy0);
static void c4_f_eml_xscal(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_a, real_T c4_x[36], int32_T c4_ix0);
static void c4_b_sqrt(SFc4_kinematicsInstanceStruct *chartInstance, real_T *c4_x);
static void c4_b_eml_xrotg(SFc4_kinematicsInstanceStruct *chartInstance, real_T *
  c4_a, real_T *c4_b, real_T *c4_c, real_T *c4_s);
static void c4_c_eml_xrot(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_x[36], int32_T c4_ix0, int32_T c4_iy0, real_T c4_c, real_T c4_s);
static void c4_d_eml_xrot(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_x[72], int32_T c4_ix0, int32_T c4_iy0, real_T c4_c, real_T c4_s);
static void c4_c_eml_xswap(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_x[36], int32_T c4_ix0, int32_T c4_iy0);
static void c4_d_eml_xswap(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_x[72], int32_T c4_ix0, int32_T c4_iy0);
static void init_dsm_address_info(SFc4_kinematicsInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c4_kinematics(SFc4_kinematicsInstanceStruct
  *chartInstance)
{
  chartInstance->c4_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c4_is_active_c4_kinematics = 0U;
}

static void initialize_params_c4_kinematics(SFc4_kinematicsInstanceStruct
  *chartInstance)
{
}

static void enable_c4_kinematics(SFc4_kinematicsInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c4_kinematics(SFc4_kinematicsInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c4_update_debugger_state_c4_kinematics(SFc4_kinematicsInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c4_kinematics(SFc4_kinematicsInstanceStruct *
  chartInstance)
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
  c4_hoistedGlobal = chartInstance->c4_is_active_c4_kinematics;
  c4_b_u = c4_hoistedGlobal;
  c4_c_y = NULL;
  sf_mex_assign(&c4_c_y, sf_mex_create("y", &c4_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c4_y, 1, c4_c_y);
  sf_mex_assign(&c4_st, c4_y, FALSE);
  return c4_st;
}

static void set_sim_state_c4_kinematics(SFc4_kinematicsInstanceStruct
  *chartInstance, const mxArray *c4_st)
{
  const mxArray *c4_u;
  real_T c4_dv0[72];
  int32_T c4_i1;
  real_T (*c4_J_inv)[72];
  c4_J_inv = (real_T (*)[72])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c4_doneDoubleBufferReInit = TRUE;
  c4_u = sf_mex_dup(c4_st);
  c4_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 0)),
                      "J_inv", c4_dv0);
  for (c4_i1 = 0; c4_i1 < 72; c4_i1++) {
    (*c4_J_inv)[c4_i1] = c4_dv0[c4_i1];
  }

  chartInstance->c4_is_active_c4_kinematics = c4_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 1)),
     "is_active_c4_kinematics");
  sf_mex_destroy(&c4_u);
  c4_update_debugger_state_c4_kinematics(chartInstance);
  sf_mex_destroy(&c4_st);
}

static void finalize_c4_kinematics(SFc4_kinematicsInstanceStruct *chartInstance)
{
}

static void sf_c4_kinematics(SFc4_kinematicsInstanceStruct *chartInstance)
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
  real_T c4_A[72];
  int32_T c4_i7;
  int32_T c4_i8;
  int32_T c4_i9;
  int32_T c4_i10;
  real_T c4_U[72];
  int32_T c4_i11;
  int32_T c4_k;
  int32_T c4_b_k;
  real_T c4_x;
  real_T c4_b_x;
  boolean_T c4_b;
  boolean_T c4_b0;
  real_T c4_c_x;
  boolean_T c4_b_b;
  boolean_T c4_b1;
  boolean_T c4_c_b;
  int32_T c4_i12;
  real_T c4_b_U[72];
  real_T c4_V[36];
  real_T c4_s[6];
  int32_T c4_i13;
  real_T c4_S[36];
  int32_T c4_c_k;
  real_T c4_d_k;
  real_T c4_d_b;
  real_T c4_y;
  real_T c4_a;
  real_T c4_tol;
  int32_T c4_r;
  int32_T c4_e_k;
  int32_T c4_f_k;
  int32_T c4_b_a;
  int32_T c4_vcol;
  int32_T c4_j;
  int32_T c4_b_j;
  real_T c4_b_y;
  real_T c4_z;
  int32_T c4_c_a;
  int32_T c4_i14;
  real_T c4_b_V[36];
  int32_T c4_i15;
  real_T c4_c_U[72];
  int32_T c4_i16;
  int32_T c4_i17;
  int32_T c4_i18;
  int32_T c4_i19;
  int32_T c4_i20;
  real_T (*c4_b_J_inv)[72];
  real_T (*c4_b_J)[72];
  boolean_T exitg1;
  c4_b_J_inv = (real_T (*)[72])ssGetOutputPortSignal(chartInstance->S, 1);
  c4_b_J = (real_T (*)[72])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 2U, chartInstance->c4_sfEvent);
  for (c4_i2 = 0; c4_i2 < 72; c4_i2++) {
    _SFD_DATA_RANGE_CHECK((*c4_b_J)[c4_i2], 0U);
  }

  for (c4_i3 = 0; c4_i3 < 72; c4_i3++) {
    _SFD_DATA_RANGE_CHECK((*c4_b_J_inv)[c4_i3], 1U);
  }

  chartInstance->c4_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 2U, chartInstance->c4_sfEvent);
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
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 4);
  for (c4_i5 = 0; c4_i5 < 72; c4_i5++) {
    c4_J_inv[c4_i5] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 5);
  for (c4_i6 = 0; c4_i6 < 72; c4_i6++) {
    c4_A[c4_i6] = c4_J[c4_i6];
  }

  c4_i7 = 0;
  for (c4_i8 = 0; c4_i8 < 6; c4_i8++) {
    c4_i9 = 0;
    for (c4_i10 = 0; c4_i10 < 12; c4_i10++) {
      c4_U[c4_i10 + c4_i7] = c4_A[c4_i9 + c4_i8];
      c4_i9 += 6;
    }

    c4_i7 += 12;
  }

  for (c4_i11 = 0; c4_i11 < 72; c4_i11++) {
    c4_A[c4_i11] = 0.0;
  }

  c4_eml_int_forloop_overflow_check(chartInstance);
  for (c4_k = 1; c4_k < 73; c4_k++) {
    c4_b_k = c4_k;
    c4_x = c4_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c4_b_k), 1, 72, 1, 0) - 1];
    c4_b_x = c4_x;
    c4_b = muDoubleScalarIsInf(c4_b_x);
    c4_b0 = !c4_b;
    c4_c_x = c4_x;
    c4_b_b = muDoubleScalarIsNaN(c4_c_x);
    c4_b1 = !c4_b_b;
    c4_c_b = (c4_b0 && c4_b1);
    if (!c4_c_b) {
      c4_eml_error(chartInstance);
    }
  }

  for (c4_i12 = 0; c4_i12 < 72; c4_i12++) {
    c4_b_U[c4_i12] = c4_U[c4_i12];
  }

  c4_eml_xgesvd(chartInstance, c4_b_U, c4_U, c4_s, c4_V);
  for (c4_i13 = 0; c4_i13 < 36; c4_i13++) {
    c4_S[c4_i13] = 0.0;
  }

  for (c4_c_k = 0; c4_c_k < 6; c4_c_k++) {
    c4_d_k = 1.0 + (real_T)c4_c_k;
    c4_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c4_d_k),
           1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", c4_d_k), 1, 6, 2, 0) - 1)) - 1] =
      c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c4_d_k), 1, 6, 1, 0) - 1];
  }

  c4_eps(chartInstance);
  c4_d_b = c4_S[0];
  c4_y = 12.0 * c4_d_b;
  c4_a = c4_y;
  c4_tol = c4_a * 2.2204460492503131E-16;
  c4_r = 0;
  c4_b_eml_int_forloop_overflow_check(chartInstance);
  c4_e_k = 1;
  exitg1 = FALSE;
  while ((exitg1 == 0U) && (c4_e_k < 7)) {
    c4_f_k = c4_e_k;
    if (!(c4_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_f_k), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
            (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_f_k), 1, 6, 2, 0) - 1)) -
          1] > c4_tol)) {
      exitg1 = TRUE;
    } else {
      c4_b_a = c4_r + 1;
      c4_r = c4_b_a;
      c4_e_k++;
    }
  }

  if ((real_T)c4_r > 0.0) {
    c4_vcol = 1;
    c4_c_eml_int_forloop_overflow_check(chartInstance, 1, c4_r);
    for (c4_j = 1; c4_j <= c4_r; c4_j++) {
      c4_b_j = c4_j;
      c4_b_y = c4_S[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
                      ("", (real_T)c4_b_j), 1, 6, 1, 0) + 6 *
                     (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c4_b_j), 1, 6, 2, 0) - 1)) - 1];
      c4_z = 1.0 / c4_b_y;
      c4_f_eml_xscal(chartInstance, c4_z, c4_V, c4_vcol);
      c4_c_a = c4_vcol + 6;
      c4_vcol = c4_c_a;
    }

    for (c4_i14 = 0; c4_i14 < 36; c4_i14++) {
      c4_b_V[c4_i14] = c4_V[c4_i14];
    }

    for (c4_i15 = 0; c4_i15 < 72; c4_i15++) {
      c4_c_U[c4_i15] = c4_U[c4_i15];
    }

    c4_eml_xgemm(chartInstance, c4_r, c4_b_V, c4_c_U, c4_A);
  }

  c4_i16 = 0;
  for (c4_i17 = 0; c4_i17 < 6; c4_i17++) {
    c4_i18 = 0;
    for (c4_i19 = 0; c4_i19 < 12; c4_i19++) {
      c4_J_inv[c4_i19 + c4_i16] = c4_A[c4_i18 + c4_i17];
      c4_i18 += 6;
    }

    c4_i16 += 12;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, -5);
  sf_debug_symbol_scope_pop();
  for (c4_i20 = 0; c4_i20 < 72; c4_i20++) {
    (*c4_b_J_inv)[c4_i20] = c4_J_inv[c4_i20];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c4_sfEvent);
  sf_debug_check_for_state_inconsistency(_kinematicsMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc4_kinematics(SFc4_kinematicsInstanceStruct
  *chartInstance)
{
}

static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber)
{
}

static const mxArray *c4_sf_marshallOut(void *chartInstanceVoid, void *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i21;
  int32_T c4_i22;
  int32_T c4_i23;
  real_T c4_b_inData[72];
  int32_T c4_i24;
  int32_T c4_i25;
  int32_T c4_i26;
  real_T c4_u[72];
  const mxArray *c4_y = NULL;
  SFc4_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc4_kinematicsInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_i21 = 0;
  for (c4_i22 = 0; c4_i22 < 6; c4_i22++) {
    for (c4_i23 = 0; c4_i23 < 12; c4_i23++) {
      c4_b_inData[c4_i23 + c4_i21] = (*(real_T (*)[72])c4_inData)[c4_i23 +
        c4_i21];
    }

    c4_i21 += 12;
  }

  c4_i24 = 0;
  for (c4_i25 = 0; c4_i25 < 6; c4_i25++) {
    for (c4_i26 = 0; c4_i26 < 12; c4_i26++) {
      c4_u[c4_i26 + c4_i24] = c4_b_inData[c4_i26 + c4_i24];
    }

    c4_i24 += 12;
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 12, 6), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static void c4_emlrt_marshallIn(SFc4_kinematicsInstanceStruct *chartInstance,
  const mxArray *c4_J_inv, const char_T *c4_identifier, real_T c4_y[72])
{
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_J_inv), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_J_inv);
}

static void c4_b_emlrt_marshallIn(SFc4_kinematicsInstanceStruct *chartInstance,
  const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId, real_T c4_y[72])
{
  real_T c4_dv1[72];
  int32_T c4_i27;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv1, 1, 0, 0U, 1, 0U, 2, 12, 6);
  for (c4_i27 = 0; c4_i27 < 72; c4_i27++) {
    c4_y[c4_i27] = c4_dv1[c4_i27];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_J_inv;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_y[72];
  int32_T c4_i28;
  int32_T c4_i29;
  int32_T c4_i30;
  SFc4_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc4_kinematicsInstanceStruct *)chartInstanceVoid;
  c4_J_inv = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_J_inv), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_J_inv);
  c4_i28 = 0;
  for (c4_i29 = 0; c4_i29 < 6; c4_i29++) {
    for (c4_i30 = 0; c4_i30 < 12; c4_i30++) {
      (*(real_T (*)[72])c4_outData)[c4_i30 + c4_i28] = c4_y[c4_i30 + c4_i28];
    }

    c4_i28 += 12;
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_b_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i31;
  int32_T c4_i32;
  int32_T c4_i33;
  real_T c4_b_inData[72];
  int32_T c4_i34;
  int32_T c4_i35;
  int32_T c4_i36;
  real_T c4_u[72];
  const mxArray *c4_y = NULL;
  SFc4_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc4_kinematicsInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_i31 = 0;
  for (c4_i32 = 0; c4_i32 < 12; c4_i32++) {
    for (c4_i33 = 0; c4_i33 < 6; c4_i33++) {
      c4_b_inData[c4_i33 + c4_i31] = (*(real_T (*)[72])c4_inData)[c4_i33 +
        c4_i31];
    }

    c4_i31 += 6;
  }

  c4_i34 = 0;
  for (c4_i35 = 0; c4_i35 < 12; c4_i35++) {
    for (c4_i36 = 0; c4_i36 < 6; c4_i36++) {
      c4_u[c4_i36 + c4_i34] = c4_b_inData[c4_i36 + c4_i34];
    }

    c4_i34 += 6;
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
  SFc4_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc4_kinematicsInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(real_T *)c4_inData;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static real_T c4_c_emlrt_marshallIn(SFc4_kinematicsInstanceStruct *chartInstance,
  const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
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
  SFc4_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc4_kinematicsInstanceStruct *)chartInstanceVoid;
  c4_nargout = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_nargout), &c4_thisId);
  sf_mex_destroy(&c4_nargout);
  *(real_T *)c4_outData = c4_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

const mxArray *sf_c4_kinematics_get_eml_resolved_functions_info(void)
{
  const mxArray *c4_nameCaptureInfo;
  c4_ResolvedFunctionInfo c4_info[149];
  const mxArray *c4_m0 = NULL;
  int32_T c4_i37;
  c4_ResolvedFunctionInfo *c4_r0;
  c4_nameCaptureInfo = NULL;
  c4_nameCaptureInfo = NULL;
  c4_info_helper(c4_info);
  c4_b_info_helper(c4_info);
  c4_c_info_helper(c4_info);
  sf_mex_assign(&c4_m0, sf_mex_createstruct("nameCaptureInfo", 1, 149), FALSE);
  for (c4_i37 = 0; c4_i37 < 149; c4_i37++) {
    c4_r0 = &c4_info[c4_i37];
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c4_r0->context)), "context", "nameCaptureInfo",
                    c4_i37);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->name, 15, 0U,
      0U, 0U, 2, 1, strlen(c4_r0->name)), "name", "nameCaptureInfo", c4_i37);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->dominantType,
      15, 0U, 0U, 0U, 2, 1, strlen(c4_r0->dominantType)), "dominantType",
                    "nameCaptureInfo", c4_i37);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", c4_r0->resolved, 15,
      0U, 0U, 0U, 2, 1, strlen(c4_r0->resolved)), "resolved", "nameCaptureInfo",
                    c4_i37);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c4_i37);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c4_i37);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->mFileTimeLo,
      7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo", c4_i37);
    sf_mex_addfield(c4_m0, sf_mex_create("nameCaptureInfo", &c4_r0->mFileTimeHi,
      7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo", c4_i37);
  }

  sf_mex_assign(&c4_nameCaptureInfo, c4_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c4_nameCaptureInfo);
  return c4_nameCaptureInfo;
}

static void c4_info_helper(c4_ResolvedFunctionInfo c4_info[149])
{
  c4_info[0].context = "";
  c4_info[0].name = "pinv";
  c4_info[0].dominantType = "double";
  c4_info[0].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/pinv.m";
  c4_info[0].fileTimeLo = 1286818828U;
  c4_info[0].fileTimeHi = 0U;
  c4_info[0].mFileTimeLo = 0U;
  c4_info[0].mFileTimeHi = 0U;
  c4_info[1].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c4_info[1].name = "eml_index_class";
  c4_info[1].dominantType = "";
  c4_info[1].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[1].fileTimeLo = 1286818778U;
  c4_info[1].fileTimeHi = 0U;
  c4_info[1].mFileTimeLo = 0U;
  c4_info[1].mFileTimeHi = 0U;
  c4_info[2].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c4_info[2].name = "eml_scalar_eg";
  c4_info[2].dominantType = "double";
  c4_info[2].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[2].fileTimeLo = 1286818796U;
  c4_info[2].fileTimeHi = 0U;
  c4_info[2].mFileTimeLo = 0U;
  c4_info[2].mFileTimeHi = 0U;
  c4_info[3].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c4_info[3].name = "svd";
  c4_info[3].dominantType = "double";
  c4_info[3].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/svd.m";
  c4_info[3].fileTimeLo = 1286818832U;
  c4_info[3].fileTimeHi = 0U;
  c4_info[3].mFileTimeLo = 0U;
  c4_info[3].mFileTimeHi = 0U;
  c4_info[4].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/svd.m";
  c4_info[4].name = "eml_index_class";
  c4_info[4].dominantType = "";
  c4_info[4].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[4].fileTimeLo = 1286818778U;
  c4_info[4].fileTimeHi = 0U;
  c4_info[4].mFileTimeLo = 0U;
  c4_info[4].mFileTimeHi = 0U;
  c4_info[5].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/svd.m";
  c4_info[5].name = "eml_int_forloop_overflow_check";
  c4_info[5].dominantType = "";
  c4_info[5].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c4_info[5].fileTimeLo = 1311255316U;
  c4_info[5].fileTimeHi = 0U;
  c4_info[5].mFileTimeLo = 0U;
  c4_info[5].mFileTimeHi = 0U;
  c4_info[6].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c4_info[6].name = "intmax";
  c4_info[6].dominantType = "char";
  c4_info[6].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/intmax.m";
  c4_info[6].fileTimeLo = 1311255316U;
  c4_info[6].fileTimeHi = 0U;
  c4_info[6].mFileTimeLo = 0U;
  c4_info[6].mFileTimeHi = 0U;
  c4_info[7].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/svd.m";
  c4_info[7].name = "isfinite";
  c4_info[7].dominantType = "double";
  c4_info[7].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c4_info[7].fileTimeLo = 1286818758U;
  c4_info[7].fileTimeHi = 0U;
  c4_info[7].mFileTimeLo = 0U;
  c4_info[7].mFileTimeHi = 0U;
  c4_info[8].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c4_info[8].name = "isinf";
  c4_info[8].dominantType = "double";
  c4_info[8].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isinf.m";
  c4_info[8].fileTimeLo = 1286818760U;
  c4_info[8].fileTimeHi = 0U;
  c4_info[8].mFileTimeLo = 0U;
  c4_info[8].mFileTimeHi = 0U;
  c4_info[9].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isfinite.m";
  c4_info[9].name = "isnan";
  c4_info[9].dominantType = "double";
  c4_info[9].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isnan.m";
  c4_info[9].fileTimeLo = 1286818760U;
  c4_info[9].fileTimeHi = 0U;
  c4_info[9].mFileTimeLo = 0U;
  c4_info[9].mFileTimeHi = 0U;
  c4_info[10].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/svd.m";
  c4_info[10].name = "eml_error";
  c4_info[10].dominantType = "char";
  c4_info[10].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_error.m";
  c4_info[10].fileTimeLo = 1305318000U;
  c4_info[10].fileTimeHi = 0U;
  c4_info[10].mFileTimeLo = 0U;
  c4_info[10].mFileTimeHi = 0U;
  c4_info[11].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/svd.m";
  c4_info[11].name = "eml_xgesvd";
  c4_info[11].dominantType = "double";
  c4_info[11].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/eml_xgesvd.m";
  c4_info[11].fileTimeLo = 1286818806U;
  c4_info[11].fileTimeHi = 0U;
  c4_info[11].mFileTimeLo = 0U;
  c4_info[11].mFileTimeHi = 0U;
  c4_info[12].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/eml_xgesvd.m";
  c4_info[12].name = "eml_lapack_xgesvd";
  c4_info[12].dominantType = "double";
  c4_info[12].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgesvd.m";
  c4_info[12].fileTimeLo = 1286818810U;
  c4_info[12].fileTimeHi = 0U;
  c4_info[12].mFileTimeLo = 0U;
  c4_info[12].mFileTimeHi = 0U;
  c4_info[13].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgesvd.m";
  c4_info[13].name = "eml_matlab_zsvdc";
  c4_info[13].dominantType = "double";
  c4_info[13].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[13].fileTimeLo = 1295284866U;
  c4_info[13].fileTimeHi = 0U;
  c4_info[13].mFileTimeLo = 0U;
  c4_info[13].mFileTimeHi = 0U;
  c4_info[14].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[14].name = "eml_index_class";
  c4_info[14].dominantType = "";
  c4_info[14].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[14].fileTimeLo = 1286818778U;
  c4_info[14].fileTimeHi = 0U;
  c4_info[14].mFileTimeLo = 0U;
  c4_info[14].mFileTimeHi = 0U;
  c4_info[15].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[15].name = "eml_scalar_eg";
  c4_info[15].dominantType = "double";
  c4_info[15].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[15].fileTimeLo = 1286818796U;
  c4_info[15].fileTimeHi = 0U;
  c4_info[15].mFileTimeLo = 0U;
  c4_info[15].mFileTimeHi = 0U;
  c4_info[16].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[16].name = "eml_index_plus";
  c4_info[16].dominantType = "int32";
  c4_info[16].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[16].fileTimeLo = 1286818778U;
  c4_info[16].fileTimeHi = 0U;
  c4_info[16].mFileTimeLo = 0U;
  c4_info[16].mFileTimeHi = 0U;
  c4_info[17].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[17].name = "eml_index_class";
  c4_info[17].dominantType = "";
  c4_info[17].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[17].fileTimeLo = 1286818778U;
  c4_info[17].fileTimeHi = 0U;
  c4_info[17].mFileTimeLo = 0U;
  c4_info[17].mFileTimeHi = 0U;
  c4_info[18].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[18].name = "min";
  c4_info[18].dominantType = "int32";
  c4_info[18].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/min.m";
  c4_info[18].fileTimeLo = 1311255318U;
  c4_info[18].fileTimeHi = 0U;
  c4_info[18].mFileTimeLo = 0U;
  c4_info[18].mFileTimeHi = 0U;
  c4_info[19].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/min.m";
  c4_info[19].name = "eml_min_or_max";
  c4_info[19].dominantType = "int32";
  c4_info[19].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c4_info[19].fileTimeLo = 1303146212U;
  c4_info[19].fileTimeHi = 0U;
  c4_info[19].mFileTimeLo = 0U;
  c4_info[19].mFileTimeHi = 0U;
  c4_info[20].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c4_info[20].name = "eml_scalar_eg";
  c4_info[20].dominantType = "int32";
  c4_info[20].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[20].fileTimeLo = 1286818796U;
  c4_info[20].fileTimeHi = 0U;
  c4_info[20].mFileTimeLo = 0U;
  c4_info[20].mFileTimeHi = 0U;
  c4_info[21].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c4_info[21].name = "eml_scalexp_alloc";
  c4_info[21].dominantType = "int32";
  c4_info[21].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c4_info[21].fileTimeLo = 1286818796U;
  c4_info[21].fileTimeHi = 0U;
  c4_info[21].mFileTimeLo = 0U;
  c4_info[21].mFileTimeHi = 0U;
  c4_info[22].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c4_info[22].name = "eml_index_class";
  c4_info[22].dominantType = "";
  c4_info[22].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[22].fileTimeLo = 1286818778U;
  c4_info[22].fileTimeHi = 0U;
  c4_info[22].mFileTimeLo = 0U;
  c4_info[22].mFileTimeHi = 0U;
  c4_info[23].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c4_info[23].name = "eml_scalar_eg";
  c4_info[23].dominantType = "int32";
  c4_info[23].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[23].fileTimeLo = 1286818796U;
  c4_info[23].fileTimeHi = 0U;
  c4_info[23].mFileTimeLo = 0U;
  c4_info[23].mFileTimeHi = 0U;
  c4_info[24].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[24].name = "max";
  c4_info[24].dominantType = "int32";
  c4_info[24].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/max.m";
  c4_info[24].fileTimeLo = 1311255316U;
  c4_info[24].fileTimeHi = 0U;
  c4_info[24].mFileTimeLo = 0U;
  c4_info[24].mFileTimeHi = 0U;
  c4_info[25].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/max.m";
  c4_info[25].name = "eml_min_or_max";
  c4_info[25].dominantType = "int32";
  c4_info[25].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c4_info[25].fileTimeLo = 1303146212U;
  c4_info[25].fileTimeHi = 0U;
  c4_info[25].mFileTimeLo = 0U;
  c4_info[25].mFileTimeHi = 0U;
  c4_info[26].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c4_info[26].name = "eml_relop";
  c4_info[26].dominantType = "function_handle";
  c4_info[26].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_relop.m";
  c4_info[26].fileTimeLo = 1292190510U;
  c4_info[26].fileTimeHi = 0U;
  c4_info[26].mFileTimeLo = 0U;
  c4_info[26].mFileTimeHi = 0U;
  c4_info[27].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c4_info[27].name = "isnan";
  c4_info[27].dominantType = "int32";
  c4_info[27].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isnan.m";
  c4_info[27].fileTimeLo = 1286818760U;
  c4_info[27].fileTimeHi = 0U;
  c4_info[27].mFileTimeLo = 0U;
  c4_info[27].mFileTimeHi = 0U;
  c4_info[28].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[28].name = "eml_index_minus";
  c4_info[28].dominantType = "int32";
  c4_info[28].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c4_info[28].fileTimeLo = 1286818778U;
  c4_info[28].fileTimeHi = 0U;
  c4_info[28].mFileTimeLo = 0U;
  c4_info[28].mFileTimeHi = 0U;
  c4_info[29].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c4_info[29].name = "eml_index_class";
  c4_info[29].dominantType = "";
  c4_info[29].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[29].fileTimeLo = 1286818778U;
  c4_info[29].fileTimeHi = 0U;
  c4_info[29].mFileTimeLo = 0U;
  c4_info[29].mFileTimeHi = 0U;
  c4_info[30].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[30].name = "eml_int_forloop_overflow_check";
  c4_info[30].dominantType = "";
  c4_info[30].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c4_info[30].fileTimeLo = 1311255316U;
  c4_info[30].fileTimeHi = 0U;
  c4_info[30].mFileTimeLo = 0U;
  c4_info[30].mFileTimeHi = 0U;
  c4_info[31].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[31].name = "eml_index_times";
  c4_info[31].dominantType = "int32";
  c4_info[31].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c4_info[31].fileTimeLo = 1286818780U;
  c4_info[31].fileTimeHi = 0U;
  c4_info[31].mFileTimeLo = 0U;
  c4_info[31].mFileTimeHi = 0U;
  c4_info[32].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c4_info[32].name = "eml_index_class";
  c4_info[32].dominantType = "";
  c4_info[32].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[32].fileTimeLo = 1286818778U;
  c4_info[32].fileTimeHi = 0U;
  c4_info[32].mFileTimeLo = 0U;
  c4_info[32].mFileTimeHi = 0U;
  c4_info[33].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[33].name = "eml_xnrm2";
  c4_info[33].dominantType = "int32";
  c4_info[33].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c4_info[33].fileTimeLo = 1299076776U;
  c4_info[33].fileTimeHi = 0U;
  c4_info[33].mFileTimeLo = 0U;
  c4_info[33].mFileTimeHi = 0U;
  c4_info[34].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c4_info[34].name = "eml_blas_inline";
  c4_info[34].dominantType = "";
  c4_info[34].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c4_info[34].fileTimeLo = 1299076768U;
  c4_info[34].fileTimeHi = 0U;
  c4_info[34].mFileTimeLo = 0U;
  c4_info[34].mFileTimeHi = 0U;
  c4_info[35].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m!below_threshold";
  c4_info[35].name = "length";
  c4_info[35].dominantType = "double";
  c4_info[35].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/length.m";
  c4_info[35].fileTimeLo = 1303146206U;
  c4_info[35].fileTimeHi = 0U;
  c4_info[35].mFileTimeLo = 0U;
  c4_info[35].mFileTimeHi = 0U;
  c4_info[36].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/length.m!intlength";
  c4_info[36].name = "eml_index_class";
  c4_info[36].dominantType = "";
  c4_info[36].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[36].fileTimeLo = 1286818778U;
  c4_info[36].fileTimeHi = 0U;
  c4_info[36].mFileTimeLo = 0U;
  c4_info[36].mFileTimeHi = 0U;
  c4_info[37].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m";
  c4_info[37].name = "eml_refblas_xnrm2";
  c4_info[37].dominantType = "int32";
  c4_info[37].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c4_info[37].fileTimeLo = 1299076784U;
  c4_info[37].fileTimeHi = 0U;
  c4_info[37].mFileTimeLo = 0U;
  c4_info[37].mFileTimeHi = 0U;
  c4_info[38].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c4_info[38].name = "abs";
  c4_info[38].dominantType = "double";
  c4_info[38].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c4_info[38].fileTimeLo = 1286818694U;
  c4_info[38].fileTimeHi = 0U;
  c4_info[38].mFileTimeLo = 0U;
  c4_info[38].mFileTimeHi = 0U;
  c4_info[39].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c4_info[39].name = "eml_scalar_abs";
  c4_info[39].dominantType = "double";
  c4_info[39].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c4_info[39].fileTimeLo = 1286818712U;
  c4_info[39].fileTimeHi = 0U;
  c4_info[39].mFileTimeLo = 0U;
  c4_info[39].mFileTimeHi = 0U;
  c4_info[40].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c4_info[40].name = "realmin";
  c4_info[40].dominantType = "char";
  c4_info[40].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/realmin.m";
  c4_info[40].fileTimeLo = 1307651242U;
  c4_info[40].fileTimeHi = 0U;
  c4_info[40].mFileTimeLo = 0U;
  c4_info[40].mFileTimeHi = 0U;
  c4_info[41].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/realmin.m";
  c4_info[41].name = "eml_realmin";
  c4_info[41].dominantType = "char";
  c4_info[41].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c4_info[41].fileTimeLo = 1307651244U;
  c4_info[41].fileTimeHi = 0U;
  c4_info[41].mFileTimeLo = 0U;
  c4_info[41].mFileTimeHi = 0U;
  c4_info[42].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c4_info[42].name = "eml_float_model";
  c4_info[42].dominantType = "char";
  c4_info[42].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c4_info[42].fileTimeLo = 1307651242U;
  c4_info[42].fileTimeHi = 0U;
  c4_info[42].mFileTimeLo = 0U;
  c4_info[42].mFileTimeHi = 0U;
  c4_info[43].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c4_info[43].name = "eml_index_class";
  c4_info[43].dominantType = "";
  c4_info[43].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[43].fileTimeLo = 1286818778U;
  c4_info[43].fileTimeHi = 0U;
  c4_info[43].mFileTimeLo = 0U;
  c4_info[43].mFileTimeHi = 0U;
  c4_info[44].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c4_info[44].name = "eml_index_minus";
  c4_info[44].dominantType = "int32";
  c4_info[44].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c4_info[44].fileTimeLo = 1286818778U;
  c4_info[44].fileTimeHi = 0U;
  c4_info[44].mFileTimeLo = 0U;
  c4_info[44].mFileTimeHi = 0U;
  c4_info[45].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c4_info[45].name = "eml_index_times";
  c4_info[45].dominantType = "int32";
  c4_info[45].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c4_info[45].fileTimeLo = 1286818780U;
  c4_info[45].fileTimeHi = 0U;
  c4_info[45].mFileTimeLo = 0U;
  c4_info[45].mFileTimeHi = 0U;
  c4_info[46].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c4_info[46].name = "eml_index_plus";
  c4_info[46].dominantType = "int32";
  c4_info[46].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[46].fileTimeLo = 1286818778U;
  c4_info[46].fileTimeHi = 0U;
  c4_info[46].mFileTimeLo = 0U;
  c4_info[46].mFileTimeHi = 0U;
  c4_info[47].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c4_info[47].name = "eml_int_forloop_overflow_check";
  c4_info[47].dominantType = "";
  c4_info[47].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c4_info[47].fileTimeLo = 1311255316U;
  c4_info[47].fileTimeHi = 0U;
  c4_info[47].mFileTimeLo = 0U;
  c4_info[47].mFileTimeHi = 0U;
  c4_info[48].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[48].name = "eml_div";
  c4_info[48].dominantType = "double";
  c4_info[48].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_div.m";
  c4_info[48].fileTimeLo = 1313347810U;
  c4_info[48].fileTimeHi = 0U;
  c4_info[48].mFileTimeLo = 0U;
  c4_info[48].mFileTimeHi = 0U;
  c4_info[49].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[49].name = "eml_xscal";
  c4_info[49].dominantType = "int32";
  c4_info[49].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m";
  c4_info[49].fileTimeLo = 1299076776U;
  c4_info[49].fileTimeHi = 0U;
  c4_info[49].mFileTimeLo = 0U;
  c4_info[49].mFileTimeHi = 0U;
  c4_info[50].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m";
  c4_info[50].name = "eml_blas_inline";
  c4_info[50].dominantType = "";
  c4_info[50].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c4_info[50].fileTimeLo = 1299076768U;
  c4_info[50].fileTimeHi = 0U;
  c4_info[50].mFileTimeLo = 0U;
  c4_info[50].mFileTimeHi = 0U;
  c4_info[51].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m!below_threshold";
  c4_info[51].name = "length";
  c4_info[51].dominantType = "double";
  c4_info[51].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/length.m";
  c4_info[51].fileTimeLo = 1303146206U;
  c4_info[51].fileTimeHi = 0U;
  c4_info[51].mFileTimeLo = 0U;
  c4_info[51].mFileTimeHi = 0U;
  c4_info[52].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m";
  c4_info[52].name = "eml_scalar_eg";
  c4_info[52].dominantType = "double";
  c4_info[52].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[52].fileTimeLo = 1286818796U;
  c4_info[52].fileTimeHi = 0U;
  c4_info[52].mFileTimeLo = 0U;
  c4_info[52].mFileTimeHi = 0U;
  c4_info[53].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xscal.m";
  c4_info[53].name = "eml_refblas_xscal";
  c4_info[53].dominantType = "int32";
  c4_info[53].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c4_info[53].fileTimeLo = 1299076784U;
  c4_info[53].fileTimeHi = 0U;
  c4_info[53].mFileTimeLo = 0U;
  c4_info[53].mFileTimeHi = 0U;
  c4_info[54].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c4_info[54].name = "eml_index_class";
  c4_info[54].dominantType = "";
  c4_info[54].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[54].fileTimeLo = 1286818778U;
  c4_info[54].fileTimeHi = 0U;
  c4_info[54].mFileTimeLo = 0U;
  c4_info[54].mFileTimeHi = 0U;
  c4_info[55].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c4_info[55].name = "eml_index_minus";
  c4_info[55].dominantType = "int32";
  c4_info[55].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c4_info[55].fileTimeLo = 1286818778U;
  c4_info[55].fileTimeHi = 0U;
  c4_info[55].mFileTimeLo = 0U;
  c4_info[55].mFileTimeHi = 0U;
  c4_info[56].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c4_info[56].name = "eml_index_times";
  c4_info[56].dominantType = "int32";
  c4_info[56].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c4_info[56].fileTimeLo = 1286818780U;
  c4_info[56].fileTimeHi = 0U;
  c4_info[56].mFileTimeLo = 0U;
  c4_info[56].mFileTimeHi = 0U;
  c4_info[57].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c4_info[57].name = "eml_index_plus";
  c4_info[57].dominantType = "int32";
  c4_info[57].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[57].fileTimeLo = 1286818778U;
  c4_info[57].fileTimeHi = 0U;
  c4_info[57].mFileTimeLo = 0U;
  c4_info[57].mFileTimeHi = 0U;
  c4_info[58].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xscal.m";
  c4_info[58].name = "eml_int_forloop_overflow_check";
  c4_info[58].dominantType = "";
  c4_info[58].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c4_info[58].fileTimeLo = 1311255316U;
  c4_info[58].fileTimeHi = 0U;
  c4_info[58].mFileTimeLo = 0U;
  c4_info[58].mFileTimeHi = 0U;
  c4_info[59].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[59].name = "eml_xdotc";
  c4_info[59].dominantType = "int32";
  c4_info[59].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m";
  c4_info[59].fileTimeLo = 1299076772U;
  c4_info[59].fileTimeHi = 0U;
  c4_info[59].mFileTimeLo = 0U;
  c4_info[59].mFileTimeHi = 0U;
  c4_info[60].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m";
  c4_info[60].name = "eml_blas_inline";
  c4_info[60].dominantType = "";
  c4_info[60].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c4_info[60].fileTimeLo = 1299076768U;
  c4_info[60].fileTimeHi = 0U;
  c4_info[60].mFileTimeLo = 0U;
  c4_info[60].mFileTimeHi = 0U;
  c4_info[61].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m";
  c4_info[61].name = "eml_xdot";
  c4_info[61].dominantType = "int32";
  c4_info[61].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m";
  c4_info[61].fileTimeLo = 1299076772U;
  c4_info[61].fileTimeHi = 0U;
  c4_info[61].mFileTimeLo = 0U;
  c4_info[61].mFileTimeHi = 0U;
  c4_info[62].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m";
  c4_info[62].name = "eml_blas_inline";
  c4_info[62].dominantType = "";
  c4_info[62].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c4_info[62].fileTimeLo = 1299076768U;
  c4_info[62].fileTimeHi = 0U;
  c4_info[62].mFileTimeLo = 0U;
  c4_info[62].mFileTimeHi = 0U;
  c4_info[63].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m!below_threshold";
  c4_info[63].name = "length";
  c4_info[63].dominantType = "double";
  c4_info[63].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/length.m";
  c4_info[63].fileTimeLo = 1303146206U;
  c4_info[63].fileTimeHi = 0U;
  c4_info[63].mFileTimeLo = 0U;
  c4_info[63].mFileTimeHi = 0U;
}

static void c4_b_info_helper(c4_ResolvedFunctionInfo c4_info[149])
{
  c4_info[64].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m";
  c4_info[64].name = "eml_refblas_xdot";
  c4_info[64].dominantType = "int32";
  c4_info[64].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m";
  c4_info[64].fileTimeLo = 1299076772U;
  c4_info[64].fileTimeHi = 0U;
  c4_info[64].mFileTimeLo = 0U;
  c4_info[64].mFileTimeHi = 0U;
  c4_info[65].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m";
  c4_info[65].name = "eml_refblas_xdotx";
  c4_info[65].dominantType = "int32";
  c4_info[65].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c4_info[65].fileTimeLo = 1299076774U;
  c4_info[65].fileTimeHi = 0U;
  c4_info[65].mFileTimeLo = 0U;
  c4_info[65].mFileTimeHi = 0U;
  c4_info[66].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c4_info[66].name = "eml_scalar_eg";
  c4_info[66].dominantType = "double";
  c4_info[66].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[66].fileTimeLo = 1286818796U;
  c4_info[66].fileTimeHi = 0U;
  c4_info[66].mFileTimeLo = 0U;
  c4_info[66].mFileTimeHi = 0U;
  c4_info[67].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c4_info[67].name = "eml_index_class";
  c4_info[67].dominantType = "";
  c4_info[67].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[67].fileTimeLo = 1286818778U;
  c4_info[67].fileTimeHi = 0U;
  c4_info[67].mFileTimeLo = 0U;
  c4_info[67].mFileTimeHi = 0U;
  c4_info[68].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c4_info[68].name = "eml_int_forloop_overflow_check";
  c4_info[68].dominantType = "";
  c4_info[68].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c4_info[68].fileTimeLo = 1311255316U;
  c4_info[68].fileTimeHi = 0U;
  c4_info[68].mFileTimeLo = 0U;
  c4_info[68].mFileTimeHi = 0U;
  c4_info[69].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m";
  c4_info[69].name = "eml_index_plus";
  c4_info[69].dominantType = "int32";
  c4_info[69].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[69].fileTimeLo = 1286818778U;
  c4_info[69].fileTimeHi = 0U;
  c4_info[69].mFileTimeLo = 0U;
  c4_info[69].mFileTimeHi = 0U;
  c4_info[70].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[70].name = "eml_xaxpy";
  c4_info[70].dominantType = "int32";
  c4_info[70].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m";
  c4_info[70].fileTimeLo = 1299076770U;
  c4_info[70].fileTimeHi = 0U;
  c4_info[70].mFileTimeLo = 0U;
  c4_info[70].mFileTimeHi = 0U;
  c4_info[71].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m";
  c4_info[71].name = "eml_blas_inline";
  c4_info[71].dominantType = "";
  c4_info[71].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c4_info[71].fileTimeLo = 1299076768U;
  c4_info[71].fileTimeHi = 0U;
  c4_info[71].mFileTimeLo = 0U;
  c4_info[71].mFileTimeHi = 0U;
  c4_info[72].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xaxpy.m!below_threshold";
  c4_info[72].name = "length";
  c4_info[72].dominantType = "double";
  c4_info[72].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/length.m";
  c4_info[72].fileTimeLo = 1303146206U;
  c4_info[72].fileTimeHi = 0U;
  c4_info[72].mFileTimeLo = 0U;
  c4_info[72].mFileTimeHi = 0U;
  c4_info[73].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xaxpy.m";
  c4_info[73].name = "eml_scalar_eg";
  c4_info[73].dominantType = "double";
  c4_info[73].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[73].fileTimeLo = 1286818796U;
  c4_info[73].fileTimeHi = 0U;
  c4_info[73].mFileTimeLo = 0U;
  c4_info[73].mFileTimeHi = 0U;
  c4_info[74].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xaxpy.m";
  c4_info[74].name = "eml_refblas_xaxpy";
  c4_info[74].dominantType = "int32";
  c4_info[74].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c4_info[74].fileTimeLo = 1299076772U;
  c4_info[74].fileTimeHi = 0U;
  c4_info[74].mFileTimeLo = 0U;
  c4_info[74].mFileTimeHi = 0U;
  c4_info[75].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c4_info[75].name = "eml_index_class";
  c4_info[75].dominantType = "";
  c4_info[75].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[75].fileTimeLo = 1286818778U;
  c4_info[75].fileTimeHi = 0U;
  c4_info[75].mFileTimeLo = 0U;
  c4_info[75].mFileTimeHi = 0U;
  c4_info[76].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c4_info[76].name = "eml_isa_uint";
  c4_info[76].dominantType = "int32";
  c4_info[76].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_isa_uint.m";
  c4_info[76].fileTimeLo = 1286818784U;
  c4_info[76].fileTimeHi = 0U;
  c4_info[76].mFileTimeLo = 0U;
  c4_info[76].mFileTimeHi = 0U;
  c4_info[77].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c4_info[77].name = "eml_index_minus";
  c4_info[77].dominantType = "int32";
  c4_info[77].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c4_info[77].fileTimeLo = 1286818778U;
  c4_info[77].fileTimeHi = 0U;
  c4_info[77].mFileTimeLo = 0U;
  c4_info[77].mFileTimeHi = 0U;
  c4_info[78].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c4_info[78].name = "eml_int_forloop_overflow_check";
  c4_info[78].dominantType = "";
  c4_info[78].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c4_info[78].fileTimeLo = 1311255316U;
  c4_info[78].fileTimeHi = 0U;
  c4_info[78].mFileTimeLo = 0U;
  c4_info[78].mFileTimeHi = 0U;
  c4_info[79].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xaxpy.m";
  c4_info[79].name = "eml_index_plus";
  c4_info[79].dominantType = "int32";
  c4_info[79].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[79].fileTimeLo = 1286818778U;
  c4_info[79].fileTimeHi = 0U;
  c4_info[79].mFileTimeLo = 0U;
  c4_info[79].mFileTimeHi = 0U;
  c4_info[80].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c4_info[80].name = "intmin";
  c4_info[80].dominantType = "char";
  c4_info[80].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/intmin.m";
  c4_info[80].fileTimeLo = 1311255318U;
  c4_info[80].fileTimeHi = 0U;
  c4_info[80].mFileTimeLo = 0U;
  c4_info[80].mFileTimeHi = 0U;
  c4_info[81].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[81].name = "abs";
  c4_info[81].dominantType = "double";
  c4_info[81].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c4_info[81].fileTimeLo = 1286818694U;
  c4_info[81].fileTimeHi = 0U;
  c4_info[81].mFileTimeLo = 0U;
  c4_info[81].mFileTimeHi = 0U;
  c4_info[82].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[82].name = "mtimes";
  c4_info[82].dominantType = "double";
  c4_info[82].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[82].fileTimeLo = 1289519692U;
  c4_info[82].fileTimeHi = 0U;
  c4_info[82].mFileTimeLo = 0U;
  c4_info[82].mFileTimeHi = 0U;
  c4_info[83].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[83].name = "realmin";
  c4_info[83].dominantType = "char";
  c4_info[83].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/realmin.m";
  c4_info[83].fileTimeLo = 1307651242U;
  c4_info[83].fileTimeHi = 0U;
  c4_info[83].mFileTimeLo = 0U;
  c4_info[83].mFileTimeHi = 0U;
  c4_info[84].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[84].name = "eps";
  c4_info[84].dominantType = "char";
  c4_info[84].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eps.m";
  c4_info[84].fileTimeLo = 1307651240U;
  c4_info[84].fileTimeHi = 0U;
  c4_info[84].mFileTimeLo = 0U;
  c4_info[84].mFileTimeHi = 0U;
  c4_info[85].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eps.m";
  c4_info[85].name = "eml_is_float_class";
  c4_info[85].dominantType = "char";
  c4_info[85].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_is_float_class.m";
  c4_info[85].fileTimeLo = 1286818782U;
  c4_info[85].fileTimeHi = 0U;
  c4_info[85].mFileTimeLo = 0U;
  c4_info[85].mFileTimeHi = 0U;
  c4_info[86].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eps.m";
  c4_info[86].name = "eml_eps";
  c4_info[86].dominantType = "char";
  c4_info[86].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c4_info[86].fileTimeLo = 1307651240U;
  c4_info[86].fileTimeHi = 0U;
  c4_info[86].mFileTimeLo = 0U;
  c4_info[86].mFileTimeHi = 0U;
  c4_info[87].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_eps.m";
  c4_info[87].name = "eml_float_model";
  c4_info[87].dominantType = "char";
  c4_info[87].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c4_info[87].fileTimeLo = 1307651242U;
  c4_info[87].fileTimeHi = 0U;
  c4_info[87].mFileTimeLo = 0U;
  c4_info[87].mFileTimeHi = 0U;
  c4_info[88].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[88].name = "max";
  c4_info[88].dominantType = "double";
  c4_info[88].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/max.m";
  c4_info[88].fileTimeLo = 1311255316U;
  c4_info[88].fileTimeHi = 0U;
  c4_info[88].mFileTimeLo = 0U;
  c4_info[88].mFileTimeHi = 0U;
  c4_info[89].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/max.m";
  c4_info[89].name = "eml_min_or_max";
  c4_info[89].dominantType = "char";
  c4_info[89].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c4_info[89].fileTimeLo = 1303146212U;
  c4_info[89].fileTimeHi = 0U;
  c4_info[89].mFileTimeLo = 0U;
  c4_info[89].mFileTimeHi = 0U;
  c4_info[90].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c4_info[90].name = "eml_scalar_eg";
  c4_info[90].dominantType = "double";
  c4_info[90].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[90].fileTimeLo = 1286818796U;
  c4_info[90].fileTimeHi = 0U;
  c4_info[90].mFileTimeLo = 0U;
  c4_info[90].mFileTimeHi = 0U;
  c4_info[91].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum";
  c4_info[91].name = "eml_scalexp_alloc";
  c4_info[91].dominantType = "double";
  c4_info[91].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m";
  c4_info[91].fileTimeLo = 1286818796U;
  c4_info[91].fileTimeHi = 0U;
  c4_info[91].mFileTimeLo = 0U;
  c4_info[91].mFileTimeHi = 0U;
  c4_info[92].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum";
  c4_info[92].name = "eml_scalar_eg";
  c4_info[92].dominantType = "double";
  c4_info[92].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[92].fileTimeLo = 1286818796U;
  c4_info[92].fileTimeHi = 0U;
  c4_info[92].mFileTimeLo = 0U;
  c4_info[92].mFileTimeHi = 0U;
  c4_info[93].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[93].name = "eml_error";
  c4_info[93].dominantType = "char";
  c4_info[93].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_error.m";
  c4_info[93].fileTimeLo = 1305318000U;
  c4_info[93].fileTimeHi = 0U;
  c4_info[93].mFileTimeLo = 0U;
  c4_info[93].mFileTimeHi = 0U;
  c4_info[94].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum";
  c4_info[94].name = "eml_const_nonsingleton_dim";
  c4_info[94].dominantType = "double";
  c4_info[94].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m";
  c4_info[94].fileTimeLo = 1286818696U;
  c4_info[94].fileTimeHi = 0U;
  c4_info[94].mFileTimeLo = 0U;
  c4_info[94].mFileTimeHi = 0U;
  c4_info[95].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum";
  c4_info[95].name = "eml_scalar_eg";
  c4_info[95].dominantType = "double";
  c4_info[95].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[95].fileTimeLo = 1286818796U;
  c4_info[95].fileTimeHi = 0U;
  c4_info[95].mFileTimeLo = 0U;
  c4_info[95].mFileTimeHi = 0U;
  c4_info[96].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum";
  c4_info[96].name = "eml_index_class";
  c4_info[96].dominantType = "";
  c4_info[96].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[96].fileTimeLo = 1286818778U;
  c4_info[96].fileTimeHi = 0U;
  c4_info[96].mFileTimeLo = 0U;
  c4_info[96].mFileTimeHi = 0U;
  c4_info[97].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c4_info[97].name = "eml_index_class";
  c4_info[97].dominantType = "";
  c4_info[97].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[97].fileTimeLo = 1286818778U;
  c4_info[97].fileTimeHi = 0U;
  c4_info[97].mFileTimeLo = 0U;
  c4_info[97].mFileTimeHi = 0U;
  c4_info[98].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c4_info[98].name = "isnan";
  c4_info[98].dominantType = "double";
  c4_info[98].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/isnan.m";
  c4_info[98].fileTimeLo = 1286818760U;
  c4_info[98].fileTimeHi = 0U;
  c4_info[98].mFileTimeLo = 0U;
  c4_info[98].mFileTimeHi = 0U;
  c4_info[99].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c4_info[99].name = "eml_index_plus";
  c4_info[99].dominantType = "int32";
  c4_info[99].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[99].fileTimeLo = 1286818778U;
  c4_info[99].fileTimeHi = 0U;
  c4_info[99].mFileTimeLo = 0U;
  c4_info[99].mFileTimeHi = 0U;
  c4_info[100].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c4_info[100].name = "eml_int_forloop_overflow_check";
  c4_info[100].dominantType = "";
  c4_info[100].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c4_info[100].fileTimeLo = 1311255316U;
  c4_info[100].fileTimeHi = 0U;
  c4_info[100].mFileTimeLo = 0U;
  c4_info[100].mFileTimeHi = 0U;
  c4_info[101].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub";
  c4_info[101].name = "eml_relop";
  c4_info[101].dominantType = "function_handle";
  c4_info[101].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_relop.m";
  c4_info[101].fileTimeLo = 1292190510U;
  c4_info[101].fileTimeHi = 0U;
  c4_info[101].mFileTimeLo = 0U;
  c4_info[101].mFileTimeHi = 0U;
  c4_info[102].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[102].name = "sqrt";
  c4_info[102].dominantType = "double";
  c4_info[102].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c4_info[102].fileTimeLo = 1286818752U;
  c4_info[102].fileTimeHi = 0U;
  c4_info[102].mFileTimeLo = 0U;
  c4_info[102].mFileTimeHi = 0U;
  c4_info[103].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c4_info[103].name = "eml_error";
  c4_info[103].dominantType = "char";
  c4_info[103].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_error.m";
  c4_info[103].fileTimeLo = 1305318000U;
  c4_info[103].fileTimeHi = 0U;
  c4_info[103].mFileTimeLo = 0U;
  c4_info[103].mFileTimeHi = 0U;
  c4_info[104].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c4_info[104].name = "eml_scalar_sqrt";
  c4_info[104].dominantType = "double";
  c4_info[104].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m";
  c4_info[104].fileTimeLo = 1286818738U;
  c4_info[104].fileTimeHi = 0U;
  c4_info[104].mFileTimeLo = 0U;
  c4_info[104].mFileTimeHi = 0U;
  c4_info[105].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[105].name = "eml_xrotg";
  c4_info[105].dominantType = "double";
  c4_info[105].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m";
  c4_info[105].fileTimeLo = 1299076776U;
  c4_info[105].fileTimeHi = 0U;
  c4_info[105].mFileTimeLo = 0U;
  c4_info[105].mFileTimeHi = 0U;
  c4_info[106].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m";
  c4_info[106].name = "eml_blas_inline";
  c4_info[106].dominantType = "";
  c4_info[106].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c4_info[106].fileTimeLo = 1299076768U;
  c4_info[106].fileTimeHi = 0U;
  c4_info[106].mFileTimeLo = 0U;
  c4_info[106].mFileTimeHi = 0U;
  c4_info[107].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xrotg.m";
  c4_info[107].name = "eml_refblas_xrotg";
  c4_info[107].dominantType = "double";
  c4_info[107].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrotg.m";
  c4_info[107].fileTimeLo = 1299076784U;
  c4_info[107].fileTimeHi = 0U;
  c4_info[107].mFileTimeLo = 0U;
  c4_info[107].mFileTimeHi = 0U;
  c4_info[108].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrotg.m";
  c4_info[108].name = "abs";
  c4_info[108].dominantType = "double";
  c4_info[108].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c4_info[108].fileTimeLo = 1286818694U;
  c4_info[108].fileTimeHi = 0U;
  c4_info[108].mFileTimeLo = 0U;
  c4_info[108].mFileTimeHi = 0U;
  c4_info[109].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrotg.m";
  c4_info[109].name = "mrdivide";
  c4_info[109].dominantType = "double";
  c4_info[109].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c4_info[109].fileTimeLo = 1325124138U;
  c4_info[109].fileTimeHi = 0U;
  c4_info[109].mFileTimeLo = 1319729966U;
  c4_info[109].mFileTimeHi = 0U;
  c4_info[110].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mrdivide.p";
  c4_info[110].name = "rdivide";
  c4_info[110].dominantType = "double";
  c4_info[110].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/rdivide.m";
  c4_info[110].fileTimeLo = 1286818844U;
  c4_info[110].fileTimeHi = 0U;
  c4_info[110].mFileTimeLo = 0U;
  c4_info[110].mFileTimeHi = 0U;
  c4_info[111].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/rdivide.m";
  c4_info[111].name = "eml_div";
  c4_info[111].dominantType = "double";
  c4_info[111].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_div.m";
  c4_info[111].fileTimeLo = 1313347810U;
  c4_info[111].fileTimeHi = 0U;
  c4_info[111].mFileTimeLo = 0U;
  c4_info[111].mFileTimeHi = 0U;
  c4_info[112].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrotg.m";
  c4_info[112].name = "sqrt";
  c4_info[112].dominantType = "double";
  c4_info[112].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/sqrt.m";
  c4_info[112].fileTimeLo = 1286818752U;
  c4_info[112].fileTimeHi = 0U;
  c4_info[112].mFileTimeLo = 0U;
  c4_info[112].mFileTimeHi = 0U;
  c4_info[113].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xrotg.m!eml_ceval_xrotg";
  c4_info[113].name = "eml_scalar_eg";
  c4_info[113].dominantType = "double";
  c4_info[113].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[113].fileTimeLo = 1286818796U;
  c4_info[113].fileTimeHi = 0U;
  c4_info[113].mFileTimeLo = 0U;
  c4_info[113].mFileTimeHi = 0U;
  c4_info[114].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[114].name = "eml_xrot";
  c4_info[114].dominantType = "int32";
  c4_info[114].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xrot.m";
  c4_info[114].fileTimeLo = 1299076776U;
  c4_info[114].fileTimeHi = 0U;
  c4_info[114].mFileTimeLo = 0U;
  c4_info[114].mFileTimeHi = 0U;
  c4_info[115].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xrot.m";
  c4_info[115].name = "eml_blas_inline";
  c4_info[115].dominantType = "";
  c4_info[115].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c4_info[115].fileTimeLo = 1299076768U;
  c4_info[115].fileTimeHi = 0U;
  c4_info[115].mFileTimeLo = 0U;
  c4_info[115].mFileTimeHi = 0U;
  c4_info[116].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xrot.m";
  c4_info[116].name = "eml_scalar_eg";
  c4_info[116].dominantType = "double";
  c4_info[116].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[116].fileTimeLo = 1286818796U;
  c4_info[116].fileTimeHi = 0U;
  c4_info[116].mFileTimeLo = 0U;
  c4_info[116].mFileTimeHi = 0U;
  c4_info[117].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xrot.m";
  c4_info[117].name = "eml_refblas_xrot";
  c4_info[117].dominantType = "int32";
  c4_info[117].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrot.m";
  c4_info[117].fileTimeLo = 1299076784U;
  c4_info[117].fileTimeHi = 0U;
  c4_info[117].mFileTimeLo = 0U;
  c4_info[117].mFileTimeHi = 0U;
  c4_info[118].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrot.m";
  c4_info[118].name = "eml_index_class";
  c4_info[118].dominantType = "";
  c4_info[118].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[118].fileTimeLo = 1286818778U;
  c4_info[118].fileTimeHi = 0U;
  c4_info[118].mFileTimeLo = 0U;
  c4_info[118].mFileTimeHi = 0U;
  c4_info[119].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrot.m";
  c4_info[119].name = "eml_int_forloop_overflow_check";
  c4_info[119].dominantType = "";
  c4_info[119].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c4_info[119].fileTimeLo = 1311255316U;
  c4_info[119].fileTimeHi = 0U;
  c4_info[119].mFileTimeLo = 0U;
  c4_info[119].mFileTimeHi = 0U;
  c4_info[120].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrot.m";
  c4_info[120].name = "mtimes";
  c4_info[120].dominantType = "double";
  c4_info[120].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[120].fileTimeLo = 1289519692U;
  c4_info[120].fileTimeHi = 0U;
  c4_info[120].mFileTimeLo = 0U;
  c4_info[120].mFileTimeHi = 0U;
  c4_info[121].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xrot.m";
  c4_info[121].name = "eml_index_plus";
  c4_info[121].dominantType = "int32";
  c4_info[121].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[121].fileTimeLo = 1286818778U;
  c4_info[121].fileTimeHi = 0U;
  c4_info[121].mFileTimeLo = 0U;
  c4_info[121].mFileTimeHi = 0U;
  c4_info[122].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m";
  c4_info[122].name = "eml_xswap";
  c4_info[122].dominantType = "int32";
  c4_info[122].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c4_info[122].fileTimeLo = 1299076778U;
  c4_info[122].fileTimeHi = 0U;
  c4_info[122].mFileTimeLo = 0U;
  c4_info[122].mFileTimeHi = 0U;
  c4_info[123].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m";
  c4_info[123].name = "eml_blas_inline";
  c4_info[123].dominantType = "";
  c4_info[123].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c4_info[123].fileTimeLo = 1299076768U;
  c4_info[123].fileTimeHi = 0U;
  c4_info[123].mFileTimeLo = 0U;
  c4_info[123].mFileTimeHi = 0U;
  c4_info[124].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xswap.m";
  c4_info[124].name = "eml_refblas_xswap";
  c4_info[124].dominantType = "int32";
  c4_info[124].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c4_info[124].fileTimeLo = 1299076786U;
  c4_info[124].fileTimeHi = 0U;
  c4_info[124].mFileTimeLo = 0U;
  c4_info[124].mFileTimeHi = 0U;
  c4_info[125].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c4_info[125].name = "eml_index_class";
  c4_info[125].dominantType = "";
  c4_info[125].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[125].fileTimeLo = 1286818778U;
  c4_info[125].fileTimeHi = 0U;
  c4_info[125].mFileTimeLo = 0U;
  c4_info[125].mFileTimeHi = 0U;
  c4_info[126].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c4_info[126].name = "abs";
  c4_info[126].dominantType = "int32";
  c4_info[126].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c4_info[126].fileTimeLo = 1286818694U;
  c4_info[126].fileTimeHi = 0U;
  c4_info[126].mFileTimeLo = 0U;
  c4_info[126].mFileTimeHi = 0U;
  c4_info[127].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c4_info[127].name = "eml_scalar_abs";
  c4_info[127].dominantType = "int32";
  c4_info[127].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c4_info[127].fileTimeLo = 1286818712U;
  c4_info[127].fileTimeHi = 0U;
  c4_info[127].mFileTimeLo = 0U;
  c4_info[127].mFileTimeHi = 0U;
}

static void c4_c_info_helper(c4_ResolvedFunctionInfo c4_info[149])
{
  c4_info[128].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c4_info[128].name = "eml_int_forloop_overflow_check";
  c4_info[128].dominantType = "";
  c4_info[128].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c4_info[128].fileTimeLo = 1311255316U;
  c4_info[128].fileTimeHi = 0U;
  c4_info[128].mFileTimeLo = 0U;
  c4_info[128].mFileTimeHi = 0U;
  c4_info[129].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xswap.m";
  c4_info[129].name = "eml_index_plus";
  c4_info[129].dominantType = "int32";
  c4_info[129].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[129].fileTimeLo = 1286818778U;
  c4_info[129].fileTimeHi = 0U;
  c4_info[129].mFileTimeLo = 0U;
  c4_info[129].mFileTimeHi = 0U;
  c4_info[130].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c4_info[130].name = "mtimes";
  c4_info[130].dominantType = "double";
  c4_info[130].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[130].fileTimeLo = 1289519692U;
  c4_info[130].fileTimeHi = 0U;
  c4_info[130].mFileTimeLo = 0U;
  c4_info[130].mFileTimeHi = 0U;
  c4_info[131].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c4_info[131].name = "eps";
  c4_info[131].dominantType = "char";
  c4_info[131].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/eps.m";
  c4_info[131].fileTimeLo = 1307651240U;
  c4_info[131].fileTimeHi = 0U;
  c4_info[131].mFileTimeLo = 0U;
  c4_info[131].mFileTimeHi = 0U;
  c4_info[132].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c4_info[132].name = "eml_int_forloop_overflow_check";
  c4_info[132].dominantType = "";
  c4_info[132].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c4_info[132].fileTimeLo = 1311255316U;
  c4_info[132].fileTimeHi = 0U;
  c4_info[132].mFileTimeLo = 0U;
  c4_info[132].mFileTimeHi = 0U;
  c4_info[133].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c4_info[133].name = "eml_index_plus";
  c4_info[133].dominantType = "int32";
  c4_info[133].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[133].fileTimeLo = 1286818778U;
  c4_info[133].fileTimeHi = 0U;
  c4_info[133].mFileTimeLo = 0U;
  c4_info[133].mFileTimeHi = 0U;
  c4_info[134].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c4_info[134].name = "eml_div";
  c4_info[134].dominantType = "double";
  c4_info[134].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_div.m";
  c4_info[134].fileTimeLo = 1313347810U;
  c4_info[134].fileTimeHi = 0U;
  c4_info[134].mFileTimeLo = 0U;
  c4_info[134].mFileTimeHi = 0U;
  c4_info[135].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c4_info[135].name = "eml_xscal";
  c4_info[135].dominantType = "int32";
  c4_info[135].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m";
  c4_info[135].fileTimeLo = 1299076776U;
  c4_info[135].fileTimeHi = 0U;
  c4_info[135].mFileTimeLo = 0U;
  c4_info[135].mFileTimeHi = 0U;
  c4_info[136].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv";
  c4_info[136].name = "eml_xgemm";
  c4_info[136].dominantType = "int32";
  c4_info[136].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c4_info[136].fileTimeLo = 1299076772U;
  c4_info[136].fileTimeHi = 0U;
  c4_info[136].mFileTimeLo = 0U;
  c4_info[136].mFileTimeHi = 0U;
  c4_info[137].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c4_info[137].name = "eml_blas_inline";
  c4_info[137].dominantType = "";
  c4_info[137].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c4_info[137].fileTimeLo = 1299076768U;
  c4_info[137].fileTimeHi = 0U;
  c4_info[137].mFileTimeLo = 0U;
  c4_info[137].mFileTimeHi = 0U;
  c4_info[138].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c4_info[138].name = "min";
  c4_info[138].dominantType = "double";
  c4_info[138].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/min.m";
  c4_info[138].fileTimeLo = 1311255318U;
  c4_info[138].fileTimeHi = 0U;
  c4_info[138].mFileTimeLo = 0U;
  c4_info[138].mFileTimeHi = 0U;
  c4_info[139].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/datafun/min.m";
  c4_info[139].name = "eml_min_or_max";
  c4_info[139].dominantType = "char";
  c4_info[139].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_min_or_max.m";
  c4_info[139].fileTimeLo = 1303146212U;
  c4_info[139].fileTimeHi = 0U;
  c4_info[139].mFileTimeLo = 0U;
  c4_info[139].mFileTimeHi = 0U;
  c4_info[140].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c4_info[140].name = "mtimes";
  c4_info[140].dominantType = "double";
  c4_info[140].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c4_info[140].fileTimeLo = 1289519692U;
  c4_info[140].fileTimeHi = 0U;
  c4_info[140].mFileTimeLo = 0U;
  c4_info[140].mFileTimeHi = 0U;
  c4_info[141].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c4_info[141].name = "eml_scalar_eg";
  c4_info[141].dominantType = "double";
  c4_info[141].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[141].fileTimeLo = 1286818796U;
  c4_info[141].fileTimeHi = 0U;
  c4_info[141].mFileTimeLo = 0U;
  c4_info[141].mFileTimeHi = 0U;
  c4_info[142].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c4_info[142].name = "eml_refblas_xgemm";
  c4_info[142].dominantType = "int32";
  c4_info[142].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c4_info[142].fileTimeLo = 1299076774U;
  c4_info[142].fileTimeHi = 0U;
  c4_info[142].mFileTimeLo = 0U;
  c4_info[142].mFileTimeHi = 0U;
  c4_info[143].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c4_info[143].name = "eml_index_minus";
  c4_info[143].dominantType = "int32";
  c4_info[143].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c4_info[143].fileTimeLo = 1286818778U;
  c4_info[143].fileTimeHi = 0U;
  c4_info[143].mFileTimeLo = 0U;
  c4_info[143].mFileTimeHi = 0U;
  c4_info[144].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c4_info[144].name = "eml_index_class";
  c4_info[144].dominantType = "";
  c4_info[144].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c4_info[144].fileTimeLo = 1286818778U;
  c4_info[144].fileTimeHi = 0U;
  c4_info[144].mFileTimeLo = 0U;
  c4_info[144].mFileTimeHi = 0U;
  c4_info[145].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c4_info[145].name = "eml_scalar_eg";
  c4_info[145].dominantType = "double";
  c4_info[145].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c4_info[145].fileTimeLo = 1286818796U;
  c4_info[145].fileTimeHi = 0U;
  c4_info[145].mFileTimeLo = 0U;
  c4_info[145].mFileTimeHi = 0U;
  c4_info[146].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c4_info[146].name = "eml_index_times";
  c4_info[146].dominantType = "int32";
  c4_info[146].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c4_info[146].fileTimeLo = 1286818780U;
  c4_info[146].fileTimeHi = 0U;
  c4_info[146].mFileTimeLo = 0U;
  c4_info[146].mFileTimeHi = 0U;
  c4_info[147].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c4_info[147].name = "eml_index_plus";
  c4_info[147].dominantType = "int32";
  c4_info[147].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c4_info[147].fileTimeLo = 1286818778U;
  c4_info[147].fileTimeHi = 0U;
  c4_info[147].mFileTimeLo = 0U;
  c4_info[147].mFileTimeHi = 0U;
  c4_info[148].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c4_info[148].name = "eml_int_forloop_overflow_check";
  c4_info[148].dominantType = "";
  c4_info[148].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c4_info[148].fileTimeLo = 1311255316U;
  c4_info[148].fileTimeHi = 0U;
  c4_info[148].mFileTimeLo = 0U;
  c4_info[148].mFileTimeHi = 0U;
}

static void c4_eml_scalar_eg(SFc4_kinematicsInstanceStruct *chartInstance)
{
}

static void c4_eml_int_forloop_overflow_check(SFc4_kinematicsInstanceStruct
  *chartInstance)
{
}

static void c4_eml_error(SFc4_kinematicsInstanceStruct *chartInstance)
{
  int32_T c4_i38;
  static char_T c4_varargin_1[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'v', 'd', '_', 'm', 'a', 't', 'r', 'i', 'x',
    'W', 'i', 't', 'h', 'N', 'a', 'N', 'I', 'n', 'f' };

  char_T c4_u[33];
  const mxArray *c4_y = NULL;
  for (c4_i38 = 0; c4_i38 < 33; c4_i38++) {
    c4_u[c4_i38] = c4_varargin_1[c4_i38];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 10, 0U, 1U, 0U, 2, 1, 33), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c4_y));
}

static void c4_eml_xgesvd(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_A[72], real_T c4_U[72], real_T c4_S[6], real_T c4_V[36])
{
  int32_T c4_i39;
  real_T c4_b_A[72];
  int32_T c4_i40;
  real_T c4_s[6];
  int32_T c4_i41;
  real_T c4_e[6];
  int32_T c4_i42;
  real_T c4_work[12];
  int32_T c4_i43;
  int32_T c4_i44;
  real_T c4_Vf[36];
  int32_T c4_q;
  int32_T c4_b_q;
  int32_T c4_a;
  int32_T c4_qp1;
  int32_T c4_b_a;
  int32_T c4_qm1;
  int32_T c4_b;
  int32_T c4_c;
  int32_T c4_c_a;
  int32_T c4_b_b;
  int32_T c4_qq;
  int32_T c4_c_b;
  int32_T c4_nmq;
  int32_T c4_d_a;
  int32_T c4_nmqp1;
  int32_T c4_i45;
  real_T c4_c_A[72];
  real_T c4_nrm;
  real_T c4_absx;
  real_T c4_d;
  real_T c4_y;
  real_T c4_d1;
  int32_T c4_b_qp1;
  int32_T c4_jj;
  int32_T c4_b_jj;
  int32_T c4_e_a;
  int32_T c4_b_c;
  int32_T c4_d_b;
  int32_T c4_c_c;
  int32_T c4_f_a;
  int32_T c4_e_b;
  int32_T c4_qjj;
  int32_T c4_i46;
  real_T c4_d_A[72];
  int32_T c4_i47;
  real_T c4_e_A[72];
  real_T c4_t;
  int32_T c4_c_q;
  int32_T c4_ii;
  int32_T c4_b_ii;
  int32_T c4_f_b;
  int32_T c4_pmq;
  int32_T c4_i48;
  real_T c4_b_e[6];
  real_T c4_b_absx;
  real_T c4_b_d;
  real_T c4_b_y;
  real_T c4_d2;
  int32_T c4_c_qp1;
  int32_T c4_c_ii;
  int32_T c4_d_qp1;
  int32_T c4_c_jj;
  int32_T c4_g_a;
  int32_T c4_d_c;
  int32_T c4_g_b;
  int32_T c4_e_c;
  int32_T c4_h_a;
  int32_T c4_h_b;
  int32_T c4_qp1jj;
  int32_T c4_i49;
  real_T c4_f_A[72];
  int32_T c4_e_qp1;
  int32_T c4_d_jj;
  int32_T c4_i_a;
  int32_T c4_f_c;
  int32_T c4_i_b;
  int32_T c4_g_c;
  int32_T c4_j_a;
  int32_T c4_j_b;
  int32_T c4_i50;
  real_T c4_b_work[12];
  int32_T c4_f_qp1;
  int32_T c4_d_ii;
  int32_T c4_m;
  int32_T c4_d_q;
  int32_T c4_k_a;
  int32_T c4_k_b;
  int32_T c4_l_a;
  int32_T c4_m_a;
  int32_T c4_h_c;
  int32_T c4_l_b;
  int32_T c4_i_c;
  int32_T c4_n_a;
  int32_T c4_m_b;
  int32_T c4_g_qp1;
  int32_T c4_e_jj;
  int32_T c4_o_a;
  int32_T c4_j_c;
  int32_T c4_n_b;
  int32_T c4_k_c;
  int32_T c4_p_a;
  int32_T c4_o_b;
  int32_T c4_i51;
  real_T c4_b_U[72];
  int32_T c4_i52;
  real_T c4_c_U[72];
  int32_T c4_e_q;
  int32_T c4_e_ii;
  int32_T c4_q_a;
  int32_T c4_i53;
  int32_T c4_f_ii;
  int32_T c4_g_ii;
  int32_T c4_f_q;
  int32_T c4_r_a;
  int32_T c4_p_b;
  int32_T c4_s_a;
  int32_T c4_l_c;
  int32_T c4_q_b;
  int32_T c4_m_c;
  int32_T c4_t_a;
  int32_T c4_r_b;
  int32_T c4_qp1q;
  int32_T c4_h_qp1;
  int32_T c4_f_jj;
  int32_T c4_u_a;
  int32_T c4_n_c;
  int32_T c4_s_b;
  int32_T c4_o_c;
  int32_T c4_v_a;
  int32_T c4_t_b;
  int32_T c4_i54;
  real_T c4_b_Vf[36];
  int32_T c4_i55;
  real_T c4_c_Vf[36];
  int32_T c4_h_ii;
  int32_T c4_g_q;
  real_T c4_rt;
  real_T c4_r;
  int32_T c4_w_a;
  int32_T c4_p_c;
  int32_T c4_u_b;
  int32_T c4_q_c;
  int32_T c4_v_b;
  int32_T c4_colq;
  int32_T c4_i56;
  int32_T c4_x_a;
  int32_T c4_r_c;
  int32_T c4_y_a;
  int32_T c4_s_c;
  real_T c4_ab_a;
  real_T c4_w_b;
  real_T c4_c_y;
  int32_T c4_x_b;
  int32_T c4_t_c;
  int32_T c4_y_b;
  int32_T c4_colqp1;
  real_T c4_iter;
  real_T c4_tiny;
  real_T c4_snorm;
  int32_T c4_i_ii;
  real_T c4_varargin_1;
  real_T c4_varargin_2;
  real_T c4_b_varargin_2;
  real_T c4_varargin_3;
  real_T c4_x;
  real_T c4_d_y;
  real_T c4_b_x;
  real_T c4_e_y;
  real_T c4_xk;
  real_T c4_yk;
  real_T c4_c_x;
  real_T c4_f_y;
  real_T c4_maxval;
  real_T c4_b_varargin_1;
  real_T c4_c_varargin_2;
  real_T c4_d_varargin_2;
  real_T c4_b_varargin_3;
  real_T c4_d_x;
  real_T c4_g_y;
  real_T c4_e_x;
  real_T c4_h_y;
  real_T c4_b_xk;
  real_T c4_b_yk;
  real_T c4_f_x;
  real_T c4_i_y;
  int32_T c4_bb_a;
  int32_T c4_cb_a;
  int32_T c4_i57;
  int32_T c4_j_ii;
  int32_T c4_db_a;
  int32_T c4_u_c;
  real_T c4_test0;
  real_T c4_ztest0;
  real_T c4_ab_b;
  real_T c4_j_y;
  real_T c4_bb_b;
  real_T c4_k_y;
  int32_T c4_eb_a;
  int32_T c4_v_c;
  real_T c4_kase;
  int32_T c4_qs;
  int32_T c4_b_m;
  int32_T c4_k_ii;
  real_T c4_test;
  int32_T c4_fb_a;
  int32_T c4_w_c;
  int32_T c4_gb_a;
  int32_T c4_x_c;
  real_T c4_ztest;
  real_T c4_cb_b;
  real_T c4_l_y;
  int32_T c4_hb_a;
  int32_T c4_ib_a;
  int32_T c4_y_c;
  real_T c4_f;
  int32_T c4_jb_a;
  int32_T c4_ab_c;
  int32_T c4_kb_a;
  int32_T c4_i58;
  int32_T c4_k;
  int32_T c4_b_k;
  real_T c4_t1;
  real_T c4_b_t1;
  real_T c4_b_f;
  real_T c4_sn;
  real_T c4_cs;
  real_T c4_b_cs;
  real_T c4_b_sn;
  int32_T c4_lb_a;
  int32_T c4_km1;
  real_T c4_mb_a;
  real_T c4_db_b;
  real_T c4_nb_a;
  real_T c4_eb_b;
  real_T c4_m_y;
  int32_T c4_ob_a;
  int32_T c4_bb_c;
  int32_T c4_fb_b;
  int32_T c4_cb_c;
  int32_T c4_gb_b;
  int32_T c4_colk;
  int32_T c4_pb_a;
  int32_T c4_db_c;
  int32_T c4_hb_b;
  int32_T c4_eb_c;
  int32_T c4_ib_b;
  int32_T c4_colm;
  int32_T c4_qb_a;
  int32_T c4_h_q;
  int32_T c4_c_k;
  real_T c4_c_t1;
  real_T c4_unusedU0;
  real_T c4_c_sn;
  real_T c4_c_cs;
  real_T c4_rb_a;
  real_T c4_jb_b;
  real_T c4_sb_a;
  real_T c4_kb_b;
  real_T c4_n_y;
  int32_T c4_tb_a;
  int32_T c4_fb_c;
  int32_T c4_lb_b;
  int32_T c4_gb_c;
  int32_T c4_mb_b;
  int32_T c4_ub_a;
  int32_T c4_hb_c;
  int32_T c4_nb_b;
  int32_T c4_ib_c;
  int32_T c4_ob_b;
  int32_T c4_colqm1;
  int32_T c4_vb_a;
  int32_T c4_mm1;
  real_T c4_d3;
  real_T c4_d4;
  real_T c4_d5;
  real_T c4_d6;
  real_T c4_d7;
  real_T c4_c_varargin_1[5];
  int32_T c4_ixstart;
  real_T c4_mtmp;
  real_T c4_g_x;
  boolean_T c4_pb_b;
  int32_T c4_ix;
  int32_T c4_b_ix;
  real_T c4_h_x;
  boolean_T c4_qb_b;
  int32_T c4_wb_a;
  int32_T c4_i59;
  int32_T c4_c_ix;
  real_T c4_xb_a;
  real_T c4_rb_b;
  boolean_T c4_p;
  real_T c4_b_mtmp;
  real_T c4_scale;
  real_T c4_sm;
  real_T c4_smm1;
  real_T c4_emm1;
  real_T c4_sqds;
  real_T c4_eqds;
  real_T c4_yb_a;
  real_T c4_sb_b;
  real_T c4_o_y;
  real_T c4_ac_a;
  real_T c4_tb_b;
  real_T c4_p_y;
  real_T c4_ub_b;
  real_T c4_bc_a;
  real_T c4_vb_b;
  real_T c4_jb_c;
  real_T c4_cc_a;
  real_T c4_wb_b;
  real_T c4_shift;
  real_T c4_dc_a;
  real_T c4_xb_b;
  real_T c4_q_y;
  real_T c4_ec_a;
  real_T c4_yb_b;
  real_T c4_r_y;
  real_T c4_fc_a;
  real_T c4_ac_b;
  real_T c4_g;
  int32_T c4_i_q;
  int32_T c4_d_k;
  int32_T c4_gc_a;
  int32_T c4_hc_a;
  int32_T c4_kp1;
  real_T c4_c_f;
  real_T c4_unusedU1;
  real_T c4_d_sn;
  real_T c4_d_cs;
  real_T c4_ic_a;
  real_T c4_bc_b;
  real_T c4_s_y;
  real_T c4_jc_a;
  real_T c4_cc_b;
  real_T c4_t_y;
  real_T c4_kc_a;
  real_T c4_dc_b;
  real_T c4_u_y;
  real_T c4_lc_a;
  real_T c4_ec_b;
  real_T c4_v_y;
  real_T c4_mc_a;
  real_T c4_fc_b;
  real_T c4_nc_a;
  real_T c4_gc_b;
  real_T c4_w_y;
  int32_T c4_oc_a;
  int32_T c4_kb_c;
  int32_T c4_hc_b;
  int32_T c4_lb_c;
  int32_T c4_ic_b;
  int32_T c4_jc_b;
  int32_T c4_mb_c;
  int32_T c4_kc_b;
  int32_T c4_colkp1;
  real_T c4_d_f;
  real_T c4_unusedU2;
  real_T c4_e_sn;
  real_T c4_e_cs;
  real_T c4_pc_a;
  real_T c4_lc_b;
  real_T c4_x_y;
  real_T c4_qc_a;
  real_T c4_mc_b;
  real_T c4_y_y;
  real_T c4_rc_a;
  real_T c4_nc_b;
  real_T c4_ab_y;
  real_T c4_sc_a;
  real_T c4_oc_b;
  real_T c4_bb_y;
  real_T c4_tc_a;
  real_T c4_pc_b;
  real_T c4_uc_a;
  real_T c4_qc_b;
  real_T c4_cb_y;
  int32_T c4_vc_a;
  int32_T c4_nb_c;
  int32_T c4_rc_b;
  int32_T c4_ob_c;
  int32_T c4_sc_b;
  int32_T c4_tc_b;
  int32_T c4_pb_c;
  int32_T c4_uc_b;
  int32_T c4_wc_a;
  int32_T c4_qb_c;
  int32_T c4_e_k;
  int32_T c4_j;
  int32_T c4_b_j;
  int32_T c4_i;
  int32_T c4_b_i;
  int32_T c4_rb_c;
  int32_T c4_xc_a;
  int32_T c4_sb_c;
  int32_T c4_vc_b;
  int32_T c4_wc_b;
  int32_T c4_yc_a;
  int32_T c4_ad_a;
  int32_T c4_tb_c;
  int32_T c4_bd_a;
  int32_T c4_ub_c;
  int32_T c4_xc_b;
  int32_T c4_yc_b;
  int32_T c4_vb_c;
  int32_T c4_ad_b;
  int32_T c4_bd_b;
  int32_T c4_wb_c;
  int32_T c4_cd_a;
  int32_T c4_xb_c;
  int32_T c4_cd_b;
  int32_T c4_dd_b;
  int32_T c4_yb_c;
  int32_T c4_ed_b;
  int32_T c4_fd_b;
  int32_T c4_dd_a;
  real_T c4_d8;
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
  for (c4_i39 = 0; c4_i39 < 72; c4_i39++) {
    c4_b_A[c4_i39] = c4_A[c4_i39];
  }

  c4_eml_scalar_eg(chartInstance);
  for (c4_i40 = 0; c4_i40 < 6; c4_i40++) {
    c4_s[c4_i40] = 0.0;
  }

  for (c4_i41 = 0; c4_i41 < 6; c4_i41++) {
    c4_e[c4_i41] = 0.0;
  }

  for (c4_i42 = 0; c4_i42 < 12; c4_i42++) {
    c4_work[c4_i42] = 0.0;
  }

  for (c4_i43 = 0; c4_i43 < 72; c4_i43++) {
    c4_U[c4_i43] = 0.0;
  }

  for (c4_i44 = 0; c4_i44 < 36; c4_i44++) {
    c4_Vf[c4_i44] = 0.0;
  }

  c4_b_eml_int_forloop_overflow_check(chartInstance);
  for (c4_q = 1; c4_q < 7; c4_q++) {
    c4_b_q = c4_q;
    c4_a = c4_b_q + 1;
    c4_qp1 = c4_a;
    c4_b_a = c4_b_q - 1;
    c4_qm1 = c4_b_a;
    c4_b = c4_qm1;
    c4_c = 12 * c4_b;
    c4_c_a = c4_b_q;
    c4_b_b = c4_c;
    c4_qq = c4_c_a + c4_b_b;
    c4_c_b = c4_b_q;
    c4_nmq = 12 - c4_c_b;
    c4_d_a = c4_nmq + 1;
    c4_nmqp1 = c4_d_a;
    if (c4_b_q <= 6) {
      for (c4_i45 = 0; c4_i45 < 72; c4_i45++) {
        c4_c_A[c4_i45] = c4_b_A[c4_i45];
      }

      c4_nrm = c4_eml_xnrm2(chartInstance, c4_nmqp1, c4_c_A, c4_qq);
      if (c4_nrm > 0.0) {
        c4_absx = c4_nrm;
        c4_d = c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c4_qq), 1, 72, 1, 0) - 1];
        if (c4_d < 0.0) {
          c4_y = -c4_absx;
        } else {
          c4_y = c4_absx;
        }

        c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_b_q), 1, 6, 1, 0) - 1] = c4_y;
        c4_d1 = c4_eml_div(chartInstance, 1.0, c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_q), 1, 6, 1, 0) - 1]);
        c4_d_eml_xscal(chartInstance, c4_nmqp1, c4_d1, c4_b_A, c4_qq);
        c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_qq), 1, 72, 1, 0) - 1] = c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK
          ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_qq), 1, 72, 1, 0) - 1]
          + 1.0;
        c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_b_q), 1, 6, 1, 0) - 1] = -c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_q), 1, 6, 1, 0) - 1];
      } else {
        c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_b_q), 1, 6, 1, 0) - 1] = 0.0;
      }
    }

    c4_b_qp1 = c4_qp1;
    c4_c_eml_int_forloop_overflow_check(chartInstance, c4_b_qp1, 6);
    for (c4_jj = c4_b_qp1; c4_jj < 7; c4_jj++) {
      c4_b_jj = c4_jj;
      c4_e_a = c4_b_jj - 1;
      c4_b_c = c4_e_a;
      c4_d_b = c4_b_c;
      c4_c_c = 12 * c4_d_b;
      c4_f_a = c4_b_q;
      c4_e_b = c4_c_c;
      c4_qjj = c4_f_a + c4_e_b;
      if (c4_b_q <= 6) {
        if (c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c4_b_q), 1, 6, 1, 0) - 1] != 0.0) {
          for (c4_i46 = 0; c4_i46 < 72; c4_i46++) {
            c4_d_A[c4_i46] = c4_b_A[c4_i46];
          }

          for (c4_i47 = 0; c4_i47 < 72; c4_i47++) {
            c4_e_A[c4_i47] = c4_b_A[c4_i47];
          }

          c4_t = c4_eml_xdotc(chartInstance, c4_nmqp1, c4_d_A, c4_qq, c4_e_A,
                              c4_qjj);
          c4_t = -c4_eml_div(chartInstance, c4_t, c4_b_A
                             [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_b_q), 1, 12, 1, 0) + 12 *
                               (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_b_q), 1, 6, 2, 0) - 1)) - 1]);
          c4_e_eml_xaxpy(chartInstance, c4_nmqp1, c4_t, c4_qq, c4_b_A, c4_qjj);
        }
      }

      c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c4_b_jj), 1, 6, 1, 0) - 1] = c4_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_qjj), 1, 72, 1, 0) - 1];
    }

    if (c4_b_q <= 6) {
      c4_c_q = c4_b_q;
      c4_c_eml_int_forloop_overflow_check(chartInstance, c4_c_q, 12);
      for (c4_ii = c4_c_q; c4_ii < 13; c4_ii++) {
        c4_b_ii = c4_ii;
        c4_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c4_b_ii), 1, 12, 1, 0) + 12 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c4_b_q), 1, 6, 2, 0) - 1)) - 1] = c4_b_A
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c4_b_ii), 1, 12, 1, 0) + 12 * (_SFD_EML_ARRAY_BOUNDS_CHECK
             ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_q), 1, 6, 2, 0) -
             1)) - 1];
      }
    }

    if (c4_b_q <= 4) {
      c4_f_b = c4_b_q;
      c4_pmq = 6 - c4_f_b;
      for (c4_i48 = 0; c4_i48 < 6; c4_i48++) {
        c4_b_e[c4_i48] = c4_e[c4_i48];
      }

      c4_nrm = c4_b_eml_xnrm2(chartInstance, c4_pmq, c4_b_e, c4_qp1);
      if (c4_nrm == 0.0) {
        c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_b_q), 1, 6, 1, 0) - 1] = 0.0;
      } else {
        c4_b_absx = c4_nrm;
        c4_b_d = c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c4_qp1), 1, 6, 1, 0) - 1];
        if (c4_b_d < 0.0) {
          c4_b_y = -c4_b_absx;
        } else {
          c4_b_y = c4_b_absx;
        }

        c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_b_q), 1, 6, 1, 0) - 1] = c4_b_y;
        c4_d2 = c4_eml_div(chartInstance, 1.0, c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_q), 1, 6, 1, 0) - 1]);
        c4_e_eml_xscal(chartInstance, c4_pmq, c4_d2, c4_e, c4_qp1);
        c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_qp1), 1, 6, 1, 0) - 1] = c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
          "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_qp1), 1, 6, 1, 0) - 1]
          + 1.0;
      }

      c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c4_b_q), 1, 6, 1, 0) - 1] = -c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_q), 1, 6, 1, 0) - 1];
      if (c4_qp1 <= 12) {
        if (c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c4_b_q), 1, 6, 1, 0) - 1] != 0.0) {
          c4_c_qp1 = c4_qp1;
          c4_c_eml_int_forloop_overflow_check(chartInstance, c4_c_qp1, 12);
          for (c4_c_ii = c4_c_qp1; c4_c_ii < 13; c4_c_ii++) {
            c4_b_ii = c4_c_ii;
            c4_work[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c4_b_ii), 1, 12, 1, 0) - 1] = 0.0;
          }

          c4_d_qp1 = c4_qp1;
          c4_c_eml_int_forloop_overflow_check(chartInstance, c4_d_qp1, 6);
          for (c4_c_jj = c4_d_qp1; c4_c_jj < 7; c4_c_jj++) {
            c4_b_jj = c4_c_jj;
            c4_g_a = c4_b_jj - 1;
            c4_d_c = c4_g_a;
            c4_g_b = c4_d_c;
            c4_e_c = 12 * c4_g_b;
            c4_h_a = c4_qp1;
            c4_h_b = c4_e_c;
            c4_qp1jj = c4_h_a + c4_h_b;
            for (c4_i49 = 0; c4_i49 < 72; c4_i49++) {
              c4_f_A[c4_i49] = c4_b_A[c4_i49];
            }

            c4_f_eml_xaxpy(chartInstance, c4_nmq,
                           c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c4_b_jj), 1, 6, 1, 0) - 1], c4_f_A,
                           c4_qp1jj, c4_work, c4_qp1);
          }

          c4_e_qp1 = c4_qp1;
          c4_c_eml_int_forloop_overflow_check(chartInstance, c4_e_qp1, 6);
          for (c4_d_jj = c4_e_qp1; c4_d_jj < 7; c4_d_jj++) {
            c4_b_jj = c4_d_jj;
            c4_t = c4_eml_div(chartInstance, -c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
              "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_jj), 1, 6, 1, 0)
                              - 1], c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_qp1), 1, 6, 1, 0) - 1]);
            c4_i_a = c4_b_jj - 1;
            c4_f_c = c4_i_a;
            c4_i_b = c4_f_c;
            c4_g_c = 12 * c4_i_b;
            c4_j_a = c4_qp1;
            c4_j_b = c4_g_c;
            c4_qp1jj = c4_j_a + c4_j_b;
            for (c4_i50 = 0; c4_i50 < 12; c4_i50++) {
              c4_b_work[c4_i50] = c4_work[c4_i50];
            }

            c4_g_eml_xaxpy(chartInstance, c4_nmq, c4_t, c4_b_work, c4_qp1,
                           c4_b_A, c4_qp1jj);
          }
        }
      }

      c4_f_qp1 = c4_qp1;
      c4_c_eml_int_forloop_overflow_check(chartInstance, c4_f_qp1, 6);
      for (c4_d_ii = c4_f_qp1; c4_d_ii < 7; c4_d_ii++) {
        c4_b_ii = c4_d_ii;
        c4_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c4_b_ii), 1, 6, 1, 0) + 6 *
               (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                  (real_T)c4_b_q), 1, 6, 2, 0) - 1)) - 1] =
          c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_b_ii), 1, 6, 1, 0) - 1];
      }
    }
  }

  c4_m = 6;
  c4_e[4] = c4_b_A[64];
  c4_e[5] = 0.0;
  c4_d_eml_int_forloop_overflow_check(chartInstance);
  for (c4_d_q = 6; c4_d_q > 0; c4_d_q--) {
    c4_b_q = c4_d_q;
    c4_k_a = c4_b_q + 1;
    c4_qp1 = c4_k_a;
    c4_k_b = c4_b_q;
    c4_nmq = 12 - c4_k_b;
    c4_l_a = c4_nmq + 1;
    c4_nmqp1 = c4_l_a;
    c4_m_a = c4_b_q - 1;
    c4_h_c = c4_m_a;
    c4_l_b = c4_h_c;
    c4_i_c = 12 * c4_l_b;
    c4_n_a = c4_b_q;
    c4_m_b = c4_i_c;
    c4_qq = c4_n_a + c4_m_b;
    if (c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_b_q), 1, 6, 1, 0) - 1] != 0.0) {
      c4_g_qp1 = c4_qp1;
      c4_c_eml_int_forloop_overflow_check(chartInstance, c4_g_qp1, 6);
      for (c4_e_jj = c4_g_qp1; c4_e_jj < 7; c4_e_jj++) {
        c4_b_jj = c4_e_jj;
        c4_o_a = c4_b_jj - 1;
        c4_j_c = c4_o_a;
        c4_n_b = c4_j_c;
        c4_k_c = 12 * c4_n_b;
        c4_p_a = c4_b_q;
        c4_o_b = c4_k_c;
        c4_qjj = c4_p_a + c4_o_b;
        for (c4_i51 = 0; c4_i51 < 72; c4_i51++) {
          c4_b_U[c4_i51] = c4_U[c4_i51];
        }

        for (c4_i52 = 0; c4_i52 < 72; c4_i52++) {
          c4_c_U[c4_i52] = c4_U[c4_i52];
        }

        c4_t = c4_eml_xdotc(chartInstance, c4_nmqp1, c4_b_U, c4_qq, c4_c_U,
                            c4_qjj);
        c4_t = -c4_eml_div(chartInstance, c4_t, c4_U[_SFD_EML_ARRAY_BOUNDS_CHECK
                           ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_qq),
                            1, 72, 1, 0) - 1]);
        c4_e_eml_xaxpy(chartInstance, c4_nmqp1, c4_t, c4_qq, c4_U, c4_qjj);
      }

      c4_e_q = c4_b_q;
      c4_c_eml_int_forloop_overflow_check(chartInstance, c4_e_q, 12);
      for (c4_e_ii = c4_e_q; c4_e_ii < 13; c4_e_ii++) {
        c4_b_ii = c4_e_ii;
        c4_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c4_b_ii), 1, 12, 1, 0) + 12 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c4_b_q), 1, 6, 2, 0) - 1)) - 1] = -c4_U
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c4_b_ii), 1, 12, 1, 0) + 12 * (_SFD_EML_ARRAY_BOUNDS_CHECK
             ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_q), 1, 6, 2, 0) -
             1)) - 1];
      }

      c4_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c4_qq), 1, 72, 1, 0) - 1] = c4_U[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_qq), 1, 72, 1, 0) - 1] + 1.0;
      c4_q_a = c4_b_q - 1;
      c4_i53 = c4_q_a;
      c4_c_eml_int_forloop_overflow_check(chartInstance, 1, c4_i53);
      for (c4_f_ii = 1; c4_f_ii <= c4_i53; c4_f_ii++) {
        c4_b_ii = c4_f_ii;
        c4_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c4_b_ii), 1, 12, 1, 0) + 12 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c4_b_q), 1, 6, 2, 0) - 1)) - 1] = 0.0;
      }
    } else {
      c4_c_eml_int_forloop_overflow_check(chartInstance, 1, 12);
      for (c4_g_ii = 1; c4_g_ii < 13; c4_g_ii++) {
        c4_b_ii = c4_g_ii;
        c4_U[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                (real_T)c4_b_ii), 1, 12, 1, 0) + 12 *
              (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                 (real_T)c4_b_q), 1, 6, 2, 0) - 1)) - 1] = 0.0;
      }

      c4_U[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c4_qq), 1, 72, 1, 0) - 1] = 1.0;
    }
  }

  c4_d_eml_int_forloop_overflow_check(chartInstance);
  for (c4_f_q = 6; c4_f_q > 0; c4_f_q--) {
    c4_b_q = c4_f_q;
    if (c4_b_q <= 4) {
      if (c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_q), 1, 6, 1, 0) - 1] != 0.0) {
        c4_r_a = c4_b_q + 1;
        c4_qp1 = c4_r_a;
        c4_p_b = c4_b_q;
        c4_pmq = 6 - c4_p_b;
        c4_s_a = c4_b_q - 1;
        c4_l_c = c4_s_a;
        c4_q_b = c4_l_c;
        c4_m_c = 6 * c4_q_b;
        c4_t_a = c4_qp1;
        c4_r_b = c4_m_c;
        c4_qp1q = c4_t_a + c4_r_b;
        c4_h_qp1 = c4_qp1;
        c4_c_eml_int_forloop_overflow_check(chartInstance, c4_h_qp1, 6);
        for (c4_f_jj = c4_h_qp1; c4_f_jj < 7; c4_f_jj++) {
          c4_b_jj = c4_f_jj;
          c4_u_a = c4_b_jj - 1;
          c4_n_c = c4_u_a;
          c4_s_b = c4_n_c;
          c4_o_c = 6 * c4_s_b;
          c4_v_a = c4_qp1;
          c4_t_b = c4_o_c;
          c4_qp1jj = c4_v_a + c4_t_b;
          for (c4_i54 = 0; c4_i54 < 36; c4_i54++) {
            c4_b_Vf[c4_i54] = c4_Vf[c4_i54];
          }

          for (c4_i55 = 0; c4_i55 < 36; c4_i55++) {
            c4_c_Vf[c4_i55] = c4_Vf[c4_i55];
          }

          c4_t = c4_b_eml_xdotc(chartInstance, c4_pmq, c4_b_Vf, c4_qp1q, c4_c_Vf,
                                c4_qp1jj);
          c4_t = -c4_eml_div(chartInstance, c4_t,
                             c4_Vf[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_qp1q), 1, 36, 1, 0) - 1]);
          c4_h_eml_xaxpy(chartInstance, c4_pmq, c4_t, c4_qp1q, c4_Vf, c4_qp1jj);
        }
      }
    }

    c4_b_eml_int_forloop_overflow_check(chartInstance);
    for (c4_h_ii = 1; c4_h_ii < 7; c4_h_ii++) {
      c4_b_ii = c4_h_ii;
      c4_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
               (real_T)c4_b_ii), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
               "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_q), 1, 6, 2, 0)
              - 1)) - 1] = 0.0;
    }

    c4_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c4_b_q), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_q), 1, 6, 2, 0) - 1))
      - 1] = 1.0;
  }

  c4_b_eml_int_forloop_overflow_check(chartInstance);
  for (c4_g_q = 1; c4_g_q < 7; c4_g_q++) {
    c4_b_q = c4_g_q;
    if (c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_b_q), 1, 6, 1, 0) - 1] != 0.0) {
      c4_rt = c4_abs(chartInstance, c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_q), 1, 6, 1, 0) - 1]);
      c4_r = c4_eml_div(chartInstance, c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_q), 1, 6, 1, 0) - 1], c4_rt);
      c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c4_b_q), 1, 6, 1, 0) - 1] = c4_rt;
      if (c4_b_q < 6) {
        c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_b_q), 1, 6, 1, 0) - 1] = c4_eml_div(chartInstance,
          c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_b_q), 1, 6, 1, 0) - 1], c4_r);
      }

      if (c4_b_q <= 12) {
        c4_w_a = c4_b_q - 1;
        c4_p_c = c4_w_a;
        c4_u_b = c4_p_c;
        c4_q_c = 12 * c4_u_b;
        c4_v_b = c4_q_c + 1;
        c4_colq = c4_v_b;
        c4_i56 = 12;
        c4_d_eml_xscal(chartInstance, c4_i56, c4_r, c4_U, c4_colq);
      }
    }

    if (c4_b_q < 6) {
      if (c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_q), 1, 6, 1, 0) - 1] != 0.0) {
        c4_rt = c4_abs(chartInstance, c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_q), 1, 6, 1, 0) - 1]);
        c4_r = c4_eml_div(chartInstance, c4_rt, c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK
                          ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_q),
                           1, 6, 1, 0) - 1]);
        c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_b_q), 1, 6, 1, 0) - 1] = c4_rt;
        c4_x_a = c4_b_q + 1;
        c4_r_c = c4_x_a;
        c4_y_a = c4_b_q + 1;
        c4_s_c = c4_y_a;
        c4_ab_a = c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c4_s_c), 1, 6, 1, 0) - 1];
        c4_w_b = c4_r;
        c4_c_y = c4_ab_a * c4_w_b;
        c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_r_c), 1, 6, 1, 0) - 1] = c4_c_y;
        c4_x_b = c4_b_q;
        c4_t_c = 6 * c4_x_b;
        c4_y_b = c4_t_c + 1;
        c4_colqp1 = c4_y_b;
        c4_f_eml_xscal(chartInstance, c4_r, c4_Vf, c4_colqp1);
      }
    }
  }

  c4_iter = 0.0;
  c4_realmin(chartInstance);
  c4_eps(chartInstance);
  c4_tiny = c4_eml_div(chartInstance, 2.2250738585072014E-308,
                       2.2204460492503131E-16);
  c4_snorm = 0.0;
  c4_b_eml_int_forloop_overflow_check(chartInstance);
  for (c4_i_ii = 1; c4_i_ii < 7; c4_i_ii++) {
    c4_b_ii = c4_i_ii;
    c4_varargin_1 = c4_abs(chartInstance, c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_ii), 1, 6, 1, 0) - 1]);
    c4_varargin_2 = c4_abs(chartInstance, c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_ii), 1, 6, 1, 0) - 1]);
    c4_b_varargin_2 = c4_varargin_1;
    c4_varargin_3 = c4_varargin_2;
    c4_x = c4_b_varargin_2;
    c4_d_y = c4_varargin_3;
    c4_b_x = c4_x;
    c4_e_y = c4_d_y;
    c4_c_eml_scalar_eg(chartInstance);
    c4_xk = c4_b_x;
    c4_yk = c4_e_y;
    c4_c_x = c4_xk;
    c4_f_y = c4_yk;
    c4_c_eml_scalar_eg(chartInstance);
    c4_maxval = muDoubleScalarMax(c4_c_x, c4_f_y);
    c4_b_varargin_1 = c4_snorm;
    c4_c_varargin_2 = c4_maxval;
    c4_d_varargin_2 = c4_b_varargin_1;
    c4_b_varargin_3 = c4_c_varargin_2;
    c4_d_x = c4_d_varargin_2;
    c4_g_y = c4_b_varargin_3;
    c4_e_x = c4_d_x;
    c4_h_y = c4_g_y;
    c4_c_eml_scalar_eg(chartInstance);
    c4_b_xk = c4_e_x;
    c4_b_yk = c4_h_y;
    c4_f_x = c4_b_xk;
    c4_i_y = c4_b_yk;
    c4_c_eml_scalar_eg(chartInstance);
    c4_snorm = muDoubleScalarMax(c4_f_x, c4_i_y);
  }

  exitg1 = FALSE;
  while ((exitg1 == 0U) && ((real_T)c4_m > 0.0)) {
    if (c4_iter >= 75.0) {
      c4_b_eml_error(chartInstance);
      exitg1 = TRUE;
    } else {
      c4_bb_a = c4_m - 1;
      c4_b_q = c4_bb_a;
      c4_cb_a = c4_m - 1;
      c4_i57 = c4_cb_a;
      c4_e_eml_int_forloop_overflow_check(chartInstance, c4_i57);
      c4_j_ii = c4_i57;
      guard3 = FALSE;
      guard4 = FALSE;
      exitg5 = FALSE;
      while ((exitg5 == 0U) && (c4_j_ii > -1)) {
        c4_b_ii = c4_j_ii;
        c4_b_q = c4_b_ii;
        if ((real_T)c4_b_ii == 0.0) {
          exitg5 = TRUE;
        } else {
          c4_db_a = c4_b_ii + 1;
          c4_u_c = c4_db_a;
          c4_test0 = c4_abs(chartInstance, c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_ii), 1,
            6, 1, 0) - 1]) + c4_abs(chartInstance,
            c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
                                      (real_T)c4_u_c), 1, 6, 1, 0) - 1]);
          c4_ztest0 = c4_abs(chartInstance, c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_ii),
            1, 6, 1, 0) - 1]);
          c4_eps(chartInstance);
          c4_ab_b = c4_test0;
          c4_j_y = 2.2204460492503131E-16 * c4_ab_b;
          if (c4_ztest0 <= c4_j_y) {
            guard4 = TRUE;
            exitg5 = TRUE;
          } else if (c4_ztest0 <= c4_tiny) {
            guard4 = TRUE;
            exitg5 = TRUE;
          } else {
            guard11 = FALSE;
            if (c4_iter > 20.0) {
              c4_eps(chartInstance);
              c4_bb_b = c4_snorm;
              c4_k_y = 2.2204460492503131E-16 * c4_bb_b;
              if (c4_ztest0 <= c4_k_y) {
                guard3 = TRUE;
                exitg5 = TRUE;
              } else {
                guard11 = TRUE;
              }
            } else {
              guard11 = TRUE;
            }

            if (guard11 == TRUE) {
              c4_j_ii--;
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
        c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_b_ii), 1, 6, 1, 0) - 1] = 0.0;
      }

      c4_eb_a = c4_m - 1;
      c4_v_c = c4_eb_a;
      if (c4_b_q == c4_v_c) {
        c4_kase = 4.0;
      } else {
        c4_qs = c4_m;
        c4_b_m = c4_m;
        c4_f_eml_int_forloop_overflow_check(chartInstance, c4_b_m, c4_b_q);
        c4_k_ii = c4_b_m;
        guard2 = FALSE;
        exitg4 = FALSE;
        while ((exitg4 == 0U) && (c4_k_ii >= c4_b_q)) {
          c4_b_ii = c4_k_ii;
          c4_qs = c4_b_ii;
          if (c4_b_ii == c4_b_q) {
            exitg4 = TRUE;
          } else {
            c4_test = 0.0;
            if (c4_b_ii < c4_m) {
              c4_test = c4_abs(chartInstance, c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
                "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_ii), 1, 6, 1, 0)
                               - 1]);
            }

            c4_fb_a = c4_b_q + 1;
            c4_w_c = c4_fb_a;
            if (c4_b_ii > c4_w_c) {
              c4_gb_a = c4_b_ii - 1;
              c4_x_c = c4_gb_a;
              c4_test += c4_abs(chartInstance, c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
                "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_x_c), 1, 6, 1, 0)
                                - 1]);
            }

            c4_ztest = c4_abs(chartInstance, c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                               (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_ii),
              1, 6, 1, 0) - 1]);
            c4_eps(chartInstance);
            c4_cb_b = c4_test;
            c4_l_y = 2.2204460492503131E-16 * c4_cb_b;
            if (c4_ztest <= c4_l_y) {
              guard2 = TRUE;
              exitg4 = TRUE;
            } else if (c4_ztest <= c4_tiny) {
              guard2 = TRUE;
              exitg4 = TRUE;
            } else {
              c4_k_ii--;
              guard2 = FALSE;
            }
          }
        }

        if (guard2 == TRUE) {
          c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_ii), 1, 6, 1, 0) - 1] = 0.0;
        }

        if (c4_qs == c4_b_q) {
          c4_kase = 3.0;
        } else if (c4_qs == c4_m) {
          c4_kase = 1.0;
        } else {
          c4_kase = 2.0;
          c4_b_q = c4_qs;
        }
      }

      c4_hb_a = c4_b_q + 1;
      c4_b_q = c4_hb_a;
      switch ((int32_T)_SFD_INTEGER_CHECK("", c4_kase)) {
       case 1:
        c4_ib_a = c4_m - 1;
        c4_y_c = c4_ib_a;
        c4_f = c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c4_y_c), 1, 6, 1, 0) - 1];
        c4_jb_a = c4_m - 1;
        c4_ab_c = c4_jb_a;
        c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_ab_c), 1, 6, 1, 0) - 1] = 0.0;
        c4_kb_a = c4_m - 1;
        c4_i58 = c4_kb_a;
        c4_f_eml_int_forloop_overflow_check(chartInstance, c4_i58, c4_b_q);
        for (c4_k = c4_i58; c4_k >= c4_b_q; c4_k--) {
          c4_b_k = c4_k;
          c4_t1 = c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_b_k), 1, 6, 1, 0) - 1];
          c4_b_t1 = c4_t1;
          c4_b_f = c4_f;
          c4_b_eml_xrotg(chartInstance, &c4_b_t1, &c4_b_f, &c4_cs, &c4_sn);
          c4_t1 = c4_b_t1;
          c4_f = c4_b_f;
          c4_b_cs = c4_cs;
          c4_b_sn = c4_sn;
          c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_k), 1, 6, 1, 0) - 1] = c4_t1;
          if (c4_b_k > c4_b_q) {
            c4_lb_a = c4_b_k - 1;
            c4_km1 = c4_lb_a;
            c4_mb_a = -c4_b_sn;
            c4_db_b = c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c4_km1), 1, 6, 1, 0) - 1];
            c4_f = c4_mb_a * c4_db_b;
            c4_nb_a = c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c4_km1), 1, 6, 1, 0) - 1];
            c4_eb_b = c4_b_cs;
            c4_m_y = c4_nb_a * c4_eb_b;
            c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c4_km1), 1, 6, 1, 0) - 1] = c4_m_y;
          }

          c4_ob_a = c4_b_k - 1;
          c4_bb_c = c4_ob_a;
          c4_fb_b = c4_bb_c;
          c4_cb_c = 6 * c4_fb_b;
          c4_gb_b = c4_cb_c + 1;
          c4_colk = c4_gb_b;
          c4_pb_a = c4_m - 1;
          c4_db_c = c4_pb_a;
          c4_hb_b = c4_db_c;
          c4_eb_c = 6 * c4_hb_b;
          c4_ib_b = c4_eb_c + 1;
          c4_colm = c4_ib_b;
          c4_c_eml_xrot(chartInstance, c4_Vf, c4_colk, c4_colm, c4_b_cs, c4_b_sn);
        }
        break;

       case 2:
        c4_qb_a = c4_b_q - 1;
        c4_qm1 = c4_qb_a;
        c4_f = c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
          "", (real_T)c4_qm1), 1, 6, 1, 0) - 1];
        c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_qm1), 1, 6, 1, 0) - 1] = 0.0;
        c4_h_q = c4_b_q;
        c4_c_eml_int_forloop_overflow_check(chartInstance, c4_h_q, c4_m);
        for (c4_c_k = c4_h_q; c4_c_k <= c4_m; c4_c_k++) {
          c4_b_k = c4_c_k;
          c4_t1 = c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_b_k), 1, 6, 1, 0) - 1];
          c4_c_t1 = c4_t1;
          c4_unusedU0 = c4_f;
          c4_b_eml_xrotg(chartInstance, &c4_c_t1, &c4_unusedU0, &c4_c_cs,
                         &c4_c_sn);
          c4_t1 = c4_c_t1;
          c4_b_cs = c4_c_cs;
          c4_b_sn = c4_c_sn;
          c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_k), 1, 6, 1, 0) - 1] = c4_t1;
          c4_rb_a = -c4_b_sn;
          c4_jb_b = c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_b_k), 1, 6, 1, 0) - 1];
          c4_f = c4_rb_a * c4_jb_b;
          c4_sb_a = c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_b_k), 1, 6, 1, 0) - 1];
          c4_kb_b = c4_b_cs;
          c4_n_y = c4_sb_a * c4_kb_b;
          c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_k), 1, 6, 1, 0) - 1] = c4_n_y;
          c4_tb_a = c4_b_k - 1;
          c4_fb_c = c4_tb_a;
          c4_lb_b = c4_fb_c;
          c4_gb_c = 12 * c4_lb_b;
          c4_mb_b = c4_gb_c + 1;
          c4_colk = c4_mb_b;
          c4_ub_a = c4_qm1 - 1;
          c4_hb_c = c4_ub_a;
          c4_nb_b = c4_hb_c;
          c4_ib_c = 12 * c4_nb_b;
          c4_ob_b = c4_ib_c + 1;
          c4_colqm1 = c4_ob_b;
          c4_d_eml_xrot(chartInstance, c4_U, c4_colk, c4_colqm1, c4_b_cs,
                        c4_b_sn);
        }
        break;

       case 3:
        c4_vb_a = c4_m - 1;
        c4_mm1 = c4_vb_a;
        c4_d3 = c4_abs(chartInstance, c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_m), 1, 6, 1, 0) - 1]);
        c4_d4 = c4_abs(chartInstance, c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_mm1), 1, 6, 1, 0) - 1]);
        c4_d5 = c4_abs(chartInstance, c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_mm1), 1, 6, 1, 0) - 1]);
        c4_d6 = c4_abs(chartInstance, c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_q), 1, 6, 1, 0) - 1]);
        c4_d7 = c4_abs(chartInstance, c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_q), 1, 6, 1, 0) - 1]);
        c4_c_varargin_1[0] = c4_d3;
        c4_c_varargin_1[1] = c4_d4;
        c4_c_varargin_1[2] = c4_d5;
        c4_c_varargin_1[3] = c4_d6;
        c4_c_varargin_1[4] = c4_d7;
        c4_ixstart = 1;
        c4_mtmp = c4_c_varargin_1[0];
        c4_g_x = c4_mtmp;
        c4_pb_b = muDoubleScalarIsNaN(c4_g_x);
        if (c4_pb_b) {
          c4_c_eml_int_forloop_overflow_check(chartInstance, 2, 5);
          c4_ix = 2;
          exitg2 = FALSE;
          while ((exitg2 == 0U) && (c4_ix < 6)) {
            c4_b_ix = c4_ix;
            c4_ixstart = c4_b_ix;
            c4_h_x = c4_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c4_b_ix), 1, 5, 1, 0) - 1];
            c4_qb_b = muDoubleScalarIsNaN(c4_h_x);
            if (!c4_qb_b) {
              c4_mtmp = c4_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)c4_b_ix), 1, 5, 1, 0) - 1];
              exitg2 = TRUE;
            } else {
              c4_ix++;
            }
          }
        }

        if (c4_ixstart < 5) {
          c4_wb_a = c4_ixstart + 1;
          c4_i59 = c4_wb_a;
          c4_c_eml_int_forloop_overflow_check(chartInstance, c4_i59, 5);
          for (c4_c_ix = c4_i59; c4_c_ix < 6; c4_c_ix++) {
            c4_b_ix = c4_c_ix;
            c4_xb_a = c4_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c4_b_ix), 1, 5, 1, 0) - 1];
            c4_rb_b = c4_mtmp;
            c4_p = (c4_xb_a > c4_rb_b);
            if (c4_p) {
              c4_mtmp = c4_c_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
                _SFD_INTEGER_CHECK("", (real_T)c4_b_ix), 1, 5, 1, 0) - 1];
            }
          }
        }

        c4_b_mtmp = c4_mtmp;
        c4_scale = c4_b_mtmp;
        c4_sm = c4_eml_div(chartInstance, c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_m), 1, 6, 1, 0) - 1],
                           c4_scale);
        c4_smm1 = c4_eml_div(chartInstance, c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_mm1), 1,
          6, 1, 0) - 1], c4_scale);
        c4_emm1 = c4_eml_div(chartInstance, c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_mm1), 1,
          6, 1, 0) - 1], c4_scale);
        c4_sqds = c4_eml_div(chartInstance, c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_q), 1,
          6, 1, 0) - 1], c4_scale);
        c4_eqds = c4_eml_div(chartInstance, c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
                              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_q), 1,
          6, 1, 0) - 1], c4_scale);
        c4_yb_a = c4_smm1 + c4_sm;
        c4_sb_b = c4_smm1 - c4_sm;
        c4_o_y = c4_yb_a * c4_sb_b;
        c4_ac_a = c4_emm1;
        c4_tb_b = c4_emm1;
        c4_p_y = c4_ac_a * c4_tb_b;
        c4_ub_b = c4_eml_div(chartInstance, c4_o_y + c4_p_y, 2.0);
        c4_bc_a = c4_sm;
        c4_vb_b = c4_emm1;
        c4_jb_c = c4_bc_a * c4_vb_b;
        c4_cc_a = c4_jb_c;
        c4_wb_b = c4_jb_c;
        c4_jb_c = c4_cc_a * c4_wb_b;
        c4_shift = 0.0;
        guard1 = FALSE;
        if (c4_ub_b != 0.0) {
          guard1 = TRUE;
        } else {
          if (c4_jb_c != 0.0) {
            guard1 = TRUE;
          }
        }

        if (guard1 == TRUE) {
          c4_dc_a = c4_ub_b;
          c4_xb_b = c4_ub_b;
          c4_q_y = c4_dc_a * c4_xb_b;
          c4_shift = c4_q_y + c4_jb_c;
          c4_b_sqrt(chartInstance, &c4_shift);
          if (c4_ub_b < 0.0) {
            c4_shift = -c4_shift;
          }

          c4_shift = c4_eml_div(chartInstance, c4_jb_c, c4_ub_b + c4_shift);
        }

        c4_ec_a = c4_sqds + c4_sm;
        c4_yb_b = c4_sqds - c4_sm;
        c4_r_y = c4_ec_a * c4_yb_b;
        c4_f = c4_r_y + c4_shift;
        c4_fc_a = c4_sqds;
        c4_ac_b = c4_eqds;
        c4_g = c4_fc_a * c4_ac_b;
        c4_i_q = c4_b_q;
        c4_c_eml_int_forloop_overflow_check(chartInstance, c4_i_q, c4_mm1);
        for (c4_d_k = c4_i_q; c4_d_k <= c4_mm1; c4_d_k++) {
          c4_b_k = c4_d_k;
          c4_gc_a = c4_b_k - 1;
          c4_km1 = c4_gc_a;
          c4_hc_a = c4_b_k + 1;
          c4_kp1 = c4_hc_a;
          c4_c_f = c4_f;
          c4_unusedU1 = c4_g;
          c4_b_eml_xrotg(chartInstance, &c4_c_f, &c4_unusedU1, &c4_d_cs,
                         &c4_d_sn);
          c4_f = c4_c_f;
          c4_b_cs = c4_d_cs;
          c4_b_sn = c4_d_sn;
          if (c4_b_k > c4_b_q) {
            c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c4_km1), 1, 6, 1, 0) - 1] = c4_f;
          }

          c4_ic_a = c4_b_cs;
          c4_bc_b = c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_b_k), 1, 6, 1, 0) - 1];
          c4_s_y = c4_ic_a * c4_bc_b;
          c4_jc_a = c4_b_sn;
          c4_cc_b = c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_b_k), 1, 6, 1, 0) - 1];
          c4_t_y = c4_jc_a * c4_cc_b;
          c4_f = c4_s_y + c4_t_y;
          c4_kc_a = c4_b_cs;
          c4_dc_b = c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_b_k), 1, 6, 1, 0) - 1];
          c4_u_y = c4_kc_a * c4_dc_b;
          c4_lc_a = c4_b_sn;
          c4_ec_b = c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_b_k), 1, 6, 1, 0) - 1];
          c4_v_y = c4_lc_a * c4_ec_b;
          c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_k), 1, 6, 1, 0) - 1] = c4_u_y - c4_v_y;
          c4_mc_a = c4_b_sn;
          c4_fc_b = c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_kp1), 1, 6, 1, 0) - 1];
          c4_g = c4_mc_a * c4_fc_b;
          c4_nc_a = c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_kp1), 1, 6, 1, 0) - 1];
          c4_gc_b = c4_b_cs;
          c4_w_y = c4_nc_a * c4_gc_b;
          c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_kp1), 1, 6, 1, 0) - 1] = c4_w_y;
          c4_oc_a = c4_b_k - 1;
          c4_kb_c = c4_oc_a;
          c4_hc_b = c4_kb_c;
          c4_lb_c = 6 * c4_hc_b;
          c4_ic_b = c4_lb_c + 1;
          c4_colk = c4_ic_b;
          c4_jc_b = c4_b_k;
          c4_mb_c = 6 * c4_jc_b;
          c4_kc_b = c4_mb_c + 1;
          c4_colkp1 = c4_kc_b;
          c4_c_eml_xrot(chartInstance, c4_Vf, c4_colk, c4_colkp1, c4_b_cs,
                        c4_b_sn);
          c4_d_f = c4_f;
          c4_unusedU2 = c4_g;
          c4_b_eml_xrotg(chartInstance, &c4_d_f, &c4_unusedU2, &c4_e_cs,
                         &c4_e_sn);
          c4_f = c4_d_f;
          c4_b_cs = c4_e_cs;
          c4_b_sn = c4_e_sn;
          c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_k), 1, 6, 1, 0) - 1] = c4_f;
          c4_pc_a = c4_b_cs;
          c4_lc_b = c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_b_k), 1, 6, 1, 0) - 1];
          c4_x_y = c4_pc_a * c4_lc_b;
          c4_qc_a = c4_b_sn;
          c4_mc_b = c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_kp1), 1, 6, 1, 0) - 1];
          c4_y_y = c4_qc_a * c4_mc_b;
          c4_f = c4_x_y + c4_y_y;
          c4_rc_a = -c4_b_sn;
          c4_nc_b = c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_b_k), 1, 6, 1, 0) - 1];
          c4_ab_y = c4_rc_a * c4_nc_b;
          c4_sc_a = c4_b_cs;
          c4_oc_b = c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_kp1), 1, 6, 1, 0) - 1];
          c4_bb_y = c4_sc_a * c4_oc_b;
          c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_kp1), 1, 6, 1, 0) - 1] = c4_ab_y + c4_bb_y;
          c4_tc_a = c4_b_sn;
          c4_pc_b = c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_kp1), 1, 6, 1, 0) - 1];
          c4_g = c4_tc_a * c4_pc_b;
          c4_uc_a = c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
            _SFD_INTEGER_CHECK("", (real_T)c4_kp1), 1, 6, 1, 0) - 1];
          c4_qc_b = c4_b_cs;
          c4_cb_y = c4_uc_a * c4_qc_b;
          c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_kp1), 1, 6, 1, 0) - 1] = c4_cb_y;
          if (c4_b_k < 12) {
            c4_vc_a = c4_b_k - 1;
            c4_nb_c = c4_vc_a;
            c4_rc_b = c4_nb_c;
            c4_ob_c = 12 * c4_rc_b;
            c4_sc_b = c4_ob_c + 1;
            c4_colk = c4_sc_b;
            c4_tc_b = c4_b_k;
            c4_pb_c = 12 * c4_tc_b;
            c4_uc_b = c4_pb_c + 1;
            c4_colkp1 = c4_uc_b;
            c4_d_eml_xrot(chartInstance, c4_U, c4_colk, c4_colkp1, c4_b_cs,
                          c4_b_sn);
          }
        }

        c4_wc_a = c4_m - 1;
        c4_qb_c = c4_wc_a;
        c4_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c4_qb_c), 1, 6, 1, 0) - 1] = c4_f;
        c4_iter++;
        break;

       default:
        if (c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c4_b_q), 1, 6, 1, 0) - 1] < 0.0) {
          c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_q), 1, 6, 1, 0) - 1] =
            -c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_q), 1, 6, 1, 0) - 1];
          c4_xc_a = c4_b_q - 1;
          c4_rb_c = c4_xc_a;
          c4_vc_b = c4_rb_c;
          c4_sb_c = 6 * c4_vc_b;
          c4_wc_b = c4_sb_c + 1;
          c4_colq = c4_wc_b;
          c4_b_eml_scalar_eg(chartInstance);
          c4_d8 = -1.0;
          c4_f_eml_xscal(chartInstance, c4_d8, c4_Vf, c4_colq);
        }

        c4_yc_a = c4_b_q + 1;
        c4_qp1 = c4_yc_a;
        exitg3 = FALSE;
        while ((exitg3 == 0U) && (c4_b_q < 6)) {
          if (c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
                "", (real_T)c4_b_q), 1, 6, 1, 0) - 1] <
              c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
                "", (real_T)c4_qp1), 1, 6, 1, 0) - 1]) {
            c4_rt = c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
              _SFD_INTEGER_CHECK("", (real_T)c4_b_q), 1, 6, 1, 0) - 1];
            c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c4_b_q), 1, 6, 1, 0) - 1] =
              c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
              "", (real_T)c4_qp1), 1, 6, 1, 0) - 1];
            c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c4_qp1), 1, 6, 1, 0) - 1] = c4_rt;
            if (c4_b_q < 6) {
              c4_bd_a = c4_b_q - 1;
              c4_tb_c = c4_bd_a;
              c4_xc_b = c4_tb_c;
              c4_ub_c = 6 * c4_xc_b;
              c4_yc_b = c4_ub_c + 1;
              c4_colq = c4_yc_b;
              c4_ad_b = c4_b_q;
              c4_vb_c = 6 * c4_ad_b;
              c4_bd_b = c4_vb_c + 1;
              c4_colqp1 = c4_bd_b;
              c4_c_eml_xswap(chartInstance, c4_Vf, c4_colq, c4_colqp1);
            }

            if (c4_b_q < 12) {
              c4_cd_a = c4_b_q - 1;
              c4_wb_c = c4_cd_a;
              c4_cd_b = c4_wb_c;
              c4_xb_c = 12 * c4_cd_b;
              c4_dd_b = c4_xb_c + 1;
              c4_colq = c4_dd_b;
              c4_ed_b = c4_b_q;
              c4_yb_c = 12 * c4_ed_b;
              c4_fd_b = c4_yb_c + 1;
              c4_colqp1 = c4_fd_b;
              c4_d_eml_xswap(chartInstance, c4_U, c4_colq, c4_colqp1);
            }

            c4_b_q = c4_qp1;
            c4_dd_a = c4_b_q + 1;
            c4_qp1 = c4_dd_a;
          } else {
            exitg3 = TRUE;
          }
        }

        c4_iter = 0.0;
        c4_ad_a = c4_m - 1;
        c4_m = c4_ad_a;
        break;
      }
    }
  }

  c4_b_eml_int_forloop_overflow_check(chartInstance);
  for (c4_e_k = 1; c4_e_k < 7; c4_e_k++) {
    c4_b_k = c4_e_k;
    c4_S[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c4_b_k), 1, 6, 1, 0) - 1] = c4_s[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c4_b_k), 1, 6, 1, 0) - 1];
  }

  c4_b_eml_int_forloop_overflow_check(chartInstance);
  for (c4_j = 1; c4_j < 7; c4_j++) {
    c4_b_j = c4_j;
    c4_b_eml_int_forloop_overflow_check(chartInstance);
    for (c4_i = 1; c4_i < 7; c4_i++) {
      c4_b_i = c4_i;
      c4_V[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c4_b_i), 1, 6, 1, 0) + 6 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_j), 1, 6, 2, 0) - 1))
        - 1] = c4_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c4_b_i), 1, 6, 1, 0) + 6 *
                      (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c4_b_j), 1, 6, 2, 0) - 1)) - 1];
    }
  }
}

static void c4_b_eml_int_forloop_overflow_check(SFc4_kinematicsInstanceStruct
  *chartInstance)
{
}

static real_T c4_eml_xnrm2(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_x[72], int32_T c4_ix0)
{
  real_T c4_y;
  int32_T c4_b_n;
  int32_T c4_b_ix0;
  int32_T c4_c_n;
  int32_T c4_c_ix0;
  real_T c4_b_x;
  real_T c4_c_x;
  real_T c4_scale;
  int32_T c4_kstart;
  int32_T c4_a;
  int32_T c4_c;
  int32_T c4_b_a;
  int32_T c4_b_c;
  int32_T c4_c_a;
  int32_T c4_b;
  int32_T c4_kend;
  int32_T c4_b_kstart;
  int32_T c4_k;
  int32_T c4_b_k;
  real_T c4_d_x;
  real_T c4_e_x;
  real_T c4_absxk;
  real_T c4_t;
  c4_b_n = c4_n;
  c4_b_ix0 = c4_ix0;
  c4_c_n = c4_b_n;
  c4_c_ix0 = c4_b_ix0;
  c4_y = 0.0;
  if ((real_T)c4_c_n < 1.0) {
  } else if ((real_T)c4_c_n == 1.0) {
    c4_b_x = c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c4_c_ix0), 1, 72, 1, 0) - 1];
    c4_c_x = c4_b_x;
    c4_y = muDoubleScalarAbs(c4_c_x);
  } else {
    c4_realmin(chartInstance);
    c4_scale = 2.2250738585072014E-308;
    c4_kstart = c4_c_ix0;
    c4_a = c4_c_n;
    c4_c = c4_a;
    c4_b_a = c4_c - 1;
    c4_b_c = c4_b_a;
    c4_c_a = c4_kstart;
    c4_b = c4_b_c;
    c4_kend = c4_c_a + c4_b;
    c4_b_kstart = c4_kstart;
    c4_c_eml_int_forloop_overflow_check(chartInstance, c4_b_kstart, c4_kend);
    for (c4_k = c4_b_kstart; c4_k <= c4_kend; c4_k++) {
      c4_b_k = c4_k;
      c4_d_x = c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c4_b_k), 1, 72, 1, 0) - 1];
      c4_e_x = c4_d_x;
      c4_absxk = muDoubleScalarAbs(c4_e_x);
      if (c4_absxk > c4_scale) {
        c4_t = c4_scale / c4_absxk;
        c4_y = 1.0 + c4_y * c4_t * c4_t;
        c4_scale = c4_absxk;
      } else {
        c4_t = c4_absxk / c4_scale;
        c4_y += c4_t * c4_t;
      }
    }

    c4_y = c4_scale * muDoubleScalarSqrt(c4_y);
  }

  return c4_y;
}

static real_T c4_abs(SFc4_kinematicsInstanceStruct *chartInstance, real_T c4_x)
{
  real_T c4_b_x;
  c4_b_x = c4_x;
  return muDoubleScalarAbs(c4_b_x);
}

static void c4_realmin(SFc4_kinematicsInstanceStruct *chartInstance)
{
}

static void c4_c_eml_int_forloop_overflow_check(SFc4_kinematicsInstanceStruct
  *chartInstance, int32_T c4_a, int32_T c4_b)
{
  int32_T c4_b_a;
  int32_T c4_b_b;
  boolean_T c4_overflow;
  boolean_T c4_safe;
  int32_T c4_i60;
  static char_T c4_cv0[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c4_u[34];
  const mxArray *c4_y = NULL;
  int32_T c4_i61;
  static char_T c4_cv1[5] = { 'i', 'n', 't', '3', '2' };

  char_T c4_b_u[5];
  const mxArray *c4_b_y = NULL;
  c4_b_a = c4_a;
  c4_b_b = c4_b;
  if (c4_b_a > c4_b_b) {
    c4_overflow = FALSE;
  } else {
    c4_overflow = (c4_b_b > 2147483646);
  }

  c4_safe = !c4_overflow;
  if (c4_safe) {
  } else {
    for (c4_i60 = 0; c4_i60 < 34; c4_i60++) {
      c4_u[c4_i60] = c4_cv0[c4_i60];
    }

    c4_y = NULL;
    sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c4_i61 = 0; c4_i61 < 5; c4_i61++) {
      c4_b_u[c4_i61] = c4_cv1[c4_i61];
    }

    c4_b_y = NULL;
    sf_mex_assign(&c4_b_y, sf_mex_create("y", c4_b_u, 10, 0U, 1U, 0U, 2, 1, 5),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c4_y, 14, c4_b_y));
  }
}

static real_T c4_eml_div(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_x, real_T c4_y)
{
  return c4_x / c4_y;
}

static void c4_eml_xscal(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, real_T c4_x[72], int32_T c4_ix0, real_T c4_b_x[72])
{
  int32_T c4_i62;
  for (c4_i62 = 0; c4_i62 < 72; c4_i62++) {
    c4_b_x[c4_i62] = c4_x[c4_i62];
  }

  c4_d_eml_xscal(chartInstance, c4_n, c4_a, c4_b_x, c4_ix0);
}

static real_T c4_eml_xdotc(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_x[72], int32_T c4_ix0, real_T c4_y[72], int32_T c4_iy0)
{
  real_T c4_d;
  int32_T c4_b_n;
  int32_T c4_b_ix0;
  int32_T c4_b_iy0;
  int32_T c4_c_n;
  int32_T c4_c_ix0;
  int32_T c4_c_iy0;
  int32_T c4_d_n;
  int32_T c4_d_ix0;
  int32_T c4_d_iy0;
  int32_T c4_e_n;
  int32_T c4_e_ix0;
  int32_T c4_e_iy0;
  int32_T c4_ix;
  int32_T c4_iy;
  int32_T c4_loop_ub;
  int32_T c4_k;
  int32_T c4_a;
  int32_T c4_b_a;
  c4_b_n = c4_n;
  c4_b_ix0 = c4_ix0;
  c4_b_iy0 = c4_iy0;
  c4_c_n = c4_b_n;
  c4_c_ix0 = c4_b_ix0;
  c4_c_iy0 = c4_b_iy0;
  c4_d_n = c4_c_n;
  c4_d_ix0 = c4_c_ix0;
  c4_d_iy0 = c4_c_iy0;
  c4_e_n = c4_d_n;
  c4_e_ix0 = c4_d_ix0;
  c4_e_iy0 = c4_d_iy0;
  c4_d = 0.0;
  if ((real_T)c4_e_n < 1.0) {
  } else {
    c4_ix = c4_e_ix0;
    c4_iy = c4_e_iy0;
    c4_c_eml_int_forloop_overflow_check(chartInstance, 1, c4_e_n);
    c4_loop_ub = c4_e_n;
    for (c4_k = 1; c4_k <= c4_loop_ub; c4_k++) {
      c4_d += c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c4_ix), 1, 72, 1, 0) - 1] * c4_y[_SFD_EML_ARRAY_BOUNDS_CHECK
        ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_iy), 1, 72, 1, 0) - 1];
      c4_a = c4_ix + 1;
      c4_ix = c4_a;
      c4_b_a = c4_iy + 1;
      c4_iy = c4_b_a;
    }
  }

  return c4_d;
}

static void c4_eml_xaxpy(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, int32_T c4_ix0, real_T c4_y[72], int32_T c4_iy0, real_T
  c4_b_y[72])
{
  int32_T c4_i63;
  for (c4_i63 = 0; c4_i63 < 72; c4_i63++) {
    c4_b_y[c4_i63] = c4_y[c4_i63];
  }

  c4_e_eml_xaxpy(chartInstance, c4_n, c4_a, c4_ix0, c4_b_y, c4_iy0);
}

static real_T c4_b_eml_xnrm2(SFc4_kinematicsInstanceStruct *chartInstance,
  int32_T c4_n, real_T c4_x[6], int32_T c4_ix0)
{
  real_T c4_y;
  int32_T c4_b_n;
  int32_T c4_b_ix0;
  int32_T c4_c_n;
  int32_T c4_c_ix0;
  real_T c4_b_x;
  real_T c4_c_x;
  real_T c4_scale;
  int32_T c4_kstart;
  int32_T c4_a;
  int32_T c4_c;
  int32_T c4_b_a;
  int32_T c4_b_c;
  int32_T c4_c_a;
  int32_T c4_b;
  int32_T c4_kend;
  int32_T c4_b_kstart;
  int32_T c4_k;
  int32_T c4_b_k;
  real_T c4_d_x;
  real_T c4_e_x;
  real_T c4_absxk;
  real_T c4_t;
  c4_b_n = c4_n;
  c4_b_ix0 = c4_ix0;
  c4_c_n = c4_b_n;
  c4_c_ix0 = c4_b_ix0;
  c4_y = 0.0;
  if ((real_T)c4_c_n < 1.0) {
  } else if ((real_T)c4_c_n == 1.0) {
    c4_b_x = c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c4_c_ix0), 1, 6, 1, 0) - 1];
    c4_c_x = c4_b_x;
    c4_y = muDoubleScalarAbs(c4_c_x);
  } else {
    c4_realmin(chartInstance);
    c4_scale = 2.2250738585072014E-308;
    c4_kstart = c4_c_ix0;
    c4_a = c4_c_n;
    c4_c = c4_a;
    c4_b_a = c4_c - 1;
    c4_b_c = c4_b_a;
    c4_c_a = c4_kstart;
    c4_b = c4_b_c;
    c4_kend = c4_c_a + c4_b;
    c4_b_kstart = c4_kstart;
    c4_c_eml_int_forloop_overflow_check(chartInstance, c4_b_kstart, c4_kend);
    for (c4_k = c4_b_kstart; c4_k <= c4_kend; c4_k++) {
      c4_b_k = c4_k;
      c4_d_x = c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c4_b_k), 1, 6, 1, 0) - 1];
      c4_e_x = c4_d_x;
      c4_absxk = muDoubleScalarAbs(c4_e_x);
      if (c4_absxk > c4_scale) {
        c4_t = c4_scale / c4_absxk;
        c4_y = 1.0 + c4_y * c4_t * c4_t;
        c4_scale = c4_absxk;
      } else {
        c4_t = c4_absxk / c4_scale;
        c4_y += c4_t * c4_t;
      }
    }

    c4_y = c4_scale * muDoubleScalarSqrt(c4_y);
  }

  return c4_y;
}

static void c4_b_eml_xscal(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, real_T c4_x[6], int32_T c4_ix0, real_T c4_b_x[6])
{
  int32_T c4_i64;
  for (c4_i64 = 0; c4_i64 < 6; c4_i64++) {
    c4_b_x[c4_i64] = c4_x[c4_i64];
  }

  c4_e_eml_xscal(chartInstance, c4_n, c4_a, c4_b_x, c4_ix0);
}

static void c4_b_eml_xaxpy(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, real_T c4_x[72], int32_T c4_ix0, real_T c4_y[12], int32_T
  c4_iy0, real_T c4_b_y[12])
{
  int32_T c4_i65;
  int32_T c4_i66;
  real_T c4_b_x[72];
  for (c4_i65 = 0; c4_i65 < 12; c4_i65++) {
    c4_b_y[c4_i65] = c4_y[c4_i65];
  }

  for (c4_i66 = 0; c4_i66 < 72; c4_i66++) {
    c4_b_x[c4_i66] = c4_x[c4_i66];
  }

  c4_f_eml_xaxpy(chartInstance, c4_n, c4_a, c4_b_x, c4_ix0, c4_b_y, c4_iy0);
}

static void c4_c_eml_xaxpy(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, real_T c4_x[12], int32_T c4_ix0, real_T c4_y[72], int32_T
  c4_iy0, real_T c4_b_y[72])
{
  int32_T c4_i67;
  int32_T c4_i68;
  real_T c4_b_x[12];
  for (c4_i67 = 0; c4_i67 < 72; c4_i67++) {
    c4_b_y[c4_i67] = c4_y[c4_i67];
  }

  for (c4_i68 = 0; c4_i68 < 12; c4_i68++) {
    c4_b_x[c4_i68] = c4_x[c4_i68];
  }

  c4_g_eml_xaxpy(chartInstance, c4_n, c4_a, c4_b_x, c4_ix0, c4_b_y, c4_iy0);
}

static void c4_d_eml_int_forloop_overflow_check(SFc4_kinematicsInstanceStruct
  *chartInstance)
{
}

static real_T c4_b_eml_xdotc(SFc4_kinematicsInstanceStruct *chartInstance,
  int32_T c4_n, real_T c4_x[36], int32_T c4_ix0, real_T c4_y[36], int32_T c4_iy0)
{
  real_T c4_d;
  int32_T c4_b_n;
  int32_T c4_b_ix0;
  int32_T c4_b_iy0;
  int32_T c4_c_n;
  int32_T c4_c_ix0;
  int32_T c4_c_iy0;
  int32_T c4_d_n;
  int32_T c4_d_ix0;
  int32_T c4_d_iy0;
  int32_T c4_e_n;
  int32_T c4_e_ix0;
  int32_T c4_e_iy0;
  int32_T c4_ix;
  int32_T c4_iy;
  int32_T c4_loop_ub;
  int32_T c4_k;
  int32_T c4_a;
  int32_T c4_b_a;
  c4_b_n = c4_n;
  c4_b_ix0 = c4_ix0;
  c4_b_iy0 = c4_iy0;
  c4_c_n = c4_b_n;
  c4_c_ix0 = c4_b_ix0;
  c4_c_iy0 = c4_b_iy0;
  c4_d_n = c4_c_n;
  c4_d_ix0 = c4_c_ix0;
  c4_d_iy0 = c4_c_iy0;
  c4_e_n = c4_d_n;
  c4_e_ix0 = c4_d_ix0;
  c4_e_iy0 = c4_d_iy0;
  c4_d = 0.0;
  if ((real_T)c4_e_n < 1.0) {
  } else {
    c4_ix = c4_e_ix0;
    c4_iy = c4_e_iy0;
    c4_c_eml_int_forloop_overflow_check(chartInstance, 1, c4_e_n);
    c4_loop_ub = c4_e_n;
    for (c4_k = 1; c4_k <= c4_loop_ub; c4_k++) {
      c4_d += c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
        "", (real_T)c4_ix), 1, 36, 1, 0) - 1] * c4_y[_SFD_EML_ARRAY_BOUNDS_CHECK
        ("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_iy), 1, 36, 1, 0) - 1];
      c4_a = c4_ix + 1;
      c4_ix = c4_a;
      c4_b_a = c4_iy + 1;
      c4_iy = c4_b_a;
    }
  }

  return c4_d;
}

static void c4_d_eml_xaxpy(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, int32_T c4_ix0, real_T c4_y[36], int32_T c4_iy0, real_T
  c4_b_y[36])
{
  int32_T c4_i69;
  for (c4_i69 = 0; c4_i69 < 36; c4_i69++) {
    c4_b_y[c4_i69] = c4_y[c4_i69];
  }

  c4_h_eml_xaxpy(chartInstance, c4_n, c4_a, c4_ix0, c4_b_y, c4_iy0);
}

static void c4_b_eml_scalar_eg(SFc4_kinematicsInstanceStruct *chartInstance)
{
}

static void c4_c_eml_xscal(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_a, real_T c4_x[36], int32_T c4_ix0, real_T c4_b_x[36])
{
  int32_T c4_i70;
  for (c4_i70 = 0; c4_i70 < 36; c4_i70++) {
    c4_b_x[c4_i70] = c4_x[c4_i70];
  }

  c4_f_eml_xscal(chartInstance, c4_a, c4_b_x, c4_ix0);
}

static void c4_eps(SFc4_kinematicsInstanceStruct *chartInstance)
{
}

static void c4_c_eml_scalar_eg(SFc4_kinematicsInstanceStruct *chartInstance)
{
}

static void c4_b_eml_error(SFc4_kinematicsInstanceStruct *chartInstance)
{
  int32_T c4_i71;
  static char_T c4_varargin_1[30] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'v', 'd', '_', 'N', 'o', 'C', 'o', 'n', 'v',
    'e', 'r', 'g', 'e', 'n', 'c', 'e' };

  char_T c4_u[30];
  const mxArray *c4_y = NULL;
  for (c4_i71 = 0; c4_i71 < 30; c4_i71++) {
    c4_u[c4_i71] = c4_varargin_1[c4_i71];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c4_y));
}

static void c4_e_eml_int_forloop_overflow_check(SFc4_kinematicsInstanceStruct
  *chartInstance, int32_T c4_a)
{
  boolean_T c4_overflow;
  boolean_T c4_safe;
  int32_T c4_i72;
  static char_T c4_cv2[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c4_u[34];
  const mxArray *c4_y = NULL;
  int32_T c4_i73;
  static char_T c4_cv3[5] = { 'i', 'n', 't', '3', '2' };

  char_T c4_b_u[5];
  const mxArray *c4_b_y = NULL;
  c4_overflow = FALSE;
  c4_safe = !c4_overflow;
  if (c4_safe) {
  } else {
    for (c4_i72 = 0; c4_i72 < 34; c4_i72++) {
      c4_u[c4_i72] = c4_cv2[c4_i72];
    }

    c4_y = NULL;
    sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c4_i73 = 0; c4_i73 < 5; c4_i73++) {
      c4_b_u[c4_i73] = c4_cv3[c4_i73];
    }

    c4_b_y = NULL;
    sf_mex_assign(&c4_b_y, sf_mex_create("y", c4_b_u, 10, 0U, 1U, 0U, 2, 1, 5),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c4_y, 14, c4_b_y));
  }
}

static void c4_f_eml_int_forloop_overflow_check(SFc4_kinematicsInstanceStruct
  *chartInstance, int32_T c4_a, int32_T c4_b)
{
  int32_T c4_b_a;
  int32_T c4_b_b;
  boolean_T c4_overflow;
  boolean_T c4_safe;
  int32_T c4_i74;
  static char_T c4_cv4[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c4_u[34];
  const mxArray *c4_y = NULL;
  int32_T c4_i75;
  static char_T c4_cv5[5] = { 'i', 'n', 't', '3', '2' };

  char_T c4_b_u[5];
  const mxArray *c4_b_y = NULL;
  c4_b_a = c4_a;
  c4_b_b = c4_b;
  if (c4_b_a < c4_b_b) {
    c4_overflow = FALSE;
  } else {
    c4_overflow = (c4_b_b < -2147483647);
  }

  c4_safe = !c4_overflow;
  if (c4_safe) {
  } else {
    for (c4_i74 = 0; c4_i74 < 34; c4_i74++) {
      c4_u[c4_i74] = c4_cv4[c4_i74];
    }

    c4_y = NULL;
    sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c4_i75 = 0; c4_i75 < 5; c4_i75++) {
      c4_b_u[c4_i75] = c4_cv5[c4_i75];
    }

    c4_b_y = NULL;
    sf_mex_assign(&c4_b_y, sf_mex_create("y", c4_b_u, 10, 0U, 1U, 0U, 2, 1, 5),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c4_y, 14, c4_b_y));
  }
}

static real_T c4_sqrt(SFc4_kinematicsInstanceStruct *chartInstance, real_T c4_x)
{
  real_T c4_b_x;
  c4_b_x = c4_x;
  c4_b_sqrt(chartInstance, &c4_b_x);
  return c4_b_x;
}

static void c4_c_eml_error(SFc4_kinematicsInstanceStruct *chartInstance)
{
  int32_T c4_i76;
  static char_T c4_varargin_1[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o',
    'o', 'l', 'b', 'o', 'x', ':', 's', 'q', 'r', 't', '_', 'd', 'o', 'm', 'a',
    'i', 'n', 'E', 'r', 'r', 'o', 'r' };

  char_T c4_u[30];
  const mxArray *c4_y = NULL;
  for (c4_i76 = 0; c4_i76 < 30; c4_i76++) {
    c4_u[c4_i76] = c4_varargin_1[c4_i76];
  }

  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c4_y));
}

static void c4_eml_xrotg(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_a, real_T c4_b, real_T *c4_b_a, real_T *c4_b_b, real_T *c4_c, real_T *c4_s)
{
  *c4_b_a = c4_a;
  *c4_b_b = c4_b;
  c4_b_eml_xrotg(chartInstance, c4_b_a, c4_b_b, c4_c, c4_s);
}

static void c4_eml_xrot(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_x[36], int32_T c4_ix0, int32_T c4_iy0, real_T c4_c, real_T c4_s, real_T
  c4_b_x[36])
{
  int32_T c4_i77;
  for (c4_i77 = 0; c4_i77 < 36; c4_i77++) {
    c4_b_x[c4_i77] = c4_x[c4_i77];
  }

  c4_c_eml_xrot(chartInstance, c4_b_x, c4_ix0, c4_iy0, c4_c, c4_s);
}

static void c4_b_eml_xrot(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_x[72], int32_T c4_ix0, int32_T c4_iy0, real_T c4_c, real_T c4_s, real_T
  c4_b_x[72])
{
  int32_T c4_i78;
  for (c4_i78 = 0; c4_i78 < 72; c4_i78++) {
    c4_b_x[c4_i78] = c4_x[c4_i78];
  }

  c4_d_eml_xrot(chartInstance, c4_b_x, c4_ix0, c4_iy0, c4_c, c4_s);
}

static void c4_eml_xswap(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_x[36], int32_T c4_ix0, int32_T c4_iy0, real_T c4_b_x[36])
{
  int32_T c4_i79;
  for (c4_i79 = 0; c4_i79 < 36; c4_i79++) {
    c4_b_x[c4_i79] = c4_x[c4_i79];
  }

  c4_c_eml_xswap(chartInstance, c4_b_x, c4_ix0, c4_iy0);
}

static void c4_b_eml_xswap(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_x[72], int32_T c4_ix0, int32_T c4_iy0, real_T c4_b_x[72])
{
  int32_T c4_i80;
  for (c4_i80 = 0; c4_i80 < 72; c4_i80++) {
    c4_b_x[c4_i80] = c4_x[c4_i80];
  }

  c4_d_eml_xswap(chartInstance, c4_b_x, c4_ix0, c4_iy0);
}

static void c4_g_eml_int_forloop_overflow_check(SFc4_kinematicsInstanceStruct
  *chartInstance)
{
}

static void c4_eml_xgemm(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_k, real_T c4_A[36], real_T c4_B[72], real_T c4_C[72])
{
  int32_T c4_b_k;
  int32_T c4_i81;
  int32_T c4_c_k;
  int32_T c4_a;
  int32_T c4_km1;
  int32_T c4_cr;
  int32_T c4_b_cr;
  int32_T c4_b_a;
  int32_T c4_i82;
  int32_T c4_c_a;
  int32_T c4_i83;
  int32_T c4_ic;
  int32_T c4_b_ic;
  int32_T c4_br;
  int32_T c4_c_cr;
  int32_T c4_ar;
  int32_T c4_d_a;
  int32_T c4_b_br;
  int32_T c4_b;
  int32_T c4_c;
  int32_T c4_e_a;
  int32_T c4_b_b;
  int32_T c4_i84;
  int32_T c4_ib;
  int32_T c4_b_ib;
  real_T c4_temp;
  int32_T c4_ia;
  int32_T c4_f_a;
  int32_T c4_i85;
  int32_T c4_g_a;
  int32_T c4_i86;
  int32_T c4_c_ic;
  int32_T c4_h_a;
  int32_T c4_i_a;
  c4_b_k = c4_k;
  for (c4_i81 = 0; c4_i81 < 72; c4_i81++) {
    c4_C[c4_i81] = 0.0;
  }

  c4_c_k = c4_b_k;
  c4_a = c4_c_k;
  c4_km1 = c4_a;
  c4_g_eml_int_forloop_overflow_check(chartInstance);
  for (c4_cr = 0; c4_cr < 67; c4_cr += 6) {
    c4_b_cr = c4_cr;
    c4_b_a = c4_b_cr + 1;
    c4_i82 = c4_b_a;
    c4_c_a = c4_b_cr + 6;
    c4_i83 = c4_c_a;
    c4_c_eml_int_forloop_overflow_check(chartInstance, c4_i82, c4_i83);
    for (c4_ic = c4_i82; c4_ic <= c4_i83; c4_ic++) {
      c4_b_ic = c4_ic;
      c4_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c4_b_ic), 1, 72, 1, 0) - 1] = 0.0;
    }
  }

  c4_br = 0;
  c4_g_eml_int_forloop_overflow_check(chartInstance);
  for (c4_c_cr = 0; c4_c_cr < 67; c4_c_cr += 6) {
    c4_b_cr = c4_c_cr;
    c4_ar = 0;
    c4_d_a = c4_br + 1;
    c4_br = c4_d_a;
    c4_b_br = c4_br;
    c4_b = c4_km1 - 1;
    c4_c = 12 * c4_b;
    c4_e_a = c4_br;
    c4_b_b = c4_c;
    c4_i84 = c4_e_a + c4_b_b;
    c4_h_eml_int_forloop_overflow_check(chartInstance, c4_b_br, c4_i84);
    for (c4_ib = c4_b_br; c4_ib <= c4_i84; c4_ib += 12) {
      c4_b_ib = c4_ib;
      if (c4_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_ib), 1, 72, 1, 0) - 1] != 0.0) {
        c4_temp = c4_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c4_b_ib), 1, 72, 1, 0) - 1];
        c4_ia = c4_ar;
        c4_f_a = c4_b_cr + 1;
        c4_i85 = c4_f_a;
        c4_g_a = c4_b_cr + 6;
        c4_i86 = c4_g_a;
        c4_c_eml_int_forloop_overflow_check(chartInstance, c4_i85, c4_i86);
        for (c4_c_ic = c4_i85; c4_c_ic <= c4_i86; c4_c_ic++) {
          c4_b_ic = c4_c_ic;
          c4_h_a = c4_ia + 1;
          c4_ia = c4_h_a;
          c4_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_ic), 1, 72, 1, 0) - 1] =
            c4_C[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_b_ic), 1, 72, 1, 0) - 1] + c4_temp *
            c4_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
            (real_T)c4_ia), 1, 36, 1, 0) - 1];
        }
      }

      c4_i_a = c4_ar + 6;
      c4_ar = c4_i_a;
    }
  }
}

static void c4_h_eml_int_forloop_overflow_check(SFc4_kinematicsInstanceStruct
  *chartInstance, int32_T c4_a, int32_T c4_b)
{
  int32_T c4_b_a;
  int32_T c4_b_b;
  boolean_T c4_overflow;
  boolean_T c4_safe;
  int32_T c4_i87;
  static char_T c4_cv6[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c4_u[34];
  const mxArray *c4_y = NULL;
  int32_T c4_i88;
  static char_T c4_cv7[5] = { 'i', 'n', 't', '3', '2' };

  char_T c4_b_u[5];
  const mxArray *c4_b_y = NULL;
  c4_b_a = c4_a;
  c4_b_b = c4_b;
  if (c4_b_a > c4_b_b) {
    c4_overflow = FALSE;
  } else {
    c4_overflow = (c4_b_b > 2147483635);
  }

  c4_safe = !c4_overflow;
  if (c4_safe) {
  } else {
    for (c4_i87 = 0; c4_i87 < 34; c4_i87++) {
      c4_u[c4_i87] = c4_cv6[c4_i87];
    }

    c4_y = NULL;
    sf_mex_assign(&c4_y, sf_mex_create("y", c4_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  FALSE);
    for (c4_i88 = 0; c4_i88 < 5; c4_i88++) {
      c4_b_u[c4_i88] = c4_cv7[c4_i88];
    }

    c4_b_y = NULL;
    sf_mex_assign(&c4_b_y, sf_mex_create("y", c4_b_u, 10, 0U, 1U, 0U, 2, 1, 5),
                  FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U,
      14, c4_y, 14, c4_b_y));
  }
}

static const mxArray *c4_d_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_u;
  const mxArray *c4_y = NULL;
  SFc4_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc4_kinematicsInstanceStruct *)chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(int32_T *)c4_inData;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_create("y", &c4_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c4_mxArrayOutData, c4_y, FALSE);
  return c4_mxArrayOutData;
}

static int32_T c4_d_emlrt_marshallIn(SFc4_kinematicsInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  int32_T c4_y;
  int32_T c4_i89;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_i89, 1, 6, 0U, 0, 0U, 0);
  c4_y = c4_i89;
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
  SFc4_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc4_kinematicsInstanceStruct *)chartInstanceVoid;
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

static uint8_T c4_e_emlrt_marshallIn(SFc4_kinematicsInstanceStruct
  *chartInstance, const mxArray *c4_b_is_active_c4_kinematics, const char_T
  *c4_identifier)
{
  uint8_T c4_y;
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_y = c4_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c4_b_is_active_c4_kinematics), &c4_thisId);
  sf_mex_destroy(&c4_b_is_active_c4_kinematics);
  return c4_y;
}

static uint8_T c4_f_emlrt_marshallIn(SFc4_kinematicsInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  uint8_T c4_y;
  uint8_T c4_u0;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_u0, 1, 3, 0U, 0, 0U, 0);
  c4_y = c4_u0;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void c4_d_eml_xscal(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, real_T c4_x[72], int32_T c4_ix0)
{
  int32_T c4_b_n;
  real_T c4_b_a;
  int32_T c4_b_ix0;
  int32_T c4_c_n;
  real_T c4_c_a;
  int32_T c4_c_ix0;
  int32_T c4_d_ix0;
  int32_T c4_d_a;
  int32_T c4_c;
  int32_T c4_b;
  int32_T c4_b_c;
  int32_T c4_e_a;
  int32_T c4_b_b;
  int32_T c4_i90;
  int32_T c4_k;
  int32_T c4_b_k;
  c4_b_n = c4_n;
  c4_b_a = c4_a;
  c4_b_ix0 = c4_ix0;
  c4_c_n = c4_b_n;
  c4_c_a = c4_b_a;
  c4_c_ix0 = c4_b_ix0;
  c4_d_ix0 = c4_c_ix0;
  c4_d_a = c4_c_n;
  c4_c = c4_d_a;
  c4_b = c4_c - 1;
  c4_b_c = c4_b;
  c4_e_a = c4_c_ix0;
  c4_b_b = c4_b_c;
  c4_i90 = c4_e_a + c4_b_b;
  c4_c_eml_int_forloop_overflow_check(chartInstance, c4_d_ix0, c4_i90);
  for (c4_k = c4_d_ix0; c4_k <= c4_i90; c4_k++) {
    c4_b_k = c4_k;
    c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c4_b_k), 1, 72, 1, 0) - 1] = c4_c_a * c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_k), 1, 72, 1, 0) - 1];
  }
}

static void c4_e_eml_xaxpy(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, int32_T c4_ix0, real_T c4_y[72], int32_T c4_iy0)
{
  int32_T c4_b_n;
  real_T c4_b_a;
  int32_T c4_b_ix0;
  int32_T c4_b_iy0;
  int32_T c4_c_n;
  real_T c4_c_a;
  int32_T c4_c_ix0;
  int32_T c4_c_iy0;
  int32_T c4_d_a;
  int32_T c4_ix;
  int32_T c4_e_a;
  int32_T c4_iy;
  int32_T c4_f_a;
  int32_T c4_i91;
  int32_T c4_k;
  int32_T c4_g_a;
  int32_T c4_c;
  int32_T c4_h_a;
  int32_T c4_b_c;
  int32_T c4_i_a;
  int32_T c4_c_c;
  int32_T c4_j_a;
  int32_T c4_k_a;
  c4_b_n = c4_n;
  c4_b_a = c4_a;
  c4_b_ix0 = c4_ix0;
  c4_b_iy0 = c4_iy0;
  c4_c_n = c4_b_n;
  c4_c_a = c4_b_a;
  c4_c_ix0 = c4_b_ix0;
  c4_c_iy0 = c4_b_iy0;
  if ((real_T)c4_c_n < 1.0) {
  } else if (c4_c_a == 0.0) {
  } else {
    c4_d_a = c4_c_ix0 - 1;
    c4_ix = c4_d_a;
    c4_e_a = c4_c_iy0 - 1;
    c4_iy = c4_e_a;
    c4_f_a = c4_c_n - 1;
    c4_i91 = c4_f_a;
    c4_c_eml_int_forloop_overflow_check(chartInstance, 0, c4_i91);
    for (c4_k = 0; c4_k <= c4_i91; c4_k++) {
      c4_g_a = c4_iy;
      c4_c = c4_g_a;
      c4_h_a = c4_iy;
      c4_b_c = c4_h_a;
      c4_i_a = c4_ix;
      c4_c_c = c4_i_a;
      c4_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c4_c + 1)), 1, 72, 1, 0) - 1] =
        c4_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c4_b_c + 1)), 1, 72, 1, 0) - 1] + c4_c_a *
        c4_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c4_c_c + 1)), 1, 72, 1, 0) - 1];
      c4_j_a = c4_ix + 1;
      c4_ix = c4_j_a;
      c4_k_a = c4_iy + 1;
      c4_iy = c4_k_a;
    }
  }
}

static void c4_e_eml_xscal(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, real_T c4_x[6], int32_T c4_ix0)
{
  int32_T c4_b_n;
  real_T c4_b_a;
  int32_T c4_b_ix0;
  int32_T c4_c_n;
  real_T c4_c_a;
  int32_T c4_c_ix0;
  int32_T c4_d_ix0;
  int32_T c4_d_a;
  int32_T c4_c;
  int32_T c4_b;
  int32_T c4_b_c;
  int32_T c4_e_a;
  int32_T c4_b_b;
  int32_T c4_i92;
  int32_T c4_k;
  int32_T c4_b_k;
  c4_b_n = c4_n;
  c4_b_a = c4_a;
  c4_b_ix0 = c4_ix0;
  c4_c_n = c4_b_n;
  c4_c_a = c4_b_a;
  c4_c_ix0 = c4_b_ix0;
  c4_d_ix0 = c4_c_ix0;
  c4_d_a = c4_c_n;
  c4_c = c4_d_a;
  c4_b = c4_c - 1;
  c4_b_c = c4_b;
  c4_e_a = c4_c_ix0;
  c4_b_b = c4_b_c;
  c4_i92 = c4_e_a + c4_b_b;
  c4_c_eml_int_forloop_overflow_check(chartInstance, c4_d_ix0, c4_i92);
  for (c4_k = c4_d_ix0; c4_k <= c4_i92; c4_k++) {
    c4_b_k = c4_k;
    c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c4_b_k), 1, 6, 1, 0) - 1] = c4_c_a * c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_k), 1, 6, 1, 0) - 1];
  }
}

static void c4_f_eml_xaxpy(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, real_T c4_x[72], int32_T c4_ix0, real_T c4_y[12], int32_T
  c4_iy0)
{
  int32_T c4_b_n;
  real_T c4_b_a;
  int32_T c4_b_ix0;
  int32_T c4_b_iy0;
  int32_T c4_c_n;
  real_T c4_c_a;
  int32_T c4_c_ix0;
  int32_T c4_c_iy0;
  int32_T c4_d_a;
  int32_T c4_ix;
  int32_T c4_e_a;
  int32_T c4_iy;
  int32_T c4_f_a;
  int32_T c4_i93;
  int32_T c4_k;
  int32_T c4_g_a;
  int32_T c4_c;
  int32_T c4_h_a;
  int32_T c4_b_c;
  int32_T c4_i_a;
  int32_T c4_c_c;
  int32_T c4_j_a;
  int32_T c4_k_a;
  c4_b_n = c4_n;
  c4_b_a = c4_a;
  c4_b_ix0 = c4_ix0;
  c4_b_iy0 = c4_iy0;
  c4_c_n = c4_b_n;
  c4_c_a = c4_b_a;
  c4_c_ix0 = c4_b_ix0;
  c4_c_iy0 = c4_b_iy0;
  if ((real_T)c4_c_n < 1.0) {
  } else if (c4_c_a == 0.0) {
  } else {
    c4_d_a = c4_c_ix0 - 1;
    c4_ix = c4_d_a;
    c4_e_a = c4_c_iy0 - 1;
    c4_iy = c4_e_a;
    c4_f_a = c4_c_n - 1;
    c4_i93 = c4_f_a;
    c4_c_eml_int_forloop_overflow_check(chartInstance, 0, c4_i93);
    for (c4_k = 0; c4_k <= c4_i93; c4_k++) {
      c4_g_a = c4_iy;
      c4_c = c4_g_a;
      c4_h_a = c4_iy;
      c4_b_c = c4_h_a;
      c4_i_a = c4_ix;
      c4_c_c = c4_i_a;
      c4_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c4_c + 1)), 1, 12, 1, 0) - 1] =
        c4_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c4_b_c + 1)), 1, 12, 1, 0) - 1] + c4_c_a *
        c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c4_c_c + 1)), 1, 72, 1, 0) - 1];
      c4_j_a = c4_ix + 1;
      c4_ix = c4_j_a;
      c4_k_a = c4_iy + 1;
      c4_iy = c4_k_a;
    }
  }
}

static void c4_g_eml_xaxpy(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, real_T c4_x[12], int32_T c4_ix0, real_T c4_y[72], int32_T
  c4_iy0)
{
  int32_T c4_b_n;
  real_T c4_b_a;
  int32_T c4_b_ix0;
  int32_T c4_b_iy0;
  int32_T c4_c_n;
  real_T c4_c_a;
  int32_T c4_c_ix0;
  int32_T c4_c_iy0;
  int32_T c4_d_a;
  int32_T c4_ix;
  int32_T c4_e_a;
  int32_T c4_iy;
  int32_T c4_f_a;
  int32_T c4_i94;
  int32_T c4_k;
  int32_T c4_g_a;
  int32_T c4_c;
  int32_T c4_h_a;
  int32_T c4_b_c;
  int32_T c4_i_a;
  int32_T c4_c_c;
  int32_T c4_j_a;
  int32_T c4_k_a;
  c4_b_n = c4_n;
  c4_b_a = c4_a;
  c4_b_ix0 = c4_ix0;
  c4_b_iy0 = c4_iy0;
  c4_c_n = c4_b_n;
  c4_c_a = c4_b_a;
  c4_c_ix0 = c4_b_ix0;
  c4_c_iy0 = c4_b_iy0;
  if ((real_T)c4_c_n < 1.0) {
  } else if (c4_c_a == 0.0) {
  } else {
    c4_d_a = c4_c_ix0 - 1;
    c4_ix = c4_d_a;
    c4_e_a = c4_c_iy0 - 1;
    c4_iy = c4_e_a;
    c4_f_a = c4_c_n - 1;
    c4_i94 = c4_f_a;
    c4_c_eml_int_forloop_overflow_check(chartInstance, 0, c4_i94);
    for (c4_k = 0; c4_k <= c4_i94; c4_k++) {
      c4_g_a = c4_iy;
      c4_c = c4_g_a;
      c4_h_a = c4_iy;
      c4_b_c = c4_h_a;
      c4_i_a = c4_ix;
      c4_c_c = c4_i_a;
      c4_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c4_c + 1)), 1, 72, 1, 0) - 1] =
        c4_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c4_b_c + 1)), 1, 72, 1, 0) - 1] + c4_c_a *
        c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c4_c_c + 1)), 1, 12, 1, 0) - 1];
      c4_j_a = c4_ix + 1;
      c4_ix = c4_j_a;
      c4_k_a = c4_iy + 1;
      c4_iy = c4_k_a;
    }
  }
}

static void c4_h_eml_xaxpy(SFc4_kinematicsInstanceStruct *chartInstance, int32_T
  c4_n, real_T c4_a, int32_T c4_ix0, real_T c4_y[36], int32_T c4_iy0)
{
  int32_T c4_b_n;
  real_T c4_b_a;
  int32_T c4_b_ix0;
  int32_T c4_b_iy0;
  int32_T c4_c_n;
  real_T c4_c_a;
  int32_T c4_c_ix0;
  int32_T c4_c_iy0;
  int32_T c4_d_a;
  int32_T c4_ix;
  int32_T c4_e_a;
  int32_T c4_iy;
  int32_T c4_f_a;
  int32_T c4_i95;
  int32_T c4_k;
  int32_T c4_g_a;
  int32_T c4_c;
  int32_T c4_h_a;
  int32_T c4_b_c;
  int32_T c4_i_a;
  int32_T c4_c_c;
  int32_T c4_j_a;
  int32_T c4_k_a;
  c4_b_n = c4_n;
  c4_b_a = c4_a;
  c4_b_ix0 = c4_ix0;
  c4_b_iy0 = c4_iy0;
  c4_c_n = c4_b_n;
  c4_c_a = c4_b_a;
  c4_c_ix0 = c4_b_ix0;
  c4_c_iy0 = c4_b_iy0;
  if ((real_T)c4_c_n < 1.0) {
  } else if (c4_c_a == 0.0) {
  } else {
    c4_d_a = c4_c_ix0 - 1;
    c4_ix = c4_d_a;
    c4_e_a = c4_c_iy0 - 1;
    c4_iy = c4_e_a;
    c4_f_a = c4_c_n - 1;
    c4_i95 = c4_f_a;
    c4_c_eml_int_forloop_overflow_check(chartInstance, 0, c4_i95);
    for (c4_k = 0; c4_k <= c4_i95; c4_k++) {
      c4_g_a = c4_iy;
      c4_c = c4_g_a;
      c4_h_a = c4_iy;
      c4_b_c = c4_h_a;
      c4_i_a = c4_ix;
      c4_c_c = c4_i_a;
      c4_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c4_c + 1)), 1, 36, 1, 0) - 1] =
        c4_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c4_b_c + 1)), 1, 36, 1, 0) - 1] + c4_c_a *
        c4_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)(c4_c_c + 1)), 1, 36, 1, 0) - 1];
      c4_j_a = c4_ix + 1;
      c4_ix = c4_j_a;
      c4_k_a = c4_iy + 1;
      c4_iy = c4_k_a;
    }
  }
}

static void c4_f_eml_xscal(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_a, real_T c4_x[36], int32_T c4_ix0)
{
  real_T c4_b_a;
  int32_T c4_b_ix0;
  real_T c4_c_a;
  int32_T c4_c_ix0;
  int32_T c4_d_ix0;
  int32_T c4_d_a;
  int32_T c4_i96;
  int32_T c4_k;
  int32_T c4_b_k;
  c4_b_a = c4_a;
  c4_b_ix0 = c4_ix0;
  c4_c_a = c4_b_a;
  c4_c_ix0 = c4_b_ix0;
  c4_d_ix0 = c4_c_ix0;
  c4_d_a = c4_c_ix0 + 5;
  c4_i96 = c4_d_a;
  c4_c_eml_int_forloop_overflow_check(chartInstance, c4_d_ix0, c4_i96);
  for (c4_k = c4_d_ix0; c4_k <= c4_i96; c4_k++) {
    c4_b_k = c4_k;
    c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c4_b_k), 1, 36, 1, 0) - 1] = c4_c_a * c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c4_b_k), 1, 36, 1, 0) - 1];
  }
}

static void c4_b_sqrt(SFc4_kinematicsInstanceStruct *chartInstance, real_T *c4_x)
{
  if (*c4_x < 0.0) {
    c4_c_eml_error(chartInstance);
  }

  *c4_x = muDoubleScalarSqrt(*c4_x);
}

static void c4_b_eml_xrotg(SFc4_kinematicsInstanceStruct *chartInstance, real_T *
  c4_a, real_T *c4_b, real_T *c4_c, real_T *c4_s)
{
  real_T c4_b_a;
  real_T c4_b_b;
  real_T c4_c_b;
  real_T c4_c_a;
  real_T c4_d_a;
  real_T c4_d_b;
  real_T c4_e_b;
  real_T c4_e_a;
  real_T c4_b_c;
  real_T c4_b_s;
  real_T c4_c_c;
  real_T c4_c_s;
  c4_b_a = *c4_a;
  c4_b_b = *c4_b;
  c4_c_b = c4_b_b;
  c4_c_a = c4_b_a;
  c4_d_a = c4_c_a;
  c4_d_b = c4_c_b;
  c4_e_b = c4_d_b;
  c4_e_a = c4_d_a;
  c4_b_c = 0.0;
  c4_b_s = 0.0;
  drotg32(&c4_e_a, &c4_e_b, &c4_b_c, &c4_b_s);
  c4_c_a = c4_e_a;
  c4_c_b = c4_e_b;
  c4_c_c = c4_b_c;
  c4_c_s = c4_b_s;
  *c4_a = c4_c_a;
  *c4_b = c4_c_b;
  *c4_c = c4_c_c;
  *c4_s = c4_c_s;
}

static void c4_c_eml_xrot(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_x[36], int32_T c4_ix0, int32_T c4_iy0, real_T c4_c, real_T c4_s)
{
  int32_T c4_b_ix0;
  int32_T c4_b_iy0;
  real_T c4_b_c;
  real_T c4_b_s;
  int32_T c4_c_ix0;
  int32_T c4_c_iy0;
  real_T c4_c_c;
  real_T c4_c_s;
  int32_T c4_ix;
  int32_T c4_iy;
  int32_T c4_k;
  real_T c4_a;
  real_T c4_b;
  real_T c4_y;
  real_T c4_b_a;
  real_T c4_b_b;
  real_T c4_b_y;
  real_T c4_temp;
  real_T c4_c_a;
  real_T c4_c_b;
  real_T c4_c_y;
  real_T c4_d_a;
  real_T c4_d_b;
  real_T c4_d_y;
  int32_T c4_e_a;
  int32_T c4_f_a;
  c4_b_ix0 = c4_ix0;
  c4_b_iy0 = c4_iy0;
  c4_b_c = c4_c;
  c4_b_s = c4_s;
  c4_c_ix0 = c4_b_ix0;
  c4_c_iy0 = c4_b_iy0;
  c4_c_c = c4_b_c;
  c4_c_s = c4_b_s;
  c4_ix = c4_c_ix0;
  c4_iy = c4_c_iy0;
  c4_b_eml_int_forloop_overflow_check(chartInstance);
  for (c4_k = 1; c4_k < 7; c4_k++) {
    c4_a = c4_c_c;
    c4_b = c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c4_ix), 1, 36, 1, 0) - 1];
    c4_y = c4_a * c4_b;
    c4_b_a = c4_c_s;
    c4_b_b = c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c4_iy), 1, 36, 1, 0) - 1];
    c4_b_y = c4_b_a * c4_b_b;
    c4_temp = c4_y + c4_b_y;
    c4_c_a = c4_c_c;
    c4_c_b = c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c4_iy), 1, 36, 1, 0) - 1];
    c4_c_y = c4_c_a * c4_c_b;
    c4_d_a = c4_c_s;
    c4_d_b = c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c4_ix), 1, 36, 1, 0) - 1];
    c4_d_y = c4_d_a * c4_d_b;
    c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c4_iy), 1, 36, 1, 0) - 1] = c4_c_y - c4_d_y;
    c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c4_ix), 1, 36, 1, 0) - 1] = c4_temp;
    c4_e_a = c4_iy + 1;
    c4_iy = c4_e_a;
    c4_f_a = c4_ix + 1;
    c4_ix = c4_f_a;
  }
}

static void c4_d_eml_xrot(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_x[72], int32_T c4_ix0, int32_T c4_iy0, real_T c4_c, real_T c4_s)
{
  int32_T c4_b_ix0;
  int32_T c4_b_iy0;
  real_T c4_b_c;
  real_T c4_b_s;
  int32_T c4_c_ix0;
  int32_T c4_c_iy0;
  real_T c4_c_c;
  real_T c4_c_s;
  int32_T c4_ix;
  int32_T c4_iy;
  int32_T c4_k;
  real_T c4_a;
  real_T c4_b;
  real_T c4_y;
  real_T c4_b_a;
  real_T c4_b_b;
  real_T c4_b_y;
  real_T c4_temp;
  real_T c4_c_a;
  real_T c4_c_b;
  real_T c4_c_y;
  real_T c4_d_a;
  real_T c4_d_b;
  real_T c4_d_y;
  int32_T c4_e_a;
  int32_T c4_f_a;
  c4_b_ix0 = c4_ix0;
  c4_b_iy0 = c4_iy0;
  c4_b_c = c4_c;
  c4_b_s = c4_s;
  c4_c_ix0 = c4_b_ix0;
  c4_c_iy0 = c4_b_iy0;
  c4_c_c = c4_b_c;
  c4_c_s = c4_b_s;
  c4_ix = c4_c_ix0;
  c4_iy = c4_c_iy0;
  c4_c_eml_int_forloop_overflow_check(chartInstance, 1, 12);
  for (c4_k = 1; c4_k < 13; c4_k++) {
    c4_a = c4_c_c;
    c4_b = c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c4_ix), 1, 72, 1, 0) - 1];
    c4_y = c4_a * c4_b;
    c4_b_a = c4_c_s;
    c4_b_b = c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c4_iy), 1, 72, 1, 0) - 1];
    c4_b_y = c4_b_a * c4_b_b;
    c4_temp = c4_y + c4_b_y;
    c4_c_a = c4_c_c;
    c4_c_b = c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c4_iy), 1, 72, 1, 0) - 1];
    c4_c_y = c4_c_a * c4_c_b;
    c4_d_a = c4_c_s;
    c4_d_b = c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c4_ix), 1, 72, 1, 0) - 1];
    c4_d_y = c4_d_a * c4_d_b;
    c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c4_iy), 1, 72, 1, 0) - 1] = c4_c_y - c4_d_y;
    c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c4_ix), 1, 72, 1, 0) - 1] = c4_temp;
    c4_e_a = c4_iy + 1;
    c4_iy = c4_e_a;
    c4_f_a = c4_ix + 1;
    c4_ix = c4_f_a;
  }
}

static void c4_c_eml_xswap(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_x[36], int32_T c4_ix0, int32_T c4_iy0)
{
  int32_T c4_b_ix0;
  int32_T c4_b_iy0;
  int32_T c4_c_ix0;
  int32_T c4_c_iy0;
  int32_T c4_ix;
  int32_T c4_iy;
  int32_T c4_k;
  real_T c4_temp;
  int32_T c4_a;
  int32_T c4_b_a;
  c4_b_ix0 = c4_ix0;
  c4_b_iy0 = c4_iy0;
  c4_c_ix0 = c4_b_ix0;
  c4_c_iy0 = c4_b_iy0;
  c4_ix = c4_c_ix0;
  c4_iy = c4_c_iy0;
  c4_b_eml_int_forloop_overflow_check(chartInstance);
  for (c4_k = 1; c4_k < 7; c4_k++) {
    c4_temp = c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c4_ix), 1, 36, 1, 0) - 1];
    c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c4_ix), 1, 36, 1, 0) - 1] = c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c4_iy), 1, 36, 1, 0) - 1];
    c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c4_iy), 1, 36, 1, 0) - 1] = c4_temp;
    c4_a = c4_ix + 1;
    c4_ix = c4_a;
    c4_b_a = c4_iy + 1;
    c4_iy = c4_b_a;
  }
}

static void c4_d_eml_xswap(SFc4_kinematicsInstanceStruct *chartInstance, real_T
  c4_x[72], int32_T c4_ix0, int32_T c4_iy0)
{
  int32_T c4_b_ix0;
  int32_T c4_b_iy0;
  int32_T c4_c_ix0;
  int32_T c4_c_iy0;
  int32_T c4_ix;
  int32_T c4_iy;
  int32_T c4_k;
  real_T c4_temp;
  int32_T c4_a;
  int32_T c4_b_a;
  c4_b_ix0 = c4_ix0;
  c4_b_iy0 = c4_iy0;
  c4_c_ix0 = c4_b_ix0;
  c4_c_iy0 = c4_b_iy0;
  c4_ix = c4_c_ix0;
  c4_iy = c4_c_iy0;
  c4_c_eml_int_forloop_overflow_check(chartInstance, 1, 12);
  for (c4_k = 1; c4_k < 13; c4_k++) {
    c4_temp = c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c4_ix), 1, 72, 1, 0) - 1];
    c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c4_ix), 1, 72, 1, 0) - 1] = c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c4_iy), 1, 72, 1, 0) - 1];
    c4_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c4_iy), 1, 72, 1, 0) - 1] = c4_temp;
    c4_a = c4_ix + 1;
    c4_ix = c4_a;
    c4_b_a = c4_iy + 1;
    c4_iy = c4_b_a;
  }
}

static void init_dsm_address_info(SFc4_kinematicsInstanceStruct *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c4_kinematics_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2593397692U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2281129989U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2630325102U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2610797978U);
}

mxArray *sf_c4_kinematics_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("aWNKcVbgfliVKdLLh9TmZF");
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

static const mxArray *sf_get_sim_state_info_c4_kinematics(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"J_inv\",},{M[8],M[0],T\"is_active_c4_kinematics\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c4_kinematics_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc4_kinematicsInstanceStruct *chartInstance;
    chartInstance = (SFc4_kinematicsInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_kinematicsMachineNumber_,
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,109);
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
      sf_debug_reset_current_state_configuration(_kinematicsMachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization()
{
  return "9dD7BWNpI8y6DwUwJPwChG";
}

static void sf_opaque_initialize_c4_kinematics(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc4_kinematicsInstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c4_kinematics((SFc4_kinematicsInstanceStruct*)
    chartInstanceVar);
  initialize_c4_kinematics((SFc4_kinematicsInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c4_kinematics(void *chartInstanceVar)
{
  enable_c4_kinematics((SFc4_kinematicsInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c4_kinematics(void *chartInstanceVar)
{
  disable_c4_kinematics((SFc4_kinematicsInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c4_kinematics(void *chartInstanceVar)
{
  sf_c4_kinematics((SFc4_kinematicsInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c4_kinematics(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c4_kinematics
    ((SFc4_kinematicsInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c4_kinematics();/* state var info */
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

extern void sf_internal_set_sim_state_c4_kinematics(SimStruct* S, const mxArray *
  st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c4_kinematics();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c4_kinematics((SFc4_kinematicsInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c4_kinematics(SimStruct* S)
{
  return sf_internal_get_sim_state_c4_kinematics(S);
}

static void sf_opaque_set_sim_state_c4_kinematics(SimStruct* S, const mxArray
  *st)
{
  sf_internal_set_sim_state_c4_kinematics(S, st);
}

static void sf_opaque_terminate_c4_kinematics(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc4_kinematicsInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c4_kinematics((SFc4_kinematicsInstanceStruct*) chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_kinematics_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc4_kinematics((SFc4_kinematicsInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c4_kinematics(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c4_kinematics((SFc4_kinematicsInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c4_kinematics(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_kinematics_optimization_info();
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
  ssSetChecksum0(S,(2542997643U));
  ssSetChecksum1(S,(4221522U));
  ssSetChecksum2(S,(3130772195U));
  ssSetChecksum3(S,(3147172935U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c4_kinematics(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c4_kinematics(SimStruct *S)
{
  SFc4_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc4_kinematicsInstanceStruct *)malloc(sizeof
    (SFc4_kinematicsInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc4_kinematicsInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c4_kinematics;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c4_kinematics;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c4_kinematics;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c4_kinematics;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c4_kinematics;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c4_kinematics;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c4_kinematics;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c4_kinematics;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c4_kinematics;
  chartInstance->chartInfo.mdlStart = mdlStart_c4_kinematics;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c4_kinematics;
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

void c4_kinematics_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c4_kinematics(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c4_kinematics(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c4_kinematics(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c4_kinematics_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
