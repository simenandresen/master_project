/* Include files */

#include "blascompat32.h"
#include "dynamics_kinematics_sfun.h"
#include "c14_dynamics_kinematics.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "dynamics_kinematics_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c14_debug_family_names[11] = { "force_n", "reset",
  "K_pos_correction", "nargin", "nargout", "pos_err", "force", "prev_alpha",
  "pos_corr", "alpha", "force_norm" };

/* Function Declarations */
static void initialize_c14_dynamics_kinematics
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance);
static void initialize_params_c14_dynamics_kinematics
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance);
static void enable_c14_dynamics_kinematics
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance);
static void disable_c14_dynamics_kinematics
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance);
static void c14_update_debugger_state_c14_dynamics_kinematics
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c14_dynamics_kinematics
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance);
static void set_sim_state_c14_dynamics_kinematics
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance, const mxArray *c14_st);
static void finalize_c14_dynamics_kinematics
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance);
static void sf_c14_dynamics_kinematics(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance);
static void c14_chartstep_c14_dynamics_kinematics
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance);
static void initSimStructsc14_dynamics_kinematics
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c14_machineNumber, uint32_T
  c14_chartNumber);
static const mxArray *c14_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData);
static real_T c14_emlrt_marshallIn(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c14_force_norm, const char_T *c14_identifier);
static real_T c14_b_emlrt_marshallIn(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId);
static void c14_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData);
static const mxArray *c14_b_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData);
static void c14_c_emlrt_marshallIn(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c14_pos_corr, const char_T *c14_identifier,
  real_T c14_y[6]);
static void c14_d_emlrt_marshallIn(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId,
  real_T c14_y[6]);
static void c14_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData);
static const mxArray *c14_c_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData);
static const mxArray *c14_d_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData);
static void c14_e_emlrt_marshallIn(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId,
  real_T c14_y[36]);
static void c14_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData);
static void c14_info_helper(c14_ResolvedFunctionInfo c14_info[32]);
static real_T c14_norm(SFc14_dynamics_kinematicsInstanceStruct *chartInstance,
  real_T c14_x[12]);
static void c14_eml_int_forloop_overflow_check
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance);
static void c14_eml_scalar_eg(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance);
static const mxArray *c14_e_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData);
static int32_T c14_f_emlrt_marshallIn(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId);
static void c14_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData);
static uint8_T c14_g_emlrt_marshallIn(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c14_b_is_active_c14_dynamics_kinematics, const
  char_T *c14_identifier);
static uint8_T c14_h_emlrt_marshallIn(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId);
static void init_dsm_address_info(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c14_dynamics_kinematics
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance)
{
  chartInstance->c14_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c14_is_active_c14_dynamics_kinematics = 0U;
}

static void initialize_params_c14_dynamics_kinematics
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance)
{
}

static void enable_c14_dynamics_kinematics
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c14_dynamics_kinematics
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c14_update_debugger_state_c14_dynamics_kinematics
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c14_dynamics_kinematics
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance)
{
  const mxArray *c14_st;
  const mxArray *c14_y = NULL;
  real_T c14_hoistedGlobal;
  real_T c14_u;
  const mxArray *c14_b_y = NULL;
  real_T c14_b_hoistedGlobal;
  real_T c14_b_u;
  const mxArray *c14_c_y = NULL;
  int32_T c14_i0;
  real_T c14_c_u[6];
  const mxArray *c14_d_y = NULL;
  uint8_T c14_c_hoistedGlobal;
  uint8_T c14_d_u;
  const mxArray *c14_e_y = NULL;
  real_T *c14_alpha;
  real_T *c14_force_norm;
  real_T (*c14_pos_corr)[6];
  c14_force_norm = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c14_alpha = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c14_pos_corr = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c14_st = NULL;
  c14_st = NULL;
  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_createcellarray(4), FALSE);
  c14_hoistedGlobal = *c14_alpha;
  c14_u = c14_hoistedGlobal;
  c14_b_y = NULL;
  sf_mex_assign(&c14_b_y, sf_mex_create("y", &c14_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c14_y, 0, c14_b_y);
  c14_b_hoistedGlobal = *c14_force_norm;
  c14_b_u = c14_b_hoistedGlobal;
  c14_c_y = NULL;
  sf_mex_assign(&c14_c_y, sf_mex_create("y", &c14_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c14_y, 1, c14_c_y);
  for (c14_i0 = 0; c14_i0 < 6; c14_i0++) {
    c14_c_u[c14_i0] = (*c14_pos_corr)[c14_i0];
  }

  c14_d_y = NULL;
  sf_mex_assign(&c14_d_y, sf_mex_create("y", c14_c_u, 0, 0U, 1U, 0U, 1, 6),
                FALSE);
  sf_mex_setcell(c14_y, 2, c14_d_y);
  c14_c_hoistedGlobal = chartInstance->c14_is_active_c14_dynamics_kinematics;
  c14_d_u = c14_c_hoistedGlobal;
  c14_e_y = NULL;
  sf_mex_assign(&c14_e_y, sf_mex_create("y", &c14_d_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c14_y, 3, c14_e_y);
  sf_mex_assign(&c14_st, c14_y, FALSE);
  return c14_st;
}

static void set_sim_state_c14_dynamics_kinematics
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance, const mxArray *c14_st)
{
  const mxArray *c14_u;
  real_T c14_dv0[6];
  int32_T c14_i1;
  real_T *c14_alpha;
  real_T *c14_force_norm;
  real_T (*c14_pos_corr)[6];
  c14_force_norm = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c14_alpha = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c14_pos_corr = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c14_doneDoubleBufferReInit = TRUE;
  c14_u = sf_mex_dup(c14_st);
  *c14_alpha = c14_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c14_u, 0)), "alpha");
  *c14_force_norm = c14_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c14_u, 1)), "force_norm");
  c14_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c14_u, 2)),
    "pos_corr", c14_dv0);
  for (c14_i1 = 0; c14_i1 < 6; c14_i1++) {
    (*c14_pos_corr)[c14_i1] = c14_dv0[c14_i1];
  }

  chartInstance->c14_is_active_c14_dynamics_kinematics = c14_g_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c14_u, 3)),
     "is_active_c14_dynamics_kinematics");
  sf_mex_destroy(&c14_u);
  c14_update_debugger_state_c14_dynamics_kinematics(chartInstance);
  sf_mex_destroy(&c14_st);
}

static void finalize_c14_dynamics_kinematics
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance)
{
}

static void sf_c14_dynamics_kinematics(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance)
{
  int32_T c14_i2;
  int32_T c14_i3;
  int32_T c14_i4;
  real_T *c14_prev_alpha;
  real_T *c14_alpha;
  real_T *c14_force_norm;
  real_T (*c14_force)[12];
  real_T (*c14_pos_corr)[6];
  real_T (*c14_pos_err)[6];
  c14_force_norm = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c14_alpha = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c14_prev_alpha = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c14_force = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 1);
  c14_pos_corr = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c14_pos_err = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 13U, chartInstance->c14_sfEvent);
  for (c14_i2 = 0; c14_i2 < 6; c14_i2++) {
    _SFD_DATA_RANGE_CHECK((*c14_pos_err)[c14_i2], 0U);
  }

  for (c14_i3 = 0; c14_i3 < 6; c14_i3++) {
    _SFD_DATA_RANGE_CHECK((*c14_pos_corr)[c14_i3], 1U);
  }

  for (c14_i4 = 0; c14_i4 < 12; c14_i4++) {
    _SFD_DATA_RANGE_CHECK((*c14_force)[c14_i4], 2U);
  }

  _SFD_DATA_RANGE_CHECK(*c14_prev_alpha, 3U);
  _SFD_DATA_RANGE_CHECK(*c14_alpha, 4U);
  _SFD_DATA_RANGE_CHECK(*c14_force_norm, 5U);
  chartInstance->c14_sfEvent = CALL_EVENT;
  c14_chartstep_c14_dynamics_kinematics(chartInstance);
  sf_debug_check_for_state_inconsistency(_dynamics_kinematicsMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c14_chartstep_c14_dynamics_kinematics
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance)
{
  real_T c14_hoistedGlobal;
  int32_T c14_i5;
  real_T c14_pos_err[6];
  int32_T c14_i6;
  real_T c14_force[12];
  real_T c14_prev_alpha;
  uint32_T c14_debug_family_var_map[11];
  real_T c14_force_n;
  real_T c14_reset;
  real_T c14_K_pos_correction[36];
  real_T c14_nargin = 3.0;
  real_T c14_nargout = 3.0;
  real_T c14_pos_corr[6];
  real_T c14_alpha;
  real_T c14_force_norm;
  int32_T c14_i7;
  real_T c14_b_force[12];
  real_T c14_a;
  real_T c14_y;
  int32_T c14_i8;
  int32_T c14_i9;
  static real_T c14_b_a[36] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0 };

  int32_T c14_i10;
  int32_T c14_i11;
  real_T c14_b[6];
  int32_T c14_i12;
  int32_T c14_i13;
  int32_T c14_i14;
  real_T c14_C[6];
  int32_T c14_i15;
  int32_T c14_i16;
  int32_T c14_i17;
  int32_T c14_i18;
  int32_T c14_i19;
  int32_T c14_i20;
  int32_T c14_i21;
  real_T *c14_b_prev_alpha;
  real_T *c14_b_alpha;
  real_T *c14_b_force_norm;
  real_T (*c14_b_pos_corr)[6];
  real_T (*c14_c_force)[12];
  real_T (*c14_b_pos_err)[6];
  c14_b_force_norm = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c14_b_alpha = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c14_b_prev_alpha = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c14_c_force = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 1);
  c14_b_pos_corr = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c14_b_pos_err = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 13U, chartInstance->c14_sfEvent);
  c14_hoistedGlobal = *c14_b_prev_alpha;
  for (c14_i5 = 0; c14_i5 < 6; c14_i5++) {
    c14_pos_err[c14_i5] = (*c14_b_pos_err)[c14_i5];
  }

  for (c14_i6 = 0; c14_i6 < 12; c14_i6++) {
    c14_force[c14_i6] = (*c14_c_force)[c14_i6];
  }

  c14_prev_alpha = c14_hoistedGlobal;
  sf_debug_symbol_scope_push_eml(0U, 11U, 11U, c14_debug_family_names,
    c14_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(&c14_force_n, 0U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c14_reset, 1U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(c14_K_pos_correction, 2U,
    c14_d_sf_marshallOut, c14_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c14_nargin, 3U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c14_nargout, 4U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c14_pos_err, 5U, c14_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c14_force, 6U, c14_c_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(&c14_prev_alpha, 7U, c14_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c14_pos_corr, 8U,
    c14_b_sf_marshallOut, c14_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(&c14_alpha, 9U, c14_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(&c14_force_norm, 10U,
    c14_sf_marshallOut, c14_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 4);
  c14_alpha = 1.0;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 6);
  for (c14_i7 = 0; c14_i7 < 12; c14_i7++) {
    c14_b_force[c14_i7] = c14_force[c14_i7];
  }

  c14_force_n = c14_norm(chartInstance, c14_b_force);
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 7);
  c14_reset = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 9);
  if (CV_EML_IF(0, 1, 0, c14_prev_alpha == 0.0)) {
    _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 10);
    c14_alpha = 0.0;
  } else {
    _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 11);
    if (CV_EML_IF(0, 1, 1, c14_force_n < 0.1)) {
      _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 12);
      c14_alpha = 1.0;
    } else {
      _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 13);
      if (CV_EML_IF(0, 1, 2, c14_force_n < 0.3)) {
        _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 14);
        c14_a = c14_force_n - 0.1;
        c14_y = c14_a * 3.0;
        c14_alpha = 1.0 - c14_y;
      } else {
        _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 16);
        c14_reset = 1.0;
        _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 17);
        c14_alpha = 0.0;
      }
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 19);
  c14_alpha = 1.0;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 20);
  for (c14_i8 = 0; c14_i8 < 36; c14_i8++) {
    c14_K_pos_correction[c14_i8] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 21);
  for (c14_i9 = 0; c14_i9 < 36; c14_i9++) {
    c14_K_pos_correction[c14_i9] = c14_b_a[c14_i9];
  }

  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 22);
  for (c14_i10 = 0; c14_i10 < 6; c14_i10++) {
    c14_pos_corr[c14_i10] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 23);
  for (c14_i11 = 0; c14_i11 < 6; c14_i11++) {
    c14_b[c14_i11] = c14_pos_err[c14_i11];
  }

  c14_eml_scalar_eg(chartInstance);
  c14_eml_scalar_eg(chartInstance);
  for (c14_i12 = 0; c14_i12 < 6; c14_i12++) {
    c14_pos_corr[c14_i12] = 0.0;
  }

  for (c14_i13 = 0; c14_i13 < 6; c14_i13++) {
    c14_pos_corr[c14_i13] = 0.0;
  }

  for (c14_i14 = 0; c14_i14 < 6; c14_i14++) {
    c14_C[c14_i14] = c14_pos_corr[c14_i14];
  }

  for (c14_i15 = 0; c14_i15 < 6; c14_i15++) {
    c14_pos_corr[c14_i15] = c14_C[c14_i15];
  }

  for (c14_i16 = 0; c14_i16 < 6; c14_i16++) {
    c14_C[c14_i16] = c14_pos_corr[c14_i16];
  }

  for (c14_i17 = 0; c14_i17 < 6; c14_i17++) {
    c14_pos_corr[c14_i17] = c14_C[c14_i17];
  }

  for (c14_i18 = 0; c14_i18 < 6; c14_i18++) {
    c14_pos_corr[c14_i18] = 0.0;
    c14_i19 = 0;
    for (c14_i20 = 0; c14_i20 < 6; c14_i20++) {
      c14_pos_corr[c14_i18] += c14_b_a[c14_i19 + c14_i18] * c14_b[c14_i20];
      c14_i19 += 6;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 24);
  c14_force_norm = 1.0;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 25);
  c14_force_norm = c14_force_n;
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, -25);
  sf_debug_symbol_scope_pop();
  for (c14_i21 = 0; c14_i21 < 6; c14_i21++) {
    (*c14_b_pos_corr)[c14_i21] = c14_pos_corr[c14_i21];
  }

  *c14_b_alpha = c14_alpha;
  *c14_b_force_norm = c14_force_norm;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 13U, chartInstance->c14_sfEvent);
}

static void initSimStructsc14_dynamics_kinematics
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c14_machineNumber, uint32_T
  c14_chartNumber)
{
}

static const mxArray *c14_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData)
{
  const mxArray *c14_mxArrayOutData = NULL;
  real_T c14_u;
  const mxArray *c14_y = NULL;
  SFc14_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc14_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c14_mxArrayOutData = NULL;
  c14_u = *(real_T *)c14_inData;
  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", &c14_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c14_mxArrayOutData, c14_y, FALSE);
  return c14_mxArrayOutData;
}

static real_T c14_emlrt_marshallIn(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c14_force_norm, const char_T *c14_identifier)
{
  real_T c14_y;
  emlrtMsgIdentifier c14_thisId;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_y = c14_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_force_norm),
    &c14_thisId);
  sf_mex_destroy(&c14_force_norm);
  return c14_y;
}

static real_T c14_b_emlrt_marshallIn(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId)
{
  real_T c14_y;
  real_T c14_d0;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_u), &c14_d0, 1, 0, 0U, 0, 0U, 0);
  c14_y = c14_d0;
  sf_mex_destroy(&c14_u);
  return c14_y;
}

static void c14_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData)
{
  const mxArray *c14_force_norm;
  const char_T *c14_identifier;
  emlrtMsgIdentifier c14_thisId;
  real_T c14_y;
  SFc14_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc14_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c14_force_norm = sf_mex_dup(c14_mxArrayInData);
  c14_identifier = c14_varName;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_y = c14_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_force_norm),
    &c14_thisId);
  sf_mex_destroy(&c14_force_norm);
  *(real_T *)c14_outData = c14_y;
  sf_mex_destroy(&c14_mxArrayInData);
}

static const mxArray *c14_b_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData)
{
  const mxArray *c14_mxArrayOutData = NULL;
  int32_T c14_i22;
  real_T c14_b_inData[6];
  int32_T c14_i23;
  real_T c14_u[6];
  const mxArray *c14_y = NULL;
  SFc14_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc14_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c14_mxArrayOutData = NULL;
  for (c14_i22 = 0; c14_i22 < 6; c14_i22++) {
    c14_b_inData[c14_i22] = (*(real_T (*)[6])c14_inData)[c14_i22];
  }

  for (c14_i23 = 0; c14_i23 < 6; c14_i23++) {
    c14_u[c14_i23] = c14_b_inData[c14_i23];
  }

  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", c14_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_assign(&c14_mxArrayOutData, c14_y, FALSE);
  return c14_mxArrayOutData;
}

static void c14_c_emlrt_marshallIn(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c14_pos_corr, const char_T *c14_identifier,
  real_T c14_y[6])
{
  emlrtMsgIdentifier c14_thisId;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_pos_corr), &c14_thisId,
    c14_y);
  sf_mex_destroy(&c14_pos_corr);
}

static void c14_d_emlrt_marshallIn(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId,
  real_T c14_y[6])
{
  real_T c14_dv1[6];
  int32_T c14_i24;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_u), c14_dv1, 1, 0, 0U, 1, 0U, 1, 6);
  for (c14_i24 = 0; c14_i24 < 6; c14_i24++) {
    c14_y[c14_i24] = c14_dv1[c14_i24];
  }

  sf_mex_destroy(&c14_u);
}

static void c14_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData)
{
  const mxArray *c14_pos_corr;
  const char_T *c14_identifier;
  emlrtMsgIdentifier c14_thisId;
  real_T c14_y[6];
  int32_T c14_i25;
  SFc14_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc14_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c14_pos_corr = sf_mex_dup(c14_mxArrayInData);
  c14_identifier = c14_varName;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_pos_corr), &c14_thisId,
    c14_y);
  sf_mex_destroy(&c14_pos_corr);
  for (c14_i25 = 0; c14_i25 < 6; c14_i25++) {
    (*(real_T (*)[6])c14_outData)[c14_i25] = c14_y[c14_i25];
  }

  sf_mex_destroy(&c14_mxArrayInData);
}

static const mxArray *c14_c_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData)
{
  const mxArray *c14_mxArrayOutData = NULL;
  int32_T c14_i26;
  real_T c14_b_inData[12];
  int32_T c14_i27;
  real_T c14_u[12];
  const mxArray *c14_y = NULL;
  SFc14_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc14_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c14_mxArrayOutData = NULL;
  for (c14_i26 = 0; c14_i26 < 12; c14_i26++) {
    c14_b_inData[c14_i26] = (*(real_T (*)[12])c14_inData)[c14_i26];
  }

  for (c14_i27 = 0; c14_i27 < 12; c14_i27++) {
    c14_u[c14_i27] = c14_b_inData[c14_i27];
  }

  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", c14_u, 0, 0U, 1U, 0U, 1, 12), FALSE);
  sf_mex_assign(&c14_mxArrayOutData, c14_y, FALSE);
  return c14_mxArrayOutData;
}

static const mxArray *c14_d_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData)
{
  const mxArray *c14_mxArrayOutData = NULL;
  int32_T c14_i28;
  int32_T c14_i29;
  int32_T c14_i30;
  real_T c14_b_inData[36];
  int32_T c14_i31;
  int32_T c14_i32;
  int32_T c14_i33;
  real_T c14_u[36];
  const mxArray *c14_y = NULL;
  SFc14_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc14_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c14_mxArrayOutData = NULL;
  c14_i28 = 0;
  for (c14_i29 = 0; c14_i29 < 6; c14_i29++) {
    for (c14_i30 = 0; c14_i30 < 6; c14_i30++) {
      c14_b_inData[c14_i30 + c14_i28] = (*(real_T (*)[36])c14_inData)[c14_i30 +
        c14_i28];
    }

    c14_i28 += 6;
  }

  c14_i31 = 0;
  for (c14_i32 = 0; c14_i32 < 6; c14_i32++) {
    for (c14_i33 = 0; c14_i33 < 6; c14_i33++) {
      c14_u[c14_i33 + c14_i31] = c14_b_inData[c14_i33 + c14_i31];
    }

    c14_i31 += 6;
  }

  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", c14_u, 0, 0U, 1U, 0U, 2, 6, 6), FALSE);
  sf_mex_assign(&c14_mxArrayOutData, c14_y, FALSE);
  return c14_mxArrayOutData;
}

static void c14_e_emlrt_marshallIn(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId,
  real_T c14_y[36])
{
  real_T c14_dv2[36];
  int32_T c14_i34;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_u), c14_dv2, 1, 0, 0U, 1, 0U, 2, 6,
                6);
  for (c14_i34 = 0; c14_i34 < 36; c14_i34++) {
    c14_y[c14_i34] = c14_dv2[c14_i34];
  }

  sf_mex_destroy(&c14_u);
}

static void c14_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData)
{
  const mxArray *c14_K_pos_correction;
  const char_T *c14_identifier;
  emlrtMsgIdentifier c14_thisId;
  real_T c14_y[36];
  int32_T c14_i35;
  int32_T c14_i36;
  int32_T c14_i37;
  SFc14_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc14_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c14_K_pos_correction = sf_mex_dup(c14_mxArrayInData);
  c14_identifier = c14_varName;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_K_pos_correction),
    &c14_thisId, c14_y);
  sf_mex_destroy(&c14_K_pos_correction);
  c14_i35 = 0;
  for (c14_i36 = 0; c14_i36 < 6; c14_i36++) {
    for (c14_i37 = 0; c14_i37 < 6; c14_i37++) {
      (*(real_T (*)[36])c14_outData)[c14_i37 + c14_i35] = c14_y[c14_i37 +
        c14_i35];
    }

    c14_i35 += 6;
  }

  sf_mex_destroy(&c14_mxArrayInData);
}

const mxArray *sf_c14_dynamics_kinematics_get_eml_resolved_functions_info(void)
{
  const mxArray *c14_nameCaptureInfo;
  c14_ResolvedFunctionInfo c14_info[32];
  const mxArray *c14_m0 = NULL;
  int32_T c14_i38;
  c14_ResolvedFunctionInfo *c14_r0;
  c14_nameCaptureInfo = NULL;
  c14_nameCaptureInfo = NULL;
  c14_info_helper(c14_info);
  sf_mex_assign(&c14_m0, sf_mex_createstruct("nameCaptureInfo", 1, 32), FALSE);
  for (c14_i38 = 0; c14_i38 < 32; c14_i38++) {
    c14_r0 = &c14_info[c14_i38];
    sf_mex_addfield(c14_m0, sf_mex_create("nameCaptureInfo", c14_r0->context, 15,
      0U, 0U, 0U, 2, 1, strlen(c14_r0->context)), "context", "nameCaptureInfo",
                    c14_i38);
    sf_mex_addfield(c14_m0, sf_mex_create("nameCaptureInfo", c14_r0->name, 15,
      0U, 0U, 0U, 2, 1, strlen(c14_r0->name)), "name", "nameCaptureInfo",
                    c14_i38);
    sf_mex_addfield(c14_m0, sf_mex_create("nameCaptureInfo",
      c14_r0->dominantType, 15, 0U, 0U, 0U, 2, 1, strlen(c14_r0->dominantType)),
                    "dominantType", "nameCaptureInfo", c14_i38);
    sf_mex_addfield(c14_m0, sf_mex_create("nameCaptureInfo", c14_r0->resolved,
      15, 0U, 0U, 0U, 2, 1, strlen(c14_r0->resolved)), "resolved",
                    "nameCaptureInfo", c14_i38);
    sf_mex_addfield(c14_m0, sf_mex_create("nameCaptureInfo", &c14_r0->fileTimeLo,
      7, 0U, 0U, 0U, 0), "fileTimeLo", "nameCaptureInfo", c14_i38);
    sf_mex_addfield(c14_m0, sf_mex_create("nameCaptureInfo", &c14_r0->fileTimeHi,
      7, 0U, 0U, 0U, 0), "fileTimeHi", "nameCaptureInfo", c14_i38);
    sf_mex_addfield(c14_m0, sf_mex_create("nameCaptureInfo",
      &c14_r0->mFileTimeLo, 7, 0U, 0U, 0U, 0), "mFileTimeLo", "nameCaptureInfo",
                    c14_i38);
    sf_mex_addfield(c14_m0, sf_mex_create("nameCaptureInfo",
      &c14_r0->mFileTimeHi, 7, 0U, 0U, 0U, 0), "mFileTimeHi", "nameCaptureInfo",
                    c14_i38);
  }

  sf_mex_assign(&c14_nameCaptureInfo, c14_m0, FALSE);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c14_nameCaptureInfo);
  return c14_nameCaptureInfo;
}

static void c14_info_helper(c14_ResolvedFunctionInfo c14_info[32])
{
  c14_info[0].context = "";
  c14_info[0].name = "norm";
  c14_info[0].dominantType = "double";
  c14_info[0].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/norm.m";
  c14_info[0].fileTimeLo = 1286818826U;
  c14_info[0].fileTimeHi = 0U;
  c14_info[0].mFileTimeLo = 0U;
  c14_info[0].mFileTimeHi = 0U;
  c14_info[1].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm";
  c14_info[1].name = "eml_index_class";
  c14_info[1].dominantType = "";
  c14_info[1].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[1].fileTimeLo = 1286818778U;
  c14_info[1].fileTimeHi = 0U;
  c14_info[1].mFileTimeLo = 0U;
  c14_info[1].mFileTimeHi = 0U;
  c14_info[2].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm";
  c14_info[2].name = "eml_xnrm2";
  c14_info[2].dominantType = "int32";
  c14_info[2].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c14_info[2].fileTimeLo = 1299076776U;
  c14_info[2].fileTimeHi = 0U;
  c14_info[2].mFileTimeLo = 0U;
  c14_info[2].mFileTimeHi = 0U;
  c14_info[3].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m";
  c14_info[3].name = "eml_blas_inline";
  c14_info[3].dominantType = "";
  c14_info[3].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c14_info[3].fileTimeLo = 1299076768U;
  c14_info[3].fileTimeHi = 0U;
  c14_info[3].mFileTimeLo = 0U;
  c14_info[3].mFileTimeHi = 0U;
  c14_info[4].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xnrm2.m";
  c14_info[4].name = "eml_refblas_xnrm2";
  c14_info[4].dominantType = "int32";
  c14_info[4].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c14_info[4].fileTimeLo = 1299076784U;
  c14_info[4].fileTimeHi = 0U;
  c14_info[4].mFileTimeLo = 0U;
  c14_info[4].mFileTimeHi = 0U;
  c14_info[5].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c14_info[5].name = "realmin";
  c14_info[5].dominantType = "char";
  c14_info[5].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/realmin.m";
  c14_info[5].fileTimeLo = 1307651242U;
  c14_info[5].fileTimeHi = 0U;
  c14_info[5].mFileTimeLo = 0U;
  c14_info[5].mFileTimeHi = 0U;
  c14_info[6].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/realmin.m";
  c14_info[6].name = "eml_realmin";
  c14_info[6].dominantType = "char";
  c14_info[6].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c14_info[6].fileTimeLo = 1307651244U;
  c14_info[6].fileTimeHi = 0U;
  c14_info[6].mFileTimeLo = 0U;
  c14_info[6].mFileTimeHi = 0U;
  c14_info[7].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_realmin.m";
  c14_info[7].name = "eml_float_model";
  c14_info[7].dominantType = "char";
  c14_info[7].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_float_model.m";
  c14_info[7].fileTimeLo = 1307651242U;
  c14_info[7].fileTimeHi = 0U;
  c14_info[7].mFileTimeLo = 0U;
  c14_info[7].mFileTimeHi = 0U;
  c14_info[8].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c14_info[8].name = "eml_index_class";
  c14_info[8].dominantType = "";
  c14_info[8].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[8].fileTimeLo = 1286818778U;
  c14_info[8].fileTimeHi = 0U;
  c14_info[8].mFileTimeLo = 0U;
  c14_info[8].mFileTimeHi = 0U;
  c14_info[9].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c14_info[9].name = "eml_index_minus";
  c14_info[9].dominantType = "int32";
  c14_info[9].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c14_info[9].fileTimeLo = 1286818778U;
  c14_info[9].fileTimeHi = 0U;
  c14_info[9].mFileTimeLo = 0U;
  c14_info[9].mFileTimeHi = 0U;
  c14_info[10].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_minus.m";
  c14_info[10].name = "eml_index_class";
  c14_info[10].dominantType = "";
  c14_info[10].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[10].fileTimeLo = 1286818778U;
  c14_info[10].fileTimeHi = 0U;
  c14_info[10].mFileTimeLo = 0U;
  c14_info[10].mFileTimeHi = 0U;
  c14_info[11].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c14_info[11].name = "eml_index_times";
  c14_info[11].dominantType = "int32";
  c14_info[11].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c14_info[11].fileTimeLo = 1286818780U;
  c14_info[11].fileTimeHi = 0U;
  c14_info[11].mFileTimeLo = 0U;
  c14_info[11].mFileTimeHi = 0U;
  c14_info[12].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_times.m";
  c14_info[12].name = "eml_index_class";
  c14_info[12].dominantType = "";
  c14_info[12].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[12].fileTimeLo = 1286818778U;
  c14_info[12].fileTimeHi = 0U;
  c14_info[12].mFileTimeLo = 0U;
  c14_info[12].mFileTimeHi = 0U;
  c14_info[13].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c14_info[13].name = "eml_index_plus";
  c14_info[13].dominantType = "int32";
  c14_info[13].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c14_info[13].fileTimeLo = 1286818778U;
  c14_info[13].fileTimeHi = 0U;
  c14_info[13].mFileTimeLo = 0U;
  c14_info[13].mFileTimeHi = 0U;
  c14_info[14].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c14_info[14].name = "eml_index_class";
  c14_info[14].dominantType = "";
  c14_info[14].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[14].fileTimeLo = 1286818778U;
  c14_info[14].fileTimeHi = 0U;
  c14_info[14].mFileTimeLo = 0U;
  c14_info[14].mFileTimeHi = 0U;
  c14_info[15].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c14_info[15].name = "eml_int_forloop_overflow_check";
  c14_info[15].dominantType = "";
  c14_info[15].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c14_info[15].fileTimeLo = 1311255316U;
  c14_info[15].fileTimeHi = 0U;
  c14_info[15].mFileTimeLo = 0U;
  c14_info[15].mFileTimeHi = 0U;
  c14_info[16].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper";
  c14_info[16].name = "intmax";
  c14_info[16].dominantType = "char";
  c14_info[16].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/intmax.m";
  c14_info[16].fileTimeLo = 1311255316U;
  c14_info[16].fileTimeHi = 0U;
  c14_info[16].mFileTimeLo = 0U;
  c14_info[16].mFileTimeHi = 0U;
  c14_info[17].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xnrm2.m";
  c14_info[17].name = "abs";
  c14_info[17].dominantType = "double";
  c14_info[17].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c14_info[17].fileTimeLo = 1286818694U;
  c14_info[17].fileTimeHi = 0U;
  c14_info[17].mFileTimeLo = 0U;
  c14_info[17].mFileTimeHi = 0U;
  c14_info[18].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/abs.m";
  c14_info[18].name = "eml_scalar_abs";
  c14_info[18].dominantType = "double";
  c14_info[18].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m";
  c14_info[18].fileTimeLo = 1286818712U;
  c14_info[18].fileTimeHi = 0U;
  c14_info[18].mFileTimeLo = 0U;
  c14_info[18].mFileTimeHi = 0U;
  c14_info[19].context = "";
  c14_info[19].name = "mtimes";
  c14_info[19].dominantType = "double";
  c14_info[19].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c14_info[19].fileTimeLo = 1289519692U;
  c14_info[19].fileTimeHi = 0U;
  c14_info[19].mFileTimeLo = 0U;
  c14_info[19].mFileTimeHi = 0U;
  c14_info[20].context = "";
  c14_info[20].name = "diag";
  c14_info[20].dominantType = "double";
  c14_info[20].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/diag.m";
  c14_info[20].fileTimeLo = 1286818686U;
  c14_info[20].fileTimeHi = 0U;
  c14_info[20].mFileTimeLo = 0U;
  c14_info[20].mFileTimeHi = 0U;
  c14_info[21].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/diag.m";
  c14_info[21].name = "eml_index_class";
  c14_info[21].dominantType = "";
  c14_info[21].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[21].fileTimeLo = 1286818778U;
  c14_info[21].fileTimeHi = 0U;
  c14_info[21].mFileTimeLo = 0U;
  c14_info[21].mFileTimeHi = 0U;
  c14_info[22].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/diag.m";
  c14_info[22].name = "eml_index_plus";
  c14_info[22].dominantType = "int32";
  c14_info[22].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_plus.m";
  c14_info[22].fileTimeLo = 1286818778U;
  c14_info[22].fileTimeHi = 0U;
  c14_info[22].mFileTimeLo = 0U;
  c14_info[22].mFileTimeHi = 0U;
  c14_info[23].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/diag.m";
  c14_info[23].name = "eml_scalar_eg";
  c14_info[23].dominantType = "double";
  c14_info[23].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c14_info[23].fileTimeLo = 1286818796U;
  c14_info[23].fileTimeHi = 0U;
  c14_info[23].mFileTimeLo = 0U;
  c14_info[23].mFileTimeHi = 0U;
  c14_info[24].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/elmat/diag.m";
  c14_info[24].name = "eml_int_forloop_overflow_check";
  c14_info[24].dominantType = "";
  c14_info[24].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m";
  c14_info[24].fileTimeLo = 1311255316U;
  c14_info[24].fileTimeHi = 0U;
  c14_info[24].mFileTimeLo = 0U;
  c14_info[24].mFileTimeHi = 0U;
  c14_info[25].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c14_info[25].name = "eml_index_class";
  c14_info[25].dominantType = "";
  c14_info[25].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_index_class.m";
  c14_info[25].fileTimeLo = 1286818778U;
  c14_info[25].fileTimeHi = 0U;
  c14_info[25].mFileTimeLo = 0U;
  c14_info[25].mFileTimeHi = 0U;
  c14_info[26].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c14_info[26].name = "eml_scalar_eg";
  c14_info[26].dominantType = "double";
  c14_info[26].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c14_info[26].fileTimeLo = 1286818796U;
  c14_info[26].fileTimeHi = 0U;
  c14_info[26].mFileTimeLo = 0U;
  c14_info[26].mFileTimeHi = 0U;
  c14_info[27].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c14_info[27].name = "eml_xgemm";
  c14_info[27].dominantType = "int32";
  c14_info[27].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c14_info[27].fileTimeLo = 1299076772U;
  c14_info[27].fileTimeHi = 0U;
  c14_info[27].mFileTimeLo = 0U;
  c14_info[27].mFileTimeHi = 0U;
  c14_info[28].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m";
  c14_info[28].name = "eml_blas_inline";
  c14_info[28].dominantType = "";
  c14_info[28].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m";
  c14_info[28].fileTimeLo = 1299076768U;
  c14_info[28].fileTimeHi = 0U;
  c14_info[28].mFileTimeLo = 0U;
  c14_info[28].mFileTimeHi = 0U;
  c14_info[29].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold";
  c14_info[29].name = "mtimes";
  c14_info[29].dominantType = "double";
  c14_info[29].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/ops/mtimes.m";
  c14_info[29].fileTimeLo = 1289519692U;
  c14_info[29].fileTimeHi = 0U;
  c14_info[29].mFileTimeLo = 0U;
  c14_info[29].mFileTimeHi = 0U;
  c14_info[30].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c14_info[30].name = "eml_scalar_eg";
  c14_info[30].dominantType = "double";
  c14_info[30].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m";
  c14_info[30].fileTimeLo = 1286818796U;
  c14_info[30].fileTimeHi = 0U;
  c14_info[30].mFileTimeLo = 0U;
  c14_info[30].mFileTimeHi = 0U;
  c14_info[31].context =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m";
  c14_info[31].name = "eml_refblas_xgemm";
  c14_info[31].dominantType = "int32";
  c14_info[31].resolved =
    "[ILXE]/usr/local/MATLAB/R2012a/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m";
  c14_info[31].fileTimeLo = 1299076774U;
  c14_info[31].fileTimeHi = 0U;
  c14_info[31].mFileTimeLo = 0U;
  c14_info[31].mFileTimeHi = 0U;
}

static real_T c14_norm(SFc14_dynamics_kinematicsInstanceStruct *chartInstance,
  real_T c14_x[12])
{
  real_T c14_y;
  real_T c14_scale;
  int32_T c14_k;
  int32_T c14_b_k;
  real_T c14_b_x;
  real_T c14_c_x;
  real_T c14_absxk;
  real_T c14_t;
  c14_y = 0.0;
  c14_scale = 2.2250738585072014E-308;
  c14_eml_int_forloop_overflow_check(chartInstance);
  for (c14_k = 1; c14_k < 13; c14_k++) {
    c14_b_k = c14_k;
    c14_b_x = c14_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c14_b_k), 1, 12, 1, 0) - 1];
    c14_c_x = c14_b_x;
    c14_absxk = muDoubleScalarAbs(c14_c_x);
    if (c14_absxk > c14_scale) {
      c14_t = c14_scale / c14_absxk;
      c14_y = 1.0 + c14_y * c14_t * c14_t;
      c14_scale = c14_absxk;
    } else {
      c14_t = c14_absxk / c14_scale;
      c14_y += c14_t * c14_t;
    }
  }

  return c14_scale * muDoubleScalarSqrt(c14_y);
}

static void c14_eml_int_forloop_overflow_check
  (SFc14_dynamics_kinematicsInstanceStruct *chartInstance)
{
}

static void c14_eml_scalar_eg(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance)
{
}

static const mxArray *c14_e_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData)
{
  const mxArray *c14_mxArrayOutData = NULL;
  int32_T c14_u;
  const mxArray *c14_y = NULL;
  SFc14_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc14_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c14_mxArrayOutData = NULL;
  c14_u = *(int32_T *)c14_inData;
  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", &c14_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c14_mxArrayOutData, c14_y, FALSE);
  return c14_mxArrayOutData;
}

static int32_T c14_f_emlrt_marshallIn(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId)
{
  int32_T c14_y;
  int32_T c14_i39;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_u), &c14_i39, 1, 6, 0U, 0, 0U, 0);
  c14_y = c14_i39;
  sf_mex_destroy(&c14_u);
  return c14_y;
}

static void c14_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData)
{
  const mxArray *c14_b_sfEvent;
  const char_T *c14_identifier;
  emlrtMsgIdentifier c14_thisId;
  int32_T c14_y;
  SFc14_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc14_dynamics_kinematicsInstanceStruct *)chartInstanceVoid;
  c14_b_sfEvent = sf_mex_dup(c14_mxArrayInData);
  c14_identifier = c14_varName;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_y = c14_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_b_sfEvent),
    &c14_thisId);
  sf_mex_destroy(&c14_b_sfEvent);
  *(int32_T *)c14_outData = c14_y;
  sf_mex_destroy(&c14_mxArrayInData);
}

static uint8_T c14_g_emlrt_marshallIn(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c14_b_is_active_c14_dynamics_kinematics, const
  char_T *c14_identifier)
{
  uint8_T c14_y;
  emlrtMsgIdentifier c14_thisId;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_y = c14_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c14_b_is_active_c14_dynamics_kinematics), &c14_thisId);
  sf_mex_destroy(&c14_b_is_active_c14_dynamics_kinematics);
  return c14_y;
}

static uint8_T c14_h_emlrt_marshallIn(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance, const mxArray *c14_u, const emlrtMsgIdentifier *c14_parentId)
{
  uint8_T c14_y;
  uint8_T c14_u0;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_u), &c14_u0, 1, 3, 0U, 0, 0U, 0);
  c14_y = c14_u0;
  sf_mex_destroy(&c14_u);
  return c14_y;
}

static void init_dsm_address_info(SFc14_dynamics_kinematicsInstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c14_dynamics_kinematics_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4278751112U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(798745070U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3480467332U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(987589873U);
}

mxArray *sf_c14_dynamics_kinematics_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("JVrB8SRALQVbHH0sZndbRE");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

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
      pr[0] = (double)(12);
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
      pr[0] = (double)(1);
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
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

static const mxArray *sf_get_sim_state_info_c14_dynamics_kinematics(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x4'type','srcId','name','auxInfo'{{M[1],M[9],T\"alpha\",},{M[1],M[10],T\"force_norm\",},{M[1],M[5],T\"pos_corr\",},{M[8],M[0],T\"is_active_c14_dynamics_kinematics\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 4, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c14_dynamics_kinematics_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc14_dynamics_kinematicsInstanceStruct *chartInstance;
    chartInstance = (SFc14_dynamics_kinematicsInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_dynamics_kinematicsMachineNumber_,
           14,
           1,
           1,
           6,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"pos_err");
          _SFD_SET_DATA_PROPS(1,2,0,1,"pos_corr");
          _SFD_SET_DATA_PROPS(2,1,1,0,"force");
          _SFD_SET_DATA_PROPS(3,1,1,0,"prev_alpha");
          _SFD_SET_DATA_PROPS(4,2,0,1,"alpha");
          _SFD_SET_DATA_PROPS(5,2,0,1,"force_norm");
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
        _SFD_CV_INIT_EML(0,1,1,3,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,610);
        _SFD_CV_INIT_EML_IF(0,1,0,175,193,216,368);
        _SFD_CV_INIT_EML_IF(0,1,1,216,236,260,368);
        _SFD_CV_INIT_EML_IF(0,1,2,260,280,320,368);
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
            1.0,0,0,(MexFcnForType)c14_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c14_b_sf_marshallOut,(MexInFcnForType)
            c14_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c14_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)c14_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)c14_sf_marshallIn);

        {
          real_T *c14_prev_alpha;
          real_T *c14_alpha;
          real_T *c14_force_norm;
          real_T (*c14_pos_err)[6];
          real_T (*c14_pos_corr)[6];
          real_T (*c14_force)[12];
          c14_force_norm = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
          c14_alpha = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c14_prev_alpha = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c14_force = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 1);
          c14_pos_corr = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S,
            1);
          c14_pos_err = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c14_pos_err);
          _SFD_SET_DATA_VALUE_PTR(1U, *c14_pos_corr);
          _SFD_SET_DATA_VALUE_PTR(2U, *c14_force);
          _SFD_SET_DATA_VALUE_PTR(3U, c14_prev_alpha);
          _SFD_SET_DATA_VALUE_PTR(4U, c14_alpha);
          _SFD_SET_DATA_VALUE_PTR(5U, c14_force_norm);
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
  return "pM5BmWlO196eiULqyXTGRH";
}

static void sf_opaque_initialize_c14_dynamics_kinematics(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc14_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c14_dynamics_kinematics
    ((SFc14_dynamics_kinematicsInstanceStruct*) chartInstanceVar);
  initialize_c14_dynamics_kinematics((SFc14_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c14_dynamics_kinematics(void *chartInstanceVar)
{
  enable_c14_dynamics_kinematics((SFc14_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c14_dynamics_kinematics(void *chartInstanceVar)
{
  disable_c14_dynamics_kinematics((SFc14_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c14_dynamics_kinematics(void *chartInstanceVar)
{
  sf_c14_dynamics_kinematics((SFc14_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c14_dynamics_kinematics
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c14_dynamics_kinematics
    ((SFc14_dynamics_kinematicsInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c14_dynamics_kinematics();/* state var info */
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

extern void sf_internal_set_sim_state_c14_dynamics_kinematics(SimStruct* S,
  const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c14_dynamics_kinematics();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c14_dynamics_kinematics((SFc14_dynamics_kinematicsInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c14_dynamics_kinematics(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c14_dynamics_kinematics(S);
}

static void sf_opaque_set_sim_state_c14_dynamics_kinematics(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c14_dynamics_kinematics(S, st);
}

static void sf_opaque_terminate_c14_dynamics_kinematics(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc14_dynamics_kinematicsInstanceStruct*) chartInstanceVar
      )->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c14_dynamics_kinematics((SFc14_dynamics_kinematicsInstanceStruct*)
      chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_dynamics_kinematics_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc14_dynamics_kinematics((SFc14_dynamics_kinematicsInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c14_dynamics_kinematics(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c14_dynamics_kinematics
      ((SFc14_dynamics_kinematicsInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c14_dynamics_kinematics(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_dynamics_kinematics_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      14);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,14,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,14,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,14,3);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,14,3);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,14);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2533050000U));
  ssSetChecksum1(S,(3464151029U));
  ssSetChecksum2(S,(2869927708U));
  ssSetChecksum3(S,(2684484371U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c14_dynamics_kinematics(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c14_dynamics_kinematics(SimStruct *S)
{
  SFc14_dynamics_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc14_dynamics_kinematicsInstanceStruct *)malloc(sizeof
    (SFc14_dynamics_kinematicsInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc14_dynamics_kinematicsInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c14_dynamics_kinematics;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c14_dynamics_kinematics;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c14_dynamics_kinematics;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c14_dynamics_kinematics;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c14_dynamics_kinematics;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c14_dynamics_kinematics;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c14_dynamics_kinematics;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c14_dynamics_kinematics;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c14_dynamics_kinematics;
  chartInstance->chartInfo.mdlStart = mdlStart_c14_dynamics_kinematics;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c14_dynamics_kinematics;
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

void c14_dynamics_kinematics_method_dispatcher(SimStruct *S, int_T method, void *
  data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c14_dynamics_kinematics(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c14_dynamics_kinematics(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c14_dynamics_kinematics(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c14_dynamics_kinematics_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
