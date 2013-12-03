/* Include files */

#include "blascompat32.h"
#include "inverse_kinematics_sfun.h"
#include "c2_inverse_kinematics.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "inverse_kinematics_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c2_debug_family_names[5] = { "newq", "nargin", "nargout",
  "q", "Jacobian_body" };

/* Function Declarations */
static void initialize_c2_inverse_kinematics
  (SFc2_inverse_kinematicsInstanceStruct *chartInstance);
static void initialize_params_c2_inverse_kinematics
  (SFc2_inverse_kinematicsInstanceStruct *chartInstance);
static void enable_c2_inverse_kinematics(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance);
static void disable_c2_inverse_kinematics(SFc2_inverse_kinematicsInstanceStruct *
  chartInstance);
static void c2_update_debugger_state_c2_inverse_kinematics
  (SFc2_inverse_kinematicsInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_inverse_kinematics
  (SFc2_inverse_kinematicsInstanceStruct *chartInstance);
static void set_sim_state_c2_inverse_kinematics
  (SFc2_inverse_kinematicsInstanceStruct *chartInstance, const mxArray *c2_st);
static void finalize_c2_inverse_kinematics(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance);
static void sf_c2_inverse_kinematics(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance);
static void initSimStructsc2_inverse_kinematics
  (SFc2_inverse_kinematicsInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_emlrt_marshallIn(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, real_T
  c2_inData_data[6], int32_T c2_inData_sizes[2]);
static void c2_b_emlrt_marshallIn(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y_data[6], int32_T c2_y_sizes[2]);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real_T c2_outData_data[6],
  int32_T c2_outData_sizes[2]);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_c_emlrt_marshallIn(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[6]);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_d_emlrt_marshallIn(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c2_body_jacobian, const char_T *c2_identifier,
  real_T c2_y[72]);
static void c2_e_emlrt_marshallIn(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[72]);
static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_f_emlrt_marshallIn(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_g_emlrt_marshallIn(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_inverse_kinematics, const
  char_T *c2_identifier);
static uint8_T c2_h_emlrt_marshallIn(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void init_dsm_address_info(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c2_inverse_kinematics
  (SFc2_inverse_kinematicsInstanceStruct *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_is_active_c2_inverse_kinematics = 0U;
}

static void initialize_params_c2_inverse_kinematics
  (SFc2_inverse_kinematicsInstanceStruct *chartInstance)
{
}

static void enable_c2_inverse_kinematics(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_inverse_kinematics(SFc2_inverse_kinematicsInstanceStruct *
  chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_inverse_kinematics
  (SFc2_inverse_kinematicsInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c2_inverse_kinematics
  (SFc2_inverse_kinematicsInstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  int32_T c2_i0;
  real_T c2_u[72];
  const mxArray *c2_b_y = NULL;
  uint8_T c2_hoistedGlobal;
  uint8_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  real_T (*c2_Jacobian_body)[72];
  c2_Jacobian_body = (real_T (*)[72])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(2), FALSE);
  for (c2_i0 = 0; c2_i0 < 72; c2_i0++) {
    c2_u[c2_i0] = (*c2_Jacobian_body)[c2_i0];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 6, 12),
                FALSE);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_hoistedGlobal = chartInstance->c2_is_active_c2_inverse_kinematics;
  c2_b_u = c2_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  sf_mex_assign(&c2_st, c2_y, FALSE);
  return c2_st;
}

static void set_sim_state_c2_inverse_kinematics
  (SFc2_inverse_kinematicsInstanceStruct *chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv0[72];
  int32_T c2_i1;
  real_T (*c2_Jacobian_body)[72];
  c2_Jacobian_body = (real_T (*)[72])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_u = sf_mex_dup(c2_st);
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 0)),
                        "Jacobian_body", c2_dv0);
  for (c2_i1 = 0; c2_i1 < 72; c2_i1++) {
    (*c2_Jacobian_body)[c2_i1] = c2_dv0[c2_i1];
  }

  chartInstance->c2_is_active_c2_inverse_kinematics = c2_g_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 1)),
     "is_active_c2_inverse_kinematics");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_inverse_kinematics(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_inverse_kinematics(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance)
{
}

static void sf_c2_inverse_kinematics(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance)
{
  int32_T c2_i2;
  int32_T c2_i3;
  int32_T c2_i4;
  real_T c2_q[6];
  uint32_T c2_debug_family_var_map[5];
  real_T c2_newq[6];
  int32_T c2_newq_sizes[2];
  real_T c2_newq_data[6];
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  real_T c2_Jacobian_body[72];
  int32_T c2_i5;
  int32_T c2_i6;
  int32_T c2_b_newq;
  int32_T c2_c_newq;
  int32_T c2_i7;
  int32_T c2_u_sizes[2];
  int32_T c2_u;
  int32_T c2_b_u;
  int32_T c2_loop_ub;
  int32_T c2_i8;
  real_T c2_u_data[6];
  const mxArray *c2_y = NULL;
  real_T c2_dv1[72];
  int32_T c2_i9;
  int32_T c2_i10;
  real_T (*c2_b_Jacobian_body)[72];
  real_T (*c2_b_q)[6];
  c2_b_Jacobian_body = (real_T (*)[72])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_b_q = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  for (c2_i2 = 0; c2_i2 < 6; c2_i2++) {
    _SFD_DATA_RANGE_CHECK((*c2_b_q)[c2_i2], 0U);
  }

  for (c2_i3 = 0; c2_i3 < 72; c2_i3++) {
    _SFD_DATA_RANGE_CHECK((*c2_b_Jacobian_body)[c2_i3], 1U);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  for (c2_i4 = 0; c2_i4 < 6; c2_i4++) {
    c2_q[c2_i4] = (*c2_b_q)[c2_i4];
  }

  sf_debug_symbol_scope_push_eml(0U, 5U, 6U, c2_debug_family_names,
    c2_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(c2_newq, MAX_uint32_T,
    c2_e_sf_marshallOut, c2_d_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_dyn_importable(c2_newq_data, (const int32_T *)
    &c2_newq_sizes, NULL, 0, -1, (void *)c2_d_sf_marshallOut, (void *)
    c2_c_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargin, 1U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c2_nargout, 2U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c2_q, 3U, c2_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c2_Jacobian_body, 4U,
    c2_sf_marshallOut, c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 3);
  for (c2_i5 = 0; c2_i5 < 72; c2_i5++) {
    c2_Jacobian_body[c2_i5] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 4);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 5);
  for (c2_i6 = 0; c2_i6 < 6; c2_i6++) {
    c2_newq[c2_i6] = 0.0;
  }

  sf_debug_symbol_switch(0U, 0U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 6);
  c2_newq_sizes[0] = 6;
  c2_newq_sizes[1] = 1;
  c2_b_newq = c2_newq_sizes[0];
  c2_c_newq = c2_newq_sizes[1];
  for (c2_i7 = 0; c2_i7 < 6; c2_i7++) {
    c2_newq_data[c2_i7] = c2_q[c2_i7];
  }

  sf_debug_symbol_switch(0U, 1U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 7);
  c2_u_sizes[0] = 6;
  c2_u_sizes[1] = 1;
  c2_u = c2_u_sizes[0];
  c2_b_u = c2_u_sizes[1];
  c2_loop_ub = c2_newq_sizes[0] * c2_newq_sizes[1] - 1;
  for (c2_i8 = 0; c2_i8 <= c2_loop_ub; c2_i8++) {
    c2_u_data[c2_i8] = c2_newq_data[c2_i8];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u_data, 0, 0U, 1U, 0U, 2,
    c2_u_sizes[0], c2_u_sizes[1]), FALSE);
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_call_debug("body_jacobian", 1U, 1U,
    14, c2_y), "body_jacobian", c2_dv1);
  for (c2_i9 = 0; c2_i9 < 72; c2_i9++) {
    c2_Jacobian_body[c2_i9] = c2_dv1[c2_i9];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -7);
  sf_debug_symbol_scope_pop();
  for (c2_i10 = 0; c2_i10 < 72; c2_i10++) {
    (*c2_b_Jacobian_body)[c2_i10] = c2_Jacobian_body[c2_i10];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  sf_debug_check_for_state_inconsistency(_inverse_kinematicsMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc2_inverse_kinematics
  (SFc2_inverse_kinematicsInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i11;
  int32_T c2_i12;
  int32_T c2_i13;
  real_T c2_b_inData[72];
  int32_T c2_i14;
  int32_T c2_i15;
  int32_T c2_i16;
  real_T c2_u[72];
  const mxArray *c2_y = NULL;
  SFc2_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc2_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i11 = 0;
  for (c2_i12 = 0; c2_i12 < 12; c2_i12++) {
    for (c2_i13 = 0; c2_i13 < 6; c2_i13++) {
      c2_b_inData[c2_i13 + c2_i11] = (*(real_T (*)[72])c2_inData)[c2_i13 +
        c2_i11];
    }

    c2_i11 += 6;
  }

  c2_i14 = 0;
  for (c2_i15 = 0; c2_i15 < 12; c2_i15++) {
    for (c2_i16 = 0; c2_i16 < 6; c2_i16++) {
      c2_u[c2_i16 + c2_i14] = c2_b_inData[c2_i16 + c2_i14];
    }

    c2_i14 += 6;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 6, 12), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_body_jacobian;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[72];
  int32_T c2_i17;
  int32_T c2_i18;
  int32_T c2_i19;
  SFc2_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc2_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c2_body_jacobian = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_body_jacobian), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_body_jacobian);
  c2_i17 = 0;
  for (c2_i18 = 0; c2_i18 < 12; c2_i18++) {
    for (c2_i19 = 0; c2_i19 < 6; c2_i19++) {
      (*(real_T (*)[72])c2_outData)[c2_i19 + c2_i17] = c2_y[c2_i19 + c2_i17];
    }

    c2_i17 += 6;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i20;
  real_T c2_b_inData[6];
  int32_T c2_i21;
  real_T c2_u[6];
  const mxArray *c2_y = NULL;
  SFc2_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc2_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i20 = 0; c2_i20 < 6; c2_i20++) {
    c2_b_inData[c2_i20] = (*(real_T (*)[6])c2_inData)[c2_i20];
  }

  for (c2_i21 = 0; c2_i21 < 6; c2_i21++) {
    c2_u[c2_i21] = c2_b_inData[c2_i21];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc2_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_emlrt_marshallIn(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d0, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_nargout;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc2_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c2_nargout = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_nargout), &c2_thisId);
  sf_mex_destroy(&c2_nargout);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, real_T
  c2_inData_data[6], int32_T c2_inData_sizes[2])
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_b_inData_sizes[2];
  int32_T c2_i22;
  real_T c2_b_inData_data[6];
  int32_T c2_u_sizes[2];
  int32_T c2_i23;
  real_T c2_u_data[6];
  const mxArray *c2_y = NULL;
  SFc2_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc2_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_b_inData_sizes[0] = 6;
  c2_b_inData_sizes[1] = 1;
  for (c2_i22 = 0; c2_i22 < 6; c2_i22++) {
    c2_b_inData_data[c2_i22] = c2_inData_data[c2_i22];
  }

  c2_u_sizes[0] = 6;
  c2_u_sizes[1] = 1;
  for (c2_i23 = 0; c2_i23 < 6; c2_i23++) {
    c2_u_data[c2_i23] = c2_b_inData_data[c2_i23];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u_data, 0, 0U, 1U, 0U, 2,
    c2_u_sizes[0], c2_u_sizes[1]), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_b_emlrt_marshallIn(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y_data[6], int32_T c2_y_sizes[2])
{
  int32_T c2_i24;
  static uint32_T c2_uv0[2] = { 6U, 1U };

  uint32_T c2_uv1[2];
  int32_T c2_i25;
  boolean_T c2_bv0[2];
  int32_T c2_tmp_sizes[2];
  real_T c2_tmp_data[6];
  int32_T c2_y;
  int32_T c2_b_y;
  int32_T c2_loop_ub;
  int32_T c2_i26;
  for (c2_i24 = 0; c2_i24 < 2; c2_i24++) {
    c2_uv1[c2_i24] = c2_uv0[c2_i24];
  }

  for (c2_i25 = 0; c2_i25 < 2; c2_i25++) {
    c2_bv0[c2_i25] = FALSE;
  }

  sf_mex_import_vs(c2_parentId, sf_mex_dup(c2_u), c2_tmp_data, 1, 0, 0U, 1, 0U,
                   2, c2_bv0, c2_uv1, c2_tmp_sizes);
  c2_y_sizes[0] = 6;
  c2_y_sizes[1] = 1;
  c2_y = c2_y_sizes[0];
  c2_b_y = c2_y_sizes[1];
  c2_loop_ub = c2_tmp_sizes[0] * c2_tmp_sizes[1] - 1;
  for (c2_i26 = 0; c2_i26 <= c2_loop_ub; c2_i26++) {
    c2_y_data[c2_i26] = c2_tmp_data[c2_i26];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real_T c2_outData_data[6],
  int32_T c2_outData_sizes[2])
{
  const mxArray *c2_newq;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y_sizes[2];
  real_T c2_y_data[6];
  int32_T c2_i27;
  SFc2_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc2_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c2_newq = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_newq), &c2_thisId,
                        c2_y_data, c2_y_sizes);
  sf_mex_destroy(&c2_newq);
  c2_outData_sizes[0] = 6;
  c2_outData_sizes[1] = 1;
  for (c2_i27 = 0; c2_i27 < 6; c2_i27++) {
    c2_outData_data[c2_i27] = c2_y_data[c2_i27];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i28;
  real_T c2_b_inData[6];
  int32_T c2_i29;
  real_T c2_u[6];
  const mxArray *c2_y = NULL;
  SFc2_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc2_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i28 = 0; c2_i28 < 6; c2_i28++) {
    c2_b_inData[c2_i28] = (*(real_T (*)[6])c2_inData)[c2_i28];
  }

  for (c2_i29 = 0; c2_i29 < 6; c2_i29++) {
    c2_u[c2_i29] = c2_b_inData[c2_i29];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 6), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_c_emlrt_marshallIn(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[6])
{
  real_T c2_dv2[6];
  int32_T c2_i30;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv2, 1, 0, 0U, 1, 0U, 2, 1, 6);
  for (c2_i30 = 0; c2_i30 < 6; c2_i30++) {
    c2_y[c2_i30] = c2_dv2[c2_i30];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_newq;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[6];
  int32_T c2_i31;
  SFc2_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc2_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c2_newq = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_newq), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_newq);
  for (c2_i31 = 0; c2_i31 < 6; c2_i31++) {
    (*(real_T (*)[6])c2_outData)[c2_i31] = c2_y[c2_i31];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray *sf_c2_inverse_kinematics_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  sf_mex_assign(&c2_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), FALSE);
  return c2_nameCaptureInfo;
}

static void c2_d_emlrt_marshallIn(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c2_body_jacobian, const char_T *c2_identifier,
  real_T c2_y[72])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_body_jacobian), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_body_jacobian);
}

static void c2_e_emlrt_marshallIn(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[72])
{
  real_T c2_dv3[72];
  int32_T c2_i32;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv3, 1, 0, 0U, 1, 0U, 2, 6, 12);
  for (c2_i32 = 0; c2_i32 < 72; c2_i32++) {
    c2_y[c2_i32] = c2_dv3[c2_i32];
  }

  sf_mex_destroy(&c2_u);
}

static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc2_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static int32_T c2_f_emlrt_marshallIn(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i33;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i33, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i33;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc2_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_g_emlrt_marshallIn(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_inverse_kinematics, const
  char_T *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_inverse_kinematics), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_inverse_kinematics);
  return c2_y;
}

static uint8_T c2_h_emlrt_marshallIn(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void init_dsm_address_info(SFc2_inverse_kinematicsInstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c2_inverse_kinematics_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3764745182U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(119290071U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3378025072U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(437032479U);
}

mxArray *sf_c2_inverse_kinematics_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("Z52LXaXMKTwxlEZtdeyNCC");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
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

static const mxArray *sf_get_sim_state_info_c2_inverse_kinematics(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"Jacobian_body\",},{M[8],M[0],T\"is_active_c2_inverse_kinematics\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_inverse_kinematics_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_inverse_kinematicsInstanceStruct *chartInstance;
    chartInstance = (SFc2_inverse_kinematicsInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_inverse_kinematicsMachineNumber_,
           2,
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
          init_script_number_translation(_inverse_kinematicsMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (_inverse_kinematicsMachineNumber_,chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(_inverse_kinematicsMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"q");
          _SFD_SET_DATA_PROPS(1,2,0,1,"Jacobian_body");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,190);
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
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 6;
          dimVector[1]= 12;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)
            c2_sf_marshallIn);
        }

        {
          real_T (*c2_q)[6];
          real_T (*c2_Jacobian_body)[72];
          c2_Jacobian_body = (real_T (*)[72])ssGetOutputPortSignal
            (chartInstance->S, 1);
          c2_q = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c2_q);
          _SFD_SET_DATA_VALUE_PTR(1U, *c2_Jacobian_body);
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
  return "gPLgDVrQMlUCdJbySkqEvD";
}

static void sf_opaque_initialize_c2_inverse_kinematics(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_inverse_kinematicsInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c2_inverse_kinematics((SFc2_inverse_kinematicsInstanceStruct*)
    chartInstanceVar);
  initialize_c2_inverse_kinematics((SFc2_inverse_kinematicsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c2_inverse_kinematics(void *chartInstanceVar)
{
  enable_c2_inverse_kinematics((SFc2_inverse_kinematicsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c2_inverse_kinematics(void *chartInstanceVar)
{
  disable_c2_inverse_kinematics((SFc2_inverse_kinematicsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c2_inverse_kinematics(void *chartInstanceVar)
{
  sf_c2_inverse_kinematics((SFc2_inverse_kinematicsInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c2_inverse_kinematics(SimStruct*
  S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_inverse_kinematics
    ((SFc2_inverse_kinematicsInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_inverse_kinematics();/* state var info */
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

extern void sf_internal_set_sim_state_c2_inverse_kinematics(SimStruct* S, const
  mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_inverse_kinematics();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_inverse_kinematics((SFc2_inverse_kinematicsInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c2_inverse_kinematics(SimStruct* S)
{
  return sf_internal_get_sim_state_c2_inverse_kinematics(S);
}

static void sf_opaque_set_sim_state_c2_inverse_kinematics(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c2_inverse_kinematics(S, st);
}

static void sf_opaque_terminate_c2_inverse_kinematics(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_inverse_kinematicsInstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c2_inverse_kinematics((SFc2_inverse_kinematicsInstanceStruct*)
      chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_inverse_kinematics_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_inverse_kinematics((SFc2_inverse_kinematicsInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_inverse_kinematics(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_inverse_kinematics
      ((SFc2_inverse_kinematicsInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_inverse_kinematics(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_inverse_kinematics_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,2,
      "gatewayCannotBeInlinedMultipleTimes"));
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,1);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,1);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2664716208U));
  ssSetChecksum1(S,(2510022313U));
  ssSetChecksum2(S,(1488910166U));
  ssSetChecksum3(S,(1043936214U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c2_inverse_kinematics(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_inverse_kinematics(SimStruct *S)
{
  SFc2_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc2_inverse_kinematicsInstanceStruct *)malloc(sizeof
    (SFc2_inverse_kinematicsInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_inverse_kinematicsInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_inverse_kinematics;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_inverse_kinematics;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_inverse_kinematics;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_inverse_kinematics;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c2_inverse_kinematics;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_inverse_kinematics;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_inverse_kinematics;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_inverse_kinematics;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_inverse_kinematics;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_inverse_kinematics;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_inverse_kinematics;
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

void c2_inverse_kinematics_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_inverse_kinematics(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_inverse_kinematics(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_inverse_kinematics(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_inverse_kinematics_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
