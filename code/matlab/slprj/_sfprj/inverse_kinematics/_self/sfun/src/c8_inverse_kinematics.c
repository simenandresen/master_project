/* Include files */

#include "blascompat32.h"
#include "inverse_kinematics_sfun.h"
#include "c8_inverse_kinematics.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "inverse_kinematics_sfun_debug_macros.h"

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c8_debug_family_names[5] = { "nargin", "nargout",
  "ee_velocity", "ee_position", "ee_velocity_environment" };

/* Function Declarations */
static void initialize_c8_inverse_kinematics
  (SFc8_inverse_kinematicsInstanceStruct *chartInstance);
static void initialize_params_c8_inverse_kinematics
  (SFc8_inverse_kinematicsInstanceStruct *chartInstance);
static void enable_c8_inverse_kinematics(SFc8_inverse_kinematicsInstanceStruct
  *chartInstance);
static void disable_c8_inverse_kinematics(SFc8_inverse_kinematicsInstanceStruct *
  chartInstance);
static void c8_update_debugger_state_c8_inverse_kinematics
  (SFc8_inverse_kinematicsInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c8_inverse_kinematics
  (SFc8_inverse_kinematicsInstanceStruct *chartInstance);
static void set_sim_state_c8_inverse_kinematics
  (SFc8_inverse_kinematicsInstanceStruct *chartInstance, const mxArray *c8_st);
static void finalize_c8_inverse_kinematics(SFc8_inverse_kinematicsInstanceStruct
  *chartInstance);
static void sf_c8_inverse_kinematics(SFc8_inverse_kinematicsInstanceStruct
  *chartInstance);
static void initSimStructsc8_inverse_kinematics
  (SFc8_inverse_kinematicsInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c8_machineNumber, uint32_T
  c8_chartNumber);
static const mxArray *c8_sf_marshallOut(void *chartInstanceVoid, void *c8_inData);
static void c8_emlrt_marshallIn(SFc8_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_ee_velocity_environment, const char_T
  *c8_identifier, real_T c8_y[6]);
static void c8_b_emlrt_marshallIn(SFc8_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[6]);
static void c8_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_b_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static const mxArray *c8_c_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static real_T c8_c_emlrt_marshallIn(SFc8_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void c8_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_d_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static int32_T c8_d_emlrt_marshallIn(SFc8_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void c8_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static uint8_T c8_e_emlrt_marshallIn(SFc8_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_b_is_active_c8_inverse_kinematics, const
  char_T *c8_identifier);
static uint8_T c8_f_emlrt_marshallIn(SFc8_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void init_dsm_address_info(SFc8_inverse_kinematicsInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c8_inverse_kinematics
  (SFc8_inverse_kinematicsInstanceStruct *chartInstance)
{
  chartInstance->c8_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c8_is_active_c8_inverse_kinematics = 0U;
}

static void initialize_params_c8_inverse_kinematics
  (SFc8_inverse_kinematicsInstanceStruct *chartInstance)
{
}

static void enable_c8_inverse_kinematics(SFc8_inverse_kinematicsInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c8_inverse_kinematics(SFc8_inverse_kinematicsInstanceStruct *
  chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c8_update_debugger_state_c8_inverse_kinematics
  (SFc8_inverse_kinematicsInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c8_inverse_kinematics
  (SFc8_inverse_kinematicsInstanceStruct *chartInstance)
{
  const mxArray *c8_st;
  const mxArray *c8_y = NULL;
  int32_T c8_i0;
  real_T c8_u[6];
  const mxArray *c8_b_y = NULL;
  uint8_T c8_hoistedGlobal;
  uint8_T c8_b_u;
  const mxArray *c8_c_y = NULL;
  real_T (*c8_ee_velocity_environment)[6];
  c8_ee_velocity_environment = (real_T (*)[6])ssGetOutputPortSignal
    (chartInstance->S, 1);
  c8_st = NULL;
  c8_st = NULL;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_createcellarray(2), FALSE);
  for (c8_i0 = 0; c8_i0 < 6; c8_i0++) {
    c8_u[c8_i0] = (*c8_ee_velocity_environment)[c8_i0];
  }

  c8_b_y = NULL;
  sf_mex_assign(&c8_b_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_setcell(c8_y, 0, c8_b_y);
  c8_hoistedGlobal = chartInstance->c8_is_active_c8_inverse_kinematics;
  c8_b_u = c8_hoistedGlobal;
  c8_c_y = NULL;
  sf_mex_assign(&c8_c_y, sf_mex_create("y", &c8_b_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c8_y, 1, c8_c_y);
  sf_mex_assign(&c8_st, c8_y, FALSE);
  return c8_st;
}

static void set_sim_state_c8_inverse_kinematics
  (SFc8_inverse_kinematicsInstanceStruct *chartInstance, const mxArray *c8_st)
{
  const mxArray *c8_u;
  real_T c8_dv0[6];
  int32_T c8_i1;
  real_T (*c8_ee_velocity_environment)[6];
  c8_ee_velocity_environment = (real_T (*)[6])ssGetOutputPortSignal
    (chartInstance->S, 1);
  chartInstance->c8_doneDoubleBufferReInit = TRUE;
  c8_u = sf_mex_dup(c8_st);
  c8_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c8_u, 0)),
                      "ee_velocity_environment", c8_dv0);
  for (c8_i1 = 0; c8_i1 < 6; c8_i1++) {
    (*c8_ee_velocity_environment)[c8_i1] = c8_dv0[c8_i1];
  }

  chartInstance->c8_is_active_c8_inverse_kinematics = c8_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c8_u, 1)),
     "is_active_c8_inverse_kinematics");
  sf_mex_destroy(&c8_u);
  c8_update_debugger_state_c8_inverse_kinematics(chartInstance);
  sf_mex_destroy(&c8_st);
}

static void finalize_c8_inverse_kinematics(SFc8_inverse_kinematicsInstanceStruct
  *chartInstance)
{
}

static void sf_c8_inverse_kinematics(SFc8_inverse_kinematicsInstanceStruct
  *chartInstance)
{
  int32_T c8_i2;
  int32_T c8_i3;
  int32_T c8_i4;
  int32_T c8_i5;
  real_T c8_ee_velocity[6];
  int32_T c8_i6;
  real_T c8_ee_position[3];
  uint32_T c8_debug_family_var_map[5];
  real_T c8_nargin = 2.0;
  real_T c8_nargout = 1.0;
  real_T c8_ee_velocity_environment[6];
  int32_T c8_i7;
  int32_T c8_i8;
  real_T (*c8_b_ee_velocity_environment)[6];
  real_T (*c8_b_ee_position)[3];
  real_T (*c8_b_ee_velocity)[6];
  c8_b_ee_velocity_environment = (real_T (*)[6])ssGetOutputPortSignal
    (chartInstance->S, 1);
  c8_b_ee_position = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c8_b_ee_velocity = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S, 0);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 7U, chartInstance->c8_sfEvent);
  for (c8_i2 = 0; c8_i2 < 6; c8_i2++) {
    _SFD_DATA_RANGE_CHECK((*c8_b_ee_velocity)[c8_i2], 0U);
  }

  for (c8_i3 = 0; c8_i3 < 3; c8_i3++) {
    _SFD_DATA_RANGE_CHECK((*c8_b_ee_position)[c8_i3], 1U);
  }

  for (c8_i4 = 0; c8_i4 < 6; c8_i4++) {
    _SFD_DATA_RANGE_CHECK((*c8_b_ee_velocity_environment)[c8_i4], 2U);
  }

  chartInstance->c8_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 7U, chartInstance->c8_sfEvent);
  for (c8_i5 = 0; c8_i5 < 6; c8_i5++) {
    c8_ee_velocity[c8_i5] = (*c8_b_ee_velocity)[c8_i5];
  }

  for (c8_i6 = 0; c8_i6 < 3; c8_i6++) {
    c8_ee_position[c8_i6] = (*c8_b_ee_position)[c8_i6];
  }

  sf_debug_symbol_scope_push_eml(0U, 5U, 5U, c8_debug_family_names,
    c8_debug_family_var_map);
  sf_debug_symbol_scope_add_eml_importable(&c8_nargin, 0U, c8_c_sf_marshallOut,
    c8_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml_importable(&c8_nargout, 1U, c8_c_sf_marshallOut,
    c8_b_sf_marshallIn);
  sf_debug_symbol_scope_add_eml(c8_ee_velocity, 2U, c8_sf_marshallOut);
  sf_debug_symbol_scope_add_eml(c8_ee_position, 3U, c8_b_sf_marshallOut);
  sf_debug_symbol_scope_add_eml_importable(c8_ee_velocity_environment, 4U,
    c8_sf_marshallOut, c8_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 3);
  for (c8_i7 = 0; c8_i7 < 6; c8_i7++) {
    c8_ee_velocity_environment[c8_i7] = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, -3);
  sf_debug_symbol_scope_pop();
  for (c8_i8 = 0; c8_i8 < 6; c8_i8++) {
    (*c8_b_ee_velocity_environment)[c8_i8] = c8_ee_velocity_environment[c8_i8];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 7U, chartInstance->c8_sfEvent);
  sf_debug_check_for_state_inconsistency(_inverse_kinematicsMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc8_inverse_kinematics
  (SFc8_inverse_kinematicsInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c8_machineNumber, uint32_T
  c8_chartNumber)
{
}

static const mxArray *c8_sf_marshallOut(void *chartInstanceVoid, void *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i9;
  real_T c8_b_inData[6];
  int32_T c8_i10;
  real_T c8_u[6];
  const mxArray *c8_y = NULL;
  SFc8_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  for (c8_i9 = 0; c8_i9 < 6; c8_i9++) {
    c8_b_inData[c8_i9] = (*(real_T (*)[6])c8_inData)[c8_i9];
  }

  for (c8_i10 = 0; c8_i10 < 6; c8_i10++) {
    c8_u[c8_i10] = c8_b_inData[c8_i10];
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 1, 6), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static void c8_emlrt_marshallIn(SFc8_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_ee_velocity_environment, const char_T
  *c8_identifier, real_T c8_y[6])
{
  emlrtMsgIdentifier c8_thisId;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_ee_velocity_environment),
                        &c8_thisId, c8_y);
  sf_mex_destroy(&c8_ee_velocity_environment);
}

static void c8_b_emlrt_marshallIn(SFc8_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[6])
{
  real_T c8_dv1[6];
  int32_T c8_i11;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv1, 1, 0, 0U, 1, 0U, 1, 6);
  for (c8_i11 = 0; c8_i11 < 6; c8_i11++) {
    c8_y[c8_i11] = c8_dv1[c8_i11];
  }

  sf_mex_destroy(&c8_u);
}

static void c8_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_ee_velocity_environment;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y[6];
  int32_T c8_i12;
  SFc8_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_ee_velocity_environment = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_ee_velocity_environment),
                        &c8_thisId, c8_y);
  sf_mex_destroy(&c8_ee_velocity_environment);
  for (c8_i12 = 0; c8_i12 < 6; c8_i12++) {
    (*(real_T (*)[6])c8_outData)[c8_i12] = c8_y[c8_i12];
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

static const mxArray *c8_b_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i13;
  real_T c8_b_inData[3];
  int32_T c8_i14;
  real_T c8_u[3];
  const mxArray *c8_y = NULL;
  SFc8_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  for (c8_i13 = 0; c8_i13 < 3; c8_i13++) {
    c8_b_inData[c8_i13] = (*(real_T (*)[3])c8_inData)[c8_i13];
  }

  for (c8_i14 = 0; c8_i14 < 3; c8_i14++) {
    c8_u[c8_i14] = c8_b_inData[c8_i14];
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static const mxArray *c8_c_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  real_T c8_u;
  const mxArray *c8_y = NULL;
  SFc8_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_u = *(real_T *)c8_inData;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", &c8_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static real_T c8_c_emlrt_marshallIn(SFc8_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  real_T c8_y;
  real_T c8_d0;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_d0, 1, 0, 0U, 0, 0U, 0);
  c8_y = c8_d0;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void c8_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_nargout;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y;
  SFc8_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_nargout = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_nargout), &c8_thisId);
  sf_mex_destroy(&c8_nargout);
  *(real_T *)c8_outData = c8_y;
  sf_mex_destroy(&c8_mxArrayInData);
}

const mxArray *sf_c8_inverse_kinematics_get_eml_resolved_functions_info(void)
{
  const mxArray *c8_nameCaptureInfo = NULL;
  c8_nameCaptureInfo = NULL;
  sf_mex_assign(&c8_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), FALSE);
  return c8_nameCaptureInfo;
}

static const mxArray *c8_d_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_u;
  const mxArray *c8_y = NULL;
  SFc8_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_u = *(int32_T *)c8_inData;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", &c8_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, FALSE);
  return c8_mxArrayOutData;
}

static int32_T c8_d_emlrt_marshallIn(SFc8_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  int32_T c8_y;
  int32_T c8_i15;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_i15, 1, 6, 0U, 0, 0U, 0);
  c8_y = c8_i15;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void c8_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_b_sfEvent;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  int32_T c8_y;
  SFc8_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_inverse_kinematicsInstanceStruct *)chartInstanceVoid;
  c8_b_sfEvent = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_b_sfEvent),
    &c8_thisId);
  sf_mex_destroy(&c8_b_sfEvent);
  *(int32_T *)c8_outData = c8_y;
  sf_mex_destroy(&c8_mxArrayInData);
}

static uint8_T c8_e_emlrt_marshallIn(SFc8_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_b_is_active_c8_inverse_kinematics, const
  char_T *c8_identifier)
{
  uint8_T c8_y;
  emlrtMsgIdentifier c8_thisId;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c8_b_is_active_c8_inverse_kinematics), &c8_thisId);
  sf_mex_destroy(&c8_b_is_active_c8_inverse_kinematics);
  return c8_y;
}

static uint8_T c8_f_emlrt_marshallIn(SFc8_inverse_kinematicsInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  uint8_T c8_y;
  uint8_T c8_u0;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_u0, 1, 3, 0U, 0, 0U, 0);
  c8_y = c8_u0;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void init_dsm_address_info(SFc8_inverse_kinematicsInstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
void sf_c8_inverse_kinematics_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2629039518U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(526718987U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2800858120U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1615472967U);
}

mxArray *sf_c8_inverse_kinematics_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("I0bLLkePTmM90QmOhJCDQG");
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

static const mxArray *sf_get_sim_state_info_c8_inverse_kinematics(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[9],T\"ee_velocity_environment\",},{M[8],M[0],T\"is_active_c8_inverse_kinematics\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c8_inverse_kinematics_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc8_inverse_kinematicsInstanceStruct *chartInstance;
    chartInstance = (SFc8_inverse_kinematicsInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (_inverse_kinematicsMachineNumber_,
           8,
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
          _SFD_SET_DATA_PROPS(1,1,1,0,"ee_position");
          _SFD_SET_DATA_PROPS(2,2,0,1,"ee_velocity_environment");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,145);
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
            1.0,0,0,(MexFcnForType)c8_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_sf_marshallOut,(MexInFcnForType)
            c8_sf_marshallIn);
        }

        {
          real_T (*c8_ee_velocity)[6];
          real_T (*c8_ee_position)[3];
          real_T (*c8_ee_velocity_environment)[6];
          c8_ee_velocity_environment = (real_T (*)[6])ssGetOutputPortSignal
            (chartInstance->S, 1);
          c8_ee_position = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S,
            1);
          c8_ee_velocity = (real_T (*)[6])ssGetInputPortSignal(chartInstance->S,
            0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c8_ee_velocity);
          _SFD_SET_DATA_VALUE_PTR(1U, *c8_ee_position);
          _SFD_SET_DATA_VALUE_PTR(2U, *c8_ee_velocity_environment);
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
  return "z7V3LJzfxWzKUlCgwlTQZC";
}

static void sf_opaque_initialize_c8_inverse_kinematics(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc8_inverse_kinematicsInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c8_inverse_kinematics((SFc8_inverse_kinematicsInstanceStruct*)
    chartInstanceVar);
  initialize_c8_inverse_kinematics((SFc8_inverse_kinematicsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c8_inverse_kinematics(void *chartInstanceVar)
{
  enable_c8_inverse_kinematics((SFc8_inverse_kinematicsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c8_inverse_kinematics(void *chartInstanceVar)
{
  disable_c8_inverse_kinematics((SFc8_inverse_kinematicsInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c8_inverse_kinematics(void *chartInstanceVar)
{
  sf_c8_inverse_kinematics((SFc8_inverse_kinematicsInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c8_inverse_kinematics(SimStruct*
  S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c8_inverse_kinematics
    ((SFc8_inverse_kinematicsInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c8_inverse_kinematics();/* state var info */
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

extern void sf_internal_set_sim_state_c8_inverse_kinematics(SimStruct* S, const
  mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c8_inverse_kinematics();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c8_inverse_kinematics((SFc8_inverse_kinematicsInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c8_inverse_kinematics(SimStruct* S)
{
  return sf_internal_get_sim_state_c8_inverse_kinematics(S);
}

static void sf_opaque_set_sim_state_c8_inverse_kinematics(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c8_inverse_kinematics(S, st);
}

static void sf_opaque_terminate_c8_inverse_kinematics(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc8_inverse_kinematicsInstanceStruct*) chartInstanceVar)
      ->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
    }

    finalize_c8_inverse_kinematics((SFc8_inverse_kinematicsInstanceStruct*)
      chartInstanceVar);
    free((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }

  unload_inverse_kinematics_optimization_info();
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc8_inverse_kinematics((SFc8_inverse_kinematicsInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c8_inverse_kinematics(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c8_inverse_kinematics
      ((SFc8_inverse_kinematicsInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c8_inverse_kinematics(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_inverse_kinematics_optimization_info();
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
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,8,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,8,1);
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,8);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(268381138U));
  ssSetChecksum1(S,(1573825178U));
  ssSetChecksum2(S,(2493018856U));
  ssSetChecksum3(S,(21207131U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
}

static void mdlRTW_c8_inverse_kinematics(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c8_inverse_kinematics(SimStruct *S)
{
  SFc8_inverse_kinematicsInstanceStruct *chartInstance;
  chartInstance = (SFc8_inverse_kinematicsInstanceStruct *)malloc(sizeof
    (SFc8_inverse_kinematicsInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc8_inverse_kinematicsInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c8_inverse_kinematics;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c8_inverse_kinematics;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c8_inverse_kinematics;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c8_inverse_kinematics;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c8_inverse_kinematics;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c8_inverse_kinematics;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c8_inverse_kinematics;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c8_inverse_kinematics;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c8_inverse_kinematics;
  chartInstance->chartInfo.mdlStart = mdlStart_c8_inverse_kinematics;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c8_inverse_kinematics;
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

void c8_inverse_kinematics_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c8_inverse_kinematics(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c8_inverse_kinematics(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c8_inverse_kinematics(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c8_inverse_kinematics_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
