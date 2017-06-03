/* Include files */

#include <stddef.h>
#include "blas.h"
#include "sixDOF_SIM_sfun.h"
#include "c9_sixDOF_SIM.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "sixDOF_SIM_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c_with_debugger(S, sfGlobalDebugInstanceStruct);

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization);
static void chart_debug_initialize_data_addresses(SimStruct *S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c9_debug_family_names[9] = { "nargin", "nargout", "mu", "I",
  "R", "cx", "cy", "cz", "M" };

static const char * c9_b_debug_family_names[9] = { "nargin", "nargout", "mu",
  "I", "R", "cx", "cy", "cz", "M" };

/* Function Declarations */
static void initialize_c9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct
  *chartInstance);
static void initialize_params_c9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct
  *chartInstance);
static void enable_c9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct *chartInstance);
static void disable_c9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct *chartInstance);
static void c9_update_debugger_state_c9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct *
  chartInstance);
static void set_sim_state_c9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct
  *chartInstance, const mxArray *c9_st);
static void finalize_c9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct *chartInstance);
static void sf_gateway_c9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct
  *chartInstance);
static void mdl_start_c9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct *chartInstance);
static void initSimStructsc9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c9_machineNumber, uint32_T
  c9_chartNumber, uint32_T c9_instanceNumber);
static const mxArray *c9_sf_marshallOut(void *chartInstanceVoid, void *c9_inData);
static void c9_emlrt_marshallIn(SFc9_sixDOF_SIMInstanceStruct *chartInstance,
  const mxArray *c9_b_M, const char_T *c9_identifier, real_T c9_y[3]);
static void c9_b_emlrt_marshallIn(SFc9_sixDOF_SIMInstanceStruct *chartInstance,
  const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId, real_T c9_y[3]);
static void c9_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static const mxArray *c9_b_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData);
static const mxArray *c9_c_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData);
static real_T c9_c_emlrt_marshallIn(SFc9_sixDOF_SIMInstanceStruct *chartInstance,
  const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId);
static void c9_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static void c9_d_emlrt_marshallIn(SFc9_sixDOF_SIMInstanceStruct *chartInstance,
  const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId, real_T c9_y[9]);
static void c9_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static void c9_info_helper(const mxArray **c9_info);
static const mxArray *c9_emlrt_marshallOut(const char * c9_u);
static const mxArray *c9_b_emlrt_marshallOut(const uint32_T c9_u);
static void c9_eml_scalar_eg(SFc9_sixDOF_SIMInstanceStruct *chartInstance);
static void c9_dimagree(SFc9_sixDOF_SIMInstanceStruct *chartInstance);
static const mxArray *c9_d_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData);
static int32_T c9_e_emlrt_marshallIn(SFc9_sixDOF_SIMInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId);
static void c9_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static uint8_T c9_f_emlrt_marshallIn(SFc9_sixDOF_SIMInstanceStruct
  *chartInstance, const mxArray *c9_b_is_active_c9_sixDOF_SIM, const char_T
  *c9_identifier);
static uint8_T c9_g_emlrt_marshallIn(SFc9_sixDOF_SIMInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId);
static void init_dsm_address_info(SFc9_sixDOF_SIMInstanceStruct *chartInstance);
static void init_simulink_io_address(SFc9_sixDOF_SIMInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct
  *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc9_sixDOF_SIM(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c9_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c9_is_active_c9_sixDOF_SIM = 0U;
}

static void initialize_params_c9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void enable_c9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c9_update_debugger_state_c9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct *
  chartInstance)
{
  const mxArray *c9_st;
  const mxArray *c9_y = NULL;
  const mxArray *c9_b_y = NULL;
  uint8_T c9_hoistedGlobal;
  uint8_T c9_u;
  const mxArray *c9_c_y = NULL;
  c9_st = NULL;
  c9_st = NULL;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_createcellmatrix(2, 1), false);
  c9_b_y = NULL;
  sf_mex_assign(&c9_b_y, sf_mex_create("y", *chartInstance->c9_M, 0, 0U, 1U, 0U,
    1, 3), false);
  sf_mex_setcell(c9_y, 0, c9_b_y);
  c9_hoistedGlobal = chartInstance->c9_is_active_c9_sixDOF_SIM;
  c9_u = c9_hoistedGlobal;
  c9_c_y = NULL;
  sf_mex_assign(&c9_c_y, sf_mex_create("y", &c9_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c9_y, 1, c9_c_y);
  sf_mex_assign(&c9_st, c9_y, false);
  return c9_st;
}

static void set_sim_state_c9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct
  *chartInstance, const mxArray *c9_st)
{
  const mxArray *c9_u;
  real_T c9_dv0[3];
  int32_T c9_i0;
  chartInstance->c9_doneDoubleBufferReInit = true;
  c9_u = sf_mex_dup(c9_st);
  c9_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("M", c9_u, 0)),
                      "M", c9_dv0);
  for (c9_i0 = 0; c9_i0 < 3; c9_i0++) {
    (*chartInstance->c9_M)[c9_i0] = c9_dv0[c9_i0];
  }

  chartInstance->c9_is_active_c9_sixDOF_SIM = c9_f_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell("is_active_c9_sixDOF_SIM", c9_u, 1)),
     "is_active_c9_sixDOF_SIM");
  sf_mex_destroy(&c9_u);
  c9_update_debugger_state_c9_sixDOF_SIM(chartInstance);
  sf_mex_destroy(&c9_st);
}

static void finalize_c9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct
  *chartInstance)
{
  int32_T c9_i1;
  real_T c9_hoistedGlobal;
  real_T c9_b_hoistedGlobal;
  real_T c9_c_hoistedGlobal;
  real_T c9_d_hoistedGlobal;
  real_T c9_e_hoistedGlobal;
  real_T c9_b_mu;
  int32_T c9_i2;
  real_T c9_b_I[9];
  real_T c9_b_R;
  real_T c9_b_cx;
  real_T c9_b_cy;
  real_T c9_b_cz;
  uint32_T c9_debug_family_var_map[9];
  real_T c9_nargin = 6.0;
  real_T c9_nargout = 1.0;
  real_T c9_b_M[3];
  real_T c9_c_mu;
  int32_T c9_i3;
  real_T c9_c_I[9];
  real_T c9_c_R;
  real_T c9_c_cx;
  real_T c9_c_cy;
  real_T c9_c_cz;
  real_T c9_b_nargin = 6.0;
  real_T c9_b_nargout = 1.0;
  real_T c9_a;
  real_T c9_b_a;
  real_T c9_c_a;
  real_T c9_ak;
  real_T c9_d_a;
  real_T c9_ar;
  real_T c9_c;
  real_T c9_A;
  real_T c9_B;
  real_T c9_x;
  real_T c9_y;
  real_T c9_b_x;
  real_T c9_b_y;
  real_T c9_c_x;
  real_T c9_c_y;
  real_T c9_d_y;
  real_T c9_e_a;
  real_T c9_b[3];
  int32_T c9_i4;
  int32_T c9_i5;
  int32_T c9_i6;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 8U, chartInstance->c9_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c9_cz, 5U, 1U, 0U,
                        chartInstance->c9_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c9_cy, 4U, 1U, 0U,
                        chartInstance->c9_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c9_cx, 3U, 1U, 0U,
                        chartInstance->c9_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c9_R, 2U, 1U, 0U,
                        chartInstance->c9_sfEvent, false);
  for (c9_i1 = 0; c9_i1 < 9; c9_i1++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c9_I)[c9_i1], 1U, 1U, 0U,
                          chartInstance->c9_sfEvent, false);
  }

  _SFD_DATA_RANGE_CHECK(*chartInstance->c9_mu, 0U, 1U, 0U,
                        chartInstance->c9_sfEvent, false);
  chartInstance->c9_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 8U, chartInstance->c9_sfEvent);
  c9_hoistedGlobal = *chartInstance->c9_mu;
  c9_b_hoistedGlobal = *chartInstance->c9_R;
  c9_c_hoistedGlobal = *chartInstance->c9_cx;
  c9_d_hoistedGlobal = *chartInstance->c9_cy;
  c9_e_hoistedGlobal = *chartInstance->c9_cz;
  c9_b_mu = c9_hoistedGlobal;
  for (c9_i2 = 0; c9_i2 < 9; c9_i2++) {
    c9_b_I[c9_i2] = (*chartInstance->c9_I)[c9_i2];
  }

  c9_b_R = c9_b_hoistedGlobal;
  c9_b_cx = c9_c_hoistedGlobal;
  c9_b_cy = c9_d_hoistedGlobal;
  c9_b_cz = c9_e_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 9U, 9U, c9_debug_family_names,
    c9_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargin, 0U, c9_b_sf_marshallOut,
    c9_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargout, 1U, c9_b_sf_marshallOut,
    c9_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c9_b_mu, 2U, c9_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c9_b_I, 3U, c9_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c9_b_R, 4U, c9_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c9_b_cx, 5U, c9_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c9_b_cy, 6U, c9_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c9_b_cz, 7U, c9_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_b_M, 8U, c9_sf_marshallOut,
    c9_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 3);
  c9_c_mu = c9_b_mu;
  for (c9_i3 = 0; c9_i3 < 9; c9_i3++) {
    c9_c_I[c9_i3] = c9_b_I[c9_i3];
  }

  c9_c_R = c9_b_R;
  c9_c_cx = c9_b_cx;
  c9_c_cy = c9_b_cy;
  c9_c_cz = c9_b_cz;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 9U, 9U, c9_b_debug_family_names,
    c9_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_b_nargin, 0U, c9_b_sf_marshallOut,
    c9_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_b_nargout, 1U, c9_b_sf_marshallOut,
    c9_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_c_mu, 2U, c9_b_sf_marshallOut,
    c9_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_c_I, 3U, c9_c_sf_marshallOut,
    c9_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_c_R, 4U, c9_b_sf_marshallOut,
    c9_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_c_cx, 5U, c9_b_sf_marshallOut,
    c9_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_c_cy, 6U, c9_b_sf_marshallOut,
    c9_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_c_cz, 7U, c9_b_sf_marshallOut,
    c9_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_b_M, 8U, c9_sf_marshallOut,
    c9_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, 5);
  c9_a = c9_c_R;
  c9_b_a = c9_a;
  c9_c_a = c9_b_a;
  c9_eml_scalar_eg(chartInstance);
  c9_dimagree(chartInstance);
  c9_ak = c9_c_a;
  c9_d_a = c9_ak;
  c9_eml_scalar_eg(chartInstance);
  c9_ar = c9_d_a;
  c9_c = muDoubleScalarPower(c9_ar, 3.0);
  c9_A = 3.0 * c9_c_mu;
  c9_B = c9_c;
  c9_x = c9_A;
  c9_y = c9_B;
  c9_b_x = c9_x;
  c9_b_y = c9_y;
  c9_c_x = c9_b_x;
  c9_c_y = c9_b_y;
  c9_d_y = c9_c_x / c9_c_y;
  c9_e_a = c9_d_y;
  c9_b[0] = (c9_c_I[8] - c9_c_I[4]) * c9_c_cy * c9_c_cz;
  c9_b[1] = (c9_c_I[0] - c9_c_I[8]) * c9_c_cz * c9_c_cx;
  c9_b[2] = (c9_c_I[4] - c9_c_I[0]) * c9_c_cx * c9_c_cy;
  for (c9_i4 = 0; c9_i4 < 3; c9_i4++) {
    c9_b_M[c9_i4] = c9_e_a * c9_b[c9_i4];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c9_sfEvent, -5);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, -3);
  _SFD_SYMBOL_SCOPE_POP();
  for (c9_i5 = 0; c9_i5 < 3; c9_i5++) {
    (*chartInstance->c9_M)[c9_i5] = c9_b_M[c9_i5];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 8U, chartInstance->c9_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_sixDOF_SIMMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c9_i6 = 0; c9_i6 < 3; c9_i6++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c9_M)[c9_i6], 6U, 1U, 0U,
                          chartInstance->c9_sfEvent, false);
  }
}

static void mdl_start_c9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void initSimStructsc9_sixDOF_SIM(SFc9_sixDOF_SIMInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c9_machineNumber, uint32_T
  c9_chartNumber, uint32_T c9_instanceNumber)
{
  (void)c9_machineNumber;
  _SFD_SCRIPT_TRANSLATION(c9_chartNumber, c9_instanceNumber, 0U,
    sf_debug_get_script_id(
    "F:\\Users\\Toby Buckley\\Documents\\grad school documents\\aa279c\\aa279c_github\\src\\gravGradient.m"));
}

static const mxArray *c9_sf_marshallOut(void *chartInstanceVoid, void *c9_inData)
{
  const mxArray *c9_mxArrayOutData = NULL;
  int32_T c9_i7;
  real_T c9_u[3];
  const mxArray *c9_y = NULL;
  SFc9_sixDOF_SIMInstanceStruct *chartInstance;
  chartInstance = (SFc9_sixDOF_SIMInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  for (c9_i7 = 0; c9_i7 < 3; c9_i7++) {
    c9_u[c9_i7] = (*(real_T (*)[3])c9_inData)[c9_i7];
  }

  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", c9_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, false);
  return c9_mxArrayOutData;
}

static void c9_emlrt_marshallIn(SFc9_sixDOF_SIMInstanceStruct *chartInstance,
  const mxArray *c9_b_M, const char_T *c9_identifier, real_T c9_y[3])
{
  emlrtMsgIdentifier c9_thisId;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_b_M), &c9_thisId, c9_y);
  sf_mex_destroy(&c9_b_M);
}

static void c9_b_emlrt_marshallIn(SFc9_sixDOF_SIMInstanceStruct *chartInstance,
  const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId, real_T c9_y[3])
{
  real_T c9_dv1[3];
  int32_T c9_i8;
  (void)chartInstance;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), c9_dv1, 1, 0, 0U, 1, 0U, 1, 3);
  for (c9_i8 = 0; c9_i8 < 3; c9_i8++) {
    c9_y[c9_i8] = c9_dv1[c9_i8];
  }

  sf_mex_destroy(&c9_u);
}

static void c9_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_b_M;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  real_T c9_y[3];
  int32_T c9_i9;
  SFc9_sixDOF_SIMInstanceStruct *chartInstance;
  chartInstance = (SFc9_sixDOF_SIMInstanceStruct *)chartInstanceVoid;
  c9_b_M = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_b_M), &c9_thisId, c9_y);
  sf_mex_destroy(&c9_b_M);
  for (c9_i9 = 0; c9_i9 < 3; c9_i9++) {
    (*(real_T (*)[3])c9_outData)[c9_i9] = c9_y[c9_i9];
  }

  sf_mex_destroy(&c9_mxArrayInData);
}

static const mxArray *c9_b_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData)
{
  const mxArray *c9_mxArrayOutData = NULL;
  real_T c9_u;
  const mxArray *c9_y = NULL;
  SFc9_sixDOF_SIMInstanceStruct *chartInstance;
  chartInstance = (SFc9_sixDOF_SIMInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  c9_u = *(real_T *)c9_inData;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", &c9_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, false);
  return c9_mxArrayOutData;
}

static const mxArray *c9_c_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData)
{
  const mxArray *c9_mxArrayOutData = NULL;
  int32_T c9_i10;
  int32_T c9_i11;
  int32_T c9_i12;
  real_T c9_u[9];
  const mxArray *c9_y = NULL;
  SFc9_sixDOF_SIMInstanceStruct *chartInstance;
  chartInstance = (SFc9_sixDOF_SIMInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  c9_i10 = 0;
  for (c9_i11 = 0; c9_i11 < 3; c9_i11++) {
    for (c9_i12 = 0; c9_i12 < 3; c9_i12++) {
      c9_u[c9_i12 + c9_i10] = (*(real_T (*)[9])c9_inData)[c9_i12 + c9_i10];
    }

    c9_i10 += 3;
  }

  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", c9_u, 0, 0U, 1U, 0U, 2, 3, 3), false);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, false);
  return c9_mxArrayOutData;
}

static real_T c9_c_emlrt_marshallIn(SFc9_sixDOF_SIMInstanceStruct *chartInstance,
  const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId)
{
  real_T c9_y;
  real_T c9_d0;
  (void)chartInstance;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), &c9_d0, 1, 0, 0U, 0, 0U, 0);
  c9_y = c9_d0;
  sf_mex_destroy(&c9_u);
  return c9_y;
}

static void c9_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_nargout;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  real_T c9_y;
  SFc9_sixDOF_SIMInstanceStruct *chartInstance;
  chartInstance = (SFc9_sixDOF_SIMInstanceStruct *)chartInstanceVoid;
  c9_nargout = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_y = c9_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_nargout), &c9_thisId);
  sf_mex_destroy(&c9_nargout);
  *(real_T *)c9_outData = c9_y;
  sf_mex_destroy(&c9_mxArrayInData);
}

static void c9_d_emlrt_marshallIn(SFc9_sixDOF_SIMInstanceStruct *chartInstance,
  const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId, real_T c9_y[9])
{
  real_T c9_dv2[9];
  int32_T c9_i13;
  (void)chartInstance;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), c9_dv2, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c9_i13 = 0; c9_i13 < 9; c9_i13++) {
    c9_y[c9_i13] = c9_dv2[c9_i13];
  }

  sf_mex_destroy(&c9_u);
}

static void c9_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_b_I;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  real_T c9_y[9];
  int32_T c9_i14;
  int32_T c9_i15;
  int32_T c9_i16;
  SFc9_sixDOF_SIMInstanceStruct *chartInstance;
  chartInstance = (SFc9_sixDOF_SIMInstanceStruct *)chartInstanceVoid;
  c9_b_I = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_b_I), &c9_thisId, c9_y);
  sf_mex_destroy(&c9_b_I);
  c9_i14 = 0;
  for (c9_i15 = 0; c9_i15 < 3; c9_i15++) {
    for (c9_i16 = 0; c9_i16 < 3; c9_i16++) {
      (*(real_T (*)[9])c9_outData)[c9_i16 + c9_i14] = c9_y[c9_i16 + c9_i14];
    }

    c9_i14 += 3;
  }

  sf_mex_destroy(&c9_mxArrayInData);
}

const mxArray *sf_c9_sixDOF_SIM_get_eml_resolved_functions_info(void)
{
  const mxArray *c9_nameCaptureInfo = NULL;
  c9_nameCaptureInfo = NULL;
  sf_mex_assign(&c9_nameCaptureInfo, sf_mex_createstruct("structure", 2, 1, 1),
                false);
  c9_info_helper(&c9_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c9_nameCaptureInfo);
  return c9_nameCaptureInfo;
}

static void c9_info_helper(const mxArray **c9_info)
{
  const mxArray *c9_rhs0 = NULL;
  const mxArray *c9_lhs0 = NULL;
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("gravGradient"), "name", "name",
                  0);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[E]F:/Users/Toby Buckley/Documents/grad school documents/aa279c/aa279c_github/src/gravGradient.m"),
                  "resolved", "resolved", 0);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1496426276U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c9_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c9_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs0), "lhs", "lhs", 0);
  sf_mex_destroy(&c9_rhs0);
  sf_mex_destroy(&c9_lhs0);
}

static const mxArray *c9_emlrt_marshallOut(const char * c9_u)
{
  const mxArray *c9_y = NULL;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", c9_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c9_u)), false);
  return c9_y;
}

static const mxArray *c9_b_emlrt_marshallOut(const uint32_T c9_u)
{
  const mxArray *c9_y = NULL;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", &c9_u, 7, 0U, 0U, 0U, 0), false);
  return c9_y;
}

static void c9_eml_scalar_eg(SFc9_sixDOF_SIMInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c9_dimagree(SFc9_sixDOF_SIMInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *c9_d_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData)
{
  const mxArray *c9_mxArrayOutData = NULL;
  int32_T c9_u;
  const mxArray *c9_y = NULL;
  SFc9_sixDOF_SIMInstanceStruct *chartInstance;
  chartInstance = (SFc9_sixDOF_SIMInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  c9_u = *(int32_T *)c9_inData;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", &c9_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, false);
  return c9_mxArrayOutData;
}

static int32_T c9_e_emlrt_marshallIn(SFc9_sixDOF_SIMInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId)
{
  int32_T c9_y;
  int32_T c9_i17;
  (void)chartInstance;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), &c9_i17, 1, 6, 0U, 0, 0U, 0);
  c9_y = c9_i17;
  sf_mex_destroy(&c9_u);
  return c9_y;
}

static void c9_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_b_sfEvent;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  int32_T c9_y;
  SFc9_sixDOF_SIMInstanceStruct *chartInstance;
  chartInstance = (SFc9_sixDOF_SIMInstanceStruct *)chartInstanceVoid;
  c9_b_sfEvent = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_y = c9_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_b_sfEvent),
    &c9_thisId);
  sf_mex_destroy(&c9_b_sfEvent);
  *(int32_T *)c9_outData = c9_y;
  sf_mex_destroy(&c9_mxArrayInData);
}

static uint8_T c9_f_emlrt_marshallIn(SFc9_sixDOF_SIMInstanceStruct
  *chartInstance, const mxArray *c9_b_is_active_c9_sixDOF_SIM, const char_T
  *c9_identifier)
{
  uint8_T c9_y;
  emlrtMsgIdentifier c9_thisId;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_y = c9_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c9_b_is_active_c9_sixDOF_SIM), &c9_thisId);
  sf_mex_destroy(&c9_b_is_active_c9_sixDOF_SIM);
  return c9_y;
}

static uint8_T c9_g_emlrt_marshallIn(SFc9_sixDOF_SIMInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId)
{
  uint8_T c9_y;
  uint8_T c9_u0;
  (void)chartInstance;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), &c9_u0, 1, 3, 0U, 0, 0U, 0);
  c9_y = c9_u0;
  sf_mex_destroy(&c9_u);
  return c9_y;
}

static void init_dsm_address_info(SFc9_sixDOF_SIMInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc9_sixDOF_SIMInstanceStruct
  *chartInstance)
{
  chartInstance->c9_mu = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    0);
  chartInstance->c9_M = (real_T (*)[3])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c9_I = (real_T (*)[9])ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c9_R = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    2);
  chartInstance->c9_cx = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    3);
  chartInstance->c9_cy = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    4);
  chartInstance->c9_cz = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    5);
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c9_sixDOF_SIM_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2050320967U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(301643096U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2172462776U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3826601008U);
}

mxArray* sf_c9_sixDOF_SIM_get_post_codegen_info(void);
mxArray *sf_c9_sixDOF_SIM_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("v6EAgPYBe8YOTNO0PklYJD");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,6,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(3);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
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
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));
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
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c9_sixDOF_SIM_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c9_sixDOF_SIM_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c9_sixDOF_SIM_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("pre");
  mxArray *fallbackReason = mxCreateString("hasBreakpoints");
  mxArray *hiddenFallbackType = mxCreateString("none");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c9_sixDOF_SIM_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c9_sixDOF_SIM_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c9_sixDOF_SIM(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"M\",},{M[8],M[0],T\"is_active_c9_sixDOF_SIM\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c9_sixDOF_SIM_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc9_sixDOF_SIMInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc9_sixDOF_SIMInstanceStruct *) chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _sixDOF_SIMMachineNumber_,
           9,
           1,
           1,
           0,
           7,
           0,
           0,
           0,
           0,
           1,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation(_sixDOF_SIMMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_sixDOF_SIMMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _sixDOF_SIMMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"mu");
          _SFD_SET_DATA_PROPS(1,1,1,0,"I");
          _SFD_SET_DATA_PROPS(2,1,1,0,"R");
          _SFD_SET_DATA_PROPS(3,1,1,0,"cx");
          _SFD_SET_DATA_PROPS(4,1,1,0,"cy");
          _SFD_SET_DATA_PROPS(5,1,1,0,"cz");
          _SFD_SET_DATA_PROPS(6,2,0,1,"M");
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
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,91);
        _SFD_CV_INIT_SCRIPT(0,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"gravGradient",0,-1,398);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c9_b_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c9_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c9_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c9_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c9_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c9_b_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c9_sf_marshallOut,(MexInFcnForType)
            c9_sf_marshallIn);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _sixDOF_SIMMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static void chart_debug_initialize_data_addresses(SimStruct *S)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc9_sixDOF_SIMInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc9_sixDOF_SIMInstanceStruct *) chartInfo->chartInstance;
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c9_mu);
        _SFD_SET_DATA_VALUE_PTR(6U, *chartInstance->c9_M);
        _SFD_SET_DATA_VALUE_PTR(1U, *chartInstance->c9_I);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c9_R);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c9_cx);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c9_cy);
        _SFD_SET_DATA_VALUE_PTR(5U, chartInstance->c9_cz);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "sVDjwqfYtGbEXmZGuDKVkBD";
}

static void sf_opaque_initialize_c9_sixDOF_SIM(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc9_sixDOF_SIMInstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c9_sixDOF_SIM((SFc9_sixDOF_SIMInstanceStruct*)
    chartInstanceVar);
  initialize_c9_sixDOF_SIM((SFc9_sixDOF_SIMInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c9_sixDOF_SIM(void *chartInstanceVar)
{
  enable_c9_sixDOF_SIM((SFc9_sixDOF_SIMInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c9_sixDOF_SIM(void *chartInstanceVar)
{
  disable_c9_sixDOF_SIM((SFc9_sixDOF_SIMInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c9_sixDOF_SIM(void *chartInstanceVar)
{
  sf_gateway_c9_sixDOF_SIM((SFc9_sixDOF_SIMInstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c9_sixDOF_SIM(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c9_sixDOF_SIM((SFc9_sixDOF_SIMInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c9_sixDOF_SIM(SimStruct* S, const mxArray
  *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c9_sixDOF_SIM((SFc9_sixDOF_SIMInstanceStruct*)
    chartInfo->chartInstance, st);
}

static void sf_opaque_terminate_c9_sixDOF_SIM(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc9_sixDOF_SIMInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_sixDOF_SIM_optimization_info();
    }

    finalize_c9_sixDOF_SIM((SFc9_sixDOF_SIMInstanceStruct*) chartInstanceVar);
    utFree(chartInstanceVar);
    if (crtInfo != NULL) {
      utFree(crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc9_sixDOF_SIM((SFc9_sixDOF_SIMInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c9_sixDOF_SIM(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    initialize_params_c9_sixDOF_SIM((SFc9_sixDOF_SIMInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c9_sixDOF_SIM(SimStruct *S)
{
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_sixDOF_SIM_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,9);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,9,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,9);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,9,6);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,9,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 6; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,9);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1246247795U));
  ssSetChecksum1(S,(1398204535U));
  ssSetChecksum2(S,(878263017U));
  ssSetChecksum3(S,(1781495534U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c9_sixDOF_SIM(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c9_sixDOF_SIM(SimStruct *S)
{
  SFc9_sixDOF_SIMInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc9_sixDOF_SIMInstanceStruct *)utMalloc(sizeof
    (SFc9_sixDOF_SIMInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc9_sixDOF_SIMInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c9_sixDOF_SIM;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c9_sixDOF_SIM;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c9_sixDOF_SIM;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c9_sixDOF_SIM;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c9_sixDOF_SIM;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c9_sixDOF_SIM;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c9_sixDOF_SIM;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c9_sixDOF_SIM;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c9_sixDOF_SIM;
  chartInstance->chartInfo.mdlStart = mdlStart_c9_sixDOF_SIM;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c9_sixDOF_SIM;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->isEnhancedMooreMachine = 0;
  crtInfo->checksum = SF_RUNTIME_INFO_CHECKSUM;
  crtInfo->fCheckOverflow = sf_runtime_overflow_check_is_on(S);
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  crtInfo->compiledInfo = NULL;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  chart_debug_initialization(S,1);
}

void c9_sixDOF_SIM_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c9_sixDOF_SIM(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c9_sixDOF_SIM(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c9_sixDOF_SIM(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c9_sixDOF_SIM_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
