/* Include files */

#include <stddef.h>
#include "blas.h"
#include "sixDOF_SIM_sfun.h"
#include "c10_sixDOF_SIM.h"
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
static const char * c10_debug_family_names[10] = { "A", "nargin", "nargout",
  "state", "A_DCM", "c", "cx", "cy", "cz", "R" };

static const char * c10_b_debug_family_names[4] = { "nargin", "nargout", "A_DCM",
  "A_p2i" };

/* Function Declarations */
static void initialize_c10_sixDOF_SIM(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance);
static void initialize_params_c10_sixDOF_SIM(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance);
static void enable_c10_sixDOF_SIM(SFc10_sixDOF_SIMInstanceStruct *chartInstance);
static void disable_c10_sixDOF_SIM(SFc10_sixDOF_SIMInstanceStruct *chartInstance);
static void c10_update_debugger_state_c10_sixDOF_SIM
  (SFc10_sixDOF_SIMInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c10_sixDOF_SIM
  (SFc10_sixDOF_SIMInstanceStruct *chartInstance);
static void set_sim_state_c10_sixDOF_SIM(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance, const mxArray *c10_st);
static void finalize_c10_sixDOF_SIM(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance);
static void sf_gateway_c10_sixDOF_SIM(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance);
static void mdl_start_c10_sixDOF_SIM(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance);
static void initSimStructsc10_sixDOF_SIM(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c10_machineNumber, uint32_T
  c10_chartNumber, uint32_T c10_instanceNumber);
static const mxArray *c10_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData);
static real_T c10_emlrt_marshallIn(SFc10_sixDOF_SIMInstanceStruct *chartInstance,
  const mxArray *c10_b_R, const char_T *c10_identifier);
static real_T c10_b_emlrt_marshallIn(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId);
static void c10_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData);
static const mxArray *c10_b_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData);
static void c10_c_emlrt_marshallIn(SFc10_sixDOF_SIMInstanceStruct *chartInstance,
  const mxArray *c10_b_c, const char_T *c10_identifier, real_T c10_y[3]);
static void c10_d_emlrt_marshallIn(SFc10_sixDOF_SIMInstanceStruct *chartInstance,
  const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId, real_T c10_y[3]);
static void c10_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData);
static const mxArray *c10_c_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData);
static const mxArray *c10_d_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData);
static void c10_e_emlrt_marshallIn(SFc10_sixDOF_SIMInstanceStruct *chartInstance,
  const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId, real_T c10_y[9]);
static void c10_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData);
static void c10_info_helper(const mxArray **c10_info);
static const mxArray *c10_emlrt_marshallOut(const char * c10_u);
static const mxArray *c10_b_emlrt_marshallOut(const uint32_T c10_u);
static real_T c10_eml_xnrm2(SFc10_sixDOF_SIMInstanceStruct *chartInstance,
  real_T c10_x[3]);
static void c10_below_threshold(SFc10_sixDOF_SIMInstanceStruct *chartInstance);
static void c10_eml_scalar_eg(SFc10_sixDOF_SIMInstanceStruct *chartInstance);
static void c10_eml_xgemm(SFc10_sixDOF_SIMInstanceStruct *chartInstance, real_T
  c10_A[9], real_T c10_B[3], real_T c10_C[3], real_T c10_b_C[3]);
static const mxArray *c10_e_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData);
static int32_T c10_f_emlrt_marshallIn(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId);
static void c10_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData);
static uint8_T c10_g_emlrt_marshallIn(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance, const mxArray *c10_b_is_active_c10_sixDOF_SIM, const char_T
  *c10_identifier);
static uint8_T c10_h_emlrt_marshallIn(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId);
static void c10_b_eml_xgemm(SFc10_sixDOF_SIMInstanceStruct *chartInstance,
  real_T c10_A[9], real_T c10_B[3], real_T c10_C[3]);
static void init_dsm_address_info(SFc10_sixDOF_SIMInstanceStruct *chartInstance);
static void init_simulink_io_address(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c10_sixDOF_SIM(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc10_sixDOF_SIM(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c10_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c10_is_active_c10_sixDOF_SIM = 0U;
}

static void initialize_params_c10_sixDOF_SIM(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void enable_c10_sixDOF_SIM(SFc10_sixDOF_SIMInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c10_sixDOF_SIM(SFc10_sixDOF_SIMInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c10_update_debugger_state_c10_sixDOF_SIM
  (SFc10_sixDOF_SIMInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c10_sixDOF_SIM
  (SFc10_sixDOF_SIMInstanceStruct *chartInstance)
{
  const mxArray *c10_st;
  const mxArray *c10_y = NULL;
  real_T c10_hoistedGlobal;
  real_T c10_u;
  const mxArray *c10_b_y = NULL;
  const mxArray *c10_c_y = NULL;
  real_T c10_b_hoistedGlobal;
  real_T c10_b_u;
  const mxArray *c10_d_y = NULL;
  real_T c10_c_hoistedGlobal;
  real_T c10_c_u;
  const mxArray *c10_e_y = NULL;
  real_T c10_d_hoistedGlobal;
  real_T c10_d_u;
  const mxArray *c10_f_y = NULL;
  uint8_T c10_e_hoistedGlobal;
  uint8_T c10_e_u;
  const mxArray *c10_g_y = NULL;
  c10_st = NULL;
  c10_st = NULL;
  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_createcellmatrix(6, 1), false);
  c10_hoistedGlobal = *chartInstance->c10_R;
  c10_u = c10_hoistedGlobal;
  c10_b_y = NULL;
  sf_mex_assign(&c10_b_y, sf_mex_create("y", &c10_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c10_y, 0, c10_b_y);
  c10_c_y = NULL;
  sf_mex_assign(&c10_c_y, sf_mex_create("y", *chartInstance->c10_c, 0, 0U, 1U,
    0U, 1, 3), false);
  sf_mex_setcell(c10_y, 1, c10_c_y);
  c10_b_hoistedGlobal = *chartInstance->c10_cx;
  c10_b_u = c10_b_hoistedGlobal;
  c10_d_y = NULL;
  sf_mex_assign(&c10_d_y, sf_mex_create("y", &c10_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c10_y, 2, c10_d_y);
  c10_c_hoistedGlobal = *chartInstance->c10_cy;
  c10_c_u = c10_c_hoistedGlobal;
  c10_e_y = NULL;
  sf_mex_assign(&c10_e_y, sf_mex_create("y", &c10_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c10_y, 3, c10_e_y);
  c10_d_hoistedGlobal = *chartInstance->c10_cz;
  c10_d_u = c10_d_hoistedGlobal;
  c10_f_y = NULL;
  sf_mex_assign(&c10_f_y, sf_mex_create("y", &c10_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c10_y, 4, c10_f_y);
  c10_e_hoistedGlobal = chartInstance->c10_is_active_c10_sixDOF_SIM;
  c10_e_u = c10_e_hoistedGlobal;
  c10_g_y = NULL;
  sf_mex_assign(&c10_g_y, sf_mex_create("y", &c10_e_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c10_y, 5, c10_g_y);
  sf_mex_assign(&c10_st, c10_y, false);
  return c10_st;
}

static void set_sim_state_c10_sixDOF_SIM(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance, const mxArray *c10_st)
{
  const mxArray *c10_u;
  real_T c10_dv0[3];
  int32_T c10_i0;
  chartInstance->c10_doneDoubleBufferReInit = true;
  c10_u = sf_mex_dup(c10_st);
  *chartInstance->c10_R = c10_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("R", c10_u, 0)), "R");
  c10_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("c", c10_u, 1)),
    "c", c10_dv0);
  for (c10_i0 = 0; c10_i0 < 3; c10_i0++) {
    (*chartInstance->c10_c)[c10_i0] = c10_dv0[c10_i0];
  }

  *chartInstance->c10_cx = c10_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("cx", c10_u, 2)), "cx");
  *chartInstance->c10_cy = c10_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("cy", c10_u, 3)), "cy");
  *chartInstance->c10_cz = c10_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("cz", c10_u, 4)), "cz");
  chartInstance->c10_is_active_c10_sixDOF_SIM = c10_g_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell("is_active_c10_sixDOF_SIM", c10_u,
       5)), "is_active_c10_sixDOF_SIM");
  sf_mex_destroy(&c10_u);
  c10_update_debugger_state_c10_sixDOF_SIM(chartInstance);
  sf_mex_destroy(&c10_st);
}

static void finalize_c10_sixDOF_SIM(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c10_sixDOF_SIM(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance)
{
  int32_T c10_i1;
  int32_T c10_i2;
  int32_T c10_i3;
  real_T c10_b_state[6];
  int32_T c10_i4;
  real_T c10_b_A_DCM[9];
  uint32_T c10_debug_family_var_map[10];
  real_T c10_A[9];
  real_T c10_nargin = 2.0;
  real_T c10_nargout = 5.0;
  real_T c10_b_c[3];
  real_T c10_b_cx;
  real_T c10_b_cy;
  real_T c10_b_cz;
  real_T c10_b_R;
  int32_T c10_i5;
  real_T c10_c_A_DCM[9];
  uint32_T c10_b_debug_family_var_map[4];
  real_T c10_b_nargin = 1.0;
  real_T c10_b_nargout = 1.0;
  int32_T c10_i6;
  int32_T c10_i7;
  int32_T c10_i8;
  int32_T c10_i9;
  int32_T c10_i10;
  real_T c10_x[3];
  int32_T c10_i11;
  real_T c10_b_x[3];
  real_T c10_B;
  real_T c10_y;
  real_T c10_b_y;
  real_T c10_c_y;
  real_T c10_d_y;
  real_T c10_a;
  int32_T c10_i12;
  int32_T c10_i13;
  int32_T c10_i14;
  int32_T c10_i15;
  real_T c10_b[9];
  int32_T c10_i16;
  int32_T c10_i17;
  int32_T c10_i18;
  int32_T c10_i19;
  real_T c10_dv1[3];
  int32_T c10_i20;
  real_T c10_b_b[9];
  int32_T c10_i21;
  real_T c10_c_x[3];
  int32_T c10_i22;
  int32_T c10_i23;
  int32_T c10_i24;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 9U, chartInstance->c10_sfEvent);
  for (c10_i1 = 0; c10_i1 < 9; c10_i1++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c10_A_DCM)[c10_i1], 1U, 1U, 0U,
                          chartInstance->c10_sfEvent, false);
  }

  for (c10_i2 = 0; c10_i2 < 6; c10_i2++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c10_state)[c10_i2], 0U, 1U, 0U,
                          chartInstance->c10_sfEvent, false);
  }

  chartInstance->c10_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 9U, chartInstance->c10_sfEvent);
  for (c10_i3 = 0; c10_i3 < 6; c10_i3++) {
    c10_b_state[c10_i3] = (*chartInstance->c10_state)[c10_i3];
  }

  for (c10_i4 = 0; c10_i4 < 9; c10_i4++) {
    c10_b_A_DCM[c10_i4] = (*chartInstance->c10_A_DCM)[c10_i4];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 10U, 10U, c10_debug_family_names,
    c10_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_A, 0U, c10_c_sf_marshallOut,
    c10_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_nargin, 1U, c10_sf_marshallOut,
    c10_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_nargout, 2U, c10_sf_marshallOut,
    c10_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c10_b_state, 3U, c10_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c10_b_A_DCM, 4U, c10_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_b_c, 5U, c10_b_sf_marshallOut,
    c10_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_b_cx, 6U, c10_sf_marshallOut,
    c10_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_b_cy, 7U, c10_sf_marshallOut,
    c10_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_b_cz, 8U, c10_sf_marshallOut,
    c10_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_b_R, 9U, c10_sf_marshallOut,
    c10_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 4);
  for (c10_i5 = 0; c10_i5 < 9; c10_i5++) {
    c10_c_A_DCM[c10_i5] = c10_b_A_DCM[c10_i5];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c10_b_debug_family_names,
    c10_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_b_nargin, 0U, c10_sf_marshallOut,
    c10_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c10_b_nargout, 1U, c10_sf_marshallOut,
    c10_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_c_A_DCM, 2U, c10_c_sf_marshallOut,
    c10_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c10_A, 3U, c10_c_sf_marshallOut,
    c10_c_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c10_sfEvent, 5);
  c10_i6 = 0;
  for (c10_i7 = 0; c10_i7 < 3; c10_i7++) {
    c10_i8 = 0;
    for (c10_i9 = 0; c10_i9 < 3; c10_i9++) {
      c10_A[c10_i9 + c10_i6] = c10_c_A_DCM[c10_i8 + c10_i7];
      c10_i8 += 3;
    }

    c10_i6 += 3;
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c10_sfEvent, -5);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 5);
  for (c10_i10 = 0; c10_i10 < 3; c10_i10++) {
    c10_x[c10_i10] = c10_b_state[c10_i10];
  }

  for (c10_i11 = 0; c10_i11 < 3; c10_i11++) {
    c10_b_x[c10_i11] = c10_x[c10_i11];
  }

  c10_b_R = c10_eml_xnrm2(chartInstance, c10_b_x);
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 6);
  c10_B = c10_b_R;
  c10_y = c10_B;
  c10_b_y = c10_y;
  c10_c_y = c10_b_y;
  c10_d_y = 1.0 / c10_c_y;
  c10_a = c10_d_y;
  c10_i12 = 0;
  for (c10_i13 = 0; c10_i13 < 3; c10_i13++) {
    c10_i14 = 0;
    for (c10_i15 = 0; c10_i15 < 3; c10_i15++) {
      c10_b[c10_i15 + c10_i12] = c10_A[c10_i14 + c10_i13];
      c10_i14 += 3;
    }

    c10_i12 += 3;
  }

  for (c10_i16 = 0; c10_i16 < 9; c10_i16++) {
    c10_b[c10_i16] *= c10_a;
  }

  for (c10_i17 = 0; c10_i17 < 3; c10_i17++) {
    c10_x[c10_i17] = c10_b_state[c10_i17];
  }

  c10_eml_scalar_eg(chartInstance);
  c10_eml_scalar_eg(chartInstance);
  for (c10_i18 = 0; c10_i18 < 3; c10_i18++) {
    c10_b_c[c10_i18] = 0.0;
  }

  for (c10_i19 = 0; c10_i19 < 3; c10_i19++) {
    c10_dv1[c10_i19] = 0.0;
  }

  for (c10_i20 = 0; c10_i20 < 9; c10_i20++) {
    c10_b_b[c10_i20] = c10_b[c10_i20];
  }

  for (c10_i21 = 0; c10_i21 < 3; c10_i21++) {
    c10_c_x[c10_i21] = c10_x[c10_i21];
  }

  c10_b_eml_xgemm(chartInstance, c10_b_b, c10_c_x, c10_dv1);
  for (c10_i22 = 0; c10_i22 < 3; c10_i22++) {
    c10_b_c[c10_i22] = c10_dv1[c10_i22];
  }

  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 7);
  c10_b_cx = c10_b_c[0];
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 8);
  c10_b_cy = c10_b_c[1];
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, 9);
  c10_b_cz = c10_b_c[2];
  _SFD_EML_CALL(0U, chartInstance->c10_sfEvent, -9);
  _SFD_SYMBOL_SCOPE_POP();
  for (c10_i23 = 0; c10_i23 < 3; c10_i23++) {
    (*chartInstance->c10_c)[c10_i23] = c10_b_c[c10_i23];
  }

  *chartInstance->c10_cx = c10_b_cx;
  *chartInstance->c10_cy = c10_b_cy;
  *chartInstance->c10_cz = c10_b_cz;
  *chartInstance->c10_R = c10_b_R;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 9U, chartInstance->c10_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_sixDOF_SIMMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c10_i24 = 0; c10_i24 < 3; c10_i24++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c10_c)[c10_i24], 2U, 1U, 0U,
                          chartInstance->c10_sfEvent, false);
  }

  _SFD_DATA_RANGE_CHECK(*chartInstance->c10_cx, 3U, 1U, 0U,
                        chartInstance->c10_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c10_cy, 4U, 1U, 0U,
                        chartInstance->c10_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c10_cz, 5U, 1U, 0U,
                        chartInstance->c10_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c10_R, 6U, 1U, 0U,
                        chartInstance->c10_sfEvent, false);
}

static void mdl_start_c10_sixDOF_SIM(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void initSimStructsc10_sixDOF_SIM(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c10_machineNumber, uint32_T
  c10_chartNumber, uint32_T c10_instanceNumber)
{
  (void)c10_machineNumber;
  _SFD_SCRIPT_TRANSLATION(c10_chartNumber, c10_instanceNumber, 0U,
    sf_debug_get_script_id(
    "F:\\Users\\Toby Buckley\\Documents\\grad school documents\\aa279c\\aa279c_github\\src\\getP2IfromA_DCM.m"));
}

static const mxArray *c10_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData)
{
  const mxArray *c10_mxArrayOutData = NULL;
  real_T c10_u;
  const mxArray *c10_y = NULL;
  SFc10_sixDOF_SIMInstanceStruct *chartInstance;
  chartInstance = (SFc10_sixDOF_SIMInstanceStruct *)chartInstanceVoid;
  c10_mxArrayOutData = NULL;
  c10_u = *(real_T *)c10_inData;
  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", &c10_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c10_mxArrayOutData, c10_y, false);
  return c10_mxArrayOutData;
}

static real_T c10_emlrt_marshallIn(SFc10_sixDOF_SIMInstanceStruct *chartInstance,
  const mxArray *c10_b_R, const char_T *c10_identifier)
{
  real_T c10_y;
  emlrtMsgIdentifier c10_thisId;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_y = c10_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_b_R), &c10_thisId);
  sf_mex_destroy(&c10_b_R);
  return c10_y;
}

static real_T c10_b_emlrt_marshallIn(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId)
{
  real_T c10_y;
  real_T c10_d0;
  (void)chartInstance;
  sf_mex_import(c10_parentId, sf_mex_dup(c10_u), &c10_d0, 1, 0, 0U, 0, 0U, 0);
  c10_y = c10_d0;
  sf_mex_destroy(&c10_u);
  return c10_y;
}

static void c10_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData)
{
  const mxArray *c10_b_R;
  const char_T *c10_identifier;
  emlrtMsgIdentifier c10_thisId;
  real_T c10_y;
  SFc10_sixDOF_SIMInstanceStruct *chartInstance;
  chartInstance = (SFc10_sixDOF_SIMInstanceStruct *)chartInstanceVoid;
  c10_b_R = sf_mex_dup(c10_mxArrayInData);
  c10_identifier = c10_varName;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_y = c10_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_b_R), &c10_thisId);
  sf_mex_destroy(&c10_b_R);
  *(real_T *)c10_outData = c10_y;
  sf_mex_destroy(&c10_mxArrayInData);
}

static const mxArray *c10_b_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData)
{
  const mxArray *c10_mxArrayOutData = NULL;
  int32_T c10_i25;
  real_T c10_u[3];
  const mxArray *c10_y = NULL;
  SFc10_sixDOF_SIMInstanceStruct *chartInstance;
  chartInstance = (SFc10_sixDOF_SIMInstanceStruct *)chartInstanceVoid;
  c10_mxArrayOutData = NULL;
  for (c10_i25 = 0; c10_i25 < 3; c10_i25++) {
    c10_u[c10_i25] = (*(real_T (*)[3])c10_inData)[c10_i25];
  }

  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", c10_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c10_mxArrayOutData, c10_y, false);
  return c10_mxArrayOutData;
}

static void c10_c_emlrt_marshallIn(SFc10_sixDOF_SIMInstanceStruct *chartInstance,
  const mxArray *c10_b_c, const char_T *c10_identifier, real_T c10_y[3])
{
  emlrtMsgIdentifier c10_thisId;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_b_c), &c10_thisId, c10_y);
  sf_mex_destroy(&c10_b_c);
}

static void c10_d_emlrt_marshallIn(SFc10_sixDOF_SIMInstanceStruct *chartInstance,
  const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId, real_T c10_y[3])
{
  real_T c10_dv2[3];
  int32_T c10_i26;
  (void)chartInstance;
  sf_mex_import(c10_parentId, sf_mex_dup(c10_u), c10_dv2, 1, 0, 0U, 1, 0U, 1, 3);
  for (c10_i26 = 0; c10_i26 < 3; c10_i26++) {
    c10_y[c10_i26] = c10_dv2[c10_i26];
  }

  sf_mex_destroy(&c10_u);
}

static void c10_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData)
{
  const mxArray *c10_b_c;
  const char_T *c10_identifier;
  emlrtMsgIdentifier c10_thisId;
  real_T c10_y[3];
  int32_T c10_i27;
  SFc10_sixDOF_SIMInstanceStruct *chartInstance;
  chartInstance = (SFc10_sixDOF_SIMInstanceStruct *)chartInstanceVoid;
  c10_b_c = sf_mex_dup(c10_mxArrayInData);
  c10_identifier = c10_varName;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_b_c), &c10_thisId, c10_y);
  sf_mex_destroy(&c10_b_c);
  for (c10_i27 = 0; c10_i27 < 3; c10_i27++) {
    (*(real_T (*)[3])c10_outData)[c10_i27] = c10_y[c10_i27];
  }

  sf_mex_destroy(&c10_mxArrayInData);
}

static const mxArray *c10_c_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData)
{
  const mxArray *c10_mxArrayOutData = NULL;
  int32_T c10_i28;
  int32_T c10_i29;
  int32_T c10_i30;
  real_T c10_u[9];
  const mxArray *c10_y = NULL;
  SFc10_sixDOF_SIMInstanceStruct *chartInstance;
  chartInstance = (SFc10_sixDOF_SIMInstanceStruct *)chartInstanceVoid;
  c10_mxArrayOutData = NULL;
  c10_i28 = 0;
  for (c10_i29 = 0; c10_i29 < 3; c10_i29++) {
    for (c10_i30 = 0; c10_i30 < 3; c10_i30++) {
      c10_u[c10_i30 + c10_i28] = (*(real_T (*)[9])c10_inData)[c10_i30 + c10_i28];
    }

    c10_i28 += 3;
  }

  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", c10_u, 0, 0U, 1U, 0U, 2, 3, 3), false);
  sf_mex_assign(&c10_mxArrayOutData, c10_y, false);
  return c10_mxArrayOutData;
}

static const mxArray *c10_d_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData)
{
  const mxArray *c10_mxArrayOutData = NULL;
  int32_T c10_i31;
  real_T c10_u[6];
  const mxArray *c10_y = NULL;
  SFc10_sixDOF_SIMInstanceStruct *chartInstance;
  chartInstance = (SFc10_sixDOF_SIMInstanceStruct *)chartInstanceVoid;
  c10_mxArrayOutData = NULL;
  for (c10_i31 = 0; c10_i31 < 6; c10_i31++) {
    c10_u[c10_i31] = (*(real_T (*)[6])c10_inData)[c10_i31];
  }

  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", c10_u, 0, 0U, 1U, 0U, 1, 6), false);
  sf_mex_assign(&c10_mxArrayOutData, c10_y, false);
  return c10_mxArrayOutData;
}

static void c10_e_emlrt_marshallIn(SFc10_sixDOF_SIMInstanceStruct *chartInstance,
  const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId, real_T c10_y[9])
{
  real_T c10_dv3[9];
  int32_T c10_i32;
  (void)chartInstance;
  sf_mex_import(c10_parentId, sf_mex_dup(c10_u), c10_dv3, 1, 0, 0U, 1, 0U, 2, 3,
                3);
  for (c10_i32 = 0; c10_i32 < 9; c10_i32++) {
    c10_y[c10_i32] = c10_dv3[c10_i32];
  }

  sf_mex_destroy(&c10_u);
}

static void c10_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData)
{
  const mxArray *c10_A;
  const char_T *c10_identifier;
  emlrtMsgIdentifier c10_thisId;
  real_T c10_y[9];
  int32_T c10_i33;
  int32_T c10_i34;
  int32_T c10_i35;
  SFc10_sixDOF_SIMInstanceStruct *chartInstance;
  chartInstance = (SFc10_sixDOF_SIMInstanceStruct *)chartInstanceVoid;
  c10_A = sf_mex_dup(c10_mxArrayInData);
  c10_identifier = c10_varName;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_A), &c10_thisId, c10_y);
  sf_mex_destroy(&c10_A);
  c10_i33 = 0;
  for (c10_i34 = 0; c10_i34 < 3; c10_i34++) {
    for (c10_i35 = 0; c10_i35 < 3; c10_i35++) {
      (*(real_T (*)[9])c10_outData)[c10_i35 + c10_i33] = c10_y[c10_i35 + c10_i33];
    }

    c10_i33 += 3;
  }

  sf_mex_destroy(&c10_mxArrayInData);
}

const mxArray *sf_c10_sixDOF_SIM_get_eml_resolved_functions_info(void)
{
  const mxArray *c10_nameCaptureInfo = NULL;
  c10_nameCaptureInfo = NULL;
  sf_mex_assign(&c10_nameCaptureInfo, sf_mex_createstruct("structure", 2, 1, 1),
                false);
  c10_info_helper(&c10_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c10_nameCaptureInfo);
  return c10_nameCaptureInfo;
}

static void c10_info_helper(const mxArray **c10_info)
{
  const mxArray *c10_rhs0 = NULL;
  const mxArray *c10_lhs0 = NULL;
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("getP2IfromA_DCM"), "name",
                  "name", 0);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c10_info, c10_emlrt_marshallOut(
    "[E]F:/Users/Toby Buckley/Documents/grad school documents/aa279c/aa279c_github/src/getP2IfromA_DCM.m"),
                  "resolved", "resolved", 0);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(1496419971U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c10_info, c10_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c10_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c10_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_rhs0), "rhs", "rhs",
                  0);
  sf_mex_addfield(*c10_info, sf_mex_duplicatearraysafe(&c10_lhs0), "lhs", "lhs",
                  0);
  sf_mex_destroy(&c10_rhs0);
  sf_mex_destroy(&c10_lhs0);
}

static const mxArray *c10_emlrt_marshallOut(const char * c10_u)
{
  const mxArray *c10_y = NULL;
  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", c10_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c10_u)), false);
  return c10_y;
}

static const mxArray *c10_b_emlrt_marshallOut(const uint32_T c10_u)
{
  const mxArray *c10_y = NULL;
  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", &c10_u, 7, 0U, 0U, 0U, 0), false);
  return c10_y;
}

static real_T c10_eml_xnrm2(SFc10_sixDOF_SIMInstanceStruct *chartInstance,
  real_T c10_x[3])
{
  real_T c10_y;
  real_T c10_scale;
  int32_T c10_k;
  int32_T c10_b_k;
  real_T c10_b_x;
  real_T c10_c_x;
  real_T c10_absxk;
  real_T c10_t;
  c10_below_threshold(chartInstance);
  c10_y = 0.0;
  c10_scale = 2.2250738585072014E-308;
  for (c10_k = 1; c10_k < 4; c10_k++) {
    c10_b_k = c10_k;
    c10_b_x = c10_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c10_b_k), 1, 3, 1, 0) - 1];
    c10_c_x = c10_b_x;
    c10_absxk = muDoubleScalarAbs(c10_c_x);
    if (c10_absxk > c10_scale) {
      c10_t = c10_scale / c10_absxk;
      c10_y = 1.0 + c10_y * c10_t * c10_t;
      c10_scale = c10_absxk;
    } else {
      c10_t = c10_absxk / c10_scale;
      c10_y += c10_t * c10_t;
    }
  }

  return c10_scale * muDoubleScalarSqrt(c10_y);
}

static void c10_below_threshold(SFc10_sixDOF_SIMInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c10_eml_scalar_eg(SFc10_sixDOF_SIMInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c10_eml_xgemm(SFc10_sixDOF_SIMInstanceStruct *chartInstance, real_T
  c10_A[9], real_T c10_B[3], real_T c10_C[3], real_T c10_b_C[3])
{
  int32_T c10_i36;
  int32_T c10_i37;
  real_T c10_b_A[9];
  int32_T c10_i38;
  real_T c10_b_B[3];
  for (c10_i36 = 0; c10_i36 < 3; c10_i36++) {
    c10_b_C[c10_i36] = c10_C[c10_i36];
  }

  for (c10_i37 = 0; c10_i37 < 9; c10_i37++) {
    c10_b_A[c10_i37] = c10_A[c10_i37];
  }

  for (c10_i38 = 0; c10_i38 < 3; c10_i38++) {
    c10_b_B[c10_i38] = c10_B[c10_i38];
  }

  c10_b_eml_xgemm(chartInstance, c10_b_A, c10_b_B, c10_b_C);
}

static const mxArray *c10_e_sf_marshallOut(void *chartInstanceVoid, void
  *c10_inData)
{
  const mxArray *c10_mxArrayOutData = NULL;
  int32_T c10_u;
  const mxArray *c10_y = NULL;
  SFc10_sixDOF_SIMInstanceStruct *chartInstance;
  chartInstance = (SFc10_sixDOF_SIMInstanceStruct *)chartInstanceVoid;
  c10_mxArrayOutData = NULL;
  c10_u = *(int32_T *)c10_inData;
  c10_y = NULL;
  sf_mex_assign(&c10_y, sf_mex_create("y", &c10_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c10_mxArrayOutData, c10_y, false);
  return c10_mxArrayOutData;
}

static int32_T c10_f_emlrt_marshallIn(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId)
{
  int32_T c10_y;
  int32_T c10_i39;
  (void)chartInstance;
  sf_mex_import(c10_parentId, sf_mex_dup(c10_u), &c10_i39, 1, 6, 0U, 0, 0U, 0);
  c10_y = c10_i39;
  sf_mex_destroy(&c10_u);
  return c10_y;
}

static void c10_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c10_mxArrayInData, const char_T *c10_varName, void *c10_outData)
{
  const mxArray *c10_b_sfEvent;
  const char_T *c10_identifier;
  emlrtMsgIdentifier c10_thisId;
  int32_T c10_y;
  SFc10_sixDOF_SIMInstanceStruct *chartInstance;
  chartInstance = (SFc10_sixDOF_SIMInstanceStruct *)chartInstanceVoid;
  c10_b_sfEvent = sf_mex_dup(c10_mxArrayInData);
  c10_identifier = c10_varName;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_y = c10_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c10_b_sfEvent),
    &c10_thisId);
  sf_mex_destroy(&c10_b_sfEvent);
  *(int32_T *)c10_outData = c10_y;
  sf_mex_destroy(&c10_mxArrayInData);
}

static uint8_T c10_g_emlrt_marshallIn(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance, const mxArray *c10_b_is_active_c10_sixDOF_SIM, const char_T
  *c10_identifier)
{
  uint8_T c10_y;
  emlrtMsgIdentifier c10_thisId;
  c10_thisId.fIdentifier = c10_identifier;
  c10_thisId.fParent = NULL;
  c10_y = c10_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c10_b_is_active_c10_sixDOF_SIM), &c10_thisId);
  sf_mex_destroy(&c10_b_is_active_c10_sixDOF_SIM);
  return c10_y;
}

static uint8_T c10_h_emlrt_marshallIn(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance, const mxArray *c10_u, const emlrtMsgIdentifier *c10_parentId)
{
  uint8_T c10_y;
  uint8_T c10_u0;
  (void)chartInstance;
  sf_mex_import(c10_parentId, sf_mex_dup(c10_u), &c10_u0, 1, 3, 0U, 0, 0U, 0);
  c10_y = c10_u0;
  sf_mex_destroy(&c10_u);
  return c10_y;
}

static void c10_b_eml_xgemm(SFc10_sixDOF_SIMInstanceStruct *chartInstance,
  real_T c10_A[9], real_T c10_B[3], real_T c10_C[3])
{
  int32_T c10_i40;
  int32_T c10_i41;
  int32_T c10_i42;
  (void)chartInstance;
  for (c10_i40 = 0; c10_i40 < 3; c10_i40++) {
    c10_C[c10_i40] = 0.0;
    c10_i41 = 0;
    for (c10_i42 = 0; c10_i42 < 3; c10_i42++) {
      c10_C[c10_i40] += c10_A[c10_i41 + c10_i40] * c10_B[c10_i42];
      c10_i41 += 3;
    }
  }
}

static void init_dsm_address_info(SFc10_sixDOF_SIMInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc10_sixDOF_SIMInstanceStruct
  *chartInstance)
{
  chartInstance->c10_c = (real_T (*)[3])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c10_cx = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c10_state = (real_T (*)[6])ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c10_A_DCM = (real_T (*)[9])ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c10_cy = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c10_cz = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c10_R = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 5);
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

void sf_c10_sixDOF_SIM_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1435466319U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2888563431U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1752923641U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2668277104U);
}

mxArray* sf_c10_sixDOF_SIM_get_post_codegen_info(void);
mxArray *sf_c10_sixDOF_SIM_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("2V1krXNbbqDkSqbs8bgAgE");
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
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,5,3,dataFields);

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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c10_sixDOF_SIM_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c10_sixDOF_SIM_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c10_sixDOF_SIM_jit_fallback_info(void)
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

mxArray *sf_c10_sixDOF_SIM_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c10_sixDOF_SIM_get_post_codegen_info(void)
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

static const mxArray *sf_get_sim_state_info_c10_sixDOF_SIM(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x6'type','srcId','name','auxInfo'{{M[1],M[4],T\"R\",},{M[1],M[11],T\"c\",},{M[1],M[5],T\"cx\",},{M[1],M[9],T\"cy\",},{M[1],M[10],T\"cz\",},{M[8],M[0],T\"is_active_c10_sixDOF_SIM\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 6, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c10_sixDOF_SIM_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc10_sixDOF_SIMInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc10_sixDOF_SIMInstanceStruct *) chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _sixDOF_SIMMachineNumber_,
           10,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"state");
          _SFD_SET_DATA_PROPS(1,1,1,0,"A_DCM");
          _SFD_SET_DATA_PROPS(2,2,0,1,"c");
          _SFD_SET_DATA_PROPS(3,2,0,1,"cx");
          _SFD_SET_DATA_PROPS(4,2,0,1,"cy");
          _SFD_SET_DATA_PROPS(5,2,0,1,"cz");
          _SFD_SET_DATA_PROPS(6,2,0,1,"R");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,206);
        _SFD_CV_INIT_SCRIPT(0,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"getP2IfromA_DCM",0,-1,196);

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c10_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c10_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c10_b_sf_marshallOut,(MexInFcnForType)
            c10_b_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c10_sf_marshallOut,(MexInFcnForType)c10_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c10_sf_marshallOut,(MexInFcnForType)c10_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c10_sf_marshallOut,(MexInFcnForType)c10_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c10_sf_marshallOut,(MexInFcnForType)c10_sf_marshallIn);
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
    SFc10_sixDOF_SIMInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc10_sixDOF_SIMInstanceStruct *) chartInfo->chartInstance;
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(2U, *chartInstance->c10_c);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c10_cx);
        _SFD_SET_DATA_VALUE_PTR(0U, *chartInstance->c10_state);
        _SFD_SET_DATA_VALUE_PTR(1U, *chartInstance->c10_A_DCM);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c10_cy);
        _SFD_SET_DATA_VALUE_PTR(5U, chartInstance->c10_cz);
        _SFD_SET_DATA_VALUE_PTR(6U, chartInstance->c10_R);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "s9FuSULbLFiZ2E1y1agHL3";
}

static void sf_opaque_initialize_c10_sixDOF_SIM(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc10_sixDOF_SIMInstanceStruct*) chartInstanceVar)
    ->S,0);
  initialize_params_c10_sixDOF_SIM((SFc10_sixDOF_SIMInstanceStruct*)
    chartInstanceVar);
  initialize_c10_sixDOF_SIM((SFc10_sixDOF_SIMInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c10_sixDOF_SIM(void *chartInstanceVar)
{
  enable_c10_sixDOF_SIM((SFc10_sixDOF_SIMInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c10_sixDOF_SIM(void *chartInstanceVar)
{
  disable_c10_sixDOF_SIM((SFc10_sixDOF_SIMInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c10_sixDOF_SIM(void *chartInstanceVar)
{
  sf_gateway_c10_sixDOF_SIM((SFc10_sixDOF_SIMInstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c10_sixDOF_SIM(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c10_sixDOF_SIM((SFc10_sixDOF_SIMInstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c10_sixDOF_SIM(SimStruct* S, const mxArray
  *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c10_sixDOF_SIM((SFc10_sixDOF_SIMInstanceStruct*)
    chartInfo->chartInstance, st);
}

static void sf_opaque_terminate_c10_sixDOF_SIM(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc10_sixDOF_SIMInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_sixDOF_SIM_optimization_info();
    }

    finalize_c10_sixDOF_SIM((SFc10_sixDOF_SIMInstanceStruct*) chartInstanceVar);
    utFree(chartInstanceVar);
    if (crtInfo != NULL) {
      utFree(crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc10_sixDOF_SIM((SFc10_sixDOF_SIMInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c10_sixDOF_SIM(SimStruct *S)
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
    initialize_params_c10_sixDOF_SIM((SFc10_sixDOF_SIMInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c10_sixDOF_SIM(SimStruct *S)
{
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_sixDOF_SIM_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,
      10);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,10,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,10);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,10,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,10,5);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=5; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 2; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,10);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(972779332U));
  ssSetChecksum1(S,(4220543234U));
  ssSetChecksum2(S,(3421813154U));
  ssSetChecksum3(S,(1305055498U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c10_sixDOF_SIM(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c10_sixDOF_SIM(SimStruct *S)
{
  SFc10_sixDOF_SIMInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc10_sixDOF_SIMInstanceStruct *)utMalloc(sizeof
    (SFc10_sixDOF_SIMInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc10_sixDOF_SIMInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c10_sixDOF_SIM;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c10_sixDOF_SIM;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c10_sixDOF_SIM;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c10_sixDOF_SIM;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c10_sixDOF_SIM;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c10_sixDOF_SIM;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c10_sixDOF_SIM;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c10_sixDOF_SIM;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c10_sixDOF_SIM;
  chartInstance->chartInfo.mdlStart = mdlStart_c10_sixDOF_SIM;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c10_sixDOF_SIM;
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

void c10_sixDOF_SIM_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c10_sixDOF_SIM(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c10_sixDOF_SIM(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c10_sixDOF_SIM(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c10_sixDOF_SIM_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
