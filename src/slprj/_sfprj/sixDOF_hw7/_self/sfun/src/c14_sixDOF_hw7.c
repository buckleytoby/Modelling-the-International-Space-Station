/* Include files */

#include <stddef.h>
#include "blas.h"
#include "sixDOF_hw7_sfun.h"
#include "c14_sixDOF_hw7.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "sixDOF_hw7_sfun_debug_macros.h"
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
static const char * c14_debug_family_names[18] = { "B", "x_k", "nargin",
  "nargout", "x", "y_meas", "u", "dt", "I_principle", "P", "Q", "R", "H", "t",
  "x_ekf", "P_ekf", "z_ekf", "PHI" };

static const char * c14_b_debug_family_names[15] = { "Ix", "Iy", "Iz", "kx",
  "ky", "kz", "A_", "B_", "nargin", "nargout", "w", "dt", "I_principle", "PHI",
  "B" };

static const char * c14_c_debug_family_names[18] = { "x_k_", "P_k_", "K_k", "t",
  "nargin", "nargout", "x_k_1", "y_k", "u_k_1", "PHI_k_1", "B", "P_k_1", "Q_k_1",
  "R_k", "H_k", "x_k", "P_k", "z_k" };

/* Function Declarations */
static void initialize_c14_sixDOF_hw7(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance);
static void initialize_params_c14_sixDOF_hw7(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance);
static void enable_c14_sixDOF_hw7(SFc14_sixDOF_hw7InstanceStruct *chartInstance);
static void disable_c14_sixDOF_hw7(SFc14_sixDOF_hw7InstanceStruct *chartInstance);
static void c14_update_debugger_state_c14_sixDOF_hw7
  (SFc14_sixDOF_hw7InstanceStruct *chartInstance);
static const mxArray *get_sim_state_c14_sixDOF_hw7
  (SFc14_sixDOF_hw7InstanceStruct *chartInstance);
static void set_sim_state_c14_sixDOF_hw7(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance, const mxArray *c14_st);
static void finalize_c14_sixDOF_hw7(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance);
static void sf_gateway_c14_sixDOF_hw7(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance);
static void mdl_start_c14_sixDOF_hw7(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance);
static void initSimStructsc14_sixDOF_hw7(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance);
static void c14_linearEuler(SFc14_sixDOF_hw7InstanceStruct *chartInstance,
  real_T c14_w[3], real_T c14_b_dt, real_T c14_b_I_principle[9], real_T
  c14_b_PHI[9], real_T c14_B[9]);
static void c14_EKF(SFc14_sixDOF_hw7InstanceStruct *chartInstance, real_T
                    c14_x_k_1[3], real_T c14_y_k[3], real_T c14_u_k_1[3], real_T
                    c14_PHI_k_1[9], real_T c14_B[9], real_T c14_P_k_1[9], real_T
                    c14_Q_k_1[9], real_T c14_R_k[9], real_T c14_H_k[9], real_T
                    c14_x_k[3], real_T c14_P_k[9], real_T c14_z_k[3]);
static void init_script_number_translation(uint32_T c14_machineNumber, uint32_T
  c14_chartNumber, uint32_T c14_instanceNumber);
static const mxArray *c14_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData);
static void c14_emlrt_marshallIn(SFc14_sixDOF_hw7InstanceStruct *chartInstance,
  const mxArray *c14_b_PHI, const char_T *c14_identifier, real_T c14_y[9]);
static void c14_b_emlrt_marshallIn(SFc14_sixDOF_hw7InstanceStruct *chartInstance,
  const mxArray *c14_b_u, const emlrtMsgIdentifier *c14_parentId, real_T c14_y[9]);
static void c14_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData);
static const mxArray *c14_b_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData);
static void c14_c_emlrt_marshallIn(SFc14_sixDOF_hw7InstanceStruct *chartInstance,
  const mxArray *c14_b_z_ekf, const char_T *c14_identifier, real_T c14_y[3]);
static void c14_d_emlrt_marshallIn(SFc14_sixDOF_hw7InstanceStruct *chartInstance,
  const mxArray *c14_b_u, const emlrtMsgIdentifier *c14_parentId, real_T c14_y[3]);
static void c14_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData);
static const mxArray *c14_c_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData);
static real_T c14_e_emlrt_marshallIn(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance, const mxArray *c14_b_u, const emlrtMsgIdentifier *c14_parentId);
static void c14_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData);
static void c14_info_helper(const mxArray **c14_info);
static const mxArray *c14_emlrt_marshallOut(const char * c14_b_u);
static const mxArray *c14_b_emlrt_marshallOut(const uint32_T c14_b_u);
static real_T c14_rdivide(SFc14_sixDOF_hw7InstanceStruct *chartInstance, real_T
  c14_b_x, real_T c14_y);
static void c14_eml_scalar_eg(SFc14_sixDOF_hw7InstanceStruct *chartInstance);
static void c14_threshold(SFc14_sixDOF_hw7InstanceStruct *chartInstance);
static void c14_b_eml_scalar_eg(SFc14_sixDOF_hw7InstanceStruct *chartInstance);
static void c14_mrdivide(SFc14_sixDOF_hw7InstanceStruct *chartInstance, real_T
  c14_A[9], real_T c14_B[9], real_T c14_y[9]);
static void c14_eml_lusolve(SFc14_sixDOF_hw7InstanceStruct *chartInstance,
  real_T c14_A[9], real_T c14_B[9], real_T c14_X[9]);
static void c14_eml_warning(SFc14_sixDOF_hw7InstanceStruct *chartInstance);
static void c14_eye(SFc14_sixDOF_hw7InstanceStruct *chartInstance, real_T c14_I
                    [9]);
static const mxArray *c14_d_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData);
static int32_T c14_f_emlrt_marshallIn(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance, const mxArray *c14_b_u, const emlrtMsgIdentifier *c14_parentId);
static void c14_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData);
static uint8_T c14_g_emlrt_marshallIn(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance, const mxArray *c14_b_is_active_c14_sixDOF_hw7, const char_T
  *c14_identifier);
static uint8_T c14_h_emlrt_marshallIn(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance, const mxArray *c14_b_u, const emlrtMsgIdentifier *c14_parentId);
static void init_dsm_address_info(SFc14_sixDOF_hw7InstanceStruct *chartInstance);
static void init_simulink_io_address(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c14_sixDOF_hw7(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc14_sixDOF_hw7(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c14_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c14_is_active_c14_sixDOF_hw7 = 0U;
}

static void initialize_params_c14_sixDOF_hw7(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void enable_c14_sixDOF_hw7(SFc14_sixDOF_hw7InstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c14_sixDOF_hw7(SFc14_sixDOF_hw7InstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c14_update_debugger_state_c14_sixDOF_hw7
  (SFc14_sixDOF_hw7InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c14_sixDOF_hw7
  (SFc14_sixDOF_hw7InstanceStruct *chartInstance)
{
  const mxArray *c14_st;
  const mxArray *c14_y = NULL;
  const mxArray *c14_b_y = NULL;
  const mxArray *c14_c_y = NULL;
  const mxArray *c14_d_y = NULL;
  const mxArray *c14_e_y = NULL;
  uint8_T c14_hoistedGlobal;
  uint8_T c14_b_u;
  const mxArray *c14_f_y = NULL;
  c14_st = NULL;
  c14_st = NULL;
  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_createcellmatrix(5, 1), false);
  c14_b_y = NULL;
  sf_mex_assign(&c14_b_y, sf_mex_create("y", *chartInstance->c14_PHI, 0, 0U, 1U,
    0U, 2, 3, 3), false);
  sf_mex_setcell(c14_y, 0, c14_b_y);
  c14_c_y = NULL;
  sf_mex_assign(&c14_c_y, sf_mex_create("y", *chartInstance->c14_P_ekf, 0, 0U,
    1U, 0U, 2, 3, 3), false);
  sf_mex_setcell(c14_y, 1, c14_c_y);
  c14_d_y = NULL;
  sf_mex_assign(&c14_d_y, sf_mex_create("y", *chartInstance->c14_x_ekf, 0, 0U,
    1U, 0U, 1, 3), false);
  sf_mex_setcell(c14_y, 2, c14_d_y);
  c14_e_y = NULL;
  sf_mex_assign(&c14_e_y, sf_mex_create("y", *chartInstance->c14_z_ekf, 0, 0U,
    1U, 0U, 1, 3), false);
  sf_mex_setcell(c14_y, 3, c14_e_y);
  c14_hoistedGlobal = chartInstance->c14_is_active_c14_sixDOF_hw7;
  c14_b_u = c14_hoistedGlobal;
  c14_f_y = NULL;
  sf_mex_assign(&c14_f_y, sf_mex_create("y", &c14_b_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c14_y, 4, c14_f_y);
  sf_mex_assign(&c14_st, c14_y, false);
  return c14_st;
}

static void set_sim_state_c14_sixDOF_hw7(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance, const mxArray *c14_st)
{
  const mxArray *c14_b_u;
  real_T c14_dv0[9];
  int32_T c14_i0;
  real_T c14_dv1[9];
  int32_T c14_i1;
  real_T c14_dv2[3];
  int32_T c14_i2;
  real_T c14_dv3[3];
  int32_T c14_i3;
  chartInstance->c14_doneDoubleBufferReInit = true;
  c14_b_u = sf_mex_dup(c14_st);
  c14_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("PHI", c14_b_u,
    0)), "PHI", c14_dv0);
  for (c14_i0 = 0; c14_i0 < 9; c14_i0++) {
    (*chartInstance->c14_PHI)[c14_i0] = c14_dv0[c14_i0];
  }

  c14_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("P_ekf", c14_b_u,
    1)), "P_ekf", c14_dv1);
  for (c14_i1 = 0; c14_i1 < 9; c14_i1++) {
    (*chartInstance->c14_P_ekf)[c14_i1] = c14_dv1[c14_i1];
  }

  c14_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("x_ekf",
    c14_b_u, 2)), "x_ekf", c14_dv2);
  for (c14_i2 = 0; c14_i2 < 3; c14_i2++) {
    (*chartInstance->c14_x_ekf)[c14_i2] = c14_dv2[c14_i2];
  }

  c14_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("z_ekf",
    c14_b_u, 3)), "z_ekf", c14_dv3);
  for (c14_i3 = 0; c14_i3 < 3; c14_i3++) {
    (*chartInstance->c14_z_ekf)[c14_i3] = c14_dv3[c14_i3];
  }

  chartInstance->c14_is_active_c14_sixDOF_hw7 = c14_g_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell("is_active_c14_sixDOF_hw7",
       c14_b_u, 4)), "is_active_c14_sixDOF_hw7");
  sf_mex_destroy(&c14_b_u);
  c14_update_debugger_state_c14_sixDOF_hw7(chartInstance);
  sf_mex_destroy(&c14_st);
}

static void finalize_c14_sixDOF_hw7(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c14_sixDOF_hw7(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance)
{
  int32_T c14_i4;
  int32_T c14_i5;
  int32_T c14_i6;
  int32_T c14_i7;
  int32_T c14_i8;
  int32_T c14_i9;
  int32_T c14_i10;
  int32_T c14_i11;
  real_T c14_hoistedGlobal;
  real_T c14_b_hoistedGlobal;
  int32_T c14_i12;
  real_T c14_b_x[3];
  int32_T c14_i13;
  real_T c14_b_y_meas[3];
  int32_T c14_i14;
  real_T c14_b_u[3];
  real_T c14_b_dt;
  int32_T c14_i15;
  real_T c14_b_I_principle[9];
  int32_T c14_i16;
  real_T c14_b_P[9];
  int32_T c14_i17;
  real_T c14_b_Q[9];
  int32_T c14_i18;
  real_T c14_b_R[9];
  int32_T c14_i19;
  real_T c14_b_H[9];
  real_T c14_b_t;
  uint32_T c14_debug_family_var_map[18];
  real_T c14_B[9];
  real_T c14_x_k[3];
  real_T c14_nargin = 10.0;
  real_T c14_nargout = 4.0;
  real_T c14_b_x_ekf[3];
  real_T c14_b_P_ekf[9];
  real_T c14_b_z_ekf[3];
  real_T c14_b_PHI[9];
  int32_T c14_i20;
  real_T c14_c_y_meas[3];
  int32_T c14_i21;
  real_T c14_c_I_principle[9];
  real_T c14_b_B[9];
  real_T c14_c_PHI[9];
  int32_T c14_i22;
  int32_T c14_i23;
  int32_T c14_i24;
  real_T c14_c_x[3];
  int32_T c14_i25;
  real_T c14_d_y_meas[3];
  int32_T c14_i26;
  real_T c14_c_u[3];
  int32_T c14_i27;
  real_T c14_d_PHI[9];
  int32_T c14_i28;
  real_T c14_c_B[9];
  int32_T c14_i29;
  real_T c14_c_P[9];
  int32_T c14_i30;
  real_T c14_c_Q[9];
  int32_T c14_i31;
  real_T c14_c_R[9];
  int32_T c14_i32;
  real_T c14_c_H[9];
  real_T c14_c_z_ekf[3];
  real_T c14_b_x_k[3];
  int32_T c14_i33;
  int32_T c14_i34;
  int32_T c14_i35;
  int32_T c14_i36;
  int32_T c14_i37;
  int32_T c14_i38;
  int32_T c14_i39;
  int32_T c14_i40;
  int32_T c14_i41;
  int32_T c14_i42;
  int32_T c14_i43;
  int32_T c14_i44;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 13U, chartInstance->c14_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c14_t, 9U, 1U, 0U,
                        chartInstance->c14_sfEvent, false);
  for (c14_i4 = 0; c14_i4 < 9; c14_i4++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c14_H)[c14_i4], 8U, 1U, 0U,
                          chartInstance->c14_sfEvent, false);
  }

  for (c14_i5 = 0; c14_i5 < 9; c14_i5++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c14_R)[c14_i5], 7U, 1U, 0U,
                          chartInstance->c14_sfEvent, false);
  }

  for (c14_i6 = 0; c14_i6 < 9; c14_i6++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c14_Q)[c14_i6], 6U, 1U, 0U,
                          chartInstance->c14_sfEvent, false);
  }

  for (c14_i7 = 0; c14_i7 < 9; c14_i7++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c14_P)[c14_i7], 5U, 1U, 0U,
                          chartInstance->c14_sfEvent, false);
  }

  for (c14_i8 = 0; c14_i8 < 9; c14_i8++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c14_I_principle)[c14_i8], 4U, 1U, 0U,
                          chartInstance->c14_sfEvent, false);
  }

  _SFD_DATA_RANGE_CHECK(*chartInstance->c14_dt, 3U, 1U, 0U,
                        chartInstance->c14_sfEvent, false);
  for (c14_i9 = 0; c14_i9 < 3; c14_i9++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c14_u)[c14_i9], 2U, 1U, 0U,
                          chartInstance->c14_sfEvent, false);
  }

  for (c14_i10 = 0; c14_i10 < 3; c14_i10++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c14_y_meas)[c14_i10], 1U, 1U, 0U,
                          chartInstance->c14_sfEvent, false);
  }

  for (c14_i11 = 0; c14_i11 < 3; c14_i11++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c14_x)[c14_i11], 0U, 1U, 0U,
                          chartInstance->c14_sfEvent, false);
  }

  chartInstance->c14_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 13U, chartInstance->c14_sfEvent);
  c14_hoistedGlobal = *chartInstance->c14_dt;
  c14_b_hoistedGlobal = *chartInstance->c14_t;
  for (c14_i12 = 0; c14_i12 < 3; c14_i12++) {
    c14_b_x[c14_i12] = (*chartInstance->c14_x)[c14_i12];
  }

  for (c14_i13 = 0; c14_i13 < 3; c14_i13++) {
    c14_b_y_meas[c14_i13] = (*chartInstance->c14_y_meas)[c14_i13];
  }

  for (c14_i14 = 0; c14_i14 < 3; c14_i14++) {
    c14_b_u[c14_i14] = (*chartInstance->c14_u)[c14_i14];
  }

  c14_b_dt = c14_hoistedGlobal;
  for (c14_i15 = 0; c14_i15 < 9; c14_i15++) {
    c14_b_I_principle[c14_i15] = (*chartInstance->c14_I_principle)[c14_i15];
  }

  for (c14_i16 = 0; c14_i16 < 9; c14_i16++) {
    c14_b_P[c14_i16] = (*chartInstance->c14_P)[c14_i16];
  }

  for (c14_i17 = 0; c14_i17 < 9; c14_i17++) {
    c14_b_Q[c14_i17] = (*chartInstance->c14_Q)[c14_i17];
  }

  for (c14_i18 = 0; c14_i18 < 9; c14_i18++) {
    c14_b_R[c14_i18] = (*chartInstance->c14_R)[c14_i18];
  }

  for (c14_i19 = 0; c14_i19 < 9; c14_i19++) {
    c14_b_H[c14_i19] = (*chartInstance->c14_H)[c14_i19];
  }

  c14_b_t = c14_b_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 18U, 18U, c14_debug_family_names,
    c14_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_B, 0U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_x_k, 1U, c14_b_sf_marshallOut,
    c14_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargin, 2U, c14_c_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargout, 3U, c14_c_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c14_b_x, 4U, c14_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c14_b_y_meas, 5U, c14_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c14_b_u, 6U, c14_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c14_b_dt, 7U, c14_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c14_b_I_principle, 8U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c14_b_P, 9U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c14_b_Q, 10U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c14_b_R, 11U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c14_b_H, 12U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c14_b_t, 13U, c14_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_b_x_ekf, 14U, c14_b_sf_marshallOut,
    c14_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_b_P_ekf, 15U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_b_z_ekf, 16U, c14_b_sf_marshallOut,
    c14_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_b_PHI, 17U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 4);
  for (c14_i20 = 0; c14_i20 < 3; c14_i20++) {
    c14_c_y_meas[c14_i20] = c14_b_y_meas[c14_i20];
  }

  for (c14_i21 = 0; c14_i21 < 9; c14_i21++) {
    c14_c_I_principle[c14_i21] = c14_b_I_principle[c14_i21];
  }

  c14_linearEuler(chartInstance, c14_c_y_meas, c14_b_dt, c14_c_I_principle,
                  c14_c_PHI, c14_b_B);
  for (c14_i22 = 0; c14_i22 < 9; c14_i22++) {
    c14_b_PHI[c14_i22] = c14_c_PHI[c14_i22];
  }

  for (c14_i23 = 0; c14_i23 < 9; c14_i23++) {
    c14_B[c14_i23] = c14_b_B[c14_i23];
  }

  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 5);
  for (c14_i24 = 0; c14_i24 < 3; c14_i24++) {
    c14_c_x[c14_i24] = c14_b_x[c14_i24];
  }

  for (c14_i25 = 0; c14_i25 < 3; c14_i25++) {
    c14_d_y_meas[c14_i25] = c14_b_y_meas[c14_i25];
  }

  for (c14_i26 = 0; c14_i26 < 3; c14_i26++) {
    c14_c_u[c14_i26] = c14_b_u[c14_i26];
  }

  for (c14_i27 = 0; c14_i27 < 9; c14_i27++) {
    c14_d_PHI[c14_i27] = c14_b_PHI[c14_i27];
  }

  for (c14_i28 = 0; c14_i28 < 9; c14_i28++) {
    c14_c_B[c14_i28] = c14_B[c14_i28];
  }

  for (c14_i29 = 0; c14_i29 < 9; c14_i29++) {
    c14_c_P[c14_i29] = c14_b_P[c14_i29];
  }

  for (c14_i30 = 0; c14_i30 < 9; c14_i30++) {
    c14_c_Q[c14_i30] = c14_b_Q[c14_i30];
  }

  for (c14_i31 = 0; c14_i31 < 9; c14_i31++) {
    c14_c_R[c14_i31] = c14_b_R[c14_i31];
  }

  for (c14_i32 = 0; c14_i32 < 9; c14_i32++) {
    c14_c_H[c14_i32] = c14_b_H[c14_i32];
  }

  c14_EKF(chartInstance, c14_c_x, c14_d_y_meas, c14_c_u, c14_d_PHI, c14_c_B,
          c14_c_P, c14_c_Q, c14_c_R, c14_c_H, c14_b_x_k, c14_c_PHI, c14_c_z_ekf);
  for (c14_i33 = 0; c14_i33 < 3; c14_i33++) {
    c14_x_k[c14_i33] = c14_b_x_k[c14_i33];
  }

  for (c14_i34 = 0; c14_i34 < 9; c14_i34++) {
    c14_b_P_ekf[c14_i34] = c14_c_PHI[c14_i34];
  }

  for (c14_i35 = 0; c14_i35 < 3; c14_i35++) {
    c14_b_z_ekf[c14_i35] = c14_c_z_ekf[c14_i35];
  }

  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, 8);
  for (c14_i36 = 0; c14_i36 < 3; c14_i36++) {
    c14_b_x_ekf[c14_i36] = c14_x_k[c14_i36];
  }

  _SFD_EML_CALL(0U, chartInstance->c14_sfEvent, -8);
  _SFD_SYMBOL_SCOPE_POP();
  for (c14_i37 = 0; c14_i37 < 3; c14_i37++) {
    (*chartInstance->c14_x_ekf)[c14_i37] = c14_b_x_ekf[c14_i37];
  }

  for (c14_i38 = 0; c14_i38 < 9; c14_i38++) {
    (*chartInstance->c14_P_ekf)[c14_i38] = c14_b_P_ekf[c14_i38];
  }

  for (c14_i39 = 0; c14_i39 < 3; c14_i39++) {
    (*chartInstance->c14_z_ekf)[c14_i39] = c14_b_z_ekf[c14_i39];
  }

  for (c14_i40 = 0; c14_i40 < 9; c14_i40++) {
    (*chartInstance->c14_PHI)[c14_i40] = c14_b_PHI[c14_i40];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 13U, chartInstance->c14_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_sixDOF_hw7MachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c14_i41 = 0; c14_i41 < 3; c14_i41++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c14_x_ekf)[c14_i41], 10U, 1U, 0U,
                          chartInstance->c14_sfEvent, false);
  }

  for (c14_i42 = 0; c14_i42 < 9; c14_i42++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c14_P_ekf)[c14_i42], 11U, 1U, 0U,
                          chartInstance->c14_sfEvent, false);
  }

  for (c14_i43 = 0; c14_i43 < 3; c14_i43++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c14_z_ekf)[c14_i43], 12U, 1U, 0U,
                          chartInstance->c14_sfEvent, false);
  }

  for (c14_i44 = 0; c14_i44 < 9; c14_i44++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c14_PHI)[c14_i44], 13U, 1U, 0U,
                          chartInstance->c14_sfEvent, false);
  }
}

static void mdl_start_c14_sixDOF_hw7(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void initSimStructsc14_sixDOF_hw7(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c14_linearEuler(SFc14_sixDOF_hw7InstanceStruct *chartInstance,
  real_T c14_w[3], real_T c14_b_dt, real_T c14_b_I_principle[9], real_T
  c14_b_PHI[9], real_T c14_B[9])
{
  uint32_T c14_debug_family_var_map[15];
  real_T c14_Ix;
  real_T c14_Iy;
  real_T c14_Iz;
  real_T c14_kx;
  real_T c14_ky;
  real_T c14_kz;
  real_T c14_A_[9];
  real_T c14_B_[9];
  real_T c14_nargin = 3.0;
  real_T c14_nargout = 2.0;
  real_T c14_A;
  real_T c14_b_B;
  real_T c14_b_A;
  real_T c14_c_B;
  real_T c14_c_A;
  real_T c14_d_B;
  int32_T c14_i45;
  static real_T c14_b[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };

  real_T c14_a;
  int32_T c14_i46;
  real_T c14_b_b[9];
  int32_T c14_i47;
  int32_T c14_i48;
  real_T c14_I[9];
  int32_T c14_k;
  int32_T c14_b_k;
  int32_T c14_i49;
  real_T c14_b_a;
  int32_T c14_i50;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 15U, 15U, c14_b_debug_family_names,
    c14_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_Ix, 0U, c14_c_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_Iy, 1U, c14_c_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_Iz, 2U, c14_c_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_kx, 3U, c14_c_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_ky, 4U, c14_c_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_kz, 5U, c14_c_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_A_, 6U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c14_B_, 7U, c14_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargin, 8U, c14_c_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargout, 9U, c14_c_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_w, 10U, c14_b_sf_marshallOut,
    c14_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_b_dt, 11U, c14_c_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_b_I_principle, 12U,
    c14_sf_marshallOut, c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_b_PHI, 13U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_B, 14U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 4);
  c14_Ix = c14_b_I_principle[0];
  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 5);
  c14_Iy = c14_b_I_principle[4];
  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 6);
  c14_Iz = c14_b_I_principle[8];
  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 7);
  c14_A = c14_Iz - c14_Iy;
  c14_b_B = c14_Ix;
  c14_kx = c14_rdivide(chartInstance, c14_A, c14_b_B);
  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 8);
  c14_b_A = c14_Ix - c14_Iz;
  c14_c_B = c14_Iy;
  c14_ky = c14_rdivide(chartInstance, c14_b_A, c14_c_B);
  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 9);
  c14_c_A = c14_Iy - c14_Ix;
  c14_d_B = c14_Iz;
  c14_kz = c14_rdivide(chartInstance, c14_c_A, c14_d_B);
  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 10);
  c14_A_[0] = 0.0;
  c14_A_[3] = -c14_kx * c14_w[2];
  c14_A_[6] = -c14_kx * c14_w[1];
  c14_A_[1] = -c14_ky * c14_w[2];
  c14_A_[4] = 0.0;
  c14_A_[7] = -c14_ky * c14_w[0];
  c14_A_[2] = -c14_kz * c14_w[1];
  c14_A_[5] = -c14_kz * c14_w[0];
  c14_A_[8] = 0.0;
  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 13);
  for (c14_i45 = 0; c14_i45 < 9; c14_i45++) {
    c14_B_[c14_i45] = c14_b[c14_i45];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 15);
  c14_a = c14_b_dt;
  for (c14_i46 = 0; c14_i46 < 9; c14_i46++) {
    c14_b_b[c14_i46] = c14_A_[c14_i46];
  }

  for (c14_i47 = 0; c14_i47 < 9; c14_i47++) {
    c14_b_b[c14_i47] *= c14_a;
  }

  for (c14_i48 = 0; c14_i48 < 9; c14_i48++) {
    c14_I[c14_i48] = 0.0;
  }

  for (c14_k = 1; c14_k < 4; c14_k++) {
    c14_b_k = c14_k;
    c14_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c14_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_b_k), 1, 3, 2, 0) - 1))
      - 1] = 1.0;
  }

  for (c14_i49 = 0; c14_i49 < 9; c14_i49++) {
    c14_b_PHI[c14_i49] = c14_b_b[c14_i49] + c14_I[c14_i49];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, 16);
  c14_b_a = c14_b_dt;
  for (c14_i50 = 0; c14_i50 < 9; c14_i50++) {
    c14_B[c14_i50] = c14_b_a * c14_b[c14_i50];
  }

  _SFD_SCRIPT_CALL(0U, chartInstance->c14_sfEvent, -16);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c14_EKF(SFc14_sixDOF_hw7InstanceStruct *chartInstance, real_T
                    c14_x_k_1[3], real_T c14_y_k[3], real_T c14_u_k_1[3], real_T
                    c14_PHI_k_1[9], real_T c14_B[9], real_T c14_P_k_1[9], real_T
                    c14_Q_k_1[9], real_T c14_R_k[9], real_T c14_H_k[9], real_T
                    c14_x_k[3], real_T c14_P_k[9], real_T c14_z_k[3])
{
  uint32_T c14_debug_family_var_map[18];
  real_T c14_x_k_[3];
  real_T c14_P_k_[9];
  real_T c14_K_k[9];
  real_T c14_b_t[9];
  real_T c14_nargin = 10.0;
  real_T c14_nargout = 3.0;
  int32_T c14_i51;
  real_T c14_a[9];
  int32_T c14_i52;
  real_T c14_b[3];
  int32_T c14_i53;
  real_T c14_y[3];
  int32_T c14_i54;
  int32_T c14_i55;
  int32_T c14_i56;
  int32_T c14_i57;
  int32_T c14_i58;
  real_T c14_b_y[3];
  int32_T c14_i59;
  int32_T c14_i60;
  int32_T c14_i61;
  int32_T c14_i62;
  int32_T c14_i63;
  real_T c14_b_b[9];
  int32_T c14_i64;
  int32_T c14_i65;
  int32_T c14_i66;
  real_T c14_c_y[9];
  int32_T c14_i67;
  int32_T c14_i68;
  int32_T c14_i69;
  int32_T c14_i70;
  int32_T c14_i71;
  int32_T c14_i72;
  int32_T c14_i73;
  int32_T c14_i74;
  int32_T c14_i75;
  real_T c14_d_y[9];
  int32_T c14_i76;
  int32_T c14_i77;
  int32_T c14_i78;
  int32_T c14_i79;
  int32_T c14_i80;
  int32_T c14_i81;
  int32_T c14_i82;
  int32_T c14_i83;
  int32_T c14_i84;
  int32_T c14_i85;
  int32_T c14_i86;
  int32_T c14_i87;
  int32_T c14_i88;
  int32_T c14_i89;
  int32_T c14_i90;
  int32_T c14_i91;
  int32_T c14_i92;
  int32_T c14_i93;
  int32_T c14_i94;
  int32_T c14_i95;
  int32_T c14_i96;
  int32_T c14_i97;
  int32_T c14_i98;
  int32_T c14_i99;
  int32_T c14_i100;
  int32_T c14_i101;
  int32_T c14_i102;
  int32_T c14_i103;
  int32_T c14_i104;
  int32_T c14_i105;
  int32_T c14_i106;
  int32_T c14_i107;
  int32_T c14_i108;
  int32_T c14_i109;
  int32_T c14_i110;
  int32_T c14_i111;
  int32_T c14_i112;
  int32_T c14_i113;
  real_T c14_e_y[9];
  int32_T c14_i114;
  int32_T c14_i115;
  int32_T c14_i116;
  real_T c14_f_y[9];
  int32_T c14_i117;
  real_T c14_g_y[9];
  real_T c14_dv4[9];
  int32_T c14_i118;
  int32_T c14_i119;
  int32_T c14_i120;
  int32_T c14_i121;
  int32_T c14_i122;
  int32_T c14_i123;
  int32_T c14_i124;
  int32_T c14_i125;
  int32_T c14_i126;
  int32_T c14_i127;
  int32_T c14_i128;
  int32_T c14_i129;
  int32_T c14_i130;
  int32_T c14_i131;
  int32_T c14_i132;
  int32_T c14_i133;
  int32_T c14_i134;
  int32_T c14_i135;
  int32_T c14_i136;
  int32_T c14_i137;
  int32_T c14_i138;
  int32_T c14_i139;
  int32_T c14_i140;
  int32_T c14_i141;
  int32_T c14_i142;
  int32_T c14_i143;
  int32_T c14_i144;
  int32_T c14_i145;
  int32_T c14_i146;
  int32_T c14_i147;
  int32_T c14_i148;
  int32_T c14_i149;
  int32_T c14_i150;
  int32_T c14_i151;
  int32_T c14_i152;
  int32_T c14_i153;
  int32_T c14_i154;
  int32_T c14_i155;
  int32_T c14_i156;
  int32_T c14_i157;
  int32_T c14_i158;
  int32_T c14_i159;
  int32_T c14_i160;
  int32_T c14_i161;
  int32_T c14_i162;
  int32_T c14_i163;
  int32_T c14_i164;
  int32_T c14_i165;
  int32_T c14_i166;
  int32_T c14_i167;
  int32_T c14_i168;
  int32_T c14_i169;
  int32_T c14_i170;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 18U, 18U, c14_c_debug_family_names,
    c14_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_x_k_, 0U, c14_b_sf_marshallOut,
    c14_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_P_k_, 1U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_K_k, 2U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_b_t, 3U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargin, 4U, c14_c_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c14_nargout, 5U, c14_c_sf_marshallOut,
    c14_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_x_k_1, 6U, c14_b_sf_marshallOut,
    c14_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_y_k, 7U, c14_b_sf_marshallOut,
    c14_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_u_k_1, 8U, c14_b_sf_marshallOut,
    c14_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_PHI_k_1, 9U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_B, 10U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_P_k_1, 11U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_Q_k_1, 12U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_R_k, 13U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_H_k, 14U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_x_k, 15U, c14_b_sf_marshallOut,
    c14_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_P_k, 16U, c14_sf_marshallOut,
    c14_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c14_z_k, 17U, c14_b_sf_marshallOut,
    c14_b_sf_marshallIn);
  CV_SCRIPT_FCN(1, 0);
  _SFD_SCRIPT_CALL(1U, chartInstance->c14_sfEvent, 13);
  for (c14_i51 = 0; c14_i51 < 9; c14_i51++) {
    c14_a[c14_i51] = c14_PHI_k_1[c14_i51];
  }

  for (c14_i52 = 0; c14_i52 < 3; c14_i52++) {
    c14_b[c14_i52] = c14_x_k_1[c14_i52];
  }

  c14_eml_scalar_eg(chartInstance);
  c14_eml_scalar_eg(chartInstance);
  c14_threshold(chartInstance);
  for (c14_i53 = 0; c14_i53 < 3; c14_i53++) {
    c14_y[c14_i53] = 0.0;
    c14_i54 = 0;
    for (c14_i55 = 0; c14_i55 < 3; c14_i55++) {
      c14_y[c14_i53] += c14_a[c14_i54 + c14_i53] * c14_b[c14_i55];
      c14_i54 += 3;
    }
  }

  for (c14_i56 = 0; c14_i56 < 9; c14_i56++) {
    c14_a[c14_i56] = c14_B[c14_i56];
  }

  for (c14_i57 = 0; c14_i57 < 3; c14_i57++) {
    c14_b[c14_i57] = c14_u_k_1[c14_i57];
  }

  c14_eml_scalar_eg(chartInstance);
  c14_eml_scalar_eg(chartInstance);
  c14_threshold(chartInstance);
  for (c14_i58 = 0; c14_i58 < 3; c14_i58++) {
    c14_b_y[c14_i58] = 0.0;
    c14_i59 = 0;
    for (c14_i60 = 0; c14_i60 < 3; c14_i60++) {
      c14_b_y[c14_i58] += c14_a[c14_i59 + c14_i58] * c14_b[c14_i60];
      c14_i59 += 3;
    }
  }

  for (c14_i61 = 0; c14_i61 < 3; c14_i61++) {
    c14_x_k_[c14_i61] = c14_y[c14_i61] + c14_b_y[c14_i61];
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c14_sfEvent, 14);
  for (c14_i62 = 0; c14_i62 < 9; c14_i62++) {
    c14_a[c14_i62] = c14_PHI_k_1[c14_i62];
  }

  for (c14_i63 = 0; c14_i63 < 9; c14_i63++) {
    c14_b_b[c14_i63] = c14_P_k_1[c14_i63];
  }

  c14_b_eml_scalar_eg(chartInstance);
  c14_b_eml_scalar_eg(chartInstance);
  c14_threshold(chartInstance);
  for (c14_i64 = 0; c14_i64 < 3; c14_i64++) {
    c14_i65 = 0;
    for (c14_i66 = 0; c14_i66 < 3; c14_i66++) {
      c14_c_y[c14_i65 + c14_i64] = 0.0;
      c14_i67 = 0;
      for (c14_i68 = 0; c14_i68 < 3; c14_i68++) {
        c14_c_y[c14_i65 + c14_i64] += c14_a[c14_i67 + c14_i64] * c14_b_b[c14_i68
          + c14_i65];
        c14_i67 += 3;
      }

      c14_i65 += 3;
    }
  }

  c14_i69 = 0;
  for (c14_i70 = 0; c14_i70 < 3; c14_i70++) {
    c14_i71 = 0;
    for (c14_i72 = 0; c14_i72 < 3; c14_i72++) {
      c14_b_b[c14_i72 + c14_i69] = c14_PHI_k_1[c14_i71 + c14_i70];
      c14_i71 += 3;
    }

    c14_i69 += 3;
  }

  c14_b_eml_scalar_eg(chartInstance);
  c14_b_eml_scalar_eg(chartInstance);
  c14_threshold(chartInstance);
  for (c14_i73 = 0; c14_i73 < 3; c14_i73++) {
    c14_i74 = 0;
    for (c14_i75 = 0; c14_i75 < 3; c14_i75++) {
      c14_d_y[c14_i74 + c14_i73] = 0.0;
      c14_i76 = 0;
      for (c14_i77 = 0; c14_i77 < 3; c14_i77++) {
        c14_d_y[c14_i74 + c14_i73] += c14_c_y[c14_i76 + c14_i73] *
          c14_b_b[c14_i77 + c14_i74];
        c14_i76 += 3;
      }

      c14_i74 += 3;
    }
  }

  for (c14_i78 = 0; c14_i78 < 9; c14_i78++) {
    c14_P_k_[c14_i78] = c14_d_y[c14_i78] + c14_Q_k_1[c14_i78];
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c14_sfEvent, 17);
  for (c14_i79 = 0; c14_i79 < 9; c14_i79++) {
    c14_a[c14_i79] = c14_H_k[c14_i79];
  }

  for (c14_i80 = 0; c14_i80 < 3; c14_i80++) {
    c14_b[c14_i80] = c14_x_k_[c14_i80];
  }

  c14_eml_scalar_eg(chartInstance);
  c14_eml_scalar_eg(chartInstance);
  for (c14_i81 = 0; c14_i81 < 3; c14_i81++) {
    c14_z_k[c14_i81] = 0.0;
  }

  for (c14_i82 = 0; c14_i82 < 3; c14_i82++) {
    c14_z_k[c14_i82] = 0.0;
  }

  for (c14_i83 = 0; c14_i83 < 3; c14_i83++) {
    c14_y[c14_i83] = c14_z_k[c14_i83];
  }

  for (c14_i84 = 0; c14_i84 < 3; c14_i84++) {
    c14_z_k[c14_i84] = c14_y[c14_i84];
  }

  c14_threshold(chartInstance);
  for (c14_i85 = 0; c14_i85 < 3; c14_i85++) {
    c14_y[c14_i85] = c14_z_k[c14_i85];
  }

  for (c14_i86 = 0; c14_i86 < 3; c14_i86++) {
    c14_z_k[c14_i86] = c14_y[c14_i86];
  }

  for (c14_i87 = 0; c14_i87 < 3; c14_i87++) {
    c14_z_k[c14_i87] = 0.0;
    c14_i88 = 0;
    for (c14_i89 = 0; c14_i89 < 3; c14_i89++) {
      c14_z_k[c14_i87] += c14_a[c14_i88 + c14_i87] * c14_b[c14_i89];
      c14_i88 += 3;
    }
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c14_sfEvent, 18);
  for (c14_i90 = 0; c14_i90 < 9; c14_i90++) {
    c14_a[c14_i90] = c14_P_k_[c14_i90];
  }

  c14_i91 = 0;
  for (c14_i92 = 0; c14_i92 < 3; c14_i92++) {
    c14_i93 = 0;
    for (c14_i94 = 0; c14_i94 < 3; c14_i94++) {
      c14_b_b[c14_i94 + c14_i91] = c14_H_k[c14_i93 + c14_i92];
      c14_i93 += 3;
    }

    c14_i91 += 3;
  }

  c14_b_eml_scalar_eg(chartInstance);
  c14_b_eml_scalar_eg(chartInstance);
  c14_threshold(chartInstance);
  for (c14_i95 = 0; c14_i95 < 3; c14_i95++) {
    c14_i96 = 0;
    for (c14_i97 = 0; c14_i97 < 3; c14_i97++) {
      c14_c_y[c14_i96 + c14_i95] = 0.0;
      c14_i98 = 0;
      for (c14_i99 = 0; c14_i99 < 3; c14_i99++) {
        c14_c_y[c14_i96 + c14_i95] += c14_a[c14_i98 + c14_i95] * c14_b_b[c14_i99
          + c14_i96];
        c14_i98 += 3;
      }

      c14_i96 += 3;
    }
  }

  for (c14_i100 = 0; c14_i100 < 9; c14_i100++) {
    c14_a[c14_i100] = c14_H_k[c14_i100];
  }

  for (c14_i101 = 0; c14_i101 < 9; c14_i101++) {
    c14_b_b[c14_i101] = c14_P_k_[c14_i101];
  }

  c14_b_eml_scalar_eg(chartInstance);
  c14_b_eml_scalar_eg(chartInstance);
  c14_threshold(chartInstance);
  for (c14_i102 = 0; c14_i102 < 3; c14_i102++) {
    c14_i103 = 0;
    for (c14_i104 = 0; c14_i104 < 3; c14_i104++) {
      c14_d_y[c14_i103 + c14_i102] = 0.0;
      c14_i105 = 0;
      for (c14_i106 = 0; c14_i106 < 3; c14_i106++) {
        c14_d_y[c14_i103 + c14_i102] += c14_a[c14_i105 + c14_i102] *
          c14_b_b[c14_i106 + c14_i103];
        c14_i105 += 3;
      }

      c14_i103 += 3;
    }
  }

  c14_i107 = 0;
  for (c14_i108 = 0; c14_i108 < 3; c14_i108++) {
    c14_i109 = 0;
    for (c14_i110 = 0; c14_i110 < 3; c14_i110++) {
      c14_b_b[c14_i110 + c14_i107] = c14_H_k[c14_i109 + c14_i108];
      c14_i109 += 3;
    }

    c14_i107 += 3;
  }

  c14_b_eml_scalar_eg(chartInstance);
  c14_b_eml_scalar_eg(chartInstance);
  c14_threshold(chartInstance);
  for (c14_i111 = 0; c14_i111 < 3; c14_i111++) {
    c14_i112 = 0;
    for (c14_i113 = 0; c14_i113 < 3; c14_i113++) {
      c14_e_y[c14_i112 + c14_i111] = 0.0;
      c14_i114 = 0;
      for (c14_i115 = 0; c14_i115 < 3; c14_i115++) {
        c14_e_y[c14_i112 + c14_i111] += c14_d_y[c14_i114 + c14_i111] *
          c14_b_b[c14_i115 + c14_i112];
        c14_i114 += 3;
      }

      c14_i112 += 3;
    }
  }

  for (c14_i116 = 0; c14_i116 < 9; c14_i116++) {
    c14_f_y[c14_i116] = c14_c_y[c14_i116];
  }

  for (c14_i117 = 0; c14_i117 < 9; c14_i117++) {
    c14_g_y[c14_i117] = c14_e_y[c14_i117] + c14_R_k[c14_i117];
  }

  c14_mrdivide(chartInstance, c14_f_y, c14_g_y, c14_dv4);
  for (c14_i118 = 0; c14_i118 < 9; c14_i118++) {
    c14_K_k[c14_i118] = c14_dv4[c14_i118];
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c14_sfEvent, 19);
  for (c14_i119 = 0; c14_i119 < 9; c14_i119++) {
    c14_a[c14_i119] = c14_K_k[c14_i119];
  }

  for (c14_i120 = 0; c14_i120 < 3; c14_i120++) {
    c14_b[c14_i120] = c14_y_k[c14_i120] - c14_z_k[c14_i120];
  }

  c14_eml_scalar_eg(chartInstance);
  c14_eml_scalar_eg(chartInstance);
  c14_threshold(chartInstance);
  for (c14_i121 = 0; c14_i121 < 3; c14_i121++) {
    c14_y[c14_i121] = 0.0;
    c14_i122 = 0;
    for (c14_i123 = 0; c14_i123 < 3; c14_i123++) {
      c14_y[c14_i121] += c14_a[c14_i122 + c14_i121] * c14_b[c14_i123];
      c14_i122 += 3;
    }
  }

  for (c14_i124 = 0; c14_i124 < 3; c14_i124++) {
    c14_x_k[c14_i124] = c14_x_k_[c14_i124] + c14_y[c14_i124];
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c14_sfEvent, 20);
  for (c14_i125 = 0; c14_i125 < 9; c14_i125++) {
    c14_a[c14_i125] = c14_K_k[c14_i125];
  }

  for (c14_i126 = 0; c14_i126 < 9; c14_i126++) {
    c14_b_b[c14_i126] = c14_H_k[c14_i126];
  }

  c14_b_eml_scalar_eg(chartInstance);
  c14_b_eml_scalar_eg(chartInstance);
  for (c14_i127 = 0; c14_i127 < 9; c14_i127++) {
    c14_b_t[c14_i127] = 0.0;
  }

  for (c14_i128 = 0; c14_i128 < 9; c14_i128++) {
    c14_b_t[c14_i128] = 0.0;
  }

  for (c14_i129 = 0; c14_i129 < 9; c14_i129++) {
    c14_e_y[c14_i129] = c14_b_t[c14_i129];
  }

  for (c14_i130 = 0; c14_i130 < 9; c14_i130++) {
    c14_b_t[c14_i130] = c14_e_y[c14_i130];
  }

  c14_threshold(chartInstance);
  for (c14_i131 = 0; c14_i131 < 9; c14_i131++) {
    c14_e_y[c14_i131] = c14_b_t[c14_i131];
  }

  for (c14_i132 = 0; c14_i132 < 9; c14_i132++) {
    c14_b_t[c14_i132] = c14_e_y[c14_i132];
  }

  for (c14_i133 = 0; c14_i133 < 3; c14_i133++) {
    c14_i134 = 0;
    for (c14_i135 = 0; c14_i135 < 3; c14_i135++) {
      c14_b_t[c14_i134 + c14_i133] = 0.0;
      c14_i136 = 0;
      for (c14_i137 = 0; c14_i137 < 3; c14_i137++) {
        c14_b_t[c14_i134 + c14_i133] += c14_a[c14_i136 + c14_i133] *
          c14_b_b[c14_i137 + c14_i134];
        c14_i136 += 3;
      }

      c14_i134 += 3;
    }
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c14_sfEvent, 21);
  c14_eye(chartInstance, c14_a);
  for (c14_i138 = 0; c14_i138 < 9; c14_i138++) {
    c14_a[c14_i138] -= c14_b_t[c14_i138];
  }

  for (c14_i139 = 0; c14_i139 < 9; c14_i139++) {
    c14_b_b[c14_i139] = c14_P_k_[c14_i139];
  }

  c14_b_eml_scalar_eg(chartInstance);
  c14_b_eml_scalar_eg(chartInstance);
  c14_threshold(chartInstance);
  for (c14_i140 = 0; c14_i140 < 3; c14_i140++) {
    c14_i141 = 0;
    for (c14_i142 = 0; c14_i142 < 3; c14_i142++) {
      c14_c_y[c14_i141 + c14_i140] = 0.0;
      c14_i143 = 0;
      for (c14_i144 = 0; c14_i144 < 3; c14_i144++) {
        c14_c_y[c14_i141 + c14_i140] += c14_a[c14_i143 + c14_i140] *
          c14_b_b[c14_i144 + c14_i141];
        c14_i143 += 3;
      }

      c14_i141 += 3;
    }
  }

  c14_eye(chartInstance, c14_e_y);
  c14_i145 = 0;
  for (c14_i146 = 0; c14_i146 < 3; c14_i146++) {
    c14_i147 = 0;
    for (c14_i148 = 0; c14_i148 < 3; c14_i148++) {
      c14_b_b[c14_i148 + c14_i145] = c14_e_y[c14_i147 + c14_i146] -
        c14_b_t[c14_i147 + c14_i146];
      c14_i147 += 3;
    }

    c14_i145 += 3;
  }

  c14_b_eml_scalar_eg(chartInstance);
  c14_b_eml_scalar_eg(chartInstance);
  c14_threshold(chartInstance);
  for (c14_i149 = 0; c14_i149 < 3; c14_i149++) {
    c14_i150 = 0;
    for (c14_i151 = 0; c14_i151 < 3; c14_i151++) {
      c14_d_y[c14_i150 + c14_i149] = 0.0;
      c14_i152 = 0;
      for (c14_i153 = 0; c14_i153 < 3; c14_i153++) {
        c14_d_y[c14_i150 + c14_i149] += c14_c_y[c14_i152 + c14_i149] *
          c14_b_b[c14_i153 + c14_i150];
        c14_i152 += 3;
      }

      c14_i150 += 3;
    }
  }

  for (c14_i154 = 0; c14_i154 < 9; c14_i154++) {
    c14_a[c14_i154] = c14_K_k[c14_i154];
  }

  for (c14_i155 = 0; c14_i155 < 9; c14_i155++) {
    c14_b_b[c14_i155] = c14_R_k[c14_i155];
  }

  c14_b_eml_scalar_eg(chartInstance);
  c14_b_eml_scalar_eg(chartInstance);
  c14_threshold(chartInstance);
  for (c14_i156 = 0; c14_i156 < 3; c14_i156++) {
    c14_i157 = 0;
    for (c14_i158 = 0; c14_i158 < 3; c14_i158++) {
      c14_c_y[c14_i157 + c14_i156] = 0.0;
      c14_i159 = 0;
      for (c14_i160 = 0; c14_i160 < 3; c14_i160++) {
        c14_c_y[c14_i157 + c14_i156] += c14_a[c14_i159 + c14_i156] *
          c14_b_b[c14_i160 + c14_i157];
        c14_i159 += 3;
      }

      c14_i157 += 3;
    }
  }

  c14_i161 = 0;
  for (c14_i162 = 0; c14_i162 < 3; c14_i162++) {
    c14_i163 = 0;
    for (c14_i164 = 0; c14_i164 < 3; c14_i164++) {
      c14_b_b[c14_i164 + c14_i161] = c14_K_k[c14_i163 + c14_i162];
      c14_i163 += 3;
    }

    c14_i161 += 3;
  }

  c14_b_eml_scalar_eg(chartInstance);
  c14_b_eml_scalar_eg(chartInstance);
  c14_threshold(chartInstance);
  for (c14_i165 = 0; c14_i165 < 3; c14_i165++) {
    c14_i166 = 0;
    for (c14_i167 = 0; c14_i167 < 3; c14_i167++) {
      c14_e_y[c14_i166 + c14_i165] = 0.0;
      c14_i168 = 0;
      for (c14_i169 = 0; c14_i169 < 3; c14_i169++) {
        c14_e_y[c14_i166 + c14_i165] += c14_c_y[c14_i168 + c14_i165] *
          c14_b_b[c14_i169 + c14_i166];
        c14_i168 += 3;
      }

      c14_i166 += 3;
    }
  }

  for (c14_i170 = 0; c14_i170 < 9; c14_i170++) {
    c14_P_k[c14_i170] = c14_d_y[c14_i170] + c14_e_y[c14_i170];
  }

  _SFD_SCRIPT_CALL(1U, chartInstance->c14_sfEvent, -21);
  _SFD_SYMBOL_SCOPE_POP();
}

static void init_script_number_translation(uint32_T c14_machineNumber, uint32_T
  c14_chartNumber, uint32_T c14_instanceNumber)
{
  (void)c14_machineNumber;
  _SFD_SCRIPT_TRANSLATION(c14_chartNumber, c14_instanceNumber, 0U,
    sf_debug_get_script_id(
    "F:\\Users\\Toby Buckley\\Documents\\grad school documents\\aa279c\\aa279c_github\\src\\linearEuler.m"));
  _SFD_SCRIPT_TRANSLATION(c14_chartNumber, c14_instanceNumber, 1U,
    sf_debug_get_script_id(
    "F:\\Users\\Toby Buckley\\Documents\\grad school documents\\aa279c\\aa279c_github\\src\\EKF.m"));
}

static const mxArray *c14_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData)
{
  const mxArray *c14_mxArrayOutData = NULL;
  int32_T c14_i171;
  int32_T c14_i172;
  int32_T c14_i173;
  real_T c14_b_u[9];
  const mxArray *c14_y = NULL;
  SFc14_sixDOF_hw7InstanceStruct *chartInstance;
  chartInstance = (SFc14_sixDOF_hw7InstanceStruct *)chartInstanceVoid;
  c14_mxArrayOutData = NULL;
  c14_i171 = 0;
  for (c14_i172 = 0; c14_i172 < 3; c14_i172++) {
    for (c14_i173 = 0; c14_i173 < 3; c14_i173++) {
      c14_b_u[c14_i173 + c14_i171] = (*(real_T (*)[9])c14_inData)[c14_i173 +
        c14_i171];
    }

    c14_i171 += 3;
  }

  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", c14_b_u, 0, 0U, 1U, 0U, 2, 3, 3),
                false);
  sf_mex_assign(&c14_mxArrayOutData, c14_y, false);
  return c14_mxArrayOutData;
}

static void c14_emlrt_marshallIn(SFc14_sixDOF_hw7InstanceStruct *chartInstance,
  const mxArray *c14_b_PHI, const char_T *c14_identifier, real_T c14_y[9])
{
  emlrtMsgIdentifier c14_thisId;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_b_PHI), &c14_thisId,
    c14_y);
  sf_mex_destroy(&c14_b_PHI);
}

static void c14_b_emlrt_marshallIn(SFc14_sixDOF_hw7InstanceStruct *chartInstance,
  const mxArray *c14_b_u, const emlrtMsgIdentifier *c14_parentId, real_T c14_y[9])
{
  real_T c14_dv5[9];
  int32_T c14_i174;
  (void)chartInstance;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_b_u), c14_dv5, 1, 0, 0U, 1, 0U, 2,
                3, 3);
  for (c14_i174 = 0; c14_i174 < 9; c14_i174++) {
    c14_y[c14_i174] = c14_dv5[c14_i174];
  }

  sf_mex_destroy(&c14_b_u);
}

static void c14_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData)
{
  const mxArray *c14_b_PHI;
  const char_T *c14_identifier;
  emlrtMsgIdentifier c14_thisId;
  real_T c14_y[9];
  int32_T c14_i175;
  int32_T c14_i176;
  int32_T c14_i177;
  SFc14_sixDOF_hw7InstanceStruct *chartInstance;
  chartInstance = (SFc14_sixDOF_hw7InstanceStruct *)chartInstanceVoid;
  c14_b_PHI = sf_mex_dup(c14_mxArrayInData);
  c14_identifier = c14_varName;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_b_PHI), &c14_thisId,
    c14_y);
  sf_mex_destroy(&c14_b_PHI);
  c14_i175 = 0;
  for (c14_i176 = 0; c14_i176 < 3; c14_i176++) {
    for (c14_i177 = 0; c14_i177 < 3; c14_i177++) {
      (*(real_T (*)[9])c14_outData)[c14_i177 + c14_i175] = c14_y[c14_i177 +
        c14_i175];
    }

    c14_i175 += 3;
  }

  sf_mex_destroy(&c14_mxArrayInData);
}

static const mxArray *c14_b_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData)
{
  const mxArray *c14_mxArrayOutData = NULL;
  int32_T c14_i178;
  real_T c14_b_u[3];
  const mxArray *c14_y = NULL;
  SFc14_sixDOF_hw7InstanceStruct *chartInstance;
  chartInstance = (SFc14_sixDOF_hw7InstanceStruct *)chartInstanceVoid;
  c14_mxArrayOutData = NULL;
  for (c14_i178 = 0; c14_i178 < 3; c14_i178++) {
    c14_b_u[c14_i178] = (*(real_T (*)[3])c14_inData)[c14_i178];
  }

  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", c14_b_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c14_mxArrayOutData, c14_y, false);
  return c14_mxArrayOutData;
}

static void c14_c_emlrt_marshallIn(SFc14_sixDOF_hw7InstanceStruct *chartInstance,
  const mxArray *c14_b_z_ekf, const char_T *c14_identifier, real_T c14_y[3])
{
  emlrtMsgIdentifier c14_thisId;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_b_z_ekf), &c14_thisId,
    c14_y);
  sf_mex_destroy(&c14_b_z_ekf);
}

static void c14_d_emlrt_marshallIn(SFc14_sixDOF_hw7InstanceStruct *chartInstance,
  const mxArray *c14_b_u, const emlrtMsgIdentifier *c14_parentId, real_T c14_y[3])
{
  real_T c14_dv6[3];
  int32_T c14_i179;
  (void)chartInstance;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_b_u), c14_dv6, 1, 0, 0U, 1, 0U, 1,
                3);
  for (c14_i179 = 0; c14_i179 < 3; c14_i179++) {
    c14_y[c14_i179] = c14_dv6[c14_i179];
  }

  sf_mex_destroy(&c14_b_u);
}

static void c14_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData)
{
  const mxArray *c14_b_z_ekf;
  const char_T *c14_identifier;
  emlrtMsgIdentifier c14_thisId;
  real_T c14_y[3];
  int32_T c14_i180;
  SFc14_sixDOF_hw7InstanceStruct *chartInstance;
  chartInstance = (SFc14_sixDOF_hw7InstanceStruct *)chartInstanceVoid;
  c14_b_z_ekf = sf_mex_dup(c14_mxArrayInData);
  c14_identifier = c14_varName;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_b_z_ekf), &c14_thisId,
    c14_y);
  sf_mex_destroy(&c14_b_z_ekf);
  for (c14_i180 = 0; c14_i180 < 3; c14_i180++) {
    (*(real_T (*)[3])c14_outData)[c14_i180] = c14_y[c14_i180];
  }

  sf_mex_destroy(&c14_mxArrayInData);
}

static const mxArray *c14_c_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData)
{
  const mxArray *c14_mxArrayOutData = NULL;
  real_T c14_b_u;
  const mxArray *c14_y = NULL;
  SFc14_sixDOF_hw7InstanceStruct *chartInstance;
  chartInstance = (SFc14_sixDOF_hw7InstanceStruct *)chartInstanceVoid;
  c14_mxArrayOutData = NULL;
  c14_b_u = *(real_T *)c14_inData;
  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", &c14_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c14_mxArrayOutData, c14_y, false);
  return c14_mxArrayOutData;
}

static real_T c14_e_emlrt_marshallIn(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance, const mxArray *c14_b_u, const emlrtMsgIdentifier *c14_parentId)
{
  real_T c14_y;
  real_T c14_d0;
  (void)chartInstance;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_b_u), &c14_d0, 1, 0, 0U, 0, 0U, 0);
  c14_y = c14_d0;
  sf_mex_destroy(&c14_b_u);
  return c14_y;
}

static void c14_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData)
{
  const mxArray *c14_nargout;
  const char_T *c14_identifier;
  emlrtMsgIdentifier c14_thisId;
  real_T c14_y;
  SFc14_sixDOF_hw7InstanceStruct *chartInstance;
  chartInstance = (SFc14_sixDOF_hw7InstanceStruct *)chartInstanceVoid;
  c14_nargout = sf_mex_dup(c14_mxArrayInData);
  c14_identifier = c14_varName;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_y = c14_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c14_nargout),
    &c14_thisId);
  sf_mex_destroy(&c14_nargout);
  *(real_T *)c14_outData = c14_y;
  sf_mex_destroy(&c14_mxArrayInData);
}

const mxArray *sf_c14_sixDOF_hw7_get_eml_resolved_functions_info(void)
{
  const mxArray *c14_nameCaptureInfo = NULL;
  c14_nameCaptureInfo = NULL;
  sf_mex_assign(&c14_nameCaptureInfo, sf_mex_createstruct("structure", 2, 2, 1),
                false);
  c14_info_helper(&c14_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c14_nameCaptureInfo);
  return c14_nameCaptureInfo;
}

static void c14_info_helper(const mxArray **c14_info)
{
  const mxArray *c14_rhs0 = NULL;
  const mxArray *c14_lhs0 = NULL;
  const mxArray *c14_rhs1 = NULL;
  const mxArray *c14_lhs1 = NULL;
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("linearEuler"), "name",
                  "name", 0);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[E]F:/Users/Toby Buckley/Documents/grad school documents/aa279c/aa279c_github/src/linearEuler.m"),
                  "resolved", "resolved", 0);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(1496640197U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c14_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c14_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_rhs0), "rhs", "rhs",
                  0);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_lhs0), "lhs", "lhs",
                  0);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(""), "context", "context", 1);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("EKF"), "name", "name", 1);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c14_info, c14_emlrt_marshallOut(
    "[E]F:/Users/Toby Buckley/Documents/grad school documents/aa279c/aa279c_github/src/EKF.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(1496643932U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c14_info, c14_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c14_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c14_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_rhs1), "rhs", "rhs",
                  1);
  sf_mex_addfield(*c14_info, sf_mex_duplicatearraysafe(&c14_lhs1), "lhs", "lhs",
                  1);
  sf_mex_destroy(&c14_rhs0);
  sf_mex_destroy(&c14_lhs0);
  sf_mex_destroy(&c14_rhs1);
  sf_mex_destroy(&c14_lhs1);
}

static const mxArray *c14_emlrt_marshallOut(const char * c14_b_u)
{
  const mxArray *c14_y = NULL;
  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", c14_b_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c14_b_u)), false);
  return c14_y;
}

static const mxArray *c14_b_emlrt_marshallOut(const uint32_T c14_b_u)
{
  const mxArray *c14_y = NULL;
  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", &c14_b_u, 7, 0U, 0U, 0U, 0), false);
  return c14_y;
}

static real_T c14_rdivide(SFc14_sixDOF_hw7InstanceStruct *chartInstance, real_T
  c14_b_x, real_T c14_y)
{
  real_T c14_c_x;
  real_T c14_b_y;
  real_T c14_d_x;
  real_T c14_c_y;
  (void)chartInstance;
  c14_c_x = c14_b_x;
  c14_b_y = c14_y;
  c14_d_x = c14_c_x;
  c14_c_y = c14_b_y;
  return c14_d_x / c14_c_y;
}

static void c14_eml_scalar_eg(SFc14_sixDOF_hw7InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c14_threshold(SFc14_sixDOF_hw7InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c14_b_eml_scalar_eg(SFc14_sixDOF_hw7InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c14_mrdivide(SFc14_sixDOF_hw7InstanceStruct *chartInstance, real_T
  c14_A[9], real_T c14_B[9], real_T c14_y[9])
{
  int32_T c14_i181;
  real_T c14_b_B[9];
  int32_T c14_i182;
  real_T c14_b_A[9];
  for (c14_i181 = 0; c14_i181 < 9; c14_i181++) {
    c14_b_B[c14_i181] = c14_B[c14_i181];
  }

  for (c14_i182 = 0; c14_i182 < 9; c14_i182++) {
    c14_b_A[c14_i182] = c14_A[c14_i182];
  }

  c14_eml_lusolve(chartInstance, c14_b_B, c14_b_A, c14_y);
}

static void c14_eml_lusolve(SFc14_sixDOF_hw7InstanceStruct *chartInstance,
  real_T c14_A[9], real_T c14_B[9], real_T c14_X[9])
{
  int32_T c14_i183;
  real_T c14_b_A[9];
  int32_T c14_r1;
  int32_T c14_r2;
  int32_T c14_r3;
  real_T c14_b_x;
  real_T c14_c_x;
  real_T c14_d_x;
  real_T c14_e_x;
  real_T c14_y;
  real_T c14_f_x;
  real_T c14_g_x;
  real_T c14_b_y;
  real_T c14_maxval;
  real_T c14_h_x;
  real_T c14_i_x;
  real_T c14_j_x;
  real_T c14_k_x;
  real_T c14_c_y;
  real_T c14_l_x;
  real_T c14_m_x;
  real_T c14_d_y;
  real_T c14_a21;
  real_T c14_n_x;
  real_T c14_o_x;
  real_T c14_p_x;
  real_T c14_q_x;
  real_T c14_e_y;
  real_T c14_r_x;
  real_T c14_s_x;
  real_T c14_f_y;
  real_T c14_d;
  real_T c14_t_x;
  real_T c14_u_x;
  real_T c14_v_x;
  real_T c14_w_x;
  real_T c14_g_y;
  real_T c14_x_x;
  real_T c14_y_x;
  real_T c14_h_y;
  real_T c14_b_d;
  real_T c14_ab_x;
  real_T c14_bb_x;
  real_T c14_cb_x;
  real_T c14_db_x;
  real_T c14_i_y;
  real_T c14_eb_x;
  real_T c14_fb_x;
  real_T c14_j_y;
  real_T c14_c_d;
  int32_T c14_rtemp;
  int32_T c14_k;
  int32_T c14_b_k;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  for (c14_i183 = 0; c14_i183 < 9; c14_i183++) {
    c14_b_A[c14_i183] = c14_A[c14_i183];
  }

  c14_r1 = 1;
  c14_r2 = 2;
  c14_r3 = 3;
  c14_b_x = c14_b_A[0];
  c14_c_x = c14_b_x;
  c14_d_x = c14_c_x;
  c14_e_x = c14_d_x;
  c14_y = muDoubleScalarAbs(c14_e_x);
  c14_f_x = 0.0;
  c14_g_x = c14_f_x;
  c14_b_y = muDoubleScalarAbs(c14_g_x);
  c14_maxval = c14_y + c14_b_y;
  c14_h_x = c14_b_A[1];
  c14_i_x = c14_h_x;
  c14_j_x = c14_i_x;
  c14_k_x = c14_j_x;
  c14_c_y = muDoubleScalarAbs(c14_k_x);
  c14_l_x = 0.0;
  c14_m_x = c14_l_x;
  c14_d_y = muDoubleScalarAbs(c14_m_x);
  c14_a21 = c14_c_y + c14_d_y;
  if (c14_a21 > c14_maxval) {
    c14_maxval = c14_a21;
    c14_r1 = 2;
    c14_r2 = 1;
  }

  c14_n_x = c14_b_A[2];
  c14_o_x = c14_n_x;
  c14_p_x = c14_o_x;
  c14_q_x = c14_p_x;
  c14_e_y = muDoubleScalarAbs(c14_q_x);
  c14_r_x = 0.0;
  c14_s_x = c14_r_x;
  c14_f_y = muDoubleScalarAbs(c14_s_x);
  c14_d = c14_e_y + c14_f_y;
  if (c14_d > c14_maxval) {
    c14_r1 = 3;
    c14_r2 = 2;
    c14_r3 = 1;
  }

  c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c14_r2), 1, 3, 1, 0) - 1] = c14_rdivide(chartInstance,
    c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c14_r2), 1, 3, 1, 0) - 1], c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r1), 1, 3, 1, 0) - 1]);
  c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c14_r3), 1, 3, 1, 0) - 1] = c14_rdivide(chartInstance,
    c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c14_r3), 1, 3, 1, 0) - 1], c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r1), 1, 3, 1, 0) - 1]);
  c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c14_r2), 1, 3, 1, 0) + 2] = c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r2), 1, 3, 1, 0) + 2] -
    c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c14_r2), 1, 3, 1, 0) - 1] * c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r1), 1, 3, 1, 0) + 2];
  c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c14_r3), 1, 3, 1, 0) + 2] = c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r3), 1, 3, 1, 0) + 2] -
    c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c14_r3), 1, 3, 1, 0) - 1] * c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r1), 1, 3, 1, 0) + 2];
  c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c14_r2), 1, 3, 1, 0) + 5] = c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r2), 1, 3, 1, 0) + 5] -
    c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c14_r2), 1, 3, 1, 0) - 1] * c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r1), 1, 3, 1, 0) + 5];
  c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c14_r3), 1, 3, 1, 0) + 5] = c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r3), 1, 3, 1, 0) + 5] -
    c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c14_r3), 1, 3, 1, 0) - 1] * c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r1), 1, 3, 1, 0) + 5];
  c14_t_x = c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
    "", (real_T)c14_r3), 1, 3, 1, 0) + 2];
  c14_u_x = c14_t_x;
  c14_v_x = c14_u_x;
  c14_w_x = c14_v_x;
  c14_g_y = muDoubleScalarAbs(c14_w_x);
  c14_x_x = 0.0;
  c14_y_x = c14_x_x;
  c14_h_y = muDoubleScalarAbs(c14_y_x);
  c14_b_d = c14_g_y + c14_h_y;
  c14_ab_x = c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
    ("", (real_T)c14_r2), 1, 3, 1, 0) + 2];
  c14_bb_x = c14_ab_x;
  c14_cb_x = c14_bb_x;
  c14_db_x = c14_cb_x;
  c14_i_y = muDoubleScalarAbs(c14_db_x);
  c14_eb_x = 0.0;
  c14_fb_x = c14_eb_x;
  c14_j_y = muDoubleScalarAbs(c14_fb_x);
  c14_c_d = c14_i_y + c14_j_y;
  if (c14_b_d > c14_c_d) {
    c14_rtemp = c14_r2;
    c14_r2 = c14_r3;
    c14_r3 = c14_rtemp;
  }

  c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c14_r3), 1, 3, 1, 0) + 2] = c14_rdivide(chartInstance,
    c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c14_r3), 1, 3, 1, 0) + 2], c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r2), 1, 3, 1, 0) + 2]);
  c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c14_r3), 1, 3, 1, 0) + 5] = c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r3), 1, 3, 1, 0) + 5] -
    c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
    (real_T)c14_r3), 1, 3, 1, 0) + 2] * c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
    (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r2), 1, 3, 1, 0) + 5];
  guard1 = false;
  guard2 = false;
  if (c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c14_r1), 1, 3, 1, 0) - 1] == 0.0) {
    guard2 = true;
  } else if (c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
              ("", (real_T)c14_r2), 1, 3, 1, 0) + 2] == 0.0) {
    guard2 = true;
  } else {
    if (c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
          (real_T)c14_r3), 1, 3, 1, 0) + 5] == 0.0) {
      guard1 = true;
    }
  }

  if (guard2 == true) {
    guard1 = true;
  }

  if (guard1 == true) {
    c14_eml_warning(chartInstance);
  }

  c14_b_eml_scalar_eg(chartInstance);
  for (c14_k = 1; c14_k < 4; c14_k++) {
    c14_b_k = c14_k;
    c14_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c14_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r1), 1, 3, 2, 0) - 1))
      - 1] = c14_rdivide(chartInstance, c14_B[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_b_k), 1, 3, 1, 0) - 1],
                         c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c14_r1), 1, 3, 1, 0) - 1]);
    c14_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c14_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r2), 1, 3, 2, 0) - 1))
      - 1] = c14_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c14_b_k), 1, 3, 1, 0) + 2] - c14_X
      [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
          c14_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c14_r1), 1, 3, 2, 0) - 1)) - 1] *
      c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c14_r1), 1, 3, 1, 0) + 2];
    c14_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c14_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r3), 1, 3, 2, 0) - 1))
      - 1] = c14_B[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c14_b_k), 1, 3, 1, 0) + 5] - c14_X
      [(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
          c14_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          _SFD_INTEGER_CHECK("", (real_T)c14_r1), 1, 3, 2, 0) - 1)) - 1] *
      c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c14_r1), 1, 3, 1, 0) + 5];
    c14_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c14_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r2), 1, 3, 2, 0) - 1))
      - 1] = c14_rdivide(chartInstance, c14_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_b_k), 1, 3, 1, 0) + 3 *
      (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c14_r2), 1, 3, 2, 0) - 1)) - 1], c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r2), 1, 3, 1, 0) + 2]);
    c14_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c14_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r3), 1, 3, 2, 0) - 1))
      - 1] = c14_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c14_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r3), 1, 3, 2, 0) - 1)) - 1] -
      c14_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
               (real_T)c14_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
               "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r2), 1, 3, 2, 0)
              - 1)) - 1] * c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c14_r2), 1, 3, 1, 0) + 5];
    c14_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c14_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r3), 1, 3, 2, 0) - 1))
      - 1] = c14_rdivide(chartInstance, c14_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_b_k), 1, 3, 1, 0) + 3 *
      (_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", (real_T)
      c14_r3), 1, 3, 2, 0) - 1)) - 1], c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r3), 1, 3, 1, 0) + 5]);
    c14_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c14_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r2), 1, 3, 2, 0) - 1))
      - 1] = c14_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c14_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r2), 1, 3, 2, 0) - 1)) - 1] -
      c14_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
               (real_T)c14_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
               "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r3), 1, 3, 2, 0)
              - 1)) - 1] * c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c14_r3), 1, 3, 1, 0) + 2];
    c14_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c14_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r1), 1, 3, 2, 0) - 1))
      - 1] = c14_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c14_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r1), 1, 3, 2, 0) - 1)) - 1] -
      c14_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
               (real_T)c14_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
               "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r3), 1, 3, 2, 0)
              - 1)) - 1] * c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c14_r3), 1, 3, 1, 0) - 1];
    c14_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c14_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r1), 1, 3, 2, 0) - 1))
      - 1] = c14_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)c14_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
      (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r1), 1, 3, 2, 0) - 1)) - 1] -
      c14_X[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
               (real_T)c14_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK(
               "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_r2), 1, 3, 2, 0)
              - 1)) - 1] * c14_b_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
      _SFD_INTEGER_CHECK("", (real_T)c14_r2), 1, 3, 1, 0) - 1];
  }
}

static void c14_eml_warning(SFc14_sixDOF_hw7InstanceStruct *chartInstance)
{
  const mxArray *c14_y = NULL;
  static char_T c14_varargin_1[27] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 's', 'i', 'n', 'g', 'u', 'l', 'a', 'r', 'M', 'a',
    't', 'r', 'i', 'x' };

  (void)chartInstance;
  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", c14_varargin_1, 10, 0U, 1U, 0U, 2, 1,
    27), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "warning", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c14_y));
}

static void c14_eye(SFc14_sixDOF_hw7InstanceStruct *chartInstance, real_T c14_I
                    [9])
{
  int32_T c14_i184;
  int32_T c14_k;
  int32_T c14_b_k;
  (void)chartInstance;
  for (c14_i184 = 0; c14_i184 < 9; c14_i184++) {
    c14_I[c14_i184] = 0.0;
  }

  for (c14_k = 1; c14_k < 4; c14_k++) {
    c14_b_k = c14_k;
    c14_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
             (real_T)c14_b_k), 1, 3, 1, 0) + 3 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
             (int32_T)_SFD_INTEGER_CHECK("", (real_T)c14_b_k), 1, 3, 2, 0) - 1))
      - 1] = 1.0;
  }
}

static const mxArray *c14_d_sf_marshallOut(void *chartInstanceVoid, void
  *c14_inData)
{
  const mxArray *c14_mxArrayOutData = NULL;
  int32_T c14_b_u;
  const mxArray *c14_y = NULL;
  SFc14_sixDOF_hw7InstanceStruct *chartInstance;
  chartInstance = (SFc14_sixDOF_hw7InstanceStruct *)chartInstanceVoid;
  c14_mxArrayOutData = NULL;
  c14_b_u = *(int32_T *)c14_inData;
  c14_y = NULL;
  sf_mex_assign(&c14_y, sf_mex_create("y", &c14_b_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c14_mxArrayOutData, c14_y, false);
  return c14_mxArrayOutData;
}

static int32_T c14_f_emlrt_marshallIn(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance, const mxArray *c14_b_u, const emlrtMsgIdentifier *c14_parentId)
{
  int32_T c14_y;
  int32_T c14_i185;
  (void)chartInstance;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_b_u), &c14_i185, 1, 6, 0U, 0, 0U, 0);
  c14_y = c14_i185;
  sf_mex_destroy(&c14_b_u);
  return c14_y;
}

static void c14_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c14_mxArrayInData, const char_T *c14_varName, void *c14_outData)
{
  const mxArray *c14_b_sfEvent;
  const char_T *c14_identifier;
  emlrtMsgIdentifier c14_thisId;
  int32_T c14_y;
  SFc14_sixDOF_hw7InstanceStruct *chartInstance;
  chartInstance = (SFc14_sixDOF_hw7InstanceStruct *)chartInstanceVoid;
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

static uint8_T c14_g_emlrt_marshallIn(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance, const mxArray *c14_b_is_active_c14_sixDOF_hw7, const char_T
  *c14_identifier)
{
  uint8_T c14_y;
  emlrtMsgIdentifier c14_thisId;
  c14_thisId.fIdentifier = c14_identifier;
  c14_thisId.fParent = NULL;
  c14_y = c14_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c14_b_is_active_c14_sixDOF_hw7), &c14_thisId);
  sf_mex_destroy(&c14_b_is_active_c14_sixDOF_hw7);
  return c14_y;
}

static uint8_T c14_h_emlrt_marshallIn(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance, const mxArray *c14_b_u, const emlrtMsgIdentifier *c14_parentId)
{
  uint8_T c14_y;
  uint8_T c14_u0;
  (void)chartInstance;
  sf_mex_import(c14_parentId, sf_mex_dup(c14_b_u), &c14_u0, 1, 3, 0U, 0, 0U, 0);
  c14_y = c14_u0;
  sf_mex_destroy(&c14_b_u);
  return c14_y;
}

static void init_dsm_address_info(SFc14_sixDOF_hw7InstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc14_sixDOF_hw7InstanceStruct
  *chartInstance)
{
  chartInstance->c14_x = (real_T (*)[3])ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c14_y_meas = (real_T (*)[3])ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c14_u = (real_T (*)[3])ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c14_x_ekf = (real_T (*)[3])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c14_dt = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c14_I_principle = (real_T (*)[9])ssGetInputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c14_P = (real_T (*)[9])ssGetInputPortSignal_wrapper
    (chartInstance->S, 5);
  chartInstance->c14_Q = (real_T (*)[9])ssGetInputPortSignal_wrapper
    (chartInstance->S, 6);
  chartInstance->c14_R = (real_T (*)[9])ssGetInputPortSignal_wrapper
    (chartInstance->S, 7);
  chartInstance->c14_H = (real_T (*)[9])ssGetInputPortSignal_wrapper
    (chartInstance->S, 8);
  chartInstance->c14_P_ekf = (real_T (*)[9])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c14_z_ekf = (real_T (*)[3])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c14_PHI = (real_T (*)[9])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c14_t = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    9);
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

void sf_c14_sixDOF_hw7_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3738758529U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3604158573U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(393196793U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(929335055U);
}

mxArray* sf_c14_sixDOF_hw7_get_post_codegen_info(void);
mxArray *sf_c14_sixDOF_hw7_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("uMQet2Oetthy0On6xGcrP");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,10,3,dataFields);

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
      pr[0] = (double)(3);
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
      pr[0] = (double)(3);
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
      pr[0] = (double)(3);
      pr[1] = (double)(3);
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
      pr[0] = (double)(3);
      pr[1] = (double)(3);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(3);
      mxSetField(mxData,6,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(3);
      mxSetField(mxData,7,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,7,"type",mxType);
    }

    mxSetField(mxData,7,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(3);
      mxSetField(mxData,8,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,8,"type",mxType);
    }

    mxSetField(mxData,8,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,9,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,9,"type",mxType);
    }

    mxSetField(mxData,9,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

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
      pr[0] = (double)(3);
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
      pr[0] = (double)(3);
      pr[1] = (double)(3);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c14_sixDOF_hw7_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c14_sixDOF_hw7_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c14_sixDOF_hw7_jit_fallback_info(void)
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

mxArray *sf_c14_sixDOF_hw7_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c14_sixDOF_hw7_get_post_codegen_info(void)
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

static const mxArray *sf_get_sim_state_info_c14_sixDOF_hw7(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x5'type','srcId','name','auxInfo'{{M[1],M[17],T\"PHI\",},{M[1],M[14],T\"P_ekf\",},{M[1],M[5],T\"x_ekf\",},{M[1],M[16],T\"z_ekf\",},{M[8],M[0],T\"is_active_c14_sixDOF_hw7\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 5, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c14_sixDOF_hw7_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc14_sixDOF_hw7InstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc14_sixDOF_hw7InstanceStruct *) chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _sixDOF_hw7MachineNumber_,
           14,
           1,
           1,
           0,
           14,
           0,
           0,
           0,
           0,
           2,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation(_sixDOF_hw7MachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_sixDOF_hw7MachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _sixDOF_hw7MachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"x");
          _SFD_SET_DATA_PROPS(1,1,1,0,"y_meas");
          _SFD_SET_DATA_PROPS(2,1,1,0,"u");
          _SFD_SET_DATA_PROPS(3,1,1,0,"dt");
          _SFD_SET_DATA_PROPS(4,1,1,0,"I_principle");
          _SFD_SET_DATA_PROPS(5,1,1,0,"P");
          _SFD_SET_DATA_PROPS(6,1,1,0,"Q");
          _SFD_SET_DATA_PROPS(7,1,1,0,"R");
          _SFD_SET_DATA_PROPS(8,1,1,0,"H");
          _SFD_SET_DATA_PROPS(9,1,1,0,"t");
          _SFD_SET_DATA_PROPS(10,2,0,1,"x_ekf");
          _SFD_SET_DATA_PROPS(11,2,0,1,"P_ekf");
          _SFD_SET_DATA_PROPS(12,2,0,1,"z_ekf");
          _SFD_SET_DATA_PROPS(13,2,0,1,"PHI");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,263);
        _SFD_CV_INIT_SCRIPT(0,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"linearEuler",0,-1,400);
        _SFD_CV_INIT_SCRIPT(1,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(1,0,"EKF",0,-1,743);

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c14_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c14_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c14_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c14_c_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c14_c_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c14_b_sf_marshallOut,(MexInFcnForType)
            c14_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)
            c14_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(12,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c14_b_sf_marshallOut,(MexInFcnForType)
            c14_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 3;
          dimVector[1]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(13,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c14_sf_marshallOut,(MexInFcnForType)
            c14_sf_marshallIn);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _sixDOF_hw7MachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static void chart_debug_initialize_data_addresses(SimStruct *S)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc14_sixDOF_hw7InstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc14_sixDOF_hw7InstanceStruct *) chartInfo->chartInstance;
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, *chartInstance->c14_x);
        _SFD_SET_DATA_VALUE_PTR(1U, *chartInstance->c14_y_meas);
        _SFD_SET_DATA_VALUE_PTR(2U, *chartInstance->c14_u);
        _SFD_SET_DATA_VALUE_PTR(10U, *chartInstance->c14_x_ekf);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c14_dt);
        _SFD_SET_DATA_VALUE_PTR(4U, *chartInstance->c14_I_principle);
        _SFD_SET_DATA_VALUE_PTR(5U, *chartInstance->c14_P);
        _SFD_SET_DATA_VALUE_PTR(6U, *chartInstance->c14_Q);
        _SFD_SET_DATA_VALUE_PTR(7U, *chartInstance->c14_R);
        _SFD_SET_DATA_VALUE_PTR(8U, *chartInstance->c14_H);
        _SFD_SET_DATA_VALUE_PTR(11U, *chartInstance->c14_P_ekf);
        _SFD_SET_DATA_VALUE_PTR(12U, *chartInstance->c14_z_ekf);
        _SFD_SET_DATA_VALUE_PTR(13U, *chartInstance->c14_PHI);
        _SFD_SET_DATA_VALUE_PTR(9U, chartInstance->c14_t);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "sxb0maEWLBJ4mnPCMXq9nhG";
}

static void sf_opaque_initialize_c14_sixDOF_hw7(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc14_sixDOF_hw7InstanceStruct*) chartInstanceVar)
    ->S,0);
  initialize_params_c14_sixDOF_hw7((SFc14_sixDOF_hw7InstanceStruct*)
    chartInstanceVar);
  initialize_c14_sixDOF_hw7((SFc14_sixDOF_hw7InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c14_sixDOF_hw7(void *chartInstanceVar)
{
  enable_c14_sixDOF_hw7((SFc14_sixDOF_hw7InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c14_sixDOF_hw7(void *chartInstanceVar)
{
  disable_c14_sixDOF_hw7((SFc14_sixDOF_hw7InstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c14_sixDOF_hw7(void *chartInstanceVar)
{
  sf_gateway_c14_sixDOF_hw7((SFc14_sixDOF_hw7InstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c14_sixDOF_hw7(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c14_sixDOF_hw7((SFc14_sixDOF_hw7InstanceStruct*)
    chartInfo->chartInstance);         /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c14_sixDOF_hw7(SimStruct* S, const mxArray
  *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c14_sixDOF_hw7((SFc14_sixDOF_hw7InstanceStruct*)
    chartInfo->chartInstance, st);
}

static void sf_opaque_terminate_c14_sixDOF_hw7(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc14_sixDOF_hw7InstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_sixDOF_hw7_optimization_info();
    }

    finalize_c14_sixDOF_hw7((SFc14_sixDOF_hw7InstanceStruct*) chartInstanceVar);
    utFree(chartInstanceVar);
    if (crtInfo != NULL) {
      utFree(crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc14_sixDOF_hw7((SFc14_sixDOF_hw7InstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c14_sixDOF_hw7(SimStruct *S)
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
    initialize_params_c14_sixDOF_hw7((SFc14_sixDOF_hw7InstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c14_sixDOF_hw7(SimStruct *S)
{
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_sixDOF_hw7_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,
      14);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,14,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,14);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 6, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 7, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 8, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 9, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,14,10);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,14,4);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=4; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 10; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,14);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2745933512U));
  ssSetChecksum1(S,(2451698942U));
  ssSetChecksum2(S,(3146613181U));
  ssSetChecksum3(S,(171753717U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c14_sixDOF_hw7(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c14_sixDOF_hw7(SimStruct *S)
{
  SFc14_sixDOF_hw7InstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc14_sixDOF_hw7InstanceStruct *)utMalloc(sizeof
    (SFc14_sixDOF_hw7InstanceStruct));
  memset(chartInstance, 0, sizeof(SFc14_sixDOF_hw7InstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c14_sixDOF_hw7;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c14_sixDOF_hw7;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c14_sixDOF_hw7;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c14_sixDOF_hw7;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c14_sixDOF_hw7;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c14_sixDOF_hw7;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c14_sixDOF_hw7;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c14_sixDOF_hw7;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c14_sixDOF_hw7;
  chartInstance->chartInfo.mdlStart = mdlStart_c14_sixDOF_hw7;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c14_sixDOF_hw7;
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

void c14_sixDOF_hw7_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c14_sixDOF_hw7(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c14_sixDOF_hw7(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c14_sixDOF_hw7(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c14_sixDOF_hw7_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
