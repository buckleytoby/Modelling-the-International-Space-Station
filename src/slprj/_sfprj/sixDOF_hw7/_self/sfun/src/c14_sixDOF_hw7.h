#ifndef __c14_sixDOF_hw7_h__
#define __c14_sixDOF_hw7_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc14_sixDOF_hw7InstanceStruct
#define typedef_SFc14_sixDOF_hw7InstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c14_sfEvent;
  boolean_T c14_isStable;
  boolean_T c14_doneDoubleBufferReInit;
  uint8_T c14_is_active_c14_sixDOF_hw7;
  real_T (*c14_x)[3];
  real_T (*c14_y_meas)[3];
  real_T (*c14_u)[3];
  real_T (*c14_x_ekf)[3];
  real_T *c14_dt;
  real_T (*c14_I_principle)[9];
  real_T (*c14_P)[9];
  real_T (*c14_Q)[9];
  real_T (*c14_R)[9];
  real_T (*c14_H)[9];
  real_T (*c14_P_ekf)[9];
  real_T (*c14_z_ekf)[3];
  real_T (*c14_PHI)[9];
  real_T *c14_t;
} SFc14_sixDOF_hw7InstanceStruct;

#endif                                 /*typedef_SFc14_sixDOF_hw7InstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c14_sixDOF_hw7_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c14_sixDOF_hw7_get_check_sum(mxArray *plhs[]);
extern void c14_sixDOF_hw7_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
