#ifndef __c2_eulerSIM_momWheel_h__
#define __c2_eulerSIM_momWheel_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc2_eulerSIM_momWheelInstanceStruct
#define typedef_SFc2_eulerSIM_momWheelInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_isStable;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_eulerSIM_momWheel;
  real_T (*c2_w)[3];
  real_T (*c2_w_dot)[3];
  real_T (*c2_M)[3];
  real_T (*c2_I)[9];
  real_T *c2_Ir;
  real_T (*c2_r_hat)[3];
  real_T *c2_wr;
  real_T *c2_wdotr;
  real_T *c2_Mr;
  real_T *c2_wdotr1;
} SFc2_eulerSIM_momWheelInstanceStruct;

#endif                                 /*typedef_SFc2_eulerSIM_momWheelInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c2_eulerSIM_momWheel_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c2_eulerSIM_momWheel_get_check_sum(mxArray *plhs[]);
extern void c2_eulerSIM_momWheel_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
