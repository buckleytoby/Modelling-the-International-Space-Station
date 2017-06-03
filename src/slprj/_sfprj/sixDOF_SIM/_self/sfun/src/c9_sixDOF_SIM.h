#ifndef __c9_sixDOF_SIM_h__
#define __c9_sixDOF_SIM_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc9_sixDOF_SIMInstanceStruct
#define typedef_SFc9_sixDOF_SIMInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c9_sfEvent;
  boolean_T c9_isStable;
  boolean_T c9_doneDoubleBufferReInit;
  uint8_T c9_is_active_c9_sixDOF_SIM;
  real_T *c9_mu;
  real_T (*c9_M)[3];
  real_T (*c9_I)[9];
  real_T *c9_R;
  real_T *c9_cx;
  real_T *c9_cy;
  real_T *c9_cz;
} SFc9_sixDOF_SIMInstanceStruct;

#endif                                 /*typedef_SFc9_sixDOF_SIMInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c9_sixDOF_SIM_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c9_sixDOF_SIM_get_check_sum(mxArray *plhs[]);
extern void c9_sixDOF_SIM_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
