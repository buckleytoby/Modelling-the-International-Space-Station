#ifndef __c10_sixDOF_SIM_h__
#define __c10_sixDOF_SIM_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc10_sixDOF_SIMInstanceStruct
#define typedef_SFc10_sixDOF_SIMInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c10_sfEvent;
  boolean_T c10_isStable;
  boolean_T c10_doneDoubleBufferReInit;
  uint8_T c10_is_active_c10_sixDOF_SIM;
  real_T (*c10_c)[3];
  real_T *c10_cx;
  real_T (*c10_state)[6];
  real_T (*c10_A_DCM)[9];
  real_T *c10_cy;
  real_T *c10_cz;
  real_T *c10_R;
} SFc10_sixDOF_SIMInstanceStruct;

#endif                                 /*typedef_SFc10_sixDOF_SIMInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c10_sixDOF_SIM_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c10_sixDOF_SIM_get_check_sum(mxArray *plhs[]);
extern void c10_sixDOF_SIM_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
