/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_RSSmodel2_api.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 05-Feb-2018 03:23:30
 */

#ifndef _CODER_RSSMODEL2_API_H
#define _CODER_RSSMODEL2_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_RSSmodel2_api.h"

/* Type Definitions */
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 /*typedef_emxArray_real_T*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void RSSmodel2(real_T xy_circ[2], real_T xy_targ[2], real_T N, real_T
                      lamda, real_T SNR, real_T K, real_T d0, real_T path_loss,
                      emxArray_real_T *meas_dist, emxArray_real_T *x,
                      emxArray_real_T *y);
extern void RSSmodel2_api(const mxArray *prhs[8], const mxArray *plhs[3]);
extern void RSSmodel2_atexit(void);
extern void RSSmodel2_initialize(void);
extern void RSSmodel2_terminate(void);
extern void RSSmodel2_xil_terminate(void);

#endif

/*
 * File trailer for _coder_RSSmodel2_api.h
 *
 * [EOF]
 */
