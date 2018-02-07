//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: RSSmodel2.h
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 05-Feb-2018 03:23:30
//
#ifndef RSSMODEL2_H
#define RSSMODEL2_H

// Include Files
#include <cmath>
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "RSSmodel2_types.h"

// Function Declarations
extern void RSSmodel2(const double xy_circ[2], const double xy_targ[2], double N,
                      double lamda, double SNR, double K, double d0, double
                      path_loss, emxArray_real_T *meas_dist, emxArray_real_T *x,
                      emxArray_real_T *y);

#endif

//
// File trailer for RSSmodel2.h
//
// [EOF]
//
