//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: RSSmodel2_initialize.cpp
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 05-Feb-2018 03:23:30
//

// Include Files
#include "rt_nonfinite.h"
#include "RSSmodel2.h"
#include "RSSmodel2_initialize.h"
#include "eml_rand_mt19937ar_stateful.h"

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void RSSmodel2_initialize()
{
  rt_InitInfAndNaN(8U);
  c_eml_rand_mt19937ar_stateful_i();
}

//
// File trailer for RSSmodel2_initialize.cpp
//
// [EOF]
//
