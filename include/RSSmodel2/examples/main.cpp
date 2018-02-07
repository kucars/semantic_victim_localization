//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 05-Feb-2018 03:23:30
//

//***********************************************************************
// This automatically generated example C main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************
// Include Files
#include "rt_nonfinite.h"
#include "RSSmodel2.h"
#include "main.h"
#include "RSSmodel2_terminate.h"
#include "RSSmodel2_emxAPI.h"
#include "RSSmodel2_initialize.h"

// Function Declarations
static void argInit_1x2_real_T(double result[2]);
static double argInit_real_T();
static void main_RSSmodel2();

// Function Definitions

//
// Arguments    : double result[2]
// Return Type  : void
//
static void argInit_1x2_real_T(double result[2])
{
  int idx1;

  // Loop over the array to initialize each element.
  for (idx1 = 0; idx1 < 2; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx1] = argInit_real_T();
  }
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_RSSmodel2()
{
  emxArray_real_T *meas_dist;
  emxArray_real_T *x;
  emxArray_real_T *y;
  double dv0[2];
  double dv1[2];
  emxInitArray_real_T(&meas_dist, 2);
  emxInitArray_real_T(&x, 1);
  emxInitArray_real_T(&y, 1);

  // Initialize function 'RSSmodel2' input arguments.
  // Initialize function input argument 'xy_circ'.
  // Initialize function input argument 'xy_targ'.
  // Call the entry-point 'RSSmodel2'.
  argInit_1x2_real_T(dv0);
  argInit_1x2_real_T(dv1);
  RSSmodel2(dv0, dv1, argInit_real_T(), argInit_real_T(), argInit_real_T(),
            argInit_real_T(), argInit_real_T(), argInit_real_T(), meas_dist, x,
            y);
  emxDestroyArray_real_T(y);
  emxDestroyArray_real_T(x);
  emxDestroyArray_real_T(meas_dist);
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
  // Initialize the application.
  // You do not need to do this more than one time.
  RSSmodel2_initialize();

  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_RSSmodel2();

  // Terminate the application.
  // You do not need to do this more than one time.
  RSSmodel2_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
