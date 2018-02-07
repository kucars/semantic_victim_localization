//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: RSSmodel2.cpp
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 05-Feb-2018 03:23:30
//

// Include Files
#include "rt_nonfinite.h"
#include "RSSmodel2.h"
#include "RSSmodel2_emxutil.h"
#include "randn.h"
#include "abs1.h"
#include <time.h>

// Function Declarations
static double rt_powd_snf(double u0, double u1);
static double rt_remd_snf(double u0, double u1);
static double rt_roundd_snf(double u);

// Function Definitions

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  double d0;
  double d1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d0 = std::abs(u0);
    d1 = std::abs(u1);
    if (rtIsInf(u1)) {
      if (d0 == 1.0) {
        y = rtNaN;
      } else if (d0 > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_remd_snf(double u0, double u1)
{
  double y;
  double b_u1;
  double tr;
  if (!((!rtIsNaN(u0)) && (!rtIsInf(u0)) && ((!rtIsNaN(u1)) && (!rtIsInf(u1)))))
  {
    y = rtNaN;
  } else {
    if (u1 < 0.0) {
      b_u1 = std::ceil(u1);
    } else {
      b_u1 = std::floor(u1);
    }

    if ((u1 != 0.0) && (u1 != b_u1)) {
      tr = u0 / u1;
      if (std::abs(tr - rt_roundd_snf(tr)) <= DBL_EPSILON * std::abs(tr)) {
        y = 0.0;
      } else {
        y = std::fmod(u0, u1);
      }
    } else {
      y = std::fmod(u0, u1);
    }
  }

  return y;
}

//
// Arguments    : double u
// Return Type  : double
//
static double rt_roundd_snf(double u)
{
  double y;
  if (std::abs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = std::floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = std::ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

//
// sigma=18;
// Arguments    : const double xy_circ[2]
//                const double xy_targ[2]
//                double N
//                double lamda
//                double SNR
//                double K
//                double d0
//                double path_loss
//                emxArray_real_T *meas_dist
//                emxArray_real_T *x
//                emxArray_real_T *y
// Return Type  : void
//
void RSSmodel2(const double xy_circ[2], const double xy_targ[2], double N,
               double lamda, double SNR, double K, double d0, double path_loss,
               emxArray_real_T *meas_dist, emxArray_real_T *x, emxArray_real_T
               *y)
{
  double radius;
  emxArray_real_T *exact_dist;
  int b_exact_dist;
  int loop_ub;
  emxArray_real_T *Stheta;
  double sigma;
  double absx;
  signed char n;
  emxArray_real_T *b_x;
  emxArray_real_T *b_y;
  emxArray_real_T *diffy;
  emxArray_real_T *PL;
  emxArray_real_T *r;
  double b_K[2];
  radius = lamda / (4.0 * std::sin(3.1415926535897931 / N));


  emxInit_real_T1(&exact_dist, 2);
  if (rtIsNaN(N)) {
    b_exact_dist = exact_dist->size[0] * exact_dist->size[1];
    exact_dist->size[0] = 1;
    exact_dist->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)exact_dist, b_exact_dist, (int)sizeof
                      (double));
    exact_dist->data[0] = rtNaN;
  } else if (N < 1.0) {
    b_exact_dist = exact_dist->size[0] * exact_dist->size[1];
    exact_dist->size[0] = 1;
    exact_dist->size[1] = 0;
    emxEnsureCapacity((emxArray__common *)exact_dist, b_exact_dist, (int)sizeof
                      (double));
  } else if (rtIsInf(N) && (1.0 == N)) {
    b_exact_dist = exact_dist->size[0] * exact_dist->size[1];
    exact_dist->size[0] = 1;
    exact_dist->size[1] = 1;
    emxEnsureCapacity((emxArray__common *)exact_dist, b_exact_dist, (int)sizeof
                      (double));
    exact_dist->data[0] = rtNaN;
  } else {
    b_exact_dist = exact_dist->size[0] * exact_dist->size[1];
    exact_dist->size[0] = 1;
    exact_dist->size[1] = (int)std::floor(N - 1.0) + 1;
    emxEnsureCapacity((emxArray__common *)exact_dist, b_exact_dist, (int)sizeof
                      (double));
    loop_ub = (int)std::floor(N - 1.0);
    for (b_exact_dist = 0; b_exact_dist <= loop_ub; b_exact_dist++) {
      exact_dist->data[exact_dist->size[0] * b_exact_dist] = 1.0 + (double)
        b_exact_dist;
    }
  }

  emxInit_real_T(&Stheta, 1);
  b_exact_dist = Stheta->size[0];
  Stheta->size[0] = exact_dist->size[1];
  emxEnsureCapacity((emxArray__common *)Stheta, b_exact_dist, (int)sizeof(double));
  loop_ub = exact_dist->size[1];
  for (b_exact_dist = 0; b_exact_dist < loop_ub; b_exact_dist++) {
    Stheta->data[b_exact_dist] = 6.2831853071795862 * (exact_dist->
      data[exact_dist->size[0] * b_exact_dist] - 1.0) / N * 180.0 /
      3.1415926535897931;
  }

  //  convert to degree
  b_exact_dist = x->size[0];
  x->size[0] = Stheta->size[0];
  emxEnsureCapacity((emxArray__common *)x, b_exact_dist, (int)sizeof(double));
  loop_ub = Stheta->size[0];
  for (b_exact_dist = 0; b_exact_dist < loop_ub; b_exact_dist++) {
    x->data[b_exact_dist] = Stheta->data[b_exact_dist];
  }

  for (loop_ub = 0; loop_ub + 1 <= Stheta->size[0]; loop_ub++) {
    if (!((!rtIsInf(x->data[loop_ub])) && (!rtIsNaN(x->data[loop_ub])))) {
      sigma = rtNaN;
    } else {
      sigma = rt_remd_snf(x->data[loop_ub], 360.0);
      absx = std::abs(sigma);
      if (absx > 180.0) {
        if (sigma > 0.0) {
          sigma -= 360.0;
        } else {
          sigma += 360.0;
        }

        absx = std::abs(sigma);
      }

      if (absx <= 45.0) {
        sigma *= 0.017453292519943295;
        n = 0;
      } else if (absx <= 135.0) {
        if (sigma > 0.0) {
          sigma = 0.017453292519943295 * (sigma - 90.0);
          n = 1;
        } else {
          sigma = 0.017453292519943295 * (sigma + 90.0);
          n = -1;
        }
      } else if (sigma > 0.0) {
        sigma = 0.017453292519943295 * (sigma - 180.0);
        n = 2;
      } else {
        sigma = 0.017453292519943295 * (sigma + 180.0);
        n = -2;
      }

      if (n == 0) {
        sigma = std::cos(sigma);
      } else if (n == 1) {
        sigma = -std::sin(sigma);
      } else if (n == -1) {
        sigma = std::sin(sigma);
      } else {
        sigma = -std::cos(sigma);
      }
    }

    x->data[loop_ub] = sigma;
  }

  //  cos the theta in degree
  b_exact_dist = y->size[0];
  y->size[0] = Stheta->size[0];
  emxEnsureCapacity((emxArray__common *)y, b_exact_dist, (int)sizeof(double));
  loop_ub = Stheta->size[0];
  for (b_exact_dist = 0; b_exact_dist < loop_ub; b_exact_dist++) {
    y->data[b_exact_dist] = Stheta->data[b_exact_dist];
  }

  for (loop_ub = 0; loop_ub + 1 <= Stheta->size[0]; loop_ub++) {
    if (!((!rtIsInf(y->data[loop_ub])) && (!rtIsNaN(y->data[loop_ub])))) {
      sigma = rtNaN;
    } else {
      sigma = rt_remd_snf(y->data[loop_ub], 360.0);
      absx = std::abs(sigma);
      if (absx > 180.0) {
        if (sigma > 0.0) {
          sigma -= 360.0;
        } else {
          sigma += 360.0;
        }

        absx = std::abs(sigma);
      }

      if (absx <= 45.0) {
        sigma *= 0.017453292519943295;
        n = 0;
      } else if (absx <= 135.0) {
        if (sigma > 0.0) {
          sigma = 0.017453292519943295 * (sigma - 90.0);
          n = 1;
        } else {
          sigma = 0.017453292519943295 * (sigma + 90.0);
          n = -1;
        }
      } else if (sigma > 0.0) {
        sigma = 0.017453292519943295 * (sigma - 180.0);
        n = 2;
      } else {
        sigma = 0.017453292519943295 * (sigma + 180.0);
        n = -2;
      }

      if (n == 0) {
        sigma = std::sin(sigma);
      } else if (n == 1) {
        sigma = std::cos(sigma);
      } else if (n == -1) {
        sigma = -std::cos(sigma);
      } else {
        sigma = -std::sin(sigma);
      }
    }

    y->data[loop_ub] = sigma;
  }

  b_exact_dist = x->size[0];
  emxEnsureCapacity((emxArray__common *)x, b_exact_dist, (int)sizeof(double));
  loop_ub = x->size[0];
  for (b_exact_dist = 0; b_exact_dist < loop_ub; b_exact_dist++) {
    x->data[b_exact_dist] = radius * x->data[b_exact_dist] + xy_circ[0];
  }

  b_exact_dist = y->size[0];
  emxEnsureCapacity((emxArray__common *)y, b_exact_dist, (int)sizeof(double));
  loop_ub = y->size[0];
  for (b_exact_dist = 0; b_exact_dist < loop_ub; b_exact_dist++) {
    y->data[b_exact_dist] = radius * y->data[b_exact_dist] + xy_circ[1];
  }

  emxInit_real_T(&b_x, 1);
  b_exact_dist = b_x->size[0];
  b_x->size[0] = x->size[0];
  emxEnsureCapacity((emxArray__common *)b_x, b_exact_dist, (int)sizeof(double));
  loop_ub = x->size[0];
  for (b_exact_dist = 0; b_exact_dist < loop_ub; b_exact_dist++) {
    b_x->data[b_exact_dist] = x->data[b_exact_dist] - xy_targ[0];
  }

  emxInit_real_T(&b_y, 1);
  b_abs(b_x, Stheta);
  b_exact_dist = b_y->size[0];
  b_y->size[0] = y->size[0];
  emxEnsureCapacity((emxArray__common *)b_y, b_exact_dist, (int)sizeof(double));
  loop_ub = y->size[0];
  emxFree_real_T(&b_x);
  for (b_exact_dist = 0; b_exact_dist < loop_ub; b_exact_dist++) {
    b_y->data[b_exact_dist] = y->data[b_exact_dist] - xy_targ[1];
  }

  emxInit_real_T(&diffy, 1);
  b_abs(b_y, diffy);
  b_exact_dist = exact_dist->size[0] * exact_dist->size[1];
  exact_dist->size[0] = 1;
  exact_dist->size[1] = (int)N;
  emxEnsureCapacity((emxArray__common *)exact_dist, b_exact_dist, (int)sizeof
                    (double));
  loop_ub = 0;
  emxFree_real_T(&b_y);
  while (loop_ub <= (int)N - 1) {
    exact_dist->data[loop_ub] = std::sqrt(Stheta->data[loop_ub] * Stheta->
      data[loop_ub] + diffy->data[loop_ub] * diffy->data[loop_ub]);
    loop_ub++;
  }

  emxFree_real_T(&diffy);
  b_exact_dist = exact_dist->size[0] * exact_dist->size[1];
  exact_dist->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)exact_dist, b_exact_dist, (int)sizeof
                    (double));
  loop_ub = exact_dist->size[0];
  b_exact_dist = exact_dist->size[1];
  loop_ub *= b_exact_dist;
  for (b_exact_dist = 0; b_exact_dist < loop_ub; b_exact_dist++) {
    exact_dist->data[b_exact_dist] /= d0;
  }

  emxInit_real_T1(&PL, 2);
  b_exact_dist = PL->size[0] * PL->size[1];
  PL->size[0] = 1;
  PL->size[1] = exact_dist->size[1];
  emxEnsureCapacity((emxArray__common *)PL, b_exact_dist, (int)sizeof(double));
  loop_ub = exact_dist->size[0] * exact_dist->size[1];
  for (b_exact_dist = 0; b_exact_dist < loop_ub; b_exact_dist++) {
    PL->data[b_exact_dist] = exact_dist->data[b_exact_dist];
  }

  for (loop_ub = 0; loop_ub + 1 <= exact_dist->size[1]; loop_ub++) {
    PL->data[loop_ub] = std::log10(PL->data[loop_ub]);
  }

  emxFree_real_T(&exact_dist);
  sigma = 10.0 * path_loss;
  b_exact_dist = PL->size[0] * PL->size[1];
  PL->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)PL, b_exact_dist, (int)sizeof(double));
  absx = -20.0 * std::log10(lamda / (12.566370614359172 * d0));
  loop_ub = PL->size[0];
  b_exact_dist = PL->size[1];
  loop_ub *= b_exact_dist;
  for (b_exact_dist = 0; b_exact_dist < loop_ub; b_exact_dist++) {
    PL->data[b_exact_dist] = absx + sigma * PL->data[b_exact_dist];
  }

  emxInit_real_T1(&r, 2);


  sigma = PL->data[0] / SNR;
  b_K[0] = (unsigned int)(int)K;
  b_K[1] = 1.0;
  srand (time(NULL));
  randn(b_K, r);
  b_exact_dist = Stheta->size[0];
  Stheta->size[0] = (int)K;
  emxEnsureCapacity((emxArray__common *)Stheta, b_exact_dist, (int)sizeof(double));
  loop_ub = (int)K;
  for (b_exact_dist = 0; b_exact_dist < loop_ub; b_exact_dist++) {
    Stheta->data[b_exact_dist] = r->data[b_exact_dist] * sigma;
  }

  emxFree_real_T(&r);
  if (sigma < 0.0) {
    loop_ub = Stheta->size[0];
    b_exact_dist = Stheta->size[0];
    Stheta->size[0] = loop_ub;
    emxEnsureCapacity((emxArray__common *)Stheta, b_exact_dist, (int)sizeof
                      (double));
    for (b_exact_dist = 0; b_exact_dist < loop_ub; b_exact_dist++) {
      Stheta->data[b_exact_dist] = rtNaN;
    }
  }

  //  arry of a random number
  if (Stheta->size[0] == 0) {
    sigma = 0.0;
  } else {
    sigma = Stheta->data[0];
    for (loop_ub = 2; loop_ub <= Stheta->size[0]; loop_ub++) {
      sigma += Stheta->data[loop_ub - 1];
    }
  }

  sigma /= (double)Stheta->size[0];
  b_exact_dist = PL->size[0] * PL->size[1];
  PL->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)PL, b_exact_dist, (int)sizeof(double));
  loop_ub = PL->size[0];
  b_exact_dist = PL->size[1];
  loop_ub *= b_exact_dist;
  emxFree_real_T(&Stheta);
  for (b_exact_dist = 0; b_exact_dist < loop_ub; b_exact_dist++) {
    PL->data[b_exact_dist] += sigma;
  }

  b_exact_dist = meas_dist->size[0] * meas_dist->size[1];
  meas_dist->size[0] = 1;
  meas_dist->size[1] = (int)N;
  emxEnsureCapacity((emxArray__common *)meas_dist, b_exact_dist, (int)sizeof
                    (double));
  for (loop_ub = 0; loop_ub < (int)N; loop_ub++) {
    meas_dist->data[loop_ub] = rt_powd_snf(10.0, (PL->data[loop_ub] + 20.0 * std::
      log10(lamda / (12.566370614359172 * d0))) / (10.0 * path_loss));
  }

  emxFree_real_T(&PL);


}

//
// File trailer for RSSmodel2.cpp
//
// [EOF]
//
