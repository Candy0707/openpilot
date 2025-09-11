#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7405913186051457856) {
   out_7405913186051457856[0] = delta_x[0] + nom_x[0];
   out_7405913186051457856[1] = delta_x[1] + nom_x[1];
   out_7405913186051457856[2] = delta_x[2] + nom_x[2];
   out_7405913186051457856[3] = delta_x[3] + nom_x[3];
   out_7405913186051457856[4] = delta_x[4] + nom_x[4];
   out_7405913186051457856[5] = delta_x[5] + nom_x[5];
   out_7405913186051457856[6] = delta_x[6] + nom_x[6];
   out_7405913186051457856[7] = delta_x[7] + nom_x[7];
   out_7405913186051457856[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4102755163050237258) {
   out_4102755163050237258[0] = -nom_x[0] + true_x[0];
   out_4102755163050237258[1] = -nom_x[1] + true_x[1];
   out_4102755163050237258[2] = -nom_x[2] + true_x[2];
   out_4102755163050237258[3] = -nom_x[3] + true_x[3];
   out_4102755163050237258[4] = -nom_x[4] + true_x[4];
   out_4102755163050237258[5] = -nom_x[5] + true_x[5];
   out_4102755163050237258[6] = -nom_x[6] + true_x[6];
   out_4102755163050237258[7] = -nom_x[7] + true_x[7];
   out_4102755163050237258[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7420929097526134639) {
   out_7420929097526134639[0] = 1.0;
   out_7420929097526134639[1] = 0.0;
   out_7420929097526134639[2] = 0.0;
   out_7420929097526134639[3] = 0.0;
   out_7420929097526134639[4] = 0.0;
   out_7420929097526134639[5] = 0.0;
   out_7420929097526134639[6] = 0.0;
   out_7420929097526134639[7] = 0.0;
   out_7420929097526134639[8] = 0.0;
   out_7420929097526134639[9] = 0.0;
   out_7420929097526134639[10] = 1.0;
   out_7420929097526134639[11] = 0.0;
   out_7420929097526134639[12] = 0.0;
   out_7420929097526134639[13] = 0.0;
   out_7420929097526134639[14] = 0.0;
   out_7420929097526134639[15] = 0.0;
   out_7420929097526134639[16] = 0.0;
   out_7420929097526134639[17] = 0.0;
   out_7420929097526134639[18] = 0.0;
   out_7420929097526134639[19] = 0.0;
   out_7420929097526134639[20] = 1.0;
   out_7420929097526134639[21] = 0.0;
   out_7420929097526134639[22] = 0.0;
   out_7420929097526134639[23] = 0.0;
   out_7420929097526134639[24] = 0.0;
   out_7420929097526134639[25] = 0.0;
   out_7420929097526134639[26] = 0.0;
   out_7420929097526134639[27] = 0.0;
   out_7420929097526134639[28] = 0.0;
   out_7420929097526134639[29] = 0.0;
   out_7420929097526134639[30] = 1.0;
   out_7420929097526134639[31] = 0.0;
   out_7420929097526134639[32] = 0.0;
   out_7420929097526134639[33] = 0.0;
   out_7420929097526134639[34] = 0.0;
   out_7420929097526134639[35] = 0.0;
   out_7420929097526134639[36] = 0.0;
   out_7420929097526134639[37] = 0.0;
   out_7420929097526134639[38] = 0.0;
   out_7420929097526134639[39] = 0.0;
   out_7420929097526134639[40] = 1.0;
   out_7420929097526134639[41] = 0.0;
   out_7420929097526134639[42] = 0.0;
   out_7420929097526134639[43] = 0.0;
   out_7420929097526134639[44] = 0.0;
   out_7420929097526134639[45] = 0.0;
   out_7420929097526134639[46] = 0.0;
   out_7420929097526134639[47] = 0.0;
   out_7420929097526134639[48] = 0.0;
   out_7420929097526134639[49] = 0.0;
   out_7420929097526134639[50] = 1.0;
   out_7420929097526134639[51] = 0.0;
   out_7420929097526134639[52] = 0.0;
   out_7420929097526134639[53] = 0.0;
   out_7420929097526134639[54] = 0.0;
   out_7420929097526134639[55] = 0.0;
   out_7420929097526134639[56] = 0.0;
   out_7420929097526134639[57] = 0.0;
   out_7420929097526134639[58] = 0.0;
   out_7420929097526134639[59] = 0.0;
   out_7420929097526134639[60] = 1.0;
   out_7420929097526134639[61] = 0.0;
   out_7420929097526134639[62] = 0.0;
   out_7420929097526134639[63] = 0.0;
   out_7420929097526134639[64] = 0.0;
   out_7420929097526134639[65] = 0.0;
   out_7420929097526134639[66] = 0.0;
   out_7420929097526134639[67] = 0.0;
   out_7420929097526134639[68] = 0.0;
   out_7420929097526134639[69] = 0.0;
   out_7420929097526134639[70] = 1.0;
   out_7420929097526134639[71] = 0.0;
   out_7420929097526134639[72] = 0.0;
   out_7420929097526134639[73] = 0.0;
   out_7420929097526134639[74] = 0.0;
   out_7420929097526134639[75] = 0.0;
   out_7420929097526134639[76] = 0.0;
   out_7420929097526134639[77] = 0.0;
   out_7420929097526134639[78] = 0.0;
   out_7420929097526134639[79] = 0.0;
   out_7420929097526134639[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_991333231328627107) {
   out_991333231328627107[0] = state[0];
   out_991333231328627107[1] = state[1];
   out_991333231328627107[2] = state[2];
   out_991333231328627107[3] = state[3];
   out_991333231328627107[4] = state[4];
   out_991333231328627107[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_991333231328627107[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_991333231328627107[7] = state[7];
   out_991333231328627107[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3447939676131973289) {
   out_3447939676131973289[0] = 1;
   out_3447939676131973289[1] = 0;
   out_3447939676131973289[2] = 0;
   out_3447939676131973289[3] = 0;
   out_3447939676131973289[4] = 0;
   out_3447939676131973289[5] = 0;
   out_3447939676131973289[6] = 0;
   out_3447939676131973289[7] = 0;
   out_3447939676131973289[8] = 0;
   out_3447939676131973289[9] = 0;
   out_3447939676131973289[10] = 1;
   out_3447939676131973289[11] = 0;
   out_3447939676131973289[12] = 0;
   out_3447939676131973289[13] = 0;
   out_3447939676131973289[14] = 0;
   out_3447939676131973289[15] = 0;
   out_3447939676131973289[16] = 0;
   out_3447939676131973289[17] = 0;
   out_3447939676131973289[18] = 0;
   out_3447939676131973289[19] = 0;
   out_3447939676131973289[20] = 1;
   out_3447939676131973289[21] = 0;
   out_3447939676131973289[22] = 0;
   out_3447939676131973289[23] = 0;
   out_3447939676131973289[24] = 0;
   out_3447939676131973289[25] = 0;
   out_3447939676131973289[26] = 0;
   out_3447939676131973289[27] = 0;
   out_3447939676131973289[28] = 0;
   out_3447939676131973289[29] = 0;
   out_3447939676131973289[30] = 1;
   out_3447939676131973289[31] = 0;
   out_3447939676131973289[32] = 0;
   out_3447939676131973289[33] = 0;
   out_3447939676131973289[34] = 0;
   out_3447939676131973289[35] = 0;
   out_3447939676131973289[36] = 0;
   out_3447939676131973289[37] = 0;
   out_3447939676131973289[38] = 0;
   out_3447939676131973289[39] = 0;
   out_3447939676131973289[40] = 1;
   out_3447939676131973289[41] = 0;
   out_3447939676131973289[42] = 0;
   out_3447939676131973289[43] = 0;
   out_3447939676131973289[44] = 0;
   out_3447939676131973289[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3447939676131973289[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3447939676131973289[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3447939676131973289[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3447939676131973289[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3447939676131973289[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3447939676131973289[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3447939676131973289[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3447939676131973289[53] = -9.8100000000000005*dt;
   out_3447939676131973289[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3447939676131973289[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3447939676131973289[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3447939676131973289[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3447939676131973289[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3447939676131973289[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3447939676131973289[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3447939676131973289[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3447939676131973289[62] = 0;
   out_3447939676131973289[63] = 0;
   out_3447939676131973289[64] = 0;
   out_3447939676131973289[65] = 0;
   out_3447939676131973289[66] = 0;
   out_3447939676131973289[67] = 0;
   out_3447939676131973289[68] = 0;
   out_3447939676131973289[69] = 0;
   out_3447939676131973289[70] = 1;
   out_3447939676131973289[71] = 0;
   out_3447939676131973289[72] = 0;
   out_3447939676131973289[73] = 0;
   out_3447939676131973289[74] = 0;
   out_3447939676131973289[75] = 0;
   out_3447939676131973289[76] = 0;
   out_3447939676131973289[77] = 0;
   out_3447939676131973289[78] = 0;
   out_3447939676131973289[79] = 0;
   out_3447939676131973289[80] = 1;
}
void h_25(double *state, double *unused, double *out_8499661956547971675) {
   out_8499661956547971675[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5861009831116841600) {
   out_5861009831116841600[0] = 0;
   out_5861009831116841600[1] = 0;
   out_5861009831116841600[2] = 0;
   out_5861009831116841600[3] = 0;
   out_5861009831116841600[4] = 0;
   out_5861009831116841600[5] = 0;
   out_5861009831116841600[6] = 1;
   out_5861009831116841600[7] = 0;
   out_5861009831116841600[8] = 0;
}
void h_24(double *state, double *unused, double *out_6592846861464662922) {
   out_6592846861464662922[0] = state[4];
   out_6592846861464662922[1] = state[5];
}
void H_24(double *state, double *unused, double *out_340022848734319826) {
   out_340022848734319826[0] = 0;
   out_340022848734319826[1] = 0;
   out_340022848734319826[2] = 0;
   out_340022848734319826[3] = 0;
   out_340022848734319826[4] = 1;
   out_340022848734319826[5] = 0;
   out_340022848734319826[6] = 0;
   out_340022848734319826[7] = 0;
   out_340022848734319826[8] = 0;
   out_340022848734319826[9] = 0;
   out_340022848734319826[10] = 0;
   out_340022848734319826[11] = 0;
   out_340022848734319826[12] = 0;
   out_340022848734319826[13] = 0;
   out_340022848734319826[14] = 1;
   out_340022848734319826[15] = 0;
   out_340022848734319826[16] = 0;
   out_340022848734319826[17] = 0;
}
void h_30(double *state, double *unused, double *out_4862212428293949325) {
   out_4862212428293949325[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5731670883973601530) {
   out_5731670883973601530[0] = 0;
   out_5731670883973601530[1] = 0;
   out_5731670883973601530[2] = 0;
   out_5731670883973601530[3] = 0;
   out_5731670883973601530[4] = 1;
   out_5731670883973601530[5] = 0;
   out_5731670883973601530[6] = 0;
   out_5731670883973601530[7] = 0;
   out_5731670883973601530[8] = 0;
}
void h_26(double *state, double *unused, double *out_3918702575392286890) {
   out_3918702575392286890[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2119506512242785376) {
   out_2119506512242785376[0] = 0;
   out_2119506512242785376[1] = 0;
   out_2119506512242785376[2] = 0;
   out_2119506512242785376[3] = 0;
   out_2119506512242785376[4] = 0;
   out_2119506512242785376[5] = 0;
   out_2119506512242785376[6] = 0;
   out_2119506512242785376[7] = 1;
   out_2119506512242785376[8] = 0;
}
void h_27(double *state, double *unused, double *out_6910296065717247256) {
   out_6910296065717247256[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3556907572173176619) {
   out_3556907572173176619[0] = 0;
   out_3556907572173176619[1] = 0;
   out_3556907572173176619[2] = 0;
   out_3556907572173176619[3] = 1;
   out_3556907572173176619[4] = 0;
   out_3556907572173176619[5] = 0;
   out_3556907572173176619[6] = 0;
   out_3556907572173176619[7] = 0;
   out_3556907572173176619[8] = 0;
}
void h_29(double *state, double *unused, double *out_139460839366896320) {
   out_139460839366896320[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1843544845303625586) {
   out_1843544845303625586[0] = 0;
   out_1843544845303625586[1] = 1;
   out_1843544845303625586[2] = 0;
   out_1843544845303625586[3] = 0;
   out_1843544845303625586[4] = 0;
   out_1843544845303625586[5] = 0;
   out_1843544845303625586[6] = 0;
   out_1843544845303625586[7] = 0;
   out_1843544845303625586[8] = 0;
}
void h_28(double *state, double *unused, double *out_5219848230981566296) {
   out_5219848230981566296[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3238854171765904988) {
   out_3238854171765904988[0] = 1;
   out_3238854171765904988[1] = 0;
   out_3238854171765904988[2] = 0;
   out_3238854171765904988[3] = 0;
   out_3238854171765904988[4] = 0;
   out_3238854171765904988[5] = 0;
   out_3238854171765904988[6] = 0;
   out_3238854171765904988[7] = 0;
   out_3238854171765904988[8] = 0;
}
void h_31(double *state, double *unused, double *out_3359254190685284778) {
   out_3359254190685284778[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5891655792993802028) {
   out_5891655792993802028[0] = 0;
   out_5891655792993802028[1] = 0;
   out_5891655792993802028[2] = 0;
   out_5891655792993802028[3] = 0;
   out_5891655792993802028[4] = 0;
   out_5891655792993802028[5] = 0;
   out_5891655792993802028[6] = 0;
   out_5891655792993802028[7] = 0;
   out_5891655792993802028[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_7405913186051457856) {
  err_fun(nom_x, delta_x, out_7405913186051457856);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4102755163050237258) {
  inv_err_fun(nom_x, true_x, out_4102755163050237258);
}
void car_H_mod_fun(double *state, double *out_7420929097526134639) {
  H_mod_fun(state, out_7420929097526134639);
}
void car_f_fun(double *state, double dt, double *out_991333231328627107) {
  f_fun(state,  dt, out_991333231328627107);
}
void car_F_fun(double *state, double dt, double *out_3447939676131973289) {
  F_fun(state,  dt, out_3447939676131973289);
}
void car_h_25(double *state, double *unused, double *out_8499661956547971675) {
  h_25(state, unused, out_8499661956547971675);
}
void car_H_25(double *state, double *unused, double *out_5861009831116841600) {
  H_25(state, unused, out_5861009831116841600);
}
void car_h_24(double *state, double *unused, double *out_6592846861464662922) {
  h_24(state, unused, out_6592846861464662922);
}
void car_H_24(double *state, double *unused, double *out_340022848734319826) {
  H_24(state, unused, out_340022848734319826);
}
void car_h_30(double *state, double *unused, double *out_4862212428293949325) {
  h_30(state, unused, out_4862212428293949325);
}
void car_H_30(double *state, double *unused, double *out_5731670883973601530) {
  H_30(state, unused, out_5731670883973601530);
}
void car_h_26(double *state, double *unused, double *out_3918702575392286890) {
  h_26(state, unused, out_3918702575392286890);
}
void car_H_26(double *state, double *unused, double *out_2119506512242785376) {
  H_26(state, unused, out_2119506512242785376);
}
void car_h_27(double *state, double *unused, double *out_6910296065717247256) {
  h_27(state, unused, out_6910296065717247256);
}
void car_H_27(double *state, double *unused, double *out_3556907572173176619) {
  H_27(state, unused, out_3556907572173176619);
}
void car_h_29(double *state, double *unused, double *out_139460839366896320) {
  h_29(state, unused, out_139460839366896320);
}
void car_H_29(double *state, double *unused, double *out_1843544845303625586) {
  H_29(state, unused, out_1843544845303625586);
}
void car_h_28(double *state, double *unused, double *out_5219848230981566296) {
  h_28(state, unused, out_5219848230981566296);
}
void car_H_28(double *state, double *unused, double *out_3238854171765904988) {
  H_28(state, unused, out_3238854171765904988);
}
void car_h_31(double *state, double *unused, double *out_3359254190685284778) {
  h_31(state, unused, out_3359254190685284778);
}
void car_H_31(double *state, double *unused, double *out_5891655792993802028) {
  H_31(state, unused, out_5891655792993802028);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
