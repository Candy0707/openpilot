#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_7405913186051457856);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4102755163050237258);
void car_H_mod_fun(double *state, double *out_7420929097526134639);
void car_f_fun(double *state, double dt, double *out_991333231328627107);
void car_F_fun(double *state, double dt, double *out_3447939676131973289);
void car_h_25(double *state, double *unused, double *out_8499661956547971675);
void car_H_25(double *state, double *unused, double *out_5861009831116841600);
void car_h_24(double *state, double *unused, double *out_6592846861464662922);
void car_H_24(double *state, double *unused, double *out_340022848734319826);
void car_h_30(double *state, double *unused, double *out_4862212428293949325);
void car_H_30(double *state, double *unused, double *out_5731670883973601530);
void car_h_26(double *state, double *unused, double *out_3918702575392286890);
void car_H_26(double *state, double *unused, double *out_2119506512242785376);
void car_h_27(double *state, double *unused, double *out_6910296065717247256);
void car_H_27(double *state, double *unused, double *out_3556907572173176619);
void car_h_29(double *state, double *unused, double *out_139460839366896320);
void car_H_29(double *state, double *unused, double *out_1843544845303625586);
void car_h_28(double *state, double *unused, double *out_5219848230981566296);
void car_H_28(double *state, double *unused, double *out_3238854171765904988);
void car_h_31(double *state, double *unused, double *out_3359254190685284778);
void car_H_31(double *state, double *unused, double *out_5891655792993802028);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}