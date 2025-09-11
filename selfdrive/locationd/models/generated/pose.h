#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_4337369684268981257);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5765766890507513398);
void pose_H_mod_fun(double *state, double *out_4143829145148051441);
void pose_f_fun(double *state, double dt, double *out_2866496863871759496);
void pose_F_fun(double *state, double dt, double *out_2805007371861002435);
void pose_h_4(double *state, double *unused, double *out_4149227121328903650);
void pose_H_4(double *state, double *unused, double *out_7915636354991959013);
void pose_h_10(double *state, double *unused, double *out_3295896223844812439);
void pose_H_10(double *state, double *unused, double *out_8598042528087430041);
void pose_h_13(double *state, double *unused, double *out_2654614890869165391);
void pose_H_13(double *state, double *unused, double *out_7318833893385259802);
void pose_h_14(double *state, double *unused, double *out_4682865712639379166);
void pose_H_14(double *state, double *unused, double *out_4832847922696586717);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}