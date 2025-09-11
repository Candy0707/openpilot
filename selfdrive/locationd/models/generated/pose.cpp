#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4337369684268981257) {
   out_4337369684268981257[0] = delta_x[0] + nom_x[0];
   out_4337369684268981257[1] = delta_x[1] + nom_x[1];
   out_4337369684268981257[2] = delta_x[2] + nom_x[2];
   out_4337369684268981257[3] = delta_x[3] + nom_x[3];
   out_4337369684268981257[4] = delta_x[4] + nom_x[4];
   out_4337369684268981257[5] = delta_x[5] + nom_x[5];
   out_4337369684268981257[6] = delta_x[6] + nom_x[6];
   out_4337369684268981257[7] = delta_x[7] + nom_x[7];
   out_4337369684268981257[8] = delta_x[8] + nom_x[8];
   out_4337369684268981257[9] = delta_x[9] + nom_x[9];
   out_4337369684268981257[10] = delta_x[10] + nom_x[10];
   out_4337369684268981257[11] = delta_x[11] + nom_x[11];
   out_4337369684268981257[12] = delta_x[12] + nom_x[12];
   out_4337369684268981257[13] = delta_x[13] + nom_x[13];
   out_4337369684268981257[14] = delta_x[14] + nom_x[14];
   out_4337369684268981257[15] = delta_x[15] + nom_x[15];
   out_4337369684268981257[16] = delta_x[16] + nom_x[16];
   out_4337369684268981257[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5765766890507513398) {
   out_5765766890507513398[0] = -nom_x[0] + true_x[0];
   out_5765766890507513398[1] = -nom_x[1] + true_x[1];
   out_5765766890507513398[2] = -nom_x[2] + true_x[2];
   out_5765766890507513398[3] = -nom_x[3] + true_x[3];
   out_5765766890507513398[4] = -nom_x[4] + true_x[4];
   out_5765766890507513398[5] = -nom_x[5] + true_x[5];
   out_5765766890507513398[6] = -nom_x[6] + true_x[6];
   out_5765766890507513398[7] = -nom_x[7] + true_x[7];
   out_5765766890507513398[8] = -nom_x[8] + true_x[8];
   out_5765766890507513398[9] = -nom_x[9] + true_x[9];
   out_5765766890507513398[10] = -nom_x[10] + true_x[10];
   out_5765766890507513398[11] = -nom_x[11] + true_x[11];
   out_5765766890507513398[12] = -nom_x[12] + true_x[12];
   out_5765766890507513398[13] = -nom_x[13] + true_x[13];
   out_5765766890507513398[14] = -nom_x[14] + true_x[14];
   out_5765766890507513398[15] = -nom_x[15] + true_x[15];
   out_5765766890507513398[16] = -nom_x[16] + true_x[16];
   out_5765766890507513398[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4143829145148051441) {
   out_4143829145148051441[0] = 1.0;
   out_4143829145148051441[1] = 0.0;
   out_4143829145148051441[2] = 0.0;
   out_4143829145148051441[3] = 0.0;
   out_4143829145148051441[4] = 0.0;
   out_4143829145148051441[5] = 0.0;
   out_4143829145148051441[6] = 0.0;
   out_4143829145148051441[7] = 0.0;
   out_4143829145148051441[8] = 0.0;
   out_4143829145148051441[9] = 0.0;
   out_4143829145148051441[10] = 0.0;
   out_4143829145148051441[11] = 0.0;
   out_4143829145148051441[12] = 0.0;
   out_4143829145148051441[13] = 0.0;
   out_4143829145148051441[14] = 0.0;
   out_4143829145148051441[15] = 0.0;
   out_4143829145148051441[16] = 0.0;
   out_4143829145148051441[17] = 0.0;
   out_4143829145148051441[18] = 0.0;
   out_4143829145148051441[19] = 1.0;
   out_4143829145148051441[20] = 0.0;
   out_4143829145148051441[21] = 0.0;
   out_4143829145148051441[22] = 0.0;
   out_4143829145148051441[23] = 0.0;
   out_4143829145148051441[24] = 0.0;
   out_4143829145148051441[25] = 0.0;
   out_4143829145148051441[26] = 0.0;
   out_4143829145148051441[27] = 0.0;
   out_4143829145148051441[28] = 0.0;
   out_4143829145148051441[29] = 0.0;
   out_4143829145148051441[30] = 0.0;
   out_4143829145148051441[31] = 0.0;
   out_4143829145148051441[32] = 0.0;
   out_4143829145148051441[33] = 0.0;
   out_4143829145148051441[34] = 0.0;
   out_4143829145148051441[35] = 0.0;
   out_4143829145148051441[36] = 0.0;
   out_4143829145148051441[37] = 0.0;
   out_4143829145148051441[38] = 1.0;
   out_4143829145148051441[39] = 0.0;
   out_4143829145148051441[40] = 0.0;
   out_4143829145148051441[41] = 0.0;
   out_4143829145148051441[42] = 0.0;
   out_4143829145148051441[43] = 0.0;
   out_4143829145148051441[44] = 0.0;
   out_4143829145148051441[45] = 0.0;
   out_4143829145148051441[46] = 0.0;
   out_4143829145148051441[47] = 0.0;
   out_4143829145148051441[48] = 0.0;
   out_4143829145148051441[49] = 0.0;
   out_4143829145148051441[50] = 0.0;
   out_4143829145148051441[51] = 0.0;
   out_4143829145148051441[52] = 0.0;
   out_4143829145148051441[53] = 0.0;
   out_4143829145148051441[54] = 0.0;
   out_4143829145148051441[55] = 0.0;
   out_4143829145148051441[56] = 0.0;
   out_4143829145148051441[57] = 1.0;
   out_4143829145148051441[58] = 0.0;
   out_4143829145148051441[59] = 0.0;
   out_4143829145148051441[60] = 0.0;
   out_4143829145148051441[61] = 0.0;
   out_4143829145148051441[62] = 0.0;
   out_4143829145148051441[63] = 0.0;
   out_4143829145148051441[64] = 0.0;
   out_4143829145148051441[65] = 0.0;
   out_4143829145148051441[66] = 0.0;
   out_4143829145148051441[67] = 0.0;
   out_4143829145148051441[68] = 0.0;
   out_4143829145148051441[69] = 0.0;
   out_4143829145148051441[70] = 0.0;
   out_4143829145148051441[71] = 0.0;
   out_4143829145148051441[72] = 0.0;
   out_4143829145148051441[73] = 0.0;
   out_4143829145148051441[74] = 0.0;
   out_4143829145148051441[75] = 0.0;
   out_4143829145148051441[76] = 1.0;
   out_4143829145148051441[77] = 0.0;
   out_4143829145148051441[78] = 0.0;
   out_4143829145148051441[79] = 0.0;
   out_4143829145148051441[80] = 0.0;
   out_4143829145148051441[81] = 0.0;
   out_4143829145148051441[82] = 0.0;
   out_4143829145148051441[83] = 0.0;
   out_4143829145148051441[84] = 0.0;
   out_4143829145148051441[85] = 0.0;
   out_4143829145148051441[86] = 0.0;
   out_4143829145148051441[87] = 0.0;
   out_4143829145148051441[88] = 0.0;
   out_4143829145148051441[89] = 0.0;
   out_4143829145148051441[90] = 0.0;
   out_4143829145148051441[91] = 0.0;
   out_4143829145148051441[92] = 0.0;
   out_4143829145148051441[93] = 0.0;
   out_4143829145148051441[94] = 0.0;
   out_4143829145148051441[95] = 1.0;
   out_4143829145148051441[96] = 0.0;
   out_4143829145148051441[97] = 0.0;
   out_4143829145148051441[98] = 0.0;
   out_4143829145148051441[99] = 0.0;
   out_4143829145148051441[100] = 0.0;
   out_4143829145148051441[101] = 0.0;
   out_4143829145148051441[102] = 0.0;
   out_4143829145148051441[103] = 0.0;
   out_4143829145148051441[104] = 0.0;
   out_4143829145148051441[105] = 0.0;
   out_4143829145148051441[106] = 0.0;
   out_4143829145148051441[107] = 0.0;
   out_4143829145148051441[108] = 0.0;
   out_4143829145148051441[109] = 0.0;
   out_4143829145148051441[110] = 0.0;
   out_4143829145148051441[111] = 0.0;
   out_4143829145148051441[112] = 0.0;
   out_4143829145148051441[113] = 0.0;
   out_4143829145148051441[114] = 1.0;
   out_4143829145148051441[115] = 0.0;
   out_4143829145148051441[116] = 0.0;
   out_4143829145148051441[117] = 0.0;
   out_4143829145148051441[118] = 0.0;
   out_4143829145148051441[119] = 0.0;
   out_4143829145148051441[120] = 0.0;
   out_4143829145148051441[121] = 0.0;
   out_4143829145148051441[122] = 0.0;
   out_4143829145148051441[123] = 0.0;
   out_4143829145148051441[124] = 0.0;
   out_4143829145148051441[125] = 0.0;
   out_4143829145148051441[126] = 0.0;
   out_4143829145148051441[127] = 0.0;
   out_4143829145148051441[128] = 0.0;
   out_4143829145148051441[129] = 0.0;
   out_4143829145148051441[130] = 0.0;
   out_4143829145148051441[131] = 0.0;
   out_4143829145148051441[132] = 0.0;
   out_4143829145148051441[133] = 1.0;
   out_4143829145148051441[134] = 0.0;
   out_4143829145148051441[135] = 0.0;
   out_4143829145148051441[136] = 0.0;
   out_4143829145148051441[137] = 0.0;
   out_4143829145148051441[138] = 0.0;
   out_4143829145148051441[139] = 0.0;
   out_4143829145148051441[140] = 0.0;
   out_4143829145148051441[141] = 0.0;
   out_4143829145148051441[142] = 0.0;
   out_4143829145148051441[143] = 0.0;
   out_4143829145148051441[144] = 0.0;
   out_4143829145148051441[145] = 0.0;
   out_4143829145148051441[146] = 0.0;
   out_4143829145148051441[147] = 0.0;
   out_4143829145148051441[148] = 0.0;
   out_4143829145148051441[149] = 0.0;
   out_4143829145148051441[150] = 0.0;
   out_4143829145148051441[151] = 0.0;
   out_4143829145148051441[152] = 1.0;
   out_4143829145148051441[153] = 0.0;
   out_4143829145148051441[154] = 0.0;
   out_4143829145148051441[155] = 0.0;
   out_4143829145148051441[156] = 0.0;
   out_4143829145148051441[157] = 0.0;
   out_4143829145148051441[158] = 0.0;
   out_4143829145148051441[159] = 0.0;
   out_4143829145148051441[160] = 0.0;
   out_4143829145148051441[161] = 0.0;
   out_4143829145148051441[162] = 0.0;
   out_4143829145148051441[163] = 0.0;
   out_4143829145148051441[164] = 0.0;
   out_4143829145148051441[165] = 0.0;
   out_4143829145148051441[166] = 0.0;
   out_4143829145148051441[167] = 0.0;
   out_4143829145148051441[168] = 0.0;
   out_4143829145148051441[169] = 0.0;
   out_4143829145148051441[170] = 0.0;
   out_4143829145148051441[171] = 1.0;
   out_4143829145148051441[172] = 0.0;
   out_4143829145148051441[173] = 0.0;
   out_4143829145148051441[174] = 0.0;
   out_4143829145148051441[175] = 0.0;
   out_4143829145148051441[176] = 0.0;
   out_4143829145148051441[177] = 0.0;
   out_4143829145148051441[178] = 0.0;
   out_4143829145148051441[179] = 0.0;
   out_4143829145148051441[180] = 0.0;
   out_4143829145148051441[181] = 0.0;
   out_4143829145148051441[182] = 0.0;
   out_4143829145148051441[183] = 0.0;
   out_4143829145148051441[184] = 0.0;
   out_4143829145148051441[185] = 0.0;
   out_4143829145148051441[186] = 0.0;
   out_4143829145148051441[187] = 0.0;
   out_4143829145148051441[188] = 0.0;
   out_4143829145148051441[189] = 0.0;
   out_4143829145148051441[190] = 1.0;
   out_4143829145148051441[191] = 0.0;
   out_4143829145148051441[192] = 0.0;
   out_4143829145148051441[193] = 0.0;
   out_4143829145148051441[194] = 0.0;
   out_4143829145148051441[195] = 0.0;
   out_4143829145148051441[196] = 0.0;
   out_4143829145148051441[197] = 0.0;
   out_4143829145148051441[198] = 0.0;
   out_4143829145148051441[199] = 0.0;
   out_4143829145148051441[200] = 0.0;
   out_4143829145148051441[201] = 0.0;
   out_4143829145148051441[202] = 0.0;
   out_4143829145148051441[203] = 0.0;
   out_4143829145148051441[204] = 0.0;
   out_4143829145148051441[205] = 0.0;
   out_4143829145148051441[206] = 0.0;
   out_4143829145148051441[207] = 0.0;
   out_4143829145148051441[208] = 0.0;
   out_4143829145148051441[209] = 1.0;
   out_4143829145148051441[210] = 0.0;
   out_4143829145148051441[211] = 0.0;
   out_4143829145148051441[212] = 0.0;
   out_4143829145148051441[213] = 0.0;
   out_4143829145148051441[214] = 0.0;
   out_4143829145148051441[215] = 0.0;
   out_4143829145148051441[216] = 0.0;
   out_4143829145148051441[217] = 0.0;
   out_4143829145148051441[218] = 0.0;
   out_4143829145148051441[219] = 0.0;
   out_4143829145148051441[220] = 0.0;
   out_4143829145148051441[221] = 0.0;
   out_4143829145148051441[222] = 0.0;
   out_4143829145148051441[223] = 0.0;
   out_4143829145148051441[224] = 0.0;
   out_4143829145148051441[225] = 0.0;
   out_4143829145148051441[226] = 0.0;
   out_4143829145148051441[227] = 0.0;
   out_4143829145148051441[228] = 1.0;
   out_4143829145148051441[229] = 0.0;
   out_4143829145148051441[230] = 0.0;
   out_4143829145148051441[231] = 0.0;
   out_4143829145148051441[232] = 0.0;
   out_4143829145148051441[233] = 0.0;
   out_4143829145148051441[234] = 0.0;
   out_4143829145148051441[235] = 0.0;
   out_4143829145148051441[236] = 0.0;
   out_4143829145148051441[237] = 0.0;
   out_4143829145148051441[238] = 0.0;
   out_4143829145148051441[239] = 0.0;
   out_4143829145148051441[240] = 0.0;
   out_4143829145148051441[241] = 0.0;
   out_4143829145148051441[242] = 0.0;
   out_4143829145148051441[243] = 0.0;
   out_4143829145148051441[244] = 0.0;
   out_4143829145148051441[245] = 0.0;
   out_4143829145148051441[246] = 0.0;
   out_4143829145148051441[247] = 1.0;
   out_4143829145148051441[248] = 0.0;
   out_4143829145148051441[249] = 0.0;
   out_4143829145148051441[250] = 0.0;
   out_4143829145148051441[251] = 0.0;
   out_4143829145148051441[252] = 0.0;
   out_4143829145148051441[253] = 0.0;
   out_4143829145148051441[254] = 0.0;
   out_4143829145148051441[255] = 0.0;
   out_4143829145148051441[256] = 0.0;
   out_4143829145148051441[257] = 0.0;
   out_4143829145148051441[258] = 0.0;
   out_4143829145148051441[259] = 0.0;
   out_4143829145148051441[260] = 0.0;
   out_4143829145148051441[261] = 0.0;
   out_4143829145148051441[262] = 0.0;
   out_4143829145148051441[263] = 0.0;
   out_4143829145148051441[264] = 0.0;
   out_4143829145148051441[265] = 0.0;
   out_4143829145148051441[266] = 1.0;
   out_4143829145148051441[267] = 0.0;
   out_4143829145148051441[268] = 0.0;
   out_4143829145148051441[269] = 0.0;
   out_4143829145148051441[270] = 0.0;
   out_4143829145148051441[271] = 0.0;
   out_4143829145148051441[272] = 0.0;
   out_4143829145148051441[273] = 0.0;
   out_4143829145148051441[274] = 0.0;
   out_4143829145148051441[275] = 0.0;
   out_4143829145148051441[276] = 0.0;
   out_4143829145148051441[277] = 0.0;
   out_4143829145148051441[278] = 0.0;
   out_4143829145148051441[279] = 0.0;
   out_4143829145148051441[280] = 0.0;
   out_4143829145148051441[281] = 0.0;
   out_4143829145148051441[282] = 0.0;
   out_4143829145148051441[283] = 0.0;
   out_4143829145148051441[284] = 0.0;
   out_4143829145148051441[285] = 1.0;
   out_4143829145148051441[286] = 0.0;
   out_4143829145148051441[287] = 0.0;
   out_4143829145148051441[288] = 0.0;
   out_4143829145148051441[289] = 0.0;
   out_4143829145148051441[290] = 0.0;
   out_4143829145148051441[291] = 0.0;
   out_4143829145148051441[292] = 0.0;
   out_4143829145148051441[293] = 0.0;
   out_4143829145148051441[294] = 0.0;
   out_4143829145148051441[295] = 0.0;
   out_4143829145148051441[296] = 0.0;
   out_4143829145148051441[297] = 0.0;
   out_4143829145148051441[298] = 0.0;
   out_4143829145148051441[299] = 0.0;
   out_4143829145148051441[300] = 0.0;
   out_4143829145148051441[301] = 0.0;
   out_4143829145148051441[302] = 0.0;
   out_4143829145148051441[303] = 0.0;
   out_4143829145148051441[304] = 1.0;
   out_4143829145148051441[305] = 0.0;
   out_4143829145148051441[306] = 0.0;
   out_4143829145148051441[307] = 0.0;
   out_4143829145148051441[308] = 0.0;
   out_4143829145148051441[309] = 0.0;
   out_4143829145148051441[310] = 0.0;
   out_4143829145148051441[311] = 0.0;
   out_4143829145148051441[312] = 0.0;
   out_4143829145148051441[313] = 0.0;
   out_4143829145148051441[314] = 0.0;
   out_4143829145148051441[315] = 0.0;
   out_4143829145148051441[316] = 0.0;
   out_4143829145148051441[317] = 0.0;
   out_4143829145148051441[318] = 0.0;
   out_4143829145148051441[319] = 0.0;
   out_4143829145148051441[320] = 0.0;
   out_4143829145148051441[321] = 0.0;
   out_4143829145148051441[322] = 0.0;
   out_4143829145148051441[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_2866496863871759496) {
   out_2866496863871759496[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_2866496863871759496[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_2866496863871759496[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_2866496863871759496[3] = dt*state[12] + state[3];
   out_2866496863871759496[4] = dt*state[13] + state[4];
   out_2866496863871759496[5] = dt*state[14] + state[5];
   out_2866496863871759496[6] = state[6];
   out_2866496863871759496[7] = state[7];
   out_2866496863871759496[8] = state[8];
   out_2866496863871759496[9] = state[9];
   out_2866496863871759496[10] = state[10];
   out_2866496863871759496[11] = state[11];
   out_2866496863871759496[12] = state[12];
   out_2866496863871759496[13] = state[13];
   out_2866496863871759496[14] = state[14];
   out_2866496863871759496[15] = state[15];
   out_2866496863871759496[16] = state[16];
   out_2866496863871759496[17] = state[17];
}
void F_fun(double *state, double dt, double *out_2805007371861002435) {
   out_2805007371861002435[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2805007371861002435[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2805007371861002435[2] = 0;
   out_2805007371861002435[3] = 0;
   out_2805007371861002435[4] = 0;
   out_2805007371861002435[5] = 0;
   out_2805007371861002435[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2805007371861002435[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2805007371861002435[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2805007371861002435[9] = 0;
   out_2805007371861002435[10] = 0;
   out_2805007371861002435[11] = 0;
   out_2805007371861002435[12] = 0;
   out_2805007371861002435[13] = 0;
   out_2805007371861002435[14] = 0;
   out_2805007371861002435[15] = 0;
   out_2805007371861002435[16] = 0;
   out_2805007371861002435[17] = 0;
   out_2805007371861002435[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2805007371861002435[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2805007371861002435[20] = 0;
   out_2805007371861002435[21] = 0;
   out_2805007371861002435[22] = 0;
   out_2805007371861002435[23] = 0;
   out_2805007371861002435[24] = 0;
   out_2805007371861002435[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2805007371861002435[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2805007371861002435[27] = 0;
   out_2805007371861002435[28] = 0;
   out_2805007371861002435[29] = 0;
   out_2805007371861002435[30] = 0;
   out_2805007371861002435[31] = 0;
   out_2805007371861002435[32] = 0;
   out_2805007371861002435[33] = 0;
   out_2805007371861002435[34] = 0;
   out_2805007371861002435[35] = 0;
   out_2805007371861002435[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2805007371861002435[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2805007371861002435[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2805007371861002435[39] = 0;
   out_2805007371861002435[40] = 0;
   out_2805007371861002435[41] = 0;
   out_2805007371861002435[42] = 0;
   out_2805007371861002435[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2805007371861002435[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2805007371861002435[45] = 0;
   out_2805007371861002435[46] = 0;
   out_2805007371861002435[47] = 0;
   out_2805007371861002435[48] = 0;
   out_2805007371861002435[49] = 0;
   out_2805007371861002435[50] = 0;
   out_2805007371861002435[51] = 0;
   out_2805007371861002435[52] = 0;
   out_2805007371861002435[53] = 0;
   out_2805007371861002435[54] = 0;
   out_2805007371861002435[55] = 0;
   out_2805007371861002435[56] = 0;
   out_2805007371861002435[57] = 1;
   out_2805007371861002435[58] = 0;
   out_2805007371861002435[59] = 0;
   out_2805007371861002435[60] = 0;
   out_2805007371861002435[61] = 0;
   out_2805007371861002435[62] = 0;
   out_2805007371861002435[63] = 0;
   out_2805007371861002435[64] = 0;
   out_2805007371861002435[65] = 0;
   out_2805007371861002435[66] = dt;
   out_2805007371861002435[67] = 0;
   out_2805007371861002435[68] = 0;
   out_2805007371861002435[69] = 0;
   out_2805007371861002435[70] = 0;
   out_2805007371861002435[71] = 0;
   out_2805007371861002435[72] = 0;
   out_2805007371861002435[73] = 0;
   out_2805007371861002435[74] = 0;
   out_2805007371861002435[75] = 0;
   out_2805007371861002435[76] = 1;
   out_2805007371861002435[77] = 0;
   out_2805007371861002435[78] = 0;
   out_2805007371861002435[79] = 0;
   out_2805007371861002435[80] = 0;
   out_2805007371861002435[81] = 0;
   out_2805007371861002435[82] = 0;
   out_2805007371861002435[83] = 0;
   out_2805007371861002435[84] = 0;
   out_2805007371861002435[85] = dt;
   out_2805007371861002435[86] = 0;
   out_2805007371861002435[87] = 0;
   out_2805007371861002435[88] = 0;
   out_2805007371861002435[89] = 0;
   out_2805007371861002435[90] = 0;
   out_2805007371861002435[91] = 0;
   out_2805007371861002435[92] = 0;
   out_2805007371861002435[93] = 0;
   out_2805007371861002435[94] = 0;
   out_2805007371861002435[95] = 1;
   out_2805007371861002435[96] = 0;
   out_2805007371861002435[97] = 0;
   out_2805007371861002435[98] = 0;
   out_2805007371861002435[99] = 0;
   out_2805007371861002435[100] = 0;
   out_2805007371861002435[101] = 0;
   out_2805007371861002435[102] = 0;
   out_2805007371861002435[103] = 0;
   out_2805007371861002435[104] = dt;
   out_2805007371861002435[105] = 0;
   out_2805007371861002435[106] = 0;
   out_2805007371861002435[107] = 0;
   out_2805007371861002435[108] = 0;
   out_2805007371861002435[109] = 0;
   out_2805007371861002435[110] = 0;
   out_2805007371861002435[111] = 0;
   out_2805007371861002435[112] = 0;
   out_2805007371861002435[113] = 0;
   out_2805007371861002435[114] = 1;
   out_2805007371861002435[115] = 0;
   out_2805007371861002435[116] = 0;
   out_2805007371861002435[117] = 0;
   out_2805007371861002435[118] = 0;
   out_2805007371861002435[119] = 0;
   out_2805007371861002435[120] = 0;
   out_2805007371861002435[121] = 0;
   out_2805007371861002435[122] = 0;
   out_2805007371861002435[123] = 0;
   out_2805007371861002435[124] = 0;
   out_2805007371861002435[125] = 0;
   out_2805007371861002435[126] = 0;
   out_2805007371861002435[127] = 0;
   out_2805007371861002435[128] = 0;
   out_2805007371861002435[129] = 0;
   out_2805007371861002435[130] = 0;
   out_2805007371861002435[131] = 0;
   out_2805007371861002435[132] = 0;
   out_2805007371861002435[133] = 1;
   out_2805007371861002435[134] = 0;
   out_2805007371861002435[135] = 0;
   out_2805007371861002435[136] = 0;
   out_2805007371861002435[137] = 0;
   out_2805007371861002435[138] = 0;
   out_2805007371861002435[139] = 0;
   out_2805007371861002435[140] = 0;
   out_2805007371861002435[141] = 0;
   out_2805007371861002435[142] = 0;
   out_2805007371861002435[143] = 0;
   out_2805007371861002435[144] = 0;
   out_2805007371861002435[145] = 0;
   out_2805007371861002435[146] = 0;
   out_2805007371861002435[147] = 0;
   out_2805007371861002435[148] = 0;
   out_2805007371861002435[149] = 0;
   out_2805007371861002435[150] = 0;
   out_2805007371861002435[151] = 0;
   out_2805007371861002435[152] = 1;
   out_2805007371861002435[153] = 0;
   out_2805007371861002435[154] = 0;
   out_2805007371861002435[155] = 0;
   out_2805007371861002435[156] = 0;
   out_2805007371861002435[157] = 0;
   out_2805007371861002435[158] = 0;
   out_2805007371861002435[159] = 0;
   out_2805007371861002435[160] = 0;
   out_2805007371861002435[161] = 0;
   out_2805007371861002435[162] = 0;
   out_2805007371861002435[163] = 0;
   out_2805007371861002435[164] = 0;
   out_2805007371861002435[165] = 0;
   out_2805007371861002435[166] = 0;
   out_2805007371861002435[167] = 0;
   out_2805007371861002435[168] = 0;
   out_2805007371861002435[169] = 0;
   out_2805007371861002435[170] = 0;
   out_2805007371861002435[171] = 1;
   out_2805007371861002435[172] = 0;
   out_2805007371861002435[173] = 0;
   out_2805007371861002435[174] = 0;
   out_2805007371861002435[175] = 0;
   out_2805007371861002435[176] = 0;
   out_2805007371861002435[177] = 0;
   out_2805007371861002435[178] = 0;
   out_2805007371861002435[179] = 0;
   out_2805007371861002435[180] = 0;
   out_2805007371861002435[181] = 0;
   out_2805007371861002435[182] = 0;
   out_2805007371861002435[183] = 0;
   out_2805007371861002435[184] = 0;
   out_2805007371861002435[185] = 0;
   out_2805007371861002435[186] = 0;
   out_2805007371861002435[187] = 0;
   out_2805007371861002435[188] = 0;
   out_2805007371861002435[189] = 0;
   out_2805007371861002435[190] = 1;
   out_2805007371861002435[191] = 0;
   out_2805007371861002435[192] = 0;
   out_2805007371861002435[193] = 0;
   out_2805007371861002435[194] = 0;
   out_2805007371861002435[195] = 0;
   out_2805007371861002435[196] = 0;
   out_2805007371861002435[197] = 0;
   out_2805007371861002435[198] = 0;
   out_2805007371861002435[199] = 0;
   out_2805007371861002435[200] = 0;
   out_2805007371861002435[201] = 0;
   out_2805007371861002435[202] = 0;
   out_2805007371861002435[203] = 0;
   out_2805007371861002435[204] = 0;
   out_2805007371861002435[205] = 0;
   out_2805007371861002435[206] = 0;
   out_2805007371861002435[207] = 0;
   out_2805007371861002435[208] = 0;
   out_2805007371861002435[209] = 1;
   out_2805007371861002435[210] = 0;
   out_2805007371861002435[211] = 0;
   out_2805007371861002435[212] = 0;
   out_2805007371861002435[213] = 0;
   out_2805007371861002435[214] = 0;
   out_2805007371861002435[215] = 0;
   out_2805007371861002435[216] = 0;
   out_2805007371861002435[217] = 0;
   out_2805007371861002435[218] = 0;
   out_2805007371861002435[219] = 0;
   out_2805007371861002435[220] = 0;
   out_2805007371861002435[221] = 0;
   out_2805007371861002435[222] = 0;
   out_2805007371861002435[223] = 0;
   out_2805007371861002435[224] = 0;
   out_2805007371861002435[225] = 0;
   out_2805007371861002435[226] = 0;
   out_2805007371861002435[227] = 0;
   out_2805007371861002435[228] = 1;
   out_2805007371861002435[229] = 0;
   out_2805007371861002435[230] = 0;
   out_2805007371861002435[231] = 0;
   out_2805007371861002435[232] = 0;
   out_2805007371861002435[233] = 0;
   out_2805007371861002435[234] = 0;
   out_2805007371861002435[235] = 0;
   out_2805007371861002435[236] = 0;
   out_2805007371861002435[237] = 0;
   out_2805007371861002435[238] = 0;
   out_2805007371861002435[239] = 0;
   out_2805007371861002435[240] = 0;
   out_2805007371861002435[241] = 0;
   out_2805007371861002435[242] = 0;
   out_2805007371861002435[243] = 0;
   out_2805007371861002435[244] = 0;
   out_2805007371861002435[245] = 0;
   out_2805007371861002435[246] = 0;
   out_2805007371861002435[247] = 1;
   out_2805007371861002435[248] = 0;
   out_2805007371861002435[249] = 0;
   out_2805007371861002435[250] = 0;
   out_2805007371861002435[251] = 0;
   out_2805007371861002435[252] = 0;
   out_2805007371861002435[253] = 0;
   out_2805007371861002435[254] = 0;
   out_2805007371861002435[255] = 0;
   out_2805007371861002435[256] = 0;
   out_2805007371861002435[257] = 0;
   out_2805007371861002435[258] = 0;
   out_2805007371861002435[259] = 0;
   out_2805007371861002435[260] = 0;
   out_2805007371861002435[261] = 0;
   out_2805007371861002435[262] = 0;
   out_2805007371861002435[263] = 0;
   out_2805007371861002435[264] = 0;
   out_2805007371861002435[265] = 0;
   out_2805007371861002435[266] = 1;
   out_2805007371861002435[267] = 0;
   out_2805007371861002435[268] = 0;
   out_2805007371861002435[269] = 0;
   out_2805007371861002435[270] = 0;
   out_2805007371861002435[271] = 0;
   out_2805007371861002435[272] = 0;
   out_2805007371861002435[273] = 0;
   out_2805007371861002435[274] = 0;
   out_2805007371861002435[275] = 0;
   out_2805007371861002435[276] = 0;
   out_2805007371861002435[277] = 0;
   out_2805007371861002435[278] = 0;
   out_2805007371861002435[279] = 0;
   out_2805007371861002435[280] = 0;
   out_2805007371861002435[281] = 0;
   out_2805007371861002435[282] = 0;
   out_2805007371861002435[283] = 0;
   out_2805007371861002435[284] = 0;
   out_2805007371861002435[285] = 1;
   out_2805007371861002435[286] = 0;
   out_2805007371861002435[287] = 0;
   out_2805007371861002435[288] = 0;
   out_2805007371861002435[289] = 0;
   out_2805007371861002435[290] = 0;
   out_2805007371861002435[291] = 0;
   out_2805007371861002435[292] = 0;
   out_2805007371861002435[293] = 0;
   out_2805007371861002435[294] = 0;
   out_2805007371861002435[295] = 0;
   out_2805007371861002435[296] = 0;
   out_2805007371861002435[297] = 0;
   out_2805007371861002435[298] = 0;
   out_2805007371861002435[299] = 0;
   out_2805007371861002435[300] = 0;
   out_2805007371861002435[301] = 0;
   out_2805007371861002435[302] = 0;
   out_2805007371861002435[303] = 0;
   out_2805007371861002435[304] = 1;
   out_2805007371861002435[305] = 0;
   out_2805007371861002435[306] = 0;
   out_2805007371861002435[307] = 0;
   out_2805007371861002435[308] = 0;
   out_2805007371861002435[309] = 0;
   out_2805007371861002435[310] = 0;
   out_2805007371861002435[311] = 0;
   out_2805007371861002435[312] = 0;
   out_2805007371861002435[313] = 0;
   out_2805007371861002435[314] = 0;
   out_2805007371861002435[315] = 0;
   out_2805007371861002435[316] = 0;
   out_2805007371861002435[317] = 0;
   out_2805007371861002435[318] = 0;
   out_2805007371861002435[319] = 0;
   out_2805007371861002435[320] = 0;
   out_2805007371861002435[321] = 0;
   out_2805007371861002435[322] = 0;
   out_2805007371861002435[323] = 1;
}
void h_4(double *state, double *unused, double *out_4149227121328903650) {
   out_4149227121328903650[0] = state[6] + state[9];
   out_4149227121328903650[1] = state[7] + state[10];
   out_4149227121328903650[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_7915636354991959013) {
   out_7915636354991959013[0] = 0;
   out_7915636354991959013[1] = 0;
   out_7915636354991959013[2] = 0;
   out_7915636354991959013[3] = 0;
   out_7915636354991959013[4] = 0;
   out_7915636354991959013[5] = 0;
   out_7915636354991959013[6] = 1;
   out_7915636354991959013[7] = 0;
   out_7915636354991959013[8] = 0;
   out_7915636354991959013[9] = 1;
   out_7915636354991959013[10] = 0;
   out_7915636354991959013[11] = 0;
   out_7915636354991959013[12] = 0;
   out_7915636354991959013[13] = 0;
   out_7915636354991959013[14] = 0;
   out_7915636354991959013[15] = 0;
   out_7915636354991959013[16] = 0;
   out_7915636354991959013[17] = 0;
   out_7915636354991959013[18] = 0;
   out_7915636354991959013[19] = 0;
   out_7915636354991959013[20] = 0;
   out_7915636354991959013[21] = 0;
   out_7915636354991959013[22] = 0;
   out_7915636354991959013[23] = 0;
   out_7915636354991959013[24] = 0;
   out_7915636354991959013[25] = 1;
   out_7915636354991959013[26] = 0;
   out_7915636354991959013[27] = 0;
   out_7915636354991959013[28] = 1;
   out_7915636354991959013[29] = 0;
   out_7915636354991959013[30] = 0;
   out_7915636354991959013[31] = 0;
   out_7915636354991959013[32] = 0;
   out_7915636354991959013[33] = 0;
   out_7915636354991959013[34] = 0;
   out_7915636354991959013[35] = 0;
   out_7915636354991959013[36] = 0;
   out_7915636354991959013[37] = 0;
   out_7915636354991959013[38] = 0;
   out_7915636354991959013[39] = 0;
   out_7915636354991959013[40] = 0;
   out_7915636354991959013[41] = 0;
   out_7915636354991959013[42] = 0;
   out_7915636354991959013[43] = 0;
   out_7915636354991959013[44] = 1;
   out_7915636354991959013[45] = 0;
   out_7915636354991959013[46] = 0;
   out_7915636354991959013[47] = 1;
   out_7915636354991959013[48] = 0;
   out_7915636354991959013[49] = 0;
   out_7915636354991959013[50] = 0;
   out_7915636354991959013[51] = 0;
   out_7915636354991959013[52] = 0;
   out_7915636354991959013[53] = 0;
}
void h_10(double *state, double *unused, double *out_3295896223844812439) {
   out_3295896223844812439[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_3295896223844812439[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_3295896223844812439[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_8598042528087430041) {
   out_8598042528087430041[0] = 0;
   out_8598042528087430041[1] = 9.8100000000000005*cos(state[1]);
   out_8598042528087430041[2] = 0;
   out_8598042528087430041[3] = 0;
   out_8598042528087430041[4] = -state[8];
   out_8598042528087430041[5] = state[7];
   out_8598042528087430041[6] = 0;
   out_8598042528087430041[7] = state[5];
   out_8598042528087430041[8] = -state[4];
   out_8598042528087430041[9] = 0;
   out_8598042528087430041[10] = 0;
   out_8598042528087430041[11] = 0;
   out_8598042528087430041[12] = 1;
   out_8598042528087430041[13] = 0;
   out_8598042528087430041[14] = 0;
   out_8598042528087430041[15] = 1;
   out_8598042528087430041[16] = 0;
   out_8598042528087430041[17] = 0;
   out_8598042528087430041[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_8598042528087430041[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_8598042528087430041[20] = 0;
   out_8598042528087430041[21] = state[8];
   out_8598042528087430041[22] = 0;
   out_8598042528087430041[23] = -state[6];
   out_8598042528087430041[24] = -state[5];
   out_8598042528087430041[25] = 0;
   out_8598042528087430041[26] = state[3];
   out_8598042528087430041[27] = 0;
   out_8598042528087430041[28] = 0;
   out_8598042528087430041[29] = 0;
   out_8598042528087430041[30] = 0;
   out_8598042528087430041[31] = 1;
   out_8598042528087430041[32] = 0;
   out_8598042528087430041[33] = 0;
   out_8598042528087430041[34] = 1;
   out_8598042528087430041[35] = 0;
   out_8598042528087430041[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_8598042528087430041[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_8598042528087430041[38] = 0;
   out_8598042528087430041[39] = -state[7];
   out_8598042528087430041[40] = state[6];
   out_8598042528087430041[41] = 0;
   out_8598042528087430041[42] = state[4];
   out_8598042528087430041[43] = -state[3];
   out_8598042528087430041[44] = 0;
   out_8598042528087430041[45] = 0;
   out_8598042528087430041[46] = 0;
   out_8598042528087430041[47] = 0;
   out_8598042528087430041[48] = 0;
   out_8598042528087430041[49] = 0;
   out_8598042528087430041[50] = 1;
   out_8598042528087430041[51] = 0;
   out_8598042528087430041[52] = 0;
   out_8598042528087430041[53] = 1;
}
void h_13(double *state, double *unused, double *out_2654614890869165391) {
   out_2654614890869165391[0] = state[3];
   out_2654614890869165391[1] = state[4];
   out_2654614890869165391[2] = state[5];
}
void H_13(double *state, double *unused, double *out_7318833893385259802) {
   out_7318833893385259802[0] = 0;
   out_7318833893385259802[1] = 0;
   out_7318833893385259802[2] = 0;
   out_7318833893385259802[3] = 1;
   out_7318833893385259802[4] = 0;
   out_7318833893385259802[5] = 0;
   out_7318833893385259802[6] = 0;
   out_7318833893385259802[7] = 0;
   out_7318833893385259802[8] = 0;
   out_7318833893385259802[9] = 0;
   out_7318833893385259802[10] = 0;
   out_7318833893385259802[11] = 0;
   out_7318833893385259802[12] = 0;
   out_7318833893385259802[13] = 0;
   out_7318833893385259802[14] = 0;
   out_7318833893385259802[15] = 0;
   out_7318833893385259802[16] = 0;
   out_7318833893385259802[17] = 0;
   out_7318833893385259802[18] = 0;
   out_7318833893385259802[19] = 0;
   out_7318833893385259802[20] = 0;
   out_7318833893385259802[21] = 0;
   out_7318833893385259802[22] = 1;
   out_7318833893385259802[23] = 0;
   out_7318833893385259802[24] = 0;
   out_7318833893385259802[25] = 0;
   out_7318833893385259802[26] = 0;
   out_7318833893385259802[27] = 0;
   out_7318833893385259802[28] = 0;
   out_7318833893385259802[29] = 0;
   out_7318833893385259802[30] = 0;
   out_7318833893385259802[31] = 0;
   out_7318833893385259802[32] = 0;
   out_7318833893385259802[33] = 0;
   out_7318833893385259802[34] = 0;
   out_7318833893385259802[35] = 0;
   out_7318833893385259802[36] = 0;
   out_7318833893385259802[37] = 0;
   out_7318833893385259802[38] = 0;
   out_7318833893385259802[39] = 0;
   out_7318833893385259802[40] = 0;
   out_7318833893385259802[41] = 1;
   out_7318833893385259802[42] = 0;
   out_7318833893385259802[43] = 0;
   out_7318833893385259802[44] = 0;
   out_7318833893385259802[45] = 0;
   out_7318833893385259802[46] = 0;
   out_7318833893385259802[47] = 0;
   out_7318833893385259802[48] = 0;
   out_7318833893385259802[49] = 0;
   out_7318833893385259802[50] = 0;
   out_7318833893385259802[51] = 0;
   out_7318833893385259802[52] = 0;
   out_7318833893385259802[53] = 0;
}
void h_14(double *state, double *unused, double *out_4682865712639379166) {
   out_4682865712639379166[0] = state[6];
   out_4682865712639379166[1] = state[7];
   out_4682865712639379166[2] = state[8];
}
void H_14(double *state, double *unused, double *out_4832847922696586717) {
   out_4832847922696586717[0] = 0;
   out_4832847922696586717[1] = 0;
   out_4832847922696586717[2] = 0;
   out_4832847922696586717[3] = 0;
   out_4832847922696586717[4] = 0;
   out_4832847922696586717[5] = 0;
   out_4832847922696586717[6] = 1;
   out_4832847922696586717[7] = 0;
   out_4832847922696586717[8] = 0;
   out_4832847922696586717[9] = 0;
   out_4832847922696586717[10] = 0;
   out_4832847922696586717[11] = 0;
   out_4832847922696586717[12] = 0;
   out_4832847922696586717[13] = 0;
   out_4832847922696586717[14] = 0;
   out_4832847922696586717[15] = 0;
   out_4832847922696586717[16] = 0;
   out_4832847922696586717[17] = 0;
   out_4832847922696586717[18] = 0;
   out_4832847922696586717[19] = 0;
   out_4832847922696586717[20] = 0;
   out_4832847922696586717[21] = 0;
   out_4832847922696586717[22] = 0;
   out_4832847922696586717[23] = 0;
   out_4832847922696586717[24] = 0;
   out_4832847922696586717[25] = 1;
   out_4832847922696586717[26] = 0;
   out_4832847922696586717[27] = 0;
   out_4832847922696586717[28] = 0;
   out_4832847922696586717[29] = 0;
   out_4832847922696586717[30] = 0;
   out_4832847922696586717[31] = 0;
   out_4832847922696586717[32] = 0;
   out_4832847922696586717[33] = 0;
   out_4832847922696586717[34] = 0;
   out_4832847922696586717[35] = 0;
   out_4832847922696586717[36] = 0;
   out_4832847922696586717[37] = 0;
   out_4832847922696586717[38] = 0;
   out_4832847922696586717[39] = 0;
   out_4832847922696586717[40] = 0;
   out_4832847922696586717[41] = 0;
   out_4832847922696586717[42] = 0;
   out_4832847922696586717[43] = 0;
   out_4832847922696586717[44] = 1;
   out_4832847922696586717[45] = 0;
   out_4832847922696586717[46] = 0;
   out_4832847922696586717[47] = 0;
   out_4832847922696586717[48] = 0;
   out_4832847922696586717[49] = 0;
   out_4832847922696586717[50] = 0;
   out_4832847922696586717[51] = 0;
   out_4832847922696586717[52] = 0;
   out_4832847922696586717[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_4337369684268981257) {
  err_fun(nom_x, delta_x, out_4337369684268981257);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5765766890507513398) {
  inv_err_fun(nom_x, true_x, out_5765766890507513398);
}
void pose_H_mod_fun(double *state, double *out_4143829145148051441) {
  H_mod_fun(state, out_4143829145148051441);
}
void pose_f_fun(double *state, double dt, double *out_2866496863871759496) {
  f_fun(state,  dt, out_2866496863871759496);
}
void pose_F_fun(double *state, double dt, double *out_2805007371861002435) {
  F_fun(state,  dt, out_2805007371861002435);
}
void pose_h_4(double *state, double *unused, double *out_4149227121328903650) {
  h_4(state, unused, out_4149227121328903650);
}
void pose_H_4(double *state, double *unused, double *out_7915636354991959013) {
  H_4(state, unused, out_7915636354991959013);
}
void pose_h_10(double *state, double *unused, double *out_3295896223844812439) {
  h_10(state, unused, out_3295896223844812439);
}
void pose_H_10(double *state, double *unused, double *out_8598042528087430041) {
  H_10(state, unused, out_8598042528087430041);
}
void pose_h_13(double *state, double *unused, double *out_2654614890869165391) {
  h_13(state, unused, out_2654614890869165391);
}
void pose_H_13(double *state, double *unused, double *out_7318833893385259802) {
  H_13(state, unused, out_7318833893385259802);
}
void pose_h_14(double *state, double *unused, double *out_4682865712639379166) {
  h_14(state, unused, out_4682865712639379166);
}
void pose_H_14(double *state, double *unused, double *out_4832847922696586717) {
  H_14(state, unused, out_4832847922696586717);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
