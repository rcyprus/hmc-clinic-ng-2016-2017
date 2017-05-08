/* Produced by CVXGEN, 2017-04-11 23:04:38 -0400.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: csolve.c. */
/* Description: mex-able file for running cvxgen solver. */
#include "mex.h"
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  int i, j;
  mxArray *xm, *cell, *xm_cell;
  double *src;
  double *dest;
  double *dest_cell;
  int valid_vars;
  int steps;
  int this_var_errors;
  int warned_diags;
  int prepare_for_c = 0;
  int extra_solves;
  const char *status_names[] = {"optval", "gap", "steps", "converged"};
  mwSize dims1x1of1[1] = {1};
  mwSize dims[1];
  const char *var_names[] = {"x"};
  const int num_var_names = 1;
  /* Avoid compiler warnings of unused variables by using a dummy assignment. */
  warned_diags = j = 0;
  extra_solves = 0;
  set_defaults();
  /* Check we got the right number of arguments. */
  if (nrhs == 0)
    mexErrMsgTxt("Not enough arguments: You need to specify at least the parameters.\n");
  if (nrhs > 1) {
    /* Assume that the second argument is the settings. */
    if (mxGetField(prhs[1], 0, "eps") != NULL)
      settings.eps = *mxGetPr(mxGetField(prhs[1], 0, "eps"));
    if (mxGetField(prhs[1], 0, "max_iters") != NULL)
      settings.max_iters = *mxGetPr(mxGetField(prhs[1], 0, "max_iters"));
    if (mxGetField(prhs[1], 0, "refine_steps") != NULL)
      settings.refine_steps = *mxGetPr(mxGetField(prhs[1], 0, "refine_steps"));
    if (mxGetField(prhs[1], 0, "verbose") != NULL)
      settings.verbose = *mxGetPr(mxGetField(prhs[1], 0, "verbose"));
    if (mxGetField(prhs[1], 0, "better_start") != NULL)
      settings.better_start = *mxGetPr(mxGetField(prhs[1], 0, "better_start"));
    if (mxGetField(prhs[1], 0, "verbose_refinement") != NULL)
      settings.verbose_refinement = *mxGetPr(mxGetField(prhs[1], 0,
            "verbose_refinement"));
    if (mxGetField(prhs[1], 0, "debug") != NULL)
      settings.debug = *mxGetPr(mxGetField(prhs[1], 0, "debug"));
    if (mxGetField(prhs[1], 0, "kkt_reg") != NULL)
      settings.kkt_reg = *mxGetPr(mxGetField(prhs[1], 0, "kkt_reg"));
    if (mxGetField(prhs[1], 0, "s_init") != NULL)
      settings.s_init = *mxGetPr(mxGetField(prhs[1], 0, "s_init"));
    if (mxGetField(prhs[1], 0, "z_init") != NULL)
      settings.z_init = *mxGetPr(mxGetField(prhs[1], 0, "z_init"));
    if (mxGetField(prhs[1], 0, "resid_tol") != NULL)
      settings.resid_tol = *mxGetPr(mxGetField(prhs[1], 0, "resid_tol"));
    if (mxGetField(prhs[1], 0, "extra_solves") != NULL)
      extra_solves = *mxGetPr(mxGetField(prhs[1], 0, "extra_solves"));
    else
      extra_solves = 0;
    if (mxGetField(prhs[1], 0, "prepare_for_c") != NULL)
      prepare_for_c = *mxGetPr(mxGetField(prhs[1], 0, "prepare_for_c"));
  }
  valid_vars = 0;
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CA");
  if (xm == NULL) {
    printf("could not find params.CA.\n");
  } else {
    if (!((mxGetM(xm) == 140) && (mxGetN(xm) == 4))) {
      printf("CA must be size (140,4), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CA must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CA must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CA must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CA;
      src = mxGetPr(xm);
      dest[0] = src[0];  /* (1,1) entry. */
      dest[1] = src[1];  /* (2,1) entry. */
      dest[2] = src[142];  /* (3,2) entry. */
      dest[3] = src[283];  /* (4,3) entry. */
      dest[4] = src[424];  /* (5,4) entry. */
      dest[5] = src[145];  /* (6,2) entry. */
      dest[6] = src[286];  /* (7,3) entry. */
      dest[7] = src[427];  /* (8,4) entry. */
      dest[8] = src[148];  /* (9,2) entry. */
      dest[9] = src[289];  /* (10,3) entry. */
      dest[10] = src[430];  /* (11,4) entry. */
      dest[11] = src[151];  /* (12,2) entry. */
      dest[12] = src[292];  /* (13,3) entry. */
      dest[13] = src[433];  /* (14,4) entry. */
      dest[14] = src[420];  /* (1,4) entry. */
      dest[15] = src[421];  /* (2,4) entry. */
      dest[16] = src[14];  /* (15,1) entry. */
      dest[17] = src[15];  /* (16,1) entry. */
      dest[18] = src[156];  /* (17,2) entry. */
      dest[19] = src[297];  /* (18,3) entry. */
      dest[20] = src[438];  /* (19,4) entry. */
      dest[21] = src[159];  /* (20,2) entry. */
      dest[22] = src[300];  /* (21,3) entry. */
      dest[23] = src[441];  /* (22,4) entry. */
      dest[24] = src[162];  /* (23,2) entry. */
      dest[25] = src[303];  /* (24,3) entry. */
      dest[26] = src[444];  /* (25,4) entry. */
      dest[27] = src[165];  /* (26,2) entry. */
      dest[28] = src[306];  /* (27,3) entry. */
      dest[29] = src[447];  /* (28,4) entry. */
      dest[30] = src[434];  /* (15,4) entry. */
      dest[31] = src[435];  /* (16,4) entry. */
      dest[32] = src[28];  /* (29,1) entry. */
      dest[33] = src[29];  /* (30,1) entry. */
      dest[34] = src[170];  /* (31,2) entry. */
      dest[35] = src[311];  /* (32,3) entry. */
      dest[36] = src[452];  /* (33,4) entry. */
      dest[37] = src[173];  /* (34,2) entry. */
      dest[38] = src[314];  /* (35,3) entry. */
      dest[39] = src[455];  /* (36,4) entry. */
      dest[40] = src[176];  /* (37,2) entry. */
      dest[41] = src[317];  /* (38,3) entry. */
      dest[42] = src[458];  /* (39,4) entry. */
      dest[43] = src[179];  /* (40,2) entry. */
      dest[44] = src[320];  /* (41,3) entry. */
      dest[45] = src[461];  /* (42,4) entry. */
      dest[46] = src[448];  /* (29,4) entry. */
      dest[47] = src[449];  /* (30,4) entry. */
      dest[48] = src[42];  /* (43,1) entry. */
      dest[49] = src[43];  /* (44,1) entry. */
      dest[50] = src[184];  /* (45,2) entry. */
      dest[51] = src[325];  /* (46,3) entry. */
      dest[52] = src[466];  /* (47,4) entry. */
      dest[53] = src[187];  /* (48,2) entry. */
      dest[54] = src[328];  /* (49,3) entry. */
      dest[55] = src[469];  /* (50,4) entry. */
      dest[56] = src[190];  /* (51,2) entry. */
      dest[57] = src[331];  /* (52,3) entry. */
      dest[58] = src[472];  /* (53,4) entry. */
      dest[59] = src[193];  /* (54,2) entry. */
      dest[60] = src[334];  /* (55,3) entry. */
      dest[61] = src[475];  /* (56,4) entry. */
      dest[62] = src[462];  /* (43,4) entry. */
      dest[63] = src[463];  /* (44,4) entry. */
      dest[64] = src[56];  /* (57,1) entry. */
      dest[65] = src[57];  /* (58,1) entry. */
      dest[66] = src[198];  /* (59,2) entry. */
      dest[67] = src[339];  /* (60,3) entry. */
      dest[68] = src[480];  /* (61,4) entry. */
      dest[69] = src[201];  /* (62,2) entry. */
      dest[70] = src[342];  /* (63,3) entry. */
      dest[71] = src[483];  /* (64,4) entry. */
      dest[72] = src[204];  /* (65,2) entry. */
      dest[73] = src[345];  /* (66,3) entry. */
      dest[74] = src[486];  /* (67,4) entry. */
      dest[75] = src[207];  /* (68,2) entry. */
      dest[76] = src[348];  /* (69,3) entry. */
      dest[77] = src[489];  /* (70,4) entry. */
      dest[78] = src[476];  /* (57,4) entry. */
      dest[79] = src[477];  /* (58,4) entry. */
      dest[80] = src[70];  /* (71,1) entry. */
      dest[81] = src[71];  /* (72,1) entry. */
      dest[82] = src[212];  /* (73,2) entry. */
      dest[83] = src[353];  /* (74,3) entry. */
      dest[84] = src[494];  /* (75,4) entry. */
      dest[85] = src[215];  /* (76,2) entry. */
      dest[86] = src[356];  /* (77,3) entry. */
      dest[87] = src[497];  /* (78,4) entry. */
      dest[88] = src[218];  /* (79,2) entry. */
      dest[89] = src[359];  /* (80,3) entry. */
      dest[90] = src[500];  /* (81,4) entry. */
      dest[91] = src[221];  /* (82,2) entry. */
      dest[92] = src[362];  /* (83,3) entry. */
      dest[93] = src[503];  /* (84,4) entry. */
      dest[94] = src[490];  /* (71,4) entry. */
      dest[95] = src[491];  /* (72,4) entry. */
      dest[96] = src[84];  /* (85,1) entry. */
      dest[97] = src[85];  /* (86,1) entry. */
      dest[98] = src[226];  /* (87,2) entry. */
      dest[99] = src[367];  /* (88,3) entry. */
      dest[100] = src[508];  /* (89,4) entry. */
      dest[101] = src[229];  /* (90,2) entry. */
      dest[102] = src[370];  /* (91,3) entry. */
      dest[103] = src[511];  /* (92,4) entry. */
      dest[104] = src[232];  /* (93,2) entry. */
      dest[105] = src[373];  /* (94,3) entry. */
      dest[106] = src[514];  /* (95,4) entry. */
      dest[107] = src[235];  /* (96,2) entry. */
      dest[108] = src[376];  /* (97,3) entry. */
      dest[109] = src[517];  /* (98,4) entry. */
      dest[110] = src[504];  /* (85,4) entry. */
      dest[111] = src[505];  /* (86,4) entry. */
      dest[112] = src[98];  /* (99,1) entry. */
      dest[113] = src[99];  /* (100,1) entry. */
      dest[114] = src[240];  /* (101,2) entry. */
      dest[115] = src[381];  /* (102,3) entry. */
      dest[116] = src[522];  /* (103,4) entry. */
      dest[117] = src[243];  /* (104,2) entry. */
      dest[118] = src[384];  /* (105,3) entry. */
      dest[119] = src[525];  /* (106,4) entry. */
      dest[120] = src[246];  /* (107,2) entry. */
      dest[121] = src[387];  /* (108,3) entry. */
      dest[122] = src[528];  /* (109,4) entry. */
      dest[123] = src[249];  /* (110,2) entry. */
      dest[124] = src[390];  /* (111,3) entry. */
      dest[125] = src[531];  /* (112,4) entry. */
      dest[126] = src[518];  /* (99,4) entry. */
      dest[127] = src[519];  /* (100,4) entry. */
      dest[128] = src[112];  /* (113,1) entry. */
      dest[129] = src[113];  /* (114,1) entry. */
      dest[130] = src[254];  /* (115,2) entry. */
      dest[131] = src[395];  /* (116,3) entry. */
      dest[132] = src[536];  /* (117,4) entry. */
      dest[133] = src[257];  /* (118,2) entry. */
      dest[134] = src[398];  /* (119,3) entry. */
      dest[135] = src[539];  /* (120,4) entry. */
      dest[136] = src[260];  /* (121,2) entry. */
      dest[137] = src[401];  /* (122,3) entry. */
      dest[138] = src[542];  /* (123,4) entry. */
      dest[139] = src[263];  /* (124,2) entry. */
      dest[140] = src[404];  /* (125,3) entry. */
      dest[141] = src[545];  /* (126,4) entry. */
      dest[142] = src[532];  /* (113,4) entry. */
      dest[143] = src[533];  /* (114,4) entry. */
      dest[144] = src[126];  /* (127,1) entry. */
      dest[145] = src[127];  /* (128,1) entry. */
      dest[146] = src[268];  /* (129,2) entry. */
      dest[147] = src[409];  /* (130,3) entry. */
      dest[148] = src[550];  /* (131,4) entry. */
      dest[149] = src[271];  /* (132,2) entry. */
      dest[150] = src[412];  /* (133,3) entry. */
      dest[151] = src[553];  /* (134,4) entry. */
      dest[152] = src[274];  /* (135,2) entry. */
      dest[153] = src[415];  /* (136,3) entry. */
      dest[154] = src[556];  /* (137,4) entry. */
      dest[155] = src[277];  /* (138,2) entry. */
      dest[156] = src[418];  /* (139,3) entry. */
      dest[157] = src[559];  /* (140,4) entry. */
      dest[158] = src[546];  /* (127,4) entry. */
      dest[159] = src[547];  /* (128,4) entry. */
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "YBu");
  if (xm == NULL) {
    printf("could not find params.YBu.\n");
  } else {
    if (!((mxGetM(xm) == 140) && (mxGetN(xm) == 1))) {
      printf("YBu must be size (140,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter YBu must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter YBu must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter YBu must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.YBu;
      src = mxGetPr(xm);
      for (i = 0; i < 140; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  if (valid_vars != 2) {
    printf("Error: %d parameters are invalid.\n", 2 - valid_vars);
    mexErrMsgTxt("invalid parameters found.");
  }
  if (prepare_for_c) {
    printf("settings.prepare_for_c == 1. thus, outputting for C.\n");
    for (i = 0; i < 140; i++)
      printf("  params.YBu[%d] = %.6g;\n", i, params.YBu[i]);
    for (i = 0; i < 160; i++)
      printf("  params.CA[%d] = %.6g;\n", i, params.CA[i]);
  }
  /* Perform the actual solve in here. */
  steps = solve();
  /* For profiling purposes, allow extra silent solves if desired. */
  settings.verbose = 0;
  for (i = 0; i < extra_solves; i++)
    solve();
  /* Update the status variables. */
  plhs[1] = mxCreateStructArray(1, dims1x1of1, 4, status_names);
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "optval", xm);
  *mxGetPr(xm) = work.optval;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "gap", xm);
  *mxGetPr(xm) = work.gap;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "steps", xm);
  *mxGetPr(xm) = steps;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "converged", xm);
  *mxGetPr(xm) = work.converged;
  /* Extract variable values. */
  plhs[0] = mxCreateStructArray(1, dims1x1of1, num_var_names, var_names);
  xm = mxCreateDoubleMatrix(4, 1, mxREAL);
  mxSetField(plhs[0], 0, "x", xm);
  dest = mxGetPr(xm);
  src = vars.x;
  for (i = 0; i < 4; i++) {
    *dest++ = *src++;
  }
}
