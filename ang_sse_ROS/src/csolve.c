/* Produced by CVXGEN, 2017-04-11 23:31:07 -0400.  */
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
    if (!((mxGetM(xm) == 240) && (mxGetN(xm) == 6))) {
      printf("CA must be size (240,6), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
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
      dest[1] = src[241];  /* (2,2) entry. */
      dest[2] = src[482];  /* (3,3) entry. */
      dest[3] = src[723];  /* (4,4) entry. */
      dest[4] = src[964];  /* (5,5) entry. */
      dest[5] = src[1205];  /* (6,6) entry. */
      dest[6] = src[6];  /* (7,1) entry. */
      dest[7] = src[247];  /* (8,2) entry. */
      dest[8] = src[488];  /* (9,3) entry. */
      dest[9] = src[729];  /* (10,4) entry. */
      dest[10] = src[970];  /* (11,5) entry. */
      dest[11] = src[1211];  /* (12,6) entry. */
      dest[12] = src[12];  /* (13,1) entry. */
      dest[13] = src[253];  /* (14,2) entry. */
      dest[14] = src[494];  /* (15,3) entry. */
      dest[15] = src[735];  /* (16,4) entry. */
      dest[16] = src[976];  /* (17,5) entry. */
      dest[17] = src[1217];  /* (18,6) entry. */
      dest[18] = src[18];  /* (19,1) entry. */
      dest[19] = src[259];  /* (20,2) entry. */
      dest[20] = src[500];  /* (21,3) entry. */
      dest[21] = src[741];  /* (22,4) entry. */
      dest[22] = src[982];  /* (23,5) entry. */
      dest[23] = src[1223];  /* (24,6) entry. */
      dest[24] = src[720];  /* (1,4) entry. */
      dest[25] = src[961];  /* (2,5) entry. */
      dest[26] = src[1202];  /* (3,6) entry. */
      dest[27] = src[726];  /* (7,4) entry. */
      dest[28] = src[967];  /* (8,5) entry. */
      dest[29] = src[1208];  /* (9,6) entry. */
      dest[30] = src[732];  /* (13,4) entry. */
      dest[31] = src[973];  /* (14,5) entry. */
      dest[32] = src[1214];  /* (15,6) entry. */
      dest[33] = src[738];  /* (19,4) entry. */
      dest[34] = src[979];  /* (20,5) entry. */
      dest[35] = src[1220];  /* (21,6) entry. */
      dest[36] = src[24];  /* (25,1) entry. */
      dest[37] = src[265];  /* (26,2) entry. */
      dest[38] = src[506];  /* (27,3) entry. */
      dest[39] = src[747];  /* (28,4) entry. */
      dest[40] = src[988];  /* (29,5) entry. */
      dest[41] = src[1229];  /* (30,6) entry. */
      dest[42] = src[30];  /* (31,1) entry. */
      dest[43] = src[271];  /* (32,2) entry. */
      dest[44] = src[512];  /* (33,3) entry. */
      dest[45] = src[753];  /* (34,4) entry. */
      dest[46] = src[994];  /* (35,5) entry. */
      dest[47] = src[1235];  /* (36,6) entry. */
      dest[48] = src[36];  /* (37,1) entry. */
      dest[49] = src[277];  /* (38,2) entry. */
      dest[50] = src[518];  /* (39,3) entry. */
      dest[51] = src[759];  /* (40,4) entry. */
      dest[52] = src[1000];  /* (41,5) entry. */
      dest[53] = src[1241];  /* (42,6) entry. */
      dest[54] = src[42];  /* (43,1) entry. */
      dest[55] = src[283];  /* (44,2) entry. */
      dest[56] = src[524];  /* (45,3) entry. */
      dest[57] = src[765];  /* (46,4) entry. */
      dest[58] = src[1006];  /* (47,5) entry. */
      dest[59] = src[1247];  /* (48,6) entry. */
      dest[60] = src[744];  /* (25,4) entry. */
      dest[61] = src[985];  /* (26,5) entry. */
      dest[62] = src[1226];  /* (27,6) entry. */
      dest[63] = src[750];  /* (31,4) entry. */
      dest[64] = src[991];  /* (32,5) entry. */
      dest[65] = src[1232];  /* (33,6) entry. */
      dest[66] = src[756];  /* (37,4) entry. */
      dest[67] = src[997];  /* (38,5) entry. */
      dest[68] = src[1238];  /* (39,6) entry. */
      dest[69] = src[762];  /* (43,4) entry. */
      dest[70] = src[1003];  /* (44,5) entry. */
      dest[71] = src[1244];  /* (45,6) entry. */
      dest[72] = src[48];  /* (49,1) entry. */
      dest[73] = src[289];  /* (50,2) entry. */
      dest[74] = src[530];  /* (51,3) entry. */
      dest[75] = src[771];  /* (52,4) entry. */
      dest[76] = src[1012];  /* (53,5) entry. */
      dest[77] = src[1253];  /* (54,6) entry. */
      dest[78] = src[54];  /* (55,1) entry. */
      dest[79] = src[295];  /* (56,2) entry. */
      dest[80] = src[536];  /* (57,3) entry. */
      dest[81] = src[777];  /* (58,4) entry. */
      dest[82] = src[1018];  /* (59,5) entry. */
      dest[83] = src[1259];  /* (60,6) entry. */
      dest[84] = src[60];  /* (61,1) entry. */
      dest[85] = src[301];  /* (62,2) entry. */
      dest[86] = src[542];  /* (63,3) entry. */
      dest[87] = src[783];  /* (64,4) entry. */
      dest[88] = src[1024];  /* (65,5) entry. */
      dest[89] = src[1265];  /* (66,6) entry. */
      dest[90] = src[66];  /* (67,1) entry. */
      dest[91] = src[307];  /* (68,2) entry. */
      dest[92] = src[548];  /* (69,3) entry. */
      dest[93] = src[789];  /* (70,4) entry. */
      dest[94] = src[1030];  /* (71,5) entry. */
      dest[95] = src[1271];  /* (72,6) entry. */
      dest[96] = src[768];  /* (49,4) entry. */
      dest[97] = src[1009];  /* (50,5) entry. */
      dest[98] = src[1250];  /* (51,6) entry. */
      dest[99] = src[774];  /* (55,4) entry. */
      dest[100] = src[1015];  /* (56,5) entry. */
      dest[101] = src[1256];  /* (57,6) entry. */
      dest[102] = src[780];  /* (61,4) entry. */
      dest[103] = src[1021];  /* (62,5) entry. */
      dest[104] = src[1262];  /* (63,6) entry. */
      dest[105] = src[786];  /* (67,4) entry. */
      dest[106] = src[1027];  /* (68,5) entry. */
      dest[107] = src[1268];  /* (69,6) entry. */
      dest[108] = src[72];  /* (73,1) entry. */
      dest[109] = src[313];  /* (74,2) entry. */
      dest[110] = src[554];  /* (75,3) entry. */
      dest[111] = src[795];  /* (76,4) entry. */
      dest[112] = src[1036];  /* (77,5) entry. */
      dest[113] = src[1277];  /* (78,6) entry. */
      dest[114] = src[78];  /* (79,1) entry. */
      dest[115] = src[319];  /* (80,2) entry. */
      dest[116] = src[560];  /* (81,3) entry. */
      dest[117] = src[801];  /* (82,4) entry. */
      dest[118] = src[1042];  /* (83,5) entry. */
      dest[119] = src[1283];  /* (84,6) entry. */
      dest[120] = src[84];  /* (85,1) entry. */
      dest[121] = src[325];  /* (86,2) entry. */
      dest[122] = src[566];  /* (87,3) entry. */
      dest[123] = src[807];  /* (88,4) entry. */
      dest[124] = src[1048];  /* (89,5) entry. */
      dest[125] = src[1289];  /* (90,6) entry. */
      dest[126] = src[90];  /* (91,1) entry. */
      dest[127] = src[331];  /* (92,2) entry. */
      dest[128] = src[572];  /* (93,3) entry. */
      dest[129] = src[813];  /* (94,4) entry. */
      dest[130] = src[1054];  /* (95,5) entry. */
      dest[131] = src[1295];  /* (96,6) entry. */
      dest[132] = src[792];  /* (73,4) entry. */
      dest[133] = src[1033];  /* (74,5) entry. */
      dest[134] = src[1274];  /* (75,6) entry. */
      dest[135] = src[798];  /* (79,4) entry. */
      dest[136] = src[1039];  /* (80,5) entry. */
      dest[137] = src[1280];  /* (81,6) entry. */
      dest[138] = src[804];  /* (85,4) entry. */
      dest[139] = src[1045];  /* (86,5) entry. */
      dest[140] = src[1286];  /* (87,6) entry. */
      dest[141] = src[810];  /* (91,4) entry. */
      dest[142] = src[1051];  /* (92,5) entry. */
      dest[143] = src[1292];  /* (93,6) entry. */
      dest[144] = src[96];  /* (97,1) entry. */
      dest[145] = src[337];  /* (98,2) entry. */
      dest[146] = src[578];  /* (99,3) entry. */
      dest[147] = src[819];  /* (100,4) entry. */
      dest[148] = src[1060];  /* (101,5) entry. */
      dest[149] = src[1301];  /* (102,6) entry. */
      dest[150] = src[102];  /* (103,1) entry. */
      dest[151] = src[343];  /* (104,2) entry. */
      dest[152] = src[584];  /* (105,3) entry. */
      dest[153] = src[825];  /* (106,4) entry. */
      dest[154] = src[1066];  /* (107,5) entry. */
      dest[155] = src[1307];  /* (108,6) entry. */
      dest[156] = src[108];  /* (109,1) entry. */
      dest[157] = src[349];  /* (110,2) entry. */
      dest[158] = src[590];  /* (111,3) entry. */
      dest[159] = src[831];  /* (112,4) entry. */
      dest[160] = src[1072];  /* (113,5) entry. */
      dest[161] = src[1313];  /* (114,6) entry. */
      dest[162] = src[114];  /* (115,1) entry. */
      dest[163] = src[355];  /* (116,2) entry. */
      dest[164] = src[596];  /* (117,3) entry. */
      dest[165] = src[837];  /* (118,4) entry. */
      dest[166] = src[1078];  /* (119,5) entry. */
      dest[167] = src[1319];  /* (120,6) entry. */
      dest[168] = src[816];  /* (97,4) entry. */
      dest[169] = src[1057];  /* (98,5) entry. */
      dest[170] = src[1298];  /* (99,6) entry. */
      dest[171] = src[822];  /* (103,4) entry. */
      dest[172] = src[1063];  /* (104,5) entry. */
      dest[173] = src[1304];  /* (105,6) entry. */
      dest[174] = src[828];  /* (109,4) entry. */
      dest[175] = src[1069];  /* (110,5) entry. */
      dest[176] = src[1310];  /* (111,6) entry. */
      dest[177] = src[834];  /* (115,4) entry. */
      dest[178] = src[1075];  /* (116,5) entry. */
      dest[179] = src[1316];  /* (117,6) entry. */
      dest[180] = src[120];  /* (121,1) entry. */
      dest[181] = src[361];  /* (122,2) entry. */
      dest[182] = src[602];  /* (123,3) entry. */
      dest[183] = src[843];  /* (124,4) entry. */
      dest[184] = src[1084];  /* (125,5) entry. */
      dest[185] = src[1325];  /* (126,6) entry. */
      dest[186] = src[126];  /* (127,1) entry. */
      dest[187] = src[367];  /* (128,2) entry. */
      dest[188] = src[608];  /* (129,3) entry. */
      dest[189] = src[849];  /* (130,4) entry. */
      dest[190] = src[1090];  /* (131,5) entry. */
      dest[191] = src[1331];  /* (132,6) entry. */
      dest[192] = src[132];  /* (133,1) entry. */
      dest[193] = src[373];  /* (134,2) entry. */
      dest[194] = src[614];  /* (135,3) entry. */
      dest[195] = src[855];  /* (136,4) entry. */
      dest[196] = src[1096];  /* (137,5) entry. */
      dest[197] = src[1337];  /* (138,6) entry. */
      dest[198] = src[138];  /* (139,1) entry. */
      dest[199] = src[379];  /* (140,2) entry. */
      dest[200] = src[620];  /* (141,3) entry. */
      dest[201] = src[861];  /* (142,4) entry. */
      dest[202] = src[1102];  /* (143,5) entry. */
      dest[203] = src[1343];  /* (144,6) entry. */
      dest[204] = src[840];  /* (121,4) entry. */
      dest[205] = src[1081];  /* (122,5) entry. */
      dest[206] = src[1322];  /* (123,6) entry. */
      dest[207] = src[846];  /* (127,4) entry. */
      dest[208] = src[1087];  /* (128,5) entry. */
      dest[209] = src[1328];  /* (129,6) entry. */
      dest[210] = src[852];  /* (133,4) entry. */
      dest[211] = src[1093];  /* (134,5) entry. */
      dest[212] = src[1334];  /* (135,6) entry. */
      dest[213] = src[858];  /* (139,4) entry. */
      dest[214] = src[1099];  /* (140,5) entry. */
      dest[215] = src[1340];  /* (141,6) entry. */
      dest[216] = src[144];  /* (145,1) entry. */
      dest[217] = src[385];  /* (146,2) entry. */
      dest[218] = src[626];  /* (147,3) entry. */
      dest[219] = src[867];  /* (148,4) entry. */
      dest[220] = src[1108];  /* (149,5) entry. */
      dest[221] = src[1349];  /* (150,6) entry. */
      dest[222] = src[150];  /* (151,1) entry. */
      dest[223] = src[391];  /* (152,2) entry. */
      dest[224] = src[632];  /* (153,3) entry. */
      dest[225] = src[873];  /* (154,4) entry. */
      dest[226] = src[1114];  /* (155,5) entry. */
      dest[227] = src[1355];  /* (156,6) entry. */
      dest[228] = src[156];  /* (157,1) entry. */
      dest[229] = src[397];  /* (158,2) entry. */
      dest[230] = src[638];  /* (159,3) entry. */
      dest[231] = src[879];  /* (160,4) entry. */
      dest[232] = src[1120];  /* (161,5) entry. */
      dest[233] = src[1361];  /* (162,6) entry. */
      dest[234] = src[162];  /* (163,1) entry. */
      dest[235] = src[403];  /* (164,2) entry. */
      dest[236] = src[644];  /* (165,3) entry. */
      dest[237] = src[885];  /* (166,4) entry. */
      dest[238] = src[1126];  /* (167,5) entry. */
      dest[239] = src[1367];  /* (168,6) entry. */
      dest[240] = src[864];  /* (145,4) entry. */
      dest[241] = src[1105];  /* (146,5) entry. */
      dest[242] = src[1346];  /* (147,6) entry. */
      dest[243] = src[870];  /* (151,4) entry. */
      dest[244] = src[1111];  /* (152,5) entry. */
      dest[245] = src[1352];  /* (153,6) entry. */
      dest[246] = src[876];  /* (157,4) entry. */
      dest[247] = src[1117];  /* (158,5) entry. */
      dest[248] = src[1358];  /* (159,6) entry. */
      dest[249] = src[882];  /* (163,4) entry. */
      dest[250] = src[1123];  /* (164,5) entry. */
      dest[251] = src[1364];  /* (165,6) entry. */
      dest[252] = src[168];  /* (169,1) entry. */
      dest[253] = src[409];  /* (170,2) entry. */
      dest[254] = src[650];  /* (171,3) entry. */
      dest[255] = src[891];  /* (172,4) entry. */
      dest[256] = src[1132];  /* (173,5) entry. */
      dest[257] = src[1373];  /* (174,6) entry. */
      dest[258] = src[174];  /* (175,1) entry. */
      dest[259] = src[415];  /* (176,2) entry. */
      dest[260] = src[656];  /* (177,3) entry. */
      dest[261] = src[897];  /* (178,4) entry. */
      dest[262] = src[1138];  /* (179,5) entry. */
      dest[263] = src[1379];  /* (180,6) entry. */
      dest[264] = src[180];  /* (181,1) entry. */
      dest[265] = src[421];  /* (182,2) entry. */
      dest[266] = src[662];  /* (183,3) entry. */
      dest[267] = src[903];  /* (184,4) entry. */
      dest[268] = src[1144];  /* (185,5) entry. */
      dest[269] = src[1385];  /* (186,6) entry. */
      dest[270] = src[186];  /* (187,1) entry. */
      dest[271] = src[427];  /* (188,2) entry. */
      dest[272] = src[668];  /* (189,3) entry. */
      dest[273] = src[909];  /* (190,4) entry. */
      dest[274] = src[1150];  /* (191,5) entry. */
      dest[275] = src[1391];  /* (192,6) entry. */
      dest[276] = src[888];  /* (169,4) entry. */
      dest[277] = src[1129];  /* (170,5) entry. */
      dest[278] = src[1370];  /* (171,6) entry. */
      dest[279] = src[894];  /* (175,4) entry. */
      dest[280] = src[1135];  /* (176,5) entry. */
      dest[281] = src[1376];  /* (177,6) entry. */
      dest[282] = src[900];  /* (181,4) entry. */
      dest[283] = src[1141];  /* (182,5) entry. */
      dest[284] = src[1382];  /* (183,6) entry. */
      dest[285] = src[906];  /* (187,4) entry. */
      dest[286] = src[1147];  /* (188,5) entry. */
      dest[287] = src[1388];  /* (189,6) entry. */
      dest[288] = src[192];  /* (193,1) entry. */
      dest[289] = src[433];  /* (194,2) entry. */
      dest[290] = src[674];  /* (195,3) entry. */
      dest[291] = src[915];  /* (196,4) entry. */
      dest[292] = src[1156];  /* (197,5) entry. */
      dest[293] = src[1397];  /* (198,6) entry. */
      dest[294] = src[198];  /* (199,1) entry. */
      dest[295] = src[439];  /* (200,2) entry. */
      dest[296] = src[680];  /* (201,3) entry. */
      dest[297] = src[921];  /* (202,4) entry. */
      dest[298] = src[1162];  /* (203,5) entry. */
      dest[299] = src[1403];  /* (204,6) entry. */
      dest[300] = src[204];  /* (205,1) entry. */
      dest[301] = src[445];  /* (206,2) entry. */
      dest[302] = src[686];  /* (207,3) entry. */
      dest[303] = src[927];  /* (208,4) entry. */
      dest[304] = src[1168];  /* (209,5) entry. */
      dest[305] = src[1409];  /* (210,6) entry. */
      dest[306] = src[210];  /* (211,1) entry. */
      dest[307] = src[451];  /* (212,2) entry. */
      dest[308] = src[692];  /* (213,3) entry. */
      dest[309] = src[933];  /* (214,4) entry. */
      dest[310] = src[1174];  /* (215,5) entry. */
      dest[311] = src[1415];  /* (216,6) entry. */
      dest[312] = src[912];  /* (193,4) entry. */
      dest[313] = src[1153];  /* (194,5) entry. */
      dest[314] = src[1394];  /* (195,6) entry. */
      dest[315] = src[918];  /* (199,4) entry. */
      dest[316] = src[1159];  /* (200,5) entry. */
      dest[317] = src[1400];  /* (201,6) entry. */
      dest[318] = src[924];  /* (205,4) entry. */
      dest[319] = src[1165];  /* (206,5) entry. */
      dest[320] = src[1406];  /* (207,6) entry. */
      dest[321] = src[930];  /* (211,4) entry. */
      dest[322] = src[1171];  /* (212,5) entry. */
      dest[323] = src[1412];  /* (213,6) entry. */
      dest[324] = src[216];  /* (217,1) entry. */
      dest[325] = src[457];  /* (218,2) entry. */
      dest[326] = src[698];  /* (219,3) entry. */
      dest[327] = src[939];  /* (220,4) entry. */
      dest[328] = src[1180];  /* (221,5) entry. */
      dest[329] = src[1421];  /* (222,6) entry. */
      dest[330] = src[222];  /* (223,1) entry. */
      dest[331] = src[463];  /* (224,2) entry. */
      dest[332] = src[704];  /* (225,3) entry. */
      dest[333] = src[945];  /* (226,4) entry. */
      dest[334] = src[1186];  /* (227,5) entry. */
      dest[335] = src[1427];  /* (228,6) entry. */
      dest[336] = src[228];  /* (229,1) entry. */
      dest[337] = src[469];  /* (230,2) entry. */
      dest[338] = src[710];  /* (231,3) entry. */
      dest[339] = src[951];  /* (232,4) entry. */
      dest[340] = src[1192];  /* (233,5) entry. */
      dest[341] = src[1433];  /* (234,6) entry. */
      dest[342] = src[234];  /* (235,1) entry. */
      dest[343] = src[475];  /* (236,2) entry. */
      dest[344] = src[716];  /* (237,3) entry. */
      dest[345] = src[957];  /* (238,4) entry. */
      dest[346] = src[1198];  /* (239,5) entry. */
      dest[347] = src[1439];  /* (240,6) entry. */
      dest[348] = src[936];  /* (217,4) entry. */
      dest[349] = src[1177];  /* (218,5) entry. */
      dest[350] = src[1418];  /* (219,6) entry. */
      dest[351] = src[942];  /* (223,4) entry. */
      dest[352] = src[1183];  /* (224,5) entry. */
      dest[353] = src[1424];  /* (225,6) entry. */
      dest[354] = src[948];  /* (229,4) entry. */
      dest[355] = src[1189];  /* (230,5) entry. */
      dest[356] = src[1430];  /* (231,6) entry. */
      dest[357] = src[954];  /* (235,4) entry. */
      dest[358] = src[1195];  /* (236,5) entry. */
      dest[359] = src[1436];  /* (237,6) entry. */
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "YBu");
  if (xm == NULL) {
    printf("could not find params.YBu.\n");
  } else {
    if (!((mxGetM(xm) == 240) && (mxGetN(xm) == 1))) {
      printf("YBu must be size (240,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
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
      for (i = 0; i < 240; i++)
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
    for (i = 0; i < 240; i++)
      printf("  params.YBu[%d] = %.6g;\n", i, params.YBu[i]);
    for (i = 0; i < 360; i++)
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
  xm = mxCreateDoubleMatrix(6, 1, mxREAL);
  mxSetField(plhs[0], 0, "x", xm);
  dest = mxGetPr(xm);
  src = vars.x;
  for (i = 0; i < 6; i++) {
    *dest++ = *src++;
  }
}
