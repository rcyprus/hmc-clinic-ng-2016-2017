/* Produced by CVXGEN, 2017-03-26 23:17:01 -0400.  */
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
      dest[1] = src[1];  /* (2,1) entry. */
      dest[2] = src[2];  /* (3,1) entry. */
      dest[3] = src[3];  /* (4,1) entry. */
      dest[4] = src[244];  /* (5,2) entry. */
      dest[5] = src[245];  /* (6,2) entry. */
      dest[6] = src[246];  /* (7,2) entry. */
      dest[7] = src[247];  /* (8,2) entry. */
      dest[8] = src[488];  /* (9,3) entry. */
      dest[9] = src[489];  /* (10,3) entry. */
      dest[10] = src[490];  /* (11,3) entry. */
      dest[11] = src[491];  /* (12,3) entry. */
      dest[12] = src[732];  /* (13,4) entry. */
      dest[13] = src[733];  /* (14,4) entry. */
      dest[14] = src[734];  /* (15,4) entry. */
      dest[15] = src[735];  /* (16,4) entry. */
      dest[16] = src[976];  /* (17,5) entry. */
      dest[17] = src[977];  /* (18,5) entry. */
      dest[18] = src[978];  /* (19,5) entry. */
      dest[19] = src[979];  /* (20,5) entry. */
      dest[20] = src[1220];  /* (21,6) entry. */
      dest[21] = src[1221];  /* (22,6) entry. */
      dest[22] = src[1222];  /* (23,6) entry. */
      dest[23] = src[1223];  /* (24,6) entry. */
      dest[24] = src[720];  /* (1,4) entry. */
      dest[25] = src[721];  /* (2,4) entry. */
      dest[26] = src[722];  /* (3,4) entry. */
      dest[27] = src[723];  /* (4,4) entry. */
      dest[28] = src[964];  /* (5,5) entry. */
      dest[29] = src[965];  /* (6,5) entry. */
      dest[30] = src[966];  /* (7,5) entry. */
      dest[31] = src[967];  /* (8,5) entry. */
      dest[32] = src[1208];  /* (9,6) entry. */
      dest[33] = src[1209];  /* (10,6) entry. */
      dest[34] = src[1210];  /* (11,6) entry. */
      dest[35] = src[1211];  /* (12,6) entry. */
      dest[36] = src[24];  /* (25,1) entry. */
      dest[37] = src[25];  /* (26,1) entry. */
      dest[38] = src[26];  /* (27,1) entry. */
      dest[39] = src[27];  /* (28,1) entry. */
      dest[40] = src[268];  /* (29,2) entry. */
      dest[41] = src[269];  /* (30,2) entry. */
      dest[42] = src[270];  /* (31,2) entry. */
      dest[43] = src[271];  /* (32,2) entry. */
      dest[44] = src[512];  /* (33,3) entry. */
      dest[45] = src[513];  /* (34,3) entry. */
      dest[46] = src[514];  /* (35,3) entry. */
      dest[47] = src[515];  /* (36,3) entry. */
      dest[48] = src[756];  /* (37,4) entry. */
      dest[49] = src[757];  /* (38,4) entry. */
      dest[50] = src[758];  /* (39,4) entry. */
      dest[51] = src[759];  /* (40,4) entry. */
      dest[52] = src[1000];  /* (41,5) entry. */
      dest[53] = src[1001];  /* (42,5) entry. */
      dest[54] = src[1002];  /* (43,5) entry. */
      dest[55] = src[1003];  /* (44,5) entry. */
      dest[56] = src[1244];  /* (45,6) entry. */
      dest[57] = src[1245];  /* (46,6) entry. */
      dest[58] = src[1246];  /* (47,6) entry. */
      dest[59] = src[1247];  /* (48,6) entry. */
      dest[60] = src[744];  /* (25,4) entry. */
      dest[61] = src[745];  /* (26,4) entry. */
      dest[62] = src[746];  /* (27,4) entry. */
      dest[63] = src[747];  /* (28,4) entry. */
      dest[64] = src[988];  /* (29,5) entry. */
      dest[65] = src[989];  /* (30,5) entry. */
      dest[66] = src[990];  /* (31,5) entry. */
      dest[67] = src[991];  /* (32,5) entry. */
      dest[68] = src[1232];  /* (33,6) entry. */
      dest[69] = src[1233];  /* (34,6) entry. */
      dest[70] = src[1234];  /* (35,6) entry. */
      dest[71] = src[1235];  /* (36,6) entry. */
      dest[72] = src[48];  /* (49,1) entry. */
      dest[73] = src[49];  /* (50,1) entry. */
      dest[74] = src[50];  /* (51,1) entry. */
      dest[75] = src[51];  /* (52,1) entry. */
      dest[76] = src[292];  /* (53,2) entry. */
      dest[77] = src[293];  /* (54,2) entry. */
      dest[78] = src[294];  /* (55,2) entry. */
      dest[79] = src[295];  /* (56,2) entry. */
      dest[80] = src[536];  /* (57,3) entry. */
      dest[81] = src[537];  /* (58,3) entry. */
      dest[82] = src[538];  /* (59,3) entry. */
      dest[83] = src[539];  /* (60,3) entry. */
      dest[84] = src[780];  /* (61,4) entry. */
      dest[85] = src[781];  /* (62,4) entry. */
      dest[86] = src[782];  /* (63,4) entry. */
      dest[87] = src[783];  /* (64,4) entry. */
      dest[88] = src[1024];  /* (65,5) entry. */
      dest[89] = src[1025];  /* (66,5) entry. */
      dest[90] = src[1026];  /* (67,5) entry. */
      dest[91] = src[1027];  /* (68,5) entry. */
      dest[92] = src[1268];  /* (69,6) entry. */
      dest[93] = src[1269];  /* (70,6) entry. */
      dest[94] = src[1270];  /* (71,6) entry. */
      dest[95] = src[1271];  /* (72,6) entry. */
      dest[96] = src[768];  /* (49,4) entry. */
      dest[97] = src[769];  /* (50,4) entry. */
      dest[98] = src[770];  /* (51,4) entry. */
      dest[99] = src[771];  /* (52,4) entry. */
      dest[100] = src[1012];  /* (53,5) entry. */
      dest[101] = src[1013];  /* (54,5) entry. */
      dest[102] = src[1014];  /* (55,5) entry. */
      dest[103] = src[1015];  /* (56,5) entry. */
      dest[104] = src[1256];  /* (57,6) entry. */
      dest[105] = src[1257];  /* (58,6) entry. */
      dest[106] = src[1258];  /* (59,6) entry. */
      dest[107] = src[1259];  /* (60,6) entry. */
      dest[108] = src[72];  /* (73,1) entry. */
      dest[109] = src[73];  /* (74,1) entry. */
      dest[110] = src[74];  /* (75,1) entry. */
      dest[111] = src[75];  /* (76,1) entry. */
      dest[112] = src[316];  /* (77,2) entry. */
      dest[113] = src[317];  /* (78,2) entry. */
      dest[114] = src[318];  /* (79,2) entry. */
      dest[115] = src[319];  /* (80,2) entry. */
      dest[116] = src[560];  /* (81,3) entry. */
      dest[117] = src[561];  /* (82,3) entry. */
      dest[118] = src[562];  /* (83,3) entry. */
      dest[119] = src[563];  /* (84,3) entry. */
      dest[120] = src[804];  /* (85,4) entry. */
      dest[121] = src[805];  /* (86,4) entry. */
      dest[122] = src[806];  /* (87,4) entry. */
      dest[123] = src[807];  /* (88,4) entry. */
      dest[124] = src[1048];  /* (89,5) entry. */
      dest[125] = src[1049];  /* (90,5) entry. */
      dest[126] = src[1050];  /* (91,5) entry. */
      dest[127] = src[1051];  /* (92,5) entry. */
      dest[128] = src[1292];  /* (93,6) entry. */
      dest[129] = src[1293];  /* (94,6) entry. */
      dest[130] = src[1294];  /* (95,6) entry. */
      dest[131] = src[1295];  /* (96,6) entry. */
      dest[132] = src[792];  /* (73,4) entry. */
      dest[133] = src[793];  /* (74,4) entry. */
      dest[134] = src[794];  /* (75,4) entry. */
      dest[135] = src[795];  /* (76,4) entry. */
      dest[136] = src[1036];  /* (77,5) entry. */
      dest[137] = src[1037];  /* (78,5) entry. */
      dest[138] = src[1038];  /* (79,5) entry. */
      dest[139] = src[1039];  /* (80,5) entry. */
      dest[140] = src[1280];  /* (81,6) entry. */
      dest[141] = src[1281];  /* (82,6) entry. */
      dest[142] = src[1282];  /* (83,6) entry. */
      dest[143] = src[1283];  /* (84,6) entry. */
      dest[144] = src[96];  /* (97,1) entry. */
      dest[145] = src[97];  /* (98,1) entry. */
      dest[146] = src[98];  /* (99,1) entry. */
      dest[147] = src[99];  /* (100,1) entry. */
      dest[148] = src[340];  /* (101,2) entry. */
      dest[149] = src[341];  /* (102,2) entry. */
      dest[150] = src[342];  /* (103,2) entry. */
      dest[151] = src[343];  /* (104,2) entry. */
      dest[152] = src[584];  /* (105,3) entry. */
      dest[153] = src[585];  /* (106,3) entry. */
      dest[154] = src[586];  /* (107,3) entry. */
      dest[155] = src[587];  /* (108,3) entry. */
      dest[156] = src[828];  /* (109,4) entry. */
      dest[157] = src[829];  /* (110,4) entry. */
      dest[158] = src[830];  /* (111,4) entry. */
      dest[159] = src[831];  /* (112,4) entry. */
      dest[160] = src[1072];  /* (113,5) entry. */
      dest[161] = src[1073];  /* (114,5) entry. */
      dest[162] = src[1074];  /* (115,5) entry. */
      dest[163] = src[1075];  /* (116,5) entry. */
      dest[164] = src[1316];  /* (117,6) entry. */
      dest[165] = src[1317];  /* (118,6) entry. */
      dest[166] = src[1318];  /* (119,6) entry. */
      dest[167] = src[1319];  /* (120,6) entry. */
      dest[168] = src[816];  /* (97,4) entry. */
      dest[169] = src[817];  /* (98,4) entry. */
      dest[170] = src[818];  /* (99,4) entry. */
      dest[171] = src[819];  /* (100,4) entry. */
      dest[172] = src[1060];  /* (101,5) entry. */
      dest[173] = src[1061];  /* (102,5) entry. */
      dest[174] = src[1062];  /* (103,5) entry. */
      dest[175] = src[1063];  /* (104,5) entry. */
      dest[176] = src[1304];  /* (105,6) entry. */
      dest[177] = src[1305];  /* (106,6) entry. */
      dest[178] = src[1306];  /* (107,6) entry. */
      dest[179] = src[1307];  /* (108,6) entry. */
      dest[180] = src[120];  /* (121,1) entry. */
      dest[181] = src[121];  /* (122,1) entry. */
      dest[182] = src[122];  /* (123,1) entry. */
      dest[183] = src[123];  /* (124,1) entry. */
      dest[184] = src[364];  /* (125,2) entry. */
      dest[185] = src[365];  /* (126,2) entry. */
      dest[186] = src[366];  /* (127,2) entry. */
      dest[187] = src[367];  /* (128,2) entry. */
      dest[188] = src[608];  /* (129,3) entry. */
      dest[189] = src[609];  /* (130,3) entry. */
      dest[190] = src[610];  /* (131,3) entry. */
      dest[191] = src[611];  /* (132,3) entry. */
      dest[192] = src[852];  /* (133,4) entry. */
      dest[193] = src[853];  /* (134,4) entry. */
      dest[194] = src[854];  /* (135,4) entry. */
      dest[195] = src[855];  /* (136,4) entry. */
      dest[196] = src[1096];  /* (137,5) entry. */
      dest[197] = src[1097];  /* (138,5) entry. */
      dest[198] = src[1098];  /* (139,5) entry. */
      dest[199] = src[1099];  /* (140,5) entry. */
      dest[200] = src[1340];  /* (141,6) entry. */
      dest[201] = src[1341];  /* (142,6) entry. */
      dest[202] = src[1342];  /* (143,6) entry. */
      dest[203] = src[1343];  /* (144,6) entry. */
      dest[204] = src[840];  /* (121,4) entry. */
      dest[205] = src[841];  /* (122,4) entry. */
      dest[206] = src[842];  /* (123,4) entry. */
      dest[207] = src[843];  /* (124,4) entry. */
      dest[208] = src[1084];  /* (125,5) entry. */
      dest[209] = src[1085];  /* (126,5) entry. */
      dest[210] = src[1086];  /* (127,5) entry. */
      dest[211] = src[1087];  /* (128,5) entry. */
      dest[212] = src[1328];  /* (129,6) entry. */
      dest[213] = src[1329];  /* (130,6) entry. */
      dest[214] = src[1330];  /* (131,6) entry. */
      dest[215] = src[1331];  /* (132,6) entry. */
      dest[216] = src[144];  /* (145,1) entry. */
      dest[217] = src[145];  /* (146,1) entry. */
      dest[218] = src[146];  /* (147,1) entry. */
      dest[219] = src[147];  /* (148,1) entry. */
      dest[220] = src[388];  /* (149,2) entry. */
      dest[221] = src[389];  /* (150,2) entry. */
      dest[222] = src[390];  /* (151,2) entry. */
      dest[223] = src[391];  /* (152,2) entry. */
      dest[224] = src[632];  /* (153,3) entry. */
      dest[225] = src[633];  /* (154,3) entry. */
      dest[226] = src[634];  /* (155,3) entry. */
      dest[227] = src[635];  /* (156,3) entry. */
      dest[228] = src[876];  /* (157,4) entry. */
      dest[229] = src[877];  /* (158,4) entry. */
      dest[230] = src[878];  /* (159,4) entry. */
      dest[231] = src[879];  /* (160,4) entry. */
      dest[232] = src[1120];  /* (161,5) entry. */
      dest[233] = src[1121];  /* (162,5) entry. */
      dest[234] = src[1122];  /* (163,5) entry. */
      dest[235] = src[1123];  /* (164,5) entry. */
      dest[236] = src[1364];  /* (165,6) entry. */
      dest[237] = src[1365];  /* (166,6) entry. */
      dest[238] = src[1366];  /* (167,6) entry. */
      dest[239] = src[1367];  /* (168,6) entry. */
      dest[240] = src[864];  /* (145,4) entry. */
      dest[241] = src[865];  /* (146,4) entry. */
      dest[242] = src[866];  /* (147,4) entry. */
      dest[243] = src[867];  /* (148,4) entry. */
      dest[244] = src[1108];  /* (149,5) entry. */
      dest[245] = src[1109];  /* (150,5) entry. */
      dest[246] = src[1110];  /* (151,5) entry. */
      dest[247] = src[1111];  /* (152,5) entry. */
      dest[248] = src[1352];  /* (153,6) entry. */
      dest[249] = src[1353];  /* (154,6) entry. */
      dest[250] = src[1354];  /* (155,6) entry. */
      dest[251] = src[1355];  /* (156,6) entry. */
      dest[252] = src[168];  /* (169,1) entry. */
      dest[253] = src[169];  /* (170,1) entry. */
      dest[254] = src[170];  /* (171,1) entry. */
      dest[255] = src[171];  /* (172,1) entry. */
      dest[256] = src[412];  /* (173,2) entry. */
      dest[257] = src[413];  /* (174,2) entry. */
      dest[258] = src[414];  /* (175,2) entry. */
      dest[259] = src[415];  /* (176,2) entry. */
      dest[260] = src[656];  /* (177,3) entry. */
      dest[261] = src[657];  /* (178,3) entry. */
      dest[262] = src[658];  /* (179,3) entry. */
      dest[263] = src[659];  /* (180,3) entry. */
      dest[264] = src[900];  /* (181,4) entry. */
      dest[265] = src[901];  /* (182,4) entry. */
      dest[266] = src[902];  /* (183,4) entry. */
      dest[267] = src[903];  /* (184,4) entry. */
      dest[268] = src[1144];  /* (185,5) entry. */
      dest[269] = src[1145];  /* (186,5) entry. */
      dest[270] = src[1146];  /* (187,5) entry. */
      dest[271] = src[1147];  /* (188,5) entry. */
      dest[272] = src[1388];  /* (189,6) entry. */
      dest[273] = src[1389];  /* (190,6) entry. */
      dest[274] = src[1390];  /* (191,6) entry. */
      dest[275] = src[1391];  /* (192,6) entry. */
      dest[276] = src[888];  /* (169,4) entry. */
      dest[277] = src[889];  /* (170,4) entry. */
      dest[278] = src[890];  /* (171,4) entry. */
      dest[279] = src[891];  /* (172,4) entry. */
      dest[280] = src[1132];  /* (173,5) entry. */
      dest[281] = src[1133];  /* (174,5) entry. */
      dest[282] = src[1134];  /* (175,5) entry. */
      dest[283] = src[1135];  /* (176,5) entry. */
      dest[284] = src[1376];  /* (177,6) entry. */
      dest[285] = src[1377];  /* (178,6) entry. */
      dest[286] = src[1378];  /* (179,6) entry. */
      dest[287] = src[1379];  /* (180,6) entry. */
      dest[288] = src[192];  /* (193,1) entry. */
      dest[289] = src[193];  /* (194,1) entry. */
      dest[290] = src[194];  /* (195,1) entry. */
      dest[291] = src[195];  /* (196,1) entry. */
      dest[292] = src[436];  /* (197,2) entry. */
      dest[293] = src[437];  /* (198,2) entry. */
      dest[294] = src[438];  /* (199,2) entry. */
      dest[295] = src[439];  /* (200,2) entry. */
      dest[296] = src[680];  /* (201,3) entry. */
      dest[297] = src[681];  /* (202,3) entry. */
      dest[298] = src[682];  /* (203,3) entry. */
      dest[299] = src[683];  /* (204,3) entry. */
      dest[300] = src[924];  /* (205,4) entry. */
      dest[301] = src[925];  /* (206,4) entry. */
      dest[302] = src[926];  /* (207,4) entry. */
      dest[303] = src[927];  /* (208,4) entry. */
      dest[304] = src[1168];  /* (209,5) entry. */
      dest[305] = src[1169];  /* (210,5) entry. */
      dest[306] = src[1170];  /* (211,5) entry. */
      dest[307] = src[1171];  /* (212,5) entry. */
      dest[308] = src[1412];  /* (213,6) entry. */
      dest[309] = src[1413];  /* (214,6) entry. */
      dest[310] = src[1414];  /* (215,6) entry. */
      dest[311] = src[1415];  /* (216,6) entry. */
      dest[312] = src[912];  /* (193,4) entry. */
      dest[313] = src[913];  /* (194,4) entry. */
      dest[314] = src[914];  /* (195,4) entry. */
      dest[315] = src[915];  /* (196,4) entry. */
      dest[316] = src[1156];  /* (197,5) entry. */
      dest[317] = src[1157];  /* (198,5) entry. */
      dest[318] = src[1158];  /* (199,5) entry. */
      dest[319] = src[1159];  /* (200,5) entry. */
      dest[320] = src[1400];  /* (201,6) entry. */
      dest[321] = src[1401];  /* (202,6) entry. */
      dest[322] = src[1402];  /* (203,6) entry. */
      dest[323] = src[1403];  /* (204,6) entry. */
      dest[324] = src[216];  /* (217,1) entry. */
      dest[325] = src[217];  /* (218,1) entry. */
      dest[326] = src[218];  /* (219,1) entry. */
      dest[327] = src[219];  /* (220,1) entry. */
      dest[328] = src[460];  /* (221,2) entry. */
      dest[329] = src[461];  /* (222,2) entry. */
      dest[330] = src[462];  /* (223,2) entry. */
      dest[331] = src[463];  /* (224,2) entry. */
      dest[332] = src[704];  /* (225,3) entry. */
      dest[333] = src[705];  /* (226,3) entry. */
      dest[334] = src[706];  /* (227,3) entry. */
      dest[335] = src[707];  /* (228,3) entry. */
      dest[336] = src[948];  /* (229,4) entry. */
      dest[337] = src[949];  /* (230,4) entry. */
      dest[338] = src[950];  /* (231,4) entry. */
      dest[339] = src[951];  /* (232,4) entry. */
      dest[340] = src[1192];  /* (233,5) entry. */
      dest[341] = src[1193];  /* (234,5) entry. */
      dest[342] = src[1194];  /* (235,5) entry. */
      dest[343] = src[1195];  /* (236,5) entry. */
      dest[344] = src[1436];  /* (237,6) entry. */
      dest[345] = src[1437];  /* (238,6) entry. */
      dest[346] = src[1438];  /* (239,6) entry. */
      dest[347] = src[1439];  /* (240,6) entry. */
      dest[348] = src[936];  /* (217,4) entry. */
      dest[349] = src[937];  /* (218,4) entry. */
      dest[350] = src[938];  /* (219,4) entry. */
      dest[351] = src[939];  /* (220,4) entry. */
      dest[352] = src[1180];  /* (221,5) entry. */
      dest[353] = src[1181];  /* (222,5) entry. */
      dest[354] = src[1182];  /* (223,5) entry. */
      dest[355] = src[1183];  /* (224,5) entry. */
      dest[356] = src[1424];  /* (225,6) entry. */
      dest[357] = src[1425];  /* (226,6) entry. */
      dest[358] = src[1426];  /* (227,6) entry. */
      dest[359] = src[1427];  /* (228,6) entry. */
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
