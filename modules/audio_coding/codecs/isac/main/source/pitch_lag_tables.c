/*
 *  Copyright (c) 2011 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/audio_coding/codecs/isac/main/source/pitch_lag_tables.h"
#include "modules/audio_coding/codecs/isac/main/source/settings.h"

/* header file for coding tables for the pitch filter side-info in the entropy coder */
/********************* Pitch Filter Gain Coefficient Tables ************************/

/* tables for use with small pitch gain */

/* cdf for quantized pitch filter lags */
const uint16_t WebRtcIsac_kQPitchLagCdf1Lo[127] = {
 0,  134,  336,  549,  778,  998,  1264,  1512,  1777,  2070,
 2423,  2794,  3051,  3361,  3708,  3979,  4315,  4610,  4933,  5269,
 5575,  5896,  6155,  6480,  6816,  7129,  7477,  7764,  8061,  8358,
 8718,  9020,  9390,  9783,  10177,  10543,  10885,  11342,  11795,  12213,
 12680,  13096,  13524,  13919,  14436,  14903,  15349,  15795,  16267,  16734,
 17266,  17697,  18130,  18632,  19080,  19447,  19884,  20315,  20735,  21288,
 21764,  22264,  22723,  23193,  23680,  24111,  24557,  25022,  25537,  26082,
 26543,  27090,  27620,  28139,  28652,  29149,  29634,  30175,  30692,  31273,
 31866,  32506,  33059,  33650,  34296,  34955,  35629,  36295,  36967,  37726,
 38559,  39458,  40364,  41293,  42256,  43215,  44231,  45253,  46274,  47359,
 48482,  49678,  50810,  51853,  53016,  54148,  55235,  56263,  57282,  58363,
 59288,  60179,  61076,  61806,  62474,  63129,  63656,  64160,  64533,  64856,
 65152,  65535,  65535,  65535,  65535,  65535,  65535};

const uint16_t WebRtcIsac_kQPitchLagCdf2Lo[20] = {
 0,  429,  3558,  5861,  8558,  11639,  15210,  19502,  24773,  31983,
 42602,  48567,  52601,  55676,  58160,  60172,  61889,  63235,  65383,  65535};

const uint16_t WebRtcIsac_kQPitchLagCdf3Lo[2] = {
 0,  65535};

const uint16_t WebRtcIsac_kQPitchLagCdf4Lo[10] = {
 0,  2966,  6368,  11182,  19431,  37793,  48532,  55353,  60626,  65535};

const uint16_t *WebRtcIsac_kQPitchLagCdfPtrLo[4] = {WebRtcIsac_kQPitchLagCdf1Lo, WebRtcIsac_kQPitchLagCdf2Lo, WebRtcIsac_kQPitchLagCdf3Lo, WebRtcIsac_kQPitchLagCdf4Lo};

/* size of first cdf table */
const uint16_t WebRtcIsac_kQPitchLagCdfSizeLo[1] = {128};

/* index limits and ranges */
const int16_t WebRtcIsac_kQIndexLowerLimitLagLo[4] = {
-140, -9,  0, -4};

const int16_t WebRtcIsac_kQIndexUpperLimitLagLo[4] = {
-20,  9,  0,  4};

/* initial index for arithmetic decoder */
const uint16_t WebRtcIsac_kQInitIndexLagLo[3] = {
 10,  1,  5};

/* mean values of pitch filter lags */
const double WebRtcIsac_kQMeanLag2Lo[19] = {
-17.21385070, -15.82678944, -14.07123081, -12.03003877, -10.01311864, -8.00794627, -5.91162987, -3.89231876, -1.90220980, -0.01879275,
 1.89144232,  3.88123171,  5.92146992,  7.96435361,  9.98923648,  11.98266347,  13.96101002,  15.74855713,  17.10976611};

const double WebRtcIsac_kQMeanLag3Lo[1] = {
 0.00000000};

const double WebRtcIsac_kQMeanLag4Lo[9] = {
-7.76246496, -5.92083980, -3.94095226, -1.89502305,  0.03724681,  1.93054221,  3.96443467,  5.91726366,  7.78434291};

const double WebRtcIsac_kQPitchLagStepsizeLo = 2.000000;


/* tables for use with medium pitch gain */

/* cdf for quantized pitch filter lags */
const uint16_t WebRtcIsac_kQPitchLagCdf1Mid[255] = {
 0,  28,  61,  88,  121,  149,  233,  331,  475,  559,
 624,  661,  689,  712,  745,  791,  815,  843,  866,  922,
 959,  1024,  1061,  1117,  1178,  1238,  1280,  1350,  1453,  1513,
 1564,  1625,  1671,  1741,  1788,  1904,  2072,  2421,  2626,  2770,
 2840,  2900,  2942,  3012,  3068,  3115,  3147,  3194,  3254,  3319,
 3366,  3520,  3678,  3780,  3850,  3911,  3957,  4032,  4106,  4185,
 4292,  4474,  4683,  4842,  5019,  5191,  5321,  5428,  5540,  5675,
 5763,  5847,  5959,  6127,  6304,  6564,  6839,  7090,  7263,  7421,
 7556,  7728,  7872,  7984,  8142,  8361,  8580,  8743,  8938,  9227,
 9409,  9539,  9674,  9795,  9930,  10060,  10177,  10382,  10614,  10861,
 11038,  11271,  11415,  11629,  11792,  12044,  12193,  12416,  12574,  12821,
 13007,  13235,  13445,  13654,  13901,  14134,  14488,  15000,  15703,  16285,
 16504,  16797,  17086,  17328,  17579,  17807,  17998,  18268,  18538,  18836,
 19087,  19274,  19474,  19716,  19935,  20270,  20833,  21303,  21532,  21741,
 21978,  22207,  22523,  22770,  23054,  23613,  23943,  24204,  24399,  24651,
 24832,  25074,  25270,  25549,  25759,  26015,  26150,  26424,  26713,  27048,
 27342,  27504,  27681,  27854,  28021,  28207,  28412,  28664,  28859,  29064,
 29278,  29548,  29748,  30107,  30377,  30656,  30856,  31164,  31452,  31755,
 32011,  32328,  32626,  32919,  33319,  33789,  34329,  34925,  35396,  35973,
 36443,  36964,  37551,  38156,  38724,  39357,  40023,  40908,  41587,  42602,
 43924,  45037,  45810,  46597,  47421,  48291,  49092,  50051,  51448,  52719,
 53440,  54241,  54944,  55977,  56676,  57299,  57872,  58389,  59059,  59688,
 60237,  60782,  61094,  61573,  61890,  62290,  62658,  63030,  63217,  63454,
 63622,  63882,  64003,  64273,  64427,  64529,  64581,  64697,  64758,  64902,
 65414,  65535,  65535,  65535,  65535,  65535,  65535,  65535,  65535,  65535,
 65535,  65535,  65535,  65535,  65535};

const uint16_t WebRtcIsac_kQPitchLagCdf2Mid[36] = {
 0,  71,  335,  581,  836,  1039,  1323,  1795,  2258,  2608,
 3005,  3591,  4243,  5344,  7163,  10583,  16848,  28078,  49448,  57007,
 60357,  61850,  62837,  63437,  63872,  64188,  64377,  64614,  64774,  64949,
 65039,  65115,  65223,  65360,  65474,  65535};

const uint16_t WebRtcIsac_kQPitchLagCdf3Mid[2] = {
 0,  65535};

const uint16_t WebRtcIsac_kQPitchLagCdf4Mid[20] = {
 0,  28,  246,  459,  667,  1045,  1523,  2337,  4337,  11347,
 44231,  56709,  60781,  62243,  63161,  63969,  64608,  65062,  65502,  65535};

const uint16_t *WebRtcIsac_kQPitchLagCdfPtrMid[4] = {WebRtcIsac_kQPitchLagCdf1Mid, WebRtcIsac_kQPitchLagCdf2Mid, WebRtcIsac_kQPitchLagCdf3Mid, WebRtcIsac_kQPitchLagCdf4Mid};

/* size of first cdf table */
const uint16_t WebRtcIsac_kQPitchLagCdfSizeMid[1] = {256};

/* index limits and ranges */
const int16_t WebRtcIsac_kQIndexLowerLimitLagMid[4] = {
-280, -17,  0, -9};

const int16_t WebRtcIsac_kQIndexUpperLimitLagMid[4] = {
-40,  17,  0,  9};

/* initial index for arithmetic decoder */
const uint16_t WebRtcIsac_kQInitIndexLagMid[3] = {
 18,  1,  10};

/* mean values of pitch filter lags */
const double WebRtcIsac_kQMeanLag2Mid[35] = {
-16.89183900, -15.86949778, -15.05476653, -14.00664348, -13.02793036, -12.07324237, -11.00542532, -10.11250602, -8.90792971, -8.02474753,
-7.00426767, -5.94055287, -4.98251338, -3.91053158, -2.98820425, -1.93524245, -0.92978085, -0.01722509,  0.91317387,  1.92973955,
 2.96908851,  3.93728974,  4.96308471,  5.92244151,  7.08673497,  8.00993708,  9.04656316,  9.98538742,  10.97851694,  11.94772884,
 13.02426166,  14.00039951,  15.01347042,  15.80758023,  16.94086895};

const double WebRtcIsac_kQMeanLag3Mid[1] = {
 0.00000000};

const double WebRtcIsac_kQMeanLag4Mid[19] = {
-8.60409403, -7.89198395, -7.03450280, -5.86260421, -4.93822322, -3.93078706, -2.91302322, -1.91824007, -0.87003282,  0.02822649,
 0.89951758,  1.87495484,  2.91802604,  3.96874074,  5.06571703,  5.93618227,  7.00520185,  7.88497726,  8.64160364};

const double WebRtcIsac_kQPitchLagStepsizeMid = 1.000000;


/* tables for use with large pitch gain */

/* cdf for quantized pitch filter lags */
const uint16_t WebRtcIsac_kQPitchLagCdf1Hi[511] = {
 0,  7,  18,  33,  69,  105,  156,  228,  315,  612,
 680,  691,  709,  724,  735,  738,  742,  746,  749,  753,
 756,  760,  764,  774,  782,  785,  789,  796,  800,  803,
 807,  814,  818,  822,  829,  832,  847,  854,  858,  869,
 876,  883,  898,  908,  934,  977,  1010,  1050,  1060,  1064,
 1075,  1078,  1086,  1089,  1093,  1104,  1111,  1122,  1133,  1136,
 1151,  1162,  1183,  1209,  1252,  1281,  1339,  1364,  1386,  1401,
 1411,  1415,  1426,  1430,  1433,  1440,  1448,  1455,  1462,  1477,
 1487,  1495,  1502,  1506,  1509,  1516,  1524,  1531,  1535,  1542,
 1553,  1556,  1578,  1589,  1611,  1625,  1639,  1643,  1654,  1665,
 1672,  1687,  1694,  1705,  1708,  1719,  1730,  1744,  1752,  1759,
 1791,  1795,  1820,  1867,  1886,  1915,  1936,  1943,  1965,  1987,
 2041,  2099,  2161,  2175,  2200,  2211,  2226,  2233,  2244,  2251,
 2266,  2280,  2287,  2298,  2309,  2316,  2331,  2342,  2356,  2378,
 2403,  2418,  2447,  2497,  2544,  2602,  2863,  2895,  2903,  2935,
 2950,  2971,  3004,  3011,  3018,  3029,  3040,  3062,  3087,  3127,
 3152,  3170,  3199,  3243,  3293,  3322,  3340,  3377,  3402,  3427,
 3474,  3518,  3543,  3579,  3601,  3637,  3659,  3706,  3731,  3760,
 3818,  3847,  3869,  3901,  3920,  3952,  4068,  4169,  4220,  4271,
 4524,  4571,  4604,  4632,  4672,  4730,  4777,  4806,  4857,  4904,
 4951,  5002,  5031,  5060,  5107,  5150,  5212,  5266,  5331,  5382,
 5432,  5490,  5544,  5610,  5700,  5762,  5812,  5874,  5972,  6022,
 6091,  6163,  6232,  6305,  6402,  6540,  6685,  6880,  7090,  7271,
 7379,  7452,  7542,  7625,  7687,  7770,  7843,  7911,  7966,  8024,
 8096,  8190,  8252,  8320,  8411,  8501,  8585,  8639,  8751,  8842,
 8918,  8986,  9066,  9127,  9203,  9269,  9345,  9406,  9464,  9536,
 9612,  9667,  9735,  9844,  9931,  10036,  10119,  10199,  10260,  10358,
 10441,  10514,  10666,  10734,  10872,  10951,  11053,  11125,  11223,  11324,
 11516,  11664,  11737,  11816,  11892,  12008,  12120,  12200,  12280,  12392,
 12490,  12576,  12685,  12812,  12917,  13003,  13108,  13210,  13300,  13384,
 13470,  13579,  13673,  13771,  13879,  13999,  14136,  14201,  14368,  14614,
 14759,  14867,  14958,  15030,  15121,  15189,  15280,  15385,  15461,  15555,
 15653,  15768,  15884,  15971,  16069,  16145,  16210,  16279,  16380,  16463,
 16539,  16615,  16688,  16818,  16919,  17017,  18041,  18338,  18523,  18649,
 18790,  18917,  19047,  19167,  19315,  19460,  19601,  19731,  19858,  20068,
 20173,  20318,  20466,  20625,  20741,  20911,  21045,  21201,  21396,  21588,
 21816,  22022,  22305,  22547,  22786,  23072,  23322,  23600,  23879,  24168,
 24433,  24769,  25120,  25511,  25895,  26289,  26792,  27219,  27683,  28077,
 28566,  29094,  29546,  29977,  30491,  30991,  31573,  32105,  32594,  33173,
 33788,  34497,  35181,  35833,  36488,  37255,  37921,  38645,  39275,  39894,
 40505,  41167,  41790,  42431,  43096,  43723,  44385,  45134,  45858,  46607,
 47349,  48091,  48768,  49405,  49955,  50555,  51167,  51985,  52611,  53078,
 53494,  53965,  54435,  54996,  55601,  56125,  56563,  56838,  57244,  57566,
 57967,  58297,  58771,  59093,  59419,  59647,  59886,  60143,  60461,  60693,
 60917,  61170,  61416,  61634,  61891,  62122,  62310,  62455,  62632,  62839,
 63103,  63436,  63639,  63805,  63906,  64015,  64192,  64355,  64475,  64558,
 64663,  64742,  64811,  64865,  64916,  64956,  64981,  65025,  65068,  65115,
 65195,  65314,  65419,  65535,  65535,  65535,  65535,  65535,  65535,  65535,
 65535,  65535,  65535,  65535,  65535,  65535,  65535,  65535,  65535,  65535,
 65535,  65535,  65535,  65535,  65535,  65535,  65535,  65535,  65535,  65535,
 65535,  65535,  65535,  65535,  65535,  65535,  65535,  65535,  65535,  65535,
 65535};

const uint16_t WebRtcIsac_kQPitchLagCdf2Hi[68] = {
 0,  7,  11,  22,  37,  52,  56,  59,  81,  85,
 89,  96,  115,  130,  137,  152,  170,  181,  193,  200,
 207,  233,  237,  259,  289,  318,  363,  433,  592,  992,
 1607,  3062,  6149,  12206,  25522,  48368,  58223,  61918,  63640,  64584,
 64943,  65098,  65206,  65268,  65294,  65335,  65350,  65372,  65387,  65402,
 65413,  65420,  65428,  65435,  65439,  65450,  65454,  65468,  65472,  65476,
 65483,  65491,  65498,  65505,  65516,  65520,  65528,  65535};

const uint16_t WebRtcIsac_kQPitchLagCdf3Hi[2] = {
 0,  65535};

const uint16_t WebRtcIsac_kQPitchLagCdf4Hi[35] = {
 0,  7,  19,  30,  41,  48,  63,  74,  82,  96,
 122,  152,  215,  330,  701,  2611,  10931,  48106,  61177,  64341,
 65112,  65238,  65309,  65338,  65364,  65379,  65401,  65427,  65453,  65465,
 65476,  65490,  65509,  65528,  65535};

const uint16_t *WebRtcIsac_kQPitchLagCdfPtrHi[4] = {WebRtcIsac_kQPitchLagCdf1Hi, WebRtcIsac_kQPitchLagCdf2Hi, WebRtcIsac_kQPitchLagCdf3Hi, WebRtcIsac_kQPitchLagCdf4Hi};

/* size of first cdf table */
const uint16_t WebRtcIsac_kQPitchLagCdfSizeHi[1] = {512};

/* index limits and ranges */
const int16_t WebRtcIsac_kQindexLowerLimitLagHi[4] = {
-552, -34,  0, -16};

const int16_t WebRtcIsac_kQindexUpperLimitLagHi[4] = {
-80,  32,  0,  17};

/* initial index for arithmetic decoder */
const uint16_t WebRtcIsac_kQInitIndexLagHi[3] = {
 34,  1,  18};

/* mean values of pitch filter lags */
const double WebRtcIsac_kQMeanLag2Hi[67] = {
-17.07263295, -16.50000000, -15.83966081, -15.55613708, -14.96948007, -14.50000000, -14.00000000, -13.48377986, -13.00000000, -12.50000000,
-11.93199636, -11.44530414, -11.04197641, -10.39910301, -10.15202337, -9.51322461, -8.93357741, -8.46456632, -8.10270672, -7.53751847,
-6.98686404, -6.50000000, -6.08463150, -5.46872991, -5.00864717, -4.50163760, -4.01382410, -3.43856708, -2.96898001, -2.46554810,
-1.96861004, -1.47106701, -0.97197237, -0.46561654, -0.00531409,  0.45767857,  0.96777907,  1.47507903,  1.97740425,  2.46695420,
 3.00695774,  3.47167185,  4.02712538,  4.49280007,  5.01087640,  5.48191963,  6.04916550,  6.51511058,  6.97297819,  7.46565499,
 8.01489405,  8.39912001,  8.91819757,  9.50000000,  10.11654065,  10.50000000,  11.03712583,  11.50000000,  12.00000000,  12.38964346,
 12.89466127,  13.43657881,  13.96013840,  14.46279912,  15.00000000,  15.39412269,  15.96662441};

const double WebRtcIsac_kQMeanLag3Hi[1] = {
 0.00000000};

const double WebRtcIsac_kQMeanLag4Hi[34] = {
-7.98331221, -7.47988769, -7.03626557, -6.52708003, -6.06982173, -5.51856292, -5.05827033, -4.45909878, -3.99125864, -3.45308135,
-3.02328139, -2.47297273, -1.94341995, -1.44699056, -0.93612243, -0.43012406,  0.01120357,  0.44054812,  0.93199883,  1.45669587,
 1.97218322,  2.50187419,  2.98748690,  3.49343202,  4.01660147,  4.50984306,  5.01402683,  5.58936797,  5.91787793,  6.59998900,
 6.85034315,  7.53503316,  7.87711194,  8.53631648};

const double WebRtcIsac_kQPitchLagStepsizeHi = 0.500000;

/* transform matrix */
const double WebRtcIsac_kTransform[4][4] = {
{-0.50000000, -0.50000000, -0.50000000, -0.50000000},
{ 0.67082039,  0.22360680, -0.22360680, -0.67082039},
{ 0.50000000, -0.50000000, -0.50000000,  0.50000000},
{ 0.22360680, -0.67082039,  0.67082039, -0.22360680}};

/* transpose transform matrix */
const double WebRtcIsac_kTransformTranspose[4][4] = {
{-0.50000000,  0.67082039,  0.50000000,  0.22360680},
{-0.50000000,  0.22360680, -0.50000000, -0.67082039},
{-0.50000000, -0.22360680, -0.50000000,  0.67082039},
{-0.50000000, -0.67082039,  0.50000000, -0.22360680}};

