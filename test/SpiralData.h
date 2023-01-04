#ifndef VARIABLE_CURVATURE_ROBOT_SPIRALDATA_H
#define VARIABLE_CURVATURE_ROBOT_SPIRALDATA_H

#include <vector>
#include <array>

std::vector<std::array<double, 3>> pathpoints = {
       {-0.5440, -0.8391,   5.0000},
       {-0.5894, -0.7993,   5.0286},
       {-0.6348, -0.7595,   5.0571},
       {-0.6801, -0.7197,   5.0857},
       {-0.7255, -0.6799,   5.1142},
       {-0.7709, -0.6401,   5.1428},
       {-0.8163, -0.6003,   5.1714},
       {-0.8616, -0.5605,   5.1999},
       {-0.9070, -0.5207,   5.2285},
       {-0.9524, -0.4809,   5.2570},
       {-0.9977, -0.4411,   5.2856},
       {-1.0431, -0.4013,   5.3142},
       {-1.0568, -0.3358,   5.3427},
       {-1.0704, -0.2704,   5.3713},
       {-1.0841, -0.2049,   5.3998},
       {-1.0978, -0.1394,   5.4284},
       {-1.1114, -0.0740,   5.4570},
       {-1.1251, -0.0085,   5.4855},
       {-1.1388,  0.0570,   5.5141},
       {-1.1524,  0.1224,   5.5426},
       {-1.1661,  0.1879,   5.5712},
       {-1.1798,  0.2534,   5.5998},
       {-1.1934,  0.3188,   5.6283},
       {-1.1624,  0.3854,   5.6569},
       {-1.1314,  0.4519,   5.6854},
       {-1.1004,  0.5185,   5.7140},
       {-1.0694,  0.5850,   5.7426},
       {-1.0383,  0.6516,   5.7711},
       {-1.0073,  0.7181,   5.7997},
       {-0.9763,  0.7846,   5.8282},
       {-0.9453,  0.8512,   5.8568},
       {-0.9142,  0.9177,   5.8854},
       {-0.8832,  0.9843,   5.9139},
       {-0.8522,  1.0508,   5.9425},
       {-0.7818,  1.0888,   5.9710},
       {-0.7115,  1.1268,   5.9996},
       {-0.6411,  1.1647,   6.0282},
       {-0.5707,  1.2027,   6.0567},
       {-0.5003,  1.2407,   6.0853},
       {-0.4299,  1.2786,   6.1138},
       {-0.3596,  1.3166,   6.1424},
       {-0.2892,  1.3546,   6.1710},
       {-0.2188,  1.3926,   6.1995},
       {-0.1484,  1.4305,   6.2281},
       {-0.0780,  1.4685,   6.2566},
       {0.0076 , 1.4562 ,  6.2852 },
       {0.0932 , 1.4438 ,  6.3138 },
       {0.1789 , 1.4315 ,  6.3423 },
       {0.2645 , 1.4191 ,  6.3709 },
       {0.3502 , 1.4068 ,  6.3994 },
       {0.4358 , 1.3944 ,  6.4280 },
       {0.5215 , 1.3821 ,  6.4566 },
       {0.6071 , 1.3697 ,  6.4851 },
       {0.6927 , 1.3573 ,  6.5137 },
       {0.7784 , 1.3450 ,  6.5422 },
       {0.8640 , 1.3326 ,  6.5708 },
       {0.9302 , 1.2672 ,  6.5994 },
       {0.9964 , 1.2017 ,  6.6279 },
       {1.0626 , 1.1362 ,  6.6565 },
       {1.1288 , 1.0708 ,  6.6850 },
       {1.1950 , 1.0053 ,  6.7136 },
       {1.2612 , 0.9398 ,  6.7422 },
       {1.3274 , 0.8744 ,  6.7707 },
       {1.3936 , 0.8089 ,  6.7993 },
       {1.4597 , 0.7434 ,  6.8278 },
       {1.5259 , 0.6780 ,  6.8564 },
       {1.5921 , 0.6125 ,  6.8850 },
       {1.6075 , 0.5140 ,  6.9135 },
       {1.6230 , 0.4156 ,  6.9421 },
       {1.6384 , 0.3171 ,  6.9706 },
       {1.6538 , 0.2186 ,  6.9992 },
       {1.6692 , 0.1202 ,  7.0278 },
       {1.6846 , 0.0217 ,  7.0563 },
       {1.7001 , 0.0768 ,  7.0849 },
       {1.7155 , 0.1753 ,  7.1134 },
       {1.7309 , 0.2737 ,  7.1420 },
       {1.7463 , 0.3722 ,  7.1706 },
       {1.7617 , 0.4707 ,  7.1991 },
       {1.7127 , 0.5649 ,  7.2277 },
       {1.6637 , 0.6592 ,  7.2562 },
       {1.6147 , 0.7535 ,  7.2848 },
       {1.5657 , 0.8478 ,  7.3134 },
       {1.5167 , 0.9420 ,  7.3419 },
       {1.4677 , 1.0363 ,  7.3705 },
       {1.4187 , 1.1306 ,  7.3990 },
       {1.3697 , 1.2249 ,  7.4276 },
       {1.3207 , 1.3191 ,  7.4562 },
       {1.2717 , 1.4134 ,  7.4847 },
       {1.2227 , 1.5077 ,  7.5133 },
       {1.1215 , 1.5575 ,  7.5418 },
       {1.0203 , 1.6074 ,  7.5704 },
       {0.9191 , 1.6572 ,  7.5990 },
       {0.8178 , 1.7070 ,  7.6275 },
       {0.7166 , 1.7569 ,  7.6561 },
       {0.6154 , 1.8067 ,  7.6846 },
       {0.5142 , 1.8566 ,  7.7132 },
       {0.4129 , 1.9064 ,  7.7418 },
       {0.3117 , 1.9562 ,  7.7703 },
       {0.2105 , 2.0061 ,  7.7989 },
       {0.1093 , 2.0559 ,  7.8274 },
       {-0.0083, -2.0350,   7.8560},
       {-0.1259, -2.0142,   7.8846},
       {-0.2435, -1.9933,   7.9131},
       {-0.3610, -1.9724,   7.9417},
       {-0.4786, -1.9515,   7.9702},
       {-0.5962, -1.9306,   7.9988},
       {-0.7138, -1.9097,   8.0274},
       {-0.8313, -1.8889,   8.0559},
       {-0.9489, -1.8680,   8.0845},
       {-1.0665, -1.8471,   8.1130},
       {-1.1840, -1.8262,   8.1416},
       {-1.2711, -1.7351,   8.1702},
       {-1.3581, -1.6439,   8.1987},
       {-1.4451, -1.5528,   8.2273},
       {-1.5321, -1.4617,   8.2558},
       {-1.6191, -1.3705,   8.2844},
       {-1.7061, -1.2794,   8.3130},
       {-1.7931, -1.1883,   8.3415},
       {-1.8801, -1.0971,   8.3701},
       {-1.9671, -1.0060,   8.3986},
       {-2.0541, -0.9149,   8.4272},
       {-2.1411, -0.8237,   8.4558},
       {-2.1583, -0.6922,   8.4843},
       {-2.1755, -0.5608,   8.5129},
       {-2.1927, -0.4293,   8.5414},
       {-2.2098, -0.2978,   8.5700},
       {-2.2270, -0.1663,   8.5986},
       {-2.2442, -0.0349,   8.6271},
       {-2.2614,  0.0966,   8.6557},
       {-2.2785,  0.2281,   8.6842},
       {-2.2957,  0.3595,   8.7128},
       {-2.3129,  0.4910,   8.7414},
       {-2.3300,  0.6225,   8.7699},
       {-2.2631,  0.7445,   8.7985},
       {-2.1961,  0.8665,   8.8270},
       {-2.1291,  0.9885,   8.8556},
       {-2.0621,  1.1105,   8.8842},
       {-1.9951,  1.2325,   8.9127},
       {-1.9282,  1.3545,   8.9413},
       {-1.8612,  1.4765,   8.9698},
       {-1.7942,  1.5985,   8.9984},
       {-1.7272,  1.7205,   9.0270},
       {-1.6602,  1.8425,   9.0555},
       {-1.5933,  1.9645,   9.0841},
       {-1.4612,  2.0263,   9.1126},
       {-1.3291,  2.0880,   9.1412},
       {-1.1970,  2.1497,   9.1698},
       {-1.0650,  2.2114,   9.1983},
       {-0.9329,  2.2731,   9.2269},
       {-0.8008,  2.3348,   9.2554},
       {-0.6688,  2.3965,   9.2840},
       {-0.5367,  2.4582,   9.3125},
       {-0.4046,  2.5199,   9.3411},
       {-0.2726,  2.5816,   9.3697},
       {-0.1405,  2.6433,   9.3982},
       {0.0090 , 2.6139 ,  9.4268 },
       {0.1585 , 2.5845 ,  9.4553 },
       {0.3080 , 2.5551 ,  9.4839 },
       {0.4575 , 2.5257 ,  9.5125 },
       {0.6070 , 2.4963 ,  9.5410 },
       {0.7565 , 2.4669 ,  9.5696 },
       {0.9060 , 2.4374 ,  9.5981 },
       {1.0555 , 2.4080 ,  9.6267 },
       {1.2051 , 2.3786 ,  9.6553 },
       {1.3546 , 2.3492 ,  9.6838 },
       {1.5041 , 2.3198 ,  9.7124 },
       {1.6119 , 2.2030 ,  9.7409 },
       {1.7197 , 2.0862 ,  9.7695 },
       {1.8275 , 1.9694 ,  9.7981 },
       {1.9354 , 1.8526 ,  9.8266 },
       {2.0432 , 1.7358 ,  9.8552 },
       {2.1510 , 1.6190 ,  9.8837 },
       {2.2588 , 1.5021 ,  9.9123 },
       {2.3667 , 1.3853 ,  9.9409 },
       {2.4745 , 1.2685 ,  9.9694 },
       {2.5823 , 1.1517 ,  9.9980 },
       {2.6901 , 1.0349 , 10.0265 },
       {2.7091 , 0.8704 , 10.0551 },
       {2.7280 , 0.7060 , 10.0837 },
       {2.7469 , 0.5415 , 10.1122 },
       {2.7659 , 0.3770 , 10.1408 },
       {2.7848 , 0.2125 , 10.1693 },
       {2.8037 , 0.0481 , 10.1979 },
       {2.8226 , 0.1164 , 10.2265 },
       {2.8416 , 0.2809 , 10.2550 },
       {2.8605 , 0.4454 , 10.2836 },
       {2.8794 , 0.6098 , 10.3121 },
       {2.8983 , 0.7743 , 10.3407 },
       {2.8134 , 0.9241 , 10.3693 },
       {2.7284 , 1.0738 , 10.3978 },
       {2.6435 , 1.2235 , 10.4264 },
       {2.5585 , 1.3733 , 10.4549 },
       {2.4735 , 1.5230 , 10.4835 },
       {2.3886 , 1.6727 , 10.5121 },
       {2.3036 , 1.8225 , 10.5406 },
       {2.2187 , 1.9722 , 10.5692 },
       {2.1337 , 2.1219 , 10.5977 },
       {2.0487 , 2.2717 , 10.6263 },
       {1.9638 , 2.4214 , 10.6549 },
       {1.8009 , 2.4950 , 10.6834 },
       {1.6380 , 2.5686 , 10.7120 },
       {1.4750 , 2.6421 , 10.7405 },
       {1.3121 , 2.7157 , 10.7691 },
       {1.1492 , 2.7893 , 10.7977 },
       {0.9863 , 2.8629 , 10.8262 },
       {0.8234 , 2.9364 , 10.8548 },
       {0.6605 , 3.0100 , 10.8833 },
       {0.4975 , 3.0836 , 10.9119 },
       {0.3346 , 3.1572 , 10.9405 },
       {0.1717 , 3.2307 , 10.9690 },
       {-0.0097, -3.1928,  10.9976},
       {-0.1912, -3.1548,  11.0261},
       {-0.3726, -3.1169,  11.0547},
       {-0.5540, -3.0790,  11.0833},
       {-0.7355, -3.0410,  11.1118},
       {-0.9169, -3.0031,  11.1404},
       {-1.0983, -2.9651,  11.1689},
       {-1.2798, -2.9272,  11.1975},
       {-1.4612, -2.8892,  11.2261},
       {-1.6426, -2.8513,  11.2546},
       {-1.8241, -2.8134,  11.2832},
       {-1.9527, -2.6709,  11.3117},
       {-2.0814, -2.5284,  11.3403},
       {-2.2100, -2.3859,  11.3689},
       {-2.3386, -2.2435,  11.3974},
       {-2.4673, -2.1010,  11.4260},
       {-2.5959, -1.9585,  11.4545},
       {-2.7246, -1.8160,  11.4831},
       {-2.8532, -1.6736,  11.5117},
       {-2.9819, -1.5311,  11.5402},
       {-3.1105, -1.3886,  11.5688},
       {-3.2392, -1.2461,  11.5973},
       {-3.2598, -1.0487,  11.6259},
       {-3.2805, -0.8512,  11.6545},
       {-3.3012, -0.6537,  11.6830},
       {-3.3219, -0.4562,  11.7116},
       {-3.3426, -0.2587,  11.7401},
       {-3.3632, -0.0613,  11.7687},
       {-3.3839,  0.1362,  11.7973},
       {-3.4046,  0.3337,  11.8258},
       {-3.4253,  0.5312,  11.8544},
       {-3.4460,  0.7287,  11.8829},
       {-3.4667,  0.9262,  11.9115},
       {-3.3637,  1.1036,  11.9401},
       {-3.2608,  1.2811,  11.9686},
       {-3.1578,  1.4586,  11.9972},
       {-3.0549,  1.6360,  12.0257},
       {-2.9520,  1.8135,  12.0543},
       {-2.8490,  1.9910,  12.0829},
       {-2.7461,  2.1684,  12.1114},
       {-2.6431,  2.3459,  12.1400},
       {-2.5402,  2.5234,  12.1685},
       {-2.4372,  2.7008,  12.1971},
       {-2.3343,  2.8783,  12.2257},
       {-2.1405,  2.9637,  12.2542},
       {-1.9468,  3.0492,  12.2828},
       {-1.7530,  3.1346,  12.3113},
       {-1.5593,  3.2201,  12.3399},
       {-1.3655,  3.3055,  12.3685},
       {-1.1717,  3.3909,  12.3970},
       {-0.9780,  3.4764,  12.4256},
       {-0.7842,  3.5618,  12.4541},
       {-0.5904,  3.6473,  12.4827},
       {-0.3967,  3.7327,  12.5113},
       {-0.2029,  3.8181,  12.5398},
       {0.0104 , 3.7717 , 12.5684 },
       {0.2238 , 3.7252 , 12.5969 },
       {0.4372 , 3.6787 , 12.6255 },
       {0.6505 , 3.6322 , 12.6541 },
       {0.8639 , 3.5858 , 12.6826 },
       {1.0773 , 3.5393 , 12.7112 },
       {1.2906 , 3.4928 , 12.7397 },
       {1.5040 , 3.4464 , 12.7683 },
       {1.7174 , 3.3999 , 12.7969 },
       {1.9307 , 3.3534 , 12.8254 },
       {2.1441 , 3.3069 , 12.8540 },
       {2.2935 , 3.1388 , 12.8825 },
       {2.4430 , 2.9706 , 12.9111 },
       {2.5925 , 2.8025 , 12.9397 },
       {2.7419 , 2.6344 , 12.9682 },
       {2.8914 , 2.4662 , 12.9968 },
       {3.0409 , 2.2981 , 13.0253 },
       {3.1903 , 2.1299 , 13.0539 },
       {3.3398 , 1.9618 , 13.0825 },
       {3.4892 , 1.7936 , 13.1110 },
       {3.6387 , 1.6255 , 13.1396 },
       {3.7882 , 1.4573 , 13.1681 },
       {3.8106 , 1.2269 , 13.1967 },
       {3.8330 , 0.9964 , 13.2253 },
       {3.8555 , 0.7659 , 13.2538 },
       {3.8779 , 0.5354 , 13.2824 },
       {3.9003 , 0.3049 , 13.3109 },
       {3.9228 , 0.0744 , 13.3395 },
       {3.9452 , 0.1560 , 13.3681 },
       {3.9676 , 0.3865 , 13.3966 },
       {3.9901 , 0.6170 , 13.4252 },
       {4.0125 , 0.8475 , 13.4537 },
       {4.0350 , 1.0780 , 13.4823 },
       {3.9140 , 1.2832 , 13.5109 },
       {3.7931 , 1.4884 , 13.5394 },
       {3.6722 , 1.6936 , 13.5680 },
       {3.5513 , 1.8988 , 13.5965 },
       {3.4304 , 2.1040 , 13.6251 },
       {3.3094 , 2.3092 , 13.6537 },
       {3.1885 , 2.5144 , 13.6822 },
       {3.0676 , 2.7196 , 13.7108 },
       {2.9467 , 2.9248 , 13.7393 },
       {2.8258 , 3.1300 , 13.7679 },
       {2.7048 , 3.3352 , 13.7965 },
       {2.4802 , 3.4325 , 13.8250 },
       {2.2556 , 3.5298 , 13.8536 },
       {2.0310 , 3.6271 , 13.8821 },
       {1.8064 , 3.7244 , 13.9107 },
       {1.5818 , 3.8217 , 13.9393 },
       {1.3572 , 3.9190 , 13.9678 },
       {1.1326 , 4.0163 , 13.9964 },
       {0.9080 , 4.1136 , 14.0249 },
       {0.6834 , 4.2109 , 14.0535 },
       {0.4587 , 4.3082 , 14.0821 },
       {0.2341 , 4.4055 , 14.1106 },
       {-0.0112, -4.3505,  14.1392},
       {-0.2564, -4.2955,  14.1677},
       {-0.5017, -4.2405,  14.1963},
       {-0.7470, -4.1855,  14.2249},
       {-0.9923, -4.1305,  14.2534},
       {-1.2376, -4.0755,  14.2820},
       {-1.4829, -4.0205,  14.3105},
       {-1.7282, -3.9655,  14.3391},
       {-1.9735, -3.9105,  14.3677},
       {-2.2188, -3.8555,  14.3962},
       {-2.4641, -3.8005,  14.4248},
       {-2.6344, -3.6067,  14.4533},
       {-2.8047, -3.4129,  14.4819},
       {-2.9749, -3.2191,  14.5105},
       {-3.1452, -3.0252,  14.5390},
       {-3.3155, -2.8314,  14.5676},
       {-3.4858, -2.6376,  14.5961},
       {-3.6561, -2.4438,  14.6247},
       {-3.8263, -2.2500,  14.6533},
       {-3.9966, -2.0562,  14.6818},
       {-4.1669, -1.8624,  14.7104},
       {-4.3372, -1.6686,  14.7389},
       {-4.3614, -1.4051,  14.7675},
       {-4.3856, -1.1416,  14.7961},
       {-4.4097, -0.8781,  14.8246},
       {-4.4339, -0.6146,  14.8532},
       {-4.4581, -0.3511,  14.8817},
       {-4.4823, -0.0876,  14.9103},
       {-4.5065,  0.1759,  14.9389},
       {-4.5307,  0.4393,  14.9674},
       {-4.5549,  0.7028,  14.9960},
       {-4.5791,  0.9663,  15.0245},
       {-4.6033,  1.2298,  15.0531},
       {-4.4644,  1.4627,  15.0817},
       {-4.3255,  1.6957,  15.1102},
       {-4.1866,  1.9286,  15.1388},
       {-4.0477,  2.1615,  15.1673},
       {-3.9088,  2.3945,  15.1959},
       {-3.7699,  2.6274,  15.2245},
       {-3.6310,  2.8603,  15.2530},
       {-3.4921,  3.0932,  15.2816},
       {-3.3532,  3.3262,  15.3101},
       {-3.2143,  3.5591,  15.3387},
       {-3.0754,  3.7920,  15.3673},
       {-2.8199,  3.9012,  15.3958},
       {-2.5644,  4.0104,  15.4244},
       {-2.3090,  4.1196,  15.4529},
       {-2.0535,  4.2287,  15.4815},
       {-1.7981,  4.3379,  15.5101},
       {-1.5426,  4.4471,  15.5386},
       {-1.2872,  4.5563,  15.5672},
       {-1.0317,  4.6654,  15.5957},
       {-0.7763,  4.7746,  15.6243},
       {-0.5208,  4.8838,  15.6529},
       {-0.2654,  4.9930,  15.6814}
};


#endif //VARIABLE_CURVATURE_ROBOT_SPIRALDATA_H
