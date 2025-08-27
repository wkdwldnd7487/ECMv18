/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     Contexts.cpp
 *  \brief    Classes providing probability descriptions and contexts (also contains context initialization values)
 */

#include "Contexts.h"

#include <algorithm>
#include <cstring>
#include <limits>

#if JVET_AG0196_CABAC_RETRAIN
namespace CabacRetrain
{
  bool tempCABAC = false;
#if JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE
  std::vector<int> vdrate0;
  std::vector<int> vdrate1;
#endif
  std::vector<int> vweight;
  std::vector<int> vrate;
  std::vector<std::pair<uint16_t,uint16_t>> vprobaInit;
  bool activate = false;
}
#endif
const uint8_t ProbModelTables::m_RenormTable_32[32] =
{
  6,  5,  4,  4,
  3,  3,  3,  3,
  2,  2,  2,  2,
  2,  2,  2,  2,
  1,  1,  1,  1,
  1,  1,  1,  1,
  1,  1,  1,  1,
  1,  1,  1,  1
};
#if EC_HIGH_PRECISION
const BinFracBits ProbModelTables::m_binFracBits[512] = {
	{ { 46,327680 } },
	{ { 139,275744 } },
	{ { 231,251595 } },
	{ { 324,235689 } },
	{ { 417,223808 } },
	{ { 511,214321 } },
	{ { 604,206424 } },
	{ { 698,199659 } },
	{ { 791,193742 } },
	{ { 885,188484 } },
	{ { 980,183753 } },
	{ { 1074,179452 } },
	{ { 1168,175510 } },
	{ { 1263,171872 } },
	{ { 1358,168494 } },
	{ { 1453,165341 } },
	{ { 1549,162385 } },
	{ { 1644,159604 } },
	{ { 1740,156977 } },
	{ { 1836,154488 } },
	{ { 1932,152124 } },
	{ { 2028,149872 } },
	{ { 2125,147723 } },
	{ { 2221,145667 } },
	{ { 2318,143697 } },
	{ { 2415,141806 } },
	{ { 2512,139988 } },
	{ { 2610,138236 } },
	{ { 2708,136548 } },
	{ { 2805,134918 } },
	{ { 2904,133342 } },
	{ { 3002,131816 } },
	{ { 3100,130339 } },
	{ { 3199,128906 } },
	{ { 3298,127516 } },
	{ { 3397,126165 } },
	{ { 3496,124852 } },
	{ { 3596,123574 } },
	{ { 3696,122330 } },
	{ { 3796,121118 } },
	{ { 3896,119936 } },
	{ { 3996,118783 } },
	{ { 4097,117657 } },
	{ { 4197,116558 } },
	{ { 4298,115483 } },
	{ { 4400,114433 } },
	{ { 4501,113405 } },
	{ { 4603,112399 } },
	{ { 4705,111414 } },
	{ { 4807,110449 } },
	{ { 4909,109504 } },
	{ { 5012,108577 } },
	{ { 5114,107668 } },
	{ { 5217,106776 } },
	{ { 5321,105900 } },
	{ { 5424,105041 } },
	{ { 5528,104196 } },
	{ { 5632,103367 } },
	{ { 5736,102552 } },
	{ { 5840,101751 } },
	{ { 5945,100963 } },
	{ { 6050,100188 } },
	{ { 6155, 99425 } },
	{ { 6260, 98675 } },
	{ { 6365, 97936 } },
	{ { 6471, 97209 } },
	{ { 6577, 96493 } },
	{ { 6683, 95787 } },
	{ { 6790, 95092 } },
	{ { 6897, 94407 } },
	{ { 7004, 93731 } },
	{ { 7111, 93065 } },
	{ { 7218, 92409 } },
	{ { 7326, 91761 } },
	{ { 7434, 91122 } },
	{ { 7542, 90492 } },
	{ { 7650, 89870 } },
	{ { 7759, 89256 } },
	{ { 7868, 88650 } },
	{ { 7977, 88051 } },
	{ { 8087, 87461 } },
	{ { 8196, 86877 } },
	{ { 8306, 86300 } },
	{ { 8416, 85731 } },
	{ { 8527, 85168 } },
	{ { 8638, 84612 } },
	{ { 8749, 84062 } },
	{ { 8860, 83519 } },
	{ { 8971, 82982 } },
	{ { 9083, 82450 } },
	{ { 9195, 81925 } },
	{ { 9307, 81406 } },
	{ { 9420, 80892 } },
	{ { 9533, 80383 } },
	{ { 9646, 79880 } },
	{ { 9759, 79383 } },
	{ { 9873, 78890 } },
	{ { 9987, 78403 } },
	{ { 10101, 77921 } },
	{ { 10215, 77443 } },
	{ { 10330, 76970 } },
	{ { 10445, 76502 } },
	{ { 10560, 76039 } },
	{ { 10676, 75580 } },
	{ { 10792, 75125 } },
	{ { 10908, 74675 } },
	{ { 11025, 74229 } },
	{ { 11141, 73787 } },
	{ { 11258, 73349 } },
	{ { 11376, 72916 } },
	{ { 11493, 72486 } },
	{ { 11611, 72060 } },
	{ { 11729, 71638 } },
	{ { 11848, 71220 } },
	{ { 11967, 70805 } },
	{ { 12086, 70394 } },
	{ { 12205, 69986 } },
	{ { 12325, 69582 } },
	{ { 12445, 69182 } },
	{ { 12565, 68784 } },
	{ { 12686, 68390 } },
	{ { 12806, 68000 } },
	{ { 12928, 67612 } },
	{ { 13049, 67228 } },
	{ { 13171, 66847 } },
	{ { 13293, 66468 } },
	{ { 13416, 66093 } },
	{ { 13538, 65721 } },
	{ { 13662, 65352 } },
	{ { 13785, 64985 } },
	{ { 13909, 64622 } },
	{ { 14033, 64261 } },
	{ { 14157, 63903 } },
	{ { 14282, 63547 } },
	{ { 14407, 63194 } },
	{ { 14532, 62844 } },
	{ { 14658, 62497 } },
	{ { 14784, 62151 } },
	{ { 14911, 61809 } },
	{ { 15037, 61469 } },
	{ { 15164, 61131 } },
	{ { 15292, 60796 } },
	{ { 15420, 60463 } },
	{ { 15548, 60132 } },
	{ { 15676, 59804 } },
	{ { 15805, 59478 } },
	{ { 15934, 59154 } },
	{ { 16064, 58833 } },
	{ { 16194, 58513 } },
	{ { 16324, 58196 } },
	{ { 16454, 57881 } },
	{ { 16585, 57568 } },
	{ { 16717, 57257 } },
	{ { 16848, 56948 } },
	{ { 16980, 56641 } },
	{ { 17113, 56336 } },
	{ { 17246, 56033 } },
	{ { 17379, 55732 } },
	{ { 17512, 55432 } },
	{ { 17646, 55135 } },
	{ { 17781, 54840 } },
	{ { 17915, 54546 } },
	{ { 18050, 54254 } },
	{ { 18186, 53964 } },
	{ { 18322, 53676 } },
	{ { 18458, 53389 } },
	{ { 18594, 53105 } },
	{ { 18731, 52821 } },
	{ { 18869, 52540 } },
	{ { 19007, 52260 } },
	{ { 19145, 51982 } },
	{ { 19284, 51706 } },
	{ { 19423, 51431 } },
	{ { 19562, 51158 } },
	{ { 19702, 50886 } },
	{ { 19842, 50616 } },
	{ { 19983, 50347 } },
	{ { 20124, 50080 } },
	{ { 20266, 49815 } },
	{ { 20408, 49550 } },
	{ { 20550, 49288 } },
	{ { 20693, 49027 } },
	{ { 20836, 48767 } },
	{ { 20980, 48509 } },
	{ { 21124, 48252 } },
	{ { 21268, 47996 } },
	{ { 21413, 47742 } },
	{ { 21559, 47489 } },
	{ { 21705, 47238 } },
	{ { 21851, 46988 } },
	{ { 21998, 46739 } },
	{ { 22145, 46491 } },
	{ { 22293, 46245 } },
	{ { 22441, 46000 } },
	{ { 22590, 45756 } },
	{ { 22739, 45514 } },
	{ { 22889, 45273 } },
	{ { 23039, 45033 } },
	{ { 23189, 44794 } },
	{ { 23340, 44556 } },
	{ { 23492, 44320 } },
	{ { 23644, 44085 } },
	{ { 23796, 43851 } },
	{ { 23949, 43618 } },
	{ { 24103, 43386 } },
	{ { 24257, 43156 } },
	{ { 24411, 42926 } },
	{ { 24566, 42698 } },
	{ { 24722, 42470 } },
	{ { 24878, 42244 } },
	{ { 25034, 42019 } },
	{ { 25191, 41795 } },
	{ { 25349, 41572 } },
	{ { 25507, 41350 } },
	{ { 25666, 41129 } },
	{ { 25825, 40909 } },
	{ { 25985, 40691 } },
	{ { 26145, 40473 } },
	{ { 26306, 40256 } },
	{ { 26467, 40040 } },
	{ { 26629, 39825 } },
	{ { 26791, 39611 } },
	{ { 26954, 39398 } },
	{ { 27118, 39186 } },
	{ { 27282, 38975 } },
	{ { 27447, 38765 } },
	{ { 27612, 38556 } },
	{ { 27778, 38348 } },
	{ { 27944, 38140 } },
	{ { 28111, 37934 } },
	{ { 28279, 37728 } },
	{ { 28447, 37524 } },
	{ { 28616, 37320 } },
	{ { 28786, 37117 } },
	{ { 28956, 36915 } },
	{ { 29126, 36714 } },
	{ { 29298, 36514 } },
	{ { 29469, 36314 } },
	{ { 29642, 36115 } },
	{ { 29815, 35918 } },
	{ { 29989, 35721 } },
	{ { 30163, 35524 } },
	{ { 30339, 35329 } },
	{ { 30514, 35135 } },
	{ { 30691, 34941 } },
	{ { 30868, 34748 } },
	{ { 31045, 34556 } },
	{ { 31224, 34364 } },
	{ { 31403, 34174 } },
	{ { 31583, 33984 } },
	{ { 31763, 33795 } },
	{ { 31944, 33606 } },
	{ { 32126, 33419 } },
	{ { 32309, 33232 } },
	{ { 32492, 33046 } },
	{ { 32676, 32860 } },
	{ { 32860, 32676 } },
	{ { 33046, 32492 } },
	{ { 33232, 32309 } },
	{ { 33419, 32126 } },
	{ { 33606, 31944 } },
	{ { 33795, 31763 } },
	{ { 33984, 31583 } },
	{ { 34174, 31403 } },
	{ { 34364, 31224 } },
	{ { 34556, 31045 } },
	{ { 34748, 30868 } },
	{ { 34941, 30691 } },
	{ { 35135, 30514 } },
	{ { 35329, 30339 } },
	{ { 35524, 30163 } },
	{ { 35721, 29989 } },
	{ { 35918, 29815 } },
	{ { 36115, 29642 } },
	{ { 36314, 29469 } },
	{ { 36514, 29298 } },
	{ { 36714, 29126 } },
	{ { 36915, 28956 } },
	{ { 37117, 28786 } },
	{ { 37320, 28616 } },
	{ { 37524, 28447 } },
	{ { 37728, 28279 } },
	{ { 37934, 28111 } },
	{ { 38140, 27944 } },
	{ { 38348, 27778 } },
	{ { 38556, 27612 } },
	{ { 38765, 27447 } },
	{ { 38975, 27282 } },
	{ { 39186, 27118 } },
	{ { 39398, 26954 } },
	{ { 39611, 26791 } },
	{ { 39825, 26629 } },
	{ { 40040, 26467 } },
	{ { 40256, 26306 } },
	{ { 40473, 26145 } },
	{ { 40691, 25985 } },
	{ { 40909, 25825 } },
	{ { 41129, 25666 } },
	{ { 41350, 25507 } },
	{ { 41572, 25349 } },
	{ { 41795, 25191 } },
	{ { 42019, 25034 } },
	{ { 42244, 24878 } },
	{ { 42470, 24722 } },
	{ { 42698, 24566 } },
	{ { 42926, 24411 } },
	{ { 43156, 24257 } },
	{ { 43386, 24103 } },
	{ { 43618, 23949 } },
	{ { 43851, 23796 } },
	{ { 44085, 23644 } },
	{ { 44320, 23492 } },
	{ { 44556, 23340 } },
	{ { 44794, 23189 } },
	{ { 45033, 23039 } },
	{ { 45273, 22889 } },
	{ { 45514, 22739 } },
	{ { 45756, 22590 } },
	{ { 46000, 22441 } },
	{ { 46245, 22293 } },
	{ { 46491, 22145 } },
	{ { 46739, 21998 } },
	{ { 46988, 21851 } },
	{ { 47238, 21705 } },
	{ { 47489, 21559 } },
	{ { 47742, 21413 } },
	{ { 47996, 21268 } },
	{ { 48252, 21124 } },
	{ { 48509, 20980 } },
	{ { 48767, 20836 } },
	{ { 49027, 20693 } },
	{ { 49288, 20550 } },
	{ { 49550, 20408 } },
	{ { 49815, 20266 } },
	{ { 50080, 20124 } },
	{ { 50347, 19983 } },
	{ { 50616, 19842 } },
	{ { 50886, 19702 } },
	{ { 51158, 19562 } },
	{ { 51431, 19423 } },
	{ { 51706, 19284 } },
	{ { 51982, 19145 } },
	{ { 52260, 19007 } },
	{ { 52540, 18869 } },
	{ { 52821, 18731 } },
	{ { 53105, 18594 } },
	{ { 53389, 18458 } },
	{ { 53676, 18322 } },
	{ { 53964, 18186 } },
	{ { 54254, 18050 } },
	{ { 54546, 17915 } },
	{ { 54840, 17781 } },
	{ { 55135, 17646 } },
	{ { 55432, 17512 } },
	{ { 55732, 17379 } },
	{ { 56033, 17246 } },
	{ { 56336, 17113 } },
	{ { 56641, 16980 } },
	{ { 56948, 16848 } },
	{ { 57257, 16717 } },
	{ { 57568, 16585 } },
	{ { 57881, 16454 } },
	{ { 58196, 16324 } },
	{ { 58513, 16194 } },
	{ { 58833, 16064 } },
	{ { 59154, 15934 } },
	{ { 59478, 15805 } },
	{ { 59804, 15676 } },
	{ { 60132, 15548 } },
	{ { 60463, 15420 } },
	{ { 60796, 15292 } },
	{ { 61131, 15164 } },
	{ { 61469, 15037 } },
	{ { 61809, 14911 } },
	{ { 62151, 14784 } },
	{ { 62497, 14658 } },
	{ { 62844, 14532 } },
	{ { 63194, 14407 } },
	{ { 63547, 14282 } },
	{ { 63903, 14157 } },
	{ { 64261, 14033 } },
	{ { 64622, 13909 } },
	{ { 64985, 13785 } },
	{ { 65352, 13662 } },
	{ { 65721, 13538 } },
	{ { 66093, 13416 } },
	{ { 66468, 13293 } },
	{ { 66847, 13171 } },
	{ { 67228, 13049 } },
	{ { 67612, 12928 } },
	{ { 68000, 12806 } },
	{ { 68390, 12686 } },
	{ { 68784, 12565 } },
	{ { 69182, 12445 } },
	{ { 69582, 12325 } },
	{ { 69986, 12205 } },
	{ { 70394, 12086 } },
	{ { 70805, 11967 } },
	{ { 71220, 11848 } },
	{ { 71638, 11729 } },
	{ { 72060, 11611 } },
	{ { 72486, 11493 } },
	{ { 72916, 11376 } },
	{ { 73349, 11258 } },
	{ { 73787, 11141 } },
	{ { 74229, 11025 } },
	{ { 74675, 10908 } },
	{ { 75125, 10792 } },
	{ { 75580, 10676 } },
	{ { 76039, 10560 } },
	{ { 76502, 10445 } },
	{ { 76970, 10330 } },
	{ { 77443, 10215 } },
	{ { 77921, 10101 } },
	{ { 78403,  9987 } },
	{ { 78890,  9873 } },
	{ { 79383,  9759 } },
	{ { 79880,  9646 } },
	{ { 80383,  9533 } },
	{ { 80892,  9420 } },
	{ { 81406,  9307 } },
	{ { 81925,  9195 } },
	{ { 82450,  9083 } },
	{ { 82982,  8971 } },
	{ { 83519,  8860 } },
	{ { 84062,  8749 } },
	{ { 84612,  8638 } },
	{ { 85168,  8527 } },
	{ { 85731,  8416 } },
	{ { 86300,  8306 } },
	{ { 86877,  8196 } },
	{ { 87461,  8087 } },
	{ { 88051,  7977 } },
	{ { 88650,  7868 } },
	{ { 89256,  7759 } },
	{ { 89870,  7650 } },
	{ { 90492,  7542 } },
	{ { 91122,  7434 } },
	{ { 91761,  7326 } },
	{ { 92409,  7218 } },
	{ { 93065,  7111 } },
	{ { 93731,  7004 } },
	{ { 94407,  6897 } },
	{ { 95092,  6790 } },
	{ { 95787,  6683 } },
	{ { 96493,  6577 } },
	{ { 97209,  6471 } },
	{ { 97936,  6365 } },
	{ { 98675,  6260 } },
	{ { 99425,  6155 } },
	{ { 100188,  6050 } },
	{ { 100963,  5945 } },
	{ { 101751,  5840 } },
	{ { 102552,  5736 } },
	{ { 103367,  5632 } },
	{ { 104196,  5528 } },
	{ { 105041,  5424 } },
	{ { 105900,  5321 } },
	{ { 106776,  5217 } },
	{ { 107668,  5114 } },
	{ { 108577,  5012 } },
	{ { 109504,  4909 } },
	{ { 110449,  4807 } },
	{ { 111414,  4705 } },
	{ { 112399,  4603 } },
	{ { 113405,  4501 } },
	{ { 114433,  4400 } },
	{ { 115483,  4298 } },
	{ { 116558,  4197 } },
	{ { 117657,  4097 } },
	{ { 118783,  3996 } },
	{ { 119936,  3896 } },
	{ { 121118,  3796 } },
	{ { 122330,  3696 } },
	{ { 123574,  3596 } },
	{ { 124852,  3496 } },
	{ { 126165,  3397 } },
	{ { 127516,  3298 } },
	{ { 128906,  3199 } },
	{ { 130339,  3100 } },
	{ { 131816,  3002 } },
	{ { 133342,  2904 } },
	{ { 134918,  2805 } },
	{ { 136548,  2708 } },
	{ { 138236,  2610 } },
	{ { 139988,  2512 } },
	{ { 141806,  2415 } },
	{ { 143697,  2318 } },
	{ { 145667,  2221 } },
	{ { 147723,  2125 } },
	{ { 149872,  2028 } },
	{ { 152124,  1932 } },
	{ { 154488,  1836 } },
	{ { 156977,  1740 } },
	{ { 159604,  1644 } },
	{ { 162385,  1549 } },
	{ { 165341,  1453 } },
	{ { 168494,  1358 } },
	{ { 171872,  1263 } },
	{ { 175510,  1168 } },
	{ { 179452,  1074 } },
	{ { 183753,   980 } },
	{ { 188484,   885 } },
	{ { 193742,   791 } },
	{ { 199659,   698 } },
	{ { 206424,   604 } },
	{ { 214321,   511 } },
	{ { 223808,   417 } },
	{ { 235689,   324 } },
	{ { 251595,   231 } },
	{ { 275744,   139 } },
	{ { 327680,    46 } },
};
#else
const BinFracBits ProbModelTables::m_binFracBits[256] = {
  { { 0x0005c, 0x48000 } }, { { 0x00116, 0x3b520 } }, { { 0x001d0, 0x356cb } }, { { 0x0028b, 0x318a9 } },
  { { 0x00346, 0x2ea40 } }, { { 0x00403, 0x2c531 } }, { { 0x004c0, 0x2a658 } }, { { 0x0057e, 0x28beb } },
  { { 0x0063c, 0x274ce } }, { { 0x006fc, 0x26044 } }, { { 0x007bc, 0x24dc9 } }, { { 0x0087d, 0x23cfc } },
  { { 0x0093f, 0x22d96 } }, { { 0x00a01, 0x21f60 } }, { { 0x00ac4, 0x2122e } }, { { 0x00b89, 0x205dd } },
  { { 0x00c4e, 0x1fa51 } }, { { 0x00d13, 0x1ef74 } }, { { 0x00dda, 0x1e531 } }, { { 0x00ea2, 0x1db78 } },
  { { 0x00f6a, 0x1d23c } }, { { 0x01033, 0x1c970 } }, { { 0x010fd, 0x1c10b } }, { { 0x011c8, 0x1b903 } },
  { { 0x01294, 0x1b151 } }, { { 0x01360, 0x1a9ee } }, { { 0x0142e, 0x1a2d4 } }, { { 0x014fc, 0x19bfc } },
  { { 0x015cc, 0x19564 } }, { { 0x0169c, 0x18f06 } }, { { 0x0176d, 0x188de } }, { { 0x0183f, 0x182e8 } },
  { { 0x01912, 0x17d23 } }, { { 0x019e6, 0x1778a } }, { { 0x01abb, 0x1721c } }, { { 0x01b91, 0x16cd5 } },
  { { 0x01c68, 0x167b4 } }, { { 0x01d40, 0x162b6 } }, { { 0x01e19, 0x15dda } }, { { 0x01ef3, 0x1591e } },
  { { 0x01fcd, 0x15480 } }, { { 0x020a9, 0x14fff } }, { { 0x02186, 0x14b99 } }, { { 0x02264, 0x1474e } },
  { { 0x02343, 0x1431b } }, { { 0x02423, 0x13f01 } }, { { 0x02504, 0x13afd } }, { { 0x025e6, 0x1370f } },
  { { 0x026ca, 0x13336 } }, { { 0x027ae, 0x12f71 } }, { { 0x02894, 0x12bc0 } }, { { 0x0297a, 0x12821 } },
  { { 0x02a62, 0x12494 } }, { { 0x02b4b, 0x12118 } }, { { 0x02c35, 0x11dac } }, { { 0x02d20, 0x11a51 } },
  { { 0x02e0c, 0x11704 } }, { { 0x02efa, 0x113c7 } }, { { 0x02fe9, 0x11098 } }, { { 0x030d9, 0x10d77 } },
  { { 0x031ca, 0x10a63 } }, { { 0x032bc, 0x1075c } }, { { 0x033b0, 0x10461 } }, { { 0x034a5, 0x10173 } },
  { { 0x0359b, 0x0fe90 } }, { { 0x03693, 0x0fbb9 } }, { { 0x0378c, 0x0f8ed } }, { { 0x03886, 0x0f62b } },
  { { 0x03981, 0x0f374 } }, { { 0x03a7e, 0x0f0c7 } }, { { 0x03b7c, 0x0ee23 } }, { { 0x03c7c, 0x0eb89 } },
  { { 0x03d7d, 0x0e8f9 } }, { { 0x03e7f, 0x0e671 } }, { { 0x03f83, 0x0e3f2 } }, { { 0x04088, 0x0e17c } },
  { { 0x0418e, 0x0df0e } }, { { 0x04297, 0x0dca8 } }, { { 0x043a0, 0x0da4a } }, { { 0x044ab, 0x0d7f3 } },
  { { 0x045b8, 0x0d5a5 } }, { { 0x046c6, 0x0d35d } }, { { 0x047d6, 0x0d11c } }, { { 0x048e7, 0x0cee3 } },
  { { 0x049fa, 0x0ccb0 } }, { { 0x04b0e, 0x0ca84 } }, { { 0x04c24, 0x0c85e } }, { { 0x04d3c, 0x0c63f } },
  { { 0x04e55, 0x0c426 } }, { { 0x04f71, 0x0c212 } }, { { 0x0508d, 0x0c005 } }, { { 0x051ac, 0x0bdfe } },
  { { 0x052cc, 0x0bbfc } }, { { 0x053ee, 0x0b9ff } }, { { 0x05512, 0x0b808 } }, { { 0x05638, 0x0b617 } },
  { { 0x0575f, 0x0b42a } }, { { 0x05888, 0x0b243 } }, { { 0x059b4, 0x0b061 } }, { { 0x05ae1, 0x0ae83 } },
  { { 0x05c10, 0x0acaa } }, { { 0x05d41, 0x0aad6 } }, { { 0x05e74, 0x0a907 } }, { { 0x05fa9, 0x0a73c } },
  { { 0x060e0, 0x0a575 } }, { { 0x06219, 0x0a3b3 } }, { { 0x06354, 0x0a1f5 } }, { { 0x06491, 0x0a03b } },
  { { 0x065d1, 0x09e85 } }, { { 0x06712, 0x09cd4 } }, { { 0x06856, 0x09b26 } }, { { 0x0699c, 0x0997c } },
  { { 0x06ae4, 0x097d6 } }, { { 0x06c2f, 0x09634 } }, { { 0x06d7c, 0x09495 } }, { { 0x06ecb, 0x092fa } },
  { { 0x0701d, 0x09162 } }, { { 0x07171, 0x08fce } }, { { 0x072c7, 0x08e3e } }, { { 0x07421, 0x08cb0 } },
  { { 0x0757c, 0x08b26 } }, { { 0x076da, 0x089a0 } }, { { 0x0783b, 0x0881c } }, { { 0x0799f, 0x0869c } },
  { { 0x07b05, 0x0851f } }, { { 0x07c6e, 0x083a4 } }, { { 0x07dd9, 0x0822d } }, { { 0x07f48, 0x080b9 } },
  { { 0x080b9, 0x07f48 } }, { { 0x0822d, 0x07dd9 } }, { { 0x083a4, 0x07c6e } }, { { 0x0851f, 0x07b05 } },
  { { 0x0869c, 0x0799f } }, { { 0x0881c, 0x0783b } }, { { 0x089a0, 0x076da } }, { { 0x08b26, 0x0757c } },
  { { 0x08cb0, 0x07421 } }, { { 0x08e3e, 0x072c7 } }, { { 0x08fce, 0x07171 } }, { { 0x09162, 0x0701d } },
  { { 0x092fa, 0x06ecb } }, { { 0x09495, 0x06d7c } }, { { 0x09634, 0x06c2f } }, { { 0x097d6, 0x06ae4 } },
  { { 0x0997c, 0x0699c } }, { { 0x09b26, 0x06856 } }, { { 0x09cd4, 0x06712 } }, { { 0x09e85, 0x065d1 } },
  { { 0x0a03b, 0x06491 } }, { { 0x0a1f5, 0x06354 } }, { { 0x0a3b3, 0x06219 } }, { { 0x0a575, 0x060e0 } },
  { { 0x0a73c, 0x05fa9 } }, { { 0x0a907, 0x05e74 } }, { { 0x0aad6, 0x05d41 } }, { { 0x0acaa, 0x05c10 } },
  { { 0x0ae83, 0x05ae1 } }, { { 0x0b061, 0x059b4 } }, { { 0x0b243, 0x05888 } }, { { 0x0b42a, 0x0575f } },
  { { 0x0b617, 0x05638 } }, { { 0x0b808, 0x05512 } }, { { 0x0b9ff, 0x053ee } }, { { 0x0bbfc, 0x052cc } },
  { { 0x0bdfe, 0x051ac } }, { { 0x0c005, 0x0508d } }, { { 0x0c212, 0x04f71 } }, { { 0x0c426, 0x04e55 } },
  { { 0x0c63f, 0x04d3c } }, { { 0x0c85e, 0x04c24 } }, { { 0x0ca84, 0x04b0e } }, { { 0x0ccb0, 0x049fa } },
  { { 0x0cee3, 0x048e7 } }, { { 0x0d11c, 0x047d6 } }, { { 0x0d35d, 0x046c6 } }, { { 0x0d5a5, 0x045b8 } },
  { { 0x0d7f3, 0x044ab } }, { { 0x0da4a, 0x043a0 } }, { { 0x0dca8, 0x04297 } }, { { 0x0df0e, 0x0418e } },
  { { 0x0e17c, 0x04088 } }, { { 0x0e3f2, 0x03f83 } }, { { 0x0e671, 0x03e7f } }, { { 0x0e8f9, 0x03d7d } },
  { { 0x0eb89, 0x03c7c } }, { { 0x0ee23, 0x03b7c } }, { { 0x0f0c7, 0x03a7e } }, { { 0x0f374, 0x03981 } },
  { { 0x0f62b, 0x03886 } }, { { 0x0f8ed, 0x0378c } }, { { 0x0fbb9, 0x03693 } }, { { 0x0fe90, 0x0359b } },
  { { 0x10173, 0x034a5 } }, { { 0x10461, 0x033b0 } }, { { 0x1075c, 0x032bc } }, { { 0x10a63, 0x031ca } },
  { { 0x10d77, 0x030d9 } }, { { 0x11098, 0x02fe9 } }, { { 0x113c7, 0x02efa } }, { { 0x11704, 0x02e0c } },
  { { 0x11a51, 0x02d20 } }, { { 0x11dac, 0x02c35 } }, { { 0x12118, 0x02b4b } }, { { 0x12494, 0x02a62 } },
  { { 0x12821, 0x0297a } }, { { 0x12bc0, 0x02894 } }, { { 0x12f71, 0x027ae } }, { { 0x13336, 0x026ca } },
  { { 0x1370f, 0x025e6 } }, { { 0x13afd, 0x02504 } }, { { 0x13f01, 0x02423 } }, { { 0x1431b, 0x02343 } },
  { { 0x1474e, 0x02264 } }, { { 0x14b99, 0x02186 } }, { { 0x14fff, 0x020a9 } }, { { 0x15480, 0x01fcd } },
  { { 0x1591e, 0x01ef3 } }, { { 0x15dda, 0x01e19 } }, { { 0x162b6, 0x01d40 } }, { { 0x167b4, 0x01c68 } },
  { { 0x16cd5, 0x01b91 } }, { { 0x1721c, 0x01abb } }, { { 0x1778a, 0x019e6 } }, { { 0x17d23, 0x01912 } },
  { { 0x182e8, 0x0183f } }, { { 0x188de, 0x0176d } }, { { 0x18f06, 0x0169c } }, { { 0x19564, 0x015cc } },
  { { 0x19bfc, 0x014fc } }, { { 0x1a2d4, 0x0142e } }, { { 0x1a9ee, 0x01360 } }, { { 0x1b151, 0x01294 } },
  { { 0x1b903, 0x011c8 } }, { { 0x1c10b, 0x010fd } }, { { 0x1c970, 0x01033 } }, { { 0x1d23c, 0x00f6a } },
  { { 0x1db78, 0x00ea2 } }, { { 0x1e531, 0x00dda } }, { { 0x1ef74, 0x00d13 } }, { { 0x1fa51, 0x00c4e } },
  { { 0x205dd, 0x00b89 } }, { { 0x2122e, 0x00ac4 } }, { { 0x21f60, 0x00a01 } }, { { 0x22d96, 0x0093f } },
  { { 0x23cfc, 0x0087d } }, { { 0x24dc9, 0x007bc } }, { { 0x26044, 0x006fc } }, { { 0x274ce, 0x0063c } },
  { { 0x28beb, 0x0057e } }, { { 0x2a658, 0x004c0 } }, { { 0x2c531, 0x00403 } }, { { 0x2ea40, 0x00346 } },
  { { 0x318a9, 0x0028b } }, { { 0x356cb, 0x001d0 } }, { { 0x3b520, 0x00116 } }, { { 0x48000, 0x0005c } },
};
#endif
void BinProbModel_Std::init( int qp, int initId )
{
  int slope = (initId >> 3) - 4;
  int offset = ((initId & 7) * 18) + 1;
  int inistate = ((slope   * (qp - 16)) >> 1) + offset;
  int stateClip = inistate < 1 ? 1 : inistate > 127 ? 127 : inistate;
  const int p1 = (stateClip << 8);
  m_state[0]   = p1 & MASK_0;
  m_state[1]   = p1 & MASK_1;
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  m_stateUsed[0] = m_state[0];
  m_stateUsed[1] = m_state[1];
#endif
}




CtxSet::CtxSet( std::initializer_list<CtxSet> ctxSets )
{
  uint16_t  minOffset = std::numeric_limits<uint16_t>::max();
  uint16_t  maxOffset = 0;
  for( auto iter = ctxSets.begin(); iter != ctxSets.end(); iter++ )
  {
    minOffset = std::min<uint16_t>( minOffset, (*iter).Offset              );
    maxOffset = std::max<uint16_t>( maxOffset, (*iter).Offset+(*iter).Size );
  }
  Offset  = minOffset;
  Size    = maxOffset - minOffset;
}





const std::vector<uint8_t>& ContextSetCfg::getInitTable( unsigned initId )
{
  CHECK( initId >= (unsigned)sm_InitTables.size(),
         "Invalid initId (" << initId << "), only " << sm_InitTables.size() << " tables defined." );
  return sm_InitTables[initId];
}

CtxSet ContextSetCfg::addCtxSet( std::initializer_list<std::initializer_list<uint8_t>> initSet2d )
{
  const std::size_t startIdx  = sm_InitTables[0].size();
  const std::size_t numValues = ( *initSet2d.begin() ).size();
  std::size_t setId     = 0;

  for( auto setIter = initSet2d.begin(); setIter != initSet2d.end() && setId < sm_InitTables.size(); setIter++, setId++ )
  {
    const std::initializer_list<uint8_t>& initSet   = *setIter;
    std::vector<uint8_t>&           initTable = sm_InitTables[setId];
    CHECK( initSet.size() != numValues,
      "Number of init values do not match for all sets (" << initSet.size() << " != " << numValues << ")." );
    initTable.resize( startIdx + numValues );
    std::size_t elemId = startIdx;
    for( auto elemIter = ( *setIter ).begin(); elemIter != ( *setIter ).end(); elemIter++, elemId++ )
    {
      initTable[elemId] = *elemIter;
    }
  }
  return CtxSet( (uint16_t)startIdx, (uint16_t)numValues );
}

#define CNU 35
#if SLICE_TYPE_WIN_SIZE
#if JVET_AH0176_LOW_DELAY_B_CTX
std::vector<std::vector<uint8_t>> ContextSetCfg::sm_InitTables( NUMBER_OF_SLICE_TYPES * 3 + 2 + 4 + 2);
#elif JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE
std::vector<std::vector<uint8_t>> ContextSetCfg::sm_InitTables( NUMBER_OF_SLICE_TYPES * 3 + 2 + 4 );
#elif JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
std::vector<std::vector<uint8_t>> ContextSetCfg::sm_InitTables( NUMBER_OF_SLICE_TYPES * 3 + 2);
#else
std::vector<std::vector<uint8_t>> ContextSetCfg::sm_InitTables(NUMBER_OF_SLICE_TYPES << 1);
#endif
#else
std::vector<std::vector<uint8_t>> ContextSetCfg::sm_InitTables(NUMBER_OF_SLICE_TYPES + 1);
#endif

#if JVET_AK0135_CABAC_RETRAIN
#include "Contexts_ecm16.0.inl"
#elif JVET_AI0281_CABAC_RETRAIN
#include "Contexts_ecm14.0.inl"
#elif JVET_AH0176_LOW_DELAY_B_CTX
#include "Contexts_ecm13.inl"
#elif JVET_ECM12_CABAC_RETRAIN
#include "Contexts_ecm12.inl"
#elif JVET_AF0133_RETRAINING_ISLICE_CTX
#include "Contexts_ecm11.inl"
#elif JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
#include "Contexts_ecm5.inl"
#elif SLICE_TYPE_WIN_SIZE
#include "Contexts_ecm1.inl"
#else
#include "Contexts_vtm.inl"
#endif

const unsigned ContextSetCfg::NumberOfContexts = (unsigned)ContextSetCfg::sm_InitTables[0].size();


// combined sets
const CtxSet ContextSetCfg::Palette = { ContextSetCfg::RotationFlag, ContextSetCfg::RunTypeFlag, ContextSetCfg::IdxRunModel, ContextSetCfg::CopyRunModel };
const CtxSet ContextSetCfg::Sao = { ContextSetCfg::SaoMergeFlag, ContextSetCfg::SaoTypeIdx };

const CtxSet ContextSetCfg::Alf = { ContextSetCfg::ctbAlfFlag, ContextSetCfg::ctbAlfAlternative, ContextSetCfg::AlfUseTemporalFilt };
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
const CtxSet ContextSetCfg::Split = { ContextSetCfg::SplitFlag, ContextSetCfg::SplitQtFlag, ContextSetCfg::SplitHvFlag, ContextSetCfg::Split12Flag };
#else
const CtxSet ContextSetCfg::Split = { ContextSetCfg::SplitFlag, ContextSetCfg::SplitQtFlag, ContextSetCfg::SplitHvFlag, ContextSetCfg::Split12Flag, ContextSetCfg::ModeConsFlag };
#endif

template <class BinProbModel>
CtxStore<BinProbModel>::CtxStore()
  : m_ctxBuffer ()
  , m_ctx       ( nullptr )
{}

template <class BinProbModel>
CtxStore<BinProbModel>::CtxStore( bool dummy )
  : m_ctxBuffer ( ContextSetCfg::NumberOfContexts )
  , m_ctx       ( m_ctxBuffer.data() )
{}

template <class BinProbModel>
CtxStore<BinProbModel>::CtxStore( const CtxStore<BinProbModel>& ctxStore )
  : m_ctxBuffer ( ctxStore.m_ctxBuffer )
  , m_ctx       ( m_ctxBuffer.data() )
{}

template <class BinProbModel>
void CtxStore<BinProbModel>::init( int qp, int initId )
{
  const std::vector<uint8_t>& initTable = ContextSetCfg::getInitTable( initId );
  CHECK( m_ctxBuffer.size() != initTable.size(),
        "Size of init table (" << initTable.size() << ") does not match size of context buffer (" << m_ctxBuffer.size() << ")." );
#if SLICE_TYPE_WIN_SIZE
  const std::vector<uint8_t> &rateInitTable = ContextSetCfg::getInitTable(NUMBER_OF_SLICE_TYPES + initId);
#else
  const std::vector<uint8_t> &rateInitTable = ContextSetCfg::getInitTable(NUMBER_OF_SLICE_TYPES);
#endif
  CHECK(m_ctxBuffer.size() != rateInitTable.size(),
        "Size of rate init table (" << rateInitTable.size() << ") does not match size of context buffer ("
                                    << m_ctxBuffer.size() << ").");
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  const std::vector<uint8_t> &weightInitTable = ContextSetCfg::getInitTable( (NUMBER_OF_SLICE_TYPES << 1) + initId );
  CHECK( m_ctxBuffer.size() != weightInitTable.size(),
         "Size of weight init table (" << weightInitTable.size() << ") does not match size of context buffer (" << m_ctxBuffer.size() << ")." );

#if JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE
  const std::vector<uint8_t> &rateOffsetInitTable0 = ContextSetCfg::getInitTable( ( NUMBER_OF_SLICE_TYPES * 3 ) + initId * 2 );
  const std::vector<uint8_t> &rateOffsetInitTable1 = ContextSetCfg::getInitTable( ( NUMBER_OF_SLICE_TYPES * 3 ) + 1 + initId * 2 );
#else
  const std::vector<uint8_t> &rateOffsetInitTable0 = ContextSetCfg::getInitTable((NUMBER_OF_SLICE_TYPES * 3));
  const std::vector<uint8_t> &rateOffsetInitTable1 = ContextSetCfg::getInitTable((NUMBER_OF_SLICE_TYPES * 3) + 1);
#endif

  CHECK(m_ctxBuffer.size() != rateOffsetInitTable0.size(),
        "Size of weight init table (" << rateOffsetInitTable0.size() << ") does not match size of context buffer ("
                                      << m_ctxBuffer.size() << ").");
  CHECK(m_ctxBuffer.size() != rateOffsetInitTable1.size(),
        "Size of weight init table (" << rateOffsetInitTable1.size() << ") does not match size of context buffer ("
                                      << m_ctxBuffer.size() << ").");
#endif


  int clippedQP = Clip3( 0, MAX_QP, qp );
  for( std::size_t k = 0; k < m_ctxBuffer.size(); k++ )
  {
    m_ctxBuffer[k].init( clippedQP, initTable[k] );
    m_ctxBuffer[k].setLog2WindowSize(rateInitTable[k]);

#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
    m_ctxBuffer[k].setAdaptRateOffset(rateOffsetInitTable0[k], 0);
    m_ctxBuffer[k].setAdaptRateOffset(rateOffsetInitTable1[k], 1);
    m_ctxBuffer[k].setAdaptRateWeight( weightInitTable[k] );
#endif
#if JVET_AG0196_CABAC_RETRAIN
    if (CabacRetrain::activate)
    {
#if JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE
      CabacRetrain::vdrate0[k]      = rateOffsetInitTable0[k];
      CabacRetrain::vdrate1[k]      = rateOffsetInitTable1[k];
#endif
      CabacRetrain::vrate[k]      = rateInitTable[k];
      CabacRetrain::vweight[k]    = weightInitTable[k];
      CabacRetrain::vprobaInit[k] = m_ctxBuffer[k].getState();
    }
#endif
  }
}

#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
#if JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE
template <class BinProbModel>
void CtxStore<BinProbModel>::saveRateOffsets( std::vector<uint8_t>& rateOffset0, std::vector<uint8_t>& rateOffset1 ) const
{
  rateOffset0.resize( m_ctxBuffer.size(), uint8_t( 0 ) );
  rateOffset1.resize( m_ctxBuffer.size(), uint8_t( 0 ) );

  for( std::size_t k = 0; k < m_ctxBuffer.size(); k++ )
  {
    rateOffset0[k] = m_ctxBuffer[k].getAdaptRateOffset( 0 );
    rateOffset1[k] = m_ctxBuffer[k].getAdaptRateOffset( 1 );
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::loadRateOffsets( const std::vector<uint8_t>& rateOffset0, const std::vector<uint8_t>& rateOffset1 )
{
  CHECK( m_ctxBuffer.size() != rateOffset0.size(),
    "Size of prob states table (" << rateOffset0.size() << ") does not match size of context buffer (" << m_ctxBuffer.size() << ")." );

  CHECK( m_ctxBuffer.size() != rateOffset1.size(),
    "Size of prob states table (" << rateOffset1.size() << ") does not match size of context buffer (" << m_ctxBuffer.size() << ")." );

  for( std::size_t k = 0; k < m_ctxBuffer.size(); k++ )
  {
    m_ctxBuffer[k].setAdaptRateOffset( rateOffset0[k], 0 );
    m_ctxBuffer[k].setAdaptRateOffset( rateOffset1[k], 1 );
  }
}
#endif

template <class BinProbModel>
void CtxStore<BinProbModel>::saveWinSizes( std::vector<uint8_t>& windows ) const
{
  windows.resize( m_ctxBuffer.size(), uint8_t( 0 ) );

  for( std::size_t k = 0; k < m_ctxBuffer.size(); k++ )
  {
    windows[k] = m_ctxBuffer[k].getWinSizes();
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::loadWinSizes( const std::vector<uint8_t>& windows )
{
  CHECK( m_ctxBuffer.size() != windows.size(),
         "Size of prob states table (" << windows.size() << ") does not match size of context buffer (" << m_ctxBuffer.size() << ")." );
  for( std::size_t k = 0; k < m_ctxBuffer.size(); k++ )
  {
    m_ctxBuffer[k].setWinSizes( windows[k] );
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::loadWeights( const std::vector<uint8_t>& weights )
{
  CHECK( m_ctxBuffer.size() != weights.size(),
         "Size of prob states table (" << weights.size() << ") does not match size of context buffer (" << m_ctxBuffer.size() << ")." );
  for( std::size_t k = 0; k < m_ctxBuffer.size(); k++ )
  {
    m_ctxBuffer[k].setAdaptRateWeight( weights[k] );
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::saveWeights( std::vector<uint8_t>& weights ) const
{
  weights.resize( m_ctxBuffer.size(), uint8_t( 0 ) );

  for( std::size_t k = 0; k < m_ctxBuffer.size(); k++ )
  {
    weights[k] = m_ctxBuffer[k].getAdaptRateWeight();
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::loadPStates( const std::vector<std::pair<uint16_t, uint16_t>>& probStates )
{
  CHECK( m_ctxBuffer.size() != probStates.size(),
         "Size of prob states table (" << probStates.size() << ") does not match size of context buffer (" << m_ctxBuffer.size() << ")." );
  for( std::size_t k = 0; k < m_ctxBuffer.size(); k++ )
  {
    m_ctxBuffer[k].setState( probStates[k] );
#if JVET_AG0196_CABAC_RETRAIN
    if (CabacRetrain::activate)
    {
      CabacRetrain::vprobaInit[k] = probStates[k];
      CabacRetrain::tempCABAC        = true;
    }
#endif
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::savePStates( std::vector<std::pair<uint16_t, uint16_t>>& probStates ) const
{
  probStates.resize( m_ctxBuffer.size(), std::pair<uint16_t, uint16_t>( 0, 0 ) );

  for( std::size_t k = 0; k < m_ctxBuffer.size(); k++ )
  {
    probStates[k] = m_ctxBuffer[k].getState();
  }
}
#else
template <class BinProbModel>
void CtxStore<BinProbModel>::loadPStates( const std::vector<uint16_t>& probStates )
{
  CHECK( m_ctxBuffer.size() != probStates.size(),
        "Size of prob states table (" << probStates.size() << ") does not match size of context buffer (" << m_ctxBuffer.size() << ")." );
  for( std::size_t k = 0; k < m_ctxBuffer.size(); k++ )
  {
    m_ctxBuffer[k].setState( probStates[k] );
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::savePStates( std::vector<uint16_t>& probStates ) const
{
  probStates.resize( m_ctxBuffer.size(), uint16_t(0) );
  for( std::size_t k = 0; k < m_ctxBuffer.size(); k++ )
  {
    probStates[k] = m_ctxBuffer[k].getState();
  }
}
#endif



template class CtxStore<BinProbModel_Std>;





Ctx::Ctx()                                  : m_BPMType( BPM_Undefined )                        {}
Ctx::Ctx( const BinProbModel_Std*   dummy ) : m_BPMType( BPM_Std   ), m_ctxStore_Std  ( true )  {}

Ctx::Ctx( const Ctx& ctx )
  : m_BPMType         ( ctx.m_BPMType )
  , m_ctxStore_Std    ( ctx.m_ctxStore_Std    )
{
  ::memcpy( m_GRAdaptStats, ctx.m_GRAdaptStats, sizeof( unsigned ) * RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS );
}

