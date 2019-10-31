/*
 * ar0820_tables.h - OnSemi AR0820 CMOS Sensor mode tables
 *
 * Copyright (c) 2018-2019, D3 Engineering.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/* Sequences copied from AR0820_MAX9295A_config10.ini */

#ifndef _AR0820_TABLES_
#define _AR0820_TABLES_


static const struct reg_sequence ar0820_mode_3848x2168[] = {
	{ 0x301A, 0x0059, 200000 }, // STATE= Sensor Reset,1
	{ 0x301A, 0x0058, 100000 }, // STATE= Sensor Reset,0
	// Disable streaming in serial mode; SERIALISER_DIS = 0, PARALLEL_EN = 0, DRIVE_PINS = 1, RESET_REGISTER_LOCK_REG=1
	// LOAD= Feb21_Sequencer
	// [Hidden: Feb21_Sequencer]
	// pix_timing_J_test4 from JIDC
	{ 0x2512, 0x8000 },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFF07 },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0xFFFF },
	{ 0x2510, 0x3001 },
	{ 0x2510, 0x3010 },
	{ 0x2510, 0x3006 },
	{ 0x2510, 0x3020 },
	{ 0x2510, 0x3008 },
	{ 0x2510, 0xB031 },
	{ 0x2510, 0xA824 },
	{ 0x2510, 0x003C },
	{ 0x2510, 0x001F },
	{ 0x2510, 0xB0F9 },
	{ 0x2510, 0x006D },
	{ 0x2510, 0x00EF },
	{ 0x2510, 0x005C },
	{ 0x2510, 0x106F },
	{ 0x2510, 0xC013 },
	{ 0x2510, 0x016E },
	{ 0x2510, 0xC806 },
	{ 0x2510, 0x106E },
	{ 0x2510, 0x0017 },
	{ 0x2510, 0x0013 },
	{ 0x2510, 0x004B },
	{ 0x2510, 0x0002 },
	{ 0x2510, 0x90F2 },
	{ 0x2510, 0x90FF },
	{ 0x2510, 0xD034 },
	{ 0x2510, 0x1032 },
	{ 0x2510, 0x0000 },
	{ 0x2510, 0x0033 },
	{ 0x2510, 0x00D1 },
	{ 0x2510, 0x092E },
	{ 0x2510, 0x1333 },
	{ 0x2510, 0x123D },
	{ 0x2510, 0x045B },
	{ 0x2510, 0x11BB },
	{ 0x2510, 0x133A },
	{ 0x2510, 0x907D },
	{ 0x2510, 0x1017 },
	{ 0x2510, 0x1115 },
	{ 0x2510, 0x14DB },
	{ 0x2510, 0x00DD },
	{ 0x2510, 0x3088 },
	{ 0x2510, 0x3084 },
	{ 0x2510, 0x2007 },
	{ 0x2510, 0x02DA },
	{ 0x2510, 0xD80C },
	{ 0x2510, 0x2009 },
	{ 0x2510, 0x01F0 },
	{ 0x2510, 0x14F0 },
	{ 0x2510, 0x018B },
	{ 0x2510, 0x128B },
	{ 0x2510, 0x00E4 },
	{ 0x2510, 0x0072 },
	{ 0x2510, 0x203B },
	{ 0x2510, 0x8A28 },
	{ 0x2510, 0x10CC },
	{ 0x2510, 0xC02A },
	{ 0x2510, 0x1064 },
	{ 0x2510, 0x0063 },
	{ 0x2510, 0x1072 },
	{ 0x2510, 0x06BE },
	{ 0x2510, 0x006E },
	{ 0x2510, 0x100E },
	{ 0x2510, 0x0019 },
	{ 0x2510, 0x0015 },
	{ 0x2510, 0x16EE },
	{ 0x2510, 0x0071 },
	{ 0x2510, 0x10BE },
	{ 0x2510, 0x1063 },
	{ 0x2510, 0x1671 },
	{ 0x2510, 0x1095 },
	{ 0x2510, 0x1019 },
	{ 0x2510, 0x3088 },
	{ 0x2510, 0x3084 },
	{ 0x2510, 0x2003 },
	{ 0x2510, 0x018B },
	{ 0x2510, 0x128B },
	{ 0x2510, 0x00E4 },
	{ 0x2510, 0x0072 },
	{ 0x2510, 0x20C4 },
	{ 0x2510, 0x10E4 },
	{ 0x2510, 0x1072 },
	{ 0x2510, 0x3041 },
	{ 0x2510, 0xD800 },
	{ 0x2510, 0x000A },
	{ 0x2510, 0x100C },
	{ 0x2510, 0x008E },
	{ 0x2510, 0x3081 },
	{ 0x2510, 0x10CB },
	{ 0x2510, 0x10D2 },
	{ 0x2510, 0xC200 },
	{ 0x2510, 0xCA00 },
	{ 0x2510, 0xD230 },
	{ 0x2510, 0x8200 },
	{ 0x2510, 0x11AE },
	{ 0x2510, 0x1039 },
	{ 0x2510, 0xD000 },
	{ 0x2510, 0x106D },
	{ 0x2510, 0x101F },
	{ 0x2510, 0x100E },
	{ 0x2510, 0x100A },
	{ 0x2510, 0x3042 },
	{ 0x2510, 0x3086 },
	{ 0x2510, 0x102F },
	{ 0x2510, 0x3090 },
	{ 0x2510, 0x9010 },
	{ 0x2510, 0xB000 },
	{ 0x2510, 0x30A0 },
	{ 0x2510, 0x1016 },
	{ 0x2510, 0x7FFF },
	{ 0x2510, 0x7FFF },
	{ 0x2510, 0x7FFF },
	{ 0x2510, 0x7FFF },
	{ 0x2510, 0x7FFF },
	{ 0x2510, 0x7FFF },
	{ 0x2510, 0x7FFF },
	// LOAD= Recommended Settings - No Bin
	// [Hidden: Recommended Settings - No Bin]
	// Char recommended settings

	// IF_0x31FE 0x0C00, ==1, ; LOAD= 1v1 CHAR No bin, ELSE; LOAD= 1v2 CHAR No bin
	// Only one of the following two sections should be applied, depending on rev of sensor:

	// [Hidden: 1v1 CHAR No bin] Rev 1.1 silicon
	// 0x350A 0xC1C1		; Vtxhi=3.3V, Vrsthi=3.3V
	// 0x350C 0xC1C1		; Vrshi and Vdcghi 3.3V
	// 0x350E 0x8D8D		; Vtxlo_x=-0.9V, Vtxlo_ro=-0.9V,
	// 0x3510 0x8D88		; Vsublo=-0.9V,
	// 0x3512 0x8C8C		; Vrstlo_ro and Vrstlo_ro  0.8V
	// 0x3514 0xA0A0		; Vdcglo_ro and Vdcglo_x=0V

	// [Hidden: 1v2 CHAR No bin] Rev 1.2 silicon
	{ 0x350A, 0xC1C0 },	// Vtxhi=2.8V, Vrsthi=3.3V
	{ 0x350C, 0xC1C1 },	// Vrshi and Vdcghi 3.3V
	{ 0x350E, 0x8C8C },	// Vtxlo_x=-0.8V, Vtxlo_ro=-0.8V,
	{ 0x3510, 0x8C88 },	// Vsublo=-0.8V
	{ 0x3512, 0x8C8C },	// Vrstlo_ro and Vrstlo_ro  0.8V
	{ 0x3514, 0xA0A0 },	// Vdcglo_ro and Vdcglo_x=0V
	{ 0x3518, 0x0000 },	// row_latch_en_tx_act and row_vaapix_en_rst disabled

	{ 0x351A, 0x8600 },	// GREF_SUPPRESS_TX_SAMP=1
	{ 0x3506, 0x004A },	// vln 5uA
	{ 0x3520, 0x1422 },	// ECL setting HCG=34, LCG=20
	{ 0x3522, 0x3E37 },	// FSC setting HCG_8x=55, HCG_4x=62
	{ 0x3524, 0x6E48 },	// FSC setting HCG_2x=72, HCG_1x=110
	{ 0x3526, 0x4237 },	// FSC setting LCG_8x=55, LCG_4x=66
	{ 0x3528, 0x6249 },	// FSC setting LCG_2x=73, LCG_1x=98
	{ 0x30FE, 0x00A8 },	// noise pedestal 168
	{ 0x342A, 0x0011 },	// CRM
	{ 0x3584, 0x0000 },	// row_ab_ctrl=0
	{ 0x354C, 0x0030 },	// pwr_dn_clamp=0, ECL off
	{ 0x3370, 0x0111 },	// DBLC gaintrig enabled
	{ 0x337A, 0x0E74 },	// scaling factor ~0.9
	{ 0x3110, 0x0011 },	// pre_hdr_gain_enabled to help soft eclipse in HDR mode
	{ 0x3100, 0x4000 },	// enable noise filtering
	// Design recommended settings dated 23 Jan 2018
	{ 0x33FC, 0x00E4 },	// pixel color indication, do not change
	{ 0x33FE, 0x00E4 },	// cb
	{ 0x301E, 0x00A8 },	// data_pedestal
	{ 0x3180, 0x0021 },	// DBLC dither enabled
	{ 0x3372, 0x710F },	// DBLC_fs0_control, dblc_scale_en
	{ 0x3E4C, 0x0404 },	// DBLC filter settings
	{ 0x3180, 0x0021 },	// DBLC_DITHER_EN
	{ 0x37a0, 0x0001 },	// coarse_integration_ad_time
	{ 0x37a4, 0x0000 },	// coarse_integration_ad_time2
	{ 0x37a8, 0x0000 },	// coarse_integration_ad_time3
	{ 0x37ac, 0x0001 },	// coarse_integration_ad_time4

	// LOAD= f2_pll_setup_800M
	// [Hidden: f2_pll_setup_800M]
	{ 0x302A, 0x0005 },	// vtpixclk p2 div
	{ 0x302C, 0x0701 },	// vtsysclk p1 div
	{ 0x302E, 0x0009 },	// pll pre divider
	{ 0x3030, 0x0086 },	// pll multiplier
	{ 0x3036, 0x0006 },	// op word clk div
	{ 0x3038, 0x0001 },	// op sys clk div
	{ 0x303A, 0x00CD },	// pll multiplier Ana
	{ 0x303C, 0x0009 },	// pll pre div Ana

	// LOAD= m1_mipi_timing_recommended_800M
	// [Hidden: m1_mipi_timing_recommended_800M]
	{ 0x31B0, 0x005C },	// frame preamble
	{ 0x31B2, 0x0041 },	// line preamble
	{ 0x31B4, 0x1187 },	//  Mipi_timing_0
	{ 0x31B6, 0x120A },	//  Mipi_timing_1
	{ 0x31B8, 0x6049 },	//  Mipi_timing_2
	{ 0x31BA, 0x0289 },	//  Mipi_timing_3
	{ 0x31BC, 0x8A07 },	//  Mipi_timing_4

	// LOAD= a01_array_setup_3848_2168_skip1
	// [Hidden: a01_array_setup_3848_2168_skip1]
	{ 0x3002, 0x0000 },	// y_addr_start
	{ 0x3004, 0x0000 },	// x_addr_start
	{ 0x3006, 0x0877 },	// y_addr_end
	{ 0x3008, 0x0F07 },	// x_addr_end
	{ 0x32FC, 0x0000 },	// no binning
	{ 0x37E0, 0x8421 },
	{ 0x37E2, 0x8421 },	// cb
	{ 0x323C, 0x8421 },
	{ 0x323E, 0x8421 },	// cb
	{ 0x3040, 0x0001 },	// no mirror or flip and stats rows at 4
	{ 0x301D, 0x0000 },	// no mirror or flip

	{ 0x3082, 0x0000 },	// 0x3082 = 1 exposures
	{ 0x30BA, 0x1100 },	// Num_exp_max
	{ 0x3012, 0x0014 },	// coarse_integration_time was 0x0140
	{ 0x3014, 0x0000 },	// fine_integration_time
	{ 0x3362, 0x0001 },	// DCG =high
	{ 0x3366, 0x0000 },	// Gain =1x
	{ 0x336A, 0x0000 },	// Gain =1x

	{ 0x300C, 0x0cac },	// line_length_pclk was 0x0A08, min = 0x0cac
	{ 0x300A, 0x08F8 },	// frame_length_lines

	// LOAD= c1_corrections_recommended
	{ 0x31AE, 0x0204 },
	{ 0x31AC, 0x0C0C },

	// -------------- TEST PATTERN ------------
	{ 0x3070, 0x0000 },	// 0 = normal operation, 1 = solid color, 2 = color bar, 3 = fade to grey, 256 = walking 1's
	{ 0x3072, 0x0200 },	// Test data Red (for solid color)
	{ 0x3074, 0x0400 },	// Test data GreenR
	{ 0x3076, 0x0100 },	// Test data Blue
	{ 0x3078, 0x0400 },	// Test data GreenB
	// -------------- TEST PATTERN ------------

	// LOAD= Disable Embedded Data and Stats
	// [Hidden: Disable Embedded Data and Stats]
	// LOAD= Embedded Data off
	// [Hidden: Embedded Data off]
	// BITFIELD = 0x301A [0058], 0x04, 0; Streaming off
	{ 0x301A, 0x0058, 100000 },
	// BITFIELD = 0x3064 [0100], 0x0100, 0; Embedded data off
	{ 0x3064, 0x0000 },
	// BITFIELD = 0x301A [0058], 0x04, 1; Streaming on
	{ 0x301A, 0x005C },
	// LOAD= Embedded Stats off
	// [Hidden: Embedded Stats off]
	// BITFIELD = 0x301A [], 0x04, 0; Streaming off
	{ 0x301A, 0x0058, 100000 },
	// BITFIELD = 0x3064 [0000], 0x80, 0; Embedded stats off
	{ 0x3064, 0x0000 },
	// BITFIELD = 0x301A [], 0x04, 1; Streaming on
	{ 0x301A, 0x0058, 128000 }, // Disable streaming.
	// IMAGE= 3848,2168, BAYER-12
};

static const struct reg_sequence ar0820_frame_sync[] = {
	{ 0x30CE, 0x0120 },  // Slave Mode: Deterministic Trigger
	{ 0x340A, 0x0077 },  // GPIO3: Input enable, output disable
	{ 0x340C, 0x0080 },  // GPIO3: Trigger Input
};

static const struct reg_sequence ar0820_start[] = {
	{ 0x301A, 0x015C, 128000 },
};

static const struct reg_sequence ar0820_stop[] = {
	{ 0x0301A, 0x0058, 100000 },
};

static const struct reg_sequence ar0820_test_pattern[] = {
	{ 0x3070, 0x0000, }, // 0 = normal operation, 1 = solid color, 2 = color bar, 3 = fade to grey, 256 = walking 1's
	{ 0x3072, 0x0200, }, // Test data Red (for solid color)
	{ 0x3074, 0x0400, }, // Test data GreenR
	{ 0x3076, 0x0100, }, // Test data Blue
	{ 0x3078, 0x0400, }, // Test data GreenB
};

enum ar0820_modes {
	AR0820_MODE_3848X2168 = 0,
	AR0820_MODE_START_STREAM,
	AR0820_MODE_STOP_STREAM,
	AR0820_MODE_TEST_PATTERN,
};

#if 0
static const struct reg_sequence *ar0820_mode_table[] = {
	[AR0820_MODE_3848X2168] = ar0820_mode_3848x2168,
	[AR0820_MODE_START_STREAM] = ar0820_start,
	[AR0820_MODE_STOP_STREAM] = ar0820_stop,
	[AR0820_MODE_TEST_PATTERN] = ar0820_test_pattern,
};
#endif

static const int ar0820_20fps[] = {
	20,
};


static const struct camera_common_frmfmt ar0820_frmfmt[] = {
	{
		.size = { 3848, 2168 },
		.framerates = ar0820_20fps,
		.num_framerates = 1,
		.hdr_en = 0,
		.mode = AR0820_MODE_3848X2168,
	},
};

#endif /* _AR0820_TABLES_ */

