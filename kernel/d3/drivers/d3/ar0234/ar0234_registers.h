/*
 * ar0234_registers.c - ar0234 register map ranges
 *
 * Copyright (c) 2022, D3 Engineering. All rights reserved.
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

#ifndef AR0234_REGISTERS_H
#define AR0234_REGISTERS_H

const struct regmap_range ar0234_regmap_yes_ranges[] = {
	regmap_reg_range(0x3000, 0x3000), // CHIP_VERSION_REG chip_version_reg
	regmap_reg_range(0x3002, 0x3002), // Y_ADDR_START y_addr_start
	regmap_reg_range(0x3004, 0x3004), // X_ADDR_START x_addr_start
	regmap_reg_range(0x3006, 0x3006), // Y_ADDR_END y_addr_end
	regmap_reg_range(0x3008, 0x3008), // X_ADDR_END x_addr_end
	regmap_reg_range(0x300A, 0x300A), // FRAME_LENGTH_LINES frame_length_lines
	regmap_reg_range(0x300C, 0x300C), // LINE_LENGTH_PCK line_length_pck
	regmap_reg_range(0x300E, 0x300E), // REVISION_NUMBER revision_number
	regmap_reg_range(0x3010, 0x3010), // LOCK_CONTROL lock_control
	regmap_reg_range(0x3012, 0x3012), // COARSE_INTEGRATION_TIME coarse_integration_time
	regmap_reg_range(0x3014, 0x3014), // FINE_INTEGRATION_TIME fine_integration_time
	regmap_reg_range(0x3016, 0x3016), // COARSE_INTEGRATION_TIME_CB coarse_integration_time_cb
	regmap_reg_range(0x3018, 0x3018), // FINE_INTEGRATION_TIME_CB fine_integration_time_cb
	regmap_reg_range(0x301A, 0x301A), // RESET_REGISTER reset_register
	regmap_reg_range(0x301C, 0x301C), // MODE_SELECT_ mode_select_
	regmap_reg_range(0x301D, 0x301D), // IMAGE_ORIENTATION_ image_orientation_
	regmap_reg_range(0x301E, 0x301E), // DATA_PEDESTAL data_pedestal
	regmap_reg_range(0x3021, 0x3021), // SOFTWARE_RESET_ software_reset_
	regmap_reg_range(0x3022, 0x3022), // GROUPED_PARAMETER_HOLD_ grouped_parameter_hold_
	regmap_reg_range(0x3023, 0x3023), // MASK_CORRUPTED_FRAMES_ mask_corrupted_frames_
	regmap_reg_range(0x3024, 0x3024), // PIXEL_ORDER_ pixel_order_
	regmap_reg_range(0x3026, 0x3026), // GPI_STATUS gpi_status
	regmap_reg_range(0x3028, 0x3028), // ROW_SPEED row_speed
	regmap_reg_range(0x302A, 0x302A), // VT_PIX_CLK_DIV vt_pix_clk_div
	regmap_reg_range(0x302C, 0x302C), // VT_SYS_CLK_DIV vt_sys_clk_div
	regmap_reg_range(0x302E, 0x302E), // PRE_PLL_CLK_DIV pre_pll_clk_div
	regmap_reg_range(0x3030, 0x3030), // PLL_MULTIPLIER pll_multiplier
	regmap_reg_range(0x3034, 0x3034), // CTX_CONTROL_REG ctx_control_reg
	regmap_reg_range(0x3036, 0x3036), // OP_PIX_CLK_DIV op_pix_clk_div
	regmap_reg_range(0x3038, 0x3038), // OP_SYS_CLK_DIV op_sys_clk_div
	regmap_reg_range(0x303A, 0x303A), // FRAME_COUNT frame_count
	regmap_reg_range(0x303C, 0x303C), // FRAME_STATUS frame_status
	regmap_reg_range(0x303E, 0x303E), // LINE_LENGTH_PCK_CB line_length_pck_cb
	regmap_reg_range(0x3040, 0x3040), // READ_MODE read_mode
	regmap_reg_range(0x3042, 0x3042), // EXTRA_DELAY extra_delay
	regmap_reg_range(0x3056, 0x3056), // GREEN1_GAIN green1_gain
	regmap_reg_range(0x3058, 0x3058), // BLUE_GAIN blue_gain
	regmap_reg_range(0x305A, 0x305A), // RED_GAIN red_gain
	regmap_reg_range(0x305C, 0x305C), // GREEN2_GAIN green2_gain
	regmap_reg_range(0x305E, 0x305E), // GLOBAL_GAIN global_gain
	regmap_reg_range(0x3060, 0x3060), // ANALOG_GAIN analog_gain
	regmap_reg_range(0x3064, 0x3064), // SMIA_TEST smia_test
	regmap_reg_range(0x3066, 0x3066), // CTX_WR_DATA_REG ctx_wr_data_reg
	regmap_reg_range(0x3068, 0x3068), // CTX_RD_DATA_REG ctx_rd_data_reg
	regmap_reg_range(0x306E, 0x306E), // DATAPATH_SELECT datapath_select
	regmap_reg_range(0x3070, 0x3070), // TEST_PATTERN_MODE test_pattern_mode
	regmap_reg_range(0x3072, 0x3072), // TEST_DATA_RED test_data_red
	regmap_reg_range(0x3074, 0x3074), // TEST_DATA_GREENR test_data_greenr
	regmap_reg_range(0x3076, 0x3076), // TEST_DATA_BLUE test_data_blue
	regmap_reg_range(0x3078, 0x3078), // TEST_DATA_GREENB test_data_greenb
	regmap_reg_range(0x3082, 0x3082), // OPERATION_MODE_CTRL operation_mode_ctrl
	regmap_reg_range(0x3084, 0x3084), // OPERATION_MODE_CTRL_CB operation_mode_ctrl_cb
	regmap_reg_range(0x3086, 0x3086), // SEQ_DATA_PORT seq_data_port
	regmap_reg_range(0x3088, 0x3088), // SEQ_CTRL_PORT seq_ctrl_port
	regmap_reg_range(0x308A, 0x308A), // X_ADDR_START_CB x_addr_start_cb
	regmap_reg_range(0x308C, 0x308C), // Y_ADDR_START_CB y_addr_start_cb
	regmap_reg_range(0x308E, 0x308E), // X_ADDR_END_CB x_addr_end_cb
	regmap_reg_range(0x3090, 0x3090), // Y_ADDR_END_CB y_addr_end_cb
	regmap_reg_range(0x30A0, 0x30A0), // X_EVEN_INC x_even_inc
	regmap_reg_range(0x30A2, 0x30A2), // X_ODD_INC x_odd_inc
	regmap_reg_range(0x30A4, 0x30A4), // Y_EVEN_INC y_even_inc
	regmap_reg_range(0x30A6, 0x30A6), // Y_ODD_INC y_odd_inc
	regmap_reg_range(0x30A8, 0x30A8), // Y_ODD_INC_CB y_odd_inc_cb
	regmap_reg_range(0x30AA, 0x30AA), // FRAME_LENGTH_LINES_CB frame_length_lines_cb
	regmap_reg_range(0x30AE, 0x30AE), // X_ODD_INC_CB x_odd_inc_cb
	regmap_reg_range(0x30B0, 0x30B0), // DIGITAL_TEST digital_test
	regmap_reg_range(0x30B2, 0x30B2), // TEMPSENS_DATA_REG tempsens_data_reg
	regmap_reg_range(0x30B4, 0x30B4), // TEMPSENS_CTRL_REG tempsens_ctrl_reg
	regmap_reg_range(0x30BA, 0x30BA), // RESERVED_MFR_30BA !! Required for gain control
	regmap_reg_range(0x30BC, 0x30BC), // GREEN1_GAIN_CB green1_gain_cb
	regmap_reg_range(0x30BE, 0x30BE), // BLUE_GAIN_CB blue_gain_cb
	regmap_reg_range(0x30C0, 0x30C0), // RED_GAIN_CB red_gain_cb
	regmap_reg_range(0x30C2, 0x30C2), // GREEN2_GAIN_CB green2_gain_cb
	regmap_reg_range(0x30C4, 0x30C4), // GLOBAL_GAIN_CB global_gain_cb
	regmap_reg_range(0x30C6, 0x30C6), // TEMPSENS_CALIB1 tempsens_calib1
	regmap_reg_range(0x30C8, 0x30C8), // TEMPSENS_CALIB2 tempsens_calib2
	regmap_reg_range(0x30CE, 0x30CE), // GRR_CONTROL1 grr_control1
	regmap_reg_range(0x30FE, 0x30FE), // NOISE_PEDESTAL noise_pedestal
	regmap_reg_range(0x3100, 0x3100), // AECTRLREG aectrlreg
	regmap_reg_range(0x3102, 0x3102), // AE_LUMA_TARGET_REG ae_luma_target_reg
	regmap_reg_range(0x3108, 0x3108), // AE_MIN_EV_STEP_REG ae_min_ev_step_reg
	regmap_reg_range(0x310A, 0x310A), // AE_MAX_EV_STEP_REG ae_max_ev_step_reg
	regmap_reg_range(0x310C, 0x310C), // AE_DAMP_OFFSET_REG ae_damp_offset_reg
	regmap_reg_range(0x310E, 0x310E), // AE_DAMP_GAIN_REG ae_damp_gain_reg
	regmap_reg_range(0x3110, 0x3110), // AE_DAMP_MAX_REG ae_damp_max_reg
	regmap_reg_range(0x311C, 0x311C), // AE_MAX_EXPOSURE_REG ae_max_exposure_reg
	regmap_reg_range(0x311E, 0x311E), // AE_MIN_EXPOSURE_REG ae_min_exposure_reg
	regmap_reg_range(0x3124, 0x3124), // AE_DARK_CUR_THRESH_REG ae_dark_cur_thresh_reg
	regmap_reg_range(0x312A, 0x312A), // AE_CURRENT_GAINS ae_current_gains
	regmap_reg_range(0x312E, 0x312E), // ACTUAL_FRAME_LENGTH_LINES actual_frame_length_lines
	regmap_reg_range(0x3140, 0x3140), // AE_ROI_X_START_OFFSET ae_roi_x_start_offset
	regmap_reg_range(0x3142, 0x3142), // AE_ROI_Y_START_OFFSET ae_roi_y_start_offset
	regmap_reg_range(0x3144, 0x3144), // AE_ROI_X_SIZE ae_roi_x_size
	regmap_reg_range(0x3146, 0x3146), // AE_ROI_Y_SIZE ae_roi_y_size
	regmap_reg_range(0x3148, 0x3148), // AE_HIST_BEGIN_PERC ae_hist_begin_perc
	regmap_reg_range(0x314A, 0x314A), // AE_HIST_END_PERC ae_hist_end_perc
	regmap_reg_range(0x314C, 0x314C), // AE_HIST_DIV ae_hist_div
	regmap_reg_range(0x314E, 0x314E), // AE_NORM_WIDTH_MIN ae_norm_width_min
	regmap_reg_range(0x3150, 0x3150), // AE_MEAN_H ae_mean_h
	regmap_reg_range(0x3152, 0x3152), // AE_MEAN_L ae_mean_l
	regmap_reg_range(0x3154, 0x3154), // AE_HIST_BEGIN_H ae_hist_begin_h
	regmap_reg_range(0x3156, 0x3156), // AE_HIST_BEGIN_L ae_hist_begin_l
	regmap_reg_range(0x3158, 0x3158), // AE_HIST_END_H ae_hist_end_h
	regmap_reg_range(0x315A, 0x315A), // AE_HIST_END_L ae_hist_end_l
	regmap_reg_range(0x315C, 0x315C), // AE_LOW_END_MEAN_H ae_low_end_mean_h
	regmap_reg_range(0x315E, 0x315E), // AE_LOW_END_MEAN_L ae_low_end_mean_l
	regmap_reg_range(0x3160, 0x3160), // AE_PERC_LOW_END ae_perc_low_end
	regmap_reg_range(0x3162, 0x3162), // AE_NORM_ABS_DEV ae_norm_abs_dev
	regmap_reg_range(0x3164, 0x3164), // AE_COARSE_INTEGRATION_TIME ae_coarse_integration_time
	regmap_reg_range(0x3166, 0x3166), // AE_AG_EXPOSURE_HI ae_ag_exposure_hi
	regmap_reg_range(0x3168, 0x3168), // AE_AG_EXPOSURE_LO ae_ag_exposure_lo
	regmap_reg_range(0x3180, 0x3180), // DELTA_DK_CONTROL delta_dk_control
	regmap_reg_range(0x31AC, 0x31AC), // DATA_FORMAT_BITS data_format_bits
	regmap_reg_range(0x31AE, 0x31AE), // SERIAL_FORMAT serial_format
	regmap_reg_range(0x31B0, 0x31B0), // FRAME_PREAMBLE frame_preamble
	regmap_reg_range(0x31B2, 0x31B2), // LINE_PREAMBLE line_preamble
	regmap_reg_range(0x31B4, 0x31B4), // MIPI_TIMING_0 mipi_timing_0
	regmap_reg_range(0x31B6, 0x31B6), // MIPI_TIMING_1 mipi_timing_1
	regmap_reg_range(0x31B8, 0x31B8), // MIPI_TIMING_2 mipi_timing_2
	regmap_reg_range(0x31BA, 0x31BA), // MIPI_TIMING_3 mipi_timing_3
	regmap_reg_range(0x31BC, 0x31BC), // MIPI_TIMING_4 mipi_timing_4
	regmap_reg_range(0x31BE, 0x31BE), // SERIAL_CONFIG_STATUS serial_config_status
	regmap_reg_range(0x31C6, 0x31C6), // SERIAL_CONTROL_STATUS serial_control_status
	regmap_reg_range(0x31C8, 0x31C8), // SERIAL_CRC_0 serial_crc_0
	regmap_reg_range(0x31D0, 0x31D0), // COMPANDING companding
	regmap_reg_range(0x31D2, 0x31D2), // STAT_FRAME_ID stat_frame_id
	regmap_reg_range(0x31D6, 0x31D6), // I2C_WRT_CHECKSUM i2c_wrt_checksum
	regmap_reg_range(0x31D8, 0x31D8), // SERIAL_TEST serial_test
	regmap_reg_range(0x31E0, 0x31E0), // PIX_DEF_ID pix_def_id
	regmap_reg_range(0x31E8, 0x31E8), // HORIZONTAL_CURSOR_POSITION horizontal_cursor_position
	regmap_reg_range(0x31EA, 0x31EA), // VERTICAL_CURSOR_POSITION vertical_cursor_position
	regmap_reg_range(0x31EC, 0x31EC), // HORIZONTAL_CURSOR_WIDTH horizontal_cursor_width
	regmap_reg_range(0x31EE, 0x31EE), // VERTICAL_CURSOR_WIDTH vertical_cursor_width
	regmap_reg_range(0x31F0, 0x31F0), // AE_STATS_CONTROL ae_stats_control
	regmap_reg_range(0x31FC, 0x31FC), // CCI_IDS cci_ids
	regmap_reg_range(0x31FE, 0x31FE), // CUSTOMER_REV customer_rev
	regmap_reg_range(0x3240, 0x3240), // AE_X1_START_OFFSET ae_x1_start_offset
	regmap_reg_range(0x3242, 0x3242), // AE_X2_START_OFFSET ae_x2_start_offset
	regmap_reg_range(0x3244, 0x3244), // AE_X3_START_OFFSET ae_x3_start_offset
	regmap_reg_range(0x3246, 0x3246), // AE_X4_START_OFFSET ae_x4_start_offset
	regmap_reg_range(0x3248, 0x3248), // AE_Y1_START_OFFSET ae_y1_start_offset
	regmap_reg_range(0x324A, 0x324A), // AE_Y2_START_OFFSET ae_y2_start_offset
	regmap_reg_range(0x324C, 0x324C), // AE_Y3_START_OFFSET ae_y3_start_offset
	regmap_reg_range(0x324E, 0x324E), // AE_Y4_START_OFFSET ae_y4_start_offset
	regmap_reg_range(0x3250, 0x3250), // AE_GRID_SEL_LO ae_grid_sel_lo
	regmap_reg_range(0x3252, 0x3252), // AE_GRID_SEL_HI ae_grid_sel_hi
	regmap_reg_range(0x3270, 0x3270), // LED_FLASH_CONTROL led_flash_control
	regmap_reg_range(0x3338, 0x3338), // MIPI_TEST_CNTRL mipi_test_cntrl
	regmap_reg_range(0x333A, 0x333A), // MIPI_COMPRESS_8_DATA_TYPE mipi_compress_8_data_type
	regmap_reg_range(0x333C, 0x333C), // MIPI_COMPRESS_7_DATA_TYPE mipi_compress_7_data_type
	regmap_reg_range(0x333E, 0x333E), // MIPI_COMPRESS_6_DATA_TYPE mipi_compress_6_data_type
	regmap_reg_range(0x3340, 0x3340), // MIPI_JPEG_PN9_DATA_TYPE mipi_jpeg_pn9_data_type
	regmap_reg_range(0x3354, 0x3354), // MIPI_CNTRL mipi_cntrl
	regmap_reg_range(0x3356, 0x3356), // MIPI_TEST_PATTERN_CNTRL mipi_test_pattern_cntrl
	regmap_reg_range(0x3358, 0x3358), // MIPI_TEST_PATTERN_STATUS mipi_test_pattern_status
	regmap_reg_range(0x3600, 0x3600), // P_GR_P0Q0 p_gr_p0q0
	regmap_reg_range(0x3602, 0x3602), // P_GR_P0Q1 p_gr_p0q1
	regmap_reg_range(0x3604, 0x3604), // P_GR_P0Q2 p_gr_p0q2
	regmap_reg_range(0x3606, 0x3606), // P_GR_P0Q3 p_gr_p0q3
	regmap_reg_range(0x3608, 0x3608), // P_GR_P0Q4 p_gr_p0q4
	regmap_reg_range(0x360A, 0x360A), // P_RD_P0Q0 p_rd_p0q0
	regmap_reg_range(0x360C, 0x360C), // P_RD_P0Q1 p_rd_p0q1
	regmap_reg_range(0x360E, 0x360E), // P_RD_P0Q2 p_rd_p0q2
	regmap_reg_range(0x3610, 0x3610), // P_RD_P0Q3 p_rd_p0q3
	regmap_reg_range(0x3612, 0x3612), // P_RD_P0Q4 p_rd_p0q4
	regmap_reg_range(0x3614, 0x3614), // P_BL_P0Q0 p_bl_p0q0
	regmap_reg_range(0x3616, 0x3616), // P_BL_P0Q1 p_bl_p0q1
	regmap_reg_range(0x3618, 0x3618), // P_BL_P0Q2 p_bl_p0q2
	regmap_reg_range(0x361A, 0x361A), // P_BL_P0Q3 p_bl_p0q3
	regmap_reg_range(0x361C, 0x361C), // P_BL_P0Q4 p_bl_p0q4
	regmap_reg_range(0x361E, 0x361E), // P_GB_P0Q0 p_gb_p0q0
	regmap_reg_range(0x3620, 0x3620), // P_GB_P0Q1 p_gb_p0q1
	regmap_reg_range(0x3622, 0x3622), // P_GB_P0Q2 p_gb_p0q2
	regmap_reg_range(0x3624, 0x3624), // P_GB_P0Q3 p_gb_p0q3
	regmap_reg_range(0x3626, 0x3626), // P_GB_P0Q4 p_gb_p0q4
	regmap_reg_range(0x3640, 0x3640), // P_GR_P1Q0 p_gr_p1q0
	regmap_reg_range(0x3642, 0x3642), // P_GR_P1Q1 p_gr_p1q1
	regmap_reg_range(0x3644, 0x3644), // P_GR_P1Q2 p_gr_p1q2
	regmap_reg_range(0x3646, 0x3646), // P_GR_P1Q3 p_gr_p1q3
	regmap_reg_range(0x3648, 0x3648), // P_GR_P1Q4 p_gr_p1q4
	regmap_reg_range(0x364A, 0x364A), // P_RD_P1Q0 p_rd_p1q0
	regmap_reg_range(0x364C, 0x364C), // P_RD_P1Q1 p_rd_p1q1
	regmap_reg_range(0x364E, 0x364E), // P_RD_P1Q2 p_rd_p1q2
	regmap_reg_range(0x3650, 0x3650), // P_RD_P1Q3 p_rd_p1q3
	regmap_reg_range(0x3652, 0x3652), // P_RD_P1Q4 p_rd_p1q4
	regmap_reg_range(0x3654, 0x3654), // P_BL_P1Q0 p_bl_p1q0
	regmap_reg_range(0x3656, 0x3656), // P_BL_P1Q1 p_bl_p1q1
	regmap_reg_range(0x3658, 0x3658), // P_BL_P1Q2 p_bl_p1q2
	regmap_reg_range(0x365A, 0x365A), // P_BL_P1Q3 p_bl_p1q3
	regmap_reg_range(0x365C, 0x365C), // P_BL_P1Q4 p_bl_p1q4
	regmap_reg_range(0x365E, 0x365E), // P_GB_P1Q0 p_gb_p1q0
	regmap_reg_range(0x3660, 0x3660), // P_GB_P1Q1 p_gb_p1q1
	regmap_reg_range(0x3662, 0x3662), // P_GB_P1Q2 p_gb_p1q2
	regmap_reg_range(0x3664, 0x3664), // P_GB_P1Q3 p_gb_p1q3
	regmap_reg_range(0x3666, 0x3666), // P_GB_P1Q4 p_gb_p1q4
	regmap_reg_range(0x3680, 0x3680), // P_GR_P2Q0 p_gr_p2q0
	regmap_reg_range(0x3682, 0x3682), // P_GR_P2Q1 p_gr_p2q1
	regmap_reg_range(0x3684, 0x3684), // P_GR_P2Q2 p_gr_p2q2
	regmap_reg_range(0x3686, 0x3686), // P_GR_P2Q3 p_gr_p2q3
	regmap_reg_range(0x3688, 0x3688), // P_GR_P2Q4 p_gr_p2q4
	regmap_reg_range(0x368A, 0x368A), // P_RD_P2Q0 p_rd_p2q0
	regmap_reg_range(0x368C, 0x368C), // P_RD_P2Q1 p_rd_p2q1
	regmap_reg_range(0x368E, 0x368E), // P_RD_P2Q2 p_rd_p2q2
	regmap_reg_range(0x3690, 0x3690), // P_RD_P2Q3 p_rd_p2q3
	regmap_reg_range(0x3692, 0x3692), // P_RD_P2Q4 p_rd_p2q4
	regmap_reg_range(0x3694, 0x3694), // P_BL_P2Q0 p_bl_p2q0
	regmap_reg_range(0x3696, 0x3696), // P_BL_P2Q1 p_bl_p2q1
	regmap_reg_range(0x3698, 0x3698), // P_BL_P2Q2 p_bl_p2q2
	regmap_reg_range(0x369A, 0x369A), // P_BL_P2Q3 p_bl_p2q3
	regmap_reg_range(0x369C, 0x369C), // P_BL_P2Q4 p_bl_p2q4
	regmap_reg_range(0x369E, 0x369E), // P_GB_P2Q0 p_gb_p2q0
	regmap_reg_range(0x36A0, 0x36A0), // P_GB_P2Q1 p_gb_p2q1
	regmap_reg_range(0x36A2, 0x36A2), // P_GB_P2Q2 p_gb_p2q2
	regmap_reg_range(0x36A4, 0x36A4), // P_GB_P2Q3 p_gb_p2q3
	regmap_reg_range(0x36A6, 0x36A6), // P_GB_P2Q4 p_gb_p2q4
	regmap_reg_range(0x36C0, 0x36C0), // P_GR_P3Q0 p_gr_p3q0
	regmap_reg_range(0x36C2, 0x36C2), // P_GR_P3Q1 p_gr_p3q1
	regmap_reg_range(0x36C4, 0x36C4), // P_GR_P3Q2 p_gr_p3q2
	regmap_reg_range(0x36C6, 0x36C6), // P_GR_P3Q3 p_gr_p3q3
	regmap_reg_range(0x36C8, 0x36C8), // P_GR_P3Q4 p_gr_p3q4
	regmap_reg_range(0x36CA, 0x36CA), // P_RD_P3Q0 p_rd_p3q0
	regmap_reg_range(0x36CC, 0x36CC), // P_RD_P3Q1 p_rd_p3q1
	regmap_reg_range(0x36CE, 0x36CE), // P_RD_P3Q2 p_rd_p3q2
	regmap_reg_range(0x36D0, 0x36D0), // P_RD_P3Q3 p_rd_p3q3
	regmap_reg_range(0x36D2, 0x36D2), // P_RD_P3Q4 p_rd_p3q4
	regmap_reg_range(0x36D4, 0x36D4), // P_BL_P3Q0 p_bl_p3q0
	regmap_reg_range(0x36D6, 0x36D6), // P_BL_P3Q1 p_bl_p3q1
	regmap_reg_range(0x36D8, 0x36D8), // P_BL_P3Q2 p_bl_p3q2
	regmap_reg_range(0x36DA, 0x36DA), // P_BL_P3Q3 p_bl_p3q3
	regmap_reg_range(0x36DC, 0x36DC), // P_BL_P3Q4 p_bl_p3q4
	regmap_reg_range(0x36DE, 0x36DE), // P_GB_P3Q0 p_gb_p3q0
	regmap_reg_range(0x36E0, 0x36E0), // P_GB_P3Q1 p_gb_p3q1
	regmap_reg_range(0x36E2, 0x36E2), // P_GB_P3Q2 p_gb_p3q2
	regmap_reg_range(0x36E4, 0x36E4), // P_GB_P3Q3 p_gb_p3q3
	regmap_reg_range(0x36E6, 0x36E6), // P_GB_P3Q4 p_gb_p3q4
	regmap_reg_range(0x3700, 0x3700), // P_GR_P4Q0 p_gr_p4q0
	regmap_reg_range(0x3702, 0x3702), // P_GR_P4Q1 p_gr_p4q1
	regmap_reg_range(0x3704, 0x3704), // P_GR_P4Q2 p_gr_p4q2
	regmap_reg_range(0x3706, 0x3706), // P_GR_P4Q3 p_gr_p4q3
	regmap_reg_range(0x3708, 0x3708), // P_GR_P4Q4 p_gr_p4q4
	regmap_reg_range(0x370A, 0x370A), // P_RD_P4Q0 p_rd_p4q0
	regmap_reg_range(0x370C, 0x370C), // P_RD_P4Q1 p_rd_p4q1
	regmap_reg_range(0x370E, 0x370E), // P_RD_P4Q2 p_rd_p4q2
	regmap_reg_range(0x3710, 0x3710), // P_RD_P4Q3 p_rd_p4q3
	regmap_reg_range(0x3712, 0x3712), // P_RD_P4Q4 p_rd_p4q4
	regmap_reg_range(0x3714, 0x3714), // P_BL_P4Q0 p_bl_p4q0
	regmap_reg_range(0x3716, 0x3716), // P_BL_P4Q1 p_bl_p4q1
	regmap_reg_range(0x3718, 0x3718), // P_BL_P4Q2 p_bl_p4q2
	regmap_reg_range(0x371A, 0x371A), // P_BL_P4Q3 p_bl_p4q3
	regmap_reg_range(0x371C, 0x371C), // P_BL_P4Q4 p_bl_p4q4
	regmap_reg_range(0x371E, 0x371E), // P_GB_P4Q0 p_gb_p4q0
	regmap_reg_range(0x3720, 0x3720), // P_GB_P4Q1 p_gb_p4q1
	regmap_reg_range(0x3722, 0x3722), // P_GB_P4Q2 p_gb_p4q2
	regmap_reg_range(0x3724, 0x3724), // P_GB_P4Q3 p_gb_p4q3
	regmap_reg_range(0x3726, 0x3726), // P_GB_P4Q4 p_gb_p4q4
	regmap_reg_range(0x3780, 0x3780), // POLY_SC_ENABLE poly_sc_enable
	regmap_reg_range(0x3782, 0x3782), // POLY_ORIGIN_C poly_origin_c
	regmap_reg_range(0x3784, 0x3784), // POLY_ORIGIN_R poly_origin_r
	regmap_reg_range(0x3786, 0x3786), // DIGITAL_CTRL_1 digital_ctrl_1
	regmap_reg_range(0x37C0, 0x37C0), // P_GR_Q5 p_gr_q5
	regmap_reg_range(0x37C2, 0x37C2), // P_RD_Q5 p_rd_q5
	regmap_reg_range(0x37C4, 0x37C4), // P_BL_Q5 p_bl_q5
	regmap_reg_range(0x37C6, 0x37C6), // P_GB_Q5 p_gb_q5
};

#endif /* !AR0234_REGISTERS_H */