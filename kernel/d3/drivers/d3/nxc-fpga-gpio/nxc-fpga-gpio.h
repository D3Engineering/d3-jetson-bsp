/*
 * Copyright (c) 2020, D3 Engineering  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 */

#ifndef NXC_FPGA_GPIO_H
#define NXC_FPGA_GPIO_H

#define NXC_ADD_GPIO_BANK(r) {r,0},{r,1},{r,2},{r,3},{r,4},{r,5},{r,6},{r,7}

#define NXC_REG_OFFSET_INPUT(reg_bank) ( reg_bank + 0 )
/* R, GPIO Input (7 - 0) */
#define NXC_REG_OFFSET_INTER_STAT(reg_bank) ( reg_bank + 1 )
/* R, GPIO Interrupt Status */
#define NXC_REG_OFFSET_INTER_MODE(reg_bank) ( reg_bank + 2 )
/* R/W, GPIO Interrupt Mode (1 = Edge, 0 = Level) */
#define NXC_REG_OFFSET_OUTPUT(reg_bank) ( reg_bank + 3 )
/* R/W, GPIO Output */
#define NXC_REG_OFFSET_OUTPUT_EN(reg_bank) ( reg_bank + 4 )
/* R/W, GPIO Output Enable */
#define NXC_REG_OFFSET_INTER_POL(reg_bank) ( reg_bank + 5 )
/* R/W, GPIO Interrupt Polarity (1 = rising edge or high level) */
#define NXC_REG_OFFSET_INTER_MASK(reg_bank) ( reg_bank + 6 )
/* R/W, GPIO Interrupt Mask (0 enables interrupt) */
#define NXC_REG_OFFSET_INTER_CLR(reg_bank) ( reg_bank + 7 )
/* W, GPIO Edge Interrupt Clear (Write 1 to clear) */
#define NXC_REG_OFFSET_INTER_EDGE(reg_bank) ( reg_bank + 8 )
/* R/W, GPIO Interrupt Both Edges */


struct nxc_fg {
	struct i2c_client *client;
	struct regmap *map;
	struct gpio_chip chip;
	struct nxc_gpio *gpio_table;
	unsigned int reset_gpio;
};

struct nxc_gpio {
	unsigned int reg_bank_start;
	unsigned int bit_offset;
};


#define NXC_BOARD_PCA_REG_COUNT 5
#define NXC_BOARD_PCA_REG_START 0xA3
#define NXC_FPGA_FW_REV_REG 0xA2

#define NXC_00C091001_FPGA_MIN_VERSION 0x12
#define NXC_00C091001_FPGA_MAX_VERSION 255
static int nxc_pca_ref_table_00c091001[] = {
	0x00, /* First 4 bits are unused */
	0x0c,
	0x09,
	0x10,
	0x01
};

#define NXC_276003001_FPGA_MIN_VERSION 0x3
#define NXC_276003001_FPGA_MAX_VERSION 255
static int nxc_pca_ref_table_276003001[] = {
	0x02, /* First 4 bits are unused */
	0x76,
	0x00,
	0x30,
	0x01
};


static struct nxc_gpio nxc_00c091001_gpio_table[] = {
	/* (0-13) J19 Expansion GPIO */
	{ 0x90, 2 }, /* FPGA_GPIO_0 */
	{ 0x90, 3 }, /* FPGA_GPIO_1 */
	{ 0x90, 4 }, /* FPGA_GPIO_2 */
	{ 0x90, 5 }, /* FPGA_GPIO_3 */
	{ 0x90, 6 }, /* FPGA_GPIO_4 */
	{ 0x90, 7 }, /* FPGA_GPIO_5 */
	NXC_ADD_GPIO_BANK(0x99), /* FPGA_GPIO_[6-13] */

	/* (14-17) J6 Buffered IO */
	{ 0x7e, 4 }, /* BUF_IO_FPGA_0 */
	{ 0x7e, 5 }, /* BUF_IO_FPGA_1 */
	{ 0x7e, 6 }, /* BUF_IO_FPGA_2 */
	{ 0x7e, 7 }, /* BUF_IO_FPGA_3 */

	/* (18-25) Bank 0x00 */
	NXC_ADD_GPIO_BANK(0x00),
	/* I2C2_M2M_EN
	 * SDS_SOM_INT
	 * I2C0_SDS_EN
	 * USB_HUB_RST
	 * CAN_TERM_SW
	 * SDS_HUB1_SW_SEH
	 * SDS_HUB1_SW_SEL
	 * I2C2_M2E_EN */

	/* (26-33) Bank 0x09 */
	NXC_ADD_GPIO_BANK(0x09),
	/* FPGA_SLEEP_SET
	 * SDS_HUB2_SW_FAULTn
	 * SDS_HUB2_SW_ST4n
	 * SDS_HUB2_SW_SEH
	 * SDS_HUB2_SW_SEL
	 * PCIE1_UART_WAKE
	 * FPGA_HB
	 * I2C2_M2M_EN */

	/* (34-41) Bank 0x12 */
	NXC_ADD_GPIO_BANK(0x12),
	/* SDS_HUB1_SW_FAULTn
	 * SDS_HUB1_SW_ST4n
	 * DP_PWR_EN
	 * DP_PWR_FAULT
	 * EXP_DONE_GP2
	 * EXP_INIT_GP3
	 * SDS_HUB0_SW_FAULTn
	 * SDS_HUB0_SW_ST4n */

	/* (42-49) Bank 0x1b */
	NXC_ADD_GPIO_BANK(0x1b),
	/* SDS_HUB0_SW_SEH
	 * SDS_HUB0_SW_SEL
	 * EN_VDD_SW_SDS[0-5] */

	/* (50-57) Bank 0x24 */
	NXC_ADD_GPIO_BANK(0x24),
	/* EN_VDD_SW_SDS[6-11]
	 * EN_VDD_SW_SDS6
	 * EXP_JTEN_GP5 */

	/* (58-65) Bank 0x2D */
	NXC_ADD_GPIO_BANK(0x2D), /* EN_VDD_SW_SDS[0-7] */

	/* (66-69) Bank 0x36 */
	{ 0x36, 0 }, /* EN_VDD_SW_SDS8 */
	{ 0x36, 1 }, /* EN_VDD_SW_SDS9 */
	{ 0x36, 2 }, /* EN_VDD_SW_SDS10 */
	{ 0x36, 3 }, /* EN_VDD_SW_SDS11 */

	/* (70-77) UB960 HUB0 GPIO */
	NXC_ADD_GPIO_BANK(0x3f), /* SDS_HUB0_GPIO[0-7] */

	/* (78-85) UB960 HUB1 GPIO */
	NXC_ADD_GPIO_BANK(0x48), /* SDS_HUB1_GPIO[0-7] */

	/* (86-93) UB960 HUB2 GPIO */
	NXC_ADD_GPIO_BANK(0x51), /* SDS_HUB2_GPIO[0-7] */

	/* (94-101) Bank 0x5A */
	NXC_ADD_GPIO_BANK(0x5a),
	/* SDS_HUB0_PDB
	 * SDS_HUB0_LOCK
	 * SDS_HUB1_PDB
	 * SDS_HUB1_LOCK
	 * SDS_HUB2_PDB
	 * SDS_HUB2_LOCK
	 * FPGA_PCLK_LOOP1A
	 * FPGA_PCLK_LOOP1B */

	/* (102-109) Bank 0x63 */
	NXC_ADD_GPIO_BANK(0x63), /* SOM_GPIO_[01-09] */

	/* (110-117) Bank 0x6C */
	NXC_ADD_GPIO_BANK(0x6c),
	/* SOM_GPIO_[10-13]
	 * M2E_SDIO_WAKE
	 * M2E_SDIO_RST
	 * SOM_CAM0_PWDN
	 * SOM_CAM1_PWDN */

	/* (118-125) Bank 0x75 */
	NXC_ADD_GPIO_BANK(0x75),
	/* FPGA_PCLK_LOOP0A
	 * FPGA_PCLK_LOOP0B
	 * PCIE0_DEVSLP
	 * SPI1_SOM_CS0
	 * SOM_FAN_TACH
	 * PCIE_SOM_RTC
	 * M2E_SDIO_WAKE
	 * M2E_SDIO_RST */

	/* (126-129) Bank 0x7E (excluding BUF_IO) */
	{ 0x7e, 0 }, /* M2E_SDIO_RST */
	{ 0x7e, 1 }, /* SDS_HUB1_SW_DIAG_EN */
	{ 0x7e, 2 }, /* SDS_HUB2_SW_DIAG_EN */
	{ 0x7e, 3 }, /* ADC_ISNS_ALERT */

	/* (130-133) Bank 0x87 */
	{ 0x87, 0 }, /* PCIE0_M2M_ALERT */
	{ 0x87, 1 }, /* M2M_SW_CTL */
	{ 0x87, 2 }, /* PCIE1_M2E_ALERT */
	{ 0x87, 3 }, /* M2E_SW_CTL */

	/* (134-135) Bank 0x90 */
	{ 0x90, 0 }, /* SOM_SLEEP_STATUS */
	{ 0x90, 1 }, /* SOM_GPIO_09 */
};


static struct nxc_gpio nxc_276003001_gpio_table[] = {
	/* (0-13) J19 Expansion GPIO */
	{ 0x90, 2 }, /* FPGA_GPIO_0 */
	{ 0x90, 3 }, /* FPGA_GPIO_1 */
	{ 0x90, 4 }, /* FPGA_GPIO_2 */
	{ 0x90, 5 }, /* FPGA_GPIO_3 */
	{ 0x90, 6 }, /* FPGA_GPIO_4 */
	{ 0x90, 7 }, /* FPGA_GPIO_5 */
	NXC_ADD_GPIO_BANK(0x99), /* FPGA_GPIO_[6-13] */

	/* (14-17) J6 Buffered IO */
	{ 0x7e, 4 }, /* BUF_IO_FPGA_0 */
	{ 0x7e, 5 }, /* BUF_IO_FPGA_1 */
	{ 0x7e, 6 }, /* BUF_IO_FPGA_2 */
	{ 0x7e, 7 }, /* BUF_IO_FPGA_3 */

	/* (18-23) Bank 0x00 */
	{ 0x00, 0 }, /* I2C_MUX_A0 */
	{ 0x00, 1 }, /* SDS_SOM_INT */
	{ 0x00, 2 }, /* I2C0_SDS_EN */
	{ 0x00, 3 }, /* USB_HUB_RST */
	{ 0x00, 4 }, /* CAN_TERM_SW */
	{ 0x00, 7 }, /* I2C2_M2E_EN */

	/* (24-27) Bank 0x09 */
	{ 0x09, 0 }, /* FPGA_SLEEP_SET */
	{ 0x09, 5 }, /* PCIE1_UART_WAKE */
	{ 0x09, 6 }, /* FPGA_HB */
	{ 0x09, 7 }, /* I2C2_M2M_EN */

	/* (28-33) Bank 0x12 */
	{ 0x12, 2 }, /* DP_PWR_EN */
	{ 0x12, 3 }, /* DP_PWR_FAULT */
	{ 0x12, 4 }, /* EXP_DONE_GP2 */
	{ 0x12, 5 }, /* EXP_INIT_GP3 */
	{ 0x12, 6 }, /* SDS_HUB0_SW_FAULT */
	{ 0x12, 7 }, /* SDS_HUB0_SW_ST4 */

	/* (34-39) Bank 0x1B */
	{ 0x1b, 0 }, /* SDS_HUB0_SW_SEH */
	{ 0x1b, 1 }, /* SDS_HUB0_SW_SEL */
	{ 0x1b, 2 }, /* EN_VDD_SW_SDS0 */
	{ 0x1b, 3 }, /* EN_VDD_SW_SDS1 */
	{ 0x1b, 4 }, /* EN_VDD_SW_SDS2 */
	{ 0x1b, 5 }, /* EN_VDD_SW_SDS3 */

	/* (40-41) Bank 0x24 */
	{ 0x24, 0 }, /* EXP_PRGM_GP4 */
	{ 0x24, 1 }, /* EXP_JTEN_GP5 */

	/* (42-48) Bank 0x2D */
	{ 0x2d, 0 }, /* GMSL0_RO */
	{ 0x2d, 1 }, /* GMSL0_TRIGGER */
	{ 0x2d, 2 }, /* GMSL0_PWDN */
	{ 0x2d, 3 }, /* GMSL0_MFP10 */
	{ 0x2d, 4 }, /* GMSL0_MFP9 */
	{ 0x2d, 5 }, /* GMSL0_INTN */
	{ 0x2d, 6 }, /* SPI_BNE_0 */

	/* (49-55) Bank 0x36 */
	{ 0x36, 0 }, /* GMSL1_RO */
	{ 0x36, 1 }, /* GMSL1_TRIGGER */
	{ 0x36, 2 }, /* GMSL1_PWDN */
	{ 0x36, 3 }, /* GMSL1_MFP10 */
	{ 0x36, 4 }, /* GMSL1_MFP9 */
	{ 0x36, 5 }, /* GMSL1_INTN */
	{ 0x36, 6 }, /* SPI_BNE_1 */

	/* (56-62) Bank 0x3F */
	{ 0x3f, 0 }, /* GMSL2_RO */
	{ 0x3f, 1 }, /* GMSL2_TRIGGER */
	{ 0x3f, 2 }, /* GMSL2_PWDN */
	{ 0x3f, 3 }, /* GMSL2_MFP10 */
	{ 0x3f, 4 }, /* GMSL2_MFP9 */
	{ 0x3f, 5 }, /* GMSL2_INTN */
	{ 0x3f, 6 }, /* SPI_BNE_2 */

	/* (63-64) Bank 0x48 */
	{ 0x48, 0 }, /* PCIE1_RTC_OUT */
	{ 0x48, 1 }, /* PCIE0_RTC_OUT */

	/* (65-68) Bank 0x51 */
	{ 0x51, 0 }, /* FPGA_3V3_GPIO_0 */
	{ 0x51, 1 }, /* FPGA_3V3_GPIO_1 */
	{ 0x51, 2 }, /* FPGA_3V3_GPIO_2 */
	{ 0x51, 3 }, /* FPGA_3V3_GPIO_3 */

	/* (69-70) Bank 0x5A */
	{ 0x5a, 0 }, /* FPGA_PCLK_LOOP1A */
	{ 0x5a, 1 }, /* FPGA_PCLK_LOOP1B */

	/* (71-78) Bank 0x63 */
	NXC_ADD_GPIO_BANK(0x63), /* SOM_GPIO_[01-09] */

	/* (79-86) Bank 0x6C */
	NXC_ADD_GPIO_BANK(0x6c),
	/* SOM_GPIO_[10-13]
	 * M2E_SDIO_WAKE
	 * M2E_SDIO_RST
	 * SOM_CAM0_PWDN
	 * SOM_CAM1_PWDN */

	/* (87-94) Bank 0x75 */
	NXC_ADD_GPIO_BANK(0x75),
	/* FPGA_PCLK_LOOP0A
	 * FPGA_PCLK_LOOP0B
	 * PCIE0_DEVSLP
	 * SPI1_SOM_CS0
	 * SOM_FAN_TACH
	 * PCIE_SOM_RTC
	 * SPI0_SOM_CS1
	 * SPI0_SOM_CS0 */

	/* (95-96) Bank 0x7E (excluding BUF_IO) */
	{ 0x7e, 0 }, /* SDS_HUB0_SW_DIAG_EN */
	{ 0x7e, 3 }, /* ADC_ISNS_ALERT */

	/* (97-100) Bank 0x87 */
	{ 0x87, 0 }, /* PCIE0_M2M_ALERT */
	{ 0x87, 1 }, /* M2M_SW_CTL */
	{ 0x87, 2 }, /* PCIE1_M2E_ALERT */
	{ 0x87, 3 }, /* M2E_SW_CTL */

	/* (101) Bank 0x90 (excluding FPGA_GPIO) */
	{ 0x90, 0 }, /* SOM_SLEEP_STATUS */
};

#endif
