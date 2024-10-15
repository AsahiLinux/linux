// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/*
 * Apple Type-C PHY driver
 *
 * Copyright (C) The Asahi Linux Contributors
 * Author: Sven Peter <sven@svenpeter.dev>
 */

#include "atc.h"
#include "trace.h"

#include <asm-generic/errno.h>
#include <dt-bindings/phy/phy.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/types.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>
#include <linux/usb/typec_tbt.h>

#define rcdev_to_apple_atcphy(_rcdev) \
	container_of(_rcdev, struct apple_atcphy, rcdev)

#define AUSPLL_APB_CMD_OVERRIDE 0x2000
#define AUSPLL_APB_CMD_OVERRIDE_REQ BIT(0)
#define AUSPLL_APB_CMD_OVERRIDE_ACK BIT(1)
#define AUSPLL_APB_CMD_OVERRIDE_UNK28 BIT(28)
#define AUSPLL_APB_CMD_OVERRIDE_CMD GENMASK(27, 3)

#define AUSPLL_FREQ_DESC_A 0x2080
#define AUSPLL_FD_FREQ_COUNT_TARGET GENMASK(9, 0)
#define AUSPLL_FD_FBDIVN_HALF BIT(10)
#define AUSPLL_FD_REV_DIVN GENMASK(13, 11)
#define AUSPLL_FD_KI_MAN GENMASK(17, 14)
#define AUSPLL_FD_KI_EXP GENMASK(21, 18)
#define AUSPLL_FD_KP_MAN GENMASK(25, 22)
#define AUSPLL_FD_KP_EXP GENMASK(29, 26)
#define AUSPLL_FD_KPKI_SCALE_HBW GENMASK(31, 30)

#define AUSPLL_FREQ_DESC_B 0x2084
#define AUSPLL_FD_FBDIVN_FRAC_DEN GENMASK(13, 0)
#define AUSPLL_FD_FBDIVN_FRAC_NUM GENMASK(27, 14)

#define AUSPLL_FREQ_DESC_C 0x2088
#define AUSPLL_FD_SDM_SSC_STEP GENMASK(7, 0)
#define AUSPLL_FD_SDM_SSC_EN BIT(8)
#define AUSPLL_FD_PCLK_DIV_SEL GENMASK(13, 9)
#define AUSPLL_FD_LFSDM_DIV GENMASK(15, 14)
#define AUSPLL_FD_LFCLK_CTRL GENMASK(19, 16)
#define AUSPLL_FD_VCLK_OP_DIVN GENMASK(21, 20)
#define AUSPLL_FD_VCLK_PRE_DIVN BIT(22)

#define AUSPLL_DCO_EFUSE_SPARE 0x222c
#define AUSPLL_RODCO_ENCAP_EFUSE GENMASK(10, 9)
#define AUSPLL_RODCO_BIAS_ADJUST_EFUSE GENMASK(14, 12)

#define AUSPLL_FRACN_CAN 0x22a4
#define AUSPLL_DLL_START_CAPCODE GENMASK(18, 17)

#define AUSPLL_CLKOUT_MASTER 0x2200
#define AUSPLL_CLKOUT_MASTER_PCLK_DRVR_EN BIT(2)
#define AUSPLL_CLKOUT_MASTER_PCLK2_DRVR_EN BIT(4)
#define AUSPLL_CLKOUT_MASTER_REFBUFCLK_DRVR_EN BIT(6)

#define AUSPLL_CLKOUT_DIV 0x2208
#define AUSPLL_CLKOUT_PLLA_REFBUFCLK_DI GENMASK(20, 16)

#define AUSPLL_BGR 0x2214
#define AUSPLL_BGR_CTRL_AVAIL BIT(0)

#define AUSPLL_CLKOUT_DTC_VREG 0x2220
#define AUSPLL_DTC_VREG_ADJUST GENMASK(16, 14)
#define AUSPLL_DTC_VREG_BYPASS BIT(7)

#define AUSPLL_FREQ_CFG 0x2224
#define AUSPLL_FREQ_REFCLK GENMASK(1, 0)

#define AUS_COMMON_SHIM_BLK_VREG 0x0a04
#define AUS_VREG_TRIM GENMASK(6, 2)

#define CIO3PLL_CLK_CTRL 0x2a00
#define CIO3PLL_CLK_PCLK_EN BIT(1)
#define CIO3PLL_CLK_REFCLK_EN BIT(5)

#define CIO3PLL_DCO_NCTRL 0x2a38
#define CIO3PLL_DCO_COARSEBIN_EFUSE0 GENMASK(6, 0)
#define CIO3PLL_DCO_COARSEBIN_EFUSE1 GENMASK(23, 17)

#define CIO3PLL_FRACN_CAN 0x2aa4
#define CIO3PLL_DLL_CAL_START_CAPCODE GENMASK(18, 17)

#define CIO3PLL_DTC_VREG 0x2a20
#define CIO3PLL_DTC_VREG_ADJUST GENMASK(16, 14)

#define ACIOPHY_CROSSBAR 0x4c
#define ACIOPHY_CROSSBAR_PROTOCOL GENMASK(4, 0)
#define ACIOPHY_CROSSBAR_PROTOCOL_USB4 0x0
#define ACIOPHY_CROSSBAR_PROTOCOL_USB4_SWAPPED 0x1
#define ACIOPHY_CROSSBAR_PROTOCOL_USB3 0xa
#define ACIOPHY_CROSSBAR_PROTOCOL_USB3_SWAPPED 0xb
#define ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP 0x10
#define ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP_SWAPPED 0x11
#define ACIOPHY_CROSSBAR_PROTOCOL_DP 0x14
#define ACIOPHY_CROSSBAR_DP_SINGLE_PMA GENMASK(16, 5)
#define ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE 0x0000
#define ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK100 0x100
#define ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK008 0x008
#define ACIOPHY_CROSSBAR_DP_BOTH_PMA BIT(17)

#define ACIOPHY_LANE_MODE 0x48
#define ACIOPHY_LANE_MODE_RX0 GENMASK(2, 0)
#define ACIOPHY_LANE_MODE_TX0 GENMASK(5, 3)
#define ACIOPHY_LANE_MODE_RX1 GENMASK(8, 6)
#define ACIOPHY_LANE_MODE_TX1 GENMASK(11, 9)
#define ACIOPHY_LANE_MODE_USB4 0
#define ACIOPHY_LANE_MODE_USB3 1
#define ACIOPHY_LANE_MODE_DP 2
#define ACIOPHY_LANE_MODE_OFF 3

#define ACIOPHY_TOP_BIST_CIOPHY_CFG1 0x84
#define ACIOPHY_TOP_BIST_CIOPHY_CFG1_CLK_EN BIT(27)
#define ACIOPHY_TOP_BIST_CIOPHY_CFG1_BIST_EN BIT(28)

#define ACIOPHY_TOP_BIST_OV_CFG 0x8c
#define ACIOPHY_TOP_BIST_OV_CFG_LN0_RESET_N_OV BIT(13)
#define ACIOPHY_TOP_BIST_OV_CFG_LN0_PWR_DOWN_OV BIT(25)

#define ACIOPHY_TOP_BIST_READ_CTRL 0x90
#define ACIOPHY_TOP_BIST_READ_CTRL_LN0_PHY_STATUS_RE BIT(2)

#define ACIOPHY_TOP_PHY_STAT 0x9c
#define ACIOPHY_TOP_PHY_STAT_LN0_UNK0 BIT(0)
#define ACIOPHY_TOP_PHY_STAT_LN0_UNK23 BIT(23)

#define ACIOPHY_TOP_BIST_PHY_CFG0 0xa8
#define ACIOPHY_TOP_BIST_PHY_CFG0_LN0_RESET_N BIT(0)

#define ACIOPHY_TOP_BIST_PHY_CFG1 0xac
#define ACIOPHY_TOP_BIST_PHY_CFG1_LN0_PWR_DOWN GENMASK(13, 10)

#define ACIOPHY_PLL_COMMON_CTRL 0x1028
#define ACIOPHY_PLL_WAIT_FOR_CMN_READY_BEFORE_RESET_EXIT BIT(24)

#define ATCPHY_POWER_CTRL 0x20000
#define ATCPHY_POWER_STAT 0x20004
#define ATCPHY_POWER_SLEEP_SMALL BIT(0)
#define ATCPHY_POWER_SLEEP_BIG BIT(1)
#define ATCPHY_POWER_CLAMP_EN BIT(2)
#define ATCPHY_POWER_APB_RESET_N BIT(3)
#define ATCPHY_POWER_PHY_RESET_N BIT(4)

#define ATCPHY_MISC 0x20008
#define ATCPHY_MISC_RESET_N BIT(0)
#define ATCPHY_MISC_LANE_SWAP BIT(2)

#define ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0 0x7000
#define DP_PMA_BYTECLK_RESET BIT(0)
#define DP_MAC_DIV20_CLK_SEL BIT(1)
#define DPTXPHY_PMA_LANE_RESET_N BIT(2)
#define DPTXPHY_PMA_LANE_RESET_N_OV BIT(3)
#define DPTX_PCLK1_SELECT GENMASK(6, 4)
#define DPTX_PCLK2_SELECT GENMASK(9, 7)
#define DPRX_PCLK_SELECT GENMASK(12, 10)
#define DPTX_PCLK1_ENABLE BIT(13)
#define DPTX_PCLK2_ENABLE BIT(14)
#define DPRX_PCLK_ENABLE BIT(15)

#define ACIOPHY_DP_PCLK_STAT 0x7044
#define ACIOPHY_AUSPLL_LOCK BIT(3)

#define LN0_AUSPMA_RX_TOP 0x9000
#define LN0_AUSPMA_RX_EQ 0xA000
#define LN0_AUSPMA_RX_SHM 0xB000
#define LN0_AUSPMA_TX_TOP 0xC000
#define LN0_AUSPMA_TX_SHM 0xD000

#define LN1_AUSPMA_RX_TOP 0x10000
#define LN1_AUSPMA_RX_EQ 0x11000
#define LN1_AUSPMA_RX_SHM 0x12000
#define LN1_AUSPMA_TX_TOP 0x13000
#define LN1_AUSPMA_TX_SHM 0x14000

#define LN_AUSPMA_RX_TOP_PMAFSM 0x0010
#define LN_AUSPMA_RX_TOP_PMAFSM_PCS_OV BIT(0)
#define LN_AUSPMA_RX_TOP_PMAFSM_PCS_REQ BIT(9)

#define LN_AUSPMA_RX_TOP_TJ_CFG_RX_TXMODE 0x00F0
#define LN_RX_TXMODE BIT(0)

#define LN_AUSPMA_RX_SHM_TJ_RXA_CTLE_CTRL0 0x00
#define LN_TX_CLK_EN BIT(20)
#define LN_TX_CLK_EN_OV BIT(21)

#define LN_AUSPMA_RX_SHM_TJ_RXA_AFE_CTRL1 0x04
#define LN_RX_DIV20_RESET_N_OV BIT(29)
#define LN_RX_DIV20_RESET_N BIT(30)

#define LN_AUSPMA_RX_SHM_TJ_RXA_UNK_CTRL2 0x08
#define LN_AUSPMA_RX_SHM_TJ_RXA_UNK_CTRL3 0x0C
#define LN_AUSPMA_RX_SHM_TJ_RXA_UNK_CTRL4 0x10
#define LN_AUSPMA_RX_SHM_TJ_RXA_UNK_CTRL5 0x14
#define LN_AUSPMA_RX_SHM_TJ_RXA_UNK_CTRL6 0x18
#define LN_AUSPMA_RX_SHM_TJ_RXA_UNK_CTRL7 0x1C
#define LN_AUSPMA_RX_SHM_TJ_RXA_UNK_CTRL8 0x20
#define LN_AUSPMA_RX_SHM_TJ_RXA_UNK_CTRL9 0x24
#define LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL10 0x28
#define LN_DTVREG_ADJUST GENMASK(31, 27)

#define LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL11 0x2C
#define LN_DTVREG_BIG_EN BIT(23)
#define LN_DTVREG_BIG_EN_OV BIT(24)
#define LN_DTVREG_SML_EN BIT(25)
#define LN_DTVREG_SML_EN_OV BIT(26)

#define LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL12 0x30
#define LN_TX_BYTECLK_RESET_SYNC_CLR BIT(22)
#define LN_TX_BYTECLK_RESET_SYNC_CLR_OV BIT(23)
#define LN_TX_BYTECLK_RESET_SYNC_EN BIT(24)
#define LN_TX_BYTECLK_RESET_SYNC_EN_OV BIT(25)
#define LN_TX_HRCLK_SEL BIT(28)
#define LN_TX_HRCLK_SEL_OV BIT(29)
#define LN_TX_PBIAS_EN BIT(30)
#define LN_TX_PBIAS_EN_OV BIT(31)

#define LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL13 0x34
#define LN_TX_PRE_EN BIT(0)
#define LN_TX_PRE_EN_OV BIT(1)
#define LN_TX_PST1_EN BIT(2)
#define LN_TX_PST1_EN_OV BIT(3)
#define LN_DTVREG_ADJUST_OV BIT(15)

#define LN_AUSPMA_RX_SHM_TJ_UNK_CTRL14A 0x38
#define LN_AUSPMA_RX_SHM_TJ_UNK_CTRL14B 0x3C
#define LN_AUSPMA_RX_SHM_TJ_UNK_CTRL15A 0x40
#define LN_AUSPMA_RX_SHM_TJ_UNK_CTRL15B 0x44
#define LN_AUSPMA_RX_SHM_TJ_RXA_SAVOS_CTRL16 0x48
#define LN_RXTERM_EN BIT(21)
#define LN_RXTERM_EN_OV BIT(22)
#define LN_RXTERM_PULLUP_LEAK_EN BIT(23)
#define LN_RXTERM_PULLUP_LEAK_EN_OV BIT(24)
#define LN_TX_CAL_CODE GENMASK(29, 25)
#define LN_TX_CAL_CODE_OV BIT(30)

#define LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL17 0x4C
#define LN_TX_MARGIN GENMASK(19, 15)
#define LN_TX_MARGIN_OV BIT(20)
#define LN_TX_MARGIN_LSB BIT(21)
#define LN_TX_MARGIN_LSB_OV BIT(22)
#define LN_TX_MARGIN_P1 GENMASK(26, 23)
#define LN_TX_MARGIN_P1_OV BIT(27)
#define LN_TX_MARGIN_P1_LSB GENMASK(29, 28)
#define LN_TX_MARGIN_P1_LSB_OV BIT(30)

#define LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18 0x50
#define LN_TX_P1_CODE GENMASK(3, 0)
#define LN_TX_P1_CODE_OV BIT(4)
#define LN_TX_P1_LSB_CODE GENMASK(6, 5)
#define LN_TX_P1_LSB_CODE_OV BIT(7)
#define LN_TX_MARGIN_PRE GENMASK(10, 8)
#define LN_TX_MARGIN_PRE_OV BIT(11)
#define LN_TX_MARGIN_PRE_LSB GENMASK(13, 12)
#define LN_TX_MARGIN_PRE_LSB_OV BIT(14)
#define LN_TX_PRE_LSB_CODE GENMASK(16, 15)
#define LN_TX_PRE_LSB_CODE_OV BIT(17)
#define LN_TX_PRE_CODE GENMASK(21, 18)
#define LN_TX_PRE_CODE_OV BIT(22)

#define LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19 0x54
#define LN_TX_TEST_EN BIT(21)
#define LN_TX_TEST_EN_OV BIT(22)
#define LN_TX_EN BIT(23)
#define LN_TX_EN_OV BIT(24)
#define LN_TX_CLK_DLY_CTRL_TAPGEN GENMASK(27, 25)
#define LN_TX_CLK_DIV2_EN BIT(28)
#define LN_TX_CLK_DIV2_EN_OV BIT(29)
#define LN_TX_CLK_DIV2_RST BIT(30)
#define LN_TX_CLK_DIV2_RST_OV BIT(31)

#define LN_AUSPMA_RX_SHM_TJ_RXA_UNK_CTRL20 0x58
#define LN_AUSPMA_RX_SHM_TJ_RXA_UNK_CTRL21 0x5C
#define LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22 0x60
#define LN_VREF_ADJUST_GRAY GENMASK(11, 7)
#define LN_VREF_ADJUST_GRAY_OV BIT(12)
#define LN_VREF_BIAS_SEL GENMASK(14, 13)
#define LN_VREF_BIAS_SEL_OV BIT(15)
#define LN_VREF_BOOST_EN BIT(16)
#define LN_VREF_BOOST_EN_OV BIT(17)
#define LN_VREF_EN BIT(18)
#define LN_VREF_EN_OV BIT(19)
#define LN_VREF_LPBKIN_DATA GENMASK(29, 28)
#define LN_VREF_TEST_RXLPBKDT_EN BIT(30)
#define LN_VREF_TEST_RXLPBKDT_EN_OV BIT(31)

#define LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG0 0x00
#define LN_BYTECLK_RESET_SYNC_EN_OV BIT(2)
#define LN_BYTECLK_RESET_SYNC_EN BIT(3)
#define LN_BYTECLK_RESET_SYNC_CLR_OV BIT(4)
#define LN_BYTECLK_RESET_SYNC_CLR BIT(5)
#define LN_BYTECLK_RESET_SYNC_SEL_OV BIT(6)

#define LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG1 0x04
#define LN_TXA_DIV2_EN_OV BIT(8)
#define LN_TXA_DIV2_EN BIT(9)
#define LN_TXA_DIV2_RESET_OV BIT(10)
#define LN_TXA_DIV2_RESET BIT(11)
#define LN_TXA_CLK_EN_OV BIT(22)
#define LN_TXA_CLK_EN BIT(23)

#define LN_AUSPMA_TX_SHM_TXA_IMP_REG0 0x08
#define LN_TXA_CAL_CTRL_OV BIT(0)
#define LN_TXA_CAL_CTRL GENMASK(18, 1)
#define LN_TXA_CAL_CTRL_BASE_OV BIT(19)
#define LN_TXA_CAL_CTRL_BASE GENMASK(23, 20)
#define LN_TXA_HIZ_OV BIT(29)
#define LN_TXA_HIZ BIT(30)

#define LN_AUSPMA_TX_SHM_TXA_IMP_REG1 0x0C
#define LN_AUSPMA_TX_SHM_TXA_IMP_REG2 0x10
#define LN_TXA_MARGIN_OV BIT(0)
#define LN_TXA_MARGIN GENMASK(18, 1)
#define LN_TXA_MARGIN_2R_OV BIT(19)
#define LN_TXA_MARGIN_2R BIT(20)

#define LN_AUSPMA_TX_SHM_TXA_IMP_REG3 0x14
#define LN_TXA_MARGIN_POST_OV BIT(0)
#define LN_TXA_MARGIN_POST GENMASK(10, 1)
#define LN_TXA_MARGIN_POST_2R_OV BIT(11)
#define LN_TXA_MARGIN_POST_2R BIT(12)
#define LN_TXA_MARGIN_POST_4R_OV BIT(13)
#define LN_TXA_MARGIN_POST_4R BIT(14)
#define LN_TXA_MARGIN_PRE_OV BIT(15)
#define LN_TXA_MARGIN_PRE GENMASK(21, 16)
#define LN_TXA_MARGIN_PRE_2R_OV BIT(22)
#define LN_TXA_MARGIN_PRE_2R BIT(23)
#define LN_TXA_MARGIN_PRE_4R_OV BIT(24)
#define LN_TXA_MARGIN_PRE_4R BIT(25)

#define LN_AUSPMA_TX_SHM_TXA_UNK_REG0 0x18
#define LN_AUSPMA_TX_SHM_TXA_UNK_REG1 0x1C
#define LN_AUSPMA_TX_SHM_TXA_UNK_REG2 0x20

#define LN_AUSPMA_TX_SHM_TXA_LDOCLK 0x24
#define LN_LDOCLK_BYPASS_SML_OV BIT(8)
#define LN_LDOCLK_BYPASS_SML BIT(9)
#define LN_LDOCLK_BYPASS_BIG_OV BIT(10)
#define LN_LDOCLK_BYPASS_BIG BIT(11)
#define LN_LDOCLK_EN_SML_OV BIT(12)
#define LN_LDOCLK_EN_SML BIT(13)
#define LN_LDOCLK_EN_BIG_OV BIT(14)
#define LN_LDOCLK_EN_BIG BIT(15)

/* LPDPTX registers */
#define LPDPTX_AUX_CFG_BLK_AUX_CTRL 0x0000
#define LPDPTX_BLK_AUX_CTRL_PWRDN BIT(4)
#define LPDPTX_BLK_AUX_RXOFFSET GENMASK(25, 22)

#define LPDPTX_AUX_CFG_BLK_AUX_LDO_CTRL 0x0008

#define LPDPTX_AUX_CFG_BLK_AUX_MARGIN 0x000c
#define LPDPTX_MARGIN_RCAL_RXOFFSET_EN BIT(5)
#define LPDPTX_AUX_MARGIN_RCAL_TXSWING GENMASK(10, 6)

#define LPDPTX_AUX_SHM_CFG_BLK_AUX_CTRL_REG0 0x0204
#define LPDPTX_CFG_PMA_AUX_SEL_LF_DATA BIT(15)

#define LPDPTX_AUX_SHM_CFG_BLK_AUX_CTRL_REG1 0x0208
#define LPDPTX_CFG_PMA_PHYS_ADJ GENMASK(22, 20)
#define LPDPTX_CFG_PMA_PHYS_ADJ_OV BIT(19)

#define LPDPTX_AUX_CONTROL 0x4000
#define LPDPTX_AUX_PWN_DOWN 0x10
#define LPDPTX_AUX_CLAMP_EN 0x04
#define LPDPTX_SLEEP_B_BIG_IN 0x02
#define LPDPTX_SLEEP_B_SML_IN 0x01
#define LPDPTX_TXTERM_CODEMSB 0x400
#define LPDPTX_TXTERM_CODE GENMASK(9, 5)

/* pipehandler registers */
#define PIPEHANDLER_OVERRIDE 0x00
#define PIPEHANDLER_OVERRIDE_RXVALID BIT(0)
#define PIPEHANDLER_OVERRIDE_RXDETECT BIT(2)

#define PIPEHANDLER_OVERRIDE_VALUES 0x04

#define PIPEHANDLER_MUX_CTRL 0x0c
#define PIPEHANDLER_MUX_MODE GENMASK(1, 0)
#define PIPEHANDLER_MUX_MODE_USB3PHY 0
#define PIPEHANDLER_MUX_MODE_DUMMY_PHY 2
#define PIPEHANDLER_CLK_SELECT GENMASK(5, 3)
#define PIPEHANDLER_CLK_USB3PHY 1
#define PIPEHANDLER_CLK_DUMMY_PHY 4
#define PIPEHANDLER_LOCK_REQ 0x10
#define PIPEHANDLER_LOCK_ACK 0x14
#define PIPEHANDLER_LOCK_EN BIT(0)

#define PIPEHANDLER_AON_GEN 0x1C
#define PIPEHANDLER_AON_GEN_DWC3_FORCE_CLAMP_EN BIT(4)
#define PIPEHANDLER_AON_GEN_DWC3_RESET_N BIT(0)

#define PIPEHANDLER_NONSELECTED_OVERRIDE 0x20
#define PIPEHANDLER_NONSELECTED_NATIVE_RESET BIT(12)
#define PIPEHANDLER_DUMMY_PHY_EN BIT(15)
#define PIPEHANDLER_NONSELECTED_NATIVE_POWER_DOWN GENMASK(3, 0)

/* USB2 PHY regs */
#define USB2PHY_USBCTL 0x00
#define USB2PHY_USBCTL_HOST_EN BIT(1)

#define USB2PHY_CTL 0x04
#define USB2PHY_CTL_RESET BIT(0)
#define USB2PHY_CTL_PORT_RESET BIT(1)
#define USB2PHY_CTL_APB_RESET_N BIT(2)
#define USB2PHY_CTL_SIDDQ BIT(3)

#define USB2PHY_SIG 0x08
#define USB2PHY_SIG_VBUSDET_FORCE_VAL BIT(0)
#define USB2PHY_SIG_VBUSDET_FORCE_EN BIT(1)
#define USB2PHY_SIG_VBUSVLDEXT_FORCE_VAL BIT(2)
#define USB2PHY_SIG_VBUSVLDEXT_FORCE_EN BIT(3)
#define USB2PHY_SIG_HOST (7 << 12)

static const struct {
	const struct atcphy_mode_configuration normal;
	const struct atcphy_mode_configuration swapped;
	bool enable_dp_aux;
	enum atcphy_pipehandler_state pipehandler_state;
} atcphy_modes[] = {
	[APPLE_ATCPHY_MODE_OFF] = {
		.normal = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_OFF, ACIOPHY_LANE_MODE_OFF},
			.dp_lane = {false, false},
			.set_swap = false,
		},
		.swapped = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3_SWAPPED,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_OFF, ACIOPHY_LANE_MODE_OFF},
			.dp_lane = {false, false},
			.set_swap = false, /* doesn't matter since the SS lanes are off */
		},
		.enable_dp_aux = false,
		.pipehandler_state = ATCPHY_PIPEHANDLER_STATE_USB2,
	},
	[APPLE_ATCPHY_MODE_USB2] = {
		.normal = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_OFF, ACIOPHY_LANE_MODE_OFF},
			.dp_lane = {false, false},
			.set_swap = false,
		},
		.swapped = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3_SWAPPED,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_OFF, ACIOPHY_LANE_MODE_OFF},
			.dp_lane = {false, false},
			.set_swap = false, /* doesn't matter since the SS lanes are off */
		},
		.enable_dp_aux = false,
		.pipehandler_state = ATCPHY_PIPEHANDLER_STATE_USB2,
	},
	[APPLE_ATCPHY_MODE_USB3] = {
		.normal = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_USB3, ACIOPHY_LANE_MODE_OFF},
			.dp_lane = {false, false},
			.set_swap = false,
		},
		.swapped = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3_SWAPPED,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_OFF, ACIOPHY_LANE_MODE_USB3},
			.dp_lane = {false, false},
			.set_swap = true,
		},
		.enable_dp_aux = false,
		.pipehandler_state = ATCPHY_PIPEHANDLER_STATE_USB3,
	},
	[APPLE_ATCPHY_MODE_USB3_DP] = {
		.normal = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK008,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_USB3, ACIOPHY_LANE_MODE_DP},
			.dp_lane = {false, true},
			.set_swap = false,
		},
		.swapped = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB3_DP_SWAPPED,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK008,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_DP, ACIOPHY_LANE_MODE_USB3},
			.dp_lane = {true, false},
			.set_swap = true,
		},
		.enable_dp_aux = true,
		.pipehandler_state = ATCPHY_PIPEHANDLER_STATE_USB3,
	},
	[APPLE_ATCPHY_MODE_USB4] = {
		.normal = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB4,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_USB4, ACIOPHY_LANE_MODE_USB4},
			.dp_lane = {false, false},
			.set_swap = false,
		},
		.swapped = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_USB4_SWAPPED,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_NONE,
			.crossbar_dp_both_pma = false,
			.lane_mode = {ACIOPHY_LANE_MODE_USB4, ACIOPHY_LANE_MODE_USB4},
			.dp_lane = {false, false},
			.set_swap = false, /* intentionally false */
		},
		.enable_dp_aux = false,
		.pipehandler_state = ATCPHY_PIPEHANDLER_STATE_USB2,
	},
	[APPLE_ATCPHY_MODE_DP] = {
		.normal = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_DP,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK100,
			.crossbar_dp_both_pma = true,
			.lane_mode = {ACIOPHY_LANE_MODE_DP, ACIOPHY_LANE_MODE_DP},
			.dp_lane = {true, true},
			.set_swap = false,
		},
		.swapped = {
			.crossbar = ACIOPHY_CROSSBAR_PROTOCOL_DP,
			.crossbar_dp_single_pma = ACIOPHY_CROSSBAR_DP_SINGLE_PMA_UNK008,
			.crossbar_dp_both_pma = false, /* intentionally false */
			.lane_mode = {ACIOPHY_LANE_MODE_DP, ACIOPHY_LANE_MODE_DP},
			.dp_lane = {true, true},
			.set_swap = false, /* intentionally false */
		},
		.enable_dp_aux = true,
		.pipehandler_state = ATCPHY_PIPEHANDLER_STATE_USB2,
	},
};

static const struct atcphy_dp_link_rate_configuration dp_lr_config[] = {
	[ATCPHY_DP_LINK_RATE_RBR] = {
		.freqinit_count_target = 0x21c,
		.fbdivn_frac_den = 0x0,
		.fbdivn_frac_num = 0x0,
		.pclk_div_sel = 0x13,
		.lfclk_ctrl = 0x5,
		.vclk_op_divn = 0x2,
		.plla_clkout_vreg_bypass = true,
		.bypass_txa_ldoclk = true,
		.txa_div2_en = true,
	},
	[ATCPHY_DP_LINK_RATE_HBR] = {
		.freqinit_count_target = 0x1c2,
		.fbdivn_frac_den = 0x3ffe,
		.fbdivn_frac_num = 0x1fff,
		.pclk_div_sel = 0x9,
		.lfclk_ctrl = 0x5,
		.vclk_op_divn = 0x2,
		.plla_clkout_vreg_bypass = true,
		.bypass_txa_ldoclk = true,
		.txa_div2_en = false,
	},
	[ATCPHY_DP_LINK_RATE_HBR2] = {
		.freqinit_count_target = 0x1c2,
		.fbdivn_frac_den = 0x3ffe,
		.fbdivn_frac_num = 0x1fff,
		.pclk_div_sel = 0x4,
		.lfclk_ctrl = 0x5,
		.vclk_op_divn = 0x0,
		.plla_clkout_vreg_bypass = true,
		.bypass_txa_ldoclk = true,
		.txa_div2_en = false,
	},
	[ATCPHY_DP_LINK_RATE_HBR3] = {
		.freqinit_count_target = 0x2a3,
		.fbdivn_frac_den = 0x3ffc,
		.fbdivn_frac_num = 0x2ffd,
		.pclk_div_sel = 0x4,
		.lfclk_ctrl = 0x6,
		.vclk_op_divn = 0x0,
		.plla_clkout_vreg_bypass = false,
		.bypass_txa_ldoclk = false,
		.txa_div2_en = false,
	},
};

static inline void mask32(void __iomem *reg, u32 mask, u32 set)
{
	u32 value = readl(reg);
	value &= ~mask;
	value |= set;
	writel(value, reg);
}

static inline void core_mask32(struct apple_atcphy *atcphy, u32 reg, u32 mask,
			       u32 set)
{
	mask32(atcphy->regs.core + reg, mask, set);
}

static inline void set32(void __iomem *reg, u32 set)
{
	mask32(reg, 0, set);
}

static inline void core_set32(struct apple_atcphy *atcphy, u32 reg, u32 set)
{
	core_mask32(atcphy, reg, 0, set);
}

static inline void clear32(void __iomem *reg, u32 clear)
{
	mask32(reg, clear, 0);
}

static inline void core_clear32(struct apple_atcphy *atcphy, u32 reg, u32 clear)
{
	core_mask32(atcphy, reg, clear, 0);
}

static void atcphy_apply_tunable(struct apple_atcphy *atcphy,
				 void __iomem *regs,
				 struct atcphy_tunable *tunable)
{
	size_t i;

	for (i = 0; i < tunable->sz; ++i)
		mask32(regs + tunable->values[i].offset,
		       tunable->values[i].mask, tunable->values[i].value);
}

static void atcphy_apply_tunables(struct apple_atcphy *atcphy,
				  enum atcphy_mode mode)
{
	int lane0 = atcphy->swap_lanes ? 1 : 0;
	int lane1 = atcphy->swap_lanes ? 0 : 1;

	atcphy_apply_tunable(atcphy, atcphy->regs.axi2af,
			     &atcphy->tunables.axi2af);
	atcphy_apply_tunable(atcphy, atcphy->regs.core,
			     &atcphy->tunables.common);

	switch (mode) {
	case APPLE_ATCPHY_MODE_USB3:
		atcphy_apply_tunable(atcphy, atcphy->regs.core,
				     &atcphy->tunables.lane_usb3[lane0]);
		atcphy_apply_tunable(atcphy, atcphy->regs.core,
				     &atcphy->tunables.lane_usb3[lane1]);
		break;

	case APPLE_ATCPHY_MODE_USB3_DP:
		atcphy_apply_tunable(atcphy, atcphy->regs.core,
				     &atcphy->tunables.lane_usb3[lane0]);
		atcphy_apply_tunable(atcphy, atcphy->regs.core,
				     &atcphy->tunables.lane_displayport[lane1]);
		break;

	case APPLE_ATCPHY_MODE_DP:
		atcphy_apply_tunable(atcphy, atcphy->regs.core,
				     &atcphy->tunables.lane_displayport[lane0]);
		atcphy_apply_tunable(atcphy, atcphy->regs.core,
				     &atcphy->tunables.lane_displayport[lane1]);
		break;

	case APPLE_ATCPHY_MODE_USB4:
		atcphy_apply_tunable(atcphy, atcphy->regs.core,
				     &atcphy->tunables.lane_usb4[lane0]);
		atcphy_apply_tunable(atcphy, atcphy->regs.core,
				     &atcphy->tunables.lane_usb4[lane1]);
		break;

	default:
		dev_warn(atcphy->dev,
			 "Unknown mode %d in atcphy_apply_tunables\n", mode);
		fallthrough;
	case APPLE_ATCPHY_MODE_OFF:
	case APPLE_ATCPHY_MODE_USB2:
		break;
	}
}

static void atcphy_setup_pll_fuses(struct apple_atcphy *atcphy)
{
	void __iomem *regs = atcphy->regs.core;

	if (!atcphy->fuses.present)
		return;

	/* CIO3PLL fuses */
	mask32(regs + CIO3PLL_DCO_NCTRL, CIO3PLL_DCO_COARSEBIN_EFUSE0,
	       FIELD_PREP(CIO3PLL_DCO_COARSEBIN_EFUSE0,
			  atcphy->fuses.cio3pll_dco_coarsebin[0]));
	mask32(regs + CIO3PLL_DCO_NCTRL, CIO3PLL_DCO_COARSEBIN_EFUSE1,
	       FIELD_PREP(CIO3PLL_DCO_COARSEBIN_EFUSE1,
			  atcphy->fuses.cio3pll_dco_coarsebin[1]));
	mask32(regs + CIO3PLL_FRACN_CAN, CIO3PLL_DLL_CAL_START_CAPCODE,
	       FIELD_PREP(CIO3PLL_DLL_CAL_START_CAPCODE,
			  atcphy->fuses.cio3pll_dll_start_capcode[0]));

	if (atcphy->quirks.t8103_cio3pll_workaround) {
		mask32(regs + AUS_COMMON_SHIM_BLK_VREG, AUS_VREG_TRIM,
		       FIELD_PREP(AUS_VREG_TRIM,
				  atcphy->fuses.aus_cmn_shm_vreg_trim));
		mask32(regs + CIO3PLL_FRACN_CAN, CIO3PLL_DLL_CAL_START_CAPCODE,
		       FIELD_PREP(CIO3PLL_DLL_CAL_START_CAPCODE,
				  atcphy->fuses.cio3pll_dll_start_capcode[1]));
		mask32(regs + CIO3PLL_DTC_VREG, CIO3PLL_DTC_VREG_ADJUST,
		       FIELD_PREP(CIO3PLL_DTC_VREG_ADJUST,
				  atcphy->fuses.cio3pll_dtc_vreg_adjust));
	} else {
		mask32(regs + CIO3PLL_DTC_VREG, CIO3PLL_DTC_VREG_ADJUST,
		       FIELD_PREP(CIO3PLL_DTC_VREG_ADJUST,
				  atcphy->fuses.cio3pll_dtc_vreg_adjust));
		mask32(regs + AUS_COMMON_SHIM_BLK_VREG, AUS_VREG_TRIM,
		       FIELD_PREP(AUS_VREG_TRIM,
				  atcphy->fuses.aus_cmn_shm_vreg_trim));
	}

	/* AUSPLL fuses */
	mask32(regs + AUSPLL_DCO_EFUSE_SPARE, AUSPLL_RODCO_ENCAP_EFUSE,
	       FIELD_PREP(AUSPLL_RODCO_ENCAP_EFUSE,
			  atcphy->fuses.auspll_rodco_encap));
	mask32(regs + AUSPLL_DCO_EFUSE_SPARE, AUSPLL_RODCO_BIAS_ADJUST_EFUSE,
	       FIELD_PREP(AUSPLL_RODCO_BIAS_ADJUST_EFUSE,
			  atcphy->fuses.auspll_rodco_bias_adjust));
	mask32(regs + AUSPLL_FRACN_CAN, AUSPLL_DLL_START_CAPCODE,
	       FIELD_PREP(AUSPLL_DLL_START_CAPCODE,
			  atcphy->fuses.auspll_fracn_dll_start_capcode));
	mask32(regs + AUSPLL_CLKOUT_DTC_VREG, AUSPLL_DTC_VREG_ADJUST,
	       FIELD_PREP(AUSPLL_DTC_VREG_ADJUST,
			  atcphy->fuses.auspll_dtc_vreg_adjust));

	/* TODO: is this actually required again? */
	mask32(regs + AUS_COMMON_SHIM_BLK_VREG, AUS_VREG_TRIM,
	       FIELD_PREP(AUS_VREG_TRIM, atcphy->fuses.aus_cmn_shm_vreg_trim));
}

static int atcphy_cio_power_off(struct apple_atcphy *atcphy)
{
	u32 reg;
	int ret;

	/* enable all reset lines */
	core_clear32(atcphy, ATCPHY_POWER_CTRL, ATCPHY_POWER_PHY_RESET_N);
	core_clear32(atcphy, ATCPHY_POWER_CTRL, ATCPHY_POWER_APB_RESET_N);
	core_set32(atcphy, ATCPHY_POWER_CTRL, ATCPHY_POWER_CLAMP_EN);
	core_clear32(atcphy, ATCPHY_MISC, ATCPHY_MISC_RESET_N);

	// TODO: why clear? is this SLEEP_N? or do we enable some power management here?
	core_clear32(atcphy, ATCPHY_POWER_CTRL, ATCPHY_POWER_SLEEP_BIG);
	ret = readl_poll_timeout(atcphy->regs.core + ATCPHY_POWER_STAT, reg,
				 !(reg & ATCPHY_POWER_SLEEP_BIG), 100, 100000);
	if (ret) {
		dev_err(atcphy->dev, "failed to sleep atcphy \"big\"\n");
		return ret;
	}

	core_clear32(atcphy, ATCPHY_POWER_CTRL, ATCPHY_POWER_SLEEP_SMALL);
	ret = readl_poll_timeout(atcphy->regs.core + ATCPHY_POWER_STAT, reg,
				 !(reg & ATCPHY_POWER_SLEEP_SMALL), 100,
				 100000);
	if (ret) {
		dev_err(atcphy->dev, "failed to sleep atcphy \"small\"\n");
		return ret;
	}

	return 0;
}

static int atcphy_cio_power_on(struct apple_atcphy *atcphy)
{
	u32 reg;
	int ret;

	core_set32(atcphy, ATCPHY_MISC, ATCPHY_MISC_RESET_N);

	// TODO: why set?! see above
	core_set32(atcphy, ATCPHY_POWER_CTRL, ATCPHY_POWER_SLEEP_SMALL);
	ret = readl_poll_timeout(atcphy->regs.core + ATCPHY_POWER_STAT, reg,
				 reg & ATCPHY_POWER_SLEEP_SMALL, 100, 100000);
	if (ret) {
		dev_err(atcphy->dev, "failed to wakeup atcphy \"small\"\n");
		return ret;
	}

	core_set32(atcphy, ATCPHY_POWER_CTRL, ATCPHY_POWER_SLEEP_BIG);
	ret = readl_poll_timeout(atcphy->regs.core + ATCPHY_POWER_STAT, reg,
				 reg & ATCPHY_POWER_SLEEP_BIG, 100, 100000);
	if (ret) {
		dev_err(atcphy->dev, "failed to wakeup atcphy \"big\"\n");
		return ret;
	}

	core_clear32(atcphy, ATCPHY_POWER_CTRL, ATCPHY_POWER_CLAMP_EN);
	core_set32(atcphy, ATCPHY_POWER_CTRL, ATCPHY_POWER_APB_RESET_N);

	return 0;
}

static void atcphy_configure_lanes(struct apple_atcphy *atcphy,
				   enum atcphy_mode mode)
{
	const struct atcphy_mode_configuration *mode_cfg;

	if (atcphy->swap_lanes)
		mode_cfg = &atcphy_modes[mode].swapped;
	else
		mode_cfg = &atcphy_modes[mode].normal;

	trace_atcphy_configure_lanes(mode, mode_cfg);

	if (mode_cfg->dp_lane[0]) {
		core_set32(atcphy, LN0_AUSPMA_RX_TOP + LN_AUSPMA_RX_TOP_PMAFSM,
			   LN_AUSPMA_RX_TOP_PMAFSM_PCS_OV);
		core_clear32(atcphy,
			     LN0_AUSPMA_RX_TOP + LN_AUSPMA_RX_TOP_PMAFSM,
			     LN_AUSPMA_RX_TOP_PMAFSM_PCS_REQ);
	}
	if (mode_cfg->dp_lane[1]) {
		core_set32(atcphy, LN1_AUSPMA_RX_TOP + LN_AUSPMA_RX_TOP_PMAFSM,
			   LN_AUSPMA_RX_TOP_PMAFSM_PCS_OV);
		core_clear32(atcphy,
			     LN1_AUSPMA_RX_TOP + LN_AUSPMA_RX_TOP_PMAFSM,
			     LN_AUSPMA_RX_TOP_PMAFSM_PCS_REQ);
	}

	core_mask32(atcphy, ACIOPHY_LANE_MODE, ACIOPHY_LANE_MODE_RX0,
		    FIELD_PREP(ACIOPHY_LANE_MODE_RX0, mode_cfg->lane_mode[0]));
	core_mask32(atcphy, ACIOPHY_LANE_MODE, ACIOPHY_LANE_MODE_TX0,
		    FIELD_PREP(ACIOPHY_LANE_MODE_TX0, mode_cfg->lane_mode[0]));
	core_mask32(atcphy, ACIOPHY_LANE_MODE, ACIOPHY_LANE_MODE_RX1,
		    FIELD_PREP(ACIOPHY_LANE_MODE_RX1, mode_cfg->lane_mode[1]));
	core_mask32(atcphy, ACIOPHY_LANE_MODE, ACIOPHY_LANE_MODE_TX1,
		    FIELD_PREP(ACIOPHY_LANE_MODE_TX1, mode_cfg->lane_mode[1]));
	core_mask32(atcphy, ACIOPHY_CROSSBAR, ACIOPHY_CROSSBAR_PROTOCOL,
		    FIELD_PREP(ACIOPHY_CROSSBAR_PROTOCOL, mode_cfg->crossbar));

	if (mode_cfg->set_swap)
		core_set32(atcphy, ATCPHY_MISC, ATCPHY_MISC_LANE_SWAP);
	else
		core_clear32(atcphy, ATCPHY_MISC, ATCPHY_MISC_LANE_SWAP);

	if (mode_cfg->crossbar_dp_both_pma)
		core_set32(atcphy, ACIOPHY_CROSSBAR,
			   ACIOPHY_CROSSBAR_DP_BOTH_PMA);
	else
		core_clear32(atcphy, ACIOPHY_CROSSBAR,
			     ACIOPHY_CROSSBAR_DP_BOTH_PMA);

	core_mask32(atcphy, ACIOPHY_CROSSBAR, ACIOPHY_CROSSBAR_DP_SINGLE_PMA,
		    FIELD_PREP(ACIOPHY_CROSSBAR_DP_SINGLE_PMA,
			       mode_cfg->crossbar_dp_single_pma));
}

static int atcphy_pipehandler_lock(struct apple_atcphy *atcphy)
{
	int ret;
	u32 reg;

	if (readl_relaxed(atcphy->regs.pipehandler + PIPEHANDLER_LOCK_REQ) &
	    PIPEHANDLER_LOCK_EN)
		dev_warn(atcphy->dev, "pipehandler already locked\n");

	set32(atcphy->regs.pipehandler + PIPEHANDLER_LOCK_REQ,
	      PIPEHANDLER_LOCK_EN);

	ret = readl_poll_timeout(atcphy->regs.pipehandler +
					 PIPEHANDLER_LOCK_ACK,
				 reg, reg & PIPEHANDLER_LOCK_EN, 1000, 1000000);
	if (ret) {
		clear32(atcphy->regs.pipehandler + PIPEHANDLER_LOCK_REQ, 1);
		dev_err(atcphy->dev,
			"pipehandler lock not acked, this type-c port is probably dead until the next reboot.\n");
	}

	return ret;
}

static int atcphy_pipehandler_unlock(struct apple_atcphy *atcphy)
{
	int ret;
	u32 reg;

	clear32(atcphy->regs.pipehandler + PIPEHANDLER_LOCK_REQ,
		PIPEHANDLER_LOCK_EN);
	ret = readl_poll_timeout(
		atcphy->regs.pipehandler + PIPEHANDLER_LOCK_ACK, reg,
		!(reg & PIPEHANDLER_LOCK_EN), 1000, 1000000);
	if (ret)
		dev_err(atcphy->dev,
			"pipehandler lock release not acked, this type-c port is probably dead until the next reboot.\n");

	return ret;
}

static int atcphy_configure_pipehandler(struct apple_atcphy *atcphy,
					enum atcphy_pipehandler_state state)
{
	int ret;
	u32 reg;

	if (atcphy->pipehandler_state == state)
		return 0;

	clear32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE_VALUES,
		14); // TODO: why 14?
	set32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE,
	      PIPEHANDLER_OVERRIDE_RXVALID | PIPEHANDLER_OVERRIDE_RXDETECT);

	ret = atcphy_pipehandler_lock(atcphy);
	if (ret)
		return ret;

	switch (state) {
	case ATCPHY_PIPEHANDLER_STATE_USB3:
		core_set32(atcphy, ACIOPHY_TOP_BIST_PHY_CFG0,
			   ACIOPHY_TOP_BIST_PHY_CFG0_LN0_RESET_N);
		core_set32(atcphy, ACIOPHY_TOP_BIST_OV_CFG,
			   ACIOPHY_TOP_BIST_OV_CFG_LN0_RESET_N_OV);
		ret = readl_poll_timeout(
			atcphy->regs.core + ACIOPHY_TOP_PHY_STAT, reg,
			!(reg & ACIOPHY_TOP_PHY_STAT_LN0_UNK23), 100, 100000);
		if (ret)
			dev_warn(
				atcphy->dev,
				"timed out waiting for ACIOPHY_TOP_PHY_STAT_LN0_UNK23\n");

			// TODO: macOS does this but this breaks waiting for
			//       ACIOPHY_TOP_PHY_STAT_LN0_UNK0 then for some reason :/
			//       this is probably status reset which clears the ln0
			//       ready status but then the ready status never comes
			//       up again
#if 0
		core_set32(atcphy, ACIOPHY_TOP_BIST_READ_CTRL,
			   ACIOPHY_TOP_BIST_READ_CTRL_LN0_PHY_STATUS_RE);
		core_clear32(atcphy, ACIOPHY_TOP_BIST_READ_CTRL,
			     ACIOPHY_TOP_BIST_READ_CTRL_LN0_PHY_STATUS_RE);
#endif
		core_mask32(atcphy, ACIOPHY_TOP_BIST_PHY_CFG1,
			    ACIOPHY_TOP_BIST_PHY_CFG1_LN0_PWR_DOWN,
			    FIELD_PREP(ACIOPHY_TOP_BIST_PHY_CFG1_LN0_PWR_DOWN,
				       3));
		core_set32(atcphy, ACIOPHY_TOP_BIST_OV_CFG,
			   ACIOPHY_TOP_BIST_OV_CFG_LN0_PWR_DOWN_OV);
		core_set32(atcphy, ACIOPHY_TOP_BIST_CIOPHY_CFG1,
			   ACIOPHY_TOP_BIST_CIOPHY_CFG1_CLK_EN);
		core_set32(atcphy, ACIOPHY_TOP_BIST_CIOPHY_CFG1,
			   ACIOPHY_TOP_BIST_CIOPHY_CFG1_BIST_EN);
		writel(0, atcphy->regs.core + ACIOPHY_TOP_BIST_CIOPHY_CFG1);

		ret = readl_poll_timeout(
			atcphy->regs.core + ACIOPHY_TOP_PHY_STAT, reg,
			(reg & ACIOPHY_TOP_PHY_STAT_LN0_UNK0), 100, 100000);
		if (ret)
			dev_warn(
				atcphy->dev,
				"timed out waiting for ACIOPHY_TOP_PHY_STAT_LN0_UNK0\n");

		ret = readl_poll_timeout(
			atcphy->regs.core + ACIOPHY_TOP_PHY_STAT, reg,
			!(reg & ACIOPHY_TOP_PHY_STAT_LN0_UNK23), 100, 100000);
		if (ret)
			dev_warn(
				atcphy->dev,
				"timed out waiting for ACIOPHY_TOP_PHY_STAT_LN0_UNK23\n");

		writel(0, atcphy->regs.core + ACIOPHY_TOP_BIST_OV_CFG);
		core_set32(atcphy, ACIOPHY_TOP_BIST_CIOPHY_CFG1,
			   ACIOPHY_TOP_BIST_CIOPHY_CFG1_CLK_EN);
		core_set32(atcphy, ACIOPHY_TOP_BIST_CIOPHY_CFG1,
			   ACIOPHY_TOP_BIST_CIOPHY_CFG1_BIST_EN);

		/* switch dwc3's superspeed PHY to the real physical PHY */
		clear32(atcphy->regs.pipehandler + PIPEHANDLER_MUX_CTRL,
			PIPEHANDLER_CLK_SELECT);
		clear32(atcphy->regs.pipehandler + PIPEHANDLER_MUX_CTRL,
			PIPEHANDLER_MUX_MODE);
		mask32(atcphy->regs.pipehandler + PIPEHANDLER_MUX_CTRL,
		       PIPEHANDLER_CLK_SELECT,
		       FIELD_PREP(PIPEHANDLER_CLK_SELECT,
				  PIPEHANDLER_CLK_USB3PHY));
		mask32(atcphy->regs.pipehandler + PIPEHANDLER_MUX_CTRL,
		       PIPEHANDLER_MUX_MODE,
		       FIELD_PREP(PIPEHANDLER_MUX_MODE,
				  PIPEHANDLER_MUX_MODE_USB3PHY));

		/* use real rx detect/valid values again */
		clear32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE,
			PIPEHANDLER_OVERRIDE_RXVALID |
				PIPEHANDLER_OVERRIDE_RXDETECT);
		break;
	default:
		dev_warn(
			atcphy->dev,
			"unknown mode in pipehandler_configure: %d, switching to safe state\n",
			state);
		fallthrough;
	case ATCPHY_PIPEHANDLER_STATE_USB2:
		/* switch dwc3's superspeed PHY back to the dummy (and also USB4 PHY?) */
		clear32(atcphy->regs.pipehandler + PIPEHANDLER_MUX_CTRL,
			PIPEHANDLER_CLK_SELECT);
		clear32(atcphy->regs.pipehandler + PIPEHANDLER_MUX_CTRL,
			PIPEHANDLER_MUX_MODE);
		mask32(atcphy->regs.pipehandler + PIPEHANDLER_MUX_CTRL,
		       PIPEHANDLER_CLK_SELECT,
		       FIELD_PREP(PIPEHANDLER_CLK_SELECT,
				  PIPEHANDLER_CLK_DUMMY_PHY));
		mask32(atcphy->regs.pipehandler + PIPEHANDLER_MUX_CTRL,
		       PIPEHANDLER_MUX_MODE,
		       FIELD_PREP(PIPEHANDLER_MUX_MODE,
				  PIPEHANDLER_MUX_MODE_DUMMY_PHY));

		/* keep ignoring rx detect and valid values from the USB3/4 PHY? */
		set32(atcphy->regs.pipehandler + PIPEHANDLER_OVERRIDE,
		      PIPEHANDLER_OVERRIDE_RXVALID |
			      PIPEHANDLER_OVERRIDE_RXDETECT);
		break;
	}

	ret = atcphy_pipehandler_unlock(atcphy);
	if (ret)
		return ret;

	// TODO: macos seems to always clear it for USB3 - what about USB2/4?
	clear32(atcphy->regs.pipehandler + PIPEHANDLER_NONSELECTED_OVERRIDE,
		PIPEHANDLER_NONSELECTED_NATIVE_RESET);

	// TODO: why? without this superspeed devices sometimes come up as highspeed
	msleep(500);

	atcphy->pipehandler_state = state;

	return 0;
}

static void atcphy_enable_dp_aux(struct apple_atcphy *atcphy)
{
	core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
		   DPTXPHY_PMA_LANE_RESET_N);
	core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
		   DPTXPHY_PMA_LANE_RESET_N_OV);

	core_mask32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
		    DPRX_PCLK_SELECT, FIELD_PREP(DPRX_PCLK_SELECT, 1));
	core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
		   DPRX_PCLK_ENABLE);

	core_mask32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
		    DPTX_PCLK1_SELECT, FIELD_PREP(DPTX_PCLK1_SELECT, 1));
	core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
		   DPTX_PCLK1_ENABLE);

	core_mask32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
		    DPTX_PCLK2_SELECT, FIELD_PREP(DPTX_PCLK2_SELECT, 1));
	core_set32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
		   DPTX_PCLK2_ENABLE);

	core_set32(atcphy, ACIOPHY_PLL_COMMON_CTRL,
		   ACIOPHY_PLL_WAIT_FOR_CMN_READY_BEFORE_RESET_EXIT);

	set32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL, LPDPTX_AUX_CLAMP_EN);
	set32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL, LPDPTX_SLEEP_B_SML_IN);
	udelay(2);
	set32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL, LPDPTX_SLEEP_B_BIG_IN);
	udelay(2);
	clear32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL, LPDPTX_AUX_CLAMP_EN);
	clear32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL, LPDPTX_AUX_PWN_DOWN);
	clear32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL,
		LPDPTX_TXTERM_CODEMSB);
	mask32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL, LPDPTX_TXTERM_CODE,
	       FIELD_PREP(LPDPTX_TXTERM_CODE, 0x16));

	set32(atcphy->regs.lpdptx + LPDPTX_AUX_CFG_BLK_AUX_LDO_CTRL, 0x1c00);
	mask32(atcphy->regs.lpdptx + LPDPTX_AUX_SHM_CFG_BLK_AUX_CTRL_REG1,
	       LPDPTX_CFG_PMA_PHYS_ADJ, FIELD_PREP(LPDPTX_CFG_PMA_PHYS_ADJ, 5));
	set32(atcphy->regs.lpdptx + LPDPTX_AUX_SHM_CFG_BLK_AUX_CTRL_REG1,
	      LPDPTX_CFG_PMA_PHYS_ADJ_OV);

	clear32(atcphy->regs.lpdptx + LPDPTX_AUX_CFG_BLK_AUX_MARGIN,
		LPDPTX_MARGIN_RCAL_RXOFFSET_EN);

	clear32(atcphy->regs.lpdptx + LPDPTX_AUX_CFG_BLK_AUX_CTRL,
		LPDPTX_BLK_AUX_CTRL_PWRDN);
	set32(atcphy->regs.lpdptx + LPDPTX_AUX_SHM_CFG_BLK_AUX_CTRL_REG0,
	      LPDPTX_CFG_PMA_AUX_SEL_LF_DATA);
	mask32(atcphy->regs.lpdptx + LPDPTX_AUX_CFG_BLK_AUX_CTRL,
	       LPDPTX_BLK_AUX_RXOFFSET, FIELD_PREP(LPDPTX_BLK_AUX_RXOFFSET, 3));

	mask32(atcphy->regs.lpdptx + LPDPTX_AUX_CFG_BLK_AUX_MARGIN,
	       LPDPTX_AUX_MARGIN_RCAL_TXSWING,
	       FIELD_PREP(LPDPTX_AUX_MARGIN_RCAL_TXSWING, 12));

	atcphy->dp_link_rate = -1;
}

static void atcphy_disable_dp_aux(struct apple_atcphy *atcphy)
{
	set32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL, LPDPTX_AUX_PWN_DOWN);
	set32(atcphy->regs.lpdptx + LPDPTX_AUX_CFG_BLK_AUX_CTRL,
	      LPDPTX_BLK_AUX_CTRL_PWRDN);
	set32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL, LPDPTX_AUX_CLAMP_EN);
	clear32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL,
		LPDPTX_SLEEP_B_SML_IN);
	udelay(2);
	clear32(atcphy->regs.lpdptx + LPDPTX_AUX_CONTROL,
		LPDPTX_SLEEP_B_BIG_IN);
	udelay(2);

	// TODO: maybe?
	core_clear32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
		     DPTXPHY_PMA_LANE_RESET_N);
	// _OV?
	core_clear32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
		     DPRX_PCLK_ENABLE);
	core_clear32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
		     DPTX_PCLK1_ENABLE);
	core_clear32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
		     DPTX_PCLK2_ENABLE);

	// clear 0x1000000 / BIT(24) maybe
	// writel(0x1830630, atcphy->regs.core + 0x1028);
}

static int
atcphy_dp_configure_lane(struct apple_atcphy *atcphy, unsigned int lane,
			 const struct atcphy_dp_link_rate_configuration *cfg)
{
	void __iomem *tx_shm, *rx_shm, *rx_top;

	switch (lane) {
	case 0:
		tx_shm = atcphy->regs.core + LN0_AUSPMA_TX_SHM;
		rx_shm = atcphy->regs.core + LN0_AUSPMA_RX_SHM;
		rx_top = atcphy->regs.core + LN0_AUSPMA_RX_TOP;
		break;
	case 1:
		tx_shm = atcphy->regs.core + LN1_AUSPMA_TX_SHM;
		rx_shm = atcphy->regs.core + LN1_AUSPMA_RX_SHM;
		rx_top = atcphy->regs.core + LN1_AUSPMA_RX_TOP;
		break;
	default:
		return -EINVAL;
	}

	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK, LN_LDOCLK_EN_SML);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK, LN_LDOCLK_EN_SML_OV);
	udelay(2);

	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK, LN_LDOCLK_EN_BIG);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK, LN_LDOCLK_EN_BIG_OV);
	udelay(2);

	if (cfg->bypass_txa_ldoclk) {
		set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK,
		      LN_LDOCLK_BYPASS_SML);
		set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK,
		      LN_LDOCLK_BYPASS_SML_OV);
		udelay(2);

		set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK,
		      LN_LDOCLK_BYPASS_BIG);
		set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK,
		      LN_LDOCLK_BYPASS_BIG_OV);
		udelay(2);
	} else {
		clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK,
			LN_LDOCLK_BYPASS_SML);
		clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK,
			LN_LDOCLK_BYPASS_SML_OV);
		udelay(2);

		clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK,
			LN_LDOCLK_BYPASS_BIG);
		clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_LDOCLK,
			LN_LDOCLK_BYPASS_BIG_OV);
		udelay(2);
	}

	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG0,
	      LN_BYTECLK_RESET_SYNC_SEL_OV);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG0,
	      LN_BYTECLK_RESET_SYNC_EN);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG0,
	      LN_BYTECLK_RESET_SYNC_EN_OV);
	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG0,
		LN_BYTECLK_RESET_SYNC_CLR);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG0,
	      LN_BYTECLK_RESET_SYNC_CLR_OV);

	if (cfg->txa_div2_en)
		set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG1,
		      LN_TXA_DIV2_EN);
	else
		clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG1,
			LN_TXA_DIV2_EN);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG1, LN_TXA_DIV2_EN_OV);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG1, LN_TXA_CLK_EN);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG1, LN_TXA_CLK_EN_OV);
	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG1, LN_TXA_DIV2_RESET);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_CFG_MAIN_REG1,
	      LN_TXA_DIV2_RESET_OV);

	mask32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG0, LN_TXA_CAL_CTRL_BASE,
	       FIELD_PREP(LN_TXA_CAL_CTRL_BASE, 0xf));
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG0, LN_TXA_CAL_CTRL_BASE_OV);
	mask32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG0, LN_TXA_CAL_CTRL,
	       FIELD_PREP(LN_TXA_CAL_CTRL, 0x3f)); // TODO: 3f?
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG0, LN_TXA_CAL_CTRL_OV);

	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG2, LN_TXA_MARGIN);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG2, LN_TXA_MARGIN_OV);
	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG2, LN_TXA_MARGIN_2R);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG2, LN_TXA_MARGIN_2R_OV);

	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_POST);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_POST_OV);
	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_POST_2R);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_POST_2R_OV);
	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_POST_4R);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_POST_4R_OV);
	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_PRE);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_PRE_OV);
	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_PRE_2R);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_PRE_2R_OV);
	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_PRE_4R);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG3, LN_TXA_MARGIN_PRE_4R_OV);

	clear32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG0, LN_TXA_HIZ);
	set32(tx_shm + LN_AUSPMA_TX_SHM_TXA_IMP_REG0, LN_TXA_HIZ_OV);

	return 0;
}

static int
atcphy_dp_configure_lane2(struct apple_atcphy *atcphy, unsigned int lane,
			 const struct atcphy_dp_link_rate_configuration *cfg)
{
	void __iomem *tx_shm, *rx_shm, *rx_top;

	switch (lane) {
	case 0:
		tx_shm = atcphy->regs.core + LN0_AUSPMA_TX_SHM;
		rx_shm = atcphy->regs.core + LN0_AUSPMA_RX_SHM;
		rx_top = atcphy->regs.core + LN0_AUSPMA_RX_TOP;
		break;
	case 1:
		tx_shm = atcphy->regs.core + LN1_AUSPMA_TX_SHM;
		rx_shm = atcphy->regs.core + LN1_AUSPMA_RX_SHM;
		rx_top = atcphy->regs.core + LN1_AUSPMA_RX_TOP;
		break;
	default:
		return -EINVAL;
	}

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_AFE_CTRL1,
		LN_RX_DIV20_RESET_N);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_AFE_CTRL1,
	      LN_RX_DIV20_RESET_N_OV);
	udelay(2);

	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_AFE_CTRL1, LN_RX_DIV20_RESET_N);

	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL12,
	      LN_TX_BYTECLK_RESET_SYNC_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL12,
	      LN_TX_BYTECLK_RESET_SYNC_EN_OV);

	mask32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_SAVOS_CTRL16, LN_TX_CAL_CODE,
	       FIELD_PREP(LN_TX_CAL_CODE, 6)); // TODO 6?
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_SAVOS_CTRL16, LN_TX_CAL_CODE_OV);

	mask32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19,
	       LN_TX_CLK_DLY_CTRL_TAPGEN,
	       FIELD_PREP(LN_TX_CLK_DLY_CTRL_TAPGEN, 3));

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL10, LN_DTVREG_ADJUST);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL13, LN_DTVREG_ADJUST_OV);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_SAVOS_CTRL16, LN_RXTERM_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_SAVOS_CTRL16, LN_RXTERM_EN_OV);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19, LN_TX_TEST_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19, LN_TX_TEST_EN_OV);

	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22,
	      LN_VREF_TEST_RXLPBKDT_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22,
	      LN_VREF_TEST_RXLPBKDT_EN_OV);
	mask32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22,
	       LN_VREF_LPBKIN_DATA, FIELD_PREP(LN_VREF_LPBKIN_DATA, 3));
	mask32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22, LN_VREF_BIAS_SEL,
	       FIELD_PREP(LN_VREF_BIAS_SEL, 2));
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22,
	      LN_VREF_BIAS_SEL_OV);
	mask32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22,
	       LN_VREF_ADJUST_GRAY, FIELD_PREP(LN_VREF_ADJUST_GRAY, 0x18));
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22,
	      LN_VREF_ADJUST_GRAY_OV);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22, LN_VREF_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22, LN_VREF_EN_OV);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22, LN_VREF_BOOST_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22,
	      LN_VREF_BOOST_EN_OV);
	udelay(2);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22, LN_VREF_BOOST_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_VREF_CTRL22,
	      LN_VREF_BOOST_EN_OV);
	udelay(2);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL13, LN_TX_PRE_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL13, LN_TX_PRE_EN_OV);
	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL13, LN_TX_PST1_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL13, LN_TX_PST1_EN_OV);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL12, LN_TX_PBIAS_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL12, LN_TX_PBIAS_EN_OV);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_SAVOS_CTRL16,
		LN_RXTERM_PULLUP_LEAK_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_SAVOS_CTRL16,
	      LN_RXTERM_PULLUP_LEAK_EN_OV);

	set32(rx_top + LN_AUSPMA_RX_TOP_TJ_CFG_RX_TXMODE, LN_RX_TXMODE);

	if (cfg->txa_div2_en)
		set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19,
		      LN_TX_CLK_DIV2_EN);
	else
		clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19,
			LN_TX_CLK_DIV2_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19,
	      LN_TX_CLK_DIV2_EN_OV);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19,
		LN_TX_CLK_DIV2_RST);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19,
	      LN_TX_CLK_DIV2_RST_OV);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL12, LN_TX_HRCLK_SEL);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL12, LN_TX_HRCLK_SEL_OV);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL17, LN_TX_MARGIN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL17, LN_TX_MARGIN_OV);
	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL17, LN_TX_MARGIN_LSB);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL17, LN_TX_MARGIN_LSB_OV);
	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL17, LN_TX_MARGIN_P1);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL17, LN_TX_MARGIN_P1_OV);
	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL17,
		LN_TX_MARGIN_P1_LSB);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL17,
	      LN_TX_MARGIN_P1_LSB_OV);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18, LN_TX_P1_CODE);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18, LN_TX_P1_CODE_OV);
	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18, LN_TX_P1_LSB_CODE);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18, LN_TX_P1_LSB_CODE_OV);
	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18, LN_TX_MARGIN_PRE);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18, LN_TX_MARGIN_PRE_OV);
	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18,
		LN_TX_MARGIN_PRE_LSB);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18,
	      LN_TX_MARGIN_PRE_LSB_OV);
	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18, LN_TX_PRE_LSB_CODE);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18,
	      LN_TX_PRE_LSB_CODE_OV);
	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18, LN_TX_PRE_CODE);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TX_CTRL18, LN_TX_PRE_CODE_OV);

	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL11, LN_DTVREG_SML_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL11, LN_DTVREG_SML_EN_OV);
	udelay(2);

	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL11, LN_DTVREG_BIG_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL11, LN_DTVREG_BIG_EN_OV);
	udelay(2);

	mask32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL10, LN_DTVREG_ADJUST,
	       FIELD_PREP(LN_DTVREG_ADJUST, 0xa));
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL13, LN_DTVREG_ADJUST_OV);
	udelay(2);

	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19, LN_TX_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_TERM_CTRL19, LN_TX_EN_OV);
	udelay(2);

	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_CTLE_CTRL0, LN_TX_CLK_EN);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_CTLE_CTRL0, LN_TX_CLK_EN_OV);

	clear32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL12,
		LN_TX_BYTECLK_RESET_SYNC_CLR);
	set32(rx_shm + LN_AUSPMA_RX_SHM_TJ_RXA_DFE_CTRL12,
	      LN_TX_BYTECLK_RESET_SYNC_CLR_OV);

	return 0;
}

static int atcphy_auspll_apb_command(struct apple_atcphy *atcphy, u32 command)
{
	int ret;
	u32 reg;

	reg = readl(atcphy->regs.core + AUSPLL_APB_CMD_OVERRIDE);
	reg &= ~AUSPLL_APB_CMD_OVERRIDE_CMD;
	reg |= FIELD_PREP(AUSPLL_APB_CMD_OVERRIDE_CMD, command);
	reg |= AUSPLL_APB_CMD_OVERRIDE_REQ;
	reg |= AUSPLL_APB_CMD_OVERRIDE_UNK28;
	writel(reg, atcphy->regs.core + AUSPLL_APB_CMD_OVERRIDE);

	ret = readl_poll_timeout(atcphy->regs.core + AUSPLL_APB_CMD_OVERRIDE,
				 reg, (reg & AUSPLL_APB_CMD_OVERRIDE_ACK), 100,
				 100000);
	if (ret) {
		dev_err(atcphy->dev, "AUSPLL APB command was not acked.\n");
		return ret;
	}

	core_clear32(atcphy, AUSPLL_APB_CMD_OVERRIDE,
		     AUSPLL_APB_CMD_OVERRIDE_REQ);

	return 0;
}

static int atcphy_dp_configure(struct apple_atcphy *atcphy,
			       enum atcphy_dp_link_rate lr)
{
	const struct atcphy_dp_link_rate_configuration *cfg = &dp_lr_config[lr];
	const struct atcphy_mode_configuration *mode_cfg;
	int ret;
	u32 reg;

	trace_atcphy_dp_configure(atcphy, lr);

	if (atcphy->dp_link_rate == lr)
		return 0;

	if (atcphy->swap_lanes)
		mode_cfg = &atcphy_modes[atcphy->mode].swapped;
	else
		mode_cfg = &atcphy_modes[atcphy->mode].normal;

	core_clear32(atcphy, AUSPLL_FREQ_CFG, AUSPLL_FREQ_REFCLK);

	core_mask32(atcphy, AUSPLL_FREQ_DESC_A, AUSPLL_FD_FREQ_COUNT_TARGET,
		    FIELD_PREP(AUSPLL_FD_FREQ_COUNT_TARGET,
			       cfg->freqinit_count_target));
	core_clear32(atcphy, AUSPLL_FREQ_DESC_A, AUSPLL_FD_FBDIVN_HALF);
	core_clear32(atcphy, AUSPLL_FREQ_DESC_A, AUSPLL_FD_REV_DIVN);
	core_mask32(atcphy, AUSPLL_FREQ_DESC_A, AUSPLL_FD_KI_MAN,
		    FIELD_PREP(AUSPLL_FD_KI_MAN, 8));
	core_mask32(atcphy, AUSPLL_FREQ_DESC_A, AUSPLL_FD_KI_EXP,
		    FIELD_PREP(AUSPLL_FD_KI_EXP, 3));
	core_mask32(atcphy, AUSPLL_FREQ_DESC_A, AUSPLL_FD_KP_MAN,
		    FIELD_PREP(AUSPLL_FD_KP_MAN, 8));
	core_mask32(atcphy, AUSPLL_FREQ_DESC_A, AUSPLL_FD_KP_EXP,
		    FIELD_PREP(AUSPLL_FD_KP_EXP, 7));
	core_clear32(atcphy, AUSPLL_FREQ_DESC_A, AUSPLL_FD_KPKI_SCALE_HBW);

	core_mask32(atcphy, AUSPLL_FREQ_DESC_B, AUSPLL_FD_FBDIVN_FRAC_DEN,
		    FIELD_PREP(AUSPLL_FD_FBDIVN_FRAC_DEN,
			       cfg->fbdivn_frac_den));
	core_mask32(atcphy, AUSPLL_FREQ_DESC_B, AUSPLL_FD_FBDIVN_FRAC_NUM,
		    FIELD_PREP(AUSPLL_FD_FBDIVN_FRAC_NUM,
			       cfg->fbdivn_frac_num));

	core_clear32(atcphy, AUSPLL_FREQ_DESC_C, AUSPLL_FD_SDM_SSC_STEP);
	core_clear32(atcphy, AUSPLL_FREQ_DESC_C, AUSPLL_FD_SDM_SSC_EN);
	core_mask32(atcphy, AUSPLL_FREQ_DESC_C, AUSPLL_FD_PCLK_DIV_SEL,
		    FIELD_PREP(AUSPLL_FD_PCLK_DIV_SEL, cfg->pclk_div_sel));
	core_mask32(atcphy, AUSPLL_FREQ_DESC_C, AUSPLL_FD_LFSDM_DIV,
		    FIELD_PREP(AUSPLL_FD_LFSDM_DIV, 1));
	core_mask32(atcphy, AUSPLL_FREQ_DESC_C, AUSPLL_FD_LFCLK_CTRL,
		    FIELD_PREP(AUSPLL_FD_LFCLK_CTRL, cfg->lfclk_ctrl));
	core_mask32(atcphy, AUSPLL_FREQ_DESC_C, AUSPLL_FD_VCLK_OP_DIVN,
		    FIELD_PREP(AUSPLL_FD_VCLK_OP_DIVN, cfg->vclk_op_divn));
	core_set32(atcphy, AUSPLL_FREQ_DESC_C, AUSPLL_FD_VCLK_PRE_DIVN);

	core_mask32(atcphy, AUSPLL_CLKOUT_DIV, AUSPLL_CLKOUT_PLLA_REFBUFCLK_DI,
		    FIELD_PREP(AUSPLL_CLKOUT_PLLA_REFBUFCLK_DI, 7));

	if (cfg->plla_clkout_vreg_bypass)
		core_set32(atcphy, AUSPLL_CLKOUT_DTC_VREG,
			   AUSPLL_DTC_VREG_BYPASS);
	else
		core_clear32(atcphy, AUSPLL_CLKOUT_DTC_VREG,
			     AUSPLL_DTC_VREG_BYPASS);

	core_set32(atcphy, AUSPLL_BGR, AUSPLL_BGR_CTRL_AVAIL);

	core_set32(atcphy, AUSPLL_CLKOUT_MASTER,
		   AUSPLL_CLKOUT_MASTER_PCLK_DRVR_EN);
	core_set32(atcphy, AUSPLL_CLKOUT_MASTER,
		   AUSPLL_CLKOUT_MASTER_PCLK2_DRVR_EN);
	core_set32(atcphy, AUSPLL_CLKOUT_MASTER,
		   AUSPLL_CLKOUT_MASTER_REFBUFCLK_DRVR_EN);

	ret = atcphy_auspll_apb_command(atcphy, 0);
	if (ret)
		return ret;

	ret = readl_poll_timeout(atcphy->regs.core + ACIOPHY_DP_PCLK_STAT, reg,
				 (reg & ACIOPHY_AUSPLL_LOCK), 100, 100000);
	if (ret) {
		dev_err(atcphy->dev, "ACIOPHY_DP_PCLK did not lock.\n");
		return ret;
	}

	ret = atcphy_auspll_apb_command(atcphy, 0x2800);
	if (ret)
		return ret;

	if (mode_cfg->dp_lane[0]) {
		ret = atcphy_dp_configure_lane(atcphy, 0, cfg);
		if (ret)
			return ret;
	}

	if (mode_cfg->dp_lane[1]) {
		ret = atcphy_dp_configure_lane(atcphy, 1, cfg);
		if (ret)
			return ret;
	}

	if (mode_cfg->dp_lane[0]) {
		ret = atcphy_dp_configure_lane2(atcphy, 0, cfg);
		if (ret)
			return ret;
	}

	if (mode_cfg->dp_lane[1]) {
		ret = atcphy_dp_configure_lane2(atcphy, 1, cfg);
		if (ret)
			return ret;
	}

	core_clear32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
		     DP_PMA_BYTECLK_RESET);
	core_clear32(atcphy, ACIOPHY_LANE_DP_CFG_BLK_TX_DP_CTRL0,
		     DP_MAC_DIV20_CLK_SEL);

	atcphy->dp_link_rate = lr;
	return 0;
}

static int atcphy_cio_configure(struct apple_atcphy *atcphy,
				enum atcphy_mode mode)
{
	int ret;

	BUG_ON(!mutex_is_locked(&atcphy->lock));

	ret = atcphy_cio_power_on(atcphy);
	if (ret)
		return ret;

	atcphy_setup_pll_fuses(atcphy);
	atcphy_apply_tunables(atcphy, mode);

	// TODO: without this sometimes device aren't recognized but no idea what it does
	// ACIOPHY_PLL_TOP_BLK_AUSPLL_PCTL_FSM_CTRL1.APB_REQ_OV_SEL = 255
	core_set32(atcphy, 0x1014, 255 << 13);
	core_set32(atcphy, AUSPLL_APB_CMD_OVERRIDE,
		   AUSPLL_APB_CMD_OVERRIDE_UNK28);

	writel(0x10000cef, atcphy->regs.core + 0x8); // ACIOPHY_CFG0
	writel(0x15570cff, atcphy->regs.core + 0x1b0); // ACIOPHY_SLEEP_CTRL
	writel(0x11833fef, atcphy->regs.core + 0x8); // ACIOPHY_CFG0

	/* enable clocks and configure lanes */
	core_set32(atcphy, CIO3PLL_CLK_CTRL, CIO3PLL_CLK_PCLK_EN);
	core_set32(atcphy, CIO3PLL_CLK_CTRL, CIO3PLL_CLK_REFCLK_EN);
	atcphy_configure_lanes(atcphy, mode);

	/* take the USB3 PHY out of reset */
	core_set32(atcphy, ATCPHY_POWER_CTRL, ATCPHY_POWER_PHY_RESET_N);

	/* setup AUX channel if DP altmode is requested */
	if (atcphy_modes[mode].enable_dp_aux)
		atcphy_enable_dp_aux(atcphy);

	atcphy->mode = mode;
	return 0;
}

static int atcphy_usb3_power_on(struct phy *phy)
{
	struct apple_atcphy *atcphy = phy_get_drvdata(phy);
	enum atcphy_pipehandler_state state;
	int ret = 0;

	/*
	 * Both usb role switch and mux set work will be running concurrently.
	 * Make sure atcphy_mux_set_work is done bringing up ATCPHY before
	 * trying to switch dwc3 to the correct PHY.
	 */
	mutex_lock(&atcphy->lock);
	if (atcphy->mode != atcphy->target_mode) {
		reinit_completion(&atcphy->atcphy_online_event);
		mutex_unlock(&atcphy->lock);
		wait_for_completion_timeout(&atcphy->atcphy_online_event,
					msecs_to_jiffies(1000));
		mutex_lock(&atcphy->lock);
	}

	if (atcphy->mode != atcphy->target_mode) {
		dev_err(atcphy->dev, "ATCPHY did not come up; won't allow dwc3 to come up.\n");
		mutex_unlock(&atcphy->lock);
		return -EINVAL;
	}

	atcphy->dwc3_online = true;
	state = atcphy_modes[atcphy->mode].pipehandler_state;
	switch (state) {
	case ATCPHY_PIPEHANDLER_STATE_USB2:
	case ATCPHY_PIPEHANDLER_STATE_USB3:
		ret = atcphy_configure_pipehandler(atcphy, state);
		break;

	case ATCPHY_PIPEHANDLER_STATE_INVALID:
	default:
		dev_warn(atcphy->dev, "Invalid state %d in usb3_set_phy\n",
			 state);
		ret = -EINVAL;
	}

	mutex_unlock(&atcphy->lock);

	return 0;
}

static int atcphy_usb3_power_off(struct phy *phy)
{
	struct apple_atcphy *atcphy = phy_get_drvdata(phy);

	mutex_lock(&atcphy->lock);

	atcphy_configure_pipehandler(atcphy, ATCPHY_PIPEHANDLER_STATE_USB2);

	atcphy->dwc3_online = false;
	complete(&atcphy->dwc3_shutdown_event);

	mutex_unlock(&atcphy->lock);

	return 0;
}

static const struct phy_ops apple_atc_usb3_phy_ops = {
	.owner = THIS_MODULE,
	.power_on = atcphy_usb3_power_on,
	.power_off = atcphy_usb3_power_off,
};

static int atcphy_usb2_power_on(struct phy *phy)
{
	struct apple_atcphy *atcphy = phy_get_drvdata(phy);

	mutex_lock(&atcphy->lock);

	/* take the PHY out of its low power state */
	clear32(atcphy->regs.usb2phy + USB2PHY_CTL, USB2PHY_CTL_SIDDQ);
	udelay(10);

	/* reset the PHY for good measure */
	clear32(atcphy->regs.usb2phy + USB2PHY_CTL, USB2PHY_CTL_APB_RESET_N);
	set32(atcphy->regs.usb2phy + USB2PHY_CTL,
	      USB2PHY_CTL_RESET | USB2PHY_CTL_PORT_RESET);
	udelay(10);
	set32(atcphy->regs.usb2phy + USB2PHY_CTL, USB2PHY_CTL_APB_RESET_N);
	clear32(atcphy->regs.usb2phy + USB2PHY_CTL,
		USB2PHY_CTL_RESET | USB2PHY_CTL_PORT_RESET);

	set32(atcphy->regs.usb2phy + USB2PHY_SIG,
	      USB2PHY_SIG_VBUSDET_FORCE_VAL | USB2PHY_SIG_VBUSDET_FORCE_EN |
		      USB2PHY_SIG_VBUSVLDEXT_FORCE_VAL |
		      USB2PHY_SIG_VBUSVLDEXT_FORCE_EN);

	/* enable the dummy PHY for the SS lanes */
	set32(atcphy->regs.pipehandler + PIPEHANDLER_NONSELECTED_OVERRIDE,
	      PIPEHANDLER_DUMMY_PHY_EN);

	mutex_unlock(&atcphy->lock);

	return 0;
}

static int atcphy_usb2_power_off(struct phy *phy)
{
	struct apple_atcphy *atcphy = phy_get_drvdata(phy);

	mutex_lock(&atcphy->lock);

	/* reset the PHY before transitioning to low power mode */
	clear32(atcphy->regs.usb2phy + USB2PHY_CTL, USB2PHY_CTL_APB_RESET_N);
	set32(atcphy->regs.usb2phy + USB2PHY_CTL,
	      USB2PHY_CTL_RESET | USB2PHY_CTL_PORT_RESET);

	/* switch the PHY to low power mode */
	set32(atcphy->regs.usb2phy + USB2PHY_CTL, USB2PHY_CTL_SIDDQ);

	mutex_unlock(&atcphy->lock);

	return 0;
}

static int atcphy_usb2_set_mode(struct phy *phy, enum phy_mode mode,
				int submode)
{
	struct apple_atcphy *atcphy = phy_get_drvdata(phy);
	int ret;

	mutex_lock(&atcphy->lock);

	switch (mode) {
	case PHY_MODE_USB_HOST:
	case PHY_MODE_USB_HOST_LS:
	case PHY_MODE_USB_HOST_FS:
	case PHY_MODE_USB_HOST_HS:
	case PHY_MODE_USB_HOST_SS:
		set32(atcphy->regs.usb2phy + USB2PHY_SIG, USB2PHY_SIG_HOST);
		set32(atcphy->regs.usb2phy + USB2PHY_USBCTL,
		      USB2PHY_USBCTL_HOST_EN);
		ret = 0;
		break;

	case PHY_MODE_USB_DEVICE:
	case PHY_MODE_USB_DEVICE_LS:
	case PHY_MODE_USB_DEVICE_FS:
	case PHY_MODE_USB_DEVICE_HS:
	case PHY_MODE_USB_DEVICE_SS:
		clear32(atcphy->regs.usb2phy + USB2PHY_SIG, USB2PHY_SIG_HOST);
		clear32(atcphy->regs.usb2phy + USB2PHY_USBCTL,
			USB2PHY_USBCTL_HOST_EN);
		ret = 0;
		break;

	default:
		dev_err(atcphy->dev, "Unknown mode for usb2 phy: %d\n", mode);
		ret = -EINVAL;
	}

	mutex_unlock(&atcphy->lock);
	return ret;
}

static const struct phy_ops apple_atc_usb2_phy_ops = {
	.owner = THIS_MODULE,
	.set_mode = atcphy_usb2_set_mode,
	/*
	 * This PHY is always matched with a dwc3 controller. Currently,
	 * first dwc3 initializes the PHY and then soft-resets itself and
	 * then finally powers on the PHY. This should be reasonable.
	 * Annoyingly, the dwc3 soft reset is never completed when the USB2 PHY
	 * is powered off so we have to pretend that these two are actually
	 * init/exit here to ensure the PHY is powered on and out of reset
	 * early enough.
	 */
	.init = atcphy_usb2_power_on,
	.exit = atcphy_usb2_power_off,
};

static int atcphy_dpphy_mux_set(struct apple_atcphy *atcphy, enum atcphy_mode target)
{
	int ret = 0;

	// TODO:
	flush_work(&atcphy->mux_set_work);

	mutex_lock(&atcphy->lock);

	if (atcphy->mode == target)
		goto out_unlock;

	atcphy->target_mode = target;

	WARN_ON(!schedule_work(&atcphy->mux_set_work));
	ret = wait_for_completion_timeout(&atcphy->atcphy_online_event,
					  msecs_to_jiffies(1000));
	if (ret == 0)
		ret = -ETIMEDOUT;
	else if (ret > 0)
		ret = 0;

out_unlock:
	mutex_unlock(&atcphy->lock);
	return ret;
}

static int atcphy_dpphy_set_mode(struct phy *phy, enum phy_mode mode,
				 int submode)
{
	struct apple_atcphy *atcphy = phy_get_drvdata(phy);

	if (!atcphy->dp_only)
		return 0;

	dev_info(atcphy->dev, "%s(mode=%u, submode=%d)\n", __func__, mode, submode);

	switch (mode) {
	case PHY_MODE_INVALID:
		if (atcphy->mode == APPLE_ATCPHY_MODE_OFF)
			return 0;
		return atcphy_dpphy_mux_set(atcphy, APPLE_ATCPHY_MODE_OFF);
	case PHY_MODE_DP:
		/* TODO: does this get called for DP-altmode? */
		if (atcphy->mode == APPLE_ATCPHY_MODE_USB3_DP ||
		    atcphy->mode == APPLE_ATCPHY_MODE_DP)
			return 0;
		return atcphy_dpphy_mux_set(atcphy, APPLE_ATCPHY_MODE_DP);
	default:
		break;
	}

	return -EINVAL;
}

static int atcphy_dpphy_validate(struct phy *phy, enum phy_mode mode,
				 int submode, union phy_configure_opts *opts_)
{
	struct phy_configure_opts_dp *opts = &opts_->dp;
	struct apple_atcphy *atcphy = phy_get_drvdata(phy);

	if (mode == PHY_MODE_INVALID) {
		memset(opts, 0, sizeof(*opts));
		return 0;
	}

	if (mode != PHY_MODE_DP)
		return -EINVAL;
	if (submode != 0)
		return -EINVAL;

	switch (atcphy->mode) {
	case APPLE_ATCPHY_MODE_USB3_DP:
		opts->lanes = 2;
		break;
	case APPLE_ATCPHY_MODE_DP:
		opts->lanes = 4;
		break;
	default:
		opts->lanes = 0;
	}

	opts->link_rate = 8100;

	for (int i = 0; i < 4; ++i) {
		opts->voltage[i] = 3;
		opts->pre[i] = 3;
	}

	return 0;
}

static int atcphy_dpphy_configure(struct phy *phy,
				  union phy_configure_opts *opts_)
{
	struct phy_configure_opts_dp *opts = &opts_->dp;
	struct apple_atcphy *atcphy = phy_get_drvdata(phy);
	enum atcphy_dp_link_rate link_rate;
	int ret = 0;

	/* might be possibly but we don't know how */
	if (opts->set_voltages)
		return -EINVAL;

	/*
	 * Just ack set_lanes for compatibility with (lp)dptx-phy
	 * The mux_set should've done this anyway
	 */
	if (opts->set_lanes) {
		if (((atcphy->mode == APPLE_ATCPHY_MODE_DP && opts->lanes != 4) ||
		     (atcphy->mode == APPLE_ATCPHY_MODE_USB3_DP && opts->lanes != 2)) &&
	            (atcphy->mode == APPLE_ATCPHY_MODE_OFF && opts->lanes != 0))
			dev_warn(atcphy->dev, "Unexpected lane count %u for mode %u\n",
				 opts->lanes, atcphy->mode);

	}

	if (opts->set_rate) {
		switch (opts->link_rate) {
		case 1620:
			link_rate = ATCPHY_DP_LINK_RATE_RBR;
			break;
		case 2700:
			link_rate = ATCPHY_DP_LINK_RATE_HBR;
			break;
		case 5400:
			link_rate = ATCPHY_DP_LINK_RATE_HBR2;
			break;
		case 8100:
			link_rate = ATCPHY_DP_LINK_RATE_HBR3;
			break;
		case 0:
			// TODO: disable!
			return 0;
			break;
		default:
			dev_err(atcphy->dev, "Unsupported link rate: %d\n",
				opts->link_rate);
			return -EINVAL;
		}

		mutex_lock(&atcphy->lock);
		ret = atcphy_dp_configure(atcphy, link_rate);
		mutex_unlock(&atcphy->lock);
	}

	return ret;
}

static const struct phy_ops apple_atc_dp_phy_ops = {
	.owner = THIS_MODULE,
	.configure = atcphy_dpphy_configure,
	.validate = atcphy_dpphy_validate,
	.set_mode = atcphy_dpphy_set_mode,
};

static struct phy *atcphy_xlate(struct device *dev,
				const struct of_phandle_args *args)
{
	struct apple_atcphy *atcphy = dev_get_drvdata(dev);

	switch (args->args[0]) {
	case PHY_TYPE_USB2:
		return atcphy->phy_usb2;
	case PHY_TYPE_USB3:
		return atcphy->phy_usb3;
	case PHY_TYPE_DP:
		return atcphy->phy_dp;
	}
	return ERR_PTR(-ENODEV);
}

static int atcphy_probe_phy(struct apple_atcphy *atcphy)
{
	atcphy->phy_usb2 =
		devm_phy_create(atcphy->dev, NULL, &apple_atc_usb2_phy_ops);
	if (IS_ERR(atcphy->phy_usb2))
		return PTR_ERR(atcphy->phy_usb2);
	phy_set_drvdata(atcphy->phy_usb2, atcphy);

	atcphy->phy_usb3 =
		devm_phy_create(atcphy->dev, NULL, &apple_atc_usb3_phy_ops);
	if (IS_ERR(atcphy->phy_usb3))
		return PTR_ERR(atcphy->phy_usb3);
	phy_set_drvdata(atcphy->phy_usb3, atcphy);

	atcphy->phy_dp =
		devm_phy_create(atcphy->dev, NULL, &apple_atc_dp_phy_ops);
	if (IS_ERR(atcphy->phy_dp))
		return PTR_ERR(atcphy->phy_dp);
	phy_set_drvdata(atcphy->phy_dp, atcphy);

	atcphy->phy_provider =
		devm_of_phy_provider_register(atcphy->dev, atcphy_xlate);
	if (IS_ERR(atcphy->phy_provider))
		return PTR_ERR(atcphy->phy_provider);

	return 0;
}

static int atcphy_dwc3_reset_assert(struct reset_controller_dev *rcdev,
				    unsigned long id)
{
	struct apple_atcphy *atcphy = rcdev_to_apple_atcphy(rcdev);

	clear32(atcphy->regs.pipehandler + PIPEHANDLER_AON_GEN,
		PIPEHANDLER_AON_GEN_DWC3_RESET_N);
	set32(atcphy->regs.pipehandler + PIPEHANDLER_AON_GEN,
	      PIPEHANDLER_AON_GEN_DWC3_FORCE_CLAMP_EN);

	return 0;
}

static int atcphy_dwc3_reset_deassert(struct reset_controller_dev *rcdev,
				      unsigned long id)
{
	struct apple_atcphy *atcphy = rcdev_to_apple_atcphy(rcdev);

	clear32(atcphy->regs.pipehandler + PIPEHANDLER_AON_GEN,
		PIPEHANDLER_AON_GEN_DWC3_FORCE_CLAMP_EN);
	set32(atcphy->regs.pipehandler + PIPEHANDLER_AON_GEN,
	      PIPEHANDLER_AON_GEN_DWC3_RESET_N);

	return 0;
}

const struct reset_control_ops atcphy_dwc3_reset_ops = {
	.assert = atcphy_dwc3_reset_assert,
	.deassert = atcphy_dwc3_reset_deassert,
};

static int atcphy_reset_xlate(struct reset_controller_dev *rcdev,
			      const struct of_phandle_args *reset_spec)
{
	return 0;
}

static int atcphy_probe_rcdev(struct apple_atcphy *atcphy)
{
	atcphy->rcdev.owner = THIS_MODULE;
	atcphy->rcdev.nr_resets = 1;
	atcphy->rcdev.ops = &atcphy_dwc3_reset_ops;
	atcphy->rcdev.of_node = atcphy->dev->of_node;
	atcphy->rcdev.of_reset_n_cells = 0;
	atcphy->rcdev.of_xlate = atcphy_reset_xlate;

	return devm_reset_controller_register(atcphy->dev, &atcphy->rcdev);
}

static int atcphy_sw_set(struct typec_switch_dev *sw,
			 enum typec_orientation orientation)
{
	struct apple_atcphy *atcphy = typec_switch_get_drvdata(sw);

	trace_atcphy_sw_set(orientation);

	mutex_lock(&atcphy->lock);
	switch (orientation) {
	case TYPEC_ORIENTATION_NONE:
		break;
	case TYPEC_ORIENTATION_NORMAL:
		atcphy->swap_lanes = false;
		break;
	case TYPEC_ORIENTATION_REVERSE:
		atcphy->swap_lanes = true;
		break;
	}
	mutex_unlock(&atcphy->lock);

	return 0;
}

static int atcphy_probe_switch(struct apple_atcphy *atcphy)
{
	struct typec_switch_desc sw_desc = {
		.drvdata = atcphy,
		.fwnode = atcphy->dev->fwnode,
		.set = atcphy_sw_set,
	};

	return PTR_ERR_OR_ZERO(typec_switch_register(atcphy->dev, &sw_desc));
}

static void atcphy_mux_set_work(struct work_struct *work)
{
	struct apple_atcphy *atcphy = container_of(work, struct apple_atcphy, mux_set_work);

	mutex_lock(&atcphy->lock);
	/*
	 * If we're transitiong to TYPEC_STATE_SAFE dwc3 will have gotten
	 * a usb-role-switch event to ROLE_NONE which is deferred to a work
	 * queue. dwc3 will try to switch the pipehandler mux to USB2 and
	 * we have to make sure that has happened before we disable ATCPHY.
	 * If we instead disable ATCPHY first dwc3 will get stuck and the
	 * port won't work anymore until a full SoC reset.
	 * We're guaranteed that no other role switch event will be generated
	 * before we return because the mux_set callback runs in the same
	 * thread that generates these. We can thus unlock the mutex, wait
	 * for dwc3_shutdown_event from the usb3 phy's power_off callback after
	 * it has taken the mutex and the lock again.
	 */
	if (atcphy->dwc3_online && atcphy->target_mode == APPLE_ATCPHY_MODE_OFF) {
		reinit_completion(&atcphy->dwc3_shutdown_event);
		mutex_unlock(&atcphy->lock);
		wait_for_completion_timeout(&atcphy->dwc3_shutdown_event,
					    msecs_to_jiffies(1000));
		mutex_lock(&atcphy->lock);
		WARN_ON(atcphy->dwc3_online);
	}

	switch (atcphy->target_mode) {
	case APPLE_ATCPHY_MODE_DP:
	case APPLE_ATCPHY_MODE_USB3_DP:
	case APPLE_ATCPHY_MODE_USB3:
	case APPLE_ATCPHY_MODE_USB4:
		atcphy_cio_configure(atcphy, atcphy->target_mode);
		break;
	default:
		dev_warn(atcphy->dev, "Unknown mode %d in atcphy_mux_set\n",
			 atcphy->target_mode);
		fallthrough;
	case APPLE_ATCPHY_MODE_USB2:
	case APPLE_ATCPHY_MODE_OFF:
		atcphy->mode = APPLE_ATCPHY_MODE_OFF;
		atcphy_disable_dp_aux(atcphy);
		atcphy_cio_power_off(atcphy);
	}

	complete(&atcphy->atcphy_online_event);
	mutex_unlock(&atcphy->lock);
}

static int atcphy_mux_set(struct typec_mux_dev *mux,
			  struct typec_mux_state *state)
{
	struct apple_atcphy *atcphy = typec_mux_get_drvdata(mux);

	// TODO: 
	flush_work(&atcphy->mux_set_work);

	mutex_lock(&atcphy->lock);
	trace_atcphy_mux_set(state);

	if (state->mode == TYPEC_STATE_SAFE) {
		atcphy->target_mode = APPLE_ATCPHY_MODE_OFF;
	} else if (state->mode == TYPEC_STATE_USB) {
		atcphy->target_mode = APPLE_ATCPHY_MODE_USB3;
	} else if (state->alt && state->alt->svid == USB_TYPEC_DP_SID) {
		switch (state->mode) {
		case TYPEC_DP_STATE_C:
		case TYPEC_DP_STATE_E:
			atcphy->target_mode = APPLE_ATCPHY_MODE_DP;
			break;
		case TYPEC_DP_STATE_D:
			atcphy->target_mode = APPLE_ATCPHY_MODE_USB3_DP;
			break;
		default:
			dev_err(atcphy->dev,
				"Unsupported DP pin assignment: 0x%lx.\n",
				state->mode);
			atcphy->target_mode = APPLE_ATCPHY_MODE_OFF;
		}
	} else if (state->alt && state->alt->svid == USB_TYPEC_TBT_SID) {
		dev_err(atcphy->dev, "USB4/TBT mode is not supported yet.\n");
		atcphy->target_mode = APPLE_ATCPHY_MODE_OFF;
	} else if (state->alt) {
		dev_err(atcphy->dev, "Unknown alternate mode SVID: 0x%x\n",
			state->alt->svid);
		atcphy->target_mode = APPLE_ATCPHY_MODE_OFF;
	} else {
		dev_err(atcphy->dev, "Unknown mode: 0x%lx\n", state->mode);
		atcphy->target_mode = APPLE_ATCPHY_MODE_OFF;
	}

	if (atcphy->mode != atcphy->target_mode)
		WARN_ON(!schedule_work(&atcphy->mux_set_work));

	mutex_unlock(&atcphy->lock);

	return 0;
}

static int atcphy_probe_mux(struct apple_atcphy *atcphy)
{
	struct typec_mux_desc mux_desc = {
		.drvdata = atcphy,
		.fwnode = atcphy->dev->fwnode,
		.set = atcphy_mux_set,
	};

	return PTR_ERR_OR_ZERO(typec_mux_register(atcphy->dev, &mux_desc));
}

static int atcphy_parse_legacy_tunable(struct apple_atcphy *atcphy,
				       struct atcphy_tunable *tunable,
				       const char *name)
{
	struct property *prop;
	const __le32 *p = NULL;
	int i;

#if 0
	WARN_TAINT_ONCE(1, TAINT_FIRMWARE_WORKAROUND,
			"parsing legacy tunable; please update m1n1");
#endif

	prop = of_find_property(atcphy->np, name, NULL);
	if (!prop) {
		dev_err(atcphy->dev, "tunable %s not found\n", name);
		return -ENOENT;
	}

	if (prop->length % (3 * sizeof(u32)))
		return -EINVAL;

	tunable->sz = prop->length / (3 * sizeof(u32));
	tunable->values = devm_kcalloc(atcphy->dev, tunable->sz,
				       sizeof(*tunable->values), GFP_KERNEL);
	if (!tunable->values)
		return -ENOMEM;

	for (i = 0; i < tunable->sz; ++i) {
		p = of_prop_next_u32(prop, p, &tunable->values[i].offset);
		p = of_prop_next_u32(prop, p, &tunable->values[i].mask);
		p = of_prop_next_u32(prop, p, &tunable->values[i].value);
	}

	trace_atcphy_parsed_tunable(name, tunable);

	return 0;
}

static int atcphy_parse_new_tunable(struct apple_atcphy *atcphy,
				    struct atcphy_tunable *tunable,
				    const char *name)
{
	struct property *prop;
	u64 *fdt_tunable;
	int ret, i;

	prop = of_find_property(atcphy->np, name, NULL);
	if (!prop) {
		dev_err(atcphy->dev, "tunable %s not found\n", name);
		return -ENOENT;
	}

	if (prop->length % (4 * sizeof(u64)))
		return -EINVAL;

	fdt_tunable = kzalloc(prop->length, GFP_KERNEL);
	if (!fdt_tunable)
		return -ENOMEM;

	tunable->sz = prop->length / (4 * sizeof(u64));
	ret = of_property_read_variable_u64_array(atcphy->np, name, fdt_tunable,
						  tunable->sz, tunable->sz);
	if (ret < 0)
		goto err_free_fdt;

	tunable->values = devm_kcalloc(atcphy->dev, tunable->sz,
				       sizeof(*tunable->values), GFP_KERNEL);
	if (!tunable->values) {
		ret = -ENOMEM;
		goto err_free_fdt;
	}

	for (i = 0; i < tunable->sz; ++i) {
		u32 offset, size, mask, value;

		offset = fdt_tunable[4 * i];
		size = fdt_tunable[4 * i + 1];
		mask = fdt_tunable[4 * i + 2];
		value = fdt_tunable[4 * i + 3];

		if (offset > U32_MAX || size != 4 || mask > U32_MAX ||
		    value > U32_MAX) {
			ret = -EINVAL;
			goto err_free_values;
		}

		tunable->values[i].offset = offset;
		tunable->values[i].mask = mask;
		tunable->values[i].value = value;
	}

	trace_atcphy_parsed_tunable(name, tunable);
	kfree(fdt_tunable);

	BUG_ON(1);
	return 0;

err_free_values:
	devm_kfree(atcphy->dev, tunable->values);
err_free_fdt:
	kfree(fdt_tunable);
	return ret;
}

static int atcphy_parse_tunable(struct apple_atcphy *atcphy,
				struct atcphy_tunable *tunable,
				const char *name)
{
	int ret;

	if (!of_find_property(atcphy->np, name, NULL)) {
		dev_err(atcphy->dev, "tunable %s not found\n", name);
		return -ENOENT;
	}

	ret = atcphy_parse_new_tunable(atcphy, tunable, name);
	if (ret)
		ret = atcphy_parse_legacy_tunable(atcphy, tunable, name);

	return ret;
}

static int atcphy_load_tunables(struct apple_atcphy *atcphy)
{
	int ret;

	ret = atcphy_parse_tunable(atcphy, &atcphy->tunables.axi2af,
				   "apple,tunable-axi2af");
	if (ret)
		return ret;
	ret = atcphy_parse_tunable(atcphy, &atcphy->tunables.common,
				   "apple,tunable-common");
	if (ret)
		return ret;
	ret = atcphy_parse_tunable(atcphy, &atcphy->tunables.lane_usb3[0],
				   "apple,tunable-lane0-usb");
	if (ret)
		return ret;
	ret = atcphy_parse_tunable(atcphy, &atcphy->tunables.lane_usb3[1],
				   "apple,tunable-lane1-usb");
	if (ret)
		return ret;
	ret = atcphy_parse_tunable(atcphy, &atcphy->tunables.lane_usb4[0],
				   "apple,tunable-lane0-cio");
	if (ret)
		return ret;
	ret = atcphy_parse_tunable(atcphy, &atcphy->tunables.lane_usb4[1],
				   "apple,tunable-lane1-cio");
	if (ret)
		return ret;
	ret = atcphy_parse_tunable(atcphy,
				   &atcphy->tunables.lane_displayport[0],
				   "apple,tunable-lane0-dp");
	if (ret)
		return ret;
	ret = atcphy_parse_tunable(atcphy,
				   &atcphy->tunables.lane_displayport[1],
				   "apple,tunable-lane1-dp");
	if (ret)
		return ret;

	return 0;
}

static int atcphy_load_fuses(struct apple_atcphy *atcphy)
{
	int ret;

	ret = nvmem_cell_read_variable_le_u32(
		atcphy->dev, "aus_cmn_shm_vreg_trim",
		&atcphy->fuses.aus_cmn_shm_vreg_trim);
	if (ret)
		return ret;
	ret = nvmem_cell_read_variable_le_u32(
		atcphy->dev, "auspll_rodco_encap",
		&atcphy->fuses.auspll_rodco_encap);
	if (ret)
		return ret;
	ret = nvmem_cell_read_variable_le_u32(
		atcphy->dev, "auspll_rodco_bias_adjust",
		&atcphy->fuses.auspll_rodco_bias_adjust);
	if (ret)
		return ret;
	ret = nvmem_cell_read_variable_le_u32(
		atcphy->dev, "auspll_fracn_dll_start_capcode",
		&atcphy->fuses.auspll_fracn_dll_start_capcode);
	if (ret)
		return ret;
	ret = nvmem_cell_read_variable_le_u32(
		atcphy->dev, "auspll_dtc_vreg_adjust",
		&atcphy->fuses.auspll_dtc_vreg_adjust);
	if (ret)
		return ret;
	ret = nvmem_cell_read_variable_le_u32(
		atcphy->dev, "cio3pll_dco_coarsebin0",
		&atcphy->fuses.cio3pll_dco_coarsebin[0]);
	if (ret)
		return ret;
	ret = nvmem_cell_read_variable_le_u32(
		atcphy->dev, "cio3pll_dco_coarsebin1",
		&atcphy->fuses.cio3pll_dco_coarsebin[1]);
	if (ret)
		return ret;
	ret = nvmem_cell_read_variable_le_u32(
		atcphy->dev, "cio3pll_dll_start_capcode",
		&atcphy->fuses.cio3pll_dll_start_capcode[0]);
	if (ret)
		return ret;
	ret = nvmem_cell_read_variable_le_u32(
		atcphy->dev, "cio3pll_dtc_vreg_adjust",
		&atcphy->fuses.cio3pll_dtc_vreg_adjust);
	if (ret)
		return ret;

	/* 
	 * Only one of the two t8103 PHYs requires the following additional fuse
	 * and a slighly different configuration sequence if it's present.
	 * The other t8103 instance and all t6000 instances don't which means
	 * we must not fail here in case the fuse isn't present.
	 */
	ret = nvmem_cell_read_variable_le_u32(
		atcphy->dev, "cio3pll_dll_start_capcode_workaround",
		&atcphy->fuses.cio3pll_dll_start_capcode[1]);
	switch (ret) {
	case 0:
		atcphy->quirks.t8103_cio3pll_workaround = true;
		break;
	case -ENOENT:
		atcphy->quirks.t8103_cio3pll_workaround = false;
		break;
	default:
		return ret;
	}

	atcphy->fuses.present = true;

	trace_atcphy_fuses(atcphy);
	return 0;
}

static int atcphy_probe(struct platform_device *pdev)
{
	struct apple_atcphy *atcphy;
	struct device *dev = &pdev->dev;
	int ret;

	atcphy = devm_kzalloc(&pdev->dev, sizeof(*atcphy), GFP_KERNEL);
	if (!atcphy)
		return -ENOMEM;

	atcphy->dev = dev;
	atcphy->np = dev->of_node;
	platform_set_drvdata(pdev, atcphy);

	mutex_init(&atcphy->lock);
	init_completion(&atcphy->dwc3_shutdown_event);
	init_completion(&atcphy->atcphy_online_event);
	INIT_WORK(&atcphy->mux_set_work, atcphy_mux_set_work);

	atcphy->regs.core = devm_platform_ioremap_resource_byname(pdev, "core");
	if (IS_ERR(atcphy->regs.core))
		return PTR_ERR(atcphy->regs.core);
	atcphy->regs.lpdptx =
		devm_platform_ioremap_resource_byname(pdev, "lpdptx");
	if (IS_ERR(atcphy->regs.lpdptx))
		return PTR_ERR(atcphy->regs.lpdptx);
	atcphy->regs.axi2af =
		devm_platform_ioremap_resource_byname(pdev, "axi2af");
	if (IS_ERR(atcphy->regs.axi2af))
		return PTR_ERR(atcphy->regs.axi2af);
	atcphy->regs.usb2phy =
		devm_platform_ioremap_resource_byname(pdev, "usb2phy");
	if (IS_ERR(atcphy->regs.usb2phy))
		return PTR_ERR(atcphy->regs.usb2phy);
	atcphy->regs.pipehandler =
		devm_platform_ioremap_resource_byname(pdev, "pipehandler");
	if (IS_ERR(atcphy->regs.pipehandler))
		return PTR_ERR(atcphy->regs.pipehandler);

	if (of_property_read_bool(dev->of_node, "nvmem-cells")) {
		ret = atcphy_load_fuses(atcphy);
		if (ret)
			return ret;
	}

	ret = atcphy_load_tunables(atcphy);
	if (ret)
		return ret;

	atcphy->dp_only = of_property_read_bool(dev->of_node, "apple,mode-fixed-dp");

	atcphy->mode = APPLE_ATCPHY_MODE_OFF;
	atcphy->pipehandler_state = ATCPHY_PIPEHANDLER_STATE_INVALID;

	if (!atcphy->dp_only) {
		ret = atcphy_probe_rcdev(atcphy);
		if (ret)
			return ret;
		ret = atcphy_probe_mux(atcphy);
		if (ret)
			return ret;
		ret = atcphy_probe_switch(atcphy);
		if (ret)
			return ret;
	}

	ret = atcphy_probe_phy(atcphy);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id atcphy_match[] = {
	{
		.compatible = "apple,t8103-atcphy",
	},
	{
		.compatible = "apple,t6000-atcphy",
	},
	{},
};
MODULE_DEVICE_TABLE(of, atcphy_match);

static struct platform_driver atcphy_driver = {
	.driver = {
		.name = "phy-apple-atc",
		.of_match_table = atcphy_match,
	},
	.probe = atcphy_probe,
};

module_platform_driver(atcphy_driver);

MODULE_AUTHOR("Sven Peter <sven@svenpeter.dev>");
MODULE_DESCRIPTION("Apple Type-C PHY driver");

MODULE_LICENSE("GPL");
