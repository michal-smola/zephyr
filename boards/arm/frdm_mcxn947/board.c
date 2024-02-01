/*
 * Copyright 2023  NXP
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <fsl_clock.h>
#include <fsl_spc.h>

#if CONFIG_USB_DC_NXP_EHCI
#include "usb_phy.h"
#include "usb.h"

/* USB PHY condfiguration */
#define BOARD_USB_PHY_D_CAL     (0x04U)
#define BOARD_USB_PHY_TXCAL45DP (0x07U)
#define BOARD_USB_PHY_TXCAL45DM (0x07U)
#endif

/* Board xtal frequency in Hz */
#define BOARD_XTAL0_CLK_HZ                        24000000U
/* Core clock frequency: 150MHz */
#define CLOCK_INIT_CORE_CLOCK                     150000000U
/* System clock frequency. */
extern uint32_t SystemCoreClock;

#if CONFIG_FLASH_MCUX_FLEXSPI_NOR
#include "memc_mcux_flexspi.h"
#include "flash_mcux_flexspi_nor.h"

int flash_flexspi_chip_init(const struct device *dev)
{
	struct flash_flexspi_nor_data *data = dev->data;
	struct device memc_dev = data->controller;
	struct memc_flexspi_data *memc_data = memc_dev.data;
	int ret = 0;
	flexspi_transfer_t transfer;

	uint32_t ResetFlashCommandSeq[4] = {
		[4 * READ_FAST_QUAD_OUTPUT] =
			FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x66,
					kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x99),
		[4 * READ_FAST_QUAD_OUTPUT + 1] =
			FLEXSPI_LUT_SEQ(kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00,
					kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00),
	};
	uint32_t FastReadSDRLUTCommandSeq[4] = {
		/* Read data */
		[4 * READ_FAST_QUAD_OUTPUT] =
			FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xEB,
					kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_4PAD, 0x18),
		[4 * READ_FAST_QUAD_OUTPUT + 1] =
			FLEXSPI_LUT_SEQ(kFLEXSPI_Command_MODE8_SDR, kFLEXSPI_4PAD, 0xF0,
					kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_4PAD, 0x04),
		[4 * READ_FAST_QUAD_OUTPUT + 2] =
			FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_SDR, kFLEXSPI_4PAD, 0x00,
					kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0x00),
	};

	FLEXSPI_UpdateLUT(memc_data->base, 4 * READ_FAST_QUAD_OUTPUT,
				ResetFlashCommandSeq, 4);

	/* Write enable */
	transfer.deviceAddress = 0;
	transfer.port = data->port;
	transfer.cmdType = kFLEXSPI_Command;
	transfer.SeqNumber = 1;

	transfer.seqIndex = READ_FAST_QUAD_OUTPUT;

	ret = memc_flexspi_transfer(&data->controller, &transfer);

	FLEXSPI_UpdateLUT(memc_data->base, 4 * READ_FAST_QUAD_OUTPUT,
				FastReadSDRLUTCommandSeq, 4);

	return ret;
}
#endif

__ramfunc static void enable_lpcac(void)
{
	SYSCON->LPCAC_CTRL |= SYSCON_LPCAC_CTRL_CLR_LPCAC_MASK;
	SYSCON->LPCAC_CTRL &= ~(SYSCON_LPCAC_CTRL_CLR_LPCAC_MASK | SYSCON_LPCAC_CTRL_DIS_LPCAC_MASK);
}

#if !(CONFIG_FLASH_DISABLE_CACHE64)
__ramfunc static void enable_cache64(void)
{
	/* Make sure the FlexSPI clock is enabled. This is required to access the cache64
	 * registers. If running from FlexSPI the clock should already be enabled anyway.
	 */
	SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL0_FLEXSPI_MASK;

	/* Configure the CACHE64_POLSEL */

	/* configure reg0 to cover the whole FlexSPI */
	CACHE64_POLSEL0->REG0_TOP = 0x7FFC00;

	/* region 0 = write-through
	 * region 1 = invalid
	 * region 2 = invalid
	 */
	CACHE64_POLSEL0->POLSEL = (CACHE64_POLSEL_POLSEL_REG0_POLICY(1) |
				   CACHE64_POLSEL_POLSEL_REG1_POLICY(3) |
				   CACHE64_POLSEL_POLSEL_REG2_POLICY(3));

	/* Configure the CACHE64_CTRL
	 * Set invaliate bits for both ways
	 * Set GO bit to start the invalidate
	 * Enable write buffer - shouldn't matter because we aren't writing
	 * Enable the cache
	 */
	CACHE64_CTRL0->CCR = (CACHE64_CTRL_CCR_INVW0_MASK |
			      CACHE64_CTRL_CCR_INVW1_MASK |
			      CACHE64_CTRL_CCR_GO_MASK |
			      CACHE64_CTRL_CCR_ENWRBUF_MASK |
			      CACHE64_CTRL_CCR_ENCACHE_MASK);
}
#endif

/* Update Active mode voltage for OverDrive mode. */
void power_mode_od(void)
{
	spc_active_mode_dcdc_option_t opt = {
		.DCDCVoltage       = kSPC_DCDC_OverdriveVoltage,
		.DCDCDriveStrength = kSPC_DCDC_NormalDriveStrength,
	};
	SPC_SetActiveModeDCDCRegulatorConfig(SPC0, &opt);

	spc_sram_voltage_config_t cfg = {
		.operateVoltage       = kSPC_sramOperateAt1P2V,
		.requestVoltageUpdate = true,
	};
	SPC_SetSRAMOperateVoltage(SPC0, &cfg);
}

static int frdm_mcxn947_init(void)
{
	enable_lpcac();

	power_mode_od();

	/* Enable SCG clock */
	CLOCK_EnableClock(kCLOCK_Scg);

	/* FRO OSC setup - begin, enable the FRO for safety switching */

	/* Switch to FRO 12M first to ensure we can change the clock setting */
	CLOCK_AttachClk(kFRO12M_to_MAIN_CLK);

	/* Set the additional number of flash wait-states */
	CLOCK_SetFLASHAccessCyclesForFreq(150000000U, kOD_Mode);

	/* Enable FRO HF(48MHz) output */
	CLOCK_SetupFROHFClocking(48000000U);

	/* Set up PLL0 */
	const pll_setup_t pll0Setup = {
		.pllctrl = SCG_APLLCTRL_SOURCE(1U) | SCG_APLLCTRL_SELI(27U) |
			   SCG_APLLCTRL_SELP(13U),
		.pllndiv = SCG_APLLNDIV_NDIV(8U),
		.pllpdiv = SCG_APLLPDIV_PDIV(1U),
		.pllmdiv = SCG_APLLMDIV_MDIV(50U),
		.pllRate = 150000000U
	};
	/* Configure PLL0 to the desired values */
	CLOCK_SetPLL0Freq(&pll0Setup);
	/* PLL0 Monitor is disabled */
	CLOCK_SetPll0MonitorMode(kSCG_Pll0MonitorDisable);

	/* Switch MAIN_CLK to PLL0 */
	CLOCK_AttachClk(kPLL0_to_MAIN_CLK);

	/* Set AHBCLKDIV divider to value 1 */
	CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 1U);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(flexcomm1), okay)
	CLOCK_SetClkDiv(kCLOCK_DivFlexcom1Clk, 1u);
	CLOCK_AttachClk(kFRO12M_to_FLEXCOMM1);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(flexcomm2), okay)
	CLOCK_SetClkDiv(kCLOCK_DivFlexcom2Clk, 1u);
	CLOCK_AttachClk(kFRO12M_to_FLEXCOMM2);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(flexcomm4), okay)
	CLOCK_SetClkDiv(kCLOCK_DivFlexcom4Clk, 1u);
	CLOCK_AttachClk(kFRO12M_to_FLEXCOMM4);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(os_timer), okay)
	CLOCK_AttachClk(kCLK_1M_to_OSTIMER);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio0), okay)
	CLOCK_EnableClock(kCLOCK_Gpio0);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio1), okay)
	CLOCK_EnableClock(kCLOCK_Gpio1);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio2), okay)
	CLOCK_EnableClock(kCLOCK_Gpio2);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio3), okay)
	CLOCK_EnableClock(kCLOCK_Gpio3);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio4), okay)
	CLOCK_EnableClock(kCLOCK_Gpio4);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio5), okay)
	CLOCK_EnableClock(kCLOCK_Gpio5);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(sc_timer), okay)
	/* attach FRO HF to SCT */
	CLOCK_SetClkDiv(kCLOCK_DivSctClk, 1u);
	CLOCK_AttachClk(kFRO_HF_to_SCT);
#endif

#if !defined(CONFIG_CODE_FLEXSPI)
#if DT_NODE_HAS_STATUS(DT_NODELABEL(flexspi), okay)
	/* Flexspi frequency 150MHz / 2 = 75MHz */
	CLOCK_SetClkDiv(kCLOCK_DivFlexspiClk, 2U);
	/* Switch FLEXSPI to PLL0 */
	CLOCK_AttachClk(kPLL0_to_FLEXSPI);
#if !(CONFIG_FLASH_DISABLE_CACHE64)
	/* Enable CACHE64 for FlexSPI */
	enable_cache64();
#endif
#endif
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpadc0), okay)
	/* attach FRO HF to ADC0 */
	CLOCK_SetClkDiv(kCLOCK_DivAdc0Clk, 1u);
	CLOCK_AttachClk(kFRO_HF_to_ADC0);
	/* enable VREF */
	SPC0->ACTIVE_CFG1 |= 0x1;
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(usb1), okay) && CONFIG_USB_DC_NXP_EHCI
	usb_phy_config_struct_t usbPhyConfig = {
		BOARD_USB_PHY_D_CAL, BOARD_USB_PHY_TXCAL45DP, BOARD_USB_PHY_TXCAL45DM,
	};

	SPC0->ACTIVE_VDELAY = 0x0500;
	/* Change the power DCDC to 1.8v (By default, DCDC is 1.8V), CORELDO to 1.1v (By deafult, CORELDO is 1.0V) */
	SPC0->ACTIVE_CFG &= ~SPC_ACTIVE_CFG_CORELDO_VDD_DS_MASK;
	SPC0->ACTIVE_CFG |= SPC_ACTIVE_CFG_DCDC_VDD_LVL(0x3) | SPC_ACTIVE_CFG_CORELDO_VDD_LVL(0x3) |
						SPC_ACTIVE_CFG_SYSLDO_VDD_DS_MASK | SPC_ACTIVE_CFG_DCDC_VDD_DS(0x2u);
	/* Wait until it is done */
	while (SPC0->SC & SPC_SC_BUSY_MASK);
	if (0u == (SCG0->LDOCSR & SCG_LDOCSR_LDOEN_MASK)) {
		SCG0->TRIM_LOCK = 0x5a5a0001U;
		SCG0->LDOCSR |= SCG_LDOCSR_LDOEN_MASK;
		/* wait LDO ready */
		while (0U == (SCG0->LDOCSR & SCG_LDOCSR_VOUT_OK_MASK));
	}
	SYSCON->AHBCLKCTRLSET[2] |= SYSCON_AHBCLKCTRL2_USB_HS_MASK | SYSCON_AHBCLKCTRL2_USB_HS_PHY_MASK;
	SCG0->SOSCCFG &= ~(SCG_SOSCCFG_RANGE_MASK | SCG_SOSCCFG_EREFS_MASK);
	/* xtal = 20 ~ 30MHz */
	SCG0->SOSCCFG = (1U << SCG_SOSCCFG_RANGE_SHIFT) | (1U << SCG_SOSCCFG_EREFS_SHIFT);
	SCG0->SOSCCSR |= SCG_SOSCCSR_SOSCEN_MASK;
	while (1) {
		if (SCG0->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK) {
			break;
		}
	}
	SYSCON->CLOCK_CTRL |= SYSCON_CLOCK_CTRL_CLKIN_ENA_MASK | SYSCON_CLOCK_CTRL_CLKIN_ENA_FM_USBH_LPT_MASK;
	CLOCK_EnableClock(kCLOCK_UsbHs);
	CLOCK_EnableClock(kCLOCK_UsbHsPhy);
	CLOCK_EnableUsbhsPhyPllClock(kCLOCK_Usbphy480M, BOARD_XTAL0_CLK_HZ);
	CLOCK_EnableUsbhsClock();
	USB_EhciPhyInit(kUSB_ControllerEhci0, BOARD_XTAL0_CLK_HZ, &usbPhyConfig);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(flexcan0), okay)
	/* attach FRO HF to FLEXCAN0 */
	CLOCK_SetClkDiv(kCLOCK_DivFlexcan0Clk, 1u);
	CLOCK_AttachClk(kFRO_HF_to_FLEXCAN0);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(flexio0), okay)
	CLOCK_SetClkDiv(kCLOCK_DivFlexioClk, 1u);
	CLOCK_AttachClk(kPLL0_to_FLEXIO);
#endif

	/* Set SystemCoreClock variable. */
	SystemCoreClock = CLOCK_INIT_CORE_CLOCK;

	return 0;
}

SYS_INIT(frdm_mcxn947_init, PRE_KERNEL_1, CONFIG_BOARD_INIT_PRIORITY);
