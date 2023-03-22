/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System/hardware module for nxp_mcxn94x platform
 *
 * This module provides routines to initialize and support board-level
 * hardware for the nxp_mcxn94x platform.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <soc.h>
#include <fsl_clock.h>

/* Core clock frequency: 150MHz */
#define CLOCK_INIT_CORE_CLOCK                     150000000U

/* System clock frequency. */
extern uint32_t SystemCoreClock;

#ifdef CONFIG_PLATFORM_SPECIFIC_INIT
/* This function is taken from the SDK */
void system_init(void)
{
#if ((__FPU_PRESENT == 1) && (__FPU_USED == 1))
	/* set CP10, CP11 Full Access in Secure mode */
	SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));
#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
	/* set CP10, CP11 Full Access in Non-secure mode */
	SCB_NS->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));
#endif /* (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U) */
#endif /* ((__FPU_PRESENT == 1) && (__FPU_USED == 1)) */

	/* set CP0, CP1 Full Access in Secure mode (enable PowerQuad) */
	SCB->CPACR |= ((3UL << 0 * 2) | (3UL << 1 * 2));
#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
	/* set CP0, CP1 Full Access in Normal mode (enable PowerQuad) */
	SCB_NS->CPACR |= ((3UL << 0 * 2) | (3UL << 1 * 2));
#endif /* (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U) */

	/* enable CP0, CP1, CP10, CP11 Non-secure Access */
	SCB->NSACR |= ((3UL << 0) | (3UL << 10));

	/* disable RAM ECC to get max RAM size */
	SYSCON->ECC_ENABLE_CTRL = 0;
}

void z_arm_platform_init(void)
{
	system_init();
}
#endif

/**
 * @brief Initialize the system clock
 */
static ALWAYS_INLINE void clock_init(void)
{
	/* Enable SCG clock */
	CLOCK_EnableClock(kCLOCK_Scg);

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
#endif
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(adc0), okay)
	/* attach FRO HF to ADC0 */
	CLOCK_SetClkDiv(kCLOCK_DivAdc0Clk, 1u);
	CLOCK_AttachClk(kFRO_HF_to_ADC0);
#endif

	/* Set SystemCoreClock variable. */
	SystemCoreClock = CLOCK_INIT_CORE_CLOCK;
}

/**
 *
 * @brief Perform basic hardware initialization
 *
 * Initialize the interrupt controller device drivers.
 * Also initialize the timer device driver, if required.
 *
 * @return 0
 */
static int nxp_mcxn94x_init(const struct device *arg)
{
	ARG_UNUSED(arg);

	/* old interrupt lock level */
	unsigned int oldLevel;

	/* disable interrupts */
	oldLevel = irq_lock();

	/* Initialize clock */
	clock_init();

	/*
	 * install default handler that simply resets the CPU if configured in
	 * the kernel, NOP otherwise
	 */
	NMI_INIT();

	/* restore interrupt state */
	irq_unlock(oldLevel);

	return 0;
}

SYS_INIT(nxp_mcxn94x_init, PRE_KERNEL_1, 0);
