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

#ifdef CONFIG_PLATFORM_SPECIFIC_INIT

/* This function is taken from the SDK */
static void system_init(void)
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

#define FLEXCOMM_CHECK_2(n) 	\
	BUILD_ASSERT((DT_NODE_HAS_COMPAT(n, nxp_kinetis_lpuart) == 0) && 	\
		     (DT_NODE_HAS_COMPAT(n, nxp_imx_lpi2c) == 0),		\
		     "Do not enable SPI and UART/I2C on the same Flexcomm node");

/* For SPI node enabled, check if UART or I2C is also enabled on the same parent Flexcomm node */
#define FLEXCOMM_CHECK(n) DT_FOREACH_CHILD_STATUS_OKAY(DT_PARENT(n), FLEXCOMM_CHECK_2)

/**
 *
 * @brief Perform basic hardware initialization
 *
 * Initialize the interrupt controller device drivers.
 * Also initialize the timer device driver, if required.
 *
 * @return 0
 */
static int nxp_mcxn94x_init(void)
{
	/* old interrupt lock level */
	unsigned int oldLevel;

	/* disable interrupts */
	oldLevel = irq_lock();

	/* SPI cannot be exist with UART or I2C on the same FlexComm Interface
	 * Throw a build error if user is enabling SPI and UART/I2C on a Flexcomm node.
	 */
	DT_FOREACH_STATUS_OKAY(nxp_imx_lpspi, FLEXCOMM_CHECK)

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
