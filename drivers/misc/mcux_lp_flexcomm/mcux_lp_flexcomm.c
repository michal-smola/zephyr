/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lp_flexcomm

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "mcux_lp_flexcomm.h"

LOG_MODULE_REGISTER(nxp_lp_flexcomm, LOG_LEVEL_ERR);

struct mcux_lpflexcomm_child {
	const struct device *dev;
	uint8_t periph;
	child_isr_t lp_flexcomm_child_isr;
};

struct mcux_lpflexcomm_data {
	struct mcux_lpflexcomm_child *children;
	size_t num_children;
};

struct mcux_lpflexcomm_config {
	LP_FLEXCOMM_Type *base;
	void (*irq_config_func)(const struct device *dev);
};

void mcux_lpflexcomm_isr(const struct device *dev)
{
	uint32_t interrupt_status;
	const struct mcux_lpflexcomm_config *config = dev->config;
	struct mcux_lpflexcomm_data *data = dev->data;
	uint32_t instance = LP_FLEXCOMM_GetInstance(config->base);
	struct mcux_lpflexcomm_child *child;

	interrupt_status = LP_FLEXCOMM_GetInterruptStatus(instance);
	if ((interrupt_status &
	    ((uint32_t)kLPFLEXCOMM_I2cSlaveInterruptFlag |
	     (uint32_t)kLPFLEXCOMM_I2cMasterInterruptFlag)) != 0U) {
		child = &data->children[LP_FLEXCOMM_PERIPH_LPI2C];

		if (child->lp_flexcomm_child_isr != NULL) {
			child->lp_flexcomm_child_isr(child->dev);
		}
	}
	if ((interrupt_status &
	     ((uint32_t)kLPFLEXCOMM_UartRxInterruptFlag |
	      (uint32_t)kLPFLEXCOMM_UartTxInterruptFlag)) != 0U) {
		child = &data->children[LP_FLEXCOMM_PERIPH_LPUART];

		if (child->lp_flexcomm_child_isr != NULL) {
			child->lp_flexcomm_child_isr(child->dev);
		}
	}
	if (((interrupt_status &
	     (uint32_t)kLPFLEXCOMM_SpiInterruptFlag)) != 0U) {
		child = &data->children[LP_FLEXCOMM_PERIPH_LPSPI];

		if (child->lp_flexcomm_child_isr != NULL) {
			child->lp_flexcomm_child_isr(child->dev);
		}
	}
}

void mcux_lpflexcomm_setirqhandler(const struct device *dev, const struct device *child_dev,
				   LP_FLEXCOMM_PERIPH_T periph, child_isr_t handler)
{
	struct mcux_lpflexcomm_data *data = dev->data;
	struct mcux_lpflexcomm_child *child;

	child = &data->children[periph];

	/* Store the interrupt handler and the child device node */
	child->lp_flexcomm_child_isr = handler;
	child->dev = child_dev;
}

static int mcux_lpflexcomm_init(const struct device *dev)
{
	const struct mcux_lpflexcomm_config *config = dev->config;
	struct mcux_lpflexcomm_data *data = dev->data;
	uint32_t instance;
	struct mcux_lpflexcomm_child *child = NULL;
	bool spi_found = false;
	bool uart_found = false;
	bool i2c_found = false;

	for (int i = 1; i < data->num_children; i++) {
		child = &data->children[i];
		if (child->periph == LP_FLEXCOMM_PERIPH_LPSPI) {
			spi_found = true;
		}
		if (child->periph == LP_FLEXCOMM_PERIPH_LPI2C) {
			i2c_found = true;
		}
		if (child->periph == LP_FLEXCOMM_PERIPH_LPUART) {
			uart_found = true;
		}
	}

	/* If SPI is enabled with another interface type return an error */
	if (spi_found && (i2c_found || uart_found)) {
		return -EINVAL;
	}

	instance = LP_FLEXCOMM_GetInstance(config->base);

	if (uart_found && i2c_found) {
		LP_FLEXCOMM_Init(instance, LP_FLEXCOMM_PERIPH_LPI2CAndLPUART);
	} else if (uart_found) {
		LP_FLEXCOMM_Init(instance, LP_FLEXCOMM_PERIPH_LPUART);
	} else if (i2c_found) {
		LP_FLEXCOMM_Init(instance, LP_FLEXCOMM_PERIPH_LPI2C);
	} else if (spi_found) {
		LP_FLEXCOMM_Init(instance, LP_FLEXCOMM_PERIPH_LPSPI);
	}

	config->irq_config_func(dev);

	return 0;
}

#define MCUX_FLEXCOMM_CHILD_INIT(child_node_id)					\
	[DT_NODE_CHILD_IDX(child_node_id) + 1] = {				\
		.periph = DT_NODE_CHILD_IDX(child_node_id) + 1,			\
	},

#define MCUX_LP_FLEXCOMM_INIT(n)						\
										\
	static struct mcux_lpflexcomm_child 					\
		mcux_lpflexcomm_children_##n[LP_FLEXCOMM_PERIPH_LPI2C + 1] = {	\
		DT_INST_FOREACH_CHILD_STATUS_OKAY(n, MCUX_FLEXCOMM_CHILD_INIT)	\
	};									\
										\
	static void mcux_lpflexcomm_config_func_##n(const struct device *dev);	\
										\
	static const struct mcux_lpflexcomm_config mcux_lpflexcomm_config_##n = { \
		.base = (LP_FLEXCOMM_Type *)DT_INST_REG_ADDR(n),		\
		.irq_config_func = mcux_lpflexcomm_config_func_##n,		\
	};									\
										\
	static struct mcux_lpflexcomm_data mcux_lpflexcomm_data_##n = {		\
		.children = mcux_lpflexcomm_children_##n,			\
		.num_children = ARRAY_SIZE(mcux_lpflexcomm_children_##n),	\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n,						\
			    &mcux_lpflexcomm_init,				\
			    NULL,						\
			    &mcux_lpflexcomm_data_##n,				\
			    &mcux_lpflexcomm_config_##n,			\
			    PRE_KERNEL_1,					\
			    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,		\
			    NULL);						\
										\
	static void mcux_lpflexcomm_config_func_##n(const struct device *dev) 	\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),		\
			    mcux_lpflexcomm_isr, DEVICE_DT_INST_GET(n), 0);	\
		irq_enable(DT_INST_IRQN(n));					\
	}

DT_INST_FOREACH_STATUS_OKAY(MCUX_LP_FLEXCOMM_INIT)
