/*
 * Copyright (c) 2023, STRIM, ALC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_imx_flexio

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <fsl_flexio.h>

#include "mcux_flexio.h"
#include <spi/spi_mcux_flexio.h>
#include <display/display_mcux_flexio_lcdif.h>

/* add new interface */

LOG_MODULE_REGISTER(mcux_flexio, CONFIG_LOG_DEFAULT_LEVEL);

struct mcux_flexio_config {
	FLEXIO_Type *base;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	struct mcux_flexio_child *children;
	size_t num_children;
	void (*irq_config_func)(const struct device *dev);
	void (*irq_enable_func)(void);
	void (*irq_disable_func)(void);
};

struct mcux_flexio_data {
	struct k_mutex lock;
};

typedef int (*child_isr_t)(const struct device *dev);
typedef void (*child_res_t)(const struct device *dev, struct mcux_flexio_child_res *child_res);

struct mcux_flexio_child_config {
	child_isr_t isr;
	child_res_t res;
};

enum mcux_flexio_type {
	NXP_IMX_FLEXIO_SPI = 0,
	NXP_FLEXIO_LCDIF = 1,
	/* add new interface */
	NXP_IMX_FLEXIO_LAST_INDEX,
};

static const struct mcux_flexio_child_config children_config[NXP_IMX_FLEXIO_LAST_INDEX] = {
#if CONFIG_SPI_MCUX_FLEXIO
	[NXP_IMX_FLEXIO_SPI] = { spi_mcux_isr, spi_mcux_res },
#endif
#if CONFIG_DISPLAY_MCUX_FLEXIO_LCDIF
	[NXP_FLEXIO_LCDIF] = { flexio_lcdif_isr, flexio_lcdif_res },
#endif
	/* add new interface */
};

struct mcux_flexio_child {
	const struct device *dev;
	const enum mcux_flexio_type compatible;
};

static void mcux_flexio_isr(const struct device *dev)
{
	const struct mcux_flexio_config *config = dev->config;
	struct mcux_flexio_child *child;
	const struct device *isr_dev;
	child_isr_t isr_func;

	for (size_t i = 0; i < config->num_children; i++) {
		child = &config->children[i];
		isr_func = children_config[child->compatible].isr;
		isr_dev = child->dev;

		if (isr_func != NULL && isr_dev != NULL) {
			int ret = isr_func(isr_dev);

			if (ret != 0) {
				LOG_ERR("isr return: %d", ret);
			}
		} else {
			LOG_ERR("isr_func or isr_dev equal NULL");
		}
	}
	SDK_ISR_EXIT_BARRIER;
}

static uint32_t count_ones(uint32_t number)
{
	uint32_t res = 0;

	while (number) {
		res++;
		number &= number - 1;
	}
	return res;
}

static int mcux_flexio_check_res_overlap(const struct device *dev)
{
	const struct mcux_flexio_config *config = dev->config;
	struct mcux_flexio_child *child;
	child_res_t res_func;

	struct mcux_flexio_child_res res = {0};

	for (size_t i = 0; i < config->num_children; i++) {
		child = &config->children[i];
		res_func = children_config[child->compatible].res;

		if (res_func != NULL) {
			struct mcux_flexio_child_res child_res;

			res_func(child->dev, &child_res);

			res.pin ^= child_res.pin;
			res.num_pin += child_res.num_pin;
			res.shifter ^= child_res.shifter;
			res.num_shifter += child_res.num_shifter;
			res.timer ^= child_res.timer;
			res.num_timer += child_res.num_timer;
		} else {
			LOG_ERR("%s: res_func not found", __func__);
			return -1;
		}
	}

	if (count_ones(res.pin) != res.num_pin) {
		LOG_ERR("Pins overlapping in %s", dev->name);
		return -2;
	}

	if (count_ones(res.shifter) != res.num_shifter) {
		LOG_ERR("Shifters overlapping in %s", dev->name);
		return -3;
	}

	if (count_ones(res.timer) != res.num_timer) {
		LOG_ERR("Timers overlapping in %s", dev->name);
		return -4;
	}

	return 0;
}

static int mcux_flexio_init(const struct device *dev)
{
	const struct mcux_flexio_config *config = dev->config;
	struct mcux_flexio_data *data = dev->data;
	flexio_config_t flexio_config;

	int err = mcux_flexio_check_res_overlap(dev);

	if (err < 0) {
		return err;
	}

	k_mutex_init(&data->lock);

	FLEXIO_GetDefaultConfig(&flexio_config);
	FLEXIO_Init(config->base, &flexio_config);
	config->irq_config_func(dev);

	return 0;
}

static void mcux_flexio_irq_enable(const struct device *flexio_dev)
{
	const struct mcux_flexio_config *config = flexio_dev->config;

	config->irq_enable_func();
}

static void mcux_flexio_irq_disable(const struct device *flexio_dev)
{
	const struct mcux_flexio_config *config = flexio_dev->config;

	config->irq_disable_func();
}

static void mcux_flexio_lock(const struct device *flexio_dev)
{
	struct mcux_flexio_data *data = flexio_dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
}

static void mcux_flexio_unlock(const struct device *flexio_dev)
{
	struct mcux_flexio_data *data = flexio_dev->data;

	k_mutex_unlock(&data->lock);
}

static int mcux_flexio_get_rate(const struct device *flexio_dev, uint32_t *rate)
{
	const struct mcux_flexio_config *config = flexio_dev->config;

	return clock_control_get_rate(config->clock_dev, config->clock_subsys, rate);
}

static const struct mcux_flexio_api mcux_flexio_driver_api = {
	.interrupt_enable = mcux_flexio_irq_enable,
	.interrupt_disable = mcux_flexio_irq_disable,
	.lock = mcux_flexio_lock,
	.unlock = mcux_flexio_unlock,
	.get_rate = mcux_flexio_get_rate,
};

BUILD_ASSERT(CONFIG_MCUX_FLEXIO_CHILD_INIT_PRIORITY > CONFIG_MCUX_FLEXIO_INIT_PRIORITY,
		"MCUX_FLEXIO_CHILD_INIT_PRIORITY must be greater than MCUX_FLEXIO_INIT_PRIORITY");

#define MCUX_FLEXIO_CHILD_INIT(child_node_id)				\
	{								\
		.dev = DEVICE_DT_GET(child_node_id),			\
		.compatible = DT_STRING_UPPER_TOKEN_BY_IDX(child_node_id, compatible, 0), \
	},

#define MCUX_FLEXIO_INIT(n)						\
	static void mcux_flexio_irq_config_func_##n(const struct device *dev); \
	static void mcux_flexio_irq_enable_func_##n(void);		\
	static void mcux_flexio_irq_disable_func_##n(void);		\
									\
	static struct mcux_flexio_child mcux_flexio_children_##n[] = {	\
		DT_INST_FOREACH_CHILD_STATUS_OKAY(n, MCUX_FLEXIO_CHILD_INIT) \
	};								\
									\
	static struct mcux_flexio_data mcux_flexio_data_##n;		\
									\
	static const struct mcux_flexio_config mcux_flexio_config_##n = { \
		.base = (FLEXIO_Type *)DT_INST_REG_ADDR(n),		\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),	\
		.clock_subsys =						\
		(clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),	\
		.children = mcux_flexio_children_##n,			\
		.num_children = ARRAY_SIZE(mcux_flexio_children_##n),	\
		.irq_config_func = mcux_flexio_irq_config_func_##n,	\
		.irq_enable_func = mcux_flexio_irq_enable_func_##n,	\
		.irq_disable_func = mcux_flexio_irq_disable_func_##n,	\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, &mcux_flexio_init,			\
				NULL,					\
				&mcux_flexio_data_##n,			\
				&mcux_flexio_config_##n,		\
				POST_KERNEL,				\
				CONFIG_MCUX_FLEXIO_INIT_PRIORITY,	\
				&mcux_flexio_driver_api);		\
									\
	static void mcux_flexio_irq_config_func_##n(const struct device *dev) \
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),	\
			mcux_flexio_isr, DEVICE_DT_INST_GET(n), 0);	\
		irq_enable(DT_INST_IRQN(n));				\
	}								\
									\
	static void mcux_flexio_irq_enable_func_##n(void)		\
	{								\
		irq_enable(DT_INST_IRQN(n));				\
	}								\
									\
	static void mcux_flexio_irq_disable_func_##n(void)		\
	{								\
		irq_disable(DT_INST_IRQN(n));				\
	}

DT_INST_FOREACH_STATUS_OKAY(MCUX_FLEXIO_INIT)
