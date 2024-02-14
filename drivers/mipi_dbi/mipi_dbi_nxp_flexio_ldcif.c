/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_mipi_dbi_flexio_lcdif

#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/mipi_dbi.h>
#include <fsl_edma.h>
#include "../misc/mcux_flexio/mcux_flexio.h"
#include <fsl_flexio_mculcd.h>

LOG_MODULE_REGISTER(display_mcux_flexio_lcdif, CONFIG_DISPLAY_LOG_LEVEL);

struct stream {
	const struct device *dma_dev;
	uint32_t channel; /* stores the channel for dma */
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk_cfg;
};

struct mcux_flexio_lcdif_config {
	FLEXIO_MCULCD_Type *flexio_lcd_dev;
	const struct device *flexio_dev;
	const struct pinctrl_dev_config *pincfg;
	const struct mcux_flexio_child *child;
	uint32_t baudrate_bps;
	uint8_t data_bus_width;
	/* Reset GPIO */
	const struct gpio_dt_spec reset;
	const struct gpio_dt_spec cs_gpio;
	const struct gpio_dt_spec rs_gpio;
	const struct gpio_dt_spec rdwr_gpio;
};

struct mcux_flexio_lcdif_data {
	struct stream dma_tx;
	struct k_sem transfer_done;
};

static int flexio_lcdif_isr(const struct device *dev)
{
	/* We use DMA for transfer, no interrupt used */
	return 0;
}

static void flexio_lcdif_dma_callback(const struct device *dev, void *arg,
				      int32_t channel, int status)
{
	const struct device *flexio_dev = (struct device *)arg;
	struct mcux_flexio_lcdif_data *lcdif_data = flexio_dev->data;
	const struct mcux_flexio_lcdif_config *config = flexio_dev->config;
	FLEXIO_MCULCD_Type *flexio_lcd = config->flexio_lcd_dev;

	FLEXIO_MCULCD_EnableTxDMA(flexio_lcd, false);

	/* Now the data are in shifter, wait for the data send out from the shifter. */
	FLEXIO_MCULCD_WaitTransmitComplete();

	/* Disable the TX shifter and the timer. */
	FLEXIO_MCULCD_ClearMultiBeatsWriteConfig(flexio_lcd);

	/* De-assert nCS. */
	FLEXIO_MCULCD_StopTransfer(flexio_lcd);

	k_sem_give(&lcdif_data->transfer_done);
}


static void flexio_lcdif_set_cs(bool set, void *param)
{
	const struct device *flexio_dev = (struct device *)param;
	const struct mcux_flexio_lcdif_config *config = flexio_dev->config;

	gpio_pin_set_dt(&config->cs_gpio, (int)set);
}

static void flexio_lcdif_set_rs(bool set, void *param)
{
	const struct device *flexio_dev = (struct device *)param;
	const struct mcux_flexio_lcdif_config *config = flexio_dev->config;

	gpio_pin_set_dt(&config->rs_gpio, (int)set);
}

static void flexio_lcdif_set_rd_wr(bool set, void *param)
{
	const struct device *flexio_dev = (struct device *)param;
	const struct mcux_flexio_lcdif_config *config = flexio_dev->config;

	gpio_pin_set_dt(&config->rdwr_gpio, (int)set);
}

static edma_modulo_t flexio_lcdif_get_edma_modulo(uint8_t shifterNum)
{
	edma_modulo_t ret = kEDMA_ModuloDisable;

	switch (shifterNum) {
	case 1U:
		ret = kEDMA_Modulo4bytes;
		break;
	case 2U:
		ret = kEDMA_Modulo8bytes;
		break;
	case 4U:
		ret = kEDMA_Modulo16bytes;
		break;
	case 8U:
		ret = kEDMA_Modulo32bytes;
		break;
	default:
		ret = kEDMA_ModuloDisable;
		break;
	}

	return ret;
}

static void flexio_lcdif_write_data_array(FLEXIO_MCULCD_Type *base,
					  const void *data,
					  size_t size)
{
	assert(size > 0U);

	uint32_t i;
	const uint8_t *data8Bit;
	FLEXIO_Type *flexioBase = base->flexioBase;

	/* Assert the RS pin. */
	base->setRSPin(true, base->userData);
	/* For 6800, de-assert the RDWR pin. */
	if (kFLEXIO_MCULCD_6800 == base->busType) {
		base->setRDWRPin(false, base->userData);
	}

	/* Configure the timer and TX shifter. */
	FLEXIO_MCULCD_SetSingleBeatWriteConfig(base);

	data8Bit = (const uint8_t *)data;

	for (i = 0; i < size; i++) {
		flexioBase->SHIFTBUF[base->txShifterStartIndex] = data8Bit[i];

		/* Wait for the data send out. */
		while (0U == ((1UL << base->timerIndex) & flexioBase->TIMSTAT)) {
        }

		/* Clear the timer stat. */
		flexioBase->TIMSTAT = 1UL << base->timerIndex;
	}

	/* Stop the timer and TX shifter. */
	FLEXIO_MCULCD_ClearSingleBeatWriteConfig(base);
}

static int mipi_dbi_flexio_ldcif_write_display(const struct device *dev,
					       const struct mipi_dbi_config *dbi_config,
					       const uint8_t *framebuf,
					       struct display_buffer_descriptor *desc,
					       enum display_pixel_format pixfmt)
{
	const struct mcux_flexio_lcdif_config *config = dev->config;
	struct mcux_flexio_lcdif_data *lcdif_data = dev->data;
	FLEXIO_MCULCD_Type *flexio_lcd = config->flexio_lcd_dev;
	struct dma_block_config *blk_cfg;
	struct stream *stream = &lcdif_data->dma_tx;
	uint8_t num_of_shifters = 0;

	ARG_UNUSED(pixfmt);
	ARG_UNUSED(dbi_config);

	num_of_shifters = (flexio_lcd->txShifterEndIndex - flexio_lcd->txShifterStartIndex + 1);

	blk_cfg = &stream->dma_blk_cfg;

	/* Assert the nCS. */
	FLEXIO_MCULCD_StartTransfer(config->flexio_lcd_dev);

	/* prepare the block for this TX DMA channel */
	memset(blk_cfg, 0, sizeof(struct dma_block_config));

	/* tx direction has memory as source and periph as dest. */
	blk_cfg->source_address = (uint32_t)framebuf;

	/* Destination is FLEXIO Shifters */
	blk_cfg->dest_address = FLEXIO_MCULCD_GetTxDataRegisterAddress(flexio_lcd);
	blk_cfg->block_size = desc->buf_size;
	/* Transfer in each DMA loop is based on the number of shifters used */
	stream->dma_cfg.source_burst_length = num_of_shifters * 4;

	stream->dma_cfg.head_block = &stream->dma_blk_cfg;
	/* Give the client dev as arg, as the callback comes from the dma */
	stream->dma_cfg.user_data = (struct device *)dev;

	/* Configure the DMA */
	dma_config(lcdif_data->dma_tx.dma_dev, lcdif_data->dma_tx.channel, &stream->dma_cfg);

	/* The DMA driver does not support setting this Modulo value which is required
	 * in case of the flexio module to form a circular chain between the Shift buffer
	 * in the FLEXIO module.
	 * TODO: Look to move this into the Zephyr EDMA HAL
	 */
	EDMA_SetModulo(DMA0, lcdif_data->dma_tx.channel, kEDMA_ModuloDisable,
			flexio_lcdif_get_edma_modulo(num_of_shifters));

	/* For 6800, de-assert the RDWR pin. */
	if (kFLEXIO_MCULCD_6800 == flexio_lcd->busType) {
		flexio_lcdif_set_rd_wr(false, (void *)dev);
	}

	mcux_flexio_lock(config->flexio_dev);
	FLEXIO_MCULCD_SetMultiBeatsWriteConfig(flexio_lcd);
	FLEXIO_MCULCD_EnableTxDMA(flexio_lcd, true);
	mcux_flexio_unlock(config->flexio_dev);

	/* Start the data transfer */
	dma_start(lcdif_data->dma_tx.dma_dev, lcdif_data->dma_tx.channel);

	/* Wait for transfer done. */
	k_sem_take(&lcdif_data->transfer_done, K_FOREVER);

	return 0;
}

static int mipi_dbi_flexio_lcdif_command_write(const struct device *dev,
					       const struct mipi_dbi_config *dbi_config,
					       uint8_t cmd, const uint8_t *data_buf,
					       size_t len)
{
	const struct mcux_flexio_lcdif_config *config = dev->config;
	FLEXIO_MCULCD_Type *flexio_lcd = config->flexio_lcd_dev;

	ARG_UNUSED(dbi_config);

	FLEXIO_MCULCD_StartTransfer(flexio_lcd);

	mcux_flexio_lock(config->flexio_dev);

	FLEXIO_MCULCD_WriteCommandBlocking(flexio_lcd, cmd);

	if ((data_buf != NULL) && (len != 0)) {
		flexio_lcdif_write_data_array(flexio_lcd, data_buf, len);
	}

	mcux_flexio_unlock(config->flexio_dev);

	FLEXIO_MCULCD_StopTransfer(flexio_lcd);

	return kStatus_Success;

}

static int mipi_dbi_flexio_lcdif_reset(const struct device *dev,
				       const struct mipi_dbi_config *dbi_config,
				       uint32_t delay)
{
	const struct mcux_flexio_lcdif_config *config = dev->config;
	flexio_mculcd_config_t flexioMcuLcdConfig;
	int err;
	uint32_t clock_freq;
	status_t status;

	config->flexio_lcd_dev->busType = dbi_config->mode;

	err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_HIGH);
	if (err) {
		return err;
	}

	err = gpio_pin_configure_dt(&config->rs_gpio, GPIO_OUTPUT_HIGH);
	if (err) {
		return err;
	}

	/* RDWR GPIO is only used in 68K mode */
	if (MIPI_DBI_MODE_6800_BUS == config->flexio_lcd_dev->busType) {
		err = gpio_pin_configure_dt(&config->rdwr_gpio, GPIO_OUTPUT_HIGH);
		if (err) {
			return err;
		}
	}

	FLEXIO_MCULCD_GetDefaultConfig(&flexioMcuLcdConfig);
	flexioMcuLcdConfig.baudRate_Bps = config->baudrate_bps;

	if (mcux_flexio_get_rate(config->flexio_dev, &clock_freq)) {
		return -EINVAL;
	}

	mcux_flexio_lock(config->flexio_dev);
	/* Resets the FlexIO module, then configures FlexIO MCULCD */
	status = FLEXIO_MCULCD_Init(config->flexio_lcd_dev, &flexioMcuLcdConfig, clock_freq);
	mcux_flexio_unlock(config->flexio_dev);

	if (kStatus_Success != status) {
		return -EINVAL;
	}

	/* Checkf if a reset port is provided to reset the LCD controller */
	if (config->reset.port == NULL) {
		return 0;
	}

	/* Reset the LCD controller. */
	err = gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_HIGH);
	if (err) {
		return err;
	}

	err = gpio_pin_set_dt(&config->reset, 0);
	if (err < 0) {
		return err;
	}

	k_usleep(delay);

	err = gpio_pin_set_dt(&config->reset, 1);
	if (err < 0) {
		return err;
	}

	LOG_DBG("%s device reset complete", dev->name);

	return 0;
}

static int flexio_lcdif_init(const struct device *dev)
{
	const struct mcux_flexio_lcdif_config *config = dev->config;
	struct mcux_flexio_lcdif_data *lcdif_data = dev->data;
	int err;
	uint8_t shifter_end = config->child->res.shifter_count - 1;

	if (!device_is_ready(lcdif_data->dma_tx.dma_dev)) {
		LOG_ERR("%s device is not ready", lcdif_data->dma_tx.dma_dev->name);
		return -ENODEV;
	}

	err = mcux_flexio_child_attach(config->flexio_dev, config->child);
	if (err < 0) {
		return err;
	}

	config->flexio_lcd_dev->txShifterStartIndex = config->child->res.shifter_index[0];
	config->flexio_lcd_dev->txShifterEndIndex = config->child->res.shifter_index[shifter_end];

	config->flexio_lcd_dev->rxShifterStartIndex = config->flexio_lcd_dev->txShifterStartIndex;
	config->flexio_lcd_dev->rxShifterEndIndex = config->flexio_lcd_dev->txShifterEndIndex;

	config->flexio_lcd_dev->timerIndex = config->child->res.timer_index[0];

	if (config->flexio_lcd_dev->txShifterEndIndex !=
	    config->flexio_lcd_dev->txShifterStartIndex + shifter_end) {
		LOG_ERR("Shifters should be continuous");
		return -ENODEV;
	}
	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		return err;
	}

	/* Pass the FlexIO LCD device as paramter to the function
	 * callbacks for setting GPIO signals.
	 */
	config->flexio_lcd_dev->userData = (void *)dev;

	k_sem_init(&lcdif_data->transfer_done, 0, 1);

	LOG_DBG("%s device init complete", dev->name);

	return 0;
}

static struct mipi_dbi_driver_api mipi_dbi_lcdif_driver_api = {
	.reset = mipi_dbi_flexio_lcdif_reset,
	.command_write = mipi_dbi_flexio_lcdif_command_write,
	.write_display = mipi_dbi_flexio_ldcif_write_display,
};

/* The width is read from device tree by the CMake file and passed in as a
 * compile time define
 */
#if (8UL == FLEXIO_MCULCD_DATA_BUS_WIDTH)
#define MCUX_FLEXIO_SOURCE_DATA_SIZE	.source_data_size = 1,
#define MCUX_FLEXIO_DATA_BUS_WIDTH	.data_bus_width = 8,
#else
#define MCUX_FLEXIO_SOURCE_DATA_SIZE	.source_data_size = 2,
#define MCUX_FLEXIO_DATA_BUS_WIDTH	.data_bus_width = 16,
#endif

#define MCUX_FLEXIO_LCDIF_DEVICE_INIT(n)						\
											\
	PINCTRL_DT_INST_DEFINE(n);							\
											\
	static FLEXIO_MCULCD_Type flexio_mculcd_##n = {					\
		.flexioBase = (FLEXIO_Type *)DT_REG_ADDR(DT_INST_PARENT(n)),		\
		.dataPinStartIndex = DT_INST_PROP(n, data_pin_start),			\
		.ENWRPinIndex = DT_INST_PROP(n, enwr_pin),				\
		.RDPinIndex = DT_INST_PROP_OR(n, rd_pin, 0),				\
		.setCSPin = flexio_lcdif_set_cs,					\
		.setRSPin = flexio_lcdif_set_rs,					\
		.setRDWRPin = flexio_lcdif_set_rd_wr,					\
	};										\
											\
	static uint8_t mcux_flexio_lcdif_shifters_##n[DT_INST_PROP(n, shifters_count)];	\
	static uint8_t mcux_flexio_lcdif_timers_##n[DT_INST_PROP(n, timers_count)];	\
											\
	static const struct mcux_flexio_child lcdif_child_##n = {			\
		.isr = flexio_lcdif_isr,						\
		.user_data = (void *)DEVICE_DT_INST_GET(n),				\
		.res = {								\
			.shifter_index = mcux_flexio_lcdif_shifters_##n,		\
			.shifter_count = ARRAY_SIZE(mcux_flexio_lcdif_shifters_##n),	\
			.timer_index = mcux_flexio_lcdif_timers_##n,			\
			.timer_count = ARRAY_SIZE(mcux_flexio_lcdif_timers_##n),	\
		}									\
	};										\
											\
	struct mcux_flexio_lcdif_config mcux_flexio_lcdif_config_##n = {		\
		.flexio_lcd_dev = &flexio_mculcd_##n,					\
		.flexio_dev = DEVICE_DT_GET(DT_INST_PARENT(n)), 			\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),				\
		.child = &mcux_flexio_ldcif_child_##n,					\
		.baudrate_bps = DT_INST_PROP(n, baudrate_bps),				\
		MCUX_FLEXIO_DATA_BUS_WIDTH						\
		.reset = GPIO_DT_SPEC_INST_GET(n, reset_gpios),				\
		.cs_gpio = GPIO_DT_SPEC_INST_GET(n, cs_gpios),				\
		.rs_gpio = GPIO_DT_SPEC_INST_GET(n, rs_gpios),				\
		.rdwr_gpio = GPIO_DT_SPEC_INST_GET_OR(n, rdwr_gpios, {0}),		\
	};										\
	struct mcux_flexio_lcdif_data mcux_flexio_lcdif_data_##n = {			\
		.dma_tx = {								\
			.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, tx)),	\
			.channel = DT_INST_DMAS_CELL_BY_NAME(n, tx, mux),		\
			.dma_cfg = {							\
				.channel_direction = MEMORY_TO_MEMORY,			\
				.dma_callback = flexio_lcdif_dma_callback,		\
				MCUX_FLEXIO_SOURCE_DATA_SIZE				\
				.dest_data_size = 4,					\
				.block_count = 1,					\
				.dma_slot = DT_INST_DMAS_CELL_BY_NAME(n, tx, source)	\
			}								\
		},									\
	};										\
	DEVICE_DT_INST_DEFINE(n,							\
		&flexio_lcdif_init,							\
		NULL,									\
		&mcux_flexio_lcdif_data_##n,						\
		&mcux_flexio_lcdif_config_##n,						\
		POST_KERNEL,								\
		CONFIG_MCUX_FLEXIO_CHILD_INIT_PRIORITY,					\
		&mipi_dbi_lcdif_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MCUX_FLEXIO_LCDIF_DEVICE_INIT)
