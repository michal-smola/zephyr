/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_flexio_lcdif

#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <fsl_edma.h>
#include "display_mcux_flexio_lcdif.h"

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
	uint32_t baudrate_bps;
	uint8_t data_bus_width;
	const struct gpio_dt_spec cs_gpio;
	const struct gpio_dt_spec rs_gpio;
	const struct gpio_dt_spec rdwr_gpio;
};

struct mcux_flexio_lcdif_data {
	struct stream dma_tx;
	struct k_sem transfer_done;
};

int flexio_lcdif_isr(const struct device *dev)
{
	/* We use DMA for transfer, no interrupt used */
	return 0;
}

void flexio_lcdif_res(const struct device *dev, struct mcux_flexio_child_res *child_res)
{
	const struct mcux_flexio_lcdif_config *config = dev->config;
	int i;

	if (child_res != NULL && config != NULL) {
		child_res->pin = (1 << config->flexio_lcd_dev->ENWRPinIndex) |
				 (1 << config->flexio_lcd_dev->RDPinIndex);
		for (i = config->flexio_lcd_dev->dataPinStartIndex;
		     i < config->flexio_lcd_dev->dataPinStartIndex + config->data_bus_width;
		     i++) {
			child_res->pin |= (1 << i);
		}

		child_res->num_pin = config->data_bus_width + 2;

		for (i = config->flexio_lcd_dev->txShifterStartIndex;
		     i <= config->flexio_lcd_dev->txShifterEndIndex;
		     i++) {
			child_res->shifter |= (1 << i);
		}

		child_res->num_shifter = config->flexio_lcd_dev->txShifterEndIndex -
				config->flexio_lcd_dev->txShifterStartIndex + 1;

		child_res->timer = (1 << config->flexio_lcd_dev->timerIndex);
		child_res->num_timer = 1;
	} else {
		LOG_ERR("%s: Error args", __func__);
	}
}

static void flexio_lcdif_dma_callback(const struct device *dev, void *arg,
			 uint32_t channel, int status)
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

int flexio_lcdif_write_memory(const struct device *dev, uint32_t command,
			const void *data, uint32_t len_byte)
{
	const struct mcux_flexio_lcdif_config *config = dev->config;
	struct mcux_flexio_lcdif_data *lcdif_data = dev->data;
	FLEXIO_MCULCD_Type *flexio_lcd = config->flexio_lcd_dev;
	struct dma_block_config *blk_cfg;
	struct stream *stream = &lcdif_data->dma_tx;
	const struct mcux_flexio_api *flexio_api = config->flexio_dev->api;
	uint8_t num_of_shifters = 0;

	num_of_shifters = (flexio_lcd->txShifterEndIndex - flexio_lcd->txShifterStartIndex + 1);

	blk_cfg = &stream->dma_blk_cfg;

	/* Assert the nCS. */
	FLEXIO_MCULCD_StartTransfer(config->flexio_lcd_dev);

	flexio_api->lock(config->flexio_dev);
	/* Send the command. */
	FLEXIO_MCULCD_WriteCommandBlocking(config->flexio_lcd_dev, command);
	flexio_api->unlock(config->flexio_dev);

	/* prepare the block for this TX DMA channel */
	memset(blk_cfg, 0, sizeof(struct dma_block_config));

	/* tx direction has memory as source and periph as dest. */
	blk_cfg->source_address = (uint32_t)data;

	/* Destination is FLEXIO Shifters */
	blk_cfg->dest_address = FLEXIO_MCULCD_GetTxDataRegisterAddress(flexio_lcd);
	blk_cfg->block_size = len_byte;
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

	flexio_api->lock(config->flexio_dev);
	FLEXIO_MCULCD_SetMultiBeatsWriteConfig(flexio_lcd);
	FLEXIO_MCULCD_EnableTxDMA(flexio_lcd, true);
	flexio_api->unlock(config->flexio_dev);

	/* Start the data transfer */
	dma_start(lcdif_data->dma_tx.dma_dev, lcdif_data->dma_tx.channel);

	/* Wait for transfer done. */
	k_sem_take(&lcdif_data->transfer_done, K_FOREVER);

	return 0;
}

int flexio_lcdif_write_command(const struct device *dev, uint32_t command)
{
	const struct mcux_flexio_lcdif_config *config = dev->config;
	FLEXIO_MCULCD_Type *flexio_lcd = config->flexio_lcd_dev;
	const struct mcux_flexio_api *flexio_api = config->flexio_dev->api;

	FLEXIO_MCULCD_StartTransfer(flexio_lcd);

	flexio_api->lock(config->flexio_dev);
	FLEXIO_MCULCD_WriteCommandBlocking(flexio_lcd, command);
	flexio_api->unlock(config->flexio_dev);

	FLEXIO_MCULCD_StopTransfer(flexio_lcd);

	return kStatus_Success;
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

int flexio_lcdif_write_data(const struct device *dev, void *data, uint32_t len_byte)
{
	const struct mcux_flexio_lcdif_config *config = dev->config;
	FLEXIO_MCULCD_Type *flexio_lcd = config->flexio_lcd_dev;
	const struct mcux_flexio_api *flexio_api = config->flexio_dev->api;

	FLEXIO_MCULCD_StartTransfer(flexio_lcd);

	flexio_api->lock(config->flexio_dev);
	flexio_lcdif_write_data_array(flexio_lcd, data, len_byte);
	flexio_api->unlock(config->flexio_dev);

	FLEXIO_MCULCD_StopTransfer(flexio_lcd);

	return kStatus_Success;
}

static int flexio_lcdif_init(const struct device *dev)
{
	const struct mcux_flexio_lcdif_config *config = dev->config;
	struct mcux_flexio_lcdif_data *lcdif_data = dev->data;
	int err;
	flexio_mculcd_config_t flexioMcuLcdConfig;
	const struct mcux_flexio_api *flexio_api = config->flexio_dev->api;
	uint32_t clock_freq;
	status_t status;

	if (!device_is_ready(config->flexio_dev)) {
		return -ENODEV;
	}

	if (!device_is_ready(lcdif_data->dma_tx.dma_dev)) {
		LOG_ERR("%s device is not ready", lcdif_data->dma_tx.dma_dev->name);
		return -ENODEV;
	}

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		return err;
	}

	err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_HIGH);
	if (err) {
		return err;
	}

	err = gpio_pin_configure_dt(&config->rs_gpio, GPIO_OUTPUT_HIGH);
	if (err) {
		return err;
	}

	/* RDWR GPIO is only used in 68K mode */
	if (kFLEXIO_MCULCD_6800 == config->flexio_lcd_dev->busType) {
		err = gpio_pin_configure_dt(&config->rdwr_gpio, GPIO_OUTPUT_HIGH);
		if (err) {
			return err;
		}
	}

	FLEXIO_MCULCD_GetDefaultConfig(&flexioMcuLcdConfig);
	flexioMcuLcdConfig.baudRate_Bps = config->baudrate_bps;
	/* Pass the FlexIO LCD device as paramter to the function
	 * callbacks for setting GPIO signals.
	 */
	config->flexio_lcd_dev->userData = (void *)dev;

	if (flexio_api->get_rate(config->flexio_dev, &clock_freq)) {
		return -EINVAL;
	}

	flexio_api->lock(config->flexio_dev);
	status = FLEXIO_MCULCD_Init(config->flexio_lcd_dev, &flexioMcuLcdConfig, clock_freq);
	flexio_api->unlock(config->flexio_dev);

	if (kStatus_Success != status) {
		return -EINVAL;
	}

	k_sem_init(&lcdif_data->transfer_done, 0, 1);

	LOG_DBG("%s device is ready", dev->name);

	return 0;
}

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
	PINCTRL_DT_INST_DEFINE(n);					\
											\
	static FLEXIO_MCULCD_Type flexio_mculcd_##n = {					\
		.flexioBase = (FLEXIO_Type *)DT_REG_ADDR(DT_INST_PARENT(n)),		\
		.busType = DT_INST_PROP(n, bus_type),					\
		.dataPinStartIndex = DT_INST_PROP(n, data_pin_start),			\
		.ENWRPinIndex = DT_INST_PROP(n, enwr_pin),				\
		.RDPinIndex = DT_INST_PROP_OR(n, rd_pin, 0),				\
		.txShifterStartIndex = DT_INST_PROP_BY_IDX(n, shifters, 0), 		\
		.txShifterEndIndex = DT_INST_PROP_BY_IDX(n, shifters, 0) +		\
				     DT_INST_PROP_LEN(n, shifters) - 1, 		\
		.rxShifterStartIndex = DT_INST_PROP_BY_IDX(n, shifters, 0), 		\
		.rxShifterEndIndex = DT_INST_PROP_BY_IDX(n, shifters, 0) + 		\
				     DT_INST_PROP_LEN(n, shifters), 			\
		.timerIndex = DT_INST_PROP_BY_IDX(n, timers, 0),			\
		.setCSPin = flexio_lcdif_set_cs,					\
		.setRSPin = flexio_lcdif_set_rs,					\
		.setRDWRPin = flexio_lcdif_set_rd_wr,					\
	};										\
											\
	struct mcux_flexio_lcdif_config mcux_flexio_lcdif_config_##n = {		\
		.flexio_lcd_dev = &flexio_mculcd_##n,					\
		.flexio_dev = DEVICE_DT_GET(DT_INST_PARENT(n)), 			\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),				\
		.baudrate_bps = DT_INST_PROP(n, baudrate_bps),				\
		MCUX_FLEXIO_DATA_BUS_WIDTH						\
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
		&flexio_lcdif_init,						\
		NULL,									\
		&mcux_flexio_lcdif_data_##n,						\
		&mcux_flexio_lcdif_config_##n,						\
		POST_KERNEL,								\
		CONFIG_MCUX_FLEXIO_CHILD_INIT_PRIORITY,					\
		NULL);

DT_INST_FOREACH_STATUS_OKAY(MCUX_FLEXIO_LCDIF_DEVICE_INIT)
