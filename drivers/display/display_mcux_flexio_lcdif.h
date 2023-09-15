/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_DISPLAY_MCUX_FLEXIO_H_
#define ZEPHYR_DRIVERS_DISPLAY_MCUX_FLEXIO_H_

#include <misc/mcux_flexio/mcux_flexio.h>
#include <fsl_flexio_mculcd.h>

int flexio_lcdif_isr(const struct device *dev);
void flexio_lcdif_res(const struct device *dev, struct mcux_flexio_child_res *child_res);

/* API for the display driver to use */
int flexio_lcdif_write_memory(const struct device *dev, uint32_t command,
			const void *data, uint32_t len_byte);
int flexio_lcdif_write_command(const struct device *dev, uint32_t command);
int flexio_lcdif_write_data(const struct device *dev, void *data, uint32_t len_byte);

#endif /* ZEPHYR_DRIVERS_DISPLAY_MCUX_FLEXIO_H_ */
