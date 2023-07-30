/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_DRIVERS_MCUX_LP_FLEXCOMM_H_
#define ZEPHYR_DRIVERS_MCUX_LP_FLEXCOMM_H_

#include "fsl_lpflexcomm.h"

typedef void (*child_isr_t)(const struct device *dev);

void mcux_lpflexcomm_setirqhandler(const struct device *dev, const struct device *child_dev,
				   LP_FLEXCOMM_PERIPH_T periph, child_isr_t handler);

#endif /* ZEPHYR_DRIVERS_MCUX_LP_FLEXCOMM_H_ */
