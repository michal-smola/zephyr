/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC__H_
#define _SOC__H_

#ifndef _ASMLANGUAGE

#include <fsl_port.h>
#include <fsl_common.h>

#define PORT_MUX_GPIO kPORT_MuxAlt0 /* GPIO setting for the Port Mux Register */

#ifdef __cplusplus
extern "C" {
#endif

uint32_t flexspi_clock_set_freq(uint32_t clock_name, uint32_t rate);

#ifdef __cplusplus
}
#endif

#endif /* !_ASMLANGUAGE */

#endif /* _SOC__H_ */
