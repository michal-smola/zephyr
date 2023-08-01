#
# Copyright 2023 NXP
#
# SPDX-License-Identifier: Apache-2.0
#

# Pass this flag so the SDK I2C, UART and SPI drivers do not init the LP
# Flexcomm SDK driver
zephyr_compile_definitions_ifdef(CONFIG_MCUX_LP_FLEXCOMM LPFLEXCOMM_INIT_NOT_USED_IN_DRIVER=1)

zephyr_sources(
  soc.c
  )
