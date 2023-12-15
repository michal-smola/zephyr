#
# Copyright 2023 NXP
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_library()
zephyr_library_sources(board.c)
zephyr_include_directories(.)

zephyr_library_include_directories_ifdef(
  CONFIG_FLASH_MCUX_FLEXSPI_NOR
  ${ZEPHYR_BASE}/drivers/memc
  ${ZEPHYR_BASE}/drivers/flash
)

# Data bus width is used by the SDK driver and processes it as a compile time option
if(CONFIG_DISPLAY_MCUX_FLEXIO_LCDIF)
  dt_nodelabel(flexio0_lcd NODELABEL "ssd1963_display")
  dt_prop(data_bus_width PATH "${flexio0_lcd}" PROPERTY "data-bus-width")
  zephyr_compile_definitions(FLEXIO_MCULCD_DATA_BUS_WIDTH=${data_bus_width})
endif()
