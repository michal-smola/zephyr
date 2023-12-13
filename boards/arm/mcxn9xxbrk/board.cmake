#
# Copyright 2023 NXP
#
# SPDX-License-Identifier: Apache-2.0
#

if(CONFIG_SOC_MCX_N947_CPU0)  ## OR CONFIG_SECOND_CORE_MCUX?
board_runner_args(jlink "--device=MCXN947_M33_0" "--reset-after-load")
board_runner_args(linkserver  "--device=MCXN947:MCX-N9XX-BRK:cm33_core0")
board_runner_args(linkserver  "--override=/device/memory/1/flash-driver=MCXN9xx_S.cfx")
board_runner_args(linkserver  "--override=/device/memory/1/location=0x10000000")
else()
message(FATAL_ERROR "Support for cpu1 not available yet")
endif()

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/linkserver.board.cmake)
