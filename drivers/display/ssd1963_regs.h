/*
 * Copyright 2023 NXP Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SSD1963_REGS_H__
#define __SSD1963_REGS_H__

/* The PLL VCO clock must be in the range of (250MHz, 800MHz). */
#define SSD1963_VCO_MIN_HZ    250000000U
#define SSD1963_VCO_MAX_HZ    800000000U
#define SSD1963_PLL_MULTI_MIN 0x0U
#define SSD1963_PLL_MULTI_MAX 0xFFU
#define SSD1963_PLL_DIV_MIN   0x0U
#define SSD1963_PLL_DIV_MAX   0x1FU

/* The PLL output frequency will be configured to about 100MHz. */
#define SSD1963_PLL_FREQ_HZ 100000000U

/* The max value of LCDC_FPR to generate the lshift clock (pixel clock). */
#define SSD1963_LCDC_FPR_MAX 0xFFFFFU

/* The maximum number of parameters for a command */
#define SSD1963_NUM_COMMAND_PARAMS 8

/* SSD1963 commands */
#define SSD1963_NOP                      0x00U
#define SSD1963_SOFT_RESET               0x01U
#define SSD1963_GET_POWER_MODE           0x0AU
#define SSD1963_GET_ADDRESS_MODE         0x0BU
#define SSD1963_GET_DISPLAY_MODE         0x0DU
#define SSD1963_GET_TEAR_EFFECT_STATUS   0x0EU
#define SSD1963_ENTER_SLEEP_MODE         0x10U
#define SSD1963_EXIT_SLEEP_MODE          0x11U
#define SSD1963_ENTER_PARTIAL_MODE       0x12U
#define SSD1963_ENTER_NORMAL_MODE        0x13U
#define SSD1963_EXIT_INVERT_MODE         0x20U
#define SSD1963_ENTER_INVERT_MODE        0x21U
#define SSD1963_SET_GAMMA_CURVE          0x26U
#define SSD1963_SET_DISPLAY_OFF          0x28U
#define SSD1963_SET_DISPLAY_ON           0x29U
#define SSD1963_SET_COLUMN_ADDRESS       0x2AU
#define SSD1963_SET_PAGE_ADDRESS         0x2BU
#define SSD1963_WRITE_MEMORY_START       0x2CU
#define SSD1963_READ_MEMORY_START        0x2EU
#define SSD1963_SET_PARTIAL_AREA         0x30U
#define SSD1963_SET_SCROLL_AREA          0x33U
#define SSD1963_SET_TEAR_OFF             0x34U
#define SSD1963_SET_TEAR_ON              0x35U
#define SSD1963_SET_ADDRESS_MODE         0x36U
#define SSD1963_SET_SCROLL_START         0x37U
#define SSD1963_EXIT_IDLE_MODE           0x38U
#define SSD1963_ENTER_IDLE_MODE          0x39U
#define SSD1963_WRITE_MEMORY_CONTINUE    0x3CU
#define SSD1963_READ_MEMORY_CONTINUE     0x3EU
#define SSD1963_SET_TEAR_SCANLINE        0x44U
#define SSD1963_GET_SCANLINE             0x45U
#define SSD1963_READ_DDB                 0xA1U
#define SSD1963_SET_LCD_MODE             0xB0U
#define SSD1963_GET_LCD_MODE             0xB1U
#define SSD1963_SET_HORI_PERIOD          0xB4U
#define SSD1963_GET_HORI_PERIOD          0xB5U
#define SSD1963_SET_VERT_PERIOD          0xB6U
#define SSD1963_GET_VERT_PERIOD          0xB7U
#define SSD1963_SET_GPIO_CONF            0xB8U
#define SSD1963_GET_GPIO_CONF            0xB9U
#define SSD1963_SET_GPIO_VALUE           0xBAU
#define SSD1963_GET_GPIO_STATUS          0xBBU
#define SSD1963_SET_POST_PROC            0xBCU
#define SSD1963_GET_POST_PROC            0xBDU
#define SSD1963_SET_PWM_CONF             0xBEU
#define SSD1963_GET_PWM_CONF             0xBFU
#define SSD1963_GET_LCD_GEN0             0xC0U
#define SSD1963_SET_LCD_GEN0             0xC1U
#define SSD1963_GET_LCD_GEN1             0xC2U
#define SSD1963_SET_LCD_GEN1             0xC3U
#define SSD1963_GET_LCD_GEN2             0xC4U
#define SSD1963_SET_LCD_GEN2             0xC5U
#define SSD1963_GET_LCD_GEN3             0xC6U
#define SSD1963_SET_LCD_GEN3             0xC7U
#define SSD1963_SET_GPIO0_ROP            0xC8U
#define SSD1963_GET_GPIO0_ROP            0xC9U
#define SSD1963_SET_GPIO1_ROP            0xCAU
#define SSD1963_GET_GPIO1_ROP            0xCBU
#define SSD1963_SET_GPIO2_ROP            0xCCU
#define SSD1963_GET_GPIO2_ROP            0xCDU
#define SSD1963_SET_GPIO3_ROP            0xCEU
#define SSD1963_GET_GPIO3_ROP            0xCFU
#define SSD1963_SET_DBC_CONF             0xD0U
#define SSD1963_GET_DBC_CONF             0xD1U
#define SSD1963_SET_DBC_TH               0xD4U
#define SSD1963_GET_DBC_TH               0xD5U
#define SSD1963_SET_PLL                  0xE0U
#define SSD1963_SET_PLL_MN               0xE2U
#define SSD1963_GET_PLL_MN               0xE3U
#define SSD1963_GET_PLL_STATUS           0xE4U
#define SSD1963_SET_DEEP_SLEEP           0xE5U
#define SSD1963_SET_LSHIFT_FREQ          0xE6U
#define SSD1963_GET_LSHIFT_FREQ          0xE7U
#define SSD1963_SET_PIXEL_DATA_INTERFACE 0xF0U
#define SSD1963_GET_PIXEL_DATA_INTERFACE 0xF1U

/* Parameter offsets for SET LCD MODE command */
#define SSD1963_SET_LCD_WIDTH_OFF        5
#define SSD1963_SET_LCD_DOT_POL_OFF      2
#define SSD1963_SET_LCD_HSYNC_POL_OFF    1
#define SSD1963_SET_LCD_VSYNC_POL_OFF    0

#define SSD1963_ADDR_MODE_FLIP_VERT           (1U << 0)
#define SSD1963_ADDR_MODE_FLIP_HORZ           (1U << 1)
#define SSD1963_ADDR_MODE_LATCH_RIGHT_TO_LEFT (1U << 2)
#define SSD1963_ADDR_MODE_BGR                 (1U << 3)
#define SSD1963_ADDR_MODE_REFRESH_BOTTOM_UP   (1U << 4)
#define SSD1963_ADDR_MODE_PAG_COL_ADDR_ORDER  (1U << 5)
#define SSD1963_ADDR_MODE_COL_ADDR_ORDER      (1U << 6)
#define SSD1963_ADDR_MODE_PAGE_ADDR_ORDER     (1U << 7)

#define SSD1963_ORIENTATION_MODE_MASK 		\
	(SSD1963_ADDR_MODE_PAGE_ADDR_ORDER |	\
	SSD1963_ADDR_MODE_PAG_COL_ADDR_ORDER |	\
	SSD1963_ADDR_MODE_COL_ADDR_ORDER)

/*
 * SSD1963 orientation modes.
 */
enum ssd1963_orientation_mode {
	/* No rotation */
	SSD1963_ORIENTATION0 = 0U,
	/* Rotate 90 degree. */
	SSD1963_ORIENTATION90 =
		SSD1963_ADDR_MODE_PAGE_ADDR_ORDER | SSD1963_ADDR_MODE_PAG_COL_ADDR_ORDER,
	/* Rotate 180 degree. */
	SSD1963_ORIENTATION180 =
		SSD1963_ADDR_MODE_PAGE_ADDR_ORDER | SSD1963_ADDR_MODE_COL_ADDR_ORDER,
	/* Rotate 270 degree. */
	SSD1963_ORIENTATION270 =
		SSD1963_ADDR_MODE_COL_ADDR_ORDER | SSD1963_ADDR_MODE_PAG_COL_ADDR_ORDER,
};

#endif /* __SSD1963_REGS_H__ */
