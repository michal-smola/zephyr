/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT solomon_ssd1963

#include <zephyr/drivers/display.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include "display_mcux_flexio_lcdif.h"
#include "ssd1963_regs.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ssd1963, CONFIG_DISPLAY_LOG_LEVEL);

struct ssd1963_config {
	const struct device *flexio_lcd_dev;
	const struct gpio_dt_spec reset_gpio;
	uint8_t bus_width;
	uint8_t pixel_format;
	uint32_t xtal_frequency;
	uint32_t pixel_clock;
	uint8_t panel_data_width;
	uint16_t panel_width;
	uint16_t panel_height;
	uint8_t vsync_polarity;
	uint8_t hsync_polarity;
	uint8_t dot_polarity;
	uint16_t hsw;
	uint16_t hfp;
	uint16_t hbp;
	uint16_t vsw;
	uint16_t vfp;
	uint16_t vbp;
};

struct ssd1963_data {
	uint8_t addr_mode;
};

static void ssd1963_set_backlight(const struct device *dev, uint8_t value)
{
	const struct ssd1963_config *config = dev->config;
	/* Param 1: sets PWM frequency; PWM freq = PLL clock / (256 * Param 1) / 256
	 * Param 2: sets PWM duty cycle: value (passed in) / 256
	 * Param 3: PWM Enable
	 * Param 4: Maximum Brightness level
	 * Param 5: Minimum Brightness level
	 * Param 6: Brightness prescaler, controls how gradually brightness is changed
	 */
	uint8_t command_param[] = {0x06U, value, 0x01U, 0xFFU, 0x00U, 0x01U};

	flexio_lcdif_write_command(config->flexio_lcd_dev, SSD1963_SET_PWM_CONF);
	flexio_lcdif_write_data(config->flexio_lcd_dev, command_param, sizeof(command_param));
}

static int ssd1963_blanking_off(const struct device *dev)
{
	const struct ssd1963_config *config = dev->config;

	flexio_lcdif_write_command(config->flexio_lcd_dev, SSD1963_SET_DISPLAY_ON);
	ssd1963_set_backlight(dev, 255);

	return 0;
}

static int ssd1963_blanking_on(const struct device *dev)
{
	const struct ssd1963_config *config = dev->config;

	flexio_lcdif_write_command(config->flexio_lcd_dev, SSD1963_SET_DISPLAY_OFF);

	return 0;
}

static int ssd1963_select_area(const struct device *dev, const uint16_t start_x,
			       const uint16_t start_y, const uint16_t end_x,
			       const uint16_t end_y)
{
	const struct ssd1963_config *config = dev->config;
	struct ssd1963_data *lcd_data = dev->data;
	/* Start of column number. */
	uint16_t sc;
	/* End of column number. */
	uint16_t ec;
	/* Start of page number. */
	uint16_t sp;
	/* End of page number. */
	uint16_t ep;
	uint8_t mode;
	/* Command parameters for set_page_address and set_column_address. */
	uint8_t command_param[4];

	mode = (lcd_data->addr_mode & SSD1963_ORIENTATION_MODE_MASK);
	switch (mode) {
	case SSD1963_ORIENTATION90:
		sp = config->panel_height - 1U - end_x;
		ep = config->panel_height - 1U - start_x;
		sc = start_y;
		ec = end_y;
		break;
	case SSD1963_ORIENTATION180:
		sp = config->panel_height - 1U - end_y;
		ep = config->panel_height - 1U - start_y;
		sc = config->panel_width - 1U - end_x;
		ec = config->panel_width - 1U - start_x;
		break;
	case SSD1963_ORIENTATION270:
		sp = start_x;
		ep = end_x;
		sc = config->panel_width - 1U - end_y;
		ec = config->panel_width - 1U - start_y;
		break;
	case SSD1963_ORIENTATION0:
	default:
		sp = start_y;
		ep = end_y;
		sc = start_x;
		ec = end_x;
		break;
	}

	/* Send the set_page_address command. */
	command_param[0] = (uint8_t)((sp & 0xFF00U) >> 8U);
	command_param[1] = (uint8_t)(sp & 0xFFU);
	command_param[2] = (uint8_t)((ep & 0xFF00U) >> 8U);
	command_param[3] = (uint8_t)(ep & 0xFFU);

	flexio_lcdif_write_command(config->flexio_lcd_dev, SSD1963_SET_PAGE_ADDRESS);
	flexio_lcdif_write_data(config->flexio_lcd_dev, command_param, 4U);

	/* Send the set_column_address command. */
	command_param[0] = (uint8_t)((sc & 0xFF00U) >> 8U);
	command_param[1] = (uint8_t)(sc & 0xFFU);
	command_param[2] = (uint8_t)((ec & 0xFF00U) >> 8U);
	command_param[3] = (uint8_t)(ec & 0xFFU);

	flexio_lcdif_write_command(config->flexio_lcd_dev, SSD1963_SET_COLUMN_ADDRESS);
	flexio_lcdif_write_data(config->flexio_lcd_dev, command_param, 4U);

	return 0;
}

static int ssd1963_write(const struct device *dev, const uint16_t x,
			 const uint16_t y,
			 const struct display_buffer_descriptor *desc,
			 const void *buf)
{
	const struct ssd1963_config *config = dev->config;

	LOG_DBG("W=%d, H=%d, @%d,%d", desc->width, desc->height, x, y);

	ssd1963_select_area(dev, x, y, desc->width, desc->height);

	flexio_lcdif_write_memory(config->flexio_lcd_dev, SSD1963_WRITE_MEMORY_START,
				  buf, desc->buf_size);

	return 0;
}

static int ssd1963_read(const struct device *dev,
			const uint16_t x, const uint16_t y,
			const struct display_buffer_descriptor *desc,
			void *buf)
{
	LOG_ERR("Read not implemented");
	return -ENOTSUP;
}

static void *ssd1963_get_framebuffer(const struct device *dev)
{
	LOG_ERR("Direct framebuffer access not available");
	return NULL;
}

static int ssd1963_set_brightness(const struct device *dev,
				  const uint8_t brightness)
{
	LOG_WRN("Set brightness not implemented");
	return -ENOTSUP;
}

static int ssd1963_set_contrast(const struct device *dev, uint8_t contrast)
{
	LOG_WRN("Set contrast not implemented");
	return -ENOTSUP;
}

static void ssd1963_get_capabilities(const struct device *dev,
				     struct display_capabilities *caps)
{
	const struct ssd1963_config *config = dev->config;
	struct ssd1963_data *lcd_data = dev->data;
	uint8_t orientation = (lcd_data->addr_mode & SSD1963_ORIENTATION_MODE_MASK);

	memset(caps, 0, sizeof(struct display_capabilities));
	caps->x_resolution = config->panel_width;
	caps->y_resolution = config->panel_height;
	caps->supported_pixel_formats = (PIXEL_FORMAT_BGR_565 | PIXEL_FORMAT_RGB_565);

	if (lcd_data->addr_mode & SSD1963_ADDR_MODE_BGR) {
		caps->current_pixel_format = PIXEL_FORMAT_BGR_565;
	} else {
		caps->current_pixel_format = PIXEL_FORMAT_RGB_565;
	}

	switch (orientation) {
	case SSD1963_ORIENTATION90:
		caps->current_orientation = DISPLAY_ORIENTATION_ROTATED_90;
		break;
	case SSD1963_ORIENTATION180:
		caps->current_orientation = DISPLAY_ORIENTATION_ROTATED_180;
		break;
	case SSD1963_ORIENTATION270:
		caps->current_orientation = DISPLAY_ORIENTATION_ROTATED_270;
		break;
	case SSD1963_ORIENTATION0:
	default:
		caps->current_orientation = DISPLAY_ORIENTATION_NORMAL;
		break;
	}
}

static int ssd1963_set_orientation(const struct device *dev,
				   const enum display_orientation
				   orientation)
{
	const struct ssd1963_config *config = dev->config;
	struct ssd1963_data *lcd_data = dev->data;
	uint8_t command_param[1];

	lcd_data->addr_mode &= (uint8_t)(~SSD1963_ORIENTATION_MODE_MASK);

	switch (orientation) {
	case DISPLAY_ORIENTATION_ROTATED_90:
		lcd_data->addr_mode |= (SSD1963_ORIENTATION90);
		break;
	case DISPLAY_ORIENTATION_ROTATED_180:
		lcd_data->addr_mode |= (SSD1963_ORIENTATION180);
		break;
	case DISPLAY_ORIENTATION_ROTATED_270:
		lcd_data->addr_mode |= (SSD1963_ORIENTATION270);
		break;
	case DISPLAY_ORIENTATION_NORMAL:
	default:
		break;
	}

	command_param[0] = lcd_data->addr_mode;
	flexio_lcdif_write_command(config->flexio_lcd_dev, SSD1963_SET_ADDRESS_MODE);
	flexio_lcdif_write_data(config->flexio_lcd_dev, command_param, 1U);

	return 0;

	return 0;
}

static int ssd1963_set_pixel_format(const struct device *dev,
				    const enum display_pixel_format pf)
{
	const struct ssd1963_config *config = dev->config;
	struct ssd1963_data *lcd_data = dev->data;
	uint8_t command_param[1];

	lcd_data->addr_mode &= (uint8_t)(~SSD1963_ADDR_MODE_BGR);

	/* Address mode. */
	if (config->pixel_format == PIXEL_FORMAT_BGR_565) {
		lcd_data->addr_mode |= SSD1963_ADDR_MODE_BGR;
	}

	command_param[0] = lcd_data->addr_mode;
	flexio_lcdif_write_command(config->flexio_lcd_dev, SSD1963_SET_ADDRESS_MODE);
	flexio_lcdif_write_data(config->flexio_lcd_dev, command_param, 1U);

	return 0;
}

static int ssd1963_get_pll_divider(uint8_t *multi, uint8_t *div, uint32_t src_clock)
{
	uint32_t multi_cur, div_cur, pll_freq_cur, vco_cur, diff_cur;
	uint32_t multi_candidate = 0U;
	uint32_t div_candidate = 0U;
	uint32_t pll_freq_candidate = 0U;
	uint32_t diff = 0xFFFFFFFFU;

	for (multi_cur = SSD1963_PLL_MULTI_MIN;
		multi_cur <= SSD1963_PLL_MULTI_MAX; multi_cur++) {
		vco_cur = src_clock * (multi_cur + 1U);

		/* VCO must be larger than SSD1963_VCO_MIN_HZ. */
		if (vco_cur <= SSD1963_VCO_MIN_HZ) {
			continue;
		}

		/* VCO must be smaller than SSD1963_VCO_MAX_HZ. */
		if (vco_cur >= SSD1963_VCO_MAX_HZ) {
			break;
		}

		div_cur = ((vco_cur + (SSD1963_PLL_FREQ_HZ / 2U)) /
			SSD1963_PLL_FREQ_HZ) - 1U;

		/*
		 * VCO frequency must be in the range of (250MHz, 800MHz). The desired
		 * PLL output frequency is 100MHz, then the div_cur here must be in the
		 * range of (1, 8). In this case, it is not necessary to check whether
		 * div_cur is in the range of (0, 31). But for safty when the desired
		 * PLL frequency is changed, here check the upper range.
		 */
#if ((((SSD1963_VCO_MAX_HZ + (SSD1963_PLL_FREQ_HZ / 2U)) /	\
	SSD1963_PLL_FREQ_HZ) - 1U) > SSD1963_PLL_DIV_MAX)
		if (div_cur > SSD1963_PLL_DIV_MAX) {
			div_cur = SSD1963_PLL_DIV_MAX;
		}
#endif

		pll_freq_cur = vco_cur / (div_cur + 1U);

		if (SSD1963_PLL_FREQ_HZ > pll_freq_cur) {
			diff_cur = SSD1963_PLL_FREQ_HZ - pll_freq_cur;
		} else {
			diff_cur = pll_freq_cur - SSD1963_PLL_FREQ_HZ;
		}

		/* Find better multi and divider. */
		if (diff > diff_cur) {
			diff = diff_cur;
			multi_candidate = multi_cur;
			div_candidate = div_cur;
			pll_freq_candidate = pll_freq_cur;
		}
	}

	*multi = (uint8_t)multi_candidate;
	*div   = (uint8_t)div_candidate;

	return pll_freq_candidate;
}

static void ssd1963_set_data_interface(const struct device *dev)
{
	const struct ssd1963_config *config = dev->config;
	uint8_t command_param[1];

	/* Data interface. */
	switch (config->bus_width) {
	case 8:
		command_param[0] = 0;
		break;
	case 9:
		command_param[0] = 6;
		break;
	case 12:
		command_param[0] = 1;
		break;
	case 16:
		command_param[0] = 3;
		break;
	case 18:
		command_param[0] = 4;
		break;
	default:
		command_param[0] = 5;
		break;
	}

	flexio_lcdif_write_command(config->flexio_lcd_dev, SSD1963_SET_PIXEL_DATA_INTERFACE);
	flexio_lcdif_write_data(config->flexio_lcd_dev, command_param, 1U);
}

static int ssd1963_init(const struct device *dev)
{
	const struct ssd1963_config *config = dev->config;
	int err;
	uint8_t multi, div;
	uint32_t pll_freq_hz;
	/* Pixel clock = PLL clock * ((fpr + 1) / 2^20) */
	uint32_t fpr;
	float fprFloat;
	uint16_t vt, vps, ht, hps;
	uint8_t command_param[SSD1963_NUM_COMMAND_PARAMS];

	if (!device_is_ready(config->flexio_lcd_dev)) {
		return -ENODEV;
	}

	/* Reset the SSD1963 LCD controller. */
	err = gpio_pin_set_dt(&config->reset_gpio, 0);
	if (err < 0) {
		return err;
	}

	k_usleep(1);

	err = gpio_pin_set_dt(&config->reset_gpio, 1);
	if (err < 0) {
		return err;
	}

	k_msleep(5);

	pll_freq_hz = ssd1963_get_pll_divider(&multi, &div, config->xtal_frequency);
	/* Could not set the PLL to desired frequency. */
	if (0U == pll_freq_hz) {
		return -EINVAL;
	}

	/* Pixel clock = PLL freq x ((LCDC_FPR + 1) / 2^20)
	 * 1048576 = 1 << 20
	 */
	fprFloat = ((float)config->pixel_clock /
		(float)pll_freq_hz) * (float)1048576.0f;
	fpr = (uint32_t)fprFloat;

	if ((fpr < 1U) || (fpr > (SSD1963_LCDC_FPR_MAX + 1U))) {
		return -EINVAL;
	}

	fpr--;

	/* Setup the PLL. */
	/* Set the multiplier and divider. */
	command_param[0] = multi;
	command_param[1] = (uint8_t)(div | (1U << 5U));
	command_param[2] = 1U << 2U;

	flexio_lcdif_write_command(config->flexio_lcd_dev, SSD1963_SET_PLL_MN);
	flexio_lcdif_write_data(config->flexio_lcd_dev, command_param, 3U);

	/* Enable PLL. */
	command_param[0] = 0x01U;
	flexio_lcdif_write_command(config->flexio_lcd_dev, SSD1963_SET_PLL);
	flexio_lcdif_write_data(config->flexio_lcd_dev, command_param, 1U);

	/* Delay at least 100us, to wait for the PLL stable. */
	k_usleep(500);

	/* Use the PLL. */
	command_param[0] = 0x03U;
	flexio_lcdif_write_command(config->flexio_lcd_dev, SSD1963_SET_PLL);
	flexio_lcdif_write_data(config->flexio_lcd_dev, command_param, 1U);

	/* Configure the pixel clock. */
	command_param[0] = ((fpr & 0xFF0000U) >> 16U);
	command_param[1] = ((fpr & 0xFF00U) >> 8U);
	command_param[2] = ((fpr & 0xFFU));
	flexio_lcdif_write_command(config->flexio_lcd_dev, SSD1963_SET_LSHIFT_FREQ);
	flexio_lcdif_write_data(config->flexio_lcd_dev, command_param, 3U);

	/* Configure LCD panel. */
	command_param[0] = (uint8_t)((uint8_t)(config->dot_polarity << SSD1963_SET_LCD_DOT_POL_OFF) |
			(uint8_t)(config->vsync_polarity << SSD1963_SET_LCD_VSYNC_POL_OFF) |
			(uint8_t)(config->hsync_polarity << SSD1963_SET_LCD_HSYNC_POL_OFF) |
			(uint8_t)(config->panel_data_width << SSD1963_SET_LCD_WIDTH_OFF));
	command_param[1] = 0x20U;
	command_param[2] = (uint8_t)((config->panel_width - 1U) >> 8);
	command_param[3] = (uint8_t)((config->panel_width - 1U) & 0xFFU);
	command_param[4] = (uint8_t)((config->panel_height - 1U) >> 8);
	command_param[5] = (uint8_t)((config->panel_height - 1U) & 0xFFU);
	command_param[6] = 0;
	flexio_lcdif_write_command(config->flexio_lcd_dev, SSD1963_SET_LCD_MODE);
	flexio_lcdif_write_data(config->flexio_lcd_dev, command_param, 7U);

	/* Horizontal period setting. */
	ht = config->panel_width + config->hsw + config->hfp + config->hbp;
	hps = config->hsw + config->hbp;
	command_param[0] = (uint8_t)((ht - 1U) >> 8U);
	command_param[1] = (uint8_t)((ht - 1U) & 0xFFU);
	command_param[2] = (uint8_t)(hps >> 8U);
	command_param[3] = (uint8_t)(hps & 0xFFU);
	command_param[4] = (uint8_t)(config->hsw - 1U);
	command_param[5] = 0U;
	command_param[6] = 0U;
	command_param[7] = 0U;
	flexio_lcdif_write_command(config->flexio_lcd_dev, SSD1963_SET_HORI_PERIOD);
	flexio_lcdif_write_data(config->flexio_lcd_dev, command_param, 8U);

	/* Vertical period setting. */
	vt = config->panel_width + config->vsw + config->vfp + config->vbp;
	vps = config->vsw + config->vbp;
	command_param[0] = (uint8_t)((vt - 1U) >> 8U);
	command_param[1] = (uint8_t)((vt - 1U) & 0xFFU);
	command_param[2] = (uint8_t)(vps >> 8U);
	command_param[3] = (uint8_t)(vps & 0xFFU);
	command_param[4] = (uint8_t)(config->vsw - 1U);
	command_param[5] = 0U;
	command_param[6] = 0U;
	flexio_lcdif_write_command(config->flexio_lcd_dev, SSD1963_SET_VERT_PERIOD);
	flexio_lcdif_write_data(config->flexio_lcd_dev, command_param, 7U);

	ssd1963_set_data_interface(dev);
	ssd1963_set_pixel_format(dev, config->pixel_format);

	LOG_DBG("%s device is ready", dev->name);

	return 0;
}

static struct display_driver_api ssd1963_driver_api = {
	.blanking_on = ssd1963_blanking_on,
	.blanking_off = ssd1963_blanking_off,
	.write = ssd1963_write,
	.read = ssd1963_read,
	.get_framebuffer = ssd1963_get_framebuffer,
	.set_brightness = ssd1963_set_brightness,
	.set_contrast = ssd1963_set_contrast,
	.get_capabilities = ssd1963_get_capabilities,
	.set_pixel_format = ssd1963_set_pixel_format,
	.set_orientation = ssd1963_set_orientation,
};

#define SSD1963_DEFINE(n)								\
	static const struct ssd1963_config ssd1963_cfg_##n = {				\
		.flexio_lcd_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, nxp_lcdif)),		\
		.reset_gpio = GPIO_DT_SPEC_INST_GET(n, reset_gpios),			\
		.bus_width = DT_INST_PROP(n, data_bus_width),				\
		.pixel_format = DT_INST_PROP(n, pixel_format),				\
		.xtal_frequency = DT_INST_PROP(n, xtal_frequency),			\
		.pixel_clock = DT_PROP(DT_INST_CHILD(n, display_timings), clock_frequency), \
		.panel_data_width = DT_INST_PROP(n, panel_data_width),			\
		.panel_width = DT_INST_PROP(n, width),					\
		.panel_height = DT_INST_PROP(n, height),				\
		.vsync_polarity = DT_PROP(DT_INST_CHILD(n, display_timings), vsync_active), \
		.hsync_polarity = DT_PROP(DT_INST_CHILD(n, display_timings), hsync_active), \
		.dot_polarity = DT_PROP(DT_INST_CHILD(n, display_timings), pixelclk_active),\
		.hsw = DT_PROP(DT_INST_CHILD(n, display_timings), hsync_len),		\
		.hfp = DT_PROP(DT_INST_CHILD(n, display_timings), hfront_porch),	\
		.hbp = DT_PROP(DT_INST_CHILD(n, display_timings), hback_porch),		\
		.vsw = DT_PROP(DT_INST_CHILD(n, display_timings), vsync_len),		\
		.vfp = DT_PROP(DT_INST_CHILD(n, display_timings), vfront_porch),	\
		.vbp = DT_PROP(DT_INST_CHILD(n, display_timings), vback_porch),		\
	};										\
											\
	static struct ssd1963_data ssd1963_data_##n;					\
											\
	DEVICE_DT_INST_DEFINE(n,							\
			 &ssd1963_init, NULL,						\
			 &ssd1963_data_##n,						\
			 &ssd1963_cfg_##n,						\
			 POST_KERNEL,							\
			 CONFIG_DISPLAY_INIT_PRIORITY,					\
			 &ssd1963_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SSD1963_DEFINE)
