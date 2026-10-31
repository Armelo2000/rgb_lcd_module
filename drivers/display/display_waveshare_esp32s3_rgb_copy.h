/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * EXPERIMENTAL, unverified-on-hardware driver. See Zephyr/README.md for the
 * list of known-uncertain items (continuous DMA looping, VSYNC event bit
 * name, PSRAM framebuffer placement).
 */
#ifndef ZEPHYR_RGB_LCD_MODULE_DRIVERS_DISPLAY_DISPLAY_WAVESHARE_ESP32S3_RGB_H_
#define ZEPHYR_RGB_LCD_MODULE_DRIVERS_DISPLAY_DISPLAY_WAVESHARE_ESP32S3_RGB_H_

#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/pinctrl.h>

/* Devicetree-derived, static panel configuration. */
struct wsrgb_config {
	uint32_t width;
	uint32_t height;
	uint32_t pixel_clk_hz;
	uint32_t hsync_len;
	uint32_t hback_porch;
	uint32_t hfront_porch;
	uint32_t vsync_len;
	uint32_t vback_porch;
	uint32_t vfront_porch;
	uint8_t data_width;
	bool hsync_idle_low;
	bool vsync_idle_low;
	bool de_idle_high;
	bool pclk_active_neg;
	uint8_t dma_channel;
	const struct pinctrl_dev_config *pcfg;
	const struct device *lcd_cam_dev; /* parent espressif,esp32-lcd-cam device */
};

/* Runtime state. */
struct wsrgb_data {
	uint8_t *fb;              /* frame buffer, ideally in PSRAM */
	size_t fb_size;
	void *desc_pool;          /* backing storage for the circular dma_descriptor_t list */
	size_t desc_pool_size;
	enum display_pixel_format pixel_format;
	struct k_sem vsync_sem;
	bool streaming;
};

#endif /* ZEPHYR_RGB_LCD_MODULE_DRIVERS_DISPLAY_DISPLAY_WAVESHARE_ESP32S3_RGB_H_ */
