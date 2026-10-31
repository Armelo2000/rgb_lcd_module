/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP32-S3 RGB scanout with a PSRAM framebuffer and SRAM bounce buffers.
 */
#ifndef ZEPHYR_RGB_LCD_MODULE_DRIVERS_DISPLAY_DISPLAY_WAVESHARE_ESP32S3_RGB_H_
#define ZEPHYR_RGB_LCD_MODULE_DRIVERS_DISPLAY_DISPLAY_WAVESHARE_ESP32S3_RGB_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>

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
	int dma_irq_source;
	int irq_source;
	int irq_priority;
	int irq_flags;
	const struct pinctrl_dev_config *pcfg;
	const struct device *lcd_cam_dev; /* parent espressif,esp32-lcd-cam device */
	const struct device *clock_dev;   /* espressif,esp32-clock controller */
	clock_control_subsys_t clock_subsys; /* ESP32_LCD_CAM_MODULE bus-clock/reset subsystem */
};

/* Runtime state. */
struct wsrgb_data {
	uint8_t *fb;              /* frame buffer, ideally in PSRAM */
	size_t fb_size;
	void *desc_pool;          /* backing storage for the circular dma_descriptor_t list */
	size_t desc_pool_size;
	void *restart_desc;       /* VSYNC entry, skipping pixels retained by LCD FIFO */
	uint8_t *bounce_buf[2];
	size_t bounce_size;
	size_t bounce_offset;
	void *bounce_eof_desc[2];
	uint32_t bounce_eof_count;
	uint32_t restart_count;
	uint32_t reported_restart_count;
	uint8_t bounce_expected;
	struct k_spinlock scanout_lock;
	volatile uint32_t vsync_count;
	uint8_t dma_channel;      /* ISR copy; config resides in flash */
	enum display_pixel_format pixel_format;
	bool streaming;
};

int count(){

	return 100;
}
#endif /* ZEPHYR_RGB_LCD_MODULE_DRIVERS_DISPLAY_DISPLAY_WAVESHARE_ESP32S3_RGB_H_ */
