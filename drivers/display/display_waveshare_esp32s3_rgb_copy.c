/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * EXPERIMENTAL out-of-tree Zephyr display driver for the ESP32-S3 LCD_CAM
 * peripheral in parallel RGB/DPI (hsync/vsync/de/pclk) timing-generator
 * mode, as used by the Waveshare ESP32-S3-Touch-LCD-7 panel.
 *
 * UNVERIFIED ON HARDWARE. This has not been compiled against a real Zephyr
 * SDK/HAL checkout, nor run on a board. Treat every low-level HAL/LL call
 * below as "matched the function signatures seen in-tree at the time of
 * writing" rather than "confirmed correct" - cross-check against your SDK's
 * actual `hal/lcd_ll.h`, `hal/gdma_hal.h`, `hal/gdma_ll.h` before relying on
 * it. See Zephyr/README.md for the full list of known-uncertain items.
 *
 * Design notes:
 *  - Zephyr's generic `espressif,esp32-gdma` driver (dma_esp32_gdma.c)
 *    always NULL-terminates its descriptor chain - there is no cyclic mode.
 *    Continuous RGB frame streaming needs the GDMA channel to loop the
 *    frame buffer forever without CPU intervention, so this driver claims
 *    one GDMA channel and drives it directly via gdma_hal_*() calls,
 *    building its own circular dma_descriptor_t list (last node's `next`
 *    points back to the first node) - mirroring what ESP-IDF's
 *    esp_lcd_panel_rgb.c does internally.
 *  - The peripheral's static timing/clock/mode registers are programmed via
 *    the same lcd_ll_*() calls Zephyr's own mipi_dbi_esp32.c uses for the
 *    DBI submode, plus additional RGB-mode-specific lcd_ll_*() calls that
 *    exist in ESP-IDF's HAL (lcd_ll_enable_rgb_mode, lcd_ll_set_*_timing,
 *    etc.) but aren't yet exercised anywhere in-tree in Zephyr.
 */

#define DT_DRV_COMPAT waveshare_esp32s3_lcd_cam_rgb

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/logging/log.h>
#include <soc.h>

/* ESP-IDF HAL/LL headers - vendored into the Zephyr ESP32 HAL module. */
#include <hal/lcd_ll.h>
#include <hal/lcd_hal.h>
#include <hal/gdma_hal.h>
#include <hal/gdma_hal_ahb.h>
#include <hal/gdma_ll.h>
#include <hal/dma_types.h>
#include <soc/lcd_cam_struct.h>
#include <soc/gdma_struct.h>

#include "display_waveshare_esp32s3_rgb.h"

#if defined(CONFIG_ESP_SPIRAM)
#include <zephyr/multi_heap/shared_multi_heap.h>
#endif

LOG_MODULE_REGISTER(wsrgb, CONFIG_DISPLAY_LOG_LEVEL);

/* The RGB565 frame buffer (several hundred KB) does not fit in internal
 * SRAM alongside the rest of the application, so it is carved out of
 * PSRAM via the shared multi-heap framework when CONFIG_ESP_SPIRAM is
 * enabled. k_aligned_alloc()/k_free() (the generic Zephyr system heap)
 * are only used as a fallback when PSRAM support isn't compiled in.
 */
static void *wsrgb_fb_alloc(size_t align, size_t bytes)
{
#if defined(CONFIG_ESP_SPIRAM)
	return shared_multi_heap_aligned_alloc(SMH_REG_ATTR_EXTERNAL, align, bytes);
#else
	return k_aligned_alloc(align, bytes);
#endif
}

static void wsrgb_fb_free(void *block)
{
#if defined(CONFIG_ESP_SPIRAM)
	shared_multi_heap_free(block);
#else
	k_free(block);
#endif
}

/* GDMA "connect to peripheral" trigger id for LCD, from
 * zephyr/drivers/dma/dma_esp32.h's public gdma_trigger_peripheral enum.
 */
#define WSRGB_GDMA_TRIG_PERIPH_LCD 5 /* ESP_GDMA_TRIG_PERIPH_LCD0 */

/* TODO(verify): exact RGB-mode "frame done"/VSYNC interrupt event bit name.
 * DBI mode uses LCD_LL_EVENT_TRANS_DONE; RGB mode's equivalent needs to be
 * confirmed against this SoC's hal/lcd_ll.h (candidates seen in ESP-IDF's
 * esp_lcd_panel_rgb.c: LCD_LL_EVENT_VSYNC_END / LCD_LL_EVENT_TRANS_DONE).
 */
#ifndef WSRGB_LL_EVENT_VSYNC
#define WSRGB_LL_EVENT_VSYNC LCD_LL_EVENT_TRANS_DONE
#endif

#define WSRGB_BYTES_PER_PIXEL(cfg) ((cfg)->data_width == 8 ? 1 : 2)

static gdma_hal_context_t s_gdma_hal;
static lcd_cam_dev_t *const s_lcd_cam = &LCD_CAM;

/* Build (or rebuild) the circular DMA descriptor chain that wraps the
 * whole frame buffer into DMA_DESCRIPTOR_BUFFER_MAX_SIZE-sized chunks and
 * links the last chunk's `next` back to the first, so GDMA loops the frame
 * forever once started.
 */
static int wsrgb_build_desc_chain(const struct device *dev)
{
	const struct wsrgb_config *cfg = dev->config;
	struct wsrgb_data *data = dev->data;

	size_t max_chunk = DMA_DESCRIPTOR_BUFFER_MAX_SIZE_4B_ALIGNED;
	size_t remaining = data->fb_size;
	size_t n_desc = DIV_ROUND_UP(data->fb_size, max_chunk);

	data->desc_pool_size = n_desc * sizeof(dma_descriptor_t);
	data->desc_pool = k_aligned_alloc(4, data->desc_pool_size);
	if (!data->desc_pool) {
		LOG_ERR("failed to allocate %u bytes for DMA descriptor pool",
			data->desc_pool_size);
		return -ENOMEM;
	}

	dma_descriptor_t *desc = (dma_descriptor_t *)data->desc_pool;
	uint8_t *buf = data->fb;

	for (size_t i = 0; i < n_desc; i++) {
		size_t chunk = MIN(remaining, max_chunk);

		desc[i].dw0.size = chunk;
		desc[i].dw0.length = chunk;
		desc[i].dw0.suc_eof = (i == n_desc - 1) ? 1 : 0;
		desc[i].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
		desc[i].buffer = buf;
		desc[i].next = &desc[(i + 1) % n_desc]; /* circular: wraps to desc[0] */

		buf += chunk;
		remaining -= chunk;
	}

	return 0;
}

static void wsrgb_free_desc_chain(struct device *dev)
{
	struct wsrgb_data *data = dev->data;

	if (data->desc_pool) {
		k_free(data->desc_pool);
		data->desc_pool = NULL;
	}
}

/* Program the static LCD_CAM timing-generator registers for RGB mode. */
static void wsrgb_configure_timing(const struct device *dev)
{
	const struct wsrgb_config *cfg = dev->config;

	/* NOTE: these lcd_ll_* names mirror ESP-IDF's esp_lcd_panel_rgb.c.
	 * Confirm exact signatures against this SoC's hal/lcd_ll.h.
	 */
	lcd_ll_enable_rgb_mode(s_lcd_cam, true);
	lcd_ll_enable_output_always_on(s_lcd_cam, true);
	lcd_ll_set_phase_cycles(s_lcd_cam, 0, 0, 1);

	/* lcd_ll_set_dma_read_stride() selects the DMA read unit width in
	 * bits (8 or 16), not a byte stride - passing anything else hits
	 * an abort() in the HAL.
	 */
	lcd_ll_set_dma_read_stride(s_lcd_cam, WSRGB_BYTES_PER_PIXEL(cfg) * 8);

	lcd_ll_set_horizontal_timing(s_lcd_cam, cfg->hsync_len, cfg->hback_porch,
				      cfg->width, cfg->hfront_porch);
	lcd_ll_set_vertical_timing(s_lcd_cam, cfg->vsync_len, cfg->vback_porch,
				    cfg->height, cfg->vfront_porch);

	lcd_ll_set_idle_level(s_lcd_cam, !cfg->hsync_idle_low, !cfg->vsync_idle_low,
			       cfg->de_idle_high);
	lcd_ll_enable_output_hsync_in_porch_region(s_lcd_cam, true);
	lcd_ll_set_pixel_clock_edge(s_lcd_cam, cfg->pclk_active_neg);

	/* Pixel clock divider - actual base clock source/coefficients need
	 * confirming; this assumes the LCD_CAM root clock is already routed
	 * by the parent espressif,esp32-lcd-cam node's clock-frequency
	 * property, and only prescale needs setting here.
	 */
	lcd_ll_set_pixel_clock_prescale(s_lcd_cam, 1);

	lcd_ll_fifo_reset(s_lcd_cam);
}

/* Claim & start the dedicated GDMA channel with the circular descriptor
 * chain, bypassing Zephyr's dma_esp32_gdma driver instance entirely.
 */
static int wsrgb_start_dma(const struct device *dev)
{
	const struct wsrgb_config *cfg = dev->config;
	struct wsrgb_data *data = dev->data;

	gdma_hal_config_t hal_config = {
		.group_id = GDMA_LL_AHB_GROUP_START_ID, /* ESP32-S3 uses AHB GDMA, not AXI */
	};

	gdma_ahb_hal_init(&s_gdma_hal, &hal_config);
	gdma_hal_reset(&s_gdma_hal, cfg->dma_channel, GDMA_CHANNEL_DIRECTION_TX);
	gdma_hal_connect_peri(&s_gdma_hal, cfg->dma_channel, GDMA_CHANNEL_DIRECTION_TX,
			       WSRGB_GDMA_TRIG_PERIPH_LCD);
	gdma_hal_enable_intr(&s_gdma_hal, cfg->dma_channel, GDMA_CHANNEL_DIRECTION_TX,
			      GDMA_LL_EVENT_TX_EOF, true);
	gdma_hal_start_with_desc(&s_gdma_hal, cfg->dma_channel, GDMA_CHANNEL_DIRECTION_TX,
				  (intptr_t)data->desc_pool);

	lcd_ll_start(s_lcd_cam);
	data->streaming = true;

	return 0;
}

static void wsrgb_stop_dma(const struct device *dev)
{
	const struct wsrgb_config *cfg = dev->config;
	struct wsrgb_data *data = dev->data;

	lcd_ll_stop(s_lcd_cam);
	gdma_hal_stop(&s_gdma_hal, cfg->dma_channel, GDMA_CHANNEL_DIRECTION_TX);
	data->streaming = false;
}

/* LCD_CAM interrupt handler - clears + (optionally) signals VSYNC/frame
 * completion. TODO(verify): WSRGB_LL_EVENT_VSYNC bit name, see top of file.
 */
static void wsrgb_isr(const struct device *dev)
{
	struct wsrgb_data *data = dev->data;
	uint32_t status = lcd_ll_get_interrupt_status(s_lcd_cam);

	lcd_ll_clear_interrupt_status(s_lcd_cam, status);

	if (status & WSRGB_LL_EVENT_VSYNC) {
		k_sem_give(&data->vsync_sem);
	}
}

/* ---- Display API ---------------------------------------------------- */

static int wsrgb_write(const struct device *dev, const uint16_t x, const uint16_t y,
			const struct display_buffer_descriptor *desc, const void *buf)
{
	const struct wsrgb_config *cfg = dev->config;
	struct wsrgb_data *data = dev->data;
	const uint8_t bypp = WSRGB_BYTES_PER_PIXEL(cfg);
	const uint8_t *src = buf;
	uint8_t *dst = data->fb;

	if ((x + desc->width) > cfg->width || (y + desc->height) > cfg->height) {
		return -EINVAL;
	}

	for (uint16_t row = 0; row < desc->height; row++) {
		size_t src_off = row * desc->pitch * bypp;
		size_t dst_off = ((y + row) * cfg->width + x) * bypp;

		memcpy(dst + dst_off, src + src_off, desc->width * bypp);
	}

	/* Frame buffer is scanned continuously by DMA; nothing else to
	 * "kick" here beyond the memcpy above becoming visible on the next
	 * pass. Callers that need tear-free updates should wait on VSYNC
	 * (not currently exposed through the Display API - see README).
	 */
	return 0;
}

static int wsrgb_read(const struct device *dev, const uint16_t x, const uint16_t y,
		       const struct display_buffer_descriptor *desc, void *buf)
{
	const struct wsrgb_config *cfg = dev->config;
	struct wsrgb_data *data = dev->data;
	const uint8_t bypp = WSRGB_BYTES_PER_PIXEL(cfg);
	uint8_t *dst = buf;
	const uint8_t *src = data->fb;

	for (uint16_t row = 0; row < desc->height; row++) {
		size_t src_off = ((y + row) * cfg->width + x) * bypp;
		size_t dst_off = row * desc->pitch * bypp;

		memcpy(dst + dst_off, src + src_off, desc->width * bypp);
	}

	return 0;
}

static void *wsrgb_get_framebuffer(const struct device *dev)
{
	struct wsrgb_data *data = dev->data;

	return data->fb;
}

static int wsrgb_blanking_off(const struct device *dev)
{
	struct wsrgb_data *data = dev->data;

	if (!data->streaming) {
		return wsrgb_start_dma(dev);
	}
	return 0;
}

static int wsrgb_blanking_on(const struct device *dev)
{
	wsrgb_stop_dma(dev);
	return 0;
}

static int wsrgb_set_pixel_format(const struct device *dev, const enum display_pixel_format fmt)
{
	struct wsrgb_data *data = dev->data;

	if (fmt != PIXEL_FORMAT_RGB_565) {
		/* Only RGB565 is wired up in this experimental driver. */
		return -ENOTSUP;
	}
	data->pixel_format = fmt;
	return 0;
}

static void wsrgb_get_capabilities(const struct device *dev, struct display_capabilities *caps)
{
	const struct wsrgb_config *cfg = dev->config;

	memset(caps, 0, sizeof(*caps));
	caps->x_resolution = cfg->width;
	caps->y_resolution = cfg->height;
	caps->supported_pixel_formats = PIXEL_FORMAT_RGB_565;
	caps->current_pixel_format = PIXEL_FORMAT_RGB_565;
	caps->screen_info = 0;
}

static const struct display_driver_api wsrgb_api = {
	.write = wsrgb_write,
	.read = wsrgb_read,
	.get_framebuffer = wsrgb_get_framebuffer,
	.blanking_on = wsrgb_blanking_on,
	.blanking_off = wsrgb_blanking_off,
	.set_pixel_format = wsrgb_set_pixel_format,
	.get_capabilities = wsrgb_get_capabilities,
};

/* ---- Init ------------------------------------------------------------ */

static int wsrgb_init(const struct device *dev)
{
	const struct wsrgb_config *cfg = dev->config;
	struct wsrgb_data *data = dev->data;
	int ret;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("pinctrl apply failed: %d", ret);
		return ret;
	}

	data->fb_size = (size_t)cfg->width * cfg->height * WSRGB_BYTES_PER_PIXEL(cfg);

	data->fb = wsrgb_fb_alloc(64, data->fb_size);
	if (!data->fb) {
		LOG_ERR("failed to allocate %u byte frame buffer", data->fb_size);
		return -ENOMEM;
	}
	memset(data->fb, 0, data->fb_size);

	k_sem_init(&data->vsync_sem, 0, 1);
	data->pixel_format = PIXEL_FORMAT_RGB_565;
	data->streaming = false;

	ret = wsrgb_build_desc_chain(dev);
	if (ret) {
		wsrgb_fb_free(data->fb);
		return ret;
	}

	wsrgb_configure_timing(dev);

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), wsrgb_isr,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	lcd_ll_enable_interrupt(s_lcd_cam, WSRGB_LL_EVENT_VSYNC, true);

	return wsrgb_start_dma(dev);
}

#define WSRGB_INIT(inst)                                                                          \
	PINCTRL_DT_INST_DEFINE(inst);                                                             \
	static const struct wsrgb_config wsrgb_cfg_##inst = {                                     \
		.width = DT_INST_PROP(inst, width),                                               \
		.height = DT_INST_PROP(inst, height),                                             \
		.pixel_clk_hz = DT_INST_PROP(inst, pixel_clk_hz),                                 \
		.hsync_len = DT_INST_PROP(inst, hsync_len),                                       \
		.hback_porch = DT_INST_PROP(inst, hback_porch),                                   \
		.hfront_porch = DT_INST_PROP(inst, hfront_porch),                                 \
		.vsync_len = DT_INST_PROP(inst, vsync_len),                                       \
		.vback_porch = DT_INST_PROP(inst, vback_porch),                                   \
		.vfront_porch = DT_INST_PROP(inst, vfront_porch),                                 \
		.data_width = DT_INST_PROP_OR(inst, data_width, 16),                              \
		.hsync_idle_low = DT_INST_PROP(inst, hsync_idle_low),                             \
		.vsync_idle_low = DT_INST_PROP(inst, vsync_idle_low),                             \
		.de_idle_high = DT_INST_PROP(inst, de_idle_high),                                 \
		.pclk_active_neg = DT_INST_PROP(inst, pclk_active_neg),                           \
		.dma_channel = DT_INST_DMAS_CELL_BY_NAME(inst, tx, channel),                      \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                     \
	};                                                                                         \
	static struct wsrgb_data wsrgb_data_##inst;                                               \
	DEVICE_DT_INST_DEFINE(inst, wsrgb_init, NULL, &wsrgb_data_##inst, &wsrgb_cfg_##inst,       \
			      POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY, &wsrgb_api);

DT_INST_FOREACH_STATUS_OKAY(WSRGB_INIT)
