/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * EXPERIMENTAL out-of-tree Zephyr display driver for the ESP32-S3 LCD_CAM
 * peripheral in parallel RGB/DPI (hsync/vsync/de/pclk) timing-generator
 * mode, as used by the Waveshare ESP32-S3-Touch-LCD-7 panel.
 *
 * Scanout uses two internal-SRAM bounce buffers filled from the PSRAM
 * framebuffer at DMA EOF. Keep PSRAM/cache accessible during scanout;
 * blank the display before operations that disable external-memory access.
 *
 * Design notes:
 *  - Zephyr's generic `espressif,esp32-gdma` driver (dma_esp32_gdma.c)
 *    always NULL-terminates its descriptor chain - there is no cyclic mode.
 *    Continuous RGB frame streaming needs the GDMA channel to loop the
 *    bounce buffers in a ring, so this driver uses one dedicated GDMA TX
 *    channel and drives it directly via gdma_hal_*() calls,
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

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
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
#include <esp_attr.h>

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

/* Confirmed against this SoC's hal/lcd_ll.h: LCD_LL_EVENT_RGB is defined
 * there as LCD_LL_EVENT_VSYNC_END (bit 0) - LCD_LL_EVENT_TRANS_DONE (bit 1,
 * aliased as LCD_LL_EVENT_I80) is the I80/DBI-mode "transaction done" event
 * and does not fire in RGB/DPI streaming mode.
 */
#ifndef WSRGB_LL_EVENT_VSYNC
#define WSRGB_LL_EVENT_VSYNC LCD_LL_EVENT_RGB
#endif

#define WSRGB_BYTES_PER_PIXEL(cfg) ((cfg)->data_width == 8 ? 1 : 2)

/* PLL_F160M, used as the LCD module's internal pixel-clock source below. */
#define WSRGB_LCD_CLK_SRC_HZ 160000000UL
#define WSRGB_RESTART_SKIP_BYTES ((LCD_LL_FIFO_DEPTH + 1U) * 2U)
#define WSRGB_BOUNCE_ROWS 10U

static gdma_hal_context_t s_gdma_hal;
static lcd_cam_dev_t *const s_lcd_cam = &LCD_CAM;

/* Build a circular descriptor chain over both SRAM bounce buffers.
 * EOF on each buffer's last descriptor triggers its refill.
 */
static int wsrgb_build_desc_chain(const struct device *dev)
{
	struct wsrgb_data *data = dev->data;

	/* Keep normal descriptor boundaries aligned for burst transfers. */
	size_t max_chunk = DMA_DESCRIPTOR_BUFFER_MAX_SIZE_64B_ALIGNED;
	size_t per_buffer = DIV_ROUND_UP(data->bounce_size, max_chunk);
	size_t n_desc = 2U * per_buffer;

	if (data->bounce_size <= WSRGB_RESTART_SKIP_BYTES) {
		LOG_ERR("Bounce buffer too small for the RGB FIFO restart");
		return -EINVAL;
	}

	data->desc_pool_size = (n_desc + 1U) * sizeof(dma_descriptor_t);
	data->desc_pool = k_aligned_alloc(4, data->desc_pool_size);
	if (!data->desc_pool) {
		LOG_ERR("failed to allocate %u bytes for DMA descriptor pool",
			data->desc_pool_size);
		return -ENOMEM;
	}

	dma_descriptor_t *desc = (dma_descriptor_t *)data->desc_pool;
	memset(desc, 0, data->desc_pool_size);
	for (size_t buffer = 0; buffer < 2U; buffer++) {
		size_t remaining = data->bounce_size;
		uint8_t *buf = data->bounce_buf[buffer];

		for (size_t node = 0; node < per_buffer; node++) {
			size_t i = buffer * per_buffer + node;
			size_t chunk = MIN(remaining, max_chunk);
			desc[i].dw0.size = chunk;
			desc[i].dw0.length = chunk;
			desc[i].dw0.suc_eof = (node == per_buffer - 1U);
			desc[i].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
			desc[i].buffer = buf;
			desc[i].next = &desc[(i + 1U) % n_desc];
			buf += chunk;
			remaining -= chunk;
		}
		data->bounce_eof_desc[buffer] = &desc[(buffer + 1U) * per_buffer - 1U];
	}

	/*
	 * S3 retains 16 FIFO pixels plus one output pixel across VSYNC.
	 * Replaying them at restart would introduce a 17-pixel horizontal shift.
	 * Only this entry is unaligned; subsequent nodes remain 64-byte aligned.
	 */
	desc[n_desc] = desc[0];
	desc[n_desc].buffer = data->bounce_buf[0] + WSRGB_RESTART_SKIP_BYTES;
	desc[n_desc].dw0.size -= WSRGB_RESTART_SKIP_BYTES;
	desc[n_desc].dw0.length -= WSRGB_RESTART_SKIP_BYTES;
	data->restart_desc = &desc[n_desc];

	return 0;
}

/* Like ESP-IDF's bounce mode, PSRAM/cache must stay accessible during scanout. */
static void IRAM_ATTR wsrgb_fill_bounce_buffer(struct wsrgb_data *data, uint8_t buffer)
{
	memcpy(data->bounce_buf[buffer], data->fb + data->bounce_offset,
	       data->bounce_size);
	data->bounce_offset += data->bounce_size;
	if (data->bounce_offset == data->fb_size) {
		data->bounce_offset = 0U;
	}
}

static void wsrgb_free_desc_chain(const struct device *dev)
{
    struct wsrgb_data *data = dev->data;

    if (data->desc_pool != NULL) {
        k_free(data->desc_pool);
        data->desc_pool = NULL;
        data->desc_pool_size = 0U;
        data->restart_desc = NULL;
    }
}

/* Program the static LCD_CAM timing-generator registers for RGB mode. */
static int wsrgb_configure_timing(const struct device *dev)
{
	const struct wsrgb_config *cfg = dev->config;

	/* NOTE: these lcd_ll_* names mirror ESP-IDF's esp_lcd_panel_rgb.c.
	 * Confirm exact signatures against this SoC's hal/lcd_ll.h.
	 */

	/*
	 * ESP-IDF's reference RGB panel driver always resets the TX
	 * controller/RGB-YUV converter before (re)configuring it. We only
	 * ever reset the FIFO (below); add the matching controller reset so
	 * the TX state machine starts from a known state rather than
	 * whatever it was left in after power-up.
	 */
	lcd_ll_reset(s_lcd_cam);

	lcd_ll_enable_rgb_mode(s_lcd_cam, true);
	lcd_ll_enable_output_always_on(s_lcd_cam, true);
	lcd_ll_set_phase_cycles(s_lcd_cam, 0, 0, 1);

	/* lcd_ll_set_dma_read_stride() selects the DMA read unit width in
	 * bits (8 or 16), not a byte stride - passing anything else hits
	 * an abort() in the HAL.
	 */
	lcd_ll_set_dma_read_stride(s_lcd_cam, WSRGB_BYTES_PER_PIXEL(cfg) * 8);
	lcd_ll_enable_color_convert(s_lcd_cam, false);

	lcd_ll_set_horizontal_timing(s_lcd_cam, cfg->hsync_len, cfg->hback_porch,
				      cfg->width, cfg->hfront_porch);
	lcd_ll_set_vertical_timing(s_lcd_cam, cfg->vsync_len, cfg->vback_porch,
				    cfg->height, cfg->vfront_porch);

	lcd_ll_set_idle_level(s_lcd_cam, !cfg->hsync_idle_low, !cfg->vsync_idle_low,
			       cfg->de_idle_high);

	/*
	 * ESP-IDF's reference driver calls this unconditionally for every
	 * RGB panel ("RGB panel always has a front and back blank (porch
	 * region)"). Omitting it leaves lcd_misc.lcd_bk_en at its power-on
	 * default (disabled), which affects frame/line blanking in RGB
	 * streaming mode.
	 */
	lcd_ll_set_blank_cycles(s_lcd_cam, 1, 1);

	lcd_ll_enable_output_hsync_in_porch_region(s_lcd_cam, true);
	lcd_ll_set_hsync_position(s_lcd_cam, 0);
	lcd_ll_set_clock_idle_level(s_lcd_cam, false);
	lcd_ll_set_pixel_clock_edge(s_lcd_cam, cfg->pclk_active_neg);

	/*
	 * The LCD module's internal pixel clock (lcd_clk) is NOT touched by
	 * the parent espressif,esp32-lcd-cam node - that only gates the
	 * shared APB bus clock via clock_control_on(). lcd_clk itself still
	 * needs a real source selected and a divider programmed here;
	 * without it PCLK never toggles, the RGB timing generator never
	 * advances, and the panel just shows its idle/no-signal state
	 * (typically a blank white screen) regardless of framebuffer
	 * content.
	 *
	 * Use the HAL's prescaler workaround for unstable hardware at mo=1.
	 * For 16 MHz it selects 160 MHz / 5 / 2, not the bypassed / 10 / 1.
	 */
	lcd_ll_enable_clock(s_lcd_cam, true);
	lcd_ll_select_clk_src(s_lcd_cam, LCD_CLK_SRC_PLL160M);

	lcd_hal_context_t lcd_hal = { .dev = s_lcd_cam };
	hal_utils_clk_div_t divider = {0};
	uint32_t actual_pclk = lcd_hal_cal_pclk_freq(
		&lcd_hal, WSRGB_LCD_CLK_SRC_HZ, cfg->pixel_clk_hz, &divider);
	if (actual_pclk == 0U) {
		LOG_ERR("Cannot generate pixel clock: %u Hz", cfg->pixel_clk_hz);
		return -EINVAL;
	}
	lcd_ll_set_group_clock_coeff(s_lcd_cam, divider.integer,
				    divider.denominator, divider.numerator);
	LOG_INF("Actual pixel clock: %u Hz (divider=%u+%u/%u, prescale=%u)",
		actual_pclk, divider.integer, divider.numerator, divider.denominator,
		s_lcd_cam->lcd_clock.lcd_clkcnt_n + 1U);

	/*
	 * Without this, lcd_ll_start() only triggers a single frame
	 * transaction and the timing generator then halts - the HAL's own
	 * doc comment says enabling "next frame" is what makes the RGB-mode
	 * LCD controller keep sending frame data continuously. Omitting it
	 * is consistent with a panel that stays in its blank/idle state
	 * after at most one (likely incomplete) frame.
	 */
	lcd_ll_enable_auto_next_frame(s_lcd_cam, true);

	lcd_ll_fifo_reset(s_lcd_cam);
	return 0;
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
	lcd_ll_stop(s_lcd_cam);
	lcd_ll_reset(s_lcd_cam);
	lcd_ll_fifo_reset(s_lcd_cam);
	gdma_hal_reset(&s_gdma_hal, cfg->dma_channel, GDMA_CHANNEL_DIRECTION_TX);
	data->bounce_offset = 0U;
	data->bounce_expected = 0U;
	data->bounce_eof_count = 0U;
	wsrgb_fill_bounce_buffer(data, 0);
	wsrgb_fill_bounce_buffer(data, 1);

	/*
	 * Our circular descriptor chain (wsrgb_build_desc_chain()) sets each
	 * descriptor's owner bit to DMA once at setup and never flips it
	 * back afterwards - there is no CPU-side re-arming step. Disabling
	 * the owner-check (and write-back/eof-till-popped, which only
	 * matter when something consumes per-descriptor completion status)
	 * means GDMA just keeps following the "next" pointers forever
	 * instead of possibly stalling once it no longer sees descriptors
	 * marked as DMA-owned.
	 */
	gdma_hal_set_strategy(&s_gdma_hal, cfg->dma_channel, GDMA_CHANNEL_DIRECTION_TX,
			       false /* en_owner_check */,
			       false /* en_desc_write_back */,
			       false /* eof_till_popped */);

	/*
	 * Burst mode for both descriptor fetches and data reads, so the AHB
	 * side can keep up with the panel's pixel clock (16 MHz * 16-bit ==
	 * 32 MB/s sustained) without starving the LCD_CAM FIFO.
	 */
	gdma_hal_enable_burst(&s_gdma_hal, cfg->dma_channel, GDMA_CHANNEL_DIRECTION_TX,
			      true /* en_data_burst */, true /* en_desc_burst */);
	/* S3's external-memory block size is separate from the burst-enable bit. */
	gdma_ll_tx_set_burst_size(s_gdma_hal.dev, cfg->dma_channel, 64);

	gdma_hal_connect_peri(&s_gdma_hal, cfg->dma_channel, GDMA_CHANNEL_DIRECTION_TX,
			       WSRGB_GDMA_TRIG_PERIPH_LCD);
	gdma_hal_enable_intr(&s_gdma_hal, cfg->dma_channel, GDMA_CHANNEL_DIRECTION_TX,
			      GDMA_LL_TX_EVENT_MASK, false);
	gdma_ll_tx_clear_interrupt_status(s_gdma_hal.dev, cfg->dma_channel,
					 GDMA_LL_TX_EVENT_MASK);

	/* Keep DMA/FIFO priming and LCD start together, including on unblank. */
	k_spinlock_key_t key = k_spin_lock(&data->scanout_lock);
	lcd_ll_enable_interrupt(s_lcd_cam, WSRGB_LL_EVENT_VSYNC, false);
	lcd_ll_clear_interrupt_status(s_lcd_cam, WSRGB_LL_EVENT_VSYNC);
	gdma_hal_start_with_desc(&s_gdma_hal, cfg->dma_channel, GDMA_CHANNEL_DIRECTION_TX,
				  (intptr_t)data->desc_pool);
	k_busy_wait(1);
	lcd_ll_start(s_lcd_cam);
	data->streaming = true;
	gdma_ll_tx_enable_interrupt(&GDMA, cfg->dma_channel, GDMA_LL_EVENT_TX_EOF, true);
	lcd_ll_enable_interrupt(s_lcd_cam, WSRGB_LL_EVENT_VSYNC, true);
	k_spin_unlock(&data->scanout_lock, key);

	return 0;
}

static void wsrgb_stop_dma(const struct device *dev)
{
	const struct wsrgb_config *cfg = dev->config;
	struct wsrgb_data *data = dev->data;

	k_spinlock_key_t key = k_spin_lock(&data->scanout_lock);
	lcd_ll_enable_interrupt(s_lcd_cam, WSRGB_LL_EVENT_VSYNC, false);
	data->streaming = false;
	gdma_ll_tx_enable_interrupt(&GDMA, cfg->dma_channel, GDMA_LL_TX_EVENT_MASK, false);
	lcd_ll_stop(s_lcd_cam);
	gdma_hal_stop(&s_gdma_hal, cfg->dma_channel, GDMA_CHANNEL_DIRECTION_TX);
	lcd_ll_clear_interrupt_status(s_lcd_cam, WSRGB_LL_EVENT_VSYNC);
	k_spin_unlock(&data->scanout_lock, key);
}

static void IRAM_ATTR wsrgb_dma_isr(void *arg)
{
	struct wsrgb_data *data = arg;
	k_spinlock_key_t key = k_spin_lock(&data->scanout_lock);
	uint32_t status = gdma_ll_tx_get_interrupt_status(&GDMA, data->dma_channel, false);
	gdma_ll_tx_clear_interrupt_status(&GDMA, data->dma_channel, status);

	if ((status & GDMA_LL_EVENT_TX_EOF) && data->streaming) {
		uintptr_t eof = gdma_ll_tx_get_eof_desc_addr(&GDMA, data->dma_channel);
		uint8_t buffer = data->bounce_expected;
		if (eof == (uintptr_t)data->bounce_eof_desc[buffer] ||
		    (buffer == 0U && eof == (uintptr_t)data->restart_desc)) {
			wsrgb_fill_bounce_buffer(data, buffer);
			data->bounce_expected ^= 1U;
			data->bounce_eof_count++;
		}
	}
	k_spin_unlock(&data->scanout_lock, key);
}

/* Recover missed bounce-buffer deadlines at VSYNC, not every healthy frame. */
static void IRAM_ATTR wsrgb_isr(void *arg)
{
	struct wsrgb_data *data = arg;
	k_spinlock_key_t key = k_spin_lock(&data->scanout_lock);
	uint32_t status = lcd_ll_get_interrupt_status(s_lcd_cam);

	lcd_ll_clear_interrupt_status(s_lcd_cam, status);

	if ((status & WSRGB_LL_EVENT_VSYNC) && data->streaming) {
		if (data->bounce_eof_count != data->fb_size / data->bounce_size) {
			gdma_ll_tx_reset_channel(&GDMA, data->dma_channel);
			gdma_ll_tx_clear_interrupt_status(&GDMA, data->dma_channel,
							 GDMA_LL_TX_EVENT_MASK);
			data->bounce_offset = 0U;
			data->bounce_expected = 0U;
			wsrgb_fill_bounce_buffer(data, 0);
			wsrgb_fill_bounce_buffer(data, 1);
			gdma_ll_tx_set_desc_addr(&GDMA, data->dma_channel,
						(uint32_t)(uintptr_t)data->restart_desc);
			gdma_ll_tx_start(&GDMA, data->dma_channel);
			data->restart_count++;
		}
		data->bounce_eof_count = 0U;
		data->vsync_count++;
	}
	k_spin_unlock(&data->scanout_lock, key);
}

/* ---- Display API ---------------------------------------------------- */


static int wsrgb_write(const struct device *dev,
                       uint16_t x,
                       uint16_t y,
                       const struct display_buffer_descriptor *desc,
                       const void *buf)
{
    const struct wsrgb_config *cfg = dev->config;
    struct wsrgb_data *data = dev->data;

    if (desc == NULL || buf == NULL || data->fb == NULL) {
        return -EINVAL;
    }

    /* This driver currently supports RGB565 only. */
    if (cfg->data_width != 16U ||
        data->pixel_format != PIXEL_FORMAT_RGB_565) {
        return -ENOTSUP;
    }

    if (desc->width == 0U || desc->height == 0U ||
        desc->pitch < desc->width) {
        return -EINVAL;
    }

    if ((uint32_t)x + desc->width > cfg->width ||
        (uint32_t)y + desc->height > cfg->height) {
        return -EINVAL;
    }

    const size_t source_required =
        (size_t)desc->pitch * desc->height * 2U;

    if (desc->buf_size < source_required) {
        return -EINVAL;
    }

    const uint8_t *src = buf;
    uint8_t *dst = data->fb;

    for (uint16_t row = 0; row < desc->height; ++row) {
        const size_t src_off =
            (size_t)row * desc->pitch * 2U;

        const size_t dst_off =
            ((size_t)y + row) * cfg->width * 2U +
            (size_t)x * 2U;

        memcpy(dst + dst_off,
               src + src_off,
               (size_t)desc->width * 2U);
    }

    /* Keep the externally exposed PSRAM framebuffer coherent. */
    for (uint16_t row = 0; row < desc->height; ++row) {
        const size_t dst_off =
            ((size_t)y + row) * cfg->width * 2U +
            (size_t)x * 2U;

        int ret = sys_cache_data_flush_range(dst + dst_off, (size_t)desc->width * 2U);
        if (ret != 0) {
            LOG_ERR("Framebuffer cache write-back failed: %d", ret);
            return ret;
        }
    }

    if (!desc->frame_incomplete) {
        k_spinlock_key_t key = k_spin_lock(&data->scanout_lock);
        uint32_t recoveries = data->restart_count - data->reported_restart_count;
        data->reported_restart_count = data->restart_count;
        k_spin_unlock(&data->scanout_lock, key);
        if (recoveries != 0U) {
            LOG_WRN("Recovered %u missed RGB bounce-buffer deadlines", recoveries);
        }
    }

    return 0;
}


static int wsrgb_read(const struct device *dev,
                      uint16_t x,
                      uint16_t y,
                      const struct display_buffer_descriptor *desc,
                      void *buf)
{
    const struct wsrgb_config *cfg = dev->config;
    const struct wsrgb_data *data = dev->data;

    if (desc == NULL || buf == NULL || data->fb == NULL) {
        return -EINVAL;
    }

    if (desc->width == 0U || desc->height == 0U ||
        desc->pitch < desc->width) {
        return -EINVAL;
    }

    if ((uint32_t)x + desc->width > cfg->width ||
        (uint32_t)y + desc->height > cfg->height) {
        return -EINVAL;
    }

    const size_t destination_required =
        (size_t)desc->pitch * desc->height * 2U;

    if (desc->buf_size < destination_required) {
        return -EINVAL;
    }

    uint8_t *dst = buf;
    const uint8_t *src = data->fb;

    for (uint16_t row = 0; row < desc->height; ++row) {
        const size_t src_off =
            ((size_t)y + row) * cfg->width * 2U +
            (size_t)x * 2U;

        const size_t dst_off =
            (size_t)row * desc->pitch * 2U;

        memcpy(dst + dst_off,
               src + src_off,
               (size_t)desc->width * 2U);
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
		int ret = wsrgb_start_dma(dev);
		if (ret != 0) {
			return ret;
		}

		const struct wsrgb_config *cfg = dev->config;
		uint32_t count = data->vsync_count;
		uint64_t frame_us =
			(uint64_t)(cfg->width + cfg->hsync_len + cfg->hback_porch +
				   cfg->hfront_porch) *
			(cfg->height + cfg->vsync_len + cfg->vback_porch +
			 cfg->vfront_porch) * 1000000U / cfg->pixel_clk_hz;
		k_usleep(2U * frame_us + 1000U);
		if (data->vsync_count == count) {
			LOG_ERR("RGB VSYNC interrupt did not fire");
			wsrgb_stop_dma(dev);
			return -ETIMEDOUT;
		}
		LOG_INF("SRAM bounce scanout active (%u frames, %u recoveries)",
			data->vsync_count - count, data->restart_count);
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
	LOG_INF("Waveshare RGB LCD init START");

    const struct wsrgb_config *cfg = dev->config;
    struct wsrgb_data *data = dev->data;
    int ret;
    intr_handle_t vsync_handle = NULL;
    intr_handle_t dma_handle = NULL;

    /*
     * This implementation only supports 16-bit RGB565 scanout.
     */
    if (cfg->width == 0U || cfg->height == 0U ||
        cfg->data_width != 16U || cfg->pixel_clk_hz == 0U ||
        cfg->pixel_clk_hz > WSRGB_LCD_CLK_SRC_HZ / 4U) {
        LOG_ERR("Invalid RGB panel dimensions, data width or pixel clock");
        return -EINVAL;
    }

    if (cfg->pcfg == NULL) {
        return -EINVAL;
    }

    /*
     * Enable the LCD_CAM peripheral's APB bus clock and deassert its
     * reset. The parent espressif,esp32-lcd-cam devicetree node does NOT
     * do this on its own - that only happens via esp32_lcd_cam_init()
     * (soc/espressif/common/esp_lcd_cam.c), which is only auto-built for
     * the stock lcd-cam-dvp/lcd-cam-mipi-dbi sub-node compatibles. Without
     * this, the whole LCD_CAM register block stays unclocked/in-reset and
     * every lcd_ll_*() register write in wsrgb_configure_timing() below is
     * a no-op - the panel just shows its idle/no-signal state regardless
     * of anything else this driver does.
     */
    if (!device_is_ready(cfg->clock_dev)) {
        LOG_ERR("LCD_CAM clock controller not ready");
        return -ENODEV;
    }

    ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
    if (ret != 0) {
        LOG_ERR("Failed to enable LCD_CAM peripheral clock: %d", ret);
        return ret;
    }

    ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret != 0) {
        LOG_ERR("pinctrl apply failed: %d", ret);
        return ret;
    }

    data->fb_size =
        (size_t)cfg->width * (size_t)cfg->height * 2U;
    uint32_t bounce_rows = MIN(WSRGB_BOUNCE_ROWS, cfg->height);
    while (cfg->height % bounce_rows != 0U) {
        bounce_rows--;
    }
    data->bounce_size = cfg->width * bounce_rows * 2U;

    /* Required for the four-byte-aligned DMA descriptors. */
    if ((data->fb_size & 3U) != 0U) {
        return -EINVAL;
    }

    data->fb = wsrgb_fb_alloc(64U, data->fb_size);
    if (data->fb == NULL) {
        LOG_ERR("Framebuffer allocation failed: %zu bytes",
                data->fb_size);
        return -ENOMEM;
    }

    memset(data->fb, 0, data->fb_size);
    ret = sys_cache_data_flush_range(data->fb, data->fb_size);
    if (ret != 0) {
        LOG_ERR("Initial framebuffer cache write-back failed: %d", ret);
        goto fail_fb;
    }
    data->pixel_format = PIXEL_FORMAT_RGB_565;
    data->streaming = false;
    data->dma_channel = cfg->dma_channel;

    ret = wsrgb_build_desc_chain(dev);
    if (ret != 0) {
        LOG_ERR("DMA descriptor allocation failed: %d", ret);
        goto fail_fb;
    }

    /*
     * These functions still require verification against the
     * exact ESP32-S3 HAL version used by your Zephyr checkout.
     */
    ret = wsrgb_configure_timing(dev);
    if (ret != 0) {
        goto fail_desc;
    }

    /*
     * ESP32's interrupt sources are dynamically muxed to CPU interrupt
     * lines, so this uses ESP-IDF's own allocator (as the in-tree
     * mipi_dbi_esp32.c driver does for this same LCD_CAM peripheral)
     * instead of a static IRQ_CONNECT(). The intrstatus variant only
     * invokes wsrgb_isr when LCD_LL_EVENT_VSYNC is actually pending in
     * the LCD_CAM interrupt status register.
     */
    lcd_ll_enable_interrupt(s_lcd_cam, WSRGB_LL_EVENT_VSYNC, false);
    lcd_ll_clear_interrupt_status(s_lcd_cam, WSRGB_LL_EVENT_VSYNC);
    ret = esp_intr_alloc_intrstatus(
        cfg->irq_source,
        ESP_PRIO_TO_FLAGS(cfg->irq_priority) | ESP_INT_FLAGS_CHECK(cfg->irq_flags) |
            ESP_INTR_FLAG_IRAM,
        (uint32_t)lcd_ll_get_interrupt_status_reg(s_lcd_cam), WSRGB_LL_EVENT_VSYNC,
        wsrgb_isr, data, &vsync_handle);
    if (ret != 0) {
        LOG_ERR("Failed to allocate RGB LCD interrupt: %d", ret);
        goto fail_desc;
    }
    ret = esp_intr_alloc_intrstatus(
        cfg->dma_irq_source, ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM,
        (uint32_t)gdma_ll_tx_get_interrupt_status_reg(&GDMA, cfg->dma_channel),
        GDMA_LL_EVENT_TX_EOF, wsrgb_dma_isr, data, &dma_handle);
    if (ret != 0) {
        LOG_ERR("Failed to allocate bounce-buffer DMA interrupt: %d", ret);
        goto fail_desc;
    }

    ret = wsrgb_start_dma(dev);
    if (ret != 0) {
        LOG_ERR("Failed to start RGB DMA: %d", ret);
        goto fail_desc;
    }


	LOG_INF("width=%d height=%d",
			cfg->width, cfg->height);

	LOG_INF("pixel clock=%d Hz",
			cfg->pixel_clk_hz);

	LOG_INF("starting LCD CAM");

	LOG_INF("starting DMA");

	LOG_INF("Waveshare RGB LCD init END");
    return 0;

fail_desc:
    if (dma_handle != NULL) {
        esp_intr_free(dma_handle);
    }
    if (vsync_handle != NULL) {
        esp_intr_free(vsync_handle);
    }
    wsrgb_free_desc_chain(dev);

fail_fb:
    wsrgb_fb_free(data->fb);
    data->fb = NULL;
    data->fb_size = 0U;

    return ret;
}

#define WSRGB_INIT(inst)                                                                          \
	PINCTRL_DT_INST_DEFINE(inst);                                                             \
	static uint8_t wsrgb_bounce_##inst[2][ROUND_UP(                                           \
		DT_INST_PROP(inst, width) * MIN(WSRGB_BOUNCE_ROWS, DT_INST_PROP(inst, height)) * 2U, \
		64U)] __aligned(64);                                                              \
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
		/*                                                                                 \
		 * The devicetree "dmas" cell is NOT a raw physical GDMA channel        \
		 * index - Zephyr's own esp32 GDMA convention (see                     \
		 * dma_esp32_gdma.c: `channel_id = channel / 2`) packs it as           \
		 * (pair_index * 2 + direction), matching the parent lcd_cam node's    \
		 * own `dmas = <&dma 6>, <&dma 7>` (rx=6, tx=7, same pair 3 for both). \
		 * Passing the raw cell value (7) straight into the gdma_hal_*()      \
		 * calls below as chan_id indexed past the hardware's actual 5-entry  \
		 * channel[] array (soc/gdma_struct.h), so every register write       \
		 * silently missed the real DMA channel entirely - nothing ever got   \
		 * transferred to the LCD_CAM FIFO, regardless of how correct the     \
		 * timing/clock/polarity configuration was.                          \
		 */                                                                               \
		.dma_channel = DT_INST_DMAS_CELL_BY_NAME(inst, tx, channel) / 2,                  \
		.dma_irq_source = DT_IRQ_BY_IDX(DT_INST_DMAS_CTLR_BY_NAME(inst, tx),               \
			DT_INST_DMAS_CELL_BY_NAME(inst, tx, channel), irq),                       \
		.irq_source = DT_INST_IRQ(inst, irq),                                             \
		.irq_priority = DT_INST_IRQ(inst, priority),                                      \
		.irq_flags = DT_INST_IRQ(inst, flags),                                            \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                     \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                            \
		.clock_subsys =                                                                   \
			(clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, offset),               \
	};                                                                                         \
	static struct wsrgb_data wsrgb_data_##inst = {                                            \
		.bounce_buf = { wsrgb_bounce_##inst[0], wsrgb_bounce_##inst[1] },                 \
	};                                                                                      \
	DEVICE_DT_INST_DEFINE(inst, wsrgb_init, NULL, &wsrgb_data_##inst, &wsrgb_cfg_##inst,       \
			      POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY, &wsrgb_api);

DT_INST_FOREACH_STATUS_OKAY(WSRGB_INIT)
