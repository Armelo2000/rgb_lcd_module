#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

BUILD_ASSERT(IS_ENABLED(CONFIG_DISPLAY_WAVESHARE_ESP32S3_RGB),
	     "The build check must compile the RGB display driver");
BUILD_ASSERT(DT_HAS_COMPAT_STATUS_OKAY(waveshare_esp32s3_lcd_cam_rgb),
	     "The build check requires an enabled RGB display node");

int main(void)
{
	const struct device *display = DEVICE_DT_GET(DT_NODELABEL(rgb_compile_display));

	if (device_is_ready(display)) {
		printk("ERROR: the compile-only display was initialized unexpectedly.\n");
		return 1;
	}

	printk("RGB LCD module compile/link check; display initialization is deferred.\n");
	return 0;
}
