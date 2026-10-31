# SPDX-License-Identifier: Apache-2.0

if(NOT COMMAND zephyr_library)
    message(FATAL_ERROR
        "Configure the rgb_lcd_module root directory, not drivers/display. "
        "The driver requires Zephyr's build environment."
    )
endif()

zephyr_library()
zephyr_library_sources_ifdef(
    CONFIG_DISPLAY_WAVESHARE_ESP32S3_RGB
    display_waveshare_esp32s3_rgb.c
)

if(CONFIG_DISPLAY_WAVESHARE_ESP32S3_RGB)
    # Zephyr's S3 HAL build otherwise includes these only for DBI and I2S.
    if(NOT CONFIG_MIPI_DBI_ESP32)
        zephyr_library_sources(
            ${ZEPHYR_HAL_ESPRESSIF_MODULE_DIR}/components/esp_hal_lcd/lcd_hal.c
        )
    endif()
    if(NOT CONFIG_I2S_ESP32)
        zephyr_library_sources(
            ${ZEPHYR_HAL_ESPRESSIF_MODULE_DIR}/components/hal/hal_utils.c
        )
    endif()
endif()
