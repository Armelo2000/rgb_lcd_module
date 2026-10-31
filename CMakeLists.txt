# SPDX-License-Identifier: Apache-2.0
# Out-of-tree Zephyr module: experimental ESP32-S3 LCD_CAM RGB display driver.

if(COMMAND zephyr_library)
    add_subdirectory(drivers/display)
else()
    cmake_minimum_required(VERSION 3.20)

    if(NOT DEFINED BOARD)
        set(BOARD esp32s3_devkitc/esp32s3/procpu)
    endif()
    set(CONF_FILE "${CMAKE_CURRENT_SOURCE_DIR}/tests/zephyr/prj.conf")
    set(KCONFIG_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/tests/zephyr/Kconfig")
    if(NOT DEFINED DTC_OVERLAY_FILE)
        set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/tests/zephyr/app.overlay")
    endif()
    list(APPEND ZEPHYR_EXTRA_MODULES "${CMAKE_CURRENT_SOURCE_DIR}")
    list(REMOVE_DUPLICATES ZEPHYR_EXTRA_MODULES)

    if(DEFINED ZEPHYR_BASE)
        get_filename_component(ZEPHYR_BASE "${ZEPHYR_BASE}" REALPATH)
    endif()
    find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
    project(rgb_lcd_module_build_check LANGUAGES C)
    target_sources(app PRIVATE tests/zephyr/src/main.c)
endif()
