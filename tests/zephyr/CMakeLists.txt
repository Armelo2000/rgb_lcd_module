# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20)

get_filename_component(rgb_lcd_module_root "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
list(APPEND ZEPHYR_EXTRA_MODULES "${rgb_lcd_module_root}")
list(REMOVE_DUPLICATES ZEPHYR_EXTRA_MODULES)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rgb_lcd_module_build_check LANGUAGES C)
target_sources(app PRIVATE src/main.c)
