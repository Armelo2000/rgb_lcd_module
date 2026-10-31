cmake_minimum_required(VERSION 3.20)

project(rgb_lcd_module_tests
    VERSION 1.0
    LANGUAGES C CXX
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(rgb_lcd_tests
    src/test_main.cpp

    # Compile the module's implementation directly
    ${CMAKE_CURRENT_LIST_DIR}/../drivers/display/display_waveshare_esp32s3_rgb.c
)

target_include_directories(rgb_lcd_tests
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/..
        ${CMAKE_CURRENT_LIST_DIR}/../drivers/display
)

enable_testing()

add_test(
    NAME rgb_lcd_basic
    COMMAND rgb_lcd_tests
)