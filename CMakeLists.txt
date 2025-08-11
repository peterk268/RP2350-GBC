# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD solderparty_rp2350_stamp_xl CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
# We also need PICO EXTRAS
set(PICO_EXTRAS_FETCH_FROM_GIT 1)
include(pico_extras_import.cmake)


project(RP2350_GBC C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(ext/FatFs_SPI)
add_subdirectory(ext/lvgl)

# Add executable. Default name is the project name, version 0.1

add_executable(RP2350_GBC 
            main.c 
            mk_ili9225.c 
            ext/minigb_apu/minigb_apu.c 
            ext/i2s/i2s.c
)

pico_set_program_name(RP2350_GBC "RP2350_GBC")
pico_set_program_version(RP2350_GBC "0.1")

# Generate PIO header
pico_generate_pio_header(RP2350_GBC ${CMAKE_CURRENT_LIST_DIR}/ext/i2s/audio_i2s.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(RP2350_GBC 1)
pico_enable_stdio_usb(RP2350_GBC 1)

target_include_directories(RP2350_GBC PRIVATE inc ext/minigb_apu ext/i2s)


# Add the standard library to the build
target_link_libraries(RP2350_GBC
        FatFs_SPI
        lvgl
        pico_stdlib pico_stdio pico_bootrom pico_multicore pico_stdio  pico_multicore
        hardware_clocks hardware_pio hardware_vreg hardware_pio hardware_pwm
        hardware_sync hardware_pll hardware_spi hardware_irq hardware_dma hardware_flash hardware_i2c pico_aon_timer hardware_powman
        hardware_sleep hardware_adc
        pico_binary_info)

target_compile_definitions(RP2350_GBC PRIVATE
        PARAM_ASSERTIONS_DISABLE_ALL=1
        PICO_ENTER_USB_BOOT_ON_EXIT=1
        PICO_STDIO_ENABLE_CRLF_SUPPORT=0
        PICO_STDIO_DEFAULT_CRLF=0
        PICO_PRINTF_SUPPORT_FLOAT=1
        PICO_PRINTF_SUPPORT_EXPONENTIAL=0
        PICO_PRINTF_SUPPORT_LONG_LONG=1
        PICO_PRINTF_SUPPORT_PTRDIFF_T=0)

function(pico_add_verbose_dis_output TARGET)
    add_custom_command(TARGET ${TARGET} POST_BUILD
        COMMAND ${CMAKE_OBJDUMP} -h $<TARGET_FILE:${TARGET}> >$<IF:$<BOOL:$<TARGET_PROPERTY:${TARGET},OUTPUT_NAME>>,$<TARGET_PROPERTY:${TARGET},OUTPUT_NAME>,$<TARGET_PROPERTY:${TARGET},NAME>>.dis
        COMMAND ${CMAKE_OBJDUMP} -drwCSl $<TARGET_FILE:${TARGET}> >>$<IF:$<BOOL:$<TARGET_PROPERTY:${TARGET},OUTPUT_NAME>>,$<TARGET_PROPERTY:${TARGET},OUTPUT_NAME>,$<TARGET_PROPERTY:${TARGET},NAME>>.dis
        )
endfunction()
    
pico_add_extra_outputs(RP2350_GBC)

