cmake_minimum_required(VERSION 3.12)
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(PicoSDKProject1 C CXX ASM)
include(PicoSDKConfig.cmake)
pico_sdk_init()

if(DEFINED SYSPROGS_FRAMEWORKS_FILE)
include(${SYSPROGS_FRAMEWORKS_FILE})
endif()
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
add_executable(hello_gpio_irq
        hello_gpio_irq.c
        )

# pull in common dependencies
target_link_libraries(hello_gpio_irq pico_stdlib Profiler)

# create map/bin/hex file etc.
pico_add_extra_outputs(hello_gpio_irq)

# add url via pico_set_program_url
# example_auto_set_url(hello_gpio_irq)
