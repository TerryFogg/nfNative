@echo off
REM Run this file to build the project outside of the IDE.
REM WARNING: if using a different machine, copy the .rsp files together with this script.
echo family.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/lib/tinyusb/hw/bsp/rp2040/family.gcc.rsp" || exit 1
echo cdc_device.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/lib/tinyusb/src/class/cdc/cdc_device.gcc.rsp" || exit 1
echo vendor_device.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/lib/tinyusb/src/class/vendor/vendor_device.gcc.rsp" || exit 1
echo tusb_fifo.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/lib/tinyusb/src/common/tusb_fifo.gcc.rsp" || exit 1
echo usbd.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/lib/tinyusb/src/device/usbd.gcc.rsp" || exit 1
echo usbd_control.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/lib/tinyusb/src/device/usbd_control.gcc.rsp" || exit 1
echo dcd_rp2040.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/lib/tinyusb/src/portable/raspberrypi/rp2040/dcd_rp2040.gcc.rsp" || exit 1
echo rp2040_usb.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/lib/tinyusb/src/portable/raspberrypi/rp2040/rp2040_usb.gcc.rsp" || exit 1
echo tusb.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/lib/tinyusb/src/tusb.gcc.rsp" || exit 1
echo critical_section.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/common/pico_sync/critical_section.gcc.rsp" || exit 1
echo lock_core.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/common/pico_sync/lock_core.gcc.rsp" || exit 1
echo mutex.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/common/pico_sync/mutex.gcc.rsp" || exit 1
echo sem.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/common/pico_sync/sem.gcc.rsp" || exit 1
echo time.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/common/pico_time/time.gcc.rsp" || exit 1
echo timeout_helper.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/common/pico_time/timeout_helper.gcc.rsp" || exit 1
echo datetime.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/common/pico_util/datetime.gcc.rsp" || exit 1
echo pheap.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/common/pico_util/pheap.gcc.rsp" || exit 1
echo queue.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/common/pico_util/queue.gcc.rsp" || exit 1
echo claim.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_claim/claim.gcc.rsp" || exit 1
echo clocks.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_clocks/clocks.gcc.rsp" || exit 1
echo flash.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_flash/flash.gcc.rsp" || exit 1
echo gpio.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_gpio/gpio.gcc.rsp" || exit 1
echo irq.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_irq/irq.gcc.rsp" || exit 1
echo pio.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_pio/pio.gcc.rsp" || exit 1
echo pll.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_pll/pll.gcc.rsp" || exit 1
echo sync.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_sync/sync.gcc.rsp" || exit 1
echo timer.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_timer/timer.gcc.rsp" || exit 1
echo uart.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_uart/uart.gcc.rsp" || exit 1
echo vreg.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_vreg/vreg.gcc.rsp" || exit 1
echo watchdog.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_watchdog/watchdog.gcc.rsp" || exit 1
echo xosc.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_xosc/xosc.gcc.rsp" || exit 1
echo bootrom.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_bootrom/bootrom.gcc.rsp" || exit 1
echo double_init_rom.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_double/double_init_rom.gcc.rsp" || exit 1
echo double_math.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_double/double_math.gcc.rsp" || exit 1
echo float_init_rom.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_float/float_init_rom.gcc.rsp" || exit 1
echo float_math.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_float/float_math.gcc.rsp" || exit 1
echo pico_malloc.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_malloc/pico_malloc.gcc.rsp" || exit 1
echo platform.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_platform/platform.gcc.rsp" || exit 1
echo printf.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_printf/printf.gcc.rsp" || exit 1
echo runtime.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_runtime/runtime.gcc.rsp" || exit 1
echo binary_info.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_standard_link/binary_info.gcc.rsp" || exit 1
echo new_delete.cpp
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_standard_link/new_delete.gcc.rsp" || exit 1
echo stdio.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_stdio/stdio.gcc.rsp" || exit 1
echo stdio_uart.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_stdio_uart/stdio_uart.gcc.rsp" || exit 1
echo stdlib.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_stdlib/stdlib.gcc.rsp" || exit 1
echo unique_id.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_unique_id/unique_id.gcc.rsp" || exit 1
echo cdc_uart.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/Local/src/cdc_uart.gcc.rsp" || exit 1
echo get_serial.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/Local/src/get_serial.gcc.rsp" || exit 1
echo led.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/Local/src/led.gcc.rsp" || exit 1
echo main.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/Local/src/main.gcc.rsp" || exit 1
echo probe.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/Local/src/probe.gcc.rsp" || exit 1
echo usb_descriptors.c
c:\sysgcc\arm-eabi\bin\arm-none-eabi-gcc.exe @"VisualGDB/Debug/Local/src/usb_descriptors.gcc.rsp" || exit 1
echo divider.S
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_divider/divider.gcc.rsp" || exit 1
echo irq_handler_chain.S
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/hardware_irq/irq_handler_chain.gcc.rsp" || exit 1
echo bit_ops_aeabi.S
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_bit_ops/bit_ops_aeabi.gcc.rsp" || exit 1
echo divider.S
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_divider/divider.gcc.rsp" || exit 1
echo double_aeabi.S
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_double/double_aeabi.gcc.rsp" || exit 1
echo double_v1_rom_shim.S
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_double/double_v1_rom_shim.gcc.rsp" || exit 1
echo float_aeabi.S
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_float/float_aeabi.gcc.rsp" || exit 1
echo float_v1_rom_shim.S
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_float/float_v1_rom_shim.gcc.rsp" || exit 1
echo pico_int64_ops_aeabi.S
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_int64_ops/pico_int64_ops_aeabi.gcc.rsp" || exit 1
echo mem_ops_aeabi.S
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_mem_ops/mem_ops_aeabi.gcc.rsp" || exit 1
echo crt0.S
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/_2_/ExternalRepositories/pico-sdk/src/rp2_common/pico_standard_link/crt0.gcc.rsp" || exit 1
echo bs2_default_padded_checksummed.s
c:\sysgcc\arm-eabi\bin\arm-none-eabi-g++.exe @"VisualGDB/Debug/bs2_default_padded_checksummed.gcc.rsp" || exit 1
