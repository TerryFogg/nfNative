//
// Copyright (c) 2017 The nanoFramework project contributors
// See LICENSE file in the project root for full license information.
//

#ifndef _TARGET_CHIBIOS_NANOCLR_H_
#define _TARGET_CHIBIOS_NANOCLR_H_

// enable SPIFFS
#define NF_FEATURE_USE_SPIFFS FALSE

// set preference to enable (or not) the RTC subsystem
#define HAL_USE_RTC TRUE

// takes care of enabling the HAL subsystems required for API options

// enable SPI
#define HAL_USE_SPI FALSE

// enable I2C
#define HAL_USE_I2C FALSE

// enable PWM
#define HAL_USE_PWM FALSE

// enable ADC
#define HAL_USE_ADC FALSE

// enable DAC
#define HAL_USE_DAC FALSE

// enable UART subsystem
#define HAL_USE_UART TRUE

// set when networking is enabled
#define HAL_USE_MAC FALSE

// set when filesystem is enabled
#define HAL_USE_SDC FALSE

// set when watchod is enabled
#define HAL_USE_WDG TRUE

// enable CAN
#define HAL_USE_CAN FALSE

// enable CRC32 module
#define HAL_NF_USE_STM32_CRC        TRUE

// enable STM32 ONEWIRE (from nf overlay)
#define HAL_NF_USE_STM32_ONEWIRE FALSE

// enable USB MSD (from ChibiOS Contrib)
 #define HAL_USBH_USE_MSD     FALSE

#endif /* _TARGET_CHIBIOS_NANOCLR_H_ */
