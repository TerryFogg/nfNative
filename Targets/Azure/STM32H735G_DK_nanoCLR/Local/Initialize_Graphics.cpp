#include <Target_BlockStorage_STM32FlashDriver.h>
//
// Copyright (c) 2017 The nanoFramework project contributors
// See LICENSE file in the project root for full license information.
//

#ifndef _INITIALIZE_GRAPHICS_H_
#define _INITIALIZE_GRAPHICS_H_ 1

#include <nanoCLR_Headers.h>
#include "GraphicsMemoryHeap.h"
#include <nanoHAL_Graphics.h>
#include "Board_STM32H735G-DK.h"

extern "C"
{
    void Initialize_Graphics()
    {
        g_GraphicsMemoryHeap.Initialize(
            0); // Initialize graphics ram heap size to all available as defined in the memory map

        DisplayInterfaceConfig display_config;
        display_config.VideoDisplay.width = 480;  // g_DisplayDriver.Attributes.LongerSide;
        display_config.VideoDisplay.height = 272; // g_DisplayDriver.Attributes.ShorterSide;
        display_config.VideoDisplay.Frequency_Divider = 5;
        display_config.VideoDisplay.enable = LL_GPIO_PIN_13;
        display_config.VideoDisplay.control = LL_GPIO_PIN_10;
        display_config.VideoDisplay.backlight = LL_GPIO_PIN_15;
        display_config.VideoDisplay.Horizontal_synchronization = 41;
        display_config.VideoDisplay.Horizontal_back_porch = 13;
        display_config.VideoDisplay.Horizontal_front_porch = 32;
        display_config.VideoDisplay.Vertical_synchronization = 10;
        display_config.VideoDisplay.Vertical_back_porch = 2;
        display_config.VideoDisplay.Vertical_front_porch = 2;
        g_DisplayInterface.Initialize(display_config);
        g_DisplayDriver.Initialize(480, 272);

        TouchInterfaceConfig touch_config;
        touch_config.i2c_touch_screen_bus_initialize = NULL;
        touch_config.Address = 0x0070;
        touch_config.I2c_bus_number = 0;

        g_TouchInterface.Initialize(touch_config);
        g_TouchDevice.Initialize();
        g_TouchPanelDriver.Initialize();


        // Test Flash Write
        //        uint32_t startAddress = 0x080A0000U + 27;
        //        uint32_t flashWords = 30;
        //        uint32_t flashWordSize = 32;
        //        uint32_t lengthInBytes = flashWords * flashWordSize;
        //
        //        uint32_t endAddress = startAddress + lengthInBytes;
        //        uint8_t buffer[lengthInBytes];
        //
        //        for (int i = 0; i < lengthInBytes; i++)
        //        {
        //            buffer[i] = i;
        //        }
        //
        //        bool result = EmbeddedFlashWrite(startAddress, lengthInBytes, buffer);
    }
}

#endif //_INITIALIZE_GRAPHICS_H_
