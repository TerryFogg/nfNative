//
// Copyright (c) 2017 The nanoFramework project contributors
// See LICENSE file in the project root for full license information.
//

#ifndef _INITIALIZE_GRAPHICS_H_
#define _INITIALIZE_GRAPHICS_H_ 1

#include <nanoCLR_Headers.h>
#include <target_board.h>
#include <nanoHAL_Graphics.h>
#include "Debug_To_Display.h"
#include "stm32h7xx_hal_gpio.h"

extern "C"
{
    void InitializeGraphics()
    {
        g_GraphicsMemoryHeap.Initialize(0);    // Initialize graphics ram heap size to all available as defined in the memory map
        
        
        
        DisplayInterfaceConfig config; 
        config.VideoDisplay.width = 480; //g_DisplayDriver.Attributes.LongerSide;
        config.VideoDisplay.height = 272; // g_DisplayDriver.Attributes.ShorterSide; 
        config.VideoDisplay.Frequency_Divider = 5;
        config.VideoDisplay.enable = GPIO_PIN_7;
        config.VideoDisplay.control = GPIO_PIN_2;
        config.VideoDisplay.backlight = GPIO_PIN_1;
        config.VideoDisplay.Horizontal_synchronization = 41;
        config.VideoDisplay.Horizontal_back_porch = 13;
        config.VideoDisplay.Horizontal_front_porch = 32;
        config.VideoDisplay.Vertical_synchronization = 10;
        config.VideoDisplay.Vertical_back_porch = 2;
        config.VideoDisplay.Vertical_front_porch = 2;
        g_DisplayInterface.Initialize(config);
        
        g_DisplayDriver.Initialize();
        
        
        TouchInterfaceConfig empty;
        g_TouchInterface.Initialize(empty);
        
        //g_TouchDevice.Initialize(TouchInterruptPin);
        //g_GestureDriver.Initialize();
        //g_InkDriver.Initialize();
        //g_TouchPanelDriver.Initialize();

        lcd_printf("\n\nGraphics Initialization: Finished\n");
    }

}

#endif //_INITIALIZE_GRAPHICS_H_
