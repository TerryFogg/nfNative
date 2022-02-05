//
// Copyright (c) 2017 The nanoFramework project contributors
// See LICENSE file in the project root for full license information.
//

#ifndef _INITIALIZE_GRAPHICS_H_
#define _INITIALIZE_GRAPHICS_H_ 1

#include <nanoCLR_Headers.h>
#include "target_board.h"
#include <nanoHAL_Graphics.h>
#include "Debug_To_Display.h"
#include "BoardInit.h"

extern "C"
{
    void InitializeGraphics()
    {
        g_GraphicsMemoryHeap.Initialize(0);       // Initialize graphics ram heap size to all available as defined in the memory map

        DisplayInterfaceConfig display_config; 
        display_config.VideoDisplay.width = 480;    //g_DisplayDriver.Attributes.LongerSide;
        display_config.VideoDisplay.height = 272;    // g_DisplayDriver.Attributes.ShorterSide; 
        display_config.VideoDisplay.Frequency_Divider = 5;
        display_config.VideoDisplay.enable = GPIO_PIN_13;
        display_config.VideoDisplay.control = GPIO_PIN_10;
        display_config.VideoDisplay.backlight = GPIO_PIN_15;
        display_config.VideoDisplay.Horizontal_synchronization = 41;
        display_config.VideoDisplay.Horizontal_back_porch = 13;
        display_config.VideoDisplay.Horizontal_front_porch = 32;
        display_config.VideoDisplay.Vertical_synchronization = 10;
        display_config.VideoDisplay.Vertical_back_porch = 2;
        display_config.VideoDisplay.Vertical_front_porch = 2;
        g_DisplayInterface.Initialize(display_config);
        g_DisplayDriver.Initialize(480,272);

        TouchInterfaceConfig touch_config;
        touch_config.i2c_touch_screen_bus_initialize = NULL;
        touch_config.Address = 0x0070;
        touch_config.I2c_bus_number =  0;
        g_TouchInterface.Initialize(touch_config);
        g_TouchDevice.Initialize(TS_INT_PIN);
        g_TouchPanelDriver.Initialize();
        
        
//        CLR_INT16 x1 = 0;
//        CLR_INT16 y1 = 0;
//        CLR_INT16 x2 = 0;
//        CLR_INT16 y2 = 0;
//        Gesture gesture =  Gesture::NoGesture;

        
//        while (1)
//        {
//            CLR_INT32 numberOfTouchPoints = g_TouchDevice.GetPoint(&x1, &y1, &x2, &y2);
//            bool ContactInProgress = (numberOfTouchPoints >= 1);
//            if (ContactInProgress)
//            {
//                lcd_printf("[%d,%d]\n", x1, y1);
//            }
//        }

        
    }
}

#endif //_INITIALIZE_GRAPHICS_H_

