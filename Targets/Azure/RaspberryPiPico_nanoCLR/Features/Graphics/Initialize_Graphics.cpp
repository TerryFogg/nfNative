#include <Target_BlockStorage.h>
#include <hardware/spi.h>
//
// Copyright (c) 2017 The nanoFramework project contributors
// See LICENSE file in the project root for full license information.
//

#include <nanoCLR_Headers.h>
#include "GraphicsMemoryHeap.h"
#include <nanoHAL_Graphics.h>
#include "BOARD.h"
#include "Debug_To_Display.h"


extern "C"
{
    void Initialize_Graphics()
    {
        // Initialize graphics ram heap size to all available as defined in the
        // memory map
        g_GraphicsMemoryHeap.Initialize(0);

        DisplayInterfaceConfig displayConfig;
        
#ifdef PICO_LCD_114    
        displayConfig.Spi.spiBus = 1; // Index into array of pin values ( spiBus - 1) == 0
        displayConfig.Spi.clock = 10;
        displayConfig.Spi.MOSI = 11;
        displayConfig.Spi.chipSelect = 9;
        displayConfig.Spi.chipSelectActiveLow = true;
        displayConfig.Spi.dataCommand = 8;
        displayConfig.Spi.dataCommandActiveLow = true;
        displayConfig.Spi.reset = 12;
        displayConfig.Spi.backLightActiveLow = true;
        displayConfig.Spi.backLight = 13;
        displayConfig.Spi.backLightActiveLow = true;
        displayConfig.Screen.width = 240;
        displayConfig.Screen.height = 135;
#endif

#ifdef ROUND_DISPLAY
        displayConfig.Spi.clock = 10;
        displayConfig.Spi.MOSI = 11;
        displayConfig.Spi.chipSelect = 9;
        displayConfig.Spi.chipSelectActiveLow = true;
        displayConfig.Spi.dataCommand = 8;
        displayConfig.Spi.dataCommandActiveLow = true;
        displayConfig.Spi.reset = 12;
        displayConfig.Spi.backLightActiveLow = true;
        displayConfig.Spi.backLight = 25;
        displayConfig.Spi.backLightActiveLow = true;
        displayConfig.Screen.width = 240;
        displayConfig.Screen.height = 240;
#endif
        
        g_DisplayInterface.Initialize(displayConfig);
        g_DisplayDriver.Initialize(displayConfig);

        //        for (int i = 0; i < 20; i++)
//        {
//            lcd_printf("Hello there how are you going today\r\n");
//        }
    }
}

