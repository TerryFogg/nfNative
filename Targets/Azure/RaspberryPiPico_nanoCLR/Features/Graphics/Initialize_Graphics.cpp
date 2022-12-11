#include <Target_BlockStorage.h>
//
// Copyright (c) 2017 The nanoFramework project contributors
// See LICENSE file in the project root for full license information.
//

#include <nanoCLR_Headers.h>
#include "GraphicsMemoryHeap.h"
#include <nanoHAL_Graphics.h>
#include "BOARD.h"
#include "Debug_To_Display.h"

#define LCD_RST_PIN 12
#define LCD_DC_PIN 8
#define LCD_BL_PIN 13
#define LCD_CS_PIN 9

extern "C"
{
    void Initialize_Graphics()
    {
        // Initialize graphics ram heap size to all available as defined in the
        // memory map
        g_GraphicsMemoryHeap.Initialize(0);

        DisplayInterfaceConfig displayConfig;
        displayConfig.Spi.spiBus = 1; // Index into array of pin values ( spiBus - 1) == 0
        displayConfig.Spi.chipSelect = LCD_CS_PIN;
        displayConfig.Spi.chipSelectActiveLow = true;
        displayConfig.Spi.dataCommand = LCD_DC_PIN;
        displayConfig.Spi.dataCommandActiveLow = true;
        displayConfig.Spi.reset = LCD_RST_PIN;
        displayConfig.Spi.backLightActiveLow = true;
        displayConfig.Spi.backLight = LCD_BL_PIN;
        displayConfig.Spi.backLightActiveLow = true;
        g_DisplayInterface.Initialize(displayConfig);

        displayConfig.Screen.width = 240;
        displayConfig.Screen.height = 135;

        g_DisplayDriver.Initialize(displayConfig);

        // no PAL events required until now
        PalEvent_Initialize();

    }
}

