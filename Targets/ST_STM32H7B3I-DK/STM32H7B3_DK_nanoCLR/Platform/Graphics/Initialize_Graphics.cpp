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

extern "C"
{
    void InitializeGraphics()
    {
        g_GraphicsMemoryHeap.Initialize(1000000);
        DisplayInterfaceConfig config; // not used for LTDC Display
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
