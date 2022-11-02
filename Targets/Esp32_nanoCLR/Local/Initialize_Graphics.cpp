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
#include "Board_PICO.h"
#include "Debug_To_Display.h"

extern "C"
{
    void Initialize_Graphics()
    {
        // Initialize graphics ram heap size to all available as defined in the
        // memory map
        void InitializeGraphics() {
          g_GraphicsMemoryHeap.Initialize();

#if defined(ESP32_WROVER_KIT_V41)

          DisplayInterfaceConfig displayConfig;
          displayConfig.Spi.spiBus =
              1; // Index into array of pin values ( spiBus - 1) == 0
          displayConfig.Spi.chipSelect.pin = GPIO_NUM_22;
          displayConfig.Spi.chipSelect.type.activeLow = true;
          displayConfig.Spi.dataCommand.pin = GPIO_NUM_21;
          displayConfig.Spi.dataCommand.type.commandLow = true;
          displayConfig.Spi.reset.pin = GPIO_NUM_18;
          displayConfig.Spi.reset.type.activeLow = true;
          displayConfig.Spi.backLight.pin = GPIO_NUM_5;
          displayConfig.Spi.backLight.type.activeLow = true;
          g_DisplayInterface.Initialize(displayConfig);
          g_DisplayDriver.Initialize();

          // Touch
          CLR_UINT8 TouchI2cAddress = 0x38;
          GPIO_PIN TouchInterruptPin = GPIO_NUM_34;
          i2c_port_t TouchI2cBus = I2C_NUM_0;
          int busSpeed = 0; // 10,000Khz
          nanoI2C_Init(TouchI2cBus, busSpeed);
          g_TouchInterface.Initialize(TouchI2cBus, TouchI2cAddress);
          g_TouchDevice.Initialize(TouchInterruptPin);
          g_GestureDriver.Initialize();
          g_InkDriver.Initialize();
          g_TouchPanelDriver.Initialize();

#elif defined(MAKERFAB_GRAPHICS_35)
          DisplayInterfaceConfig displayConfig;
          displayConfig.Spi.spiBus =
              2; // Index into array of pin values ( spiBus - 1) == 1
          displayConfig.Spi.chipSelect.pin = GPIO_NUM_15;
          displayConfig.Spi.chipSelect.type.activeLow = true;
          displayConfig.Spi.dataCommand.pin = GPIO_NUM_33;
          displayConfig.Spi.dataCommand.type.commandLow = true;
          displayConfig.Spi.reset.pin = IMPLEMENTED_IN_HARDWARE;
          displayConfig.Spi.reset.type.activeLow = true;
          displayConfig.Spi.backLight.pin = IMPLEMENTED_IN_HARDWARE;
          displayConfig.Spi.backLight.type.activeLow = true;
          g_DisplayInterface.Initialize(displayConfig);
          g_DisplayDriver.Initialize();

          // Touch
          CLR_UINT8 TouchI2cAddress = 0x38;
          GPIO_PIN TouchInterruptPin = GPIO_NUM_34;
          i2c_port_t TouchI2cBus = I2C_NUM_0;
          int busSpeed = 0; // 10,000Khz
          nanoI2C_Init(TouchI2cBus, busSpeed);
          g_TouchInterface.Initialize(TouchI2cBus, TouchI2cAddress);
          g_TouchDevice.Initialize(TouchInterruptPin);
          g_GestureDriver.Initialize();
          g_InkDriver.Initialize();
          g_TouchPanelDriver.Initialize();

#elif defined(MAKERFAB_MAKEPYTHON)
          DisplayInterfaceConfig displayConfig;
          displayConfig.Spi.spiBus =
              2; // Index into array of pin values  ( spiBus - 1) == 1
          displayConfig.Spi.chipSelect.pin = GPIO_NUM_15;
          displayConfig.Spi.chipSelect.type.activeLow = true;
          displayConfig.Spi.dataCommand.pin = GPIO_NUM_22;
          displayConfig.Spi.dataCommand.type.commandLow = true;
          displayConfig.Spi.reset.pin = GPIO_NUM_21;
          displayConfig.Spi.reset.type.activeLow = true;
          displayConfig.Spi.backLight.pin = GPIO_NUM_5;
          displayConfig.Spi.backLight.type.activeLow = false;
          g_DisplayInterface.Initialize(displayConfig);
          g_DisplayDriver.Initialize();
#endif

          // no PAL events required until now
          PalEvent_Initialize();

          // Start Network Debugger
          // SOCKETS_DbgInitialize( 0 );
        }
    }
}

#endif //_INITIALIZE_GRAPHICS_H_



