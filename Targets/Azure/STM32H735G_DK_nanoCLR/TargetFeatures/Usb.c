//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2021 STMicroelectronics.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_usb.h"
#include "TargetFeatures.h"

void USBD_Clock_Config(void);



void Initialize_USB()
{
    USBD_Clock_Config();
    //        USBD_Init(&hUsbDeviceHS, &DFU_Desc, DEVICE_HS);
    //        USBD_RegisterClass(&hUsbDeviceHS, &USBD_DFU);
    //        USBD_DFU_RegisterMedia(&hUsbDeviceHS, &USBD_DFU_Flash_fops);
    //        USBD_Start(&hUsbDeviceHS);
}

void USBD_Clock_Config(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

    // Enable HSI48
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    // Configure the clock recovery system (CRS)
    __HAL_RCC_CRS_CLK_ENABLE();                          // Enable CRS Clock
    RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;     // Default Synchro Signal division factor (not divided)
    RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB1; // Set the SYNCSRC[1:0] bits according to
                                                         // CRS_Source value

    // HSI48 is synchronized with USB SOF at 1KHz rate
    RCC_CRSInitStruct.ReloadValue = RCC_CRS_RELOADVALUE_DEFAULT;
    RCC_CRSInitStruct.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
    RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;

    // Set the TRIM[5:0] to the default value
    RCC_CRSInitStruct.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;

    HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct); // Start automatic synchronization
}
