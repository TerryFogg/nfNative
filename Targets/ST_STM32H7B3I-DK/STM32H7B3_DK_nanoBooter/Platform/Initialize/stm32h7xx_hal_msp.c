//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include <stm32h7xx_hal.h>
#include <tx_api.h>
#include <BoardInit.h>
#include <targetHAL.h>

extern UART_HandleTypeDef wpUartHandle;
static DMA_HandleTypeDef hdma_tx;
static DMA_HandleTypeDef hdma_rx;

void HAL_MspInit(void)
{
    // Add User Code here
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    // Add user interrupt information here
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    if (huart->Instance == USART1)
    {
        // Enable the clock for the selected USART
        wpUSART_CLK_ENABLE();

        // NOTE: The following setup allows of USART RX/TX pins on different ports
        // Samples quite often ( OR pins) when pins on same port
        // UART TX GPIO pin configuration and clock
        wpUSART_TX_GPIO_CLK_ENABLE();
        GPIO_InitStruct.Pin = wpUSART_TX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = wpUSART_TX_AF;
        HAL_GPIO_Init(wpUSART_TX_GPIO_PORT, &GPIO_InitStruct);

        // UART RX GPIO pin configuration and clock
        wpUSART_RX_GPIO_CLK_ENABLE();
        GPIO_InitStruct.Pin = wpUSART_RX_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = wpUSART_RX_AF;
        HAL_GPIO_Init(wpUSART_RX_GPIO_PORT, &GPIO_InitStruct);

        //----------------------------------------------
        // Setup the DMA to support USART communications
        //----------------------------------------------

        // Setup the DMA to use peripheral to memory mode for the USART
        wpDMA_CLK_ENABLE();

        // Receive DMA configuration
        hdma_rx.Instance = wpUSART_RX_DMA_STREAM;
        hdma_rx.Init.Request = wpUSART_RX_DMA_REQUEST;
        hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_rx.Init.Mode = DMA_NORMAL;
        hdma_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_rx))
        {
            HAL_AssertEx();
        }
        __HAL_LINKDMA(&wpUartHandle, hdmarx, hdma_rx); // Associate the DMA handle to the the USART
        //++++++++++++++++++++++++++++++++++++++++++++

        // Transmit DMA configuration
        hdma_tx.Instance = wpUSART_TX_DMA_STREAM;
        hdma_tx.Init.Request = wpUSART_TX_DMA_REQUEST;
        hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_tx.Init.Mode = DMA_NORMAL;
        hdma_tx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_tx))
        {
            HAL_AssertEx();
        }
        __HAL_LINKDMA(&wpUartHandle, hdmatx, hdma_tx); // Associate the DMA handle to the the USART
        //++++++++++++++++++++++++++++++++++++++++++++

        //------------------------------------------
        // Setup NVIC configuration for DMA transfer
        //------------------------------------------

        //NVIC configuration for DMA transfer complete interrupt (USART1_RX)
        HAL_NVIC_SetPriority(wpUSART_DMA_RX_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(wpUSART_DMA_RX_IRQn);
        //+++++++++++++++++++++++++++++++++++++++++

        // NVIC configuration for DMA transfer complete interrupt (USART1_TX)
        HAL_NVIC_SetPriority(wpUSART_DMA_TX_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(wpUSART_DMA_TX_IRQn);
        //+++++++++++++++++++++++++++++++++++++++++

        // NVIC for USART interrupt
        HAL_NVIC_SetPriority(wpUSART_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(wpUSART_IRQn);
    }
}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART3)
    {
        /* USER CODE BEGIN USART3_MspDeInit 0 */

        /* USER CODE END USART3_MspDeInit 0 */
          /* Peripheral clock disable */
        __HAL_RCC_USART3_CLK_DISABLE();

        /**USART3 GPIO Configuration
        PD8     ------> USART3_TX
        PD9     ------> USART3_RX
        */
        HAL_GPIO_DeInit(GPIOD, wpUSART_RX_PIN | wpUSART_TX_PIN);

        /* USART3 DMA DeInit */
        HAL_DMA_DeInit(huart->hdmarx);

        /* USART3 interrupt DeInit */
        HAL_NVIC_DisableIRQ(USART3_IRQn);
        /* USER CODE BEGIN USART3_MspDeInit 1 */

        /* USER CODE END USART3_MspDeInit 1 */
    }

}


#ifdef HAL_RTC_MODULE_ENABLED
void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{
    (void)hrtc;

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    // Configue LSE as RTC clock source
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        while (1)
        {
        }
    }
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        while (1)
        {
        }
    }

    // Enable RTC Clock
    __HAL_RCC_RTC_ENABLE();
}
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
{
    (void)hrtc;

    // Reset peripherals
    __HAL_RCC_RTC_DISABLE();
}

#endif // HAL_RTC_MODULE_ENABLED
void HAL_CRC_MspInit(CRC_HandleTypeDef* hcrc)
{
    (void)hcrc;
    /* CRC Peripheral clock enable */
    __HAL_RCC_CRC_CLK_ENABLE();
}
void HAL_CRC_MspDeInit(CRC_HandleTypeDef* hcrc)
{
    (void)hcrc;
    /* Enable CRC reset state */
    __HAL_RCC_CRC_FORCE_RESET();

    /* Release CRC from reset state */
    __HAL_RCC_CRC_RELEASE_RESET();
}
#ifdef HAL_RNG_MODULE_ENABLED
void HAL_RNG_MspInit(RNG_HandleTypeDef* hrng)
{
    if (hrng->Instance == RNG)
    {
        // Peripheral clock enable
        __HAL_RCC_RNG_CLK_ENABLE();
    }
}

void HAL_RNG_MspDeInit(RNG_HandleTypeDef* hrng)
{
    if (hrng->Instance == RNG)
    {
        // Peripheral clock disable
        __HAL_RCC_RNG_CLK_DISABLE();
    }
}

#endif // HAL_RNG_MODULE_ENABLED
uint32_t HAL_GetTick(void)
{
    return tx_time_get() * 10;
}
