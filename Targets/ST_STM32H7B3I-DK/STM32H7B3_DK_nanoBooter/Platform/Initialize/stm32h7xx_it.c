//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2017 STMicroelectronics.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include <stm32h7xx_hal.h>
#include <BoardInit.h>

#include <tx_api.h>

extern UART_HandleTypeDef UartHandle;

extern DMA_HandleTypeDef s_DMAHandle;
extern TX_EVENT_FLAGS_GROUP wpUartEvent;

/******************************************************************************/
/*            Cortex-M7 Processor Exceptions Handlers                         */
/******************************************************************************/

// NMI_Handler       : See hard_fault_handler.c
// HardFault_Handler : See hard_fault_handler.c
// MemManage_Handler : See hard_fault_handler.c
// BusFault_Handler  : See hard_fault_handler.c
// PendSV_Handler    : See tx_thread_schedule.S
// SysTick_Handler   : See tx_initialize_low_level.S
// DebugMon_Handler  : See startup_stm32h7b3xxq.s
// SVC_Handler       : See startup_stm32h7b3xxq.s

/**************************************************/
/*      STM32H7B3x Peripheral interrupt handlers  */
/**************************************************/

void EXTI15_10_IRQHandler(void)             // This function handles external lines 15 to 10 interrupt request.
{
    HAL_GPIO_EXTI_IRQHandler(BUTTON_USER_PIN);
}

void DMA2_Stream1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(UartHandle.hdmarx);
}

/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA
  *         used for USART data reception
  */
void DMA2_Stream7_IRQHandler(void)
{
    HAL_DMA_IRQHandler(UartHandle.hdmatx);
}

/**
  * @brief  This function handles UART interrupt request.
  * @param  None
  * @retval None
  * @Note   This function relates to the DMA used for USART data transmission
  */

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&UartHandle);
}

