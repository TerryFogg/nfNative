//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include <stm32h7xx_hal.h>
#include <BoardInit.h>
#include <wpUSART_Communications.h>
#include <tx_api.h>

extern UART_HandleTypeDef wpUartHandle;
TX_EVENT_FLAGS_GROUP wpUartEvent;

//****************************************************
//                                                   *
//  Cortex-M7 Processor Exceptions Handlers          *              
//                                                   *
//****************************************************
//                                                   *
// NMI_Handler       : See hard_fault_handler.c      *
// HardFault_Handler : See hard_fault_handler.c      *
// MemManage_Handler : See hard_fault_handler.c      *
// BusFault_Handler  : See hard_fault_handler.c      *
// PendSV_Handler    : See tx_thread_schedule.S      *
// SysTick_Handler   : See tx_initialize_low_level.S *
// DebugMon_Handler  : See startup_stm32h7b3xxq.s    *
// SVC_Handler       : See startup_stm32h7b3xxq.s    *
//                                                   *
//****************************************************

//********************************************
//  STM32H7B3x Peripheral interrupt handlers
//********************************************

// This function handles external lines 15 to 10 interrupt request.
void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(BUTTON_USER_PIN);
}

//*******************************
//  Wire protocol UART handlers
//*******************************
//  Wire protocol receive
void DMA2_Stream1_IRQHandler(void)
{
    // DMA2_Stream1 -- is linked to USART
    HAL_DMA_IRQHandler(wpUartHandle.hdmarx);
}
//  Wire protocol transmit
void DMA2_Stream7_IRQHandler(void)
{
    HAL_DMA_IRQHandler(wpUartHandle.hdmatx);
}


//void USART1_IRQHandler(void)
//{
//    if (((&wpUartHandle)->Instance->ISR & UART_FLAG_IDLE) != 0   && ((&wpUartHandle)->Instance->CR1 & USART_CR1_IDLEIE) != 0) 
//    {
//        __HAL_UART_CLEAR_IDLEFLAG(&wpUartHandle);
//        HAL_UART_RxCpltCallback(&wpUartHandle);
//    }
//    // pass the work to HAL function
//    HAL_UART_IRQHandler(&wpUartHandle);
//}
//
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//    UART_RX_Check(&hdma_usart1_rx);
//}
//
//void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
//    UART_RX_Check(&hdma_usart1_rx);
//}






void HAL_UART_RxCpltCallback(UART_HandleTypeDef* UartHandle)
{
    if (UartHandle->Instance == wpUSART)
    {
        ReadNextComplete(UartHandle);
    }
}
//  Configured for wire protocol on USART1
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&wpUartHandle);
}

