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

////*******************************
////  Wire protocol UART handlers
////*******************************
//
//void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == wpUSART) // Wire Protocol UART
//    {
//        WP_DMA_Receive_half_complete();
//    }
//}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
//{
//    if (huart->Instance == wpUSART)    // Wire Protocol UART
//    {
//        WP_DMA_Receive_complete();
//    }
//}
//
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
//{
//    if (huart->Instance == wpUSART) // Wire Protocol UART
//    {
//        WP_DMA_Transfer_complete();
//    }
// 
//}
//
////  Wire protocol transmit
////  Configured for wire protocol on USART1
//void USART1_IRQHandler(void)
//{
//    WP_USART_Interrupt();
//}

