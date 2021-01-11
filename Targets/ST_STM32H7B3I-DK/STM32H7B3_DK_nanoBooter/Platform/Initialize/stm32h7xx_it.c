//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2017 STMicroelectronics.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include <stm32h7xx_hal.h>
#include <BoardInit.h>

#include <tx_api.h>

extern UART_HandleTypeDef WProtocolUart;
extern DMA_HandleTypeDef s_DMAHandle;
extern TX_EVENT_FLAGS_GROUP wpUartEvent;

extern UART_HandleTypeDef UartHandle;

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

///////////////////////////////////
// defined in hard fault handler //
// void NMI_Handler(void)        //
///////////////////////////////////

///////////////////////////////////
// defined in hard fault handler //
// void HardFault_Handler(void)  //
///////////////////////////////////

///////////////////////////////////
// defined in hard fault handler //
// void MemManage_Handler(void)  //
///////////////////////////////////

///////////////////////////////////
// defined in hard fault handler //
// void BusFault_Handler(void)   //
///////////////////////////////////

///////////////////////////////////
// defined in hard fault handler //
// void UsageFault_Handler(void) //
///////////////////////////////////

///////////////////////////////////
// defined in hard fault handler //
// void DebugMon_Handler(void)   //
///////////////////////////////////

//////////////////////////////////
// defined in ThreadX low level //
// void PendSV_Handler(void)    //
//////////////////////////////////

//////////////////////////////////
// defined in ThreadX low level //
// void SysTick_Handler(void)   //
//////////////////////////////////

/**
  * @brief  This function handles external lines 15 to 10 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
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


// ----

/*
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&WProtocolUart);
}

void DMA1_Channel4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(WProtocolUart.hdmatx);
}

void DMA1_Channel5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(WProtocolUart.hdmarx);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    if (UartHandle->Instance == USART1)
    {
        // use event flags group as a variable to transmit the amount of transmitted  bytes
        tx_event_flags_set(&wpUartEvent, UartHandle->TxXferSize - UartHandle->TxXferCount, TX_OR);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    if (UartHandle->Instance == USART1)
    {
        // use event flags group as a variable to transmit the amount of received bytes
        tx_event_flags_set(&wpUartEvent, UartHandle->RxXferSize - UartHandle->RxXferCount, TX_OR);
    }
}
*/
