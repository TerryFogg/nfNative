//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//
#include <nanoHAL_v2.h>
#include <wpUSART_Communications.h>
#include "WireProtocol_Message.h"

#include <stm32h7xx_Usart.h>

#include <stm32h7xx.h>
#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_def.h>
#include <stm32h7xx_hal_uart.h>
#include <BoardInit.h>

/*
Board: STM32H7B3I-DK

   _STM32H7B3I-DK_    _STLINK_3E_________      ___PC____ 
  |   ___________ |  |_______            |    |         |
  |  |     USART  |  | USART |           |    |         |
  |  | (PA10) TX-------RX    |           |    | Virtual |
  |  |            |  |       | usb_dev_hs|<==>| Com     |
  |  |  (PA9) RX-------TX    |           |    | Port    |
  |  |____________|  |_______|           |    |         |
  |_______________|  |___________________|    |_________|
                       
*/

UART_HandleTypeDef wpUartHandle;
ReadPacketState CurrentReadPacketState;
WritePacketState CurrentWritePacketState;

// Place buffers used in DMA transfers in memory regions accessible by the DMA hardware.
uint8_t aTxBuffer[TXBUFFERSIZE]  __attribute__((section(".DMA2_RAM"))) __attribute__((aligned(4)));
uint8_t aRxBuffer[RXBUFFER] __attribute__((section(".DMA2_RAM"))) __attribute__((aligned(4)));

struct PacketsReceived
{
    long CurrentIndex;
    long ProcessedIndex;
    long NumberNotProcessed;
    long TotalPackets;
    struct {
        int None;
        int Parity_error;
        int Noise_error;
        int Frame_error;
        int Overrun_error;
        int DMA_transfer_error;
        int Receiver_Timeout_error;
    } CommsErrors;
    struct {
        uint32_t Sequence;
        bool Processed;
        ReadPacketState State;
        uint8_t Data[RXBUFFER];
    } Packets[NUMBER_PACKETS];
} CircularBuffer;

    
static DMA_HandleTypeDef hdma_tx;
static DMA_HandleTypeDef hdma_rx;


bool InitWireProtocolCommunications()
{

    // Setup the USART - Clocks, Pins, Mode
    //-------------------------------------
       
    USART_RESET(USART1);
    USART_DISABLE(USART1);

    wpUartHandle.Init.BaudRate = 115200;
    USART_8_BIT_WORD_LENGTH(USART1);
    USART_STOP_1_BITS(USART1);
    USART_PARITY_NONE(USART1);
    USART_RTS_DISABLE(USART1);
    USART_CTS_DISABLE(USART1);

    USART_THREE_SAMPLE_BIT(USART1);
    USART_OVERSAMPLING_16(USART1);

    USART_TXFIFO_THESHOLD_INTERRUPT_DISABLE(USART1);
    USART_RXFIFO_THESHOLD_INTERRUPT_DISABLE(USART1);
    USART_TXFIFO_THRESHOLD_1_8(USART1);
    USART_RXFIFO_THRESHOLD_1_8(USART1);

    USART_RECEIVER_ENABLE(USART1);
    USART_TRANSMITTER_ENABLE(USART1);
    
    //In asynchronous mode, the following bits must be kept cleared
    //-------------------------------------------------------------
    LIN_MODE_DISABLE(USART1);
    USART_SCLK_DISABLE(USART1); 
    SMARTCARD_MODE_DISABLE(USART1);
    USART_NORMAL_DUPLEX(USART1);
    IRDA_MODE_DISABLE(USART1);
    
    // Setup the physical GPIO ports for the USART
    // -------------------------------------------
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // UART TX GPIO pin configuration and clock
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // UART RX GPIO pin configuration and clock
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    //----------------------------------------------
    // Setup the DMA to support USART communications
    //----------------------------------------------

    // Setup the DMA to use peripheral to memory mode for the USART
     __HAL_RCC_DMA2_CLK_ENABLE();

    // Receive DMA configuration
    hdma_rx.Instance = DMA2_Stream1;
    hdma_rx.Init.Request = DMA_REQUEST_USART1_RX;
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
    __HAL_LINKDMA(&wpUartHandle, hdmarx, hdma_rx);       // Associate the DMA handle to the the USART
    //++++++++++++++++++++++++++++++++++++++++++++

    // Transmit DMA configuration
    hdma_tx.Instance = DMA2_Stream7;
    hdma_tx.Init.Request = DMA_REQUEST_USART1_TX;
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
    __HAL_LINKDMA(&wpUartHandle, hdmatx, hdma_tx);       // Associate the DMA handle to the the USART
    //++++++++++++++++++++++++++++++++++++++++++++

    //------------------------------------------
    // Setup NVIC configuration for DMA transfer
    //------------------------------------------

    //NVIC configuration for DMA transfer complete interrupt (USART1_RX)
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    //+++++++++++++++++++++++++++++++++++++++++

    // NVIC configuration for DMA transfer complete interrupt (USART1_TX)
    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    //+++++++++++++++++++++++++++++++++++++++++

    // NVIC for USART interrupt
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++
    


    RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;

    
    
    
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    USARTx_TX_GPIO_CLK_ENABLE();
    USARTx_RX_GPIO_CLK_ENABLE();

    
    
    // Select SysClk as source of USART1 clocks
// TODO: Check what these 3 code line do and if correct
    /* Select SysClk as source of USART1 clocks */

    RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit = { 0 };
    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    RCC_PeriphClkInit.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

    /* Enable USARTx clock */
    USARTx_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/
    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = USARTx_TX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = USARTx_TX_AF;

    HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = USARTx_RX_PIN;
    GPIO_InitStruct.Alternate = USARTx_RX_AF;

    HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
    
    
    
    //------------------------------------    
  
    
    // Set up the prescaler clock and main clock for the USART
    /*  
      RCC_USART1CLKSOURCE_D2PCLK2 : APB2 Clock selected as USART1 clock
      RCC_USART1CLKSOURCE_PLL2    : PLL2_Q Clock selected as USART1 clock
      RCC_USART1CLKSOURCE_PLL3    : PLL3_Q Clock selected as USART1 clock
      RCC_USART1CLKSOURCE_HSI     : HSI selected as USART1 clock
      RCC_USART1CLKSOURCE_CSI     : CSI Clock selected as USART1 clock
      RCC_USART1CLKSOURCE_LSE     : LSE selected as USART1 clock   
      */ 
    
          UART_CLOCKSOURCE_D2PCLK2
          UART_CLOCKSOURCE_PLL2
          UART_CLOCKSOURCE_PLL3
          UART_CLOCKSOURCE_HSI
          UART_CLOCKSOURCE_CSI
          UART_CLOCKSOURCE_LSE
          ///    


        
        
              MODIFY_REG(wpUSART->PRESC, USART_PRESC_PRESCALER, UART_PRESCALER_DIV1);
 
    
    usartdiv = (uint16_t)(UART_DIV_SAMPLING16(pclk, huart->Init.BaudRate, huart->Init.ClockPrescaler));
    huart->Instance->BRR = usartdiv;    
        
    //-----------------------

    
    // Clear ISR function pointers
    wpUSART->RxISR = NULL;
    wpUSART->TxISR = NULL;   
        
    
    
    
    


    
    USART_ENABLE(wpUSART);
    

    // Check if Transmitter and Receiver are in Idle state
    bool TransmitterReady = CHECK_BIT(USART1->CR1, USART_CR1_TE_Pos);
    bool ReceiverReady = CHECK_BIT(USART1->CR1, USART_CR1_RE_Pos);
 

    
   
    
    
    
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
     /* 
    USART Auto baud rate detection using MODE 3 : 0x55 character for determining the baud rate.
 
    In all ABR modes, the baud rate is measured several times during the synchronization data reception, and is compared each time to the previous measurement.

    Important:
    - Prior to activating the auto baud rate detection, the USART_BRR register must be initialized by writing a non-zero baud rate value. 
    - The automatic baud rate detection is activated by setting the ABREN bit in the USART_CR2 register.
    - The USART then waits for the first character on the RX line. 

    The auto baud rate operation completion is indicated by the setting of the ABRF flag in the USART_ISR register. 

    If the line is noisy, the correct baud rate detection cannot be guaranteed. 
    In this case the BRR value may be corrupted and the ABRE error flag is set. 
    This also happens if the communication speed is not compatible with the automatic baud rate detection range.


    The auto baud rate detection can be re-launched later by resetting the ABRF flag (by writing 
    a ‘0’).
    When FIFO management is disabled and an auto baud rate error occurs, the ABRE flag is set through RXNE and FE bits.
    When FIFO management is enabled and an auto baud rate error occurs, the ABRE flag is set through RXFNE and FE bits.
    If the FIFO mode is enabled, the auto baud rate detection should be made using the data on the first RXFIFO location. 
    So, prior to launching the auto baud rate detection, make sure that the RXFIFO is empty by checking the RXFNE flag in USART_ISR register. 

    Note: The BRR value might be corrupted if the USART is disabled (UE = 0) during an auto baud 
    rate operation.

    */

    

 
    /*##-2- Configure the AutoBaudRate method */
    UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_AUTOBAUDRATE_INIT;
    UartHandle.AdvancedInit.AutoBaudRateEnable = 
    UART_ADVFEATURE_AUTOBAUDRATE_ENABLE;
    /*Uncomment your appropriate mode */
    //UartHandle.AdvancedInit.AutoBaudRateMode = 
    UART_ADVFEATURE_AUTOBAUDRATE_ONSTARTBIT; 
    //UartHandle.AdvancedInit.AutoBaudRateMode = 
    UART_ADVFEATURE_AUTOBAUDRATE_ONFALLINGEDGE;
    //UartHandle.AdvancedInit.AutoBaudRateMode = 
    UART_ADVFEATURE_AUTOBAUDRATE_ON0X7FFRAME;
    //UartHandle.AdvancedInit.AutoBaudRateMode = 
    UART_ADVFEATURE_AUTOBAUDRATE_ON0X55FRAME; 
    if (HAL_UART_Init(&UartHandle) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }
    /* Wait until Receive enable acknowledge flag is set */
    while (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_REACK) == RESET)
    {} 
    /* Wait until Transmit enable acknowledge flag is set */ 
    while (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_TEACK) == RESET)
    {}
    /* Loop until the end of Autobaudrate phase */
    while (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_ABRF) == RESET)
    {
    }
    
    /* 
 USART Auto baud rate detection using MODE 3 : 0x55 character for determining the baud rate.
 
In all ABR modes, the baud rate is measured several times during the synchronization data reception, and is compared each time to the previous measurement.

Important:
- Prior to activating the auto baud rate detection, the USART_BRR register must be initialized by writing a non-zero baud rate value. 
- The automatic baud rate detection is activated by setting the ABREN bit in the USART_CR2 register.
- The USART then waits for the first character on the RX line. 

The auto baud rate operation completion is indicated by the setting of the ABRF flag in the USART_ISR register. 

If the line is noisy, the correct baud rate detection cannot be guaranteed. 
In this case the BRR value may be corrupted and the ABRE error flag is set. 
This also happens if the communication speed is not compatible with the automatic baud rate detection range.


The auto baud rate detection can be re-launched later by resetting the ABRF flag (by writing 
a ‘0’).
When FIFO management is disabled and an auto baud rate error occurs, the ABRE flag is set through RXNE and FE bits.
When FIFO management is enabled and an auto baud rate error occurs, the ABRE flag is set through RXFNE and FE bits.
If the FIFO mode is enabled, the auto baud rate detection should be made using the data on the first RXFIFO location. 
So, prior to launching the auto baud rate detection, make sure that the RXFIFO is empty by checking the RXFNE flag in USART_ISR register. 

Note: The BRR value might be corrupted if the USART is disabled (UE = 0) during an auto baud 
rate operation.

*/

    

 
/*##-2- Configure the AutoBaudRate method */
    UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_AUTOBAUDRATE_INIT;
    UartHandle.AdvancedInit.AutoBaudRateEnable = 
    UART_ADVFEATURE_AUTOBAUDRATE_ENABLE;
    /*Uncomment your appropriate mode */
    //UartHandle.AdvancedInit.AutoBaudRateMode = 
    UART_ADVFEATURE_AUTOBAUDRATE_ONSTARTBIT; 
    //UartHandle.AdvancedInit.AutoBaudRateMode = 
    UART_ADVFEATURE_AUTOBAUDRATE_ONFALLINGEDGE;
    //UartHandle.AdvancedInit.AutoBaudRateMode = 
    UART_ADVFEATURE_AUTOBAUDRATE_ON0X7FFRAME;
    //UartHandle.AdvancedInit.AutoBaudRateMode = 
    UART_ADVFEATURE_AUTOBAUDRATE_ON0X55FRAME; 
    if (HAL_UART_Init(&UartHandle) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }
    /* Wait until Receive enable acknowledge flag is set */
    while (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_REACK) == RESET)
    {} 
    /* Wait until Transmit enable acknowledge flag is set */ 
    while (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_TEACK) == RESET)
    {}
    /* Loop until the end of Autobaudrate phase */
    while (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_ABRF) == RESET)
    {
    }
    
    //    Once the whole initialization is complete, the USART waits until data is received from the 
    //HyperTerminal before launching the automatic baud rate detection phase.The end of this 
    //phase is monitored by the ABRF flag.
    //• If the auto baud rate operation is unsuccessful, the ABRE flag is set 
    //• If the auto baud rate operation is completed successfully, an acknowledgment data is 
    //transmitted to the HyperTerminal.
 
    /* If AutoBaudBate error occurred */
    if(__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_ABRE) != RESET)
    {
        Error_Handler();
    }
    else
    {
        /* Wait until RXNE flag is set */
        while (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE) == RESET)
        {} 
        /* Send acknowledgement message*/
        if (HAL_UART_Transmit_DMA(&UartHandle, (uint8_t *)aTxBuffer, TXBUFFERSIZE) 
        != HAL_OK)
        {
            /* Transfer error in transmission process */
            Error_Handler();
        }
        while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY)
        {
        }
    }

        
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    
    
    
    
    
    
    
    
    
      ///////////////////-0--------------------------------
      if(wpUartHandle.AdvancedInit.AdvFeatureInit != UART_ADVFEATURE_NO_INIT)
    {
        UART_AdvFeatureConfig(huart);
    }

    /* In asynchronous mode, the following bits must be kept cleared:
    - LINEN and CLKEN bits in the USART_CR2 register,
    - SCEN, HDSEL and IREN  bits in the USART_CR3 register.*/
    CLEAR_BIT(wpUartHandle.Instance->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
    CLEAR_BIT(wpUartHandle.Instance->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));

    __HAL_UART_ENABLE(huart);

    /* TEACK and/or REACK to check before moving wpUartHandle.gState and wpUartHandle.RxState to Ready */
    return (UART_CheckIdleState(huart));
        
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    // ?????????????????
    if(HAL_UARTEx_SetTxFifoThreshold(&wpUartHandle, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        HAL_AssertEx();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&wpUartHandle, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        HAL_AssertEx();
    }
    if (HAL_UARTEx_DisableFifoMode(&wpUartHandle) != HAL_OK)
    {
        HAL_AssertEx();
    }
    // ?????????????????????

    return true;
}
bool WritePacket(uint8_t* ptr, uint16_t size)
{
    memcpy(aTxBuffer, ptr, size);

    if (HAL_UART_Transmit_DMA(&wpUartHandle, (uint8_t*)aTxBuffer, size) != HAL_OK)
    {
        HAL_AssertEx();
    }
}
bool ReadNextPacket(uint8_t* ptr, uint16_t* size)
{
    if (wpUartHandle.RxState == HAL_UART_STATE_READY)
    {
        if (HAL_UART_Receive_DMA(&wpUartHandle, aRxBuffer, sizeof(aRxBuffer)) != HAL_OK)
        {
            HAL_AssertEx();
        }
    }

    for (int i = 0; i < 10; i++)
    {
        if (CircularBuffer.NumberNotProcessed > 0)
        {
            memcpy(ptr, CircularBuffer.Packets[CircularBuffer.ProcessedIndex].Data, sizeof(aRxBuffer));
            *size = 0;
            CircularBuffer.NumberNotProcessed--;
            CircularBuffer.ProcessedIndex++;
            if (CircularBuffer.ProcessedIndex >= NUMBER_PACKETS)
            {
                CircularBuffer.ProcessedIndex = 0;
            }
            return true;
        }
        else
        {
            tx_thread_sleep(25);
        }
    }
    return false;
}
void ReadNextComplete(UART_HandleTypeDef* UartHandle)
{
    SCB_InvalidateDCache_by_Addr((uint32_t *)aRxBuffer, sizeof(aRxBuffer));                 // Tricky one

    CircularBuffer.TotalPackets++;
    CircularBuffer.NumberNotProcessed++;
    CircularBuffer.Packets[CircularBuffer.CurrentIndex].State = ReadFull;
    CircularBuffer.Packets[CircularBuffer.CurrentIndex].Processed = false;
    CircularBuffer.Packets[CircularBuffer.CurrentIndex].Sequence = CircularBuffer.TotalPackets;

    memcpy(CircularBuffer.Packets[CircularBuffer.CurrentIndex].Data, aRxBuffer, sizeof(aRxBuffer));

    CircularBuffer.CurrentIndex++;
    if (CircularBuffer.CurrentIndex >= NUMBER_PACKETS)
    {
        CircularBuffer.CurrentIndex = 0;
    }

}


