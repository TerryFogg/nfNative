

#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_hal.h"


#include <stm32h7xx.h>
#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_def.h>
#include <stm32h7xx_hal_uart.h>

#include "lwrb.h"

// For this specific example, all variables are by default configured in D1 RAM. This is configured in linker script
uint8_t usart_rx_dma_buffer[64];
lwrb_t  usart_rx_rb;                // Ring buffer instance for RX data
uint8_t usart_rx_rb_data[128];      //Ring buffer data array for RX DMA
lwrb_t   usart_tx_rb;               // Ring buffer instance for TX data
uint8_t usart_tx_rb_data[128];      // Ring buffer data array for TX DMA

volatile size_t usart_tx_dma_current_len;                    // Length of currently active TX DMA transfer





/**
 * \brief           USART1 Initialization Function
 */
void usart_init(void) 
{
    LL_USART_InitTypeDef USART_InitStruct = { 0 };
    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    
    /* Configure USART1 */
    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    LL_USART_Init(USART1, &USART_InitStruct);
    LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_7_8);
    LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_7_8);
    LL_USART_EnableFIFO(USART1);
    LL_USART_ConfigAsyncMode(USART1);
    LL_USART_EnableDMAReq_RX(USART1);
    LL_USART_EnableDMAReq_TX(USART1);
    LL_USART_EnableIT_IDLE(USART1);
    

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

    /*
     * USART1 GPIO Configuration
     *
     * PA10   ------> USART1_TX
     * PA9    ------> USART1_RX
     */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART1_RX Init */
    LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_1, LL_DMAMUX1_REQ_USART1_RX);
    LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_1, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_1);
    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_1, LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_RECEIVE));
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_1, (uint32_t)usart_rx_dma_buffer);
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_1, ARRAY_LEN(usart_rx_dma_buffer));

    /* USART1_TX Init */
    LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_7, LL_DMAMUX1_REQ_USART1_TX);
    LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_7, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_7, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_7, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_7);
    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_7, LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT));

    /* Enable DMA RX HT & TC interrupts */
    LL_DMA_EnableIT_HT(DMA2, LL_DMA_STREAM_1);
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_1);
    /* Enable DMA TX TC interrupts */
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_7);

    /* DMA interrupt init */
    NVIC_SetPriority(DMA2_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    NVIC_SetPriority(DMA2_Stream7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);


    /* USART interrupt, same priority as DMA channel */
    NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(USART1_IRQn);

    /* Enable USART and DMA RX */
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
    LL_USART_Enable(USART1);

    /* Polling USART1 initialization */
    while (!LL_USART_IsActiveFlag_TEACK(USART1) || !LL_USART_IsActiveFlag_REACK(USART1)) {}
}



/**
 * \brief           Check for new data received with DMA
 *
 * User must select context to call this function from:
 * - Only interrupts (DMA HT, DMA TC, UART IDLE) with same preemption priority level
 * - Only thread context (outside interrupts)
 *
 * If called from both context-es, exclusive access protection must be implemented
 * This mode is not advised as it usually means architecture design problems
 *
 * When IDLE interrupt is not present, application must rely only on thread context,
 * by manually calling function as quickly as possible, to make sure data are read from raw buffer and processed.
 *
 * Not doing reads fast enough may cause DMA to overflow unread received bytes, hence application will lost useful data.
 *
 * Solutions to this are:
 * - Improve architecture design to achieve faster reads
 * - Increase raw buffer size and allow DMA to write more data before this function is called
 */
void usart_rx_check(void)
{
    static size_t OldPositionInBuffer;
    size_t currentPositionInBuffer;
    
    currentPositionInBuffer = ARRAY_LEN(usart_rx_dma_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_0);
    if (currentPositionInBuffer != OldPositionInBuffer) 
    {
        /* Check change in received data */
        if (currentPositionInBuffer > OldPositionInBuffer)   // Single Block of data ( current - old)
            {
                usart_process_data(&usart_rx_dma_buffer[OldPositionInBuffer], currentPositionInBuffer - OldPositionInBuffer);
            }
        else {
            // Two blocks of data ( end-old and current- start)
usart_process_data(&usart_rx_dma_buffer[OldPositionInBuffer], ARRAY_LEN(usart_rx_dma_buffer) - OldPositionInBuffer);
            if (currentPositionInBuffer > 0) {
                usart_process_data(&usart_rx_dma_buffer[0], currentPositionInBuffer);
            }
        }
        OldPositionInBuffer = currentPositionInBuffer;   // Save current position as old for next transfers
    }
}

/**
 * \brief           Check if DMA is active and if not try to send data
 *
 * This function can be called either by application to start data transfer
 * or from DMA TX interrupt after previous transfer just finished
 *
 * \return          `1` if transfer just started, `0` if on-going or no data to transmit
 */
uint8_t usart_start_tx_dma_transfer(void) 
{
    uint32_t primask;
    uint8_t started = 0;

    /*
     * First check if transfer is currently in-active,
     * by examining the value of usart_tx_dma_current_len variable.
     *
     * This variable is set before DMA transfer is started and cleared in DMA TX complete interrupt.
     *
     * It is not necessary to disable the interrupts before checking the variable:
     *
     * When usart_tx_dma_current_len == 0
     *    - This function is called by either application or TX DMA interrupt
     *    - When called from interrupt, it was just reset before the call,
     *         indicating transfer just completed and ready for more
     *    - When called from an application, transfer was previously already in-active
     *         and immediate call from interrupt cannot happen at this moment
     *
     * When usart_tx_dma_current_len != 0
     *    - This function is called only by an application.
     *    - It will never be called from interrupt with usart_tx_dma_current_len != 0 condition
     *
     * Disabling interrupts before checking for next transfer is advised
     * only if multiple operating system threads can access to this function w/o
     * exclusive access protection (mutex) configured,
	 * or if application calls this function from multiple interrupts.
	 *
	 * This example assumes worst use case scenario, hence interrupts are disabled prior every check
     */
    primask = __get_PRIMASK();
    __disable_irq();
    if (usart_tx_dma_current_len == 0
            && (usart_tx_dma_current_len = lwrb_get_linear_block_read_length(&usart_tx_rb)) > 0) 
    {
        /* Disable channel if enabled */
        LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);

        /* Clear all flags */
        LL_DMA_ClearFlag_TC1(DMA1);
        LL_DMA_ClearFlag_HT1(DMA1);
        LL_DMA_ClearFlag_TE1(DMA1);
        LL_DMA_ClearFlag_DME1(DMA1);
        LL_DMA_ClearFlag_FE1(DMA1);

        /* Prepare DMA data and length */
        LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, usart_tx_dma_current_len);
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)lwrb_get_linear_block_read_address(&usart_tx_rb));

        /* Start transfer */
        LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
        started = 1;
    }
    __set_PRIMASK(primask);
    return started;
}

/**
 * \brief           Process received data over UART
 * Data are written to RX ringbuffer for application processing at latter stage
 * \param[in]       data: Data to process
 * \param[in]       len: Length in units of bytes
 */
void usart_process_data(const void* data, size_t len) 
{
    lwrb_write(&usart_rx_rb, data, len); /* Write data to receive buffer */
}

/**
 * \brief           Send string over USART
 * \param[in]       str: String to send
 */
void usart_send_string(const char* str) 
{
    lwrb_write(&usart_tx_rb, str, strlen(str)); /* Write data to transmit buffer */
    usart_start_tx_dma_transfer();
}


/* Interrupt handlers here */

/**
 * \brief           DMA2 stream1 interrupt handler for USART1 RX
 */
void DMA2_Stream1_IRQHandler(void)
{
    /* Check half-transfer complete interrupt */
    if (LL_DMA_IsEnabledIT_HT(DMA2, LL_DMA_STREAM_1) && LL_DMA_IsActiveFlag_HT0(DMA2)) 
    {
        LL_DMA_ClearFlag_HT0(DMA2); /* Clear half-transfer complete flag */
        usart_rx_check(); /* Check for data to process */
    }

    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA2, LL_DMA_STREAM_1) && LL_DMA_IsActiveFlag_TC0(DMA2)) 
    {
        LL_DMA_ClearFlag_TC0(DMA2); /* Clear transfer complete flag */
        usart_rx_check(); /* Check for data to process */
    }

    /* Implement other events when needed */
}

/**
 * \brief           DMA1 stream1 interrupt handler for USART1 TX
 */
void DMA2_Stream7_IRQHandler(void)
{
    /* Check transfer complete */
    if (LL_DMA_IsEnabledIT_TC(DMA2, LL_DMA_STREAM_7) && LL_DMA_IsActiveFlag_TC1(DMA2)) 
    {
        LL_DMA_ClearFlag_TC1(DMA2); /* Clear transfer complete flag */
        lwrb_skip(&usart_tx_rb, usart_tx_dma_current_len); /* Skip sent data, mark as read */
        usart_tx_dma_current_len = 0; /* Clear length variable */
        usart_start_tx_dma_transfer(); /* Start sending more data */
    }

    /* Implement other events when needed */
}

/**
 * \brief           USART1 global interrupt handler
 */
void USART1_IRQHandler(void) {
    /* Check for IDLE line interrupt */
    if (LL_USART_IsEnabledIT_IDLE(USART1) && LL_USART_IsActiveFlag_IDLE(USART1))
    {
        LL_USART_ClearFlag_IDLE(USART1); /* Clear IDLE line flag */
        usart_rx_check(); /* Check for data to process */
    }

    /* Implement other events when needed */
}

/**
 * \brief           System Clock Configuration
 */
void SystemClock_Config(void) {
    /* Configure flash latency */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    if (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4) {
        while (1) {}
    }

    /* Configure power supply and voltage scale */
    LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE0);

    /* Uncomment if used on STM32H745/H755 Nucleo */
    /* Dual-Core Nucleo board used external SMPS instead of LDO */
    /* Manually enable it */
    //PWR->CR3 |= 1 << 2;

    /* Configure HSE */
    LL_RCC_HSE_EnableBypass();
    LL_RCC_HSE_Enable();
    while (!LL_RCC_HSE_IsReady()) {}

    /* Configure PLL */
    LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSE);
    LL_RCC_PLL1P_Enable();
    LL_RCC_PLL1Q_Enable();
    LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
    LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
    LL_RCC_PLL1_SetM(1);
    LL_RCC_PLL1_SetN(120);
    LL_RCC_PLL1_SetP(2);
    LL_RCC_PLL1_SetQ(20);
    LL_RCC_PLL1_SetR(2);
    LL_RCC_PLL1_Enable();
    while (!LL_RCC_PLL1_IsReady()) {}

    /* Intermediate AHB prescaler 2 when target frequency clock is higher than 80 MHz */
    LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);
    LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
    LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_2);
    LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_2);

    /* Configure systick */
    LL_Init1msTick(480000000);
    LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
    LL_SetSystemCoreClock(480000000);
    LL_RCC_SetUSARTClockSource(LL_RCC_USART234578_CLKSOURCE_PCLK1);
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++





main(void) {
    uint8_t state, cmd, len;

    /* MCU Configuration */
    //    SCB_DisableDCache();
    //    SCB_DisableICache();

        /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
        LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SYSCFG);

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize ringbuff for TX & RX */
    lwrb_init(&usart_tx_rb, usart_tx_rb_data, sizeof(usart_tx_rb_data));
    lwrb_init(&usart_rx_rb, usart_rx_rb_data, sizeof(usart_rx_rb_data));

    /* Initialize all configured peripherals */
    usart_init();
    usart_send_string("USART DMA example: DMA HT & TC + USART IDLE LINE interrupts\r\n");
    usart_send_string("Start sending data to STM32\r\n");

    /* After this point, do not use usart_send_string function anymore */
    /* Send packet data over UART from PC (or other STM32 device) */

    /* Infinite loop */
    state = 0;
    while (1) {
        uint8_t b;

        /* Process RX ringbuffer */

        /* Packet format: START_BYTE, CMD, LEN[, DATA[0], DATA[len - 1]], STOP BYTE */
        /* DATA bytes are included only if LEN > 0 */
        /* An example, send sequence of these bytes: 0x55, 0x01, 0x01, 0xFF, 0xAA */

        /* Read byte by byte */

        if (lwrb_read(&usart_rx_rb, &b, 1) == 1) {
            lwrb_write(&usart_tx_rb, &b, 1); /* Write data to transmit buffer */
            usart_start_tx_dma_transfer();
            switch (state) {
            case 0: {
                    /* Wait for start byte */
                    if (b == 0x55) {
                        ++state;
                    }
                    break;
                }
            case 1: {
                    /* Check packet command */
                    cmd = b;
                    ++state;
                    break;
                }
            case 2: {
                    /* Packet data length */
                    len = b;
                    ++state;
                    if (len == 0) {
                        ++state; /* Ignore data part if len = 0 */
                    }
                    break;
                }
            case 3: {
                    /* Data for command */
                    --len; /* Decrease for received character */
                    if (len == 0) {
                        ++state;
                    }
                    break;
                }
            case 4: {
                    /* End of packet */
                    if (b == 0xAA) {
                        /* Packet is valid */

                        /* Send out response with CMD = 0xFF */
                        b = 0x55; /* Start byte */
                        lwrb_write(&usart_tx_rb, &b, 1);
                        cmd = 0xFF; /* Command = 0xFF = OK response */
                        lwrb_write(&usart_tx_rb, &cmd, 1);
                        b = 0x00; /* Len = 0 */
                        lwrb_write(&usart_tx_rb, &b, 1);
                        b = 0xAA; /* Stop byte */
                        lwrb_write(&usart_tx_rb, &b, 1);

                        /* Flush everything */
                        usart_start_tx_dma_transfer();
                    }
                    state = 0;
                    break;
                }
            }
        }

        /* Do other tasks ... */
    }
}


