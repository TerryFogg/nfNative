

#include "stm32h7xx_hal.h"

void UART_RX_Check(DMA_HandleTypeDef *hdma);
void UART_RX_Process(const void *data, size_t len);


#define DMA_BUFFER_MAX 16
char rx_buffer[DMA_BUFFER_MAX] = { 0 };


UART_HandleTypeDef huart1;


int main(void) {
    HAL_UART_Receive_DMA(&huart1, (uint8_t*)rx_buffer, DMA_BUFFER_MAX);
    while (1) {...}
}


/*
 DMA RX and IDLE Line detection

The good combination for using DMA to get unknown length of data is to use DMA in Circular mode, with big enough memory buffer, 
then use DMA Half-Transfer, Full-Transfer and the IDLE line detection to notify application to process received data. 
HAL DMA Receiving function automatically notifies the application by calling HAL_UART_RxHalfCpltCallback() 
and HAL_UART_RxCpltCallback(). Therefore, it is only needed to override the USART1_IRQHandler() function. 
To more simple and still left HAL Handler processes other cases, it is better to add a small code just to 
check the IDLE flag and process received data before handing over the interrupt to original HAL_UART_IRQHandler() function.
*/
void USART1_IRQHandler(void)
{
    if (((&huart1)->Instance->ISR & UART_FLAG_IDLE) != 0   && ((&huart1)->Instance->CR1 & USART_CR1_IDLEIE) != 0) 
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        HAL_UART_RxCpltCallback(&huart1);
    }
    // pass the work to HAL function
    HAL_UART_IRQHandler(&huart1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    UART_RX_Check(&hdma_usart1_rx);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
    UART_RX_Check(&hdma_usart1_rx);
}

#define UART_BUFFER_MAX   64
char uart_buffer[UART_BUFFER_MAX] = { 0 };
size_t uart_buffer_idx = 0;
char uart_new_string = 0;

void UART_RX_Process(const void *data, size_t len) {
    for (int i = 0; i < len; i++) {
        char c = ((char*) data)[i];
        if (uart_buffer_idx < UART_BUFFER_MAX - 2) {
            uart_buffer[uart_buffer_idx++] = c;
            uart_buffer[uart_buffer_idx] = '\0';
        }
        if (c == '\n') {
            uart_buffer_idx = 0;
            uart_new_string = 1;
        }
    }
}

void UART_RX_Check(DMA_HandleTypeDef *hdma) {
    static size_t old_pos = 0;
    size_t rx_pos = DMA_BUFFER_MAX - hdma->Instance->CNDTR;
    if (rx_pos != old_pos) {
         // new data
      if(rx_pos > old_pos) {
             // no overflow
        UART_RX_Process(&rx_buffer[old_pos], rx_pos - old_pos);
        } else {
             // overflow
          UART_RX_Process(&rx_buffer[old_pos], DMA_BUFFER_MAX - old_pos);
            if (rx_pos > 0) {
                 // run up
              UART_RX_Process(&rx_buffer[old_pos], rx_pos);
            }
        }
        old_pos = rx_pos;
    }
}

int mainLoop(void) {
    while (1) {
        if (uart_new_string == 1) {
            uart_new_string = 0;
            if (strncmp(uart_buffer, "stop", 4) == 0) {...}
        }
    }
}

