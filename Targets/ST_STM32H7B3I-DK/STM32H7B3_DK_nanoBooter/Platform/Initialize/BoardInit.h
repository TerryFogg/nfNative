
#pragma once

#include <nanoCLR_Headers.h>
enum nanoBooterStatusTypes { ok = 0, communications_failure = 1 };
typedef enum nanoBooterStatusTypes eBooterStatus;

void BoardInit();
void LedsAndBoardInit();
void nanoBooterStatus(uint32_t nanoBooterState);

// STM32H7B3I-DK board Leds and push button
#define LED_GPIO_PORT			GPIOG
#define BUTTON_USER_GPIO_PORT	GPIOC
#define LED_BLUE				GPIO_PIN_2
#define LED_RED					GPIO_PIN_11
#define BUTTON_USER_PIN			GPIO_PIN_13

/* Definition for USARTx clock resources */
#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA2_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_10
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF7_USART1
#define USARTx_RX_PIN                    GPIO_PIN_9
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF7_USART1

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_STREAM             DMA2_Stream7
#define USARTx_RX_DMA_STREAM             DMA2_Stream1

/* Definition for USARTx's DMA Request */
#define USARTx_TX_DMA_REQUEST            DMA_REQUEST_USART1_TX
#define USARTx_RX_DMA_REQUEST            DMA_REQUEST_USART1_RX

/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn               DMA2_Stream7_IRQn
#define USARTx_DMA_RX_IRQn               DMA2_Stream1_IRQn
#define USARTx_DMA_TX_IRQHandler         DMA2_Stream7_IRQHandler
#define USARTx_DMA_RX_IRQHandler         DMA2_Stream1_IRQHandler

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART1_IRQn
#define USARTx_IRQHandler                USART1_IRQHandler