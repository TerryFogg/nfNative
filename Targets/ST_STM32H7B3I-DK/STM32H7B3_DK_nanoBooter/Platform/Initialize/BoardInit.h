
#pragma once

#include <nanoCLR_Headers.h>

enum nanoBooterStatusTypes { ok = 0, communications_failure = 1 };
typedef enum nanoBooterStatusTypes eBooterStatus;

void BoardInit();
void LedsAndBoardInit();
void nanoBooterStatus(uint32_t nanoBooterState);
void Startup_Rtos();

eBooterStatus nanoBooterState;

// STM32H7B3I-DK board Leds and push button
#define LED_GPIO_PORT			GPIOG
#define BUTTON_USER_GPIO_PORT	GPIOC
#define LED_BLUE				GPIO_PIN_2
#define LED_RED					GPIO_PIN_11
#define BUTTON_USER_PIN			GPIO_PIN_13

/* Definition for USARTwp clock resources */
#define USARTwp                           USART1
#define USARTwp_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA2_CLK_ENABLE()
#define USARTwp_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTwp_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTwp_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USARTwp_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

/* Definition for USARTwp Pins */
#define USARTwp_TX_PIN                    GPIO_PIN_10
#define USARTwp_TX_GPIO_PORT              GPIOA
#define USARTwp_TX_AF                     GPIO_AF7_USART1
#define USARTwp_RX_PIN                    GPIO_PIN_9
#define USARTwp_RX_GPIO_PORT              GPIOA
#define USARTwp_RX_AF                     GPIO_AF7_USART1

/* Definition for USARTwp's DMA */
#define USARTwp_TX_DMA_STREAM             DMA2_Stream7
#define USARTwp_RX_DMA_STREAM             DMA2_Stream1

/* Definition for USARTwp's DMA Request */
#define USARTwp_TX_DMA_REQUEST            DMA_REQUEST_USART1_TX
#define USARTwp_RX_DMA_REQUEST            DMA_REQUEST_USART1_RX

/* Definition for USARTwp's NVIC */
#define USARTwp_DMA_TX_IRQn               DMA2_Stream7_IRQn
#define USARTwp_DMA_RX_IRQn               DMA2_Stream1_IRQn
#define USARTwp_DMA_TX_IRQHandler         DMA2_Stream7_IRQHandler
#define USARTwp_DMA_RX_IRQHandler         DMA2_Stream1_IRQHandler

/* Definition for USARTwp's NVIC */
#define USARTwp_IRQn                      USART1_IRQn
#define USARTwp_IRQHandler                USART1_IRQHandler