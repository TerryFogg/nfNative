                                                         
                                                         
### ESP32 for later followup

##### Autobaud

UART_LOWPULSE_MIN_CNT stores minimum low-pulse width, UART_HIGHPULSE_MIN_CNT stores
minimum high-pulse width.

By reading these two registers, software can calculate the baud rate of the
transmitter.
So you have to calculate and set the baud yourself based on those values, it is not really "auto".


##### Maximum baud rate
The fact that 2M baud seems to communicate well, yet flashing only runs at 825.8 kbit/s leads me to believe that there's a bottleneck at the flash writing, but I can't seem to find any documentation of a maximum flashing speed.



 # STM32 Random examples

### Uart Examples

##### Polled

```
#include "main.h"
 
uint8_t UART1_rxBuffer[12] = {0};
 
UART_HandleTypeDef huart1;
 
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
 
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
 
    HAL_UART_Receive (&huart1, UART1_rxBuffer, 12, 5000);
    HAL_UART_Transmit(&huart1, UART1_rxBuffer, 12, 100);
    
    while (1)
    {
 
    }
 
}
```
 ##### Interrupt
```
#include "main.h"
 
uint8_t UART1_rxBuffer[12] = {0};
 
UART_HandleTypeDef huart1;
 
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
 
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
 
    HAL_UART_Receive_IT (&huart1, UART1_rxBuffer, 12);
 
    while (1)
    {
 
    }
 
}
 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_Transmit(&huart1, UART1_rxBuffer, 12, 100);
    HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 12);
}
```

####  DMA

```
#include "main.h"
 
uint8_t UART1_rxBuffer[12] = {0};
 
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
 
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
 
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
 
    HAL_UART_Receive_DMA (&huart1, UART1_rxBuffer, 12);
 
    while (1)
    {
 
    }
 
}
 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_Transmit(&huart1, UART1_rxBuffer, 12, 100);
    HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, 12);
}

```



![U S A R T Block Diagram](Resources/USART_blockDiagram.png)