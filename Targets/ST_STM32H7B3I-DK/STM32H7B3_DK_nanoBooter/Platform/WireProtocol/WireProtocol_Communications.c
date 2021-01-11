#include <nanoHAL_v2.h>
#include <WireProtocol_Communications.h>
#include <BoardInit.h>
#include "WireProtocol_Message.h"
#include <stm32h7xx.h>
#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_def.h>
#include <stm32h7xx_hal_uart.h>

/*
Board: STM32H7B3I-DK

   _STM32H7B3I-DK___      _STLINK_3E__________      __PC______
  |      ___________|    |_______             |    |          |
  |     |      USART|    |USART  |            |    |          |
  |     | (PA10) TX |____| RX    |  usb_dev_hs|____| Virtual  |
  |     |           |    |       |            |____| Com Port |
  |     | (PA9)  RX |____| TX    |            |    |          |
  |     |___________|    |_______|            |    |          |
  |                 |    |                    |    |          |
  |              GND|____|GND                 |    |          |
  |_________________|    |____________________|    |__________|

*/

UART_HandleTypeDef UartHandle;
ReadPacketState CurrentReadPacketState;
WritePacketState CurrentWritePacketState;

static int debugCounter = 0;
struct PacketsReceived
{
	long CurrentIndex;
	long ProcessedIndex;
	long NumberNotProcessed;
	long TotalPackets;
	struct {
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


/* Buffer used for transmission :
   Size should be a Multiple of cache line size (32 bytes) */
ALIGN_32BYTES(uint8_t aTxBuffer[TXBUFFERSIZE]);

/* Buffer used for reception :
   Size should be a Multiple of cache line size (32 bytes) */
ALIGN_32BYTES(uint8_t aRxBuffer[RXBUFFER]);

GPIO_InitTypeDef  GPIO_InitStruct;


bool InitWireProtocolCommunications()
{
	// Use USART as defined in the header file
	UartHandle.Instance = USARTx;
	UartHandle.Init.BaudRate = 921600;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits = UART_STOPBITS_1;
	UartHandle.Init.Parity = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_DeInit(&UartHandle) != HAL_OK)
	{
		HAL_AssertEx();
	}
	if (HAL_UART_Init(&UartHandle) != HAL_OK)  //  HAL_UART_MspInit is called from HAL_UART_Init
	{
		HAL_AssertEx();
	}

	HAL_UART_Abort(&UartHandle);  // Cancel any outstanding reads/writes in place

	CurrentReadPacketState = ReadNotSet;
	CurrentWritePacketState = WriteNotSet;

	printf("Queue the first Read after init \n");
	if (HAL_UART_Receive_DMA(&UartHandle, (uint8_t*)aRxBuffer, RXBUFFER) != HAL_OK)
	{
		HAL_AssertEx();
	}

	CurrentReadPacketState = Listening;

	CircularBuffer.CurrentIndex = 0;
	CircularBuffer.ProcessedIndex = 0;

	return true;
}

bool WritePacket(uint8_t* ptr, uint16_t size)
{
	memcpy(aTxBuffer, ptr, size);

	if (HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aTxBuffer, size) != HAL_OK)
	{
		HAL_AssertEx();
	}
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef* UartHandle)
//{
//	CurrentWritePacketState = ReadFull;
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* UartHandle)
{
	printf("HAL_UART_RxCpltCallback\n");

	// Completion occurs when a complete packet is read
	if (UartHandle->Instance == USART1)
	{
		SCB_InvalidateDCache_by_Addr((uint32_t*)aRxBuffer, sizeof(aRxBuffer)); // Tricky one

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
}


bool ReadNextPacket(uint8_t* ptr, uint16_t* size)
{
	//debugCounter++;
	//if (debugCounter % 10 == 0)
	//{
		switch (UartHandle.RxState)
		{
		case  HAL_UART_STATE_READY:
			printf("ReadNextPacket: HAL UART RxState (%s)\n", "HAL_UART_STATE_READY");
			break;
		case HAL_UART_STATE_BUSY_RX:
			printf("ReadNextPacket: HAL UART RxState (%s)\n", "HAL_UART_STATE_BUSY_RX");
			break;
		default:
			printf("ReadNextPacket: other state (%d)\n", UartHandle.RxState);
			break;
		}
	//}

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
			if (UartHandle.RxState == HAL_UART_STATE_READY)
			{
				printf("Queue Read\n");
				if (HAL_UART_Receive_DMA(&UartHandle, (uint8_t*)aRxBuffer, RXBUFFER) != HAL_OK)
				{
					HAL_AssertEx();
				}
				while (UartHandle.RxState == HAL_UART_STATE_READY)
				{
					tx_thread_sleep(10);
				}

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

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
	int counter = 0;
	if (huart->ErrorCode != HAL_UART_ERROR_NONE)
	{
		switch (huart->ErrorCode)
		{
		case  HAL_UART_ERROR_PE:
			printf("Parity Error\n");
			break;
		case  HAL_UART_ERROR_NE:
			CircularBuffer.CommsErrors.Noise_error++;
			printf("Noise Error\n");
			break;
		case  HAL_UART_ERROR_FE:
			CircularBuffer.CommsErrors.Frame_error++;
			printf("Frame Error\n");
			break;
		case  HAL_UART_ERROR_ORE:
			CircularBuffer.CommsErrors.Overrun_error++;
			printf("Overrun Error\n");
			break;
		case  HAL_UART_ERROR_DMA:
			CircularBuffer.CommsErrors.DMA_transfer_error++;
			printf("DMA Transfer Error\n");
			break;
		case  HAL_UART_ERROR_RTO:
			CircularBuffer.CommsErrors.Receiver_Timeout_error++;
			printf("Receiver Timeout Error\n");
			break;
		}

		HAL_UART_Abort(huart);

		HAL_DMA_Abort(huart->hdmarx);
		if (huart->RxState == HAL_UART_STATE_READY)
		{
			printf("Queue Read\n");
			if (HAL_UART_Receive_DMA(huart, (uint8_t*)aRxBuffer, RXBUFFER) != HAL_OK)
			{
				HAL_AssertEx();
			}
		}
	}
	else
	{
		printf("Error None");
	}
}


