//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//
#include "DisplayInterface.h"

#include "sys_dev_spi_native.h"
#include <nanoPAL.h>
#include <target_platform.h>

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#define NUMBER_OF_LINES 2
#define SPI_MAX_TRANSFER_SIZE (240 * 2 * NUMBER_OF_LINES) // 240 pixels 2 words wide (16 bit colour)

#define SPI_PORT spi1
#define LCD_CLK_PIN 10
#define LCD_MOSI_PIN 11

#define MAX_SPI_BAUD_RATE 62500000

struct DisplayInterface g_DisplayInterface;
DisplayInterfaceConfig g_DisplayInterfaceConfig;
// Saved gpio pins
CLR_INT16 lcdReset;
CLR_INT16 lcdDC;
CLR_INT16 lcdBacklight;
CLR_INT16 lcdchipSelect;
CLR_INT16 outputBufferSize;
CLR_UINT8 spiBuffer[SPI_MAX_TRANSFER_SIZE];
CLR_UINT8 spiCommandMode = 0; // 0 Command first byte, 1 = Command all bytes

// Grab some unused dma channels
static uint dma_tx;
static uint dma_rx;

// Display Interface
void DisplayInterface::Initialize(DisplayInterfaceConfig &config)
{

    lcdchipSelect = config.Spi.chipSelect;
    spi_init(SPI_PORT, MAX_SPI_BAUD_RATE);
    gpio_set_function(LCD_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LCD_MOSI_PIN, GPIO_FUNC_SPI);

    // Configure GPIO
    // ==============
    // Reset
    lcdReset = config.Spi.reset;
    gpio_init(lcdReset);
    gpio_set_dir(lcdReset, GPIO_OUT);
    gpio_put(lcdReset, 1);

    // Data/Command
    lcdDC = config.Spi.dataCommand;
    gpio_init(lcdDC);
    gpio_set_dir(lcdDC, GPIO_OUT);
    gpio_put(lcdDC, 1);

    // Chip Select
    lcdchipSelect = config.Spi.chipSelect;
    gpio_init(lcdchipSelect);
    gpio_set_dir(lcdchipSelect, GPIO_OUT);
    gpio_put(lcdchipSelect, 1);

    // Backlight
    lcdBacklight = config.Spi.backLight;
    gpio_init(lcdBacklight);
    gpio_set_dir(lcdBacklight, GPIO_OUT);
    gpio_put(lcdBacklight, 1);

    // Setup dma to write the data out, this allows the CPU to continue while the data is sent
    // We set the outbound DMA to transfer from a memory buffer to the SPI transmit FIFO paced by the SPI TX FIFO DREQ

#if defined DMA
    // Rough out of code, not tested
    //------------------------------
    dma_tx = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, spi_get_dreq(SPI_PORT, true));
    dma_channel_configure(dma_tx, &c,
                          &spi_get_hw(SPI_PORT)->dr, // write address
                          spiBuffer,                 // read address
                          DMA_SIZE_16,               // element count (each element is of size transfer_data_size)
                          false);                    // don't start yet

#endif

    gpio_put(lcdReset, 1);
    PLATFORM_DELAY(100);
    gpio_put(lcdReset, 0);
    PLATFORM_DELAY(100);
    gpio_put(lcdReset, 1);
    PLATFORM_DELAY(100);
    return;
}
void DisplayInterface::SetCommandMode(int mode)
{
    spiCommandMode = mode;
}
void DisplayInterface::GetTransferBuffer(CLR_UINT8 *&TransferBuffer,
                                         CLR_UINT32 &TransferBufferSize)
{
    TransferBuffer = spiBuffer;
    TransferBufferSize = sizeof(spiBuffer);
}

void DisplayInterface::ClearFrameBuffer()
{
    // Set screen to black
}

void DisplayInterface::WriteToFrameBuffer(CLR_UINT8 command, CLR_UINT8 data[],
                                          CLR_UINT32 dataCount,
                                          CLR_UINT32 frameOffset)
{
    (void)frameOffset;
    
    
    
    gpio_put(lcdchipSelect, GpioPinValue_Low);
    gpio_put(lcdDC, GpioPinValue_Low);
    SendBytes(&command, 1);
    gpio_put(lcdDC, GpioPinValue_High);
    SendBytes(data, dataCount);
    return;
}
void DisplayInterface::SendCommand(CLR_UINT8 arg_count, ...)
{
    va_list ap;
    va_start(ap, arg_count);

    // Parse arguments into parameters buffer
    CLR_UINT8 parameters[arg_count];
    for (int i = 0; i < arg_count; i++)
    {
        parameters[i] = va_arg(ap, int);
    }
    gpio_put(lcdchipSelect, GpioPinValue_Low);

    // Send only first byte (command) with D/C signal low
    gpio_put(lcdDC, GpioPinValue_Low);
    SendBytes(&parameters[0], 1);

    // Data mode send remaining parameters
    gpio_put(lcdDC, GpioPinValue_High);
    if (arg_count > 1)
    {
        SendBytes(&parameters[1], arg_count - 1);
    }
    gpio_put(lcdchipSelect, GpioPinValue_High);
}
void DisplayInterface::DisplayBacklight(bool on) // true = on
{
    if (on)
    {
        gpio_put(lcdBacklight, 1);
    }
    else
    {
        gpio_put(lcdBacklight, 0);
    }
    return;
}
void DisplayInterface::SendBytes(CLR_UINT8 *data, CLR_UINT32 length)
{
    if (length == 0)
        return; // no need to send anything

    spi_write_blocking(SPI_PORT, data, length);

#if defined DMA
    // Rough out of code, not tested
    //------------------------------
    // start the DMA
    while (dma_channel_is_busy(dma_tx))
    {
        tx_thread_relinquish();
    };
    dma_start_channel_mask((1u << dma_tx));

    // Block here as returning at this stage will cause the buffer to be re-written with the
    // next part of the full graphics output
    while (dma_channel_is_busy(dma_tx))
    {
        tx_thread_relinquish();
    };
#endif
}
