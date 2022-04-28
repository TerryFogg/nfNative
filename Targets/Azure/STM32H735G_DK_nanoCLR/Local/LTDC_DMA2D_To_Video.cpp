//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

// -------------------------------------------------------
// For the STM32
// Directly drive a LCD TFT using the LTDC controller

// -    - This driver uses the adequate timing and setting for the RK043FN48H LCD component

#include <stdarg.h>
#include <stdio.h>

#include "DisplayInterface.h"
#include <nanoCLR_Interop.h>

#include "BoardInit.h"

#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_dma2d.h"

#include "RGB565_320x240.h"

// LCD Display control pin
#define LCD_DISP_CTRL_PIN       LL_GPIO_PIN_10
#define LCD_DISP_CTRL_GPIO_PORT GPIOD

// LCD Display enable pin
#define LCD_DISP_EN_PIN       LL_GPIO_PIN_13
#define LCD_DISP_EN_GPIO_PORT GPIOE

// Back-light control pin
#define LCD_BL_CTRL_PIN       LL_GPIO_PIN_15
#define LCD_BL_CTRL_GPIO_PORT GPIOG

#define LCD_LAYER_0_ADDRESS         0x70000000U
#define PIXEL_FORMAT_RGB565         0x00000002U
#define LTDC_BLENDING_FACTOR1_PAxCA 0x00000600U /*!< Blending factor : Cte Alpha x Pixel Alpha*/

// Board Specific Graphics
#define LTDC_HSPOLARITY_AL          0x00000000U     /*!< Horizontal Synchronization is active low. */
#define LTDC_VSPOLARITY_AL          0x00000000U     /*!< Vertical Synchronization is active low. */
#define LTDC_DEPOLARITY_AL          0x00000000U     /*!< Data Enable, is active low. */
#define LTDC_PCPOLARITY_IPC         0x00000000U     /*!< input pixel clock. */
#define LTDC_IT_TE                  LTDC_IER_TERRIE /*!< LTDC Transfer Error Interrupt  */
#define LTDC_IT_FU                  LTDC_IER_FUIE   /*!< LTDC FIFO Underrun Interrupt   */
#define MAX_LAYER                   2U
#define LTDC_BLENDING_FACTOR2_PAxCA 0x00000007U /*!< Blending factor : Cte Alpha x Pixel Alpha*/

struct DisplayInterface g_DisplayInterface;
extern CLR_UINT32 Graphics_frame_buffer; // Framebuffer set externally

// Default to landscape
CLR_UINT32 lcd_x_size = 480;
CLR_UINT32 lcd_y_size = 272;

// Timer handler declaration
// static TIM_HandleTypeDef hlcd_tim;

uint32_t Width;
uint32_t Height;

void DisplayInterface::Initialize(DisplayInterfaceConfig &config)
{
    Width = config.VideoDisplay.width;
    Height = config.VideoDisplay.height;

    LTDCClock_Config();

    // Enable THE CLOCKS
    LL_APB3_GRP1_EnableClock(LL_APB3_GRP1_PERIPH_LTDC);  //
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA); //
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB); //
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC); //
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD); //   LCD_DISP_CTRL_GPIO_CLK_ENABLE();
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE); //   LCD_DISP_EN_GPIO_CLK_ENABLE();
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOG); //   LCD_BL_CTRL_GPIO_CLK_ENABLE();
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOH); //

    // LTDC Pins configuration
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_14;

    // LTDC Pins configuration
    // GPIO A configuration
    GPIO_InitStruct.Pin = LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_6;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // GPIO B configuration
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // GPIO C configuration
    GPIO_InitStruct.Pin = LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // GPIO D configuration
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_3 | LL_GPIO_PIN_6;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // GPIO E configuration
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LCD_BL_CTRL_PIN;
    LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    // GPIO G configuration
    GPIO_InitStruct.Pin = LL_GPIO_PIN_7 | LL_GPIO_PIN_14;
    LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    // GPIO H configuration
    GPIO_InitStruct.Pin =
        LL_GPIO_PIN_3 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LCD_DISP_CTRL_PIN | LL_GPIO_PIN_11 | LCD_BL_CTRL_PIN;
    LL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_9;
    LL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    // Initialize the LTDC
    LL_APB3_GRP1_ForceReset(LL_APB3_GRP1_PERIPH_LTDC); // Toggle Sw reset of LTDC IP
    LL_APB3_GRP1_ReleaseReset(LL_APB3_GRP1_PERIPH_LTDC);

    // Set Enable, Control and Backlight pins to output mode
    LL_GPIO_InitTypeDef GPIO_InitStructLCD = {0};
    GPIO_InitStructLCD.Mode = LL_GPIO_OUTPUT_PUSHPULL; // Output for all pins

    GPIO_InitStructLCD.Pin = config.VideoDisplay.backlight;
    LL_GPIO_Init(LCD_BL_CTRL_GPIO_PORT, &GPIO_InitStructLCD); // Backlight

    GPIO_InitStructLCD.Pin = config.VideoDisplay.enable;
    LL_GPIO_Init(LCD_DISP_EN_GPIO_PORT, &GPIO_InitStructLCD); // Enable

    GPIO_InitStructLCD.Pin = config.VideoDisplay.control;
    LL_GPIO_Init(LCD_DISP_CTRL_GPIO_PORT, &GPIO_InitStructLCD); // Control

    LL_GPIO_ResetOutputPin(LCD_DISP_EN_GPIO_PORT, LCD_DISP_EN_PIN);
    LL_GPIO_SetOutputPin(LCD_DISP_CTRL_GPIO_PORT, LCD_DISP_CTRL_PIN);
    LL_GPIO_SetOutputPin(LCD_BL_CTRL_GPIO_PORT, LCD_BL_CTRL_PIN);

    LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_DMA2D); // Enable the DMA2D clock
    LL_AHB3_GRP1_ForceReset(LL_AHB3_GRP1_PERIPH_DMA2D);  // Toggle Sw reset of DMA2D IP
    LL_AHB3_GRP1_ReleaseReset(LL_AHB3_GRP1_PERIPH_DMA2D);

    LL_DMA2D_SetOutputColorMode(DMA2D, LL_DMA2D_OUTPUT_MODE_RGB565); // Configure the DMA2D Color Mode
    LL_DMA2D_FGND_SetAlpha(DMA2D, 0xFF);                             // Always write opaque
    LL_DMA2D_FGND_SetColorMode(DMA2D, LL_DMA2D_INPUT_MODE_RGB565);   // Foreground layer format is RGB565 (16 bpp)

    // Configure the HS, VS, DE and PC polarity
    LTDC->GCR &= ~(LTDC_GCR_HSPOL | LTDC_GCR_VSPOL | LTDC_GCR_DEPOL | LTDC_GCR_PCPOL);
    LTDC->GCR |= (uint32_t)(LTDC_HSPOLARITY_AL | LTDC_VSPOLARITY_AL | LTDC_DEPOLARITY_AL | LTDC_PCPOLARITY_IPC);

    // Set Synchronization size
    LTDC->SSCR &= ~(LTDC_SSCR_VSH | LTDC_SSCR_HSW);
    LTDC->SSCR |=
        (((config.VideoDisplay.Horizontal_synchronization - 1U) << 16U) |
         (config.VideoDisplay.Vertical_synchronization - 1U));

    // Set Accumulated Back porch
    LTDC->BPCR &= ~(LTDC_BPCR_AVBP | LTDC_BPCR_AHBP);
    LTDC->BPCR |=
        (((config.VideoDisplay.Horizontal_synchronization + (config.VideoDisplay.Horizontal_back_porch - 11U) - 1U)
          << 16U) |
         (config.VideoDisplay.Vertical_synchronization + config.VideoDisplay.Vertical_back_porch - 1U));

    // Set Accumulated Active Width
    LTDC->AWCR &= ~(LTDC_AWCR_AAH | LTDC_AWCR_AAW);
    LTDC->AWCR |=
        (((config.VideoDisplay.Horizontal_synchronization + Width + config.VideoDisplay.Horizontal_back_porch - 1U)
          << 16U) |
         (config.VideoDisplay.Vertical_synchronization + Height + config.VideoDisplay.Vertical_back_porch - 1U));

    // Set Total Width
    LTDC->TWCR &= ~(LTDC_TWCR_TOTALH | LTDC_TWCR_TOTALW);
    LTDC->TWCR |=
        (((config.VideoDisplay.Horizontal_synchronization + Width + (config.VideoDisplay.Horizontal_back_porch - 11U) +
           config.VideoDisplay.Horizontal_front_porch - 1U)
          << 16U) |
         (config.VideoDisplay.Vertical_synchronization + Height + config.VideoDisplay.Vertical_back_porch +
          config.VideoDisplay.Vertical_front_porch - 1U));

    // Set the background color value
    LTDC->BCCR &= ~(LTDC_BCCR_BCBLUE | LTDC_BCCR_BCGREEN | LTDC_BCCR_BCRED);
    LTDC->BCCR |= (0 | 0 | 0);

    LTDC->IER |= (LTDC_IT_TE | LTDC_IT_FU); // Enable the Transfer Error and FIFO underrun interrupts
    LTDC->GCR |= LTDC_GCR_LTDCEN;           // Enable LTDC by setting LTDCEN bit

    // Configure up a default LTDC Layer 0
    // We don't use the layer blending feature, all the 'smarts' are performed by
    // code on the graphics buffer before we DMA2D it to the graphics frame buffer
    //
    // Configure the LTDC Layer
    //-------------------
    LTDC_Layer_TypeDef *Layer0 = (LTDC_Layer_TypeDef *)((uint32_t)LTDC + 0x84U);

    // Configure the horizontal start and stop position
    Layer0->WHPCR &= ~(LTDC_LxWHPCR_WHSTPOS | LTDC_LxWHPCR_WHSPPOS);

    Layer0->WHPCR =
        ((0 + ((LTDC->BPCR & LTDC_BPCR_AHBP) >> 16U) + 1U) | ((Width + ((LTDC->BPCR & LTDC_BPCR_AHBP) >> 16U)) << 16U));

    // Configure the vertical start and stop position
    Layer0->WVPCR &= ~(LTDC_LxWVPCR_WVSTPOS | LTDC_LxWVPCR_WVSPPOS);
    Layer0->WVPCR = ((0 + (LTDC->BPCR & LTDC_BPCR_AVBP) + 1U) | ((Height + (LTDC->BPCR & LTDC_BPCR_AVBP)) << 16U));

    // Specifies the pixel format
    Layer0->PFCR &= ~(LTDC_LxPFCR_PF);
    Layer0->PFCR = (PIXEL_FORMAT_RGB565);

    // Configure the default color values
    Layer0->DCCR &= ~(LTDC_LxDCCR_DCBLUE | LTDC_LxDCCR_DCGREEN | LTDC_LxDCCR_DCRED | LTDC_LxDCCR_DCALPHA);
    Layer0->DCCR = (0 | 0 | 0 | 0);

    // Specifies the constant alpha value
    Layer0->CACR &= ~(LTDC_LxCACR_CONSTA);
    Layer0->CACR = (255);

    // Specifies the blending factors
    Layer0->BFCR &= ~(LTDC_LxBFCR_BF2 | LTDC_LxBFCR_BF1);
    Layer0->BFCR = (LTDC_BLENDING_FACTOR1_PAxCA | LTDC_BLENDING_FACTOR2_PAxCA);

    // Configure the color frame buffer start address
    Layer0->CFBAR &= ~(LTDC_LxCFBAR_CFBADD);
    Layer0->CFBAR = (LCD_LAYER_0_ADDRESS);

    // Configure the color frame buffer pitch in byte
    Layer0->CFBLR &= ~(LTDC_LxCFBLR_CFBLL | LTDC_LxCFBLR_CFBP);
    Layer0->CFBLR = (((Width * 2) << 16U) | (((Width - 0) * 2) + 7U));
    // Configure the frame buffer line number
    Layer0->CFBLNR &= ~(LTDC_LxCFBLNR_CFBLNBR);
    Layer0->CFBLNR = Height;

    // Enable LTDC_Layer by setting LEN bit
    Layer0->CR |= (uint32_t)LTDC_LxCR_LEN;

    // Set the Immediate Reload type
    LTDC->SRCR = LTDC_SRCR_IMR;

    return;
}

void DisplayInterface::GetTransferBuffer(CLR_UINT8 *&TransferBuffer, CLR_UINT32 &TransferBufferSize)
{
    TransferBuffer = (CLR_UINT8 *)&Graphics_frame_buffer;
    TransferBufferSize = (lcd_x_size * lcd_y_size * 2);
}

#define LCD_COLOR_RGB565_WHITE 0xFFFFU

void DisplayInterface::ClearFrameBuffer()
{
    while (LL_DMA2D_IsTransferOngoing(DMA2D))
        ; // DMA2D is asynchronous, we may return here from a previous call before it was finished

    LL_DMA2D_SetMode(DMA2D, LL_DMA2D_MODE_R2M);                         // Configured as register to memory mode
    LL_DMA2D_SetOutputColor(DMA2D, LCD_COLOR_RGB565_WHITE);             // Clear screen colour
    LL_DMA2D_SetOutputMemAddr(DMA2D, (uint32_t)&Graphics_frame_buffer); // LCD data address
    LL_DMA2D_SetLineOffset(DMA2D, 0);     // Configure DMA2D output line offset to LCD width - image width for display
    LL_DMA2D_ConfigSize(DMA2D, 272, 480); // Configure DMA2D
    DMA2D->OPFCCR = PIXEL_FORMAT_RGB565;  // Format color
    LL_DMA2D_Start(DMA2D);                // Start the transfer
    //    while (DMA2D->CR & DMA2D_CR_START)
    //    {
    //    } // Wait for dma2d transmission to complete

    //    WriteToFrameBuffer(0, (CLR_UINT8 *)RGB565_320x240, 0, 0);
}

void DisplayInterface::WriteToFrameBuffer(
    CLR_UINT8 command,
    CLR_UINT8 data[],
    CLR_UINT32 dataCount,
    CLR_UINT32 frameOffset)
{

    while (LL_DMA2D_IsTransferOngoing(DMA2D))
        ; // DMA2D is asynchronous, so we may return here from a previous call before it was finished

    LL_DMA2D_SetMode(DMA2D, LL_DMA2D_MODE_M2M);                         // Configured as memory to memory mode
    LL_DMA2D_FGND_SetMemAddr(DMA2D, (uint32_t)data);                    // Source buffer in format RGB565
    LL_DMA2D_SetOutputMemAddr(DMA2D, (uint32_t)&Graphics_frame_buffer); // LCD data address
    LL_DMA2D_ConfigSize(DMA2D, 272, 480);                               // Configure DMA2D
    LL_DMA2D_SetLineOffset(DMA2D, 0); // Configure DMA2D output line offset to LCD width - image width for display
    LL_DMA2D_EnableIT_TC(DMA2D);      // Enable the transfer complete
    LL_DMA2D_Start(DMA2D);            // Start the transfer

    //    SCB_InvalidateDCache_by_Addr((uint32_t *)Graphics_frame_buffer, 272 * 480 * 2);
    //    SCB_InvalidateDCache_by_Addr((uint32_t *)Graphics_frame_buffer, 272 * 480 * 2);
    SCB_CleanInvalidateDCache();
    // Wait for dma2d transmission to complete
}

void DisplayInterface::DisplayBacklight(bool on)
{
}

void DisplayInterface::SendCommand(CLR_UINT8 NbrParams, ...)
{
}

int32_t BSP_LCD_SetBrightness(uint32_t Instance, uint32_t Brightness)
{
    //    TIM_SET_COMPARE(&hlcd_tim, LCD_TIMx_CHANNEL, 2U*Brightness);
    return 1;
}

//
// brief Set the brightness value *@param Instance LCD Instance *@param Brightness [00:Min(black), 100 Max] *
//    @retval BSP status

int32_t BSP_LCD_GetBrightness(uint32_t Instance, uint32_t *Brightness)
{
    return 1;
}
