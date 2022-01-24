//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

// -------------------------------------------------------
// For the STM32
// Directly drive a LCD TFT using the LTDC controller

// -    - This driver uses the adequate timing and setting for the RK043FN48H LCD component

#define STM32H7B3xx

#include <stdarg.h>
#include <stdio.h>

#include "DisplayInterface.h"
#include <nanoCLR_Interop.h>

#include "stm32h735g_discovery_ospi.h"
#include "BoardInit.h"

#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_dma2d.h"

#include "RGB565_320x240.h"

/* LCD Display control pin */
#define LCD_DISP_CTRL_PIN       GPIO_PIN_10
#define LCD_DISP_CTRL_GPIO_PORT GPIOD

/* LCD Display enable pin */
#define LCD_DISP_EN_PIN                GPIO_PIN_13
#define LCD_DISP_EN_GPIO_PORT          GPIOE
#define LCD_DISP_EN_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOE_CLK_ENABLE()
#define LCD_DISP_EN_GPIO_CLK_DISABLE() __HAL_RCC_GPIOE_CLK_DISABLE()

/* Back-light control pin */
#define LCD_BL_CTRL_PIN       GPIO_PIN_15
#define LCD_BL_CTRL_GPIO_PORT GPIOG

struct DisplayInterface g_DisplayInterface;
extern CLR_UINT32 GraphicsVideoFrameBufferBegin; // Framebuffer set externally
extern DMA2D_HandleTypeDef Dma2dHandle;

// Default to landscape
CLR_UINT32 lcd_x_size = 480;
CLR_UINT32 lcd_y_size = 272;

/* Timer handler declaration */
static TIM_HandleTypeDef hlcd_tim;

uint32_t Width;
uint32_t Height;
uint32_t ltdc_pixel_format = LTDC_PIXEL_FORMAT_RGB565;

void DisplayInterface::Initialize(DisplayInterfaceConfig &config)
{
    Width = config.VideoDisplay.width;
    Height = config.VideoDisplay.height;

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
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_14;

    // LTDC Pins configuration
    // GPIO A configuration
    GPIO_InitStruct.Pin = LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_6;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // GPIO B configuration
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // GPIO C configuration
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // GPIO D configuration
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_3 | GPIO_PIN_6;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // GPIO E configuration
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_11 | GPIO_PIN_12 | LCD_BL_CTRL_PIN;
    LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    // GPIO G configuration
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_14;
    LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    // GPIO H configuration
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_8 | GPIO_PIN_9 | LCD_DISP_CTRL_PIN | GPIO_PIN_11 | LCD_BL_CTRL_PIN;
    LL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
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
    LL_DMA2D_FGND_SetColorMode(DMA2D,
                               LL_DMA2D_INPUT_MODE_RGB565); // Foreground layer format is RGB565 (16 bpp)

    // RK043FN48H LCD clock configuration
    // LCD clock configuration
    // PLL3_VCO Input = HSE_VALUE/PLL3M = 4 Mhz
    // PLL3_VCO Output = PLL3_VCO Input * PLL3N = 800 Mhz
    // PLLLCDCLK = PLL3_VCO Output/PLL3R = 800/83 = 9.63 Mhz
    // LTDC clock frequency = PLLLCDCLK = 9.63 Mhz
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
    PeriphClkInitStruct.PLL3.PLL3M = 5;
    PeriphClkInitStruct.PLL3.PLL3N = 160;
    PeriphClkInitStruct.PLL3.PLL3P = 2;
    PeriphClkInitStruct.PLL3.PLL3Q = 2;
    PeriphClkInitStruct.PLL3.PLL3R = 83;
    PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
    PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL1VCOWIDE;
    PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL1VCIRANGE_0;

    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    LTDC_HandleTypeDef hlcd_ltdc;
    hlcd_ltdc.Instance = LTDC;
    hlcd_ltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
    hlcd_ltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
    hlcd_ltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
    hlcd_ltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
    hlcd_ltdc.Init.HorizontalSync = config.VideoDisplay.Horizontal_synchronization - 1U;
    hlcd_ltdc.Init.AccumulatedHBP =
        (config.VideoDisplay.Horizontal_synchronization + (config.VideoDisplay.Horizontal_back_porch - 11U) - 1U);
    hlcd_ltdc.Init.AccumulatedActiveW =
        config.VideoDisplay.Horizontal_synchronization + Width + config.VideoDisplay.Horizontal_back_porch - 1U;
    hlcd_ltdc.Init.TotalWidth = config.VideoDisplay.Horizontal_synchronization + Width +
                                (config.VideoDisplay.Horizontal_back_porch - 11U) +
                                config.VideoDisplay.Horizontal_front_porch - 1U;
    hlcd_ltdc.Init.VerticalSync = config.VideoDisplay.Vertical_synchronization - 1U;
    hlcd_ltdc.Init.AccumulatedVBP =
        config.VideoDisplay.Vertical_synchronization + config.VideoDisplay.Vertical_back_porch - 1U;
    hlcd_ltdc.Init.AccumulatedActiveH =
        config.VideoDisplay.Vertical_synchronization + Height + config.VideoDisplay.Vertical_back_porch - 1U;
    hlcd_ltdc.Init.TotalHeigh = config.VideoDisplay.Vertical_synchronization + Height +
                                config.VideoDisplay.Vertical_back_porch + config.VideoDisplay.Vertical_front_porch - 1U;
    hlcd_ltdc.Init.Backcolor.Blue = 0xFF;
    hlcd_ltdc.Init.Backcolor.Green = 0xFF;
    hlcd_ltdc.Init.Backcolor.Red = 0xFF;
    HAL_LTDC_Init(&hlcd_ltdc);

    // Configure up a default LTDC Layer 0
    // We don't use the layer blending feature, all the 'smarts' are performed by
    // code on the graphics buffer before we DMA2D it to the graphics frame buffer
    //
    LTDC_LayerCfgTypeDef LayerCfg;
    LayerCfg.WindowX0 = 0;
    LayerCfg.WindowX1 = Width;
    LayerCfg.WindowY0 = 0;
    LayerCfg.WindowY1 = Height;
    LayerCfg.PixelFormat = ltdc_pixel_format;
    LayerCfg.Alpha = 255;
    LayerCfg.Alpha0 = 0;
    LayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
    LayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
    LayerCfg.FBStartAdress = LCD_LAYER_0_ADDRESS;
    LayerCfg.ImageWidth = Width;
    LayerCfg.ImageHeight = Height;
    LayerCfg.Backcolor.Blue = 0;
    LayerCfg.Backcolor.Green = 0;
    LayerCfg.Backcolor.Red = 0;
    HAL_LTDC_ConfigLayer(&hlcd_ltdc, &LayerCfg, 0);

    return;
}

void DisplayInterface::GetTransferBuffer(CLR_UINT8 *&TransferBuffer, CLR_UINT32 &TransferBufferSize)
{

    //     *(__IO uint16_t*)(hlcd_ltdc.LayerCfg[Lcd_Ctx[Instance].ActiveLayer].FBStartAdress +
    //     (2U*((Ypos*Lcd_Ctx[Instance].XSize) + Xpos))) = (uint16_t)Color;

    TransferBuffer = (CLR_UINT8 *)&GraphicsVideoFrameBufferBegin;
    TransferBufferSize = (lcd_x_size * lcd_y_size * 2);
}

#define LCD_COLOR_RGB565_WHITE 0xFFFFU
#define NO_LINE_OFFSET         0

void DisplayInterface::ClearFrameBuffer()
{

    while (LL_DMA2D_IsTransferOngoing(DMA2D))
        ; // DMA2D is asynchronous, so we may return here from a previous call before it was finished

    LL_DMA2D_SetMode(DMA2D, LL_DMA2D_MODE_R2M);                                 // Configured as register to memory mode
    LL_DMA2D_SetOutputColor(DMA2D, LCD_COLOR_RGB565_WHITE);                     // Clear screen colour
    LL_DMA2D_SetOutputMemAddr(DMA2D, (uint32_t)&GraphicsVideoFrameBufferBegin); // LCD data address
    LL_DMA2D_SetLineOffset(DMA2D, 0);     // Configure DMA2D output line offset to LCD width - image width for display
    DMA2D->OPFCCR = ltdc_pixel_format;    // Format color
    LL_DMA2D_ConfigSize(DMA2D, 272, 480); // Configure DMA2D transfer number of lines and number of pixels per line
    LL_DMA2D_Start(DMA2D);                // Start the transfer
    while (DMA2D->CR & DMA2D_CR_START)
    {
    } // Wait for dma2d transmission to complete

    WriteToFrameBuffer(0, (CLR_UINT8 *)RGB565_320x240, 0, 0);
}

void DisplayInterface::WriteToFrameBuffer(
    CLR_UINT8 command,
    CLR_UINT8 data[],
    CLR_UINT32 dataCount,
    CLR_UINT32 frameOffset)
{

    while (LL_DMA2D_IsTransferOngoing(DMA2D))
        ; // DMA2D is asynchronous, so we may return here from a previous call before it was finished

    LL_DMA2D_SetMode(DMA2D, LL_DMA2D_MODE_M2M);                                 // Configured as memory to memory mode
    LL_DMA2D_FGND_SetMemAddr(DMA2D, (uint32_t)data);                            // Source buffer in format RGB565
    LL_DMA2D_SetOutputMemAddr(DMA2D, (uint32_t)&GraphicsVideoFrameBufferBegin); // LCD data address
    LL_DMA2D_ConfigSize(DMA2D, 240, 320); // Configure DMA2D transfer number of lines and number of pixels per line
    LL_DMA2D_SetLineOffset(
        DMA2D,
        Width - 320);            // Configure DMA2D output line offset to LCD width - image width for display
    LL_DMA2D_EnableIT_TC(DMA2D); // Enable the transfer complete
    LL_DMA2D_Start(DMA2D);       // Start the transfer
    while (DMA2D->CR & DMA2D_CR_START)
    {
    }

    //   SCB_InvalidateDCache_by_Addr((uint32_t *)GraphicsVideoFrameBufferBegin, 272 * 480 * 2);
    // SCB_CleanInvalidateDCache();
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
    //    __HAL_TIM_SET_COMPARE(&hlcd_tim, LCD_TIMx_CHANNEL, 2U*Brightness);
    return BSP_ERROR_NONE;
}

/**
 * @brief  Set the brightness value
 * @param  Instance    LCD Instance
 * @param  Brightness [00: Min (black), 100 Max]
 * @retval BSP status
 */
int32_t BSP_LCD_GetBrightness(uint32_t Instance, uint32_t *Brightness)
{
    return BSP_ERROR_NONE;
}
