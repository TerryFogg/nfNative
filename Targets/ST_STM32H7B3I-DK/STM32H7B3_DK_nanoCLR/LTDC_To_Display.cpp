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

#include "stm32h7b3i_discovery.h"
#include "stm32h7b3i_discovery_lcd.h"
#include "stm32h7b3i_discovery_sdram.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_dma2d.h"


#define LAYER_SIZE_X 			240
#define LAYER_SIZE_Y			160
#define LAYER_BYTE_PER_PIXEL	2 /* RGB565 format */

void OnError_Handler(uint32_t condition);


struct DisplayInterface g_DisplayInterface;
extern CLR_UINT32 GraphicsVideoFrameBufferBegin;     // Framebuffer set externally
// Default to landscape
CLR_UINT32 lcd_x_size = 480;
CLR_UINT32 lcd_y_size = 272;


void DisplayInterface::Initialize(DisplayInterfaceConfig &config)
{
    BSP_LCD_Init();
    ClearFrameBuffer();
    return;
}

void DisplayInterface::GetTransferBuffer(CLR_UINT8 * &TransferBuffer, CLR_UINT32 &TransferBufferSize)
{
    
    //     *(__IO uint16_t*)(hlcd_ltdc.LayerCfg[Lcd_Ctx[Instance].ActiveLayer].FBStartAdress + (2U*((Ypos*Lcd_Ctx[Instance].XSize) + Xpos))) = (uint16_t)Color;

TransferBuffer = (CLR_UINT8 *)&GraphicsVideoFrameBufferBegin;
    TransferBufferSize = (lcd_x_size * lcd_y_size * 2);
}

void DisplayInterface::ClearFrameBuffer()
{
    CLR_UINT32 *frameBuffer = (CLR_UINT32 *)&GraphicsVideoFrameBufferBegin;
    CLR_INT32 DataCount32 = (lcd_x_size * lcd_y_size * 2) / 4;
    for (CLR_INT32 i = 0; i < DataCount32; i++)
    {
        // Note: Write out 4 byte ints measured to be faster than bytes (* 3.5-4 times)
        *frameBuffer = 0;
        frameBuffer++;
    }
}

void DisplayInterface::WriteToFrameBuffer(
    CLR_UINT8 command,
    CLR_UINT8 data[],
    CLR_UINT32 dataCount,
    CLR_UINT32 frameOffset)
{
    UNUSED(command);

    CLR_UINT16 *pFrameBuffer = (CLR_UINT16 *)&GraphicsVideoFrameBufferBegin;
    pFrameBuffer += frameOffset;

    CLR_UINT16 *p16Data = (CLR_UINT16 *)&data[0];
    dataCount /= 2;    // copy 16 bits

    for(CLR_UINT32 i = 0 ; i < dataCount ; i++)
    {
        *pFrameBuffer++ = *p16Data++;
    }
}

void DisplayInterface::DisplayBacklight(bool on)
{
}

void DisplayInterface::SendCommand(CLR_UINT8 NbrParams, ...)
{
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


/**
  * @brief DMA2D configuration.
  * @note  This function Configure tha DMA2D peripheral :
  *        1) Configure the transfer mode : memory to memory
  *        2) Configure the output color mode as RGB565
  *        3) Configure the transfer from FLASH to SRAM   
  *        4) Configure the data size : 240x160 (pixels)  
  * @retval
  *  None
  */
static void DMA2D_Config(void)
{
    /* Configure the DMA2D Color Mode */  
    LL_DMA2D_SetOutputColorMode(DMA2D, LL_DMA2D_OUTPUT_MODE_RGB565);
  
    /* Foreground Configuration:     */
    /* Set Alpha value to full opaque */
    LL_DMA2D_FGND_SetAlpha(DMA2D, 0xFF);
    /* Foreground layer format is RGB565 (16 bpp) */
    LL_DMA2D_FGND_SetColorMode(DMA2D, LL_DMA2D_INPUT_MODE_RGB565);
}

/**
  * @brief  On Error Handler on condition TRUE.
  * @param  condition : Can be TRUE or FALSE
  * @retval None
  */
void OnError_Handler(uint32_t condition)
{
    if (condition)
    {
        BSP_LED_On(LED3);
        while (1) {
            ;
        } /* Blocking on error */
    }
}

//++++++++++++++++++++++++++++++++++++++
 
#include "stm32h7b3i_discovery_lcd.h"
#include "stm32h7b3i_discovery_bus.h"
#include "stm32h7b3i_discovery_sdram.h"


/* Timer handler declaration */
static TIM_HandleTypeDef hlcd_tim;
DMA2D_HandleTypeDef hlcd_dma2d;
LTDC_HandleTypeDef  hlcd_ltdc;
BSP_LCD_Ctx_t       Lcd_Ctx;

int32_t BSP_LCD_Init()
{
   
        
    uint32_t Orientation = LCD_ORIENTATION_LANDSCAPE;
    uint32_t Width = LCD_DEFAULT_WIDTH;
    uint32_t Height = LCD_DEFAULT_HEIGHT;
    int32_t ret = BSP_ERROR_NONE;
    uint32_t ltdc_pixel_format = LTDC_PIXEL_FORMAT_RGB565;
    MX_LTDC_LayerConfig_t config;

    Lcd_Ctx.BppFactor = 2U;

    /* Store pixel format, xsize and ysize information */
    Lcd_Ctx.PixelFormat = ltdc_pixel_format;
    Lcd_Ctx.XSize  = Width;
    Lcd_Ctx.YSize  = Height;

    /* Initializes peripherals instance value */
    hlcd_ltdc.Instance = LTDC;
    hlcd_dma2d.Instance = DMA2D;

    GPIO_InitTypeDef  gpio_init_structure;

    /** Enable the LTDC clock */
    __HAL_RCC_LTDC_CLK_ENABLE();

    /* Enable GPIOs clock */
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOJ_CLK_ENABLE();
    __HAL_RCC_GPIOK_CLK_ENABLE();

    /*** LTDC Pins configuration ***/
    /* GPIO I configuration */
    gpio_init_structure.Pin       = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
    gpio_init_structure.Pull      = GPIO_NOPULL;
    gpio_init_structure.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio_init_structure.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOI, &gpio_init_structure);

    /* GPIO J configuration */
    gpio_init_structure.Pin       = GPIO_PIN_All;
    gpio_init_structure.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOJ, &gpio_init_structure);
    /* GPIO K configuration */
    gpio_init_structure.Pin       = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    gpio_init_structure.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOK, &gpio_init_structure);


    /** Toggle Sw reset of LTDC IP */
    __HAL_RCC_LTDC_FORCE_RESET();
    __HAL_RCC_LTDC_RELEASE_RESET();

    LCD_DISP_CTRL_GPIO_CLK_ENABLE();
    LCD_BL_CTRL_GPIO_CLK_ENABLE();
    LCD_DISP_EN_GPIO_CLK_ENABLE();

    // LCD_DISP_EN GPIO configuration
    gpio_init_structure.Pin       = LCD_DISP_EN_PIN;                // LCD_DISP pin has to be manually controlled
    gpio_init_structure.Mode      = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(LCD_DISP_EN_GPIO_PORT, &gpio_init_structure);

    /* LCD_DISP_CTRL GPIO configuration */
    gpio_init_structure.Pin       = LCD_DISP_CTRL_PIN;     // LCD_DISP pin has to be manually controlled
    gpio_init_structure.Mode      = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(LCD_DISP_CTRL_GPIO_PORT, &gpio_init_structure);

    /* LCD_BL_CTRL GPIO configuration */
    gpio_init_structure.Pin       = LCD_BL_CTRL_PIN;     // LCD_BL_CTRL pin has to be manually controlled
    gpio_init_structure.Mode      = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(LCD_BL_CTRL_GPIO_PORT, &gpio_init_structure);

    HAL_Delay(30);


    /* De-assert display enable LCD_DISP_EN pin */
    HAL_GPIO_WritePin(LCD_DISP_EN_GPIO_PORT, LCD_DISP_EN_PIN, GPIO_PIN_RESET);

    /* Assert display enable LCD_DISP_CTRL pin */
    HAL_GPIO_WritePin(LCD_DISP_CTRL_GPIO_PORT, LCD_DISP_CTRL_PIN, GPIO_PIN_SET);

    /* Assert backlight LCD_BL_CTRL pin */
    HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_PORT, LCD_BL_CTRL_PIN, GPIO_PIN_SET);


    // DMA2D - INIT
    /** Enable the DMA2D clock */
    __HAL_RCC_DMA2D_CLK_ENABLE();

    /** Toggle Sw reset of DMA2D IP */
    __HAL_RCC_DMA2D_FORCE_RESET();
    __HAL_RCC_DMA2D_RELEASE_RESET();


    /* RK043FN48H LCD clock configuration */
    /* LCD clock configuration */
    /* PLL3_VCO Input = HSE_VALUE/PLL3M = 4 Mhz */
    /* PLL3_VCO Output = PLL3_VCO Input * PLL3N = 800 Mhz */
    /* PLLLCDCLK = PLL3_VCO Output/PLL3R = 800/83 = 9.63 Mhz */
    /* LTDC clock frequency = PLLLCDCLK = 9.63 Mhz */
    RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
    PeriphClkInitStruct.PeriphClockSelection   = RCC_PERIPHCLK_LTDC;
    PeriphClkInitStruct.PLL3.PLL3M = 6;
    PeriphClkInitStruct.PLL3.PLL3N = 200;
    PeriphClkInitStruct.PLL3.PLL3P = 10;
    PeriphClkInitStruct.PLL3.PLL3Q = 10;
    PeriphClkInitStruct.PLL3.PLL3R = 83;
    PeriphClkInitStruct.PLL3.PLL3VCOSEL = 0;
    PeriphClkInitStruct.PLL3.PLL3FRACN = 0;

    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
     
    // Init the ltdc
        
    hlcd_ltdc.Instance = LTDC;
    hlcd_ltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
    hlcd_ltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
    hlcd_ltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
    hlcd_ltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;

    hlcd_ltdc.Init.HorizontalSync     = RK043FN48H_HSYNC - 1U;
    hlcd_ltdc.Init.AccumulatedHBP     = (RK043FN48H_HSYNC + (RK043FN48H_HBP - 11U) - 1U);
    hlcd_ltdc.Init.AccumulatedActiveW = RK043FN48H_HSYNC + Width + RK043FN48H_HBP - 1U;
    hlcd_ltdc.Init.TotalWidth         = RK043FN48H_HSYNC + Width + (RK043FN48H_HBP - 11U) + RK043FN48H_HFP - 1U;
    hlcd_ltdc.Init.VerticalSync       = RK043FN48H_VSYNC - 1U;
    hlcd_ltdc.Init.AccumulatedVBP     = RK043FN48H_VSYNC + RK043FN48H_VBP - 1U;
    hlcd_ltdc.Init.AccumulatedActiveH = RK043FN48H_VSYNC + Height + RK043FN48H_VBP - 1U;
    hlcd_ltdc.Init.TotalHeigh         = RK043FN48H_VSYNC + Height + RK043FN48H_VBP + RK043FN48H_VFP - 1U;

    hlcd_ltdc.Init.Backcolor.Blue  = 0xFF;
    hlcd_ltdc.Init.Backcolor.Green = 0xFF;
    hlcd_ltdc.Init.Backcolor.Red   = 0xFF;

    HAL_LTDC_Init(&hlcd_ltdc);
        

    // Configure default LTDC Layer 0
config.X0          = 0;
    config.X1          = Width;
    config.Y0          = 0;
    config.Y1          = Height;
    config.PixelFormat = ltdc_pixel_format;
    config.Address     = LCD_LAYER_0_ADDRESS;

    LTDC_LayerCfgTypeDef pLayerCfg;

    pLayerCfg.WindowX0 = config.X0;
    pLayerCfg.WindowX1 = config.X1;
    pLayerCfg.WindowY0 = config.Y0;
    pLayerCfg.WindowY1 = config.Y1;
    pLayerCfg.PixelFormat = config.PixelFormat;
    pLayerCfg.Alpha = 255;
    pLayerCfg.Alpha0 = 0;
    pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
    pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
    pLayerCfg.FBStartAdress = config.Address;
    pLayerCfg.ImageWidth = (config.X1 - config.X0);
    pLayerCfg.ImageHeight = (config.Y1 - config.Y0);
    pLayerCfg.Backcolor.Blue = 0;
    pLayerCfg.Backcolor.Green = 0;
    pLayerCfg.Backcolor.Red = 0;
    HAL_LTDC_ConfigLayer(&hlcd_ltdc, &pLayerCfg, 0);

    TIM_OC_InitTypeDef LCD_TIM_Config;

    /* Timer_Clock = 2 x  APB1_clock = 280 MHz */
    /* PWM_freq = Timer_Clock /(Period x (Prescaler + 1))*/
    /* PWM_freq = 280 MHz /(200 x (27 + 1)) = 50000 Hz*/
    hlcd_tim.Instance = LCD_TIMx;

   
    GPIO_InitTypeDef gpio_init_structure1;
    LCD_BL_CTRL_GPIO_CLK_ENABLE();
    /* TIMx Peripheral clock enable */
    LCD_TIMx_CLK_ENABLE();

    /* Timer channel configuration */
    gpio_init_structure1.Mode      = GPIO_MODE_AF_PP;
    gpio_init_structure1.Pull      = GPIO_NOPULL;
    gpio_init_structure1.Speed     = GPIO_SPEED_FREQ_MEDIUM;
    gpio_init_structure1.Alternate = LCD_TIMx_CHANNEL_AF;
    gpio_init_structure1.Pin       = LCD_BL_CTRL_PIN; /* BL_CTRL */

    HAL_GPIO_Init(LCD_BL_CTRL_GPIO_PORT, &gpio_init_structure1);
    hlcd_tim.Init.Prescaler         = LCD_TIMX_PRESCALER_VALUE;
    hlcd_tim.Init.Period            = LCD_TIMX_PERIOD_VALUE - 1U;
    hlcd_tim.Init.ClockDivision     = 0;
    hlcd_tim.Init.CounterMode       = TIM_COUNTERMODE_UP;
    hlcd_tim.Init.RepetitionCounter = 0;
    hlcd_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    (void)HAL_TIM_PWM_Init(&hlcd_tim);

    /* Common configuration for all channels */
    LCD_TIM_Config.OCMode       = TIM_OCMODE_PWM1;
    LCD_TIM_Config.OCPolarity   = TIM_OCPOLARITY_HIGH;
    LCD_TIM_Config.OCFastMode   = TIM_OCFAST_DISABLE;
    LCD_TIM_Config.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    LCD_TIM_Config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    LCD_TIM_Config.OCIdleState  = TIM_OCIDLESTATE_RESET;

    /* Set the default pulse value for channel: 50% duty cycle */
    LCD_TIM_Config.Pulse = 100;

    (void)HAL_TIM_PWM_ConfigChannel(&hlcd_tim, &LCD_TIM_Config, LCD_TIMx_CHANNEL);

    /* Start PWM Timer channel */
    (void)HAL_TIM_PWM_Start(&hlcd_tim, LCD_TIMx_CHANNEL);
    
        

    /* By default the reload is activated and executed immediately */
    Lcd_Ctx.ReloadEnable = 1U;

    return BSP_ERROR_NONE;
}

int32_t BSP_LCD_SetBrightness(uint32_t Instance, uint32_t Brightness)
{
    __HAL_TIM_SET_COMPARE(&hlcd_tim, LCD_TIMx_CHANNEL, 2U*Brightness);
    Lcd_Ctx.Brightness = Brightness;
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
    *Brightness = Lcd_Ctx.Brightness;
    return BSP_ERROR_NONE;
}


//int32_t BSP_LCD_DisplayOn(uint32_t Instance)
//{
//    GPIO_InitTypeDef gpio_init_structure;
//
//    __HAL_LTDC_ENABLE(&hlcd_ltdc);
//
//    /* Assert LCD_DISP_EN pin */
//    HAL_GPIO_WritePin(LCD_DISP_CTRL_GPIO_PORT, LCD_DISP_CTRL_PIN, GPIO_PIN_SET);
//    /* Assert LCD_BL_CTRL pin */
//    HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_PORT, LCD_BL_CTRL_PIN, GPIO_PIN_SET);
//
//    gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
//    gpio_init_structure.Pull      = GPIO_NOPULL;
//    gpio_init_structure.Speed     = GPIO_SPEED_FREQ_MEDIUM;
//    gpio_init_structure.Alternate = LCD_TIMx_CHANNEL_AF;
//    gpio_init_structure.Pin       = LCD_BL_CTRL_PIN; /* BL_CTRL */
//    HAL_GPIO_Init(LCD_BL_CTRL_GPIO_PORT, &gpio_init_structure);
//
//    return BSP_ERROR_NONE;
//}
//int32_t BSP_LCD_DisplayOff(uint32_t Instance)
//{
//    GPIO_InitTypeDef gpio_init_structure;
//    __HAL_LTDC_DISABLE(&hlcd_ltdc);
//    gpio_init_structure.Mode      = GPIO_MODE_OUTPUT_PP;
//    gpio_init_structure.Pull      = GPIO_NOPULL;
//    gpio_init_structure.Speed     = GPIO_SPEED_FREQ_MEDIUM;
//    gpio_init_structure.Pin       = LCD_BL_CTRL_PIN; /* BL_CTRL */
//    HAL_GPIO_Init(LCD_BL_CTRL_GPIO_PORT, &gpio_init_structure);
//
//    /* Assert LCD_DISP_EN pin */
//    HAL_GPIO_WritePin(LCD_DISP_CTRL_GPIO_PORT, LCD_DISP_CTRL_PIN, GPIO_PIN_RESET);
//    /* Assert LCD_BL_CTRL pin */
//    HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_PORT, LCD_BL_CTRL_PIN, GPIO_PIN_RESET);
//
//    return BSP_ERROR_NONE;
//}
