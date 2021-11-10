
#include "stm32h7b3i_discovery_lcd.h"
#include "stm32h7b3i_discovery_bus.h"
#include "stm32h7b3i_discovery_sdram.h"

//static void LL_ConvertLineToRGB(uint32_t Instance, uint32_t *pSrc, uint32_t *pDst, uint32_t xSize, uint32_t ColorMode);
//
//
//int32_t BSP_LCD_GetPixelFormat(uint32_t Instance, uint32_t *PixelFormat)
//{
//    /* Only RGB565 format is supported */
//    *PixelFormat = Lcd_Ctx.PixelFormat;
//    return BSP_ERROR_NONE;
//}
//int32_t BSP_LCD_SetActiveLayer(uint32_t Instance, uint32_t LayerIndex)
//{
//    Lcd_Ctx.ActiveLayer = LayerIndex;
//    return BSP_ERROR_NONE;
//}
//int32_t BSP_LCD_SetTransparency(uint32_t Instance, uint32_t LayerIndex, uint8_t Transparency)
//{
//    if (Lcd_Ctx.ReloadEnable == 1U)
//    {
//        (void)HAL_LTDC_SetAlpha(&hlcd_ltdc, Transparency, LayerIndex);
//    }
//    else
//    {
//        (void)HAL_LTDC_SetAlpha_NoReload(&hlcd_ltdc, Transparency, LayerIndex);
//    }
//
//    return BSP_ERROR_NONE;
//}
//int32_t BSP_LCD_SetLayerAddress(uint32_t Instance, uint32_t LayerIndex, uint32_t Address)
//{
//    if (Lcd_Ctx.ReloadEnable == 1U)
//    {
//        (void)HAL_LTDC_SetAddress(&hlcd_ltdc, Address, LayerIndex);
//    }
//    else
//    {
//        (void)HAL_LTDC_SetAddress_NoReload(&hlcd_ltdc, Address, LayerIndex);
//    }
//
//    return BSP_ERROR_NONE;
//}
//int32_t BSP_LCD_SetLayerWindow(uint32_t Instance, uint16_t LayerIndex, uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
//{
//    if (Lcd_Ctx.ReloadEnable == 1U)
//    {
//        /* Reconfigure the layer size  and position */
//        (void)HAL_LTDC_SetWindowSize(&hlcd_ltdc, Width, Height, LayerIndex);
//        (void)HAL_LTDC_SetWindowPosition(&hlcd_ltdc, Xpos, Ypos, LayerIndex);
//    }
//    else
//    {
//        /* Reconfigure the layer size and position */
//        (void)HAL_LTDC_SetWindowSize_NoReload(&hlcd_ltdc, Width, Height, LayerIndex);
//        (void)HAL_LTDC_SetWindowPosition_NoReload(&hlcd_ltdc, Xpos, Ypos, LayerIndex);
//    }
//
//    Lcd_Ctx.XSize = Width;
//    Lcd_Ctx.YSize = Height;
//
//    return BSP_ERROR_NONE;
//}
//int32_t BSP_LCD_SetColorKeying(uint32_t Instance, uint32_t LayerIndex, uint32_t Color)
//{
//    if (Lcd_Ctx.ReloadEnable == 1U)
//    {
//        /* Configure and Enable the color Keying for LCD Layer */
//        (void)HAL_LTDC_ConfigColorKeying(&hlcd_ltdc, Color, LayerIndex);
//        (void)HAL_LTDC_EnableColorKeying(&hlcd_ltdc, LayerIndex);
//    }
//    else
//    {
//        /* Configure and Enable the color Keying for LCD Layer */
//        (void)HAL_LTDC_ConfigColorKeying_NoReload(&hlcd_ltdc, Color, LayerIndex);
//        (void)HAL_LTDC_EnableColorKeying_NoReload(&hlcd_ltdc, LayerIndex);
//    }
//
//    return BSP_ERROR_NONE;
//}
//int32_t BSP_LCD_ResetColorKeying(uint32_t Instance, uint32_t LayerIndex)
//{
//    if (Lcd_Ctx.ReloadEnable == 1U)
//    {
//        /* Disable the color Keying for LCD Layer */
//        (void)HAL_LTDC_DisableColorKeying(&hlcd_ltdc, LayerIndex);
//    }
//    else
//    {
//        /* Disable the color Keying for LCD Layer */
//        (void)HAL_LTDC_DisableColorKeying_NoReload(&hlcd_ltdc, LayerIndex);
//    }
//
//    return BSP_ERROR_NONE;
//}
//int32_t BSP_LCD_GetXSize(uint32_t Instance, uint32_t *XSize)
//{
//    *XSize = Lcd_Ctx.XSize;
//    return BSP_ERROR_NONE;
//}
//int32_t BSP_LCD_GetYSize(uint32_t Instance, uint32_t *YSize)
//{
//    *YSize = Lcd_Ctx.YSize;
//    return BSP_ERROR_NONE;
//}
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
//int32_t BSP_LCD_DrawBitmap(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint8_t *pBmp)
//{
//    uint32_t index, width, height, bit_pixel;
//    uint32_t Address;
//    uint32_t input_color_mode;
//    uint8_t *pbmp;
//
//    /* Get bitmap data address offset */
//    index = (uint32_t)pBmp[10] + ((uint32_t)pBmp[11] << 8) + ((uint32_t)pBmp[12] << 16)  + ((uint32_t)pBmp[13] << 24);
//
//    /* Read bitmap width */
//    width = (uint32_t)pBmp[18] + ((uint32_t)pBmp[19] << 8) + ((uint32_t)pBmp[20] << 16)  + ((uint32_t)pBmp[21] << 24);
//
//    /* Read bitmap height */
//    height = (uint32_t)pBmp[22] + ((uint32_t)pBmp[23] << 8) + ((uint32_t)pBmp[24] << 16)  + ((uint32_t)pBmp[25] << 24);
//
//    /* Read bit/pixel */
//    bit_pixel = (uint32_t)pBmp[28] + ((uint32_t)pBmp[29] << 8);
//
//    /* Set the address */
//    Address = hlcd_ltdc.LayerCfg[Lcd_Ctx.ActiveLayer].FBStartAdress + (((Lcd_Ctx.XSize*Ypos) + Xpos)*Lcd_Ctx.BppFactor);
//
//    /* Get the layer pixel format */
//    if ((bit_pixel / 8U) == 4U)
//    {
//        input_color_mode = DMA2D_INPUT_ARGB8888;
//    }
//    else if ((bit_pixel / 8U) == 2U)
//    {
//        input_color_mode = DMA2D_INPUT_RGB565;
//    }
//    else
//    {
//        input_color_mode = DMA2D_INPUT_RGB888;
//    }
//
//    /* Bypass the bitmap header */
//    pbmp = pBmp + (index + (width * (height - 1U) * (bit_pixel / 8U)));
//
//    /* Convert picture to ARGB8888 pixel format */
//    for (index = 0; index < height; index++)
//    {
//        /* Pixel format conversion */
//        LL_ConvertLineToRGB(Instance, (uint32_t *)pbmp, (uint32_t *)Address, width, input_color_mode);
//
//        /* Increment the source and destination buffers */
//        Address +=  (Lcd_Ctx.XSize * Lcd_Ctx.BppFactor);
//        pbmp -= width*(bit_pixel / 8U);
//    }
//
//    return BSP_ERROR_NONE;
//}
//int32_t BSP_LCD_ReadPixel(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t *Color)
//{
//    if (hlcd_ltdc.LayerCfg[Lcd_Ctx.ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_ARGB8888)
//    {
//        /* Read data value from SDRAM memory */
//        *Color = *(__IO uint32_t*)(hlcd_ltdc.LayerCfg[Lcd_Ctx.ActiveLayer].FBStartAdress + (4U*((Ypos*Lcd_Ctx.XSize) + Xpos)));
//    }
//    else /* if((hlcd_ltdc.LayerCfg[layer].PixelFormat == LTDC_PIXEL_FORMAT_RGB565) */
//    {
//        /* Read data value from SDRAM memory */
//        *Color = *(__IO uint16_t*)(hlcd_ltdc.LayerCfg[Lcd_Ctx.ActiveLayer].FBStartAdress + (2U*((Ypos*Lcd_Ctx.XSize) + Xpos)));
//    }
//
//    return BSP_ERROR_NONE;
//}
//int32_t BSP_LCD_WritePixel(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Color)
//{
//    if (hlcd_ltdc.LayerCfg[Lcd_Ctx.ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_ARGB8888)
//    {
//        /* Write data value to SDRAM memory */
//        *(__IO uint32_t*)(hlcd_ltdc.LayerCfg[Lcd_Ctx.ActiveLayer].FBStartAdress + (4U*((Ypos*Lcd_Ctx.XSize) + Xpos))) = Color;
//    }
//    else
//    {
//        /* Write data value to SDRAM memory */
//        *(__IO uint16_t*)(hlcd_ltdc.LayerCfg[Lcd_Ctx.ActiveLayer].FBStartAdress + (2U*((Ypos*Lcd_Ctx.XSize) + Xpos))) = (uint16_t)Color;
//    }
//
//    return BSP_ERROR_NONE;
//}
//
//
///**
//  * @brief  Converts a line to an RGB pixel format.
//  * @param  Instance LCD Instance
//  * @param  pSrc Pointer to source buffer
//  * @param  pDst Output color
//  * @param  xSize Buffer width
//  * @param  ColorMode Input color mode
//  */
//static void LL_ConvertLineToRGB(uint32_t Instance, uint32_t *pSrc, uint32_t *pDst, uint32_t xSize, uint32_t ColorMode)
//{
//    uint32_t output_color_mode;
//
//    switch (Lcd_Ctx.PixelFormat)
//    {
//    case LCD_PIXEL_FORMAT_RGB565:
//        output_color_mode = DMA2D_OUTPUT_RGB565; /* RGB565 */
//        break;
//    case LCD_PIXEL_FORMAT_RGB888:
//    default:
//        output_color_mode = DMA2D_OUTPUT_ARGB8888; /* ARGB8888 */
//        break;
//    }
//
//    /* Configure the DMA2D Mode, Color Mode and output offset */
//    hlcd_dma2d.Init.Mode         = DMA2D_M2M_PFC;
//    hlcd_dma2d.Init.ColorMode    = output_color_mode;
//    hlcd_dma2d.Init.OutputOffset = 0;
//
//    /* Foreground Configuration */
//    hlcd_dma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
//    hlcd_dma2d.LayerCfg[1].InputAlpha = 0xFF;
//    hlcd_dma2d.LayerCfg[1].InputColorMode = ColorMode;
//    hlcd_dma2d.LayerCfg[1].InputOffset = 0;
//
//    hlcd_dma2d.Instance = DMA2D;
//
//    /* DMA2D Initialization */
//    if (HAL_DMA2D_Init(&hlcd_dma2d) == HAL_OK)
//    {
//        if (HAL_DMA2D_ConfigLayer(&hlcd_dma2d, 1) == HAL_OK)
//        {
//            if (HAL_DMA2D_Start(&hlcd_dma2d, (uint32_t)pSrc, (uint32_t)pDst, xSize, 1) == HAL_OK)
//            {
//                /* Polling For DMA transfer */
//                (void)HAL_DMA2D_PollForTransfer(&hlcd_dma2d, 50);
//            }
//        }
//    }
//}
