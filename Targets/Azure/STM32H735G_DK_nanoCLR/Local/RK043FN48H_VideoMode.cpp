//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include "Graphics.h"
#include "DisplayInterface.h"
#include "Display.h"
#include "InternalFont.h"

//
// RK043FN02H 
// Description : 4.3 inch TFT 480 * 272 pixels with LED 
// Backlight and capacitive touch panel
//
//
// This display is a video memory less (I think, little information available) display that relies on the microcontroller sending out the information
// with horizontal and vertical sync
//

struct DisplayDriver g_DisplayDriver;
extern DisplayInterface g_DisplayInterface;
int lcd_with;
int lcd_height;

bool DisplayDriver::Initialize(int initial_lcd_width, int initial_lcd_height)
{
    lcd_with =  initial_lcd_width;
    lcd_height =  initial_lcd_height;
    SetupDisplayAttributes();
    SetDefaultOrientation();

    Clear();
    return 0;
}

void DisplayDriver::SetupDisplayAttributes()
{
    // Define the LCD/TFT resolution
    Attributes.LongerSide = lcd_with;
    Attributes.ShorterSide = lcd_height;
    Attributes.PowerSave = PowerSaveState::NORMAL;
    Attributes.BitsPerPixel = 16;
    g_DisplayInterface.GetTransferBuffer(Attributes.TransferBuffer, Attributes.TransferBufferSize);
    g_DisplayInterface.DisplayBacklight(true);
}

bool DisplayDriver::ChangeOrientation(DisplayOrientation orientation)
{
    switch (orientation)
    {
    case PORTRAIT:
        Attributes.Height = Attributes.LongerSide;
        Attributes.Width = Attributes.ShorterSide;

        break;
    case PORTRAIT180:
        Attributes.Height = Attributes.LongerSide;
        Attributes.Width = Attributes.ShorterSide;
        // Landscape only at the moment
            break;
    case LANDSCAPE:
        Attributes.Height = Attributes.ShorterSide;
        Attributes.Width = Attributes.LongerSide;
        break;
    case LANDSCAPE180:
        Attributes.Height = Attributes.ShorterSide;
        Attributes.Width = Attributes.LongerSide;
        break;
    }
    return true;
}

void DisplayDriver::SetDefaultOrientation()
{
    ChangeOrientation(LANDSCAPE);
}

bool DisplayDriver::Uninitialize()
{
    Clear();
    return TRUE;
}

void DisplayDriver::PowerSave(PowerSaveState powerState)
{
    switch (powerState)
    {
    default:
        // Illegal fall through to Power on
case PowerSaveState::NORMAL :
    //        ??
    break;
    case PowerSaveState::SLEEP:
        //       ??
        break;
    }
    return;
}
void DisplayDriver::Clear()
{
    // Clear Frame buffer
    g_DisplayInterface.ClearFrameBuffer();
}

void DisplayDriver::DisplayBrightness(CLR_INT16 brightness)
{
    _ASSERTE(brightness >= 0 && brightness <= 100);
}

void DisplayDriver::BitBlt(int x, int y, int width, int height, CLR_UINT32 data[])
{
    CLR_UINT32 dataSize = width * height * 2;
    if (dataSize == (Attributes.Width * Attributes.Height * 2))
    {
        // Full screen
        g_DisplayInterface.WriteToFrameBuffer(0, (CLR_UINT8 *)data, dataSize, 0);
    }
    else
    {
        // Partial bitblt
        CLR_UINT16 *p16data = (CLR_UINT16 *)&data[0];
        CLR_UINT32 srcOffset = (y * (CLR_UINT32)Attributes.Width) + x;
        p16data += srcOffset;
        dataSize = width * 2;
        CLR_UINT32 targetOffset = srcOffset;                 // Target 16bit offset in frame
        while (height--)
        {
            g_DisplayInterface.WriteToFrameBuffer(0, (CLR_UINT8 *)p16data, dataSize, targetOffset);
            p16data += Attributes.Width;                    // Next display row in data[]
            targetOffset += Attributes.Width;               // Next offset in target frame
        }
    }
}

void DisplayDriver::SendDataDirect(CLR_INT16 x, CLR_INT16 y, CLR_INT16 width, CLR_INT16 height, CLR_UINT16 data[])
{
    // This driver is written to use the "video mode" of the controller, so we just write to the memory buffer.
    BitBlt(x, y, width, height, (CLR_UINT32 *)data);
}

CLR_INT16 DisplayDriver::PixelsPerWord()
{
    return (32 / Attributes.BitsPerPixel);
}
CLR_INT16 DisplayDriver::WidthInWords()
{
    return ((Attributes.Width + (PixelsPerWord() - 1)) / PixelsPerWord());
}
CLR_INT16 DisplayDriver::SizeInWords()
{
    return (WidthInWords() * Attributes.Height);
}
CLR_INT16 DisplayDriver::SizeInBytes()
{
    return (SizeInWords() * sizeof(CLR_UINT32));
}
void DisplayDriver::WriteChar(unsigned char c, int row, int col)
{
    // convert to LCD pixel coordinates
    int fontHeight = InternalFont::Font_Height();
    int fontWidth = InternalFont::Font_Width();

    int pixelRow = row * fontHeight;
    int pixelCol = col * fontWidth;

    if (pixelRow > (Attributes.Width - fontWidth))
        return;
    if (pixelCol > (Attributes.Height - fontHeight))
        return;

    const CLR_UINT8 *font = InternalFont::Font_GetGlyph(c);

    CLR_UINT16 *ScreenBuffer = (CLR_UINT16 *)Attributes.TransferBuffer;

    for (int y = 0; y < fontHeight; y++)
    {
        for (int x = 0; x < fontWidth; x += 1)
        {
            CLR_UINT16 pixelColour = ((font[y] >> (fontWidth - 1 - x)) & 0x01) ? 0xFFFF : 0x0000;
            ScreenBuffer[(pixelCol + y) * Attributes.Width + (pixelRow + x)] = pixelColour;
        }
    }
    SCB_CleanInvalidateDCache();
}