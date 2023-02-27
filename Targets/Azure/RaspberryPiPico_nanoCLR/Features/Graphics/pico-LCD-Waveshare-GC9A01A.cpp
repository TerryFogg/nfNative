#ifdef ROUND_DISPLAY
//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include "Display.h"
#include "DisplayInterface.h"
#include "Graphics.h"
#include "InternalFont.h"

/*
GCA9A01A   
   Display resolution: 240x240：S1-S360

    TFT LCD driver with on-chip full display RAM: 129,600 bytes
    System Interface
        - 8-bits, 9-bits, 12-bits,16-bits, 18-bits interface with 8080-I /8080-II series MCU
        - 6-bits, 12-bits, 16-bits, 18-bits RGB interface with graphic controller
        - 8-bits, 9-bits 24bit Serial Peripheral Interface (SPI) and 2 data lane SPI
    Display mode:
        - Full color mode (Idle mode OFF): 262K-color (selectable color depth mode by software)
    Reduce color mode (Idle mode ON): 8-color
    Power saving mode:
        - Sleep mode
    On chip functions:
        - Timing generator
        - Oscillator
        - DC/DC converter
        - Dot/column inversion
    Low -power consumption architecture
*/
enum GCA9A01A : CLR_UINT8
{
    Enter_Sleep = 0x10,
    Sleep_OUT = 0x11,
    Display_Inversion_Off = 0x20,
    Display_Inversion_On = 0x21,
    Display_OFF = 0x28,
    Display_ON = 0x29,
    Column_Address_Set = 0x2A,
    Page_Address_Set = 0x2B,
    Memory_Write = 0x2C,
    Tearing_Effect_Line_on = 0x35,
    Memory_Access_Control = 0x36,
    Idle_Mode_OFF = 0x38,
    Idle_Mode_ON = 0x39,

    Pixel_Format_Set = 0x3A,

    Memory_Write_Continue = 0x3C,

    UNDOCUMENTED_0x62 = 0x62,
    UNDOCUMENTED_0x63 = 0x63,
    UNDOCUMENTED_0x64 = 0x64,
    UNDOCUMENTED_0x66 = 0x66,
    UNDOCUMENTED_0x67 = 0x67,
    UNDOCUMENTED_0x70 = 0x70,
    UNDOCUMENTED_0x74 = 0x74,
    UNDOCUMENTED_0x84 = 0x84,
    UNDOCUMENTED_0x85 = 0x85,
    UNDOCUMENTED_0x86 = 0x86,
    UNDOCUMENTED_0x87 = 0x87,
    UNDOCUMENTED_0x88 = 0x88,
    UNDOCUMENTED_0x89 = 0x89,
    UNDOCUMENTED_0x8A = 0x8A,
    UNDOCUMENTED_0x8B = 0x8B,
    UNDOCUMENTED_0x8C = 0x8C,
    UNDOCUMENTED_0x8D = 0x8D,
    UNDOCUMENTED_0x8E = 0x8E,
    UNDOCUMENTED_0x8F = 0x8F,
    UNDOCUMENTED_0x90 = 0x90,
    UNDOCUMENTED_0x98 = 0x98,
    UNDOCUMENTED_0xAE = 0xAE,
    DISPLAY_FUNCION_CONTROL = 0xB6,
    UNDOCUMENTED_0xBC = 0xBC,
    UNDOCUMENTED_0xBD = 0xBD,
    Power_Control_2 = 0xC3,
    Power_Control_3 = 0xC4,
    Power_Control_4 = 0xC9,
    UNDOCUMENTED_0xCD = 0xCD,
    UNDOCUMENTED_0xDF = 0xDF,
    UNDOCUMENTED_0xBE = 0xBE,
    UNDOCUMENTED_0xE1 = 0xE1, // Negative_Voltage_Gamma?
    Frame_Rate = 0xE8,
    UNDOCUMENTED_0xEB = 0xEB, // Power on Sequence?
    UNDOCUMENTED_0xED = 0xED,
    Inter_Register_Enable2 = 0xEF,
    SET_GAMMA1 = 0xF0,
    SET_GAMMA2 = 0xF1,
    SET_GAMMA3 = 0xF2,
    SET_GAMMA4 = 0xF3,
    Inter_Register_Enable1 = 0xFE,
    UNDOCUMENTED_0xFF = 0xFF
};
enum GCA9A01A_ORIENTATION : CLR_UINT8
{
    MADCTL_MH = 0x04,  // sets the Horizontal Refresh, 0=Left-Right and 1=Right-Left
    MADCTL_BGR = 0x08, // Blue-Green-Red pixel order
    MADCTL_ML = 0x10,  // sets the Vertical Refresh, 0=Top-Bottom and 1=Bottom-Top
    MADCTL_MV = 0x20,  // sets the Row/Column Swap, 0=Normal and 1=Swapped
    MADCTL_MX = 0x40,  // sets the Column Order, 0=Left-Right and 1=Right-Left
    MADCTL_MY = 0x80   // sets the Row Order, 0=Top-Bottom and 1=Bottom-Top
};

struct DisplayDriver g_DisplayDriver;
extern DisplayInterface g_DisplayInterface;
int lcd_width;
int lcd_height;
bool DisplayDriver::Initialize(DisplayInterfaceConfig &config)
{
    lcd_width = config.Screen.width;
    lcd_height = config.Screen.height;

    SetupDisplayAttributes();
    g_DisplayInterface.SendCommand(1, Inter_Register_Enable2);
    g_DisplayInterface.SendCommand(2, UNDOCUMENTED_0xEB, 0x14);
    g_DisplayInterface.SendCommand(1, Inter_Register_Enable1);
    g_DisplayInterface.SendCommand(1, Inter_Register_Enable2);
    g_DisplayInterface.SendCommand(2, UNDOCUMENTED_0xEB, 0x14);
    g_DisplayInterface.SendCommand(2, UNDOCUMENTED_0x84, 0x40);
    g_DisplayInterface.SendCommand(2, UNDOCUMENTED_0x85, 0xFF);
    g_DisplayInterface.SendCommand(2, UNDOCUMENTED_0x86, 0xFF);
    g_DisplayInterface.SendCommand(2, UNDOCUMENTED_0x87, 0xFF);
    g_DisplayInterface.SendCommand(2, UNDOCUMENTED_0x88, 0x0A);
    g_DisplayInterface.SendCommand(2, UNDOCUMENTED_0x89, 0x21);
    g_DisplayInterface.SendCommand(2, UNDOCUMENTED_0x8A, 0x00);
    g_DisplayInterface.SendCommand(2, UNDOCUMENTED_0x8B, 0x80);
    g_DisplayInterface.SendCommand(2, UNDOCUMENTED_0x8C, 0x01);
    g_DisplayInterface.SendCommand(2, UNDOCUMENTED_0x8D, 0x01);
    g_DisplayInterface.SendCommand(2, UNDOCUMENTED_0x8E, 0xFF);
    g_DisplayInterface.SendCommand(2, UNDOCUMENTED_0x8F, 0xFF);
    g_DisplayInterface.SendCommand(3, DISPLAY_FUNCION_CONTROL, 0x00, 0x20);
    g_DisplayInterface.SendCommand(2, Pixel_Format_Set, 0x55);
    g_DisplayInterface.SendCommand(5, UNDOCUMENTED_0x90, 0x08, 0x08, 0x08, 0x08);
    g_DisplayInterface.SendCommand(2, UNDOCUMENTED_0xBD, 0x06);
    g_DisplayInterface.SendCommand(2, UNDOCUMENTED_0xBC, 0x00);
    g_DisplayInterface.SendCommand(4, UNDOCUMENTED_0xFF, 0x60, 0x01, 0x04);
    g_DisplayInterface.SendCommand(2, Power_Control_2, 0x13);
    g_DisplayInterface.SendCommand(2, Power_Control_3, 0x13);
    g_DisplayInterface.SendCommand(2, Power_Control_4, 0x22);
    g_DisplayInterface.SendCommand(2, UNDOCUMENTED_0xBE, 0x11);
    g_DisplayInterface.SendCommand(3, UNDOCUMENTED_0xE1, 0x10, 0x0E);
    g_DisplayInterface.SendCommand(4, UNDOCUMENTED_0xDF, 0x21, 0x0C, 0x02);
    g_DisplayInterface.SendCommand(7, SET_GAMMA1, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A);
    g_DisplayInterface.SendCommand(7, SET_GAMMA2, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F);
    g_DisplayInterface.SendCommand(7, SET_GAMMA3, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A);
    g_DisplayInterface.SendCommand(7, SET_GAMMA4, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F);
    g_DisplayInterface.SendCommand(3, UNDOCUMENTED_0xED, 0x1B, 0x0B);
    g_DisplayInterface.SendCommand(2, UNDOCUMENTED_0xAE, 0x77);
    g_DisplayInterface.SendCommand(2, UNDOCUMENTED_0xCD, 0x63);
    g_DisplayInterface.SendCommand(10, UNDOCUMENTED_0x70, 0x07, 0x07, 0x04, 0x0E, 0x0F, 0x09, 0x07, 0x08, 0x03);
    g_DisplayInterface.SendCommand(2, Frame_Rate, 0x34);
    g_DisplayInterface.SendCommand(13, UNDOCUMENTED_0x62, 0x18, 0x0D, 0x71, 0xED, 0x70, 0X70, 0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70);
    g_DisplayInterface.SendCommand(13, UNDOCUMENTED_0x63, 0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, 0x18, 0x13, 0x71, 0xF3, 0x70, 0x70);
    g_DisplayInterface.SendCommand(8, UNDOCUMENTED_0x64, 0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07);
    g_DisplayInterface.SendCommand(11, UNDOCUMENTED_0x66, 0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00);
    g_DisplayInterface.SendCommand(11, UNDOCUMENTED_0x67, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98);
    g_DisplayInterface.SendCommand(8, UNDOCUMENTED_0x74, 0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00);
    g_DisplayInterface.SendCommand(3, UNDOCUMENTED_0x98, 0x3E, 0x07);
    g_DisplayInterface.SendCommand(1, Tearing_Effect_Line_on);

    SetDefaultOrientation();

    g_DisplayInterface.SendCommand(1, Display_Inversion_On);
    g_DisplayInterface.SendCommand(1, Sleep_OUT);
    OS_DELAY(120);
    g_DisplayInterface.SendCommand(1, Display_ON);
    OS_DELAY(20);

    g_DisplayInterface.DisplayBacklight(true);
    Clear();

    return true;
}
void DisplayDriver::SetupDisplayAttributes()
{
    // Define the LCD/TFT resolution
    Attributes.LongerSide = lcd_width;
    Attributes.ShorterSide = lcd_height;
    Attributes.PowerSave = PowerSaveState::NORMAL;
    Attributes.BitsPerPixel = 16;
    g_DisplayInterface.GetTransferBuffer(Attributes.TransferBuffer, Attributes.TransferBufferSize);
    return;
}
bool DisplayDriver::ChangeOrientation(DisplayOrientation orientation)
{
    switch (orientation)
    {
    case PORTRAIT:
        Attributes.Height = Attributes.ShorterSide;
        Attributes.Width = Attributes.LongerSide;
        g_DisplayInterface.SendCommand(2, Memory_Access_Control, MADCTL_BGR);
        break;
    case PORTRAIT180:
        Attributes.Height = Attributes.ShorterSide;
        Attributes.Width = Attributes.LongerSide;
        g_DisplayInterface.SendCommand(2, Memory_Access_Control, (MADCTL_MY | MADCTL_MX | MADCTL_BGR));
        break;
    case LANDSCAPE:
        Attributes.Height = Attributes.ShorterSide;
        Attributes.Width = Attributes.LongerSide;
        g_DisplayInterface.SendCommand(2, Memory_Access_Control, (MADCTL_MY | MADCTL_MX | MADCTL_BGR));
        break;
    case LANDSCAPE180:
        Attributes.Height = Attributes.ShorterSide;
        Attributes.Width = Attributes.LongerSide;
        g_DisplayInterface.SendCommand(2, Memory_Access_Control, (MADCTL_MY | MADCTL_BGR));
        break;
    }
    return true;
}
void DisplayDriver::SetDefaultOrientation()
{
    ChangeOrientation(PORTRAIT);
}
bool DisplayDriver::Uninitialize()
{
    Clear();
    return true;
}
void DisplayDriver::PowerSave(PowerSaveState powerState)
{
    switch (powerState)
    {
    default:
        // illegal fall through to Power on
    case PowerSaveState::NORMAL:
        break;
    case PowerSaveState::SLEEP:
        break;
    }
    return;
}
void DisplayDriver::Clear()
{
    // Clear the transfer buffer
    memset(Attributes.TransferBuffer, 0xFF, Attributes.TransferBufferSize);
    
    // Clear the GA9A01A controller frame
    SetWindow(0, 0, Attributes.Width - 1, Attributes.Height - 1);

    int totalBytesToClear = Attributes.Width * Attributes.Height * 2;
    int fullTransferBuffersCount = totalBytesToClear / Attributes.TransferBufferSize;
    int remainderTransferBuffer = totalBytesToClear % Attributes.TransferBufferSize;

    g_DisplayInterface.SendCommand(1, Memory_Write);

    for (int i = 0; i < fullTransferBuffersCount; i++)
    {
        g_DisplayInterface.SendData(Attributes.TransferBuffer, Attributes.TransferBufferSize);
    }
    if (remainderTransferBuffer > 0)
    {
        g_DisplayInterface.SendData(Attributes.TransferBuffer, remainderTransferBuffer);
    }
    
}
void DisplayDriver::DisplayBrightness(CLR_INT16 brightness)
{
    _ASSERTE(brightness >= 0 && brightness <= 100);
    //    g_DisplayInterface.SendCommand(2, Write_Display_Brightness, (CLR_UINT8)brightness);
    return;
}
bool DisplayDriver::SetWindow(CLR_INT16 x1, CLR_INT16 y1, CLR_INT16 x2, CLR_INT16 y2)
{
    CLR_UINT8 Column_Address_Set_Data[4];
    Column_Address_Set_Data[0] = 0;
    Column_Address_Set_Data[1] = x1 & 0xFF;
    Column_Address_Set_Data[2] = (x2 >> 8) & 0xFF;
    Column_Address_Set_Data[3] = (x2 & 0xFF);

    CLR_UINT8 Page_Address_Set_Data[4];
    Page_Address_Set_Data[0] = 0x00;
    Page_Address_Set_Data[1] = y1 & 0xFF;
    Page_Address_Set_Data[2] = (y2 >> 8) & 0xFF;
    Page_Address_Set_Data[3] = (y2 & 0xFF);

    g_DisplayInterface.SendCommand(
        5,
        Column_Address_Set,
        Column_Address_Set_Data[0],
        Column_Address_Set_Data[1],
        Column_Address_Set_Data[2],
        Column_Address_Set_Data[3]);

    g_DisplayInterface.SendCommand(
        5,
        Page_Address_Set,
        Page_Address_Set_Data[0],
        Page_Address_Set_Data[1],
        Page_Address_Set_Data[2],
        Page_Address_Set_Data[3]);

    return true;
}
void DisplayDriver::BitBlt(int x, int y, int width, int height, int stride, int screenX, int screenY, CLR_UINT32 data[])
{
    // 16 bit colour  RRRRRGGGGGGBBBBB mode 565

    ASSERT((x >= 0) && ((x + width) <= Attributes.Width));
    ASSERT((y >= 0) && ((y + height) <= Attributes.Height));

    SetWindow(x, y, (x + width - 1), (y + height - 1));

    CLR_UINT16 *StartOfLine_src = (CLR_UINT16 *)&data[0];

    // Position to offset in data[] for start of window
    CLR_UINT16 offset = (y * Attributes.Width) + x;
    StartOfLine_src += offset;

    CLR_UINT8 *transferBufferIndex = Attributes.TransferBuffer;
    CLR_UINT32 transferBufferCount = Attributes.TransferBufferSize;
    CLR_UINT8 command = Memory_Write;

    while (height--)
    {
        CLR_UINT16 *src;
        int xCount;

        src = StartOfLine_src;
        xCount = width;

        while (xCount--)
        {
            CLR_UINT16 data = *src++;
            // Swap bytes
            *transferBufferIndex++ = (data >> 8);
            *transferBufferIndex++ = data & 0xff;
            transferBufferCount -= 2;

            // Send over SPI if no room for another 2 bytes
            if (transferBufferCount < 1)
            {
                // Transfer buffer full, send it
                g_DisplayInterface.WriteToFrameBuffer(
                    command,
                    Attributes.TransferBuffer,
                    (Attributes.TransferBufferSize - transferBufferCount));

                // Reset transfer ptrs/count
                transferBufferIndex = Attributes.TransferBuffer;
                transferBufferCount = Attributes.TransferBufferSize;
                command = Memory_Write_Continue;
            }
        }
        // Next row in data[]
        StartOfLine_src += Attributes.Width;
    }

    // Send remaining data in transfer buffer to SPI
    if (transferBufferCount < Attributes.TransferBufferSize)
    {
        // Transfer buffer full, send it
        g_DisplayInterface.WriteToFrameBuffer(
            command,
            Attributes.TransferBuffer,
            (Attributes.TransferBufferSize - transferBufferCount));
    }

    return;
}
void DisplayDriver::SendDataDirect(CLR_INT16 x, CLR_INT16 y, CLR_INT16 width, CLR_INT16 height, CLR_UINT16 data[])
{
    CLR_UINT16 numberOfBytes = width * height * 2; // 2 bytes per pixel
    SetWindow(x, y, (x + width - 1), (y + height - 1));
    g_DisplayInterface.WriteToFrameBuffer(Memory_Write, (CLR_UINT8 *)&data[0], numberOfBytes);
    return;
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
    CLR_UINT16 fontdata[8 * 8];
    int width = 8;
    int height = 8;

    // convert to LCD pixel coordinates
    row *= width;
    col *= height;

    if (row > (Attributes.Width - width))
        return;
    if (col > (Attributes.Height - height))
        return;

    const CLR_UINT8 *font = InternalFont::Font_GetGlyph(c);
    int i = 0;
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            CLR_UINT16 val;
            // the font data is mirrored
            // if ((font[y] & (1 << (width - x - 1))) != 0)
            CLR_UINT8 ff = font[y];
            int x1 = (1 << (width - x - 1));

            if ((ff & x1) != 0)
            {
                val = 0xFFFF; // Text Colour => all bits one's - should be white
            }
            else
            {
                val = 0x0000; // background  => all bits zero - should be black
            }
            fontdata[i] = val;
            i++;
        }
    }

    SendDataDirect(row, col, width, height, fontdata);

} // done

#endif

