//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include "BoardInit.h"
#include "Debug_To_Display.h"
#include "wpUSART_Communications.h"
#include <nanoHAL_v2.h>
#include <nanoPAL_BlockStorage.h>

int main(void)
{

    HardFaultReporting *data = &g_HardFault;

    BoardInit();

    bool WaitForDebuggerRequesed = BoardUserButton_Pressed();

    if (WaitForDebuggerRequesed || g_HardFault.count != 0)
    {
        WaitForDebuggerRequesed = true;

        lcd_printf("\f");
        lcd_printf("      |-------------------------------------------------|\n");
        if (g_HardFault.count != 0)
        {
            lcd_printf("      | HARD Fault recorded                             |\n");
            lcd_printf("      | ...................                             |\n");
        }
        lcd_printf("      |                                                 |\n");
        lcd_printf("      | Waiting for the debugger                        |\n");
        lcd_printf("      |                                                 |\n");
        lcd_printf("      |-------------------------------------------------|\n");

        g_HardFault.count = 0;
    }

    Startup_Rtos(WaitForDebuggerRequesed);
}
