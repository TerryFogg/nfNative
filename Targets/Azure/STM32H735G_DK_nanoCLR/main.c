//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <nanoHAL_v2.h>
#include <nanoPAL_BlockStorage.h>
#include "wpUSART_Communications.h"
#include "BoardInit.h"

int main(void)
{
    BoardInit();
    Startup_Rtos();
}