//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2021 STMicroelectronics.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include "TargetFeatures.h"
 #include "stm32h7xx_ll_bus.h"

void Initialize_CRC()
{

    // At boot time (reset) the following are the defaults
    // that may need to be changed
    // LL_CRC_DEFAULT_CRC32_POLY
    // LL_CRC_POLYLENGTH_32B
    // LL_CRC_DEFAULT_CRC_INITVALUE
    // LL_CRC_INDATA_REVERSE_NONE
    // LL_CRC_OUTDATA_REVERSE_NONE
    // Enable peripheral clock for CRC
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_CRC);
}
