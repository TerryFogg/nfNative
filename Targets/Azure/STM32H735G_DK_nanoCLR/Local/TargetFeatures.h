#pragma once
//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2021 STMicroelectronics.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include "stdio.h"

#ifdef __cplusplus
extern "C" {
#endif

void Initialize_Graphics(void);
void Initialize_Audio();
void Initialize_microSD();
void Initialize_USB();
void Initialize_Ethernet();
void Initialize_CRC();
void Initialize_FDCAN();

#ifdef __cplusplus
}
#endif
