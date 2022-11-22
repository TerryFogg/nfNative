#pragma once
//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include <WireProtocol.h>
#include <WireProtocol_Message.h>
#include <nanoHAL_v2.h>
#include <tx_api.h>

void InitWireProtocolCommunications();
int wp_ReadBytes(uint8_t **ptr, uint32_t *size, uint32_t wait_time);
bool wp_WriteBytes(uint8_t *ptr, uint16_t size);

