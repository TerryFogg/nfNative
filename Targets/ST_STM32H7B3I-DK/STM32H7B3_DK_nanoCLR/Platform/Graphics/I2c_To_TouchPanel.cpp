//
// Copyright (c) 2017 The nanoFramework project contributors
// See LICENSE file in the project root for full license information.
//



#include "nanoCLR_Types.h"
#include <nanoPAL.h>
#include <target_platform.h>
#include "TouchInterface.h"
#include "Debug_To_Display.h"
#include "stm32h7b3i_discovery_ts.h"
#include "stm32h7b3i_discovery_bus.h"

TouchInterface g_TouchInterface;

bool TouchInterface::Initialize(TouchInterfaceConfig config)
{
    uint32_t ft5336_id = 0;
    FT5336_Object_t ts_comp_obj;
    FT5336_IO_t io_comp_ctx;
    bool ret;

    io_comp_ctx.Init    = BSP_I2C4_Init;
    io_comp_ctx.ReadReg = BSP_I2C4_ReadReg;
    io_comp_ctx.Address = TS_I2C_ADDRESS;
    if (FT5336_RegisterBusIO(&ts_comp_obj, &io_comp_ctx) < 0)
    {
        ret = false;
    }
    else if (FT5336_ReadID(&ts_comp_obj, &ft5336_id) < 0)
    {
        ret = false;
    }
    else if (ft5336_id != FT5336_ID)
    {
        ret = false;
    }

    return true;
}

bool TouchInterface::Write(CLR_UINT8 *dataToSend, CLR_UINT16 numberOfValuesToSend)
{
    (void) dataToSend;
    (void) numberOfValuesToSend;
    return true;
}

bool TouchInterface::Write_Read(
    CLR_UINT8 *dataToSend,
    CLR_UINT16 numberOfValuesToSend,
    CLR_UINT8 *dataReturned,
    CLR_UINT16 numberValuesExpected)
{
    
    (void) dataToSend;
    (void) numberOfValuesToSend;
    (void) dataReturned;
    (void) numberValuesExpected;

    return true;
}

