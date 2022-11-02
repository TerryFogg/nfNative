//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//
#include "Stm32Values.h"
#include "nanoCLR_Types.h"
#include "nanoCLR_Interop.h"
#include "sys_dev_i2c_native.h"


typedef Library_sys_dev_i2c_native_System_Device_I2c_I2cConnectionSettings I2cConnectionSettings;
HRESULT
Library_sys_dev_i2c_native_System_Device_I2c_I2cDevice::NativeInit___VOID(CLR_RT_StackFrame &stack)
{
    NANOCLR_HEADER();


    // get a pointer to the managed object instance and check that it's not NULL
    CLR_RT_HeapBlock *pThis = stack.This();

    if (pThis != NULL)
    {
        // get a pointer to the managed I2C connectionSettings object instance
        CLR_RT_HeapBlock *connectionSettings = pThis[FIELD___connectionSettings].Dereference();
        // get bus index
        int busIndex = (uint8_t)connectionSettings[I2cConnectionSettings::FIELD___busId].NumericByRef().s4;
        // clear pointer to working thread
        I2cBusSpeed busSpeed =
            (I2cBusSpeed)connectionSettings[I2cConnectionSettings::FIELD___busSpeed].NumericByRef().s4;

        // ... More code here
    }

    NANOCLR_NOCLEANUP();
}
HRESULT
Library_sys_dev_i2c_native_System_Device_I2c_I2cDevice::NativeDispose___VOID(CLR_RT_StackFrame &stack)
{
    return 1;
}
HRESULT Library_sys_dev_i2c_native_System_Device_I2c_I2cDevice::
    NativeTransmit___SystemDeviceI2cI2cTransferResult__SystemSpanByte__SystemSpanByte(CLR_RT_StackFrame &stack)
{
    return 1;
}
