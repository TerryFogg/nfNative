//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//
#include "Stm32Values.h"
#include "nanoCLR_Types.h"
#include "sys_dev_adc_native.h"

enum AdcResolution
{
    _8Bit,
    _12Bit,
    _10Bit,
    _16Bit
};

static enum AdcResolution selectedResolution;

HRESULT
Library_sys_dev_adc_native_System_Device_Adc_AdcChannel::NativeReadValue___I4(CLR_RT_StackFrame &stack)
{
    return 1;
}
HRESULT Library_sys_dev_adc_native_System_Device_Adc_AdcChannel::NativeDisposeChannel___VOID(CLR_RT_StackFrame &stack)
{
    return 1;
}
HRESULT Library_sys_dev_adc_native_System_Device_Adc_AdcController::NativeOpenChannel___VOID__I4(
    CLR_RT_StackFrame &stack)
{
    return 1;
}
HRESULT Library_sys_dev_adc_native_System_Device_Adc_AdcController::NativeGetChannelCount___I4(CLR_RT_StackFrame &stack)
{
    return 1;
}
HRESULT Library_sys_dev_adc_native_System_Device_Adc_AdcController::NativeGetMaxValue___I4(CLR_RT_StackFrame &stack)
{
    CLR_INT32 maxValue = 0;
    switch (selectedResolution)
    {
        case _8Bit:
            maxValue = 255;
            break;
        case _10Bit:
            maxValue = 1023;
            break;
        case _12Bit:
            maxValue = 2047;
            break;
        case _16Bit:
            maxValue = 65535;
            break;
    }
    // maxValue;
    return 1;
}
HRESULT Library_sys_dev_adc_native_System_Device_Adc_AdcController::NativeGetMinValue___I4(CLR_RT_StackFrame &stack)
{
    return 1;
}
HRESULT Library_sys_dev_adc_native_System_Device_Adc_AdcController::NativeIsChannelModeSupported___BOOLEAN__I4(
    CLR_RT_StackFrame &stack)
{
    return 1;
}
HRESULT Library_sys_dev_adc_native_System_Device_Adc_AdcController::NativeGetResolutionInBits___I4(
    CLR_RT_StackFrame &stack)
{
    CLR_INT32 resolution = 0;
    switch (selectedResolution)
    {
        case _8Bit:
            resolution = 8;
            break;
        case _10Bit:
            resolution = 10;
            break;
        case _12Bit:
            resolution = 12;
            break;
        case _16Bit:
            resolution = 16;
            break;
    }
    // resolution;
    return 1;
}
HRESULT Library_sys_dev_adc_native_System_Device_Adc_AdcController::NativeInit___VOID(CLR_RT_StackFrame &stack)
{
    return 1;
}
