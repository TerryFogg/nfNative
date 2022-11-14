//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//
//
// CPU_SPI_xxxx
//
// Read / Write physical SPI bus
//

#include <nanoCLR_Checks.h>
#include <nanoCLR_Interop.h>
#include <nanoCLR_Runtime.h>
#include <nanoHAL.h>
#include <targetHAL.h>


// struct representing the SPI transaction per bus
struct NF_PAL_SPI {
  int i;
};

// Remove device from bus
// return true of OK, false = error
bool CPU_SPI_Remove_Device(uint32_t deviceHandle)
{ 
  return true;
}

// Initialise the physical SPI bus
// Bus index 0 or 1
// return true of successful, false if error
bool CPU_SPI_Initialize(uint8_t busIndex, const SPI_DEVICE_CONFIGURATION &spiDeviceConfig) 
{
  return true;
}

// Uninitialise the bus
bool CPU_SPI_Uninitialize(uint8_t busIndex) 
{
  return true;
}

//
//  Add a device to a SPI bus
//  returns device handle as uint32_t
//
HRESULT CPU_SPI_Add_Device(const SPI_DEVICE_CONFIGURATION &spiDeviceConfig, uint32_t &handle) 
{
  return S_OK;
}

void CPU_SPI_Wait_Busy(uint32_t deviceHandle, SPI_DEVICE_CONFIGURATION &sdev) 
{
  return;
}

HRESULT CPU_SPI_nWrite_nRead(uint32_t deviceHandle,
                             SPI_DEVICE_CONFIGURATION &sdev,
                             SPI_WRITE_READ_SETTINGS &wrc, uint8_t *writeData,
                             int32_t writeSize, uint8_t *readData,
                             int32_t readSize) 
{
  NANOCLR_HEADER();
  NANOCLR_NOCLEANUP();
}

// Performs a read/write operation on 16-bit word data.
HRESULT CPU_SPI_nWrite16_nRead16(uint32_t deviceHandle,
                                 SPI_DEVICE_CONFIGURATION &sdev,
                                 SPI_WRITE_READ_SETTINGS &swrs,
                                 uint16_t *writePtr, int32_t writeSize,
                                 uint16_t *readPtr, int32_t readSize) 
{
  return CPU_SPI_nWrite_nRead(deviceHandle, sdev, swrs, (uint8_t *)writePtr, writeSize, (uint8_t *)readPtr, readSize);
}

// Return status of current SPI operation
// Used to find status of an Async SPI call
SPI_OP_STATUS CPU_SPI_OP_Status(uint8_t busIndex, uint32_t deviceHandle) {
  (void)deviceHandle;
  return SPI_OP_READY;
}

// Return map of available SPI buses as a bit map
uint32_t CPU_SPI_PortsMap() {
  return 0x03;
}

// Returns the SPI clock, MISO and MOSI pin numbers for a specified SPI module.
void CPU_SPI_GetPins(uint32_t busIndex, GPIO_PIN &clockPin, GPIO_PIN &misoPin, GPIO_PIN &mosiPin) 
{
  return;
}

// Return SPI minimum clock frequency
HRESULT CPU_SPI_MinClockFrequency(uint32_t busIndex, int32_t *frequency) 
{
  return S_OK;
}
// Return SPI maximum clock frequency
//
// Maximum frequency will depend on current configuration
// If using native SPI pins then maximum is 80mhz
// if SPI pins are routed over GPIO matrix then 40mhz half duplex 26mhz full
HRESULT CPU_SPI_MaxClockFrequency(uint32_t busIndex, int32_t *frequency) 
{
  return S_OK;
}
//
// Return the number of chip select lines available on the bus.
//
uint32_t CPU_SPI_ChipSelectLineCount(uint32_t busIndex) 
{
  (void)busIndex;
  return MAX_SPI_DEVICES;
}
