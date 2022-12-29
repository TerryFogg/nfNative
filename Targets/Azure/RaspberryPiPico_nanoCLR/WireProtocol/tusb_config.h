#pragma once

//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//
#define CFG_TUSB_OS OPT_OS_PICO

#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define CFG_TUD_ENDPOINT0_SIZE 64 // Endpoint 0 (EP0IN and EP0OUT) are reserved in every device for control purposes


//#define CFG_TUSB_MEM_ALIGN          __attribute__ ((aligned(4)))

// Defines used in usbd.h to include class drivers
#define CFG_TUD_CDC               1  // Include code for cdc device
#define CFG_TUD_MSC               0  // Don't include mass storage usb code
#define CFG_TUD_HID               0  // Dont' include code for mouse/keyboard type devices
#define CFG_TUD_MIDI              0  // Don't include midi usb code support
#define CFG_TUD_VENDOR            0  // Not a "VENDOR" device

// Defines used in cdc_device.c to define size of array buffers
#define CFG_TUD_CDC_RX_BUFSIZE 512
#define CFG_TUD_CDC_TX_BUFSIZE 512

