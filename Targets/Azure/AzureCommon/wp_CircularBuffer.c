
//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include "wp_CircularBuffer.h"

CLR_UINT8 wp_InitializeBuffer(CircularBuffer_t *buffer, void *data, CLR_INT32 size)
{
    if (buffer == NULL || data == NULL || size == 0)
    {
        return 0;
    }
    memset((void *)buffer, 0x00, sizeof(*buffer));
    buffer->size = size;
    buffer->buffer = data;
    memset(data, 0x00, size);
    return 1;
}
CLR_INT32 wp_WriteBuffer(CircularBuffer_t *buffer, const void *data, CLR_INT32 btw)
{
    CLR_INT32 tocopy;
    CLR_INT32 free;
    CLR_INT32 size;
    const CLR_UINT8 *d = data;

    if (data == NULL || btw == 0)
    {
        return 0;
    }

    // Calculate maximum number of bytes available to write
    // Use temporary values in case they are changed during operations
    CLR_INT32 w = buffer->w;
    CLR_INT32 r = buffer->r;
    if (w == r)
    {
        size = buffer->size;
    }
    else if (r > w)
    {
        size = r - w;
    }
    else
    {
        size = buffer->size - (w - r);
    }
    // buffer free size is always 1 less than actual size
    free = size - 1;

    btw = BUF_MIN(free, btw);
    if (btw == 0)
    {
        return 0;
    }

    // Step 1: Write data to linear part of buffer
    tocopy = BUF_MIN(buffer->size - buffer->w, btw);
    memcpy(&buffer->buffer[buffer->w], d, tocopy);
    buffer->w += tocopy;
    btw -= tocopy;

    // Step 2: Write data to beginning of buffer (overflow part)
    if (btw > 0)
    {
        memcpy(buffer->buffer, &d[tocopy], btw);
        buffer->w = btw;
    }

    // Step 3: Check end of buffer
    if (buffer->w >= buffer->size)
    {
        buffer->w = 0;
    }
    return tocopy + btw;
}
CLR_INT32 wp_ReadBuffer(CircularBuffer_t *buffer, void *data, CLR_INT32 btr)
{
    CLR_INT32 tocopy;
    CLR_INT32 full;
    CLR_UINT8 *d = data;

    if (data == NULL || btr == 0)
    {
        return 0;
    }

    // Calculate maximum number of bytes available to read
    full = wp_BufferBytesWaiting(buffer);
    btr = BUF_MIN(full, btr);
    if (btr == 0)
    {
        return 0;
    }

    // Step 1: Read data from linear part of buffer
    tocopy = BUF_MIN(buffer->size - buffer->r, btr);
    memcpy(d, &buffer->buffer[buffer->r], tocopy);
    buffer->r += tocopy;
    btr -= tocopy;

    // Step 2: Read data from beginning of buffer (overflow part)
    if (btr > 0)
    {
        memcpy(&d[tocopy], buffer->buffer, btr);
        buffer->r = btr;
    }

    // Step 3: Check end of buffer
    if (buffer->r >= buffer->size)
    {
        buffer->r = 0;
    }
    return tocopy + btr;
}
CLR_INT32 wp_BufferBytesWaiting(CircularBuffer_t *buffer)
{
    CLR_INT32 size;

    // Use temporary values in case they are changed during operations
    CLR_INT32 w = buffer->w;
    CLR_INT32 r = buffer->r;
    if (w == r)
    {
        size = 0;
    }
    else if (w > r)
    {
        size = w - r;
    }
    else
    {
        size = buffer->size - (r - w);
    }
    return size;
}
