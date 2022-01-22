
#include "lwrb.h"
#define BUF_IS_VALID(b)                 ((b) != NULL && (b)->buff != NULL && (b)->size > 0)
#define BUF_MIN(x, y)                   ((x) < (y) ? (x) : (y))
#define BUF_MAX(x, y)                   ((x) > (y) ? (x) : (y))

uint8_t lwrb_init(lwrb_t* buff, void* buffdata, size_t size) 
{
    if (buff == NULL || buffdata == NULL || size == 0) {
        return 0;
    }
    memset((void*)buff, 0x00, sizeof(*buff));
    buff->size = size;
    buff->buff = buffdata;
    return 1;
}
void lwrb_free(lwrb_t* buff) 
{
    if (BUF_IS_VALID(buff)) {
        buff->buff = NULL;
    }
}
size_t lwrb_write(lwrb_t* buff, const void* data, size_t btw) 
{
    size_t tocopy;
    size_t   free;
    const uint8_t* d = data;

    if (!BUF_IS_VALID(buff) || data == NULL || btw == 0) {
        return 0;
    }

    /* Calculate maximum number of bytes available to write */
    free = lwrb_get_free(buff);
    btw = BUF_MIN(free, btw);
    if (btw == 0) {
        return 0;
    }

    /* Step 1: Write data to linear part of buffer */
    tocopy = BUF_MIN(buff->size - buff->w, btw);
    memcpy(&buff->buff[buff->w], d, tocopy);
    buff->w += tocopy;
    btw -= tocopy;

    /* Step 2: Write data to beginning of buffer (overflow part) */
    if (btw > 0) {
        memcpy(buff->buff, &d[tocopy], btw);
        buff->w = btw;
    }

    /* Step 3: Check end of buffer */
    if (buff->w >= buff->size) {
        buff->w = 0;
    }
    return tocopy + btw;
}
size_t lwrb_read(lwrb_t* buff, void* data, size_t btr) 
{
    size_t tocopy;
    size_t full;
    uint8_t* d = data;

    if (!BUF_IS_VALID(buff) || data == NULL || btr == 0) {
        return 0;
    }

    /* Calculate maximum number of bytes available to read */
    full = lwrb_get_full(buff);
    btr = BUF_MIN(full, btr);
    if (btr == 0) {
        return 0;
    }

    /* Step 1: Read data from linear part of buffer */
    tocopy = BUF_MIN(buff->size - buff->r, btr);
    memcpy(d, &buff->buff[buff->r], tocopy);
    buff->r += tocopy;
    btr -= tocopy;

    /* Step 2: Read data from beginning of buffer (overflow part) */
    if (btr > 0) {
        memcpy(&d[tocopy], buff->buff, btr);
        buff->r = btr;
    }

    /* Step 3: Check end of buffer */
    if (buff->r >= buff->size) {
        buff->r = 0;
    }
        return tocopy + btr;
}
size_t lwrb_get_free(lwrb_t* buff)
{
    size_t size;
    size_t w;
    size_t r;

    if (!BUF_IS_VALID(buff)) {
        return 0;
    }
    /* Use temporary values in case they are changed during operations */
    w = buff->w;
    r = buff->r;
    if (w == r) {
        size = buff->size;
    }
    else if (r > w) {
        size = r - w;
    }
    else {
        size = buff->size - (w - r);
    }
    /* Buffer free size is always 1 less than actual size */
    return size - 1;
}
size_t lwrb_get_full(lwrb_t* buff)
{
    size_t w;
    size_t r;
    size_t size;

    if (!BUF_IS_VALID(buff)) {
        return 0;
    }
    /* Use temporary values in case they are changed during operations */
    w = buff->w;
    r = buff->r;
    if (w == r) {
        size = 0;
    }
    else if (w > r) {
        size = w - r;
    }
    else {
        size = buff->size - (r - w);
    }
    return size;
}
void*  lwrb_get_linear_block_read_address(lwrb_t* buff) 
{
    if (!BUF_IS_VALID(buff)) {
        return NULL;
    }
    return &buff->buff[buff->r];
}
size_t lwrb_get_linear_block_read_length(lwrb_t* buff) 
{
    size_t w;
    size_t r;
    size_t len;

    if (!BUF_IS_VALID(buff)) {
        return 0;
    }
    /* Use temporary values in case they are changed during operations */
    w = buff->w;
    r = buff->r;
    if (w > r) {
        len = w - r;
    }
    else if (r > w) {
        len = buff->size - r;
    }
    else {
        len = 0;
    }
    return len;
}
size_t  lwrb_skip(lwrb_t* buff, size_t len) 
{
    size_t full;

    if (!BUF_IS_VALID(buff) || len == 0) {
        return 0;
    }
    full = lwrb_get_full(buff);
    len = BUF_MIN(len, full);
    buff->r += len;
    if (buff->r >= buff->size) {
        buff->r -= buff->size;
    }
        return len;
}
