#pragma once

#include <string.h>
#include <stdint.h>

struct lwrb;
typedef struct lwrb {
    uint8_t* buff;                              /*!< Pointer to buffer data.
                                                    Buffer is considered initialized when `buff != NULL` and `size > 0` */
    size_t size;                                /*!< Size of buffer data. Size of actual buffer is `1` byte less than value holds */
    size_t r;                                   /*!< Next read pointer. Buffer is considered empty when `r == w` and full when `w == r - 1` */
    size_t w;                                   /*!< Next write pointer. Buffer is considered empty when `r == w` and full when `w == r - 1` */
} lwrb_t;

uint8_t     lwrb_init(lwrb_t* buff, void* buffdata, size_t size);
void        lwrb_free(lwrb_t* buff);

/* Read/Write functions */
size_t      lwrb_write(lwrb_t* buff, const void* data, size_t btw);
size_t      lwrb_read(lwrb_t* buff, void* data, size_t btr);

/* Buffer size information */
size_t      lwrb_get_free(lwrb_t* buff);
size_t      lwrb_get_full(lwrb_t* buff);

/* Read data block management */
void*       lwrb_get_linear_block_read_address(lwrb_t* buff);
size_t      lwrb_get_linear_block_read_length(lwrb_t* buff);
size_t      lwrb_skip(lwrb_t* buff, size_t len);

/* Write data block management */
void*       lwrb_get_linear_block_write_address(lwrb_t* buff);
size_t      lwrb_get_linear_block_write_length(lwrb_t* buff);
size_t      lwrb_advance(lwrb_t* buff, size_t len);

