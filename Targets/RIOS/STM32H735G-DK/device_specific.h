#pragma once

#include <stdint.h> /* exact-width integer types, ANSI C'99 */


__attribute__((always_inline)) static inline void __enable_irq(void) {
  __asm volatile("cpsie i" : : : "memory");
}

__attribute__((always_inline)) static inline void __disable_irq(void) {
  __asm volatile("cpsid i" : : : "memory");
}

static __inline uint32_t __get_PRIMASK(void) {
  register uint32_t __regPriMask __asm("primask");
  return (__regPriMask);
}


#define SST_INT_LOCK()   __disable_irq();
#define SST_INT_UNLOCK()  __enable_irq();
