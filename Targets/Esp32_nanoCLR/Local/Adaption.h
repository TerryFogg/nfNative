#pragma once

#define LOCK_FROM_ISR                                                                                                  \
    uint32_t primask = __get_PRIMASK();                                                                                \
    __disable_irq();
#define UNLOCK_FROM_ISR __disable_irq();
