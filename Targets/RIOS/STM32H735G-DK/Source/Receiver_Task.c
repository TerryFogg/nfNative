//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <stm32h7xx_hal.h>

__attribute__((noreturn)) void Receiver_Task(uint32_t parameter) {
  (void)parameter;

  while (1) {
  }
}

void WP_Message_PrepareReception_Platform() {
  // empty on purpose, nothing to configure
}
