//
// Copyright (c) 2017 The nanoFramework project contributors
// See LICENSE file in the project root for full license information.
//
#include "TouchInterface.h"
#include "nanoPAL.h"
#include "nanoCLR_Types.h"
#include "target_platform.h"

#include "stm32h735xx.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_exti.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_i2c.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_system.h"

void Initialize_I2C();
void Initialize_TouchInterrupt();
static bool I2C_WaitOnTXISFlagUntilTimeout();
static bool I2C_IsAcknowledgeFailed();
#define I2C_MEMADD_SIZE_8BIT (0x00000001U)

// Touch screen interrupt signal
#define TS_INT_PIN             LL_GPIO_PIN_2
#define TS_INT_GPIO_PORT       GPIOG
#define TS_INT_EXTI_IRQn       EXTI2_IRQn
#define TS_INTERRUPT_ROUTINE() void EXTI2_IRQHandler(void)
#define TS_IT_PRIORITY         15U
#define MILLISECONDS25         25000 // 25 thousand microseconds
static CLR_UINT16 touchScreenAddress;
static CLR_UINT16 i2cBusNumber;
TouchInterface g_TouchInterface;
TOUCH_INTERRUPT_SERVICE_ROUTINE touchInterruptRoutine;

bool TouchInterface::Initialize(TouchInterfaceConfig config)
{
    touchScreenAddress = config.Address;
    i2cBusNumber = config.I2c_bus_number;
    Initialize_I2C();
    return true;
}
bool TouchInterface::Write_Register_Read_Data(
    CLR_UINT8 touchRegister,
    CLR_UINT16 numberOfValuesToSend,
    CLR_UINT8 *dataReturned,
    CLR_UINT16 numberValuesExpected)
{
    // Check bus busy flag
    while (LL_I2C_IsActiveFlag_BUSY(I2C4))
    {
    }
    // Prepare transfer parameters
    __IO uint16_t XferSize;
    uint8_t *pBuffPtr = dataReturned;
    __IO uint16_t XferCount = numberValuesExpected;

    LL_I2C_HandleTransfer(
        I2C4,
        touchScreenAddress,
        LL_I2C_ADDRSLAVE_7BIT,
        I2C_MEMADD_SIZE_8BIT,
        LL_I2C_MODE_SOFTEND,
        LL_I2C_GENERATE_START_WRITE);
    if (I2C_WaitOnTXISFlagUntilTimeout() != true) // Wait until TXIS flag is set
    {
        return false;
    }
    LL_I2C_TransmitData8(I2C4, touchRegister);
    while (LL_I2C_IsActiveFlag_TC(I2C4))
    {
    }
    XferSize = XferCount;
    LL_I2C_HandleTransfer(
        I2C4,
        touchScreenAddress,
        LL_I2C_ADDRSLAVE_7BIT,
        XferSize,
        LL_I2C_MODE_AUTOEND,
        LL_I2C_GENERATE_START_READ);
    do
    {
        while (!LL_I2C_IsActiveFlag_RXNE(I2C4))
        {
        };
        *pBuffPtr = I2C4->RXDR;
        pBuffPtr++;
        XferSize--;
        XferCount--;
        if ((XferCount != 0U) && (XferSize == 0U))
        {
            while (LL_I2C_IsActiveFlag_TCR(I2C4))
            {
            };

            XferSize = XferCount;
            LL_I2C_HandleTransfer(
                I2C4,
                touchScreenAddress,
                LL_I2C_ADDRSLAVE_7BIT,
                XferSize,
                LL_I2C_MODE_AUTOEND,
                LL_I2C_GENERATE_NOSTARTSTOP);
        }
    } while (XferCount > 0U);

    // No need to Check TC flag, with AUTOEND mode the stop is automatically
    // generated

    while (!LL_I2C_IsActiveFlag_STOP(I2C4))
    {
    };
    if (I2C_IsAcknowledgeFailed() != true) // Check if a NACK is detected
    {
        return false;
    }
    // Clear STOP Flag
    ((I2C_ISR_STOPF == I2C_ISR_TXE) ? (I2C4->ISR |= I2C_ISR_STOPF) : (I2C4->ICR = I2C_ISR_STOPF));

    // Clear Configuration Register 2
    (I2C4->CR2 &=
     (uint32_t) ~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN)));

    return true;
}
void TouchInterface::SetTouchInterruptCallback(TOUCH_INTERRUPT_SERVICE_ROUTINE touchIsrProc)
{
    touchInterruptRoutine = touchIsrProc;
    Initialize_TouchInterrupt();
}
bool TouchInterface::WriteCommand(uint8_t touchRegister, uint8_t touchCommand)
{
    while (LL_I2C_IsActiveFlag_BUSY(I2C4)) // Check bus busy flag
    {
    }
    LL_I2C_HandleTransfer(
        I2C4,
        touchScreenAddress,
        LL_I2C_ADDRSLAVE_7BIT,
        2,
        LL_I2C_MODE_AUTOEND,
        LL_I2C_GENERATE_START_WRITE);

    while (!LL_I2C_IsActiveFlag_TXE(I2C4))
    {
    }
    //
    if (I2C_WaitOnTXISFlagUntilTimeout() != true) // Wait until TXIS flag is set
    {
        return false;
    }
    LL_I2C_TransmitData8(I2C4, touchRegister);
    while (LL_I2C_IsActiveFlag_TC(I2C4))
    {
    };
    LL_I2C_TransmitData8(I2C4, touchCommand);
    while (LL_I2C_IsActiveFlag_TC(I2C4))
    {
    };

    return true;
}
void TouchInterface::Enable_TouchInterrupt() {
  NVIC_EnableIRQ((TS_INT_EXTI_IRQn));
}
void TouchInterface::Disable_TouchInterrupt() {
  NVIC_DisableIRQ((TS_INT_EXTI_IRQn));
}
void Initialize_I2C()
{
    uint32_t I2C_TIMING = 0x70D04648; // Timing register value is computed with
                                      // the STM32CubeMX Tool Fast Mode @400kHz
                                      // with I2CCLK = 125 MHz Rise time = 100ns
                                      // Fall time = 10ns
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOF);

    // SCL Pin as : Alternate function, High Speed, Open drain, No Pull
    LL_GPIO_SetPinMode(GPIOF, LL_GPIO_PIN_14, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOF, LL_GPIO_PIN_14, LL_GPIO_AF_4);
    LL_GPIO_SetPinSpeed(GPIOF, LL_GPIO_PIN_14, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(GPIOF, LL_GPIO_PIN_14, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(GPIOF, LL_GPIO_PIN_14, LL_GPIO_PULL_NO);

    // SDA Pin as : Alternate function, High Speed, Open drain, No Pull
    LL_GPIO_SetPinMode(GPIOF, LL_GPIO_PIN_15, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOF, LL_GPIO_PIN_15, LL_GPIO_AF_4);
    LL_GPIO_SetPinSpeed(GPIOF, LL_GPIO_PIN_15, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(GPIOF, LL_GPIO_PIN_15, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(GPIOF, LL_GPIO_PIN_15, LL_GPIO_PULL_NO);

    LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_I2C4);
    LL_APB4_GRP1_ForceReset(LL_APB4_GRP1_PERIPH_I2C4);
    LL_APB4_GRP1_ReleaseReset(LL_APB4_GRP1_PERIPH_I2C4);

    LL_I2C_Disable(I2C4); // Disable before updating configuration   registers

    LL_I2C_SetTiming(I2C4, I2C_TIMING); // SDA setup, hold time and the SCL high, low period.
    LL_I2C_SetOwnAddress1(I2C4, 0x00, LL_I2C_OWNADDRESS1_7BIT);
    LL_I2C_EnableAutoEndMode(I2C4); // TC flag is set when NBYTES data are
                                    // transferred, stretching SCL low.
    LL_I2C_AcknowledgeNextData(I2C4, LL_I2C_NACK);
    LL_I2C_DisableOwnAddress1(I2C4); // Address2 configuration
    LL_I2C_SetOwnAddress2(I2C4, 0x00, LL_I2C_OWNADDRESS2_NOMASK);
    LL_I2C_EnableClockStretching(I2C4);
    LL_I2C_DisableGeneralCall(I2C4);

    LL_I2C_Enable(I2C4);
}
static bool I2C_WaitOnTXISFlagUntilTimeout()
{
    while (READ_BIT(I2C4->ISR, I2C_ISR_TXIS) == RESET)
    {
        if (I2C_IsAcknowledgeFailed() != true) // Check if a NACK is detected
        {
            return false;
        }
    }
    return true;
}
static bool I2C_IsAcknowledgeFailed()
{
    if (READ_BIT(I2C4->ISR, I2C_ISR_NACKF) == SET)
    {
        // AutoEnd should be initiate after AF
        while (READ_BIT(I2C4->ISR, I2C_ISR_STOPF) == RESET) // Wait until STOP Flag is reset
        {
        }
        ((I2C_ISR_NACKF == I2C_ISR_TXE) ? (I2C4->ISR |= I2C_ISR_NACKF)
                                        : (I2C4->ICR = I2C_ISR_NACKF)); // Clear NACKF Flag
        ((I2C_ISR_STOPF == I2C_ISR_TXE) ? (I2C4->ISR |= I2C_ISR_STOPF) : (I2C4->ICR = I2C_ISR_STOPF)); // Clear

        if (READ_BIT(I2C4->ISR, I2C_ISR_TXIS) == SET) // If a pending TXIS flag is set
        {
            I2C4->TXDR = 0x00U; // Write a dummy data in TXDR to clear it
        }

        if (READ_BIT(I2C4->ISR, I2C_ISR_TXIS) == RESET) // Flush TX register if not empty
        {
            (I2C_ISR_TXE == I2C_ISR_TXE) ? (I2C4->ISR |= I2C_ISR_TXE) : (I2C4->ICR = I2C_ISR_TXE);
        }
        (I2C4->CR2 // Clear Configuration Register 2
         &=
         (uint32_t) ~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN)));
        return false;
    }
    return true;
}
void Initialize_TouchInterrupt()
{
  I2C_IsAcknowledgeFailed();
    // Peripheral clock already enabled in "Clock_configuration.c"
    LL_GPIO_SetPinMode(GPIOG, TS_INT_PIN, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOG, TS_INT_PIN, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinSpeed(GPIOG, TS_INT_PIN, LL_GPIO_SPEED_FREQ_HIGH);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_2);
    LL_EXTI_DisableEvent_0_31(LL_EXTI_LINE_2);

    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_2);  // Touch Down
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_2); // Touch Up

    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTG, LL_SYSCFG_EXTI_LINE2);
    LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SYSCFG);
    NVIC_SetPriority(TS_INT_EXTI_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), TS_IT_PRIORITY, 0));
    // Disable the interrupt, enabled from a call from the c# code
    NVIC_DisableIRQ((TS_INT_EXTI_IRQn));
}

extern "C"
{
    TS_INTERRUPT_ROUTINE()
    {
        // Clear interrupt and call registered callback
        touchInterruptRoutine();
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
    }
}
