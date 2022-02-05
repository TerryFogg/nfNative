//
// Copyright (c) 2017 The nanoFramework project contributors
// See LICENSE file in the project root for full license information.
//

#include "nanoCLR_Types.h"
#include <nanoPAL.h>
#include <target_platform.h>
#include "TouchInterface.h"
#include "nanoCLR_Types.h"
//#include "stm32h735g_discovery_ts.h"
//#include "stm32h735g_discovery_bus.h"
//#include "stm32h7xx_hal_i2c.h"
#include "stm32h7xx.h"
#include "Debug_To_Display.h"

typedef enum
{
    HAL_I2C_STATE_RESET = 0x00U,          /*!< Peripheral is not yet Initialized         */
    HAL_I2C_STATE_READY = 0x20U,          /*!< Peripheral Initialized and ready for use  */
    HAL_I2C_STATE_BUSY = 0x24U,           /*!< An internal process is ongoing            */
    HAL_I2C_STATE_BUSY_TX = 0x21U,        /*!< Data Transmission process is ongoing */
    HAL_I2C_STATE_BUSY_RX = 0x22U,        /*!< Data Reception process is ongoing */
    HAL_I2C_STATE_LISTEN = 0x28U,         /*!< Address Listen Mode is ongoing  */
    HAL_I2C_STATE_BUSY_TX_LISTEN = 0x29U, /*!< Address Listen Mode and Data
                                             Transmission process is ongoing */
    HAL_I2C_STATE_BUSY_RX_LISTEN = 0x2AU, /*!< Address Listen Mode and Data
                                             Reception process is ongoing */
    HAL_I2C_STATE_ABORT = 0x60U,          /*!< Abort user request ongoing                */
    HAL_I2C_STATE_TIMEOUT = 0xA0U,        /*!< Timeout state */
    HAL_I2C_STATE_ERROR = 0xE0U           /*!< Error                                     */

} HAL_I2C_StateTypeDef;
typedef enum
{
    HAL_I2C_MODE_NONE = 0x00U,   /*!< No I2C communication on going             */
    HAL_I2C_MODE_MASTER = 0x10U, /*!< I2C communication is in Master Mode       */
    HAL_I2C_MODE_SLAVE = 0x20U,  /*!< I2C communication is in Slave Mode        */
    HAL_I2C_MODE_MEM = 0x40U     /*!< I2C communication is in Memory Mode       */

} HAL_I2C_ModeTypeDef;
typedef struct
{
    uint32_t Timing; /*!< Specifies the I2C_TIMINGR_register value.
                       This parameter calculated by referring to I2C
                       initialization section in Reference manual */

    uint32_t OwnAddress1; /*!< Specifies the first device own address.
                            This parameter can be a 7-bit or 10-bit address. */

    uint32_t AddressingMode; /*!< Specifies if 7-bit or 10-bit addressing mode is
                               selected. This parameter can be a value of @ref
                               I2C_ADDRESSING_MODE */

    uint32_t DualAddressMode; /*!< Specifies if dual addressing mode is selected.
                                This parameter can be a value of @ref
                                I2C_DUAL_ADDRESSING_MODE */

    uint32_t OwnAddress2; /*!< Specifies the second device own address if dual
                            addressing mode is selected This parameter can be a
                            7-bit address. */

    uint32_t OwnAddress2Masks; /*!< Specifies the acknowledge mask address second
                                 device own address if dual addressing mode is
                                 selected This parameter can be a value of @ref
                                 I2C_OWN_ADDRESS2_MASKS */

    uint32_t GeneralCallMode; /*!< Specifies if general call mode is selected.
                                This parameter can be a value of @ref
                                I2C_GENERAL_CALL_ADDRESSING_MODE */

    uint32_t NoStretchMode; /*!< Specifies if nostretch mode is selected.
                              This parameter can be a value of @ref
                              I2C_NOSTRETCH_MODE */

} I2C_InitTypeDef;
typedef struct __I2C_HandleTypeDef
{
    I2C_TypeDef *Instance; /*!< I2C registers base address                */

    I2C_InitTypeDef Init; /*!< I2C communication parameters              */

    uint8_t *pBuffPtr; /*!< Pointer to I2C transfer buffer            */

    uint16_t XferSize; /*!< I2C transfer size                         */

    __IO uint16_t XferCount; /*!< I2C transfer counter                      */

    __IO uint32_t XferOptions; /*!< I2C sequantial transfer options, this parameter can
                                    be a value of @ref I2C_XFEROPTIONS */

    __IO uint32_t PreviousState; /*!< I2C communication Previous state          */

    HAL_StatusTypeDef (*XferISR)(
        struct __I2C_HandleTypeDef *hi2c,
        uint32_t ITFlags,
        uint32_t ITSources); /*!< I2C transfer IRQ handler function pointer */

    DMA_HandleTypeDef *hdmatx; /*!< I2C Tx DMA handle parameters              */

    DMA_HandleTypeDef *hdmarx; /*!< I2C Rx DMA handle parameters              */

    HAL_LockTypeDef Lock; /*!< I2C locking object                        */

    __IO HAL_I2C_StateTypeDef State; /*!< I2C communication state */

    __IO HAL_I2C_ModeTypeDef Mode; /*!< I2C communication mode */

    __IO uint32_t ErrorCode; /*!< I2C Error code                            */

    __IO uint32_t AddrEventCount; /*!< I2C Address Event counter */

} I2C_HandleTypeDef;

HAL_StatusTypeDef HAL_I2C_Mem_Read(
    I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress,
    uint16_t MemAddress,
    uint16_t MemAddSize,
    uint8_t *pData,
    uint16_t Size,
    uint32_t Timeout);
static HAL_StatusTypeDef I2C_WaitOnSTOPFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef I2C_WaitOnFlagUntilTimeout(
    I2C_HandleTypeDef *hi2c,
    uint32_t Flag,
    FlagStatus Status,
    uint32_t Timeout,
    uint32_t Tickstart);
static HAL_StatusTypeDef I2C_RequestMemoryRead(
    I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress,
    uint16_t MemAddress,
    uint16_t MemAddSize,
    uint32_t Timeout,
    uint32_t Tickstart);

static HAL_StatusTypeDef I2C_WaitOnTXISFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef I2C_IsAcknowledgeFailed(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);
static void I2C_Flush_TXDR(I2C_HandleTypeDef *hi2c);
static void I2C_TransferConfig(
    I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress,
    uint8_t Size,
    uint32_t Mode,
    uint32_t Request);


#define I2C_FLAG_TC  I2C_ISR_TC
#define I2C_FLAG_TXE I2C_ISR_TXE

#define I2C_MEM_ADD_LSB(__ADDRESS__) ((uint8_t)((uint16_t)((__ADDRESS__) & (uint16_t)(0x00FFU))))
#define __HAL_I2C_CLEAR_FLAG(__HANDLE__, __FLAG__)                                                                     \
    (((__FLAG__) == I2C_FLAG_TXE) ? ((__HANDLE__)->Instance->ISR |= (__FLAG__))                                        \
                                  : ((__HANDLE__)->Instance->ICR = (__FLAG__)))
#define I2C_FLAG_AF      I2C_ISR_NACKF
#define HAL_I2C_ERROR_AF (0x00000004U) /*!< ACKF error            */

#define I2C_FLAG_TXIS            I2C_ISR_TXIS
#define I2C_SOFTEND_MODE         (0x00000000U)
#define I2C_GENERATE_START_WRITE (uint32_t)(0x80000000U | I2C_CR2_START)
#define I2C_NO_STARTSTOP (0x00000000U)

#define I2C_ADDRESSINGMODE_7BIT                         (0x00000001U)
#define I2C_DUALADDRESS_DISABLE                         (0x00000000U)
#define I2C_OA2_NOMASK                                  ((uint8_t)0x00U)
#define I2C_GENERALCALL_DISABLE                         (0x00000000U)
#define I2C_NOSTRETCH_DISABLE                           (0x00000000U)
#define I2C_DUALADDRESS_ENABLE                          I2C_OAR2_OA2EN
#define I2C_MEMADD_SIZE_8BIT                            (0x00000001U)
#define HAL_I2C_ERROR_INVALID_PARAM                     (0x00000200U) /*!< Invalid Parameters error  */
#define I2C_FLAG_BUSY                                   I2C_ISR_BUSY
#define I2C_TIMEOUT_BUSY                                (25U)         /*!< 25 ms */
#define HAL_I2C_ERROR_NONE                              (0x00000000U) /*!< No error              */
#define I2C_RELOAD_MODE                                 I2C_CR2_RELOAD
#define I2C_GENERATE_START_READ                         (uint32_t)(0x80000000U | I2C_CR2_START | I2C_CR2_RD_WRN)
#define I2C_AUTOEND_MODE                                I2C_CR2_AUTOEND
#define I2C_FLAG_RXNE                                   I2C_ISR_RXNE
#define I2C_FLAG_TCR                                    I2C_ISR_TCR
#define __HAL_I2C_ENABLE_IT(__HANDLE__, __INTERRUPT__)  ((__HANDLE__)->Instance->CR1 |= (__INTERRUPT__))
#define __HAL_I2C_DISABLE_IT(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->CR1 &= (~(__INTERRUPT__)))
#define __HAL_I2C_ENABLE(__HANDLE__)                    (SET_BIT((__HANDLE__)->Instance->CR1, I2C_CR1_PE))
#define I2C_RESET_CR2(__HANDLE__)                                                                                      \
    ((__HANDLE__)->Instance->CR2 &=                                                                                    \
     (uint32_t) ~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN)))
#define __HAL_I2C_DISABLE_IT(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->CR1 &= (~(__INTERRUPT__)))
#define __HAL_I2C_GET_FLAG(__HANDLE__, __FLAG__)                                                                       \
    (((((__HANDLE__)->Instance->ISR) & (__FLAG__)) == (__FLAG__)) ? SET : RESET)
#define __HAL_I2C_DISABLE(__HANDLE__) (CLEAR_BIT((__HANDLE__)->Instance->CR1, I2C_CR1_PE))

#define MAX_NBYTE_SIZE               255U
#define I2C_FLAG_STOPF               I2C_ISR_STOPF
#define HAL_I2C_ERROR_TIMEOUT        (0x00000020U) /*!< Timeout error         */
#define I2C_MEM_ADD_MSB(__ADDRESS__) ((uint8_t)((uint16_t)(((uint16_t)((__ADDRESS__) & (uint16_t)(0xFF00U))) >> 8U)))

#define I2C_SPEED_FREQ_STANDARD     0U   /* 100 kHz */
#define I2C_SPEED_FREQ_FAST         1U   /* 400 kHz */
#define I2C_SPEED_FREQ_FAST_PLUS    2U   /* 1 MHz */
#define I2C_ANALOG_FILTER_DELAY_MIN 50U  /* ns */
#define I2C_ANALOG_FILTER_DELAY_MAX 260U /* ns */
#define I2C_USE_ANALOG_FILTER       1U
#define I2C_DIGITAL_FILTER_COEF     0U
#define I2C_PRESC_MAX               16U
#define I2C_SCLDEL_MAX              16U
#define I2C_SDADEL_MAX              16U
#define I2C_SCLH_MAX                256U
#define I2C_SCLL_MAX                256U
#define SEC2NSEC                    1000000000UL
#define I2C_VALID_TIMING_NBR        128U
#define TIMING_CLEAR_MASK           (0xF0FFFFFFU) /*!< I2C TIMING clear register Mask */
#define FREQ_100KHZ                 100000U

/* Definition for I2C4 clock resources */
#define BUS_I2C4                        I2C4
#define BUS_I2C4_CLK_ENABLE()           __HAL_RCC_I2C4_CLK_ENABLE()
#define BUS_I2C4_CLK_DISABLE()          __HAL_RCC_I2C4_CLK_DISABLE()
#define BUS_I2C4_SCL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOF_CLK_ENABLE()
#define BUS_I2C4_SCL_GPIO_CLK_DISABLE() __HAL_RCC_GPIOF_CLK_DISABLE()
#define BUS_I2C4_SDA_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOF_CLK_ENABLE()
#define BUS_I2C4_SDA_GPIO_CLK_DISABLE() __HAL_RCC_GPIOF_CLK_DISABLE()
#define BUS_I2C4_FORCE_RESET()          __HAL_RCC_I2C4_FORCE_RESET()
#define BUS_I2C4_RELEASE_RESET()        __HAL_RCC_I2C4_RELEASE_RESET()
#define BUS_I2C4_SCL_PIN                GPIO_PIN_14
#define BUS_I2C4_SDA_PIN                GPIO_PIN_15
#define BUS_I2C4_SCL_GPIO_PORT          GPIOF
#define BUS_I2C4_SDA_GPIO_PORT          GPIOF
#define BUS_I2C4_SCL_AF                 GPIO_AF4_I2C4
#define BUS_I2C4_SDA_AF                 GPIO_AF4_I2C4
#define BUS_I2C4_FREQUENCY              100000U /* Frequency of I2Cn = 100 KHz*/

#define TS_I2C_ADDRESS     0x70U
#define BSP_TS_IT_PRIORITY 15U

/**
 * @brief Touch screen interrupt signal
 */
#define TS_INT_PIN                GPIO_PIN_2
#define TS_INT_GPIO_PORT          GPIOG
#define TS_INT_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOG_CLK_ENABLE()
#define TS_INT_GPIO_CLK_DISABLE() __HAL_RCC_GPIOG_CLK_DISABLE()
#define TS_INT_EXTI_IRQn          EXTI2_IRQn
#define TS_EXTI_LINE              EXTI_LINE_2

static CLR_UINT16 touchScreenAddress;
static CLR_UINT16 i2cBusNumber;

typedef struct
{
    uint32_t freq;      /* Frequency in Hz */
    uint32_t freq_min;  /* Minimum frequency in Hz */
    uint32_t freq_max;  /* Maximum frequency in Hz */
    uint32_t hddat_min; /* Minimum data hold time in ns */
    uint32_t vddat_max; /* Maximum data valid time in ns */
    uint32_t sudat_min; /* Minimum data setup time in ns */
    uint32_t lscl_min;  /* Minimum low period of the SCL clock in ns */
    uint32_t hscl_min;  /* Minimum high period of SCL clock in ns */
    uint32_t trise;     /* Rise time in ns */
    uint32_t tfall;     /* Fall time in ns */
    uint32_t dnf;       /* Digital noise filter coefficient */
} I2C_Charac_t;

typedef struct
{
    uint32_t presc;   /* Timing prescaler */
    uint32_t tscldel; /* SCL delay */
    uint32_t tsdadel; /* SDA delay */
    uint32_t sclh;    /* SCL high period */
    uint32_t scll;    /* SCL low period */
} I2C_Timings_t;
static I2C_Timings_t I2c_valid_timing[I2C_VALID_TIMING_NBR];
static uint32_t I2c_valid_timing_nbr = 0;
I2C_HandleTypeDef ts_i2c;
TouchInterface g_TouchInterface;

static const I2C_Charac_t I2C_Charac[] = {
    [I2C_SPEED_FREQ_STANDARD] =
        {
            .freq = 100000,
            .freq_min = 80000,
            .freq_max = 120000,
            .hddat_min = 0,
            .vddat_max = 3450,
            .sudat_min = 250,
            .lscl_min = 4700,
            .hscl_min = 4000,
            .trise = 640,
            .tfall = 20,
            .dnf = I2C_DIGITAL_FILTER_COEF,
        },
    [I2C_SPEED_FREQ_FAST] =
        {
            .freq = 400000,
            .freq_min = 320000,
            .freq_max = 480000,
            .hddat_min = 0,
            .vddat_max = 900,
            .sudat_min = 100,
            .lscl_min = 1300,
            .hscl_min = 600,
            .trise = 250,
            .tfall = 100,
            .dnf = I2C_DIGITAL_FILTER_COEF,
        },
    [I2C_SPEED_FREQ_FAST_PLUS] =
        {
            .freq = 1000000,
            .freq_min = 800000,
            .freq_max = 1200000,
            .hddat_min = 0,
            .vddat_max = 450,
            .sudat_min = 50,
            .lscl_min = 500,
            .hscl_min = 260,
            .trise = 60,
            .tfall = 100,
            .dnf = I2C_DIGITAL_FILTER_COEF,
        },
};

static uint32_t I2C_Compute_SCLL_SCLH(uint32_t clock_src_freq, uint32_t I2C_speed)
{
    uint32_t ret = 0xFFFFFFFFU;
    uint32_t ti2cclk;
    uint32_t ti2cspeed;
    uint32_t prev_error;
    uint32_t dnf_delay;
    uint32_t clk_min, clk_max;
    uint32_t scll, sclh;
    uint32_t tafdel_min;

    ti2cclk = (SEC2NSEC + (clock_src_freq / 2U)) / clock_src_freq;
    ti2cspeed = (SEC2NSEC + (I2C_Charac[I2C_speed].freq / 2U)) / I2C_Charac[I2C_speed].freq;

    tafdel_min = I2C_ANALOG_FILTER_DELAY_MIN;

    /* tDNF = DNF x tI2CCLK */
    dnf_delay = I2C_Charac[I2C_speed].dnf * ti2cclk;

    clk_max = SEC2NSEC / I2C_Charac[I2C_speed].freq_min;
    clk_min = SEC2NSEC / I2C_Charac[I2C_speed].freq_max;

    prev_error = ti2cspeed;

    for (uint32_t count = 0; count < I2c_valid_timing_nbr; count++)
    {
        /* tPRESC = (PRESC+1) x tI2CCLK*/
        uint32_t tpresc = (I2c_valid_timing[count].presc + 1U) * ti2cclk;

        for (scll = 0; scll < I2C_SCLL_MAX; scll++)
        {
            /* tLOW(min) <= tAF(min) + tDNF + 2 x tI2CCLK + [(SCLL+1) x tPRESC ] */
            uint32_t tscl_l = tafdel_min + dnf_delay + (2U * ti2cclk) + ((scll + 1U) * tpresc);

            /* The I2CCLK period tI2CCLK must respect the following conditions:
            tI2CCLK < (tLOW - tfilters) / 4 and tI2CCLK < tHIGH */
            if ((tscl_l > I2C_Charac[I2C_speed].lscl_min) && (ti2cclk < ((tscl_l - tafdel_min - dnf_delay) / 4U)))
            {
                for (sclh = 0; sclh < I2C_SCLH_MAX; sclh++)
                {
                    /* tHIGH(min) <= tAF(min) + tDNF + 2 x tI2CCLK + [(SCLH+1) x tPRESC]
                     */
                    uint32_t tscl_h = tafdel_min + dnf_delay + (2U * ti2cclk) + ((sclh + 1U) * tpresc);

                    /* tSCL = tf + tLOW + tr + tHIGH */
                    uint32_t tscl = tscl_l + tscl_h + I2C_Charac[I2C_speed].trise + I2C_Charac[I2C_speed].tfall;

                    if ((tscl >= clk_min) && (tscl <= clk_max) && (tscl_h >= I2C_Charac[I2C_speed].hscl_min) &&
                        (ti2cclk < tscl_h))
                    {
                        int32_t error = (int32_t)tscl - (int32_t)ti2cspeed;

                        if (error < 0)
                        {
                            error = -error;
                        }

                        /* look for the timings with the lowest clock error */
                        if ((uint32_t)error < prev_error)
                        {
                            prev_error = (uint32_t)error;
                            I2c_valid_timing[count].scll = scll;
                            I2c_valid_timing[count].sclh = sclh;
                            ret = count;
                        }
                    }
                }
            }
        }
    }

    return ret;
}

static void I2C_Compute_PRESC_SCLDEL_SDADEL(uint32_t clock_src_freq, uint32_t I2C_speed)
{
    uint32_t prev_presc = I2C_PRESC_MAX;
    uint32_t ti2cclk;
    int32_t tsdadel_min, tsdadel_max;
    int32_t tscldel_min;
    uint32_t presc, scldel, sdadel;
    uint32_t tafdel_min, tafdel_max;

    ti2cclk = (SEC2NSEC + (clock_src_freq / 2U)) / clock_src_freq;

    tafdel_min = I2C_ANALOG_FILTER_DELAY_MIN;
    tafdel_max = I2C_ANALOG_FILTER_DELAY_MAX;

    /* tDNF = DNF x tI2CCLK
       tPRESC = (PRESC+1) x tI2CCLK
       SDADEL >= {tf +tHD;DAT(min) - tAF(min) - tDNF - [3 x tI2CCLK]} / {tPRESC}
       SDADEL <= {tVD;DAT(max) - tr - tAF(max) - tDNF- [4 x tI2CCLK]} / {tPRESC}
     */

    tsdadel_min = (int32_t)I2C_Charac[I2C_speed].tfall + (int32_t)I2C_Charac[I2C_speed].hddat_min -
                  (int32_t)tafdel_min - (int32_t)(((int32_t)I2C_Charac[I2C_speed].dnf + 3) * (int32_t)ti2cclk);

    tsdadel_max = (int32_t)I2C_Charac[I2C_speed].vddat_max - (int32_t)I2C_Charac[I2C_speed].trise -
                  (int32_t)tafdel_max - (int32_t)(((int32_t)I2C_Charac[I2C_speed].dnf + 4) * (int32_t)ti2cclk);

    /* {[tr+ tSU;DAT(min)] / [tPRESC]} - 1 <= SCLDEL */
    tscldel_min = (int32_t)I2C_Charac[I2C_speed].trise + (int32_t)I2C_Charac[I2C_speed].sudat_min;

    if (tsdadel_min <= 0)
    {
        tsdadel_min = 0;
    }

    if (tsdadel_max <= 0)
    {
        tsdadel_max = 0;
    }

    for (presc = 0; presc < I2C_PRESC_MAX; presc++)
    {
        for (scldel = 0; scldel < I2C_SCLDEL_MAX; scldel++)
        {
            /* TSCLDEL = (SCLDEL+1) * (PRESC+1) * TI2CCLK */
            uint32_t tscldel = (scldel + 1U) * (presc + 1U) * ti2cclk;

            if (tscldel >= (uint32_t)tscldel_min)
            {
                for (sdadel = 0; sdadel < I2C_SDADEL_MAX; sdadel++)
                {
                    /* TSDADEL = SDADEL * (PRESC+1) * TI2CCLK */
                    uint32_t tsdadel = (sdadel * (presc + 1U)) * ti2cclk;

                    if ((tsdadel >= (uint32_t)tsdadel_min) && (tsdadel <= (uint32_t)tsdadel_max))
                    {
                        if (presc != prev_presc)
                        {
                            I2c_valid_timing[I2c_valid_timing_nbr].presc = presc;
                            I2c_valid_timing[I2c_valid_timing_nbr].tscldel = scldel;
                            I2c_valid_timing[I2c_valid_timing_nbr].tsdadel = sdadel;
                            prev_presc = presc;
                            I2c_valid_timing_nbr++;

                            if (I2c_valid_timing_nbr >= I2C_VALID_TIMING_NBR)
                            {
                                return;
                            }
                        }
                    }
                }
            }
        }
    }
}

static uint32_t I2C_GetTiming(uint32_t clock_src_freq, uint32_t i2c_freq)
{
    uint32_t ret = 0;
    uint32_t speed;
    uint32_t idx;

    for (speed = 0; speed <= (uint32_t)I2C_SPEED_FREQ_FAST_PLUS; speed++)
    {
        if ((i2c_freq >= I2C_Charac[speed].freq_min) && (i2c_freq <= I2C_Charac[speed].freq_max))
        {
            I2C_Compute_PRESC_SCLDEL_SDADEL(clock_src_freq, speed);
            idx = I2C_Compute_SCLL_SCLH(clock_src_freq, speed);

            if (idx < I2C_VALID_TIMING_NBR)
            {
                ret = ((I2c_valid_timing[idx].presc & 0x0FU) << 28) | ((I2c_valid_timing[idx].tscldel & 0x0FU) << 20) |
                      ((I2c_valid_timing[idx].tsdadel & 0x0FU) << 16) | ((I2c_valid_timing[idx].sclh & 0xFFU) << 8) |
                      ((I2c_valid_timing[idx].scll & 0xFFU) << 0);
            }
            break;
        }
    }
    return ret;
}

bool TouchInterface::Initialize(TouchInterfaceConfig config)
{
    touchScreenAddress = config.Address;
    i2cBusNumber = config.I2c_bus_number;

    /*** Configure the GPIOs ***/
    BUS_I2C4_SCL_GPIO_CLK_ENABLE(); /* Enable SCL GPIO clock */
    BUS_I2C4_SDA_GPIO_CLK_ENABLE(); /* Enable SDA GPIO clock */

    GPIO_InitTypeDef gpio_init_i2c_structure;
    /* Configure I2C Tx as alternate function */
    gpio_init_i2c_structure.Pin = BUS_I2C4_SCL_PIN;
    gpio_init_i2c_structure.Mode = GPIO_MODE_AF_OD;
    gpio_init_i2c_structure.Pull = GPIO_NOPULL;
    gpio_init_i2c_structure.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_i2c_structure.Alternate = BUS_I2C4_SCL_AF;
    HAL_GPIO_Init(BUS_I2C4_SCL_GPIO_PORT, &gpio_init_i2c_structure);

    /* Configure I2C Rx as alternate function */
    gpio_init_i2c_structure.Pin = BUS_I2C4_SDA_PIN;
    gpio_init_i2c_structure.Mode = GPIO_MODE_AF_OD;
    gpio_init_i2c_structure.Pull = GPIO_NOPULL;
    gpio_init_i2c_structure.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_i2c_structure.Alternate = BUS_I2C4_SDA_AF;
    HAL_GPIO_Init(BUS_I2C4_SDA_GPIO_PORT, &gpio_init_i2c_structure);

    BUS_I2C4_CLK_ENABLE();    /* Enable I2C clock */
    BUS_I2C4_FORCE_RESET();   /* Force the I2C peripheral clock reset */
    BUS_I2C4_RELEASE_RESET(); /* Release the I2C peripheral clock reset */

    // TS_INT pin is active on low level on new touch available
    GPIO_InitTypeDef gpio_init_structure;
    gpio_init_structure.Pin = TS_INT_PIN;
    gpio_init_structure.Pull = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_structure.Mode = GPIO_MODE_IT_FALLING;
    HAL_GPIO_Init(TS_INT_GPIO_PORT, &gpio_init_structure);

    HAL_NVIC_SetPriority((IRQn_Type)(TS_INT_EXTI_IRQn), BSP_TS_IT_PRIORITY, 0x00);
    //    HAL_NVIC_EnableIRQ((IRQn_Type)(TS_INT_EXTI_IRQn));
    HAL_NVIC_DisableIRQ((IRQn_Type)(TS_INT_EXTI_IRQn));

    ts_i2c.Instance = BUS_I2C4;
    ts_i2c.State = HAL_I2C_STATE_READY;
    ts_i2c.PreviousState = HAL_I2C_MODE_NONE;
    ts_i2c.Mode = HAL_I2C_MODE_NONE;

    uint32_t result = HAL_RCC_GetPCLK2Freq(); // Returns the PCLK2 frequency
    ts_i2c.Init.Timing = I2C_GetTiming(result, FREQ_100KHZ);
    ts_i2c.Init.OwnAddress1 = 0;
    ts_i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    ts_i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    ts_i2c.Init.OwnAddress2 = 0;
    ts_i2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    ts_i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    ts_i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    __HAL_I2C_DISABLE(&ts_i2c);
    ts_i2c.Instance->TIMINGR = ts_i2c.Init.Timing & TIMING_CLEAR_MASK;  // Configure I2Cx: Frequency range
    ts_i2c.Instance->OAR1 &= ~I2C_OAR1_OA1EN;                           // Disable Own Address1 before set
                                                                        // the Own Address1 configuration
    ts_i2c.Instance->OAR1 = (I2C_OAR1_OA1EN | ts_i2c.Init.OwnAddress1); // I2C_ADDRESSINGMODE_7BIT
    ts_i2c.Instance->CR2 |= (I2C_CR2_AUTOEND | I2C_CR2_NACK); // Enable the AUTOEND by default, and enable NACK (should
                                                              // be disable only during Slave process
    ts_i2c.Instance->OAR2 &= ~I2C_DUALADDRESS_ENABLE;         // Disable Own Address2 before set the Own
                                                              // Address2 configuration
    ts_i2c.Instance->OAR2 =
        (ts_i2c.Init.DualAddressMode | ts_i2c.Init.OwnAddress2 |
         (ts_i2c.Init.OwnAddress2Masks << 8)); // Configure I2Cx: Dual mode and Own Address2
    ts_i2c.Instance->CR1 =
        (ts_i2c.Init.GeneralCallMode | ts_i2c.Init.NoStretchMode); // Configure I2Cx: Generalcall and NoStretch mode
    __HAL_I2C_ENABLE(&ts_i2c);

    return true;
}

bool TouchInterface::Write(CLR_UINT8 *dataToSend, CLR_UINT16 numberOfValuesToSend)
{
    return true;
}

bool TouchInterface::Write_Read(
    CLR_UINT8 *dataToSend,
    CLR_UINT16 numberOfValuesToSend,
    CLR_UINT8 *dataReturned,
    CLR_UINT16 numberValuesExpected)
{
    uint16_t Reg = (uint16_t)*dataToSend;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &ts_i2c,
        touchScreenAddress,
        Reg,
        I2C_MEMADD_SIZE_8BIT,
        dataReturned,
        numberValuesExpected,
        1000);
    return (status == HAL_OK);
}

//------------------------------------------------------------------------------------------------

HAL_StatusTypeDef HAL_I2C_Mem_Read(
    I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress,
    uint16_t MemAddress,
    uint16_t MemAddSize,
    uint8_t *pData,
    uint16_t Size,
    uint32_t Timeout)
{
    uint32_t tickstart;

    /* Check the parameters */
    assert_param(IS_I2C_MEMADD_SIZE(MemAddSize));

    if (hi2c->State == HAL_I2C_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
            return HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(hi2c);

        /* Init tickstart for timeout management*/
        tickstart = HAL_GetTick();

        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY, tickstart) != HAL_OK)
        {
            return HAL_ERROR;
        }

        hi2c->State = HAL_I2C_STATE_BUSY_RX;
        hi2c->Mode = HAL_I2C_MODE_MEM;
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        hi2c->pBuffPtr = pData;
        hi2c->XferCount = Size;
        hi2c->XferISR = NULL;

        /* Send Slave Address and Memory Address */
        if (I2C_RequestMemoryRead(hi2c, DevAddress, MemAddress, MemAddSize, Timeout, tickstart) != HAL_OK)
        {
            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);
            return HAL_ERROR;
        }

        /* Send Slave Address */
        /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and
         * generate RESTART */
        if (hi2c->XferCount > MAX_NBYTE_SIZE)
        {
            hi2c->XferSize = MAX_NBYTE_SIZE;
            I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE, I2C_GENERATE_START_READ);
        }
        else
        {
            hi2c->XferSize = hi2c->XferCount;
            I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_GENERATE_START_READ);
        }

        do
        {
            /* Wait until RXNE flag is set */
            if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_RXNE, RESET, Timeout, tickstart) != HAL_OK)
            {
                return HAL_ERROR;
            }

            /* Read data from RXDR */
            *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->RXDR;

            /* Increment Buffer pointer */
            hi2c->pBuffPtr++;

            hi2c->XferSize--;
            hi2c->XferCount--;

            if ((hi2c->XferCount != 0U) && (hi2c->XferSize == 0U))
            {
                /* Wait until TCR flag is set */
                if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, RESET, Timeout, tickstart) != HAL_OK)
                {
                    return HAL_ERROR;
                }

                if (hi2c->XferCount > MAX_NBYTE_SIZE)
                {
                    hi2c->XferSize = MAX_NBYTE_SIZE;
                    I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
                }
                else
                {
                    hi2c->XferSize = hi2c->XferCount;
                    I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
                }
            }
        } while (hi2c->XferCount > 0U);

        /* No need to Check TC flag, with AUTOEND mode the stop is automatically
         * generated */
        /* Wait until STOPF flag is reset */
        if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
        {
            return HAL_ERROR;
        }

        /* Clear STOP Flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

        /* Clear Configuration Register 2 */
        I2C_RESET_CR2(hi2c);

        hi2c->State = HAL_I2C_STATE_READY;
        hi2c->Mode = HAL_I2C_MODE_NONE;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

static HAL_StatusTypeDef I2C_WaitOnSTOPFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)
{
    while (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF) == RESET)
    {
        /* Check if a NACK is detected */
        if (I2C_IsAcknowledgeFailed(hi2c, Timeout, Tickstart) != HAL_OK)
        {
            return HAL_ERROR;
        }

        /* Check for the Timeout */
        if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
        {
            hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
            hi2c->State = HAL_I2C_STATE_READY;
            hi2c->Mode = HAL_I2C_MODE_NONE;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

static HAL_StatusTypeDef I2C_WaitOnFlagUntilTimeout(
    I2C_HandleTypeDef *hi2c,
    uint32_t Flag,
    FlagStatus Status,
    uint32_t Timeout,
    uint32_t Tickstart)
{
    while (__HAL_I2C_GET_FLAG(hi2c, Flag) == Status)
    {
        /* Check for the Timeout */
        if (Timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
            {
                hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
                hi2c->State = HAL_I2C_STATE_READY;
                hi2c->Mode = HAL_I2C_MODE_NONE;

                /* Process Unlocked */
                __HAL_UNLOCK(hi2c);
                return HAL_ERROR;
            }
        }
    }
    return HAL_OK;
}

static HAL_StatusTypeDef I2C_RequestMemoryRead(
    I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress,
    uint16_t MemAddress,
    uint16_t MemAddSize,
    uint32_t Timeout,
    uint32_t Tickstart)
{
    I2C_TransferConfig(hi2c, DevAddress, (uint8_t)MemAddSize, I2C_SOFTEND_MODE, I2C_GENERATE_START_WRITE);

    /* Wait until TXIS flag is set */
    if (I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout, Tickstart) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* If Memory address size is 8Bit */
    if (MemAddSize == I2C_MEMADD_SIZE_8BIT)
    {
        /* Send Memory Address */
        hi2c->Instance->TXDR = I2C_MEM_ADD_LSB(MemAddress);
    }
    /* If Memory address size is 16Bit */
    else
    {
        /* Send MSB of Memory Address */
        hi2c->Instance->TXDR = I2C_MEM_ADD_MSB(MemAddress);

        /* Wait until TXIS flag is set */
        if (I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout, Tickstart) != HAL_OK)
        {
            return HAL_ERROR;
        }

        /* Send LSB of Memory Address */
        hi2c->Instance->TXDR = I2C_MEM_ADD_LSB(MemAddress);
    }

    /* Wait until TC flag is set */
    if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TC, RESET, Timeout, Tickstart) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef I2C_WaitOnTXISFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)
{
    while (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXIS) == RESET)
    {
        /* Check if a NACK is detected */
        if (I2C_IsAcknowledgeFailed(hi2c, Timeout, Tickstart) != HAL_OK)
        {
            return HAL_ERROR;
        }

        /* Check for the Timeout */
        if (Timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
            {
                hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
                hi2c->State = HAL_I2C_STATE_READY;
                hi2c->Mode = HAL_I2C_MODE_NONE;

                /* Process Unlocked */
                __HAL_UNLOCK(hi2c);

                return HAL_ERROR;
            }
        }
    }
    return HAL_OK;
}

static HAL_StatusTypeDef I2C_IsAcknowledgeFailed(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)
{
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF) == SET)
    {
        /* Wait until STOP Flag is reset */
        /* AutoEnd should be initiate after AF */
        while (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF) == RESET)
        {
            /* Check for the Timeout */
            if (Timeout != HAL_MAX_DELAY)
            {
                if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
                {
                    hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
                    hi2c->State = HAL_I2C_STATE_READY;
                    hi2c->Mode = HAL_I2C_MODE_NONE;

                    /* Process Unlocked */
                    __HAL_UNLOCK(hi2c);

                    return HAL_ERROR;
                }
            }
        }

        /* Clear NACKF Flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

        /* Clear STOP Flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

        /* Flush TX register */
        I2C_Flush_TXDR(hi2c);

        /* Clear Configuration Register 2 */
        I2C_RESET_CR2(hi2c);

        hi2c->ErrorCode |= HAL_I2C_ERROR_AF;
        hi2c->State = HAL_I2C_STATE_READY;
        hi2c->Mode = HAL_I2C_MODE_NONE;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
    }
    return HAL_OK;
}
static void I2C_Flush_TXDR(I2C_HandleTypeDef *hi2c)
{
    /* If a pending TXIS flag is set */
    /* Write a dummy data in TXDR to clear it */
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXIS) != RESET)
    {
        hi2c->Instance->TXDR = 0x00U;
    }

    /* Flush TX register if not empty */
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXE) == RESET)
    {
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_TXE);
    }
}

static void I2C_TransferConfig(
    I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress,
    uint8_t Size,
    uint32_t Mode,
    uint32_t Request)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_INSTANCE(hi2c->Instance));
    assert_param(IS_TRANSFER_MODE(Mode));
    assert_param(IS_TRANSFER_REQUEST(Request));

    /* update CR2 register */
    MODIFY_REG(
        hi2c->Instance->CR2,
        ((I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND |
          (I2C_CR2_RD_WRN & (uint32_t)(Request >> (31U - I2C_CR2_RD_WRN_Pos))) | I2C_CR2_START | I2C_CR2_STOP)),
        (uint32_t)(((uint32_t)DevAddress & I2C_CR2_SADD) | (((uint32_t)Size << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES) | (uint32_t)Mode | (uint32_t)Request));
}
