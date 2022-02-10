#pragma once
//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2021 STMicroelectronics.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include "stm32h7xx.h" // Includes    "stm32h735xx.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_rcc.h"
#include <stm32h7xx_ll_sdmmc.h>
#include <stm32h7xx_ll_tim.h>

//---------------------------------------------------------------------------------------------
// Definition for hyperam


#define OSPI_RAM_CS_PIN LL_GPIO_PIN_12
#define OSPI_RAM_DQS_PIN LL_GPIO_PIN_12
#define OSPI_RAM_CLK_PIN LL_GPIO_PIN_4
#define OSPI_RAM_D0_PIN LL_GPIO_PIN_0
#define OSPI_RAM_D1_PIN LL_GPIO_PIN_1
#define OSPI_RAM_D2_PIN LL_GPIO_PIN_2
#define OSPI_RAM_D3_PIN LL_GPIO_PIN_3
#define OSPI_RAM_D4_PIN LL_GPIO_PIN_0
#define OSPI_RAM_D5_PIN LL_GPIO_PIN_1
#define OSPI_RAM_D6_PIN LL_GPIO_PIN_10
#define OSPI_RAM_D7_PIN LL_GPIO_PIN_11


#define S70KL1281_RAM_SIZE         0x1000000 /* 128 MBits => 16 MBytes */
#define BSP_OSPI_RAM_BurstLength_t S70KL1281_BurstLength_t

#define S70KL1281_CR0_BLENGTH_128 0x0000U /*!< 128 bytes burst length */
#define S70KL1281_CR0_BLENGTH_64  0x0001U /*!< 64 bytes burst length */
#define S70KL1281_CR0_BLENGTH_16  0x0002U /*!< 16 bytes burst length */
#define S70KL1281_CR0_BLENGTH_32  0x0003U /*!< 32 bytes burst length */

typedef enum
{
    S70KL1281_BURST_16_BYTES = S70KL1281_CR0_BLENGTH_16,
    S70KL1281_BURST_32_BYTES = S70KL1281_CR0_BLENGTH_32,
    S70KL1281_BURST_64_BYTES = S70KL1281_CR0_BLENGTH_64,
    S70KL1281_BURST_128_BYTES = S70KL1281_CR0_BLENGTH_128
} S70KL1281_BurstLength_t;

#define S70KL1281_CR0_BLENGTH_32 0x0003U /*!< 32 bytes burst length */

#define BSP_OSPI_RAM_BURST_32_BYTES         (BSP_OSPI_RAM_BurstLength_t) S70KL1281_CR0_BLENGTH_32



#define OSPI_FUNCTIONAL_MODE_INDIRECT_READ  ((uint32_t)OCTOSPI_CR_FMODE_0) /*!< Indirect read mode     */
#define OSPI_FUNCTIONAL_MODE_MEMORY_MAPPED  ((uint32_t)OCTOSPI_CR_FMODE)   /*!< Memory-mapped mode     */
#define OSPI_FUNCTIONAL_MODE_INDIRECT_WRITE ((uint32_t)0x00000000)         /*!< Indirect write mode    */

#define OSPI_ENABLE(__HANDLE__)                   SET_BIT((__HANDLE__)->Instance->CR, OCTOSPI_CR_EN)
#define OSPI_DUALQUAD_DISABLE                       ((uint32_t)0x00000000U)         // Dual-Quad mode disabled */
#define OSPI_MEMTYPE_HYPERBUS                       ((uint32_t)OCTOSPI_DCR1_MTYP_2) // Hyperbus mode     */
#define OSPI_FREERUNCLK_DISABLE                     ((uint32_t)0x00000000U) // CLK is not free running               */
#define OSPI_CLOCK_MODE_0                           ((uint32_t)0x00000000U) // CLK must stay low while nCS is high  */
#define OSPI_WRAP_NOT_SUPPORTED                     ((uint32_t)0x00000000U) // wrapped reads are not supported by the memory
#define OSPI_SAMPLE_SHIFTING_NONE                   ((uint32_t)0x00000000U)      /*!< No shift        */
#define OSPI_DHQC_ENABLE                            ((uint32_t)OCTOSPI_TCR_DHQC) /*!< Delay Hold 1/4 cycle */
#define HAL_OSPI_ADDRESS_NONE                           ((uint32_t)0x00000000U)      /*!< No address               */
#define OSPI_LATENCY_ON_WRITE                       ((uint32_t)0x00000000U)     /*!< Latency on write accesses    */
#define OSPI_FIXED_LATENCY                          ((uint32_t)OCTOSPI_HLCR_LM) /*!< Fixed latency            */
#define OSPI_TIMEOUT_DEFAULT_VALUE                  ((uint32_t)5000U)           /* 5 s */
#define __HAL_OSPI_ENABLE_IT(__HANDLE__, __INTERRUPT__) SET_BIT((__HANDLE__)->Instance->CR, (__INTERRUPT__))
#define OSPI_FLAG_TC                                OCTOSPI_SR_TCF // Programmed number of data have been transferred or aborted
#define OSPI_FLAG_FT                                                                                               \
    OCTOSPI_SR_FTF // Fifo threshold reached or data left after read from memory
                   // is complete
#define HAL_OSPI_ERROR_INVALID_PARAM     ((uint32_t)0x00000008U) /*!< Invalid parameters error                   */
#define OSPI_REGISTER_ADDRESS_SPACE  ((uint32_t)OCTOSPI_DCR1_MTYP_0) /*!< HyperBus register mode */
#define OSPI_NO_LATENCY_ON_WRITE     ((uint32_t)OCTOSPI_HLCR_WZL)    /*!< No latency on write accesses */
#define HAL_OSPI_MEMORY_ADDRESS_SPACE    ((uint32_t)0x00000000U)         /*!< HyperBus memory mode   */
#define OSPI_ADDRESS_32_BITS         ((uint32_t)OCTOSPI_CCR_ADSIZE)  /*!< 32-bit address */
#define OSPI_DQS_ENABLE              ((uint32_t)OCTOSPI_CCR_DQSE)    /*!< DQS enabled  */
#define OSPI_TIMEOUT_COUNTER_DISABLE ((uint32_t)0x00000000U) /*!< Timeout counter disabled, nCS remains active */
#define HAL_OSPI_ERROR_INVALID_SEQUENCE  ((uint32_t)0x00000010U) /*!< Sequence of the state machine is incorrect */
#define HAL_OSPI_ERROR_NONE              ((uint32_t)0x00000000U) /*!< No error                                   */
#define HAL_OSPI_STATE_RESET             ((uint32_t)0x00000000U) /*!< Initial state */
#define OSPI_FREERUNCLK_ENABLE       ((uint32_t)OCTOSPI_DCR1_FRCK) /*!< CLK is free running (always provided) */
#define HAL_OSPI_STATE_HYPERBUS_INIT                                                                                   \
    ((uint32_t)0x00000001U)                                    // Initialization done in hyperbus mode but timing
                                                               // configuration not done
#define HAL_OSPI_STATE_BUSY_MEM_MAPPED ((uint32_t)0x00000088U) /*!< Memory-mapped on-going */
#define OSPI_TIMEOUT_COUNTER_ENABLE                                                                                \
    ((uint32_t)OCTOSPI_CR_TCEN)                        // Timeout counter enabled, nCS released when
                                                       // timeout expires
#define OSPI_FLAG_TO       OCTOSPI_SR_TOF          /*!< Timeout flag: timeout occurs in memory-mapped mode */
#define OSPI_IT_TO         OCTOSPI_CR_TOIE         /*!< Interrupt on the timeout flag           */
#define HAL_OSPI_STATE_ERROR   ((uint32_t)0x00000200U) // Blocking error, driver should be re-initialized
#define HAL_OSPI_ERROR_TIMEOUT ((uint32_t)0x00000001U) /*!< Timeout error                              */
#define __HAL_OSPI_GET_FLAG(__HANDLE__, __FLAG__)                                                                      \
    ((READ_BIT((__HANDLE__)->Instance->SR, (__FLAG__)) != 0U) ? SET : RESET)
#define OSPI_CLEAR_FLAG(__HANDLE__, __FLAG__) WRITE_REG((__HANDLE__)->Instance->FCR, (__FLAG__))
#define HAL_OSPI_STATE_READY                        ((uint32_t)0x00000002U) /*!< Driver ready to be used */
#define OSPI_FLAG_BUSY                          OCTOSPI_SR_BUSY         /*!< Busy flag: operation is ongoing */
#define HAL_OSPI_STATE_CMD_CFG                                                                                         \
    ((uint32_t)0x00000004U) // Command (regular or hyperbus) configured, ready for
                            // an action

#define RW_RECOVERY_TIME                      3U /* 40ns @ 60MHz */
#define DEFAULT_INITIAL_LATENCY               6U
#define OPTIMAL_FIXED_INITIAL_LATENCY         3U
#define S70KL1281_CR0_IL_3_CLOCK              0x00E0U /*!< 3 clock latency */
#define S70KL1281_CR0_FLE                     0x0008U /*!< Fixed latency enable */
#define S70KL1281_CR0_IL                      0x00F0U /*!< Initial latency */
#define S70KL1281_CR0_HBE                     0x0004U /*!< Hybrid burst enable */
#define S70KL1281_CR0_BLENGTH                 0x0003U /*!< Burst length */
#define S70KL1281_CR0_ADDRESS                 0x00001000U
#define S70KL1281_ERROR                       (-1)
#define S70KL1281_OK                          (0)
#define OPTIMAL_FIXED_INITIAL_LATENCY_REG_VAL S70KL1281_CR0_IL_3_CLOCK

// Definition for OCTOSPI2 clock resources

#define OSPI2_CLK_ENABLE()            __HAL_RCC_OSPI2_CLK_ENABLE()
#define OSPI2_CLK_DISABLE()           __HAL_RCC_OSPI2_CLK_DISABLE()
#define OSPI2_CS_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOG_CLK_ENABLE()
#define OSPI2_CLK_P_GPIO_CLK_ENABLE() __HAL_RCC_GPIOF_CLK_ENABLE()
#define OSPI2_D0_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOF_CLK_ENABLE()
#define OSPI2_D1_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOF_CLK_ENABLE()
#define OSPI2_D2_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOF_CLK_ENABLE()
#define OSPI2_D3_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOF_CLK_ENABLE()
#define OSPI2_D4_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOG_CLK_ENABLE()
#define OSPI2_D5_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOG_CLK_ENABLE()
#define OSPI2_D6_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOG_CLK_ENABLE()
#define OSPI2_D7_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOG_CLK_ENABLE()
#define OSPI2_DQS_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOF_CLK_ENABLE()

#define OSP2_FORCE_RESET()   __HAL_RCC_OSPI2_FORCE_RESET()
#define OSP2_RELEASE_RESET() __HAL_RCC_OSPI2_RELEASE_RESET()

typedef enum
{
    HAL_MDMA_STATE_RESET = 0x00U, /*!< MDMA not yet initialized or disabled */
    HAL_MDMA_STATE_READY = 0x01U, /*!< MDMA initialized and ready for use   */
    HAL_MDMA_STATE_BUSY = 0x02U,  /*!< MDMA process is ongoing              */
    HAL_MDMA_STATE_ERROR = 0x03U, /*!< MDMA error state                     */
    HAL_MDMA_STATE_ABORT = 0x04U, /*!< MDMA Abort state                     */

} HAL_MDMA_StateTypeDef;

typedef struct
{
    __IO uint32_t CTCR;     /*!< New CTCR register configuration for the given MDMA
                               linked list node   */
    __IO uint32_t CBNDTR;   /*!< New CBNDTR register configuration for the given
                               MDMA linked list node */
    __IO uint32_t CSAR;     /*!< New CSAR register configuration for the given MDMA
                               linked list node   */
    __IO uint32_t CDAR;     /*!< New CDAR register configuration for the given MDMA
                               linked list node   */
    __IO uint32_t CBRUR;    /*!< New CBRUR register configuration for the given MDMA
                               linked list node  */
    __IO uint32_t CLAR;     /*!< New CLAR register configuration for the given MDMA
                               linked list node   */
    __IO uint32_t CTBR;     /*!< New CTBR register configuration for the given MDMA
                               linked list node   */
    __IO uint32_t Reserved; /*!< Reserved register */
    __IO uint32_t CMAR;     /*!< New CMAR register configuration for the given MDMA
                               linked list node   */
    __IO uint32_t CMDR;     /*!< New CMDR register configuration for the given MDMA
                               linked list node   */

} MDMA_LinkNodeTypeDef;

typedef struct
{

    uint32_t Request; /*!< Specifies the MDMA request.
                          This parameter can be a value of @ref
                         MDMA_Request_selection*/

    uint32_t TransferTriggerMode; /*!< Specifies the Trigger Transfer mode : each
                                     request triggers a : a buffer transfer, a block
                                     transfer, a repeated block transfer or a linked
                                     list transfer This parameter can be a value of
                                     @ref MDMA_Transfer_TriggerMode  */

    uint32_t Priority; /*!< Specifies the software priority for the MDMAy channelx.
                           This parameter can be a value of @ref MDMA_Priority_level */

    uint32_t Endianness; /*!< Specifies if the MDMA transactions preserve the
                            Little endianness. This parameter can be a value of
                            @ref MDMA_Endianness */

    uint32_t SourceInc; /*!< Specifies if the Source increment mode .
                            This parameter can be a value of @ref
                           MDMA_Source_increment_mode */

    uint32_t DestinationInc; /*!< Specifies if the Destination increment mode .
                                 This parameter can be a value of @ref
                                MDMA_Destination_increment_mode */

    uint32_t SourceDataSize; /*!< Specifies the source data size.
                                 This parameter can be a value of @ref
                                MDMA_Source_data_size */

    uint32_t DestDataSize; /*!< Specifies the destination data size.
                                This parameter can be a value of @ref
                              MDMA_Destination_data_size */

    uint32_t DataAlignment; /*!< Specifies the source to destination Memory data
                               packing/padding mode. This parameter can be a value of
                               @ref MDMA_data_Alignment */

    uint32_t BufferTransferLength; /*!< Specifies the buffer Transfer Length
                                      (number of bytes), this is the number of
                                      bytes to be transferred in a single transfer
                                      (1 byte to 128 bytes)*/

    uint32_t SourceBurst; /*!< Specifies the Burst transfer configuration for the
                             source memory transfers. It specifies the amount of data
                             to be transferred in a single non interruptable
                              transaction.
                              This parameter can be a value of @ref MDMA_Source_burst
                              @note : the burst may be FIXED/INCR based on SourceInc
                             value , the BURST must be programmed as to ensure that the
                             burst size will be lower than than BufferTransferLength */

    uint32_t DestBurst; /*!< Specifies the Burst transfer configuration for the
                           destination memory transfers. It specifies the amount
                           of data to be transferred in a single non interruptable
                             transaction.
                             This parameter can be a value of @ref
                           MDMA_Destination_burst
                             @note : the burst may be FIXED/INCR based on
                           DestinationInc value , the BURST must be programmed as
                           to ensure that the burst size will be lower than than
                             BufferTransferLength */

    int32_t SourceBlockAddressOffset; /*!< this field specifies the Next block source
                                         address offset signed value : if > 0 then
                                         increment the next block source Address by
                                         offset from where the last block ends if < 0
                                         then  decrement the next block source Address
                                         by offset from where the last block ends if
                                         == 0, the next block source address starts
                                         from where the last block ends
                                       */

    int32_t DestBlockAddressOffset; /*!< this field specifies the Next block
                                       destination address offset signed value :
                                       if > 0 then  increment the next block
                                       destination Address by offset from where
                                       the last block ends if < 0 then  decrement
                                       the next block destination Address by
                                       offset from where the last block ends if ==
                                       0, the next block destination address
                                       starts from where the last block ends
                                    */

} MDMA_InitTypeDef;

typedef struct
{
    uint32_t FifoThreshold;         /* This is the threshold used by the Peripheral to generate
                                       the interrupt indicating that data are available in
                                       reception or free place is available in transmission.
                                       This parameter can be a value between 1 and 32 */
    uint32_t DualQuad;              /* It enables or not the dual-quad mode which allow to
                                       access up to quad mode on two different devices to
                                       increase the throughput. This parameter can be a value
                                       of @ref OSPI_DualQuad */
    uint32_t MemoryType;            /* It indicates the external device type connected to the
                                       OSPI. This parameter can be a value of @ref
                                       OSPI_MemoryType */
    uint32_t DeviceSize;            /* It defines the size of the external device connected
                                       to the OSPI, it corresponds to the number of address
                                       bits required to access the external device. This
                                       parameter can be a value between 1 and 32 */
    uint32_t ChipSelectHighTime;    /* It defines the minimun number of clocks which the
                                       chip select must remain high between commands.
                                       This parameter can be a value between 1 and 8 */
    uint32_t FreeRunningClock;      /* It enables or not the free running clock.
                                       This parameter can be a value of @ref
                                       OSPI_FreeRunningClock */
    uint32_t ClockMode;             /* It indicates the level of clock when the chip select is
                                       released. This parameter can be a value of @ref
                                       OSPI_ClockMode */
    uint32_t WrapSize;              /* It indicates the wrap-size corresponding the external
                                       device configuration.  This parameter can be a value of
                                       @ref OSPI_WrapSize */
    uint32_t ClockPrescaler;        /* It specifies the prescaler factor used for generating
                                       the external clock based on the AHB clock.
                                       This parameter can be a value between 1 and 256 */
    uint32_t SampleShifting;        /* It allows to delay to 1/2 cycle the data sampling
                                       in order to take in account external signal
                                       delays. This parameter can be a value of @ref
                                       OSPI_SampleShifting */
    uint32_t DelayHoldQuarterCycle; /* It allows to hold to 1/4 cycle the data.
                                       This parameter can be a value of @ref
                                       OSPI_DelayHoldQuarterCycle */
    uint32_t ChipSelectBoundary;    /* It enables the transaction boundary feature and
                                       defines the boundary of bytes to release the chip
                                       select.
                                       This parameter can be a value between 0 and 31 */
    uint32_t ClkChipSelectHighTime; /* It defines the number of clocks provided on the
                                       CLK/nCLK pins when the chip select is set to
                                       high at the end of a transaction.
                                       This parameter can be a value between 0 and 7 */
    uint32_t DelayBlockBypass;      /* It enables the delay block bypass, so the
                                       sampling is not affected by the delay block.
                                       This parameter can be a value of @ref
                                       OSPI_DelayBlockBypass */
    uint32_t MaxTran;               /* It enables the communication regulation feature. The chip
                                       select is released every MaxTran+1 bytes when the other
                                       OctoSPI request the access to the bus.
                                       This parameter can be a value between 0 and 255 */
    uint32_t Refresh;               /* It enables the refresh rate feature. The chip select is
                                       released every Refresh+1 clock cycles.
                                       This parameter can be a value between 0 and 0xFFFFFFFF */
} OSPI_InitTypeDef;

typedef struct __MDMA_HandleTypeDef
{
    MDMA_Channel_TypeDef *Instance; /*!< Register base address                  */

    MDMA_InitTypeDef Init; /*!< MDMA communication parameters          */

    HAL_LockTypeDef Lock; /*!< MDMA locking object                    */

    __IO HAL_MDMA_StateTypeDef State; /*!< MDMA transfer state */

    void *Parent; /*!< Parent object state                    */

    void (*XferCpltCallback)(struct __MDMA_HandleTypeDef *hmdma); /*!< MDMA transfer complete callback */

    void (*XferBufferCpltCallback)(struct __MDMA_HandleTypeDef *hmdma); /*!< MDMA buffer transfer complete callback */

    void (*XferBlockCpltCallback)(struct __MDMA_HandleTypeDef *hmdma); /*!< MDMA block transfer complete callback  */

    void (*XferRepeatBlockCpltCallback)(
        struct __MDMA_HandleTypeDef *hmdma); /*!< MDMA block transfer repeat callback    */

    void (*XferErrorCallback)(struct __MDMA_HandleTypeDef *hmdma); /*!< MDMA transfer error callback */

    void (*XferAbortCallback)(struct __MDMA_HandleTypeDef *hmdma); /*!< MDMA transfer Abort callback */

    MDMA_LinkNodeTypeDef *FirstLinkedListNodeAddress; /*!< specifies the first node address of the
                                                         transfer list (after the initial node
                                                         defined by the Init struct) this parameter
                                                         is used internally by the MDMA driver to
                                                         construct the linked list node
                                                      */

    MDMA_LinkNodeTypeDef *LastLinkedListNodeAddress; /*!< specifies the last node address of the
                                                        transfer list this parameter is used
                                                        internally by the MDMA driver to construct
                                                        the linked list node
                                                      */
    uint32_t LinkedListNodeCounter;                  /*!< Number of nodes in the MDMA linked list */

    __IO uint32_t ErrorCode; /*!< MDMA Error code                        */

} MDMA_HandleTypeDef;

typedef struct
{
    OCTOSPI_TypeDef *Instance; /* OSPI registers base address */
    OSPI_InitTypeDef Init;     /* OSPI initialization parameters                   */
    uint8_t *pBuffPtr;         /* Address of the OSPI buffer for transfer          */
    __IO uint32_t XferSize;    /* Number of data to transfer                       */
    __IO uint32_t XferCount;   /* Counter of data transferred */
    MDMA_HandleTypeDef *hmdma; /* Handle of the MDMA channel used for the transfer  */
    __IO uint32_t State;       /* Internal state of the OSPI HAL driver            */
    __IO uint32_t ErrorCode;   /* Error code in case of HAL driver internal error  */
    uint32_t Timeout;          /* Timeout used for the OSPI external device access */
} OSPI_HandleTypeDef;

typedef struct
{
    uint32_t RWRecoveryTime;   /* It indicates the number of cycles for the device
                                  read write recovery time.
                                  This parameter can be a value between 0 and 255 */
    uint32_t AccessTime;       /* It indicates the number of cycles for the device acces
                                  time. This parameter can be a value between 0 and 255 */
    uint32_t WriteZeroLatency; /* It enables or not the latency for the write
                                  access. This parameter can be a value of @ref
                                  OSPI_WriteZeroLatency */
    uint32_t LatencyMode;      /* It configures the latency mode.
                                  This parameter can be a value of @ref OSPI_LatencyMode */
} OSPI_HyperbusCfgTypeDef;

typedef struct
{
    uint32_t TimeOutActivation; /* Specifies if the timeout counter is enabled to
                                   release the chip select.  This parameter can be
                                   a value of @ref OSPI_TimeOutActivation */
    uint32_t TimeOutPeriod;     /* Specifies the number of clock to wait when the FIFO
                                   is full before to release the chip select. This
                                   parameter can be any value between 0 and 0xFFFF */
} OSPI_MemoryMappedTypeDef;
typedef struct
{
    uint32_t AddressSpace; /* It indicates the address space accessed by the
                              command. This parameter can be a value of @ref
                              OSPI_AddressSpace */
    uint32_t Address;      /* It contains the address to be sent tot he device.
                              This parameter can be a value between 0 and 0xFFFFFFFF */
    uint32_t AddressSize;  /* It indicates the size of the address.
                              This parameter can be a value of @ref OSPI_AddressSize */
    uint32_t NbData;       /* It indicates the number of data transferred with this command.
                              This field is only used for indirect mode.
                              This parameter can be a value between 1 and 0xFFFFFFFF
                              In case of autopolling mode, this parameter can be any value
                              between 1 and 4 */
    uint32_t DQSMode;      /* It enables or not the data strobe management.
                              This parameter can be a value of @ref OSPI_DQSMode */
} OSPI_HyperbusCmdTypeDef;
