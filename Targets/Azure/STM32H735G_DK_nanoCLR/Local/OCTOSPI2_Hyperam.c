
#include "stm32h735xx.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_rcc.h"

#define OSPI_TIMEOUT_DEFAULT_VALUE ((uint32_t)5000U) // 5 s

#define OSPI_FUNCTIONAL_MODE_MEMORY_MAPPED ((uint32_t)OCTOSPI_CR_FMODE) // Memory-mapped mode
#define OSPI_WRAP_NOT_SUPPORTED            ((uint32_t)0x00000000U)      // wrapped reads not supported by the memory
#define OSPI_DELAY_BLOCK_USED              ((uint32_t)0x00000000U)      // Sampling clock is delayed by the delay block
#define OSPI_MEMTYPE_HYPERBUS              ((uint32_t)OCTOSPI_DCR1_MTYP_2) // Hyperbus mode
#define OSPI_CLOCK_MODE_0                  ((uint32_t)0x00000000U)         // CLK must stay low while nCS is high
#define OSPI_DUALQUAD_DISABLE              ((uint32_t)0x00000000U)         // Dual-Quad mode disabled
#define OSPI_SAMPLE_SHIFTING_NONE          ((uint32_t)0x00000000U)         // No shift
#define OSPI_DHQC_ENABLE                   ((uint32_t)OCTOSPI_TCR_DHQC)    // Delay Hold 1/4 cycle
#define OSPI_FLAG_BUSY                     OCTOSPI_SR_BUSY                 // Busy flag: operation is ongoing
#define OSPI_LATENCY_ON_WRITE              ((uint32_t)0x00000000U)         // Latency on write accesses
#define OSPI_FIXED_LATENCY                 ((uint32_t)OCTOSPI_HLCR_LM)     // Fixed latency/
#define OSPI_MEMORY_ADDRESS_SPACE          ((uint32_t)0x00000000U)         // HyperBus memory mode
#define OSPI_ADDRESS_32_BITS               ((uint32_t)OCTOSPI_CCR_ADSIZE)  // 32-bit address
#define OSPI_DQS_ENABLE                    ((uint32_t)OCTOSPI_CCR_DQSE)    // DQS enabled
#define OSPI_TIMEOUT_COUNTER_DISABLE       (uint32_t)0x00000000U // Timeout counter disabled, nCS remains active

#define OSPI_RAM_CS_PIN  LL_GPIO_PIN_12 // Port G
#define OSPI_RAM_DQS_PIN LL_GPIO_PIN_12 // Port F
#define OSPI_RAM_CLK_PIN LL_GPIO_PIN_4

#define OSPI_RAM_D0_PIN LL_GPIO_PIN_0
#define OSPI_RAM_D1_PIN LL_GPIO_PIN_1
#define OSPI_RAM_D2_PIN LL_GPIO_PIN_2
#define OSPI_RAM_D3_PIN LL_GPIO_PIN_3
#define OSPI_RAM_D4_PIN LL_GPIO_PIN_0
#define OSPI_RAM_D5_PIN LL_GPIO_PIN_1
#define OSPI_RAM_D6_PIN LL_GPIO_PIN_10
#define OSPI_RAM_D7_PIN LL_GPIO_PIN_11

// Hyperam definitions based on the Integrated Circuit S....
#define OSPI_HYPERRAM_SIZE        24
#define OSPI_HYPERRAM_RW_REC_TIME 3
#define OSPI_HYPERRAM_LATENCY     6

#define OCTOSPI2_DISABLE CLEAR_BIT(OCTOSPI2->CR, OCTOSPI_CR_EN)
#define OCTOSPI2_ENABLE  SET_BIT(OCTOSPI2->CR, OCTOSPI_CR_EN)

//
// OCTOSPI2
// 1. Octal configuration - 8 data lines
// In regular octal-SPI mode, the eight bits are sent/received simultaneously
// over the IO[0:7] signals
// 2. HyperBus protocol
// The HyperBus frame is composed of the following phases:
// • Command / address phase
// • Data phase
// OCTOSPI operating mode, Mmeory-mapped
//
// 3.0 V / 1.8 V,  128 Mb(16 MB),
//     HyperRAM™ Self - Refresh DRAM
// Double-transfer rate (DTR) mode
// 1.8V - 166MHz (faster)

void Initialize_OCTOSPI2_Hyperam()
{
    uint32_t FifoThreshold = 4;
    uint32_t ChipSelectHighTime = 8;
    uint32_t ClkChipSelectHighTime = 0;
    uint32_t ClockPrescaler = 4;
    uint32_t MaxTran = 0;
    uint32_t ChipSelectBoundary = 23;
    uint32_t Refresh = 250; // The chip select should be released every 4µs

    // Initialize OctoSPI
    LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_OSPI2);
    LL_AHB3_GRP1_ForceReset(LL_AHB3_GRP1_PERIPH_OSPI2); // Reset the OctoSPI memory interface
    LL_AHB3_GRP1_ReleaseReset(LL_AHB3_GRP1_PERIPH_OSPI2);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOF); //  Ports F and G have
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOG); //  OCTOSPI2 signals on port pins

    LL_GPIO_InitTypeDef gpio_InitStruct = {0};
    // Common pin settings
    gpio_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    gpio_InitStruct.Pull = LL_GPIO_PULL_UP;
    gpio_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;

    gpio_InitStruct.Pin = OSPI_RAM_CS_PIN; // Chip Select
    gpio_InitStruct.Alternate = LL_GPIO_AF_3;
    LL_GPIO_Init(GPIOG, &gpio_InitStruct);

    gpio_InitStruct.Pin = OSPI_RAM_DQS_PIN;   // DQS
    gpio_InitStruct.Alternate = LL_GPIO_AF_9; // GPIO_AF9_OCTOSPIM_P2
    LL_GPIO_Init(GPIOF, &gpio_InitStruct);

    gpio_InitStruct.Pin = OSPI_RAM_CLK_PIN;   // Clock
    gpio_InitStruct.Alternate = LL_GPIO_AF_9; // GPIO_AF9_OCTOSPIM_P2
    LL_GPIO_Init(GPIOF, &gpio_InitStruct);

    gpio_InitStruct.Pin = OSPI_RAM_D0_PIN | OSPI_RAM_D1_PIN | OSPI_RAM_D2_PIN | OSPI_RAM_D3_PIN;
    gpio_InitStruct.Alternate = LL_GPIO_AF_9; // GPIO_AF9_OCTOSPIM_P2
    LL_GPIO_Init(GPIOF, &gpio_InitStruct);

    gpio_InitStruct.Pin = OSPI_RAM_D4_PIN | OSPI_RAM_D5_PIN | OSPI_RAM_D7_PIN;
    gpio_InitStruct.Alternate = LL_GPIO_AF_9; // GPIO_AF9_OCTOSPIM_P2
    LL_GPIO_Init(GPIOG, &gpio_InitStruct);

    gpio_InitStruct.Pin = OSPI_RAM_D6_PIN;
    gpio_InitStruct.Alternate = LL_GPIO_AF_3; // GPIO_AF9_OCTOSPIM_P2
    LL_GPIO_Init(GPIOG, &gpio_InitStruct);

    // Enable and set OctoSPI interrupt to the lowest priority
    NVIC_SetPriority(OCTOSPI2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x0F, 0));
    NVIC_EnableIRQ((OCTOSPI2_IRQn));

    OCTOSPI2_DISABLE;

    CLEAR_BIT(
        OCTOSPI2->DCR1,     // Device Configuration register 1
        OCTOSPI_DCR1_FRCK); // Disable free running clock

    // Configure
    MODIFY_REG(
        OCTOSPI2->DCR1,                                               // Device Configuration register 1
                                                                      // SETTINGS
        (OCTOSPI_DCR1_MTYP                                            // Memory type
         | OCTOSPI_DCR1_DEVSIZE                                       // Device size
         | OCTOSPI_DCR1_CSHT                                          // Chip select high time
         | OCTOSPI_DCR1_CKCSHT                                        // Clocked chip select high time
         | OCTOSPI_DCR1_DLYBYP                                        // Delay block bypass
         | OCTOSPI_DCR1_FRCK                                          // Free running clock
         | OCTOSPI_DCR1_CKMODE),                                      // Clock mode
                                                                      // VALUES
        OSPI_MEMTYPE_HYPERBUS                                         // Hyperbus
            | ((OSPI_HYPERRAM_SIZE - 1U) << OCTOSPI_DCR1_DEVSIZE_Pos) // Size
            | ((ChipSelectHighTime - 1U) << OCTOSPI_DCR1_CSHT_Pos)    // Chip select high time
            | (ClkChipSelectHighTime << OCTOSPI_DCR1_CKCSHT_Pos)      // Clocked chip select high time
            | OSPI_DELAY_BLOCK_USED                                   // Delay block bypass
            | OSPI_CLOCK_MODE_0);                                     // CLK must stay low while nCS is high

    MODIFY_REG(
        OCTOSPI2->DCR2,         // Device Configuration register 2
        OCTOSPI_DCR2_WRAPSIZE,  //
        OSPI_WRAP_NOT_SUPPORTED // SK does not support wrap
    );
    // Configure clock prescaler
    MODIFY_REG(
        OCTOSPI2->DCR2,                                       //  Device Configuration register 2
        OCTOSPI_DCR2_PRESCALER,                               //
        ((ClockPrescaler - 1U) << OCTOSPI_DCR2_PRESCALER_Pos) // 4
    );

    OCTOSPI2->DCR3 =                                      // Device Configuration register 3
        ((ChipSelectBoundary << OCTOSPI_DCR3_CSBOUND_Pos) //
         | (MaxTran << OCTOSPI_DCR3_MAXTRAN_Pos));        //

    OCTOSPI2->DCR4 = Refresh; //

    MODIFY_REG(
        OCTOSPI2->CR,                                   //   Control register
        OCTOSPI_CR_FTHRES,                              //
        ((FifoThreshold - 1U) << OCTOSPI_CR_FTHRES_Pos) //
    );
    /* Wait till busy flag is reset */
    while (READ_BIT(OCTOSPI2->SR, OSPI_FLAG_BUSY) == SET)
    {
    };

    MODIFY_REG(
        OCTOSPI2->TCR,                                 // Timing Configuration register
        (OCTOSPI_TCR_SSHIFT | OCTOSPI_TCR_DHQC),       //
        (OSPI_SAMPLE_SHIFTING_NONE | OSPI_DHQC_ENABLE) // Configure sample shifting and delay hold
                                                       // quarter cycle
    );
    MODIFY_REG(
        OCTOSPI2->CR,         //  Control register
        OCTOSPI_CR_DQM,       //
        OSPI_DUALQUAD_DISABLE // Configure Dual Quad mode
    );

    OCTOSPI2_ENABLE;

    // Wait till busy flag is reset
    while (READ_BIT(OCTOSPI2->SR, OSPI_FLAG_BUSY) == SET)
    {
    };

    // Configure the Hyperbus to access memory space

    WRITE_REG(
        OCTOSPI2->HLCR,                                       // Hyperbus configuration Latency register
        ((OSPI_HYPERRAM_RW_REC_TIME << OCTOSPI_HLCR_TRWR_Pos) // ccc
         | (OSPI_HYPERRAM_LATENCY << OCTOSPI_HLCR_TACC_Pos)   // cccc
         | OSPI_LATENCY_ON_WRITE                              //
         | OSPI_FIXED_LATENCY));                              // ccc

    // Memory-mapped mode configuration

    /* Wait till busy flag is reset */
    while (READ_BIT(OCTOSPI2->SR, OSPI_FLAG_BUSY) == SET)
    {
    };
    /* Re-initialize the value of the functional mode */
    MODIFY_REG(
        OCTOSPI2->CR,     //  Control register
        OCTOSPI_CR_FMODE, // Functional Mode
        0U);
    /* Configure the address space in the DCR1 register */
    MODIFY_REG(
        OCTOSPI2->DCR1,             //   Device Configuration register 1
        OCTOSPI_DCR1_MTYP_0,        //   Bit 24, Memory Type
        OSPI_MEMORY_ADDRESS_SPACE); //   HyperBus memory mode

    /* Configure the CCR and WCCR registers with the address size and the
       following configuration :
       - DQS signal enabled (used as RWDS)
       - DTR mode enabled on address and data
       - address and data on 8 lines */
    WRITE_REG(
        OCTOSPI2->CCR,           // Communication Configuration register
        (OSPI_DQS_ENABLE         //
         | OCTOSPI_CCR_DDTR      //  Data Double Transfer Rate
         | OCTOSPI_CCR_DMODE_2   //
         | OSPI_ADDRESS_32_BITS  //
         | OCTOSPI_CCR_ADDTR     //
         | OCTOSPI_CCR_ADMODE_2) //
    );
    WRITE_REG(
        OCTOSPI2->WCCR,           //  Write Communication Configuration register
        (OSPI_DQS_ENABLE          //
         | OCTOSPI_WCCR_DDTR      //
         | OCTOSPI_WCCR_DMODE_2   //
         | OSPI_ADDRESS_32_BITS   //
         | OCTOSPI_WCCR_ADDTR     //
         | OCTOSPI_WCCR_ADMODE_2) //
    );
    /* Configure the DLR register with the number of data */
    uint32_t NbData = 1;
    WRITE_REG(
        OCTOSPI2->DLR, // Data Length register
        (NbData - 1U)  //
    );
    /* Configure the AR register with the address value */
    uint32_t Address = 0;
    WRITE_REG(
        OCTOSPI2->AR, //  Address register
        Address       //
    );

    //------------------------------
    // Set Hyperam to memory mapped
    //------------------------------
    while (READ_BIT(OCTOSPI2->SR, OSPI_FLAG_BUSY) == SET)
    {
    };
    /* Configure CR register with functional mode as memory-mapped */
    MODIFY_REG(                                //
        OCTOSPI2->CR,                          //  Control register
        (OCTOSPI_CR_TCEN | OCTOSPI_CR_FMODE),  //  Setting bits, Timeout Counter Enable and Functional Mode
        (OSPI_TIMEOUT_COUNTER_DISABLE          //  Disable Timer Counter
         | OSPI_FUNCTIONAL_MODE_MEMORY_MAPPED) //  Set Functional mode to memory mapped
    );
}
