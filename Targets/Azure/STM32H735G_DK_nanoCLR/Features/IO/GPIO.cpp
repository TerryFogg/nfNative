//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//
#include "sys_dev_gpio_native_target.h"
#include <tx_api.h>
#include <tx_port.h>
#include "stm32h7xx_ll_gpio.h"
#include <stm32h735xx.h>
#include <targetPAL.h>

enum LEDS : int
{
    User_LED_2 = 35, // PC2
    User_LED_1 = 36  // PC3
};
enum ArduinoPin : int
{
    Analog_0 = 33,  // PC0
    Analog_1 = 115, // PH2
    Analog_2 = 252, // PA0_C
    Analog_3 = 253, // PA1_C
    Analog_4 = 254, // PC2_C
    Analog_5 = 255, // PC3_C
    D0 = 32,        // PB15
    D1 = 31,        // PB14
    D2 = 100,       // PG3
    D3 = 1,         // PA0
    D4 = 101,       // PG4
    D5 = 79,        // PE14
    D6 = 64,        // PD15
    D7 = 102,       // PG5
    D8 = 68,        // PE3
    D9 = 24,        // PB7
    D10 = 87,       // PF6
    D11 = 90,       // PF9 - SPI5_MOSI
    D12 = 89,       // PF8 - SPI5_MISO
    D13 = 88,       // PF7 - SPI5_SCK
    D14 = 96,       // PF15 - I2C4_SDA
    D15 = 95        // PF14 - I2C4_SCL
};
enum STMOD : int
{
    ST_1_7_RX = 87,           // PF6
    ST_27_TX = 88,            // PF7
    ST_35_MISO = 89,          // PF8
    ST_45_MOSI = 90,          // PF9
    ST_7_I2C4_SCL = 95,       // PF14
    ST_8_SPI5_MOSI = 92,      // PF11
    ST_9_SPI5_MISO = 120,     // PH7
    ST_10_I2C4_SDA = 96,      // PF15
    ST_11_INT = 126,          // PH12
    ST_12_RST = 114,          // PH1
    ST_13 = 6,                // PA5  _ADC12_INP19, _DAC1_OUT2,
    ST_14_PWM_4_CH3 = 63,     // PD14
    ST_18_DFSDM1_DATIN2 = 72, // PE7
    ST_18_DFSDM1_CKOUT = 74,  // PE9
    ST_19_DFSDM1_DATIN4 = 75, // PE10
    ST_20_DFSDM1_DATIN6 = 94  // PF13
};
// UFBGA176+25 ballout
enum FDCANPin : int
{
    // PA0_C AND THIS IS A ESCOP
    FDCAN2_RX = 22,  // PB5
    FDCAN2_TX = 23,  // PB6
    Audio_Int = 73,  // PE8
    FDCAN3_RX = 87,  // PF6
    FDCAN3_TX = 88,  // PF7
    FDCAN1_TX = 127, // PH13
    FDCAN1_RX = 128  // PH14
};

typedef uint32_t ioline_t;

// The STM32H735 MCU has registers A,B,C,D,E,F,G,H
GPIO_TypeDef *GpioPorts[] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH};
#define PINS_PER_PORT  16
#define GPIO_MAX_PIN   sizeof(GpioPorts) * 16
#define GPIO_PORT(pin) (GpioPorts[pin / PINS_PER_PORT])
#define GPIO_PIN(pin)  (pin - (pin % PINS_PER_PORT) * PINS_PER_PORT)
#define PAL_LOW        0U
#define PAL_HIGH       1U
// Both edges callback
#define PAL_EVENT_MODE_BOTH_EDGES 3U
#define PAL_PORT_BIT(n)           ((uint32_t)(1U << (n)))
#define PAL_STM32_MODE_ALTERNATE  (2U << 0U)
#define PAL_STM32_ALTERNATE(n)    ((n) << 7U)
#define PAL_MODE_ALTERNATE(n)     (PAL_STM32_MODE_ALTERNATE | PAL_STM32_ALTERNATE(n))

static GPIO_PIN InterruptPin;
HAL_COMPLETION m_gpioCompletion; // TODO: ????????????

// Double linkedlist to hold the state of each Input pin
struct gpio_input_state : public HAL_DblLinkedNode<gpio_input_state>
{
    // Pin number
    GPIO_PIN pinNumber;
    // debounce timer for this Pin
    TX_TIMER debounceTimer;
    // Ptr to user ISR or null
    GPIO_INTERRUPT_SERVICE_ROUTINE isrPtr;
    // debounce Millsecs, no debounce=0
    uint32_t debounceMs;
    // Interrupt mode
    uint8_t mode;
    // Param to user isr call
    void *param;
    // Expected state for debounce handler
    bool expected;
    // True if waiting for debounce timer to complete
    bool waitingDebounce;
};

// Double Linked list for GPIO input status
static HAL_DblLinkedList<gpio_input_state> gpioInputList;
//  reserved - 1 bit per pin
static uint16_t pinReserved[sizeof(GpioPorts)];

TX_INTERRUPT_SAVE_AREA

bool IsValidGpioPin(GPIO_PIN pinNumber)
{
    return (pinNumber <= GPIO_MAX_PIN);
}
gpio_input_state *GetGpioWithInterrupt(uint16_t gpioPin)
{
    gpio_input_state *ptr = gpioInputList.FirstNode();
    while (ptr->Next() != NULL)
    {
        if (GPIO_PIN(ptr->pinNumber) == gpioPin)
        {
            return ptr;
        }
        ptr = ptr->Next();
    }
    return NULL;
}
static void DebounceTimerCallback(uint32_t id)
{
    gpio_input_state *pState = (gpio_input_state *)id;
    GPIO_TypeDef *GPIO_port = GPIO_PORT(pState->pinNumber);
    uint32_t PinMask = GPIO_PIN(pState->pinNumber);
    bool actual = LL_GPIO_IsInputPinSet(GPIO_port, PinMask);
    if (actual == pState->expected)
    {
        pState->isrPtr(pState->pinNumber, actual, pState->param);
        if (pState->mode == GPIO_INT_EDGE_BOTH) // both edges
        {
            pState->expected ^= 1;
        }
    }
    pState->waitingDebounce = false;
}
void GpioEventCallback(void *arg)
{
    NATIVE_INTERRUPT_START
    TX_DISABLE
    gpio_input_state *pGpio = (gpio_input_state *)arg;
    if (pGpio != NULL)
    {
        // Ignore any pin changes during debounce
        if (!pGpio->waitingDebounce)
        {
            // check if there is a debounce time set
            if (pGpio->debounceMs > 0)
            {
                // Set flag we are waiting for debounce on this pin
                pGpio->waitingDebounce = true;

                // setup timer
                tx_timer_deactivate(&pGpio->debounceTimer);
                tx_timer_change(&pGpio->debounceTimer, 0, pGpio->debounceMs / 10);
                tx_timer_activate(&pGpio->debounceTimer);
            }
            else
            {
                GPIO_TypeDef *GPIO_port = GPIO_PORT(pGpio->pinNumber);
                uint32_t PinMask = GPIO_PIN(pGpio->pinNumber);
                bool pinSet = LL_GPIO_IsInputPinSet(GPIO_port, PinMask);

                // TODO: ??????????????? TX_RESTORE for isrPtr?
                TX_RESTORE
                pGpio->isrPtr(pGpio->pinNumber, pinSet, pGpio->param);
                TX_DISABLE
            }
        }
    }
    TX_RESTORE
    NATIVE_INTERRUPT_END
}
// Get pointer to gpio_input_state for Gpio pin
gpio_input_state *GetInputState(GPIO_PIN pinNumber)
{
    gpio_input_state *ptr = gpioInputList.FirstNode();
    while (ptr->Next() != NULL)
    {
        if (ptr->pinNumber == pinNumber)
        {
            return ptr;
        }

        ptr = ptr->Next();
    }
    return NULL;
}
// Allocate a new gpio_input_state and add to end of list
// if already exist then just return current ptr
gpio_input_state *AllocateGpioInputState(GPIO_PIN pinNumber)
{
    gpio_input_state *ptr = GetInputState(pinNumber);
    if (ptr == NULL)
    {
        ptr = (gpio_input_state *)platform_malloc(sizeof(gpio_input_state));
        // sanity check
        if (ptr != NULL)
        {
            memset(ptr, 0, sizeof(gpio_input_state));
            ptr->pinNumber = pinNumber;
            tx_timer_create(
                &ptr->debounceTimer,
                (char *)"GPIO debounce timer",
                DebounceTimerCallback,
                0,
                0,
                1,
                TX_NO_ACTIVATE);

            gpioInputList.LinkAtBack(ptr);
        }
    }
    return ptr;
}
void UnlinkInputState(gpio_input_state *pState)
{
    tx_timer_delete(&pState->debounceTimer);
    // disable the EXT interrupt channel
    // it's OK to do always this, no matter if it's enabled or not
    GPIO_TypeDef *GPIO_port = GPIO_PORT(pState->pinNumber);
    uint32_t PinMask = GPIO_PIN(pState->pinNumber);

    ///////////////////////palDisablePadEventI(GPIO_port, PinMask);

    pState->Unlink();
    platform_free(pState);
}
// Delete gpio_input_state from List and tidy up ( Timer & ISR handler )
void DeleteInputState(GPIO_PIN pinNumber)
{
    gpio_input_state *pState = GetInputState(pinNumber);

    if (pState)
    {
        UnlinkInputState(pState);
    }
}
bool CPU_GPIO_Initialize()
{
    // Initialise Double linked list for input pin states
    gpioInputList.Initialize();
    // Make sure all pins are not reserved
    memset(pinReserved, 0, sizeof(pinReserved));
    return true;
}
bool CPU_GPIO_Uninitialize()
{
    NANOCLR_FOREACH_NODE(gpio_input_state, pGpio, gpioInputList)
    {
        UnlinkInputState(pGpio);
    }
    NANOCLR_FOREACH_NODE_END();

    return true;
}
// Set/reset reserved state of pin
bool CPU_GPIO_ReservePin(GPIO_PIN pinNumber, bool fReserve)
{
    // Check if valid pin number
    if (!IsValidGpioPin(pinNumber))
    {
        return false;
    }

    int port = pinNumber >> 4, bit = 1 << (pinNumber & 0x0F);
    bool ret = true;
    GLOBAL_LOCK();

    if (fReserve)
    {
        if (pinReserved[port] & bit)
        {
            ret = false; // already reserved
        }
        else
        {
            pinReserved[port] |= bit;
        }
    }
    else
    {
        pinReserved[port] &= ~bit;
    }

    GLOBAL_UNLOCK();
    return ret;
}
// Return if Pin is reserved
bool CPU_GPIO_PinIsBusy(GPIO_PIN pinNumber)
{
    // Check if valid pin number
    if (!IsValidGpioPin(pinNumber))
    {
        return false;
    }

    int port = pinNumber >> 4, sh = pinNumber & 0x0F;
    return (pinReserved[port] >> sh) & 1;
}
// Return maximum number of pins
int32_t CPU_GPIO_GetPinCount()
{
    return GPIO_MAX_PIN;
}
// Get current state of pin
GpioPinValue CPU_GPIO_GetPinState(GPIO_PIN pin)
{
    GPIO_TypeDef *GPIO_port = GPIO_PORT(pin);
    uint32_t PinMask = GPIO_PIN(pin);
    return (GpioPinValue)LL_GPIO_IsInputPinSet(GPIO_port, PinMask);
}
// Set Pin state
void CPU_GPIO_SetPinState(GPIO_PIN pin, GpioPinValue pinState)
{
    GPIO_TypeDef *GPIO_port = GPIO_PORT(pin);
    uint32_t PinMask = GPIO_PIN(pin);
    if (pinState == GpioPinValue::GpioPinValue_High)
    {
        LL_GPIO_SetOutputPin(GPIO_port, PinMask);
    }
    else
    {
        LL_GPIO_ResetOutputPin(GPIO_port, PinMask);
    }
}
void CPU_GPIO_TogglePinState(GPIO_PIN pin)
{
    GPIO_TypeDef *GPIO_port = GPIO_PORT(pin);
    uint32_t PinMask = GPIO_PIN(pin);
    LL_GPIO_TogglePin(GPIO_port, PinMask);
}
bool CPU_GPIO_EnableInputPin(
    GPIO_PIN pinNumber,
    uint32_t debounceTimeMilliseconds,
    GPIO_INTERRUPT_SERVICE_ROUTINE pinISR,
    void *isrParam,
    GPIO_INT_EDGE intEdge,
    PinMode driveMode)
{
    gpio_input_state *pState;
    // Check Input drive mode
    if (driveMode >= (int)PinMode_Output)
    {
        return false;
    }
    // Set as Input GPIO_INT_EDGE intEdge, GPIO_RESISTOR ResistorState
    if (!CPU_GPIO_SetDriveMode(pinNumber, driveMode))
    {
        return false;
    }
    pState = AllocateGpioInputState(pinNumber);
    // Link ISR ptr supplied and not already set up
    // CPU_GPIO_EnableInputPin could be called a 2nd time with changed parameters
    if (pinISR != NULL && (pState->isrPtr == NULL))
    {
        // there are callbacks registered and...
        // the drive mode is input so need to setup the interrupt

        GPIO_TypeDef *GPIO_port = GPIO_PORT(pState->pinNumber);
        uint32_t PinMask = GPIO_PIN(pState->pinNumber);

        ////////////////////pal_lld_enablelineevent(ioLine, PAL_EVENT_MODE_BOTH_EDGES);

        NVIC_EnableIRQ((IRQn_Type)(InterruptPin));
        NVIC_DisableIRQ((IRQn_Type)(InterruptPin));

        /////////////////////////////palSetLineCallbackI(ioLine, GpioEventCallback, pState);

        // store parameters & configs
        pState->isrPtr = pinISR;
        pState->mode = intEdge;
        pState->param = (void *)isrParam;
        pState->debounceMs = (uint32_t)(debounceTimeMilliseconds);

        switch (intEdge)
        {
            case GPIO_INT_EDGE_LOW:
            case GPIO_INT_LEVEL_LOW:
                pState->expected = PAL_LOW;
                break;

            case GPIO_INT_EDGE_HIGH:
            case GPIO_INT_LEVEL_HIGH:
                pState->expected = PAL_HIGH;
                break;

            case GPIO_INT_EDGE_BOTH:
                pState->expected = !CPU_GPIO_GetPinState(pinNumber); // expected NOT current state
                break;

            default:
                break;
        }
    }
    else if (pinISR == NULL && (pState->isrPtr != NULL))
    {
        // there is no managed handler setup anymore

        // disable the EXT interrupt channel
        // it's OK to do always this, no matter if it's enabled or not

        GPIO_TypeDef *GPIO_port = GPIO_PORT(pState->pinNumber);
        uint32_t PinMask = GPIO_PIN(pState->pinNumber);

        ////////////////////////pal_lld_disablelineevent(GetIoLine(pState->pinNumber));

        // clear parameters & configs
        pState->isrPtr = NULL;
        pState->mode = GPIO_INT_NONE;
        pState->param = NULL;
        pState->debounceMs = 0;
    }

    return true;
}

// Enable an output pin
//
// pinNumber    -   Gpio pin number
// InitialState -   Initial state of pin
// driveMode    -   Drive mode and resistors
// return       -   True if succesful, false invalid pin, pin not putput,
// invalid drive mode for ouptput
//
bool CPU_GPIO_EnableOutputPin(GPIO_PIN pinNumber, GpioPinValue InitialState, PinMode driveMode)
{
    // check not an output drive mode
    if (driveMode < (int)PinMode_Output)
    {
        return false;
    }
    // If this is currently an input pin then clean up
    DeleteInputState(pinNumber);
    if (CPU_GPIO_SetDriveMode(pinNumber, driveMode) == false)
    {
        return false;
    }
    CPU_GPIO_SetPinState(pinNumber, InitialState);
    return true;
}
void CPU_GPIO_DisablePin(GPIO_PIN pinNumber, PinMode driveMode, uint32_t alternateFunction)
{
    DeleteInputState(pinNumber);
    GLOBAL_LOCK();
    CPU_GPIO_SetDriveMode(pinNumber, driveMode);
    GPIO_TypeDef *GPIO_port = GPIO_PORT(pinNumber);
    uint32_t PinMask = GPIO_PIN(pinNumber);
    ///////////////////////pal_lld_setgroupmode(GPIO_port, PAL_PORT_BIT(PinMask), 0U,
    /// PAL_MODE_ALTERNATE(alternateFunction));
    GLOBAL_UNLOCK();
    CPU_GPIO_ReservePin(pinNumber, false);
}

// Validate pin and set drive mode
// return true if ok
bool CPU_GPIO_SetDriveMode(GPIO_PIN pinNumber, PinMode driveMode)
{
    GPIO_TypeDef *GPIO_port = GPIO_PORT(pinNumber);
    uint32_t PinMask = GPIO_PIN(pinNumber);

    // Note:
    // TODO : How to use the output speed information?
    // INPUT:
    //        Data present on the I/O pin is sampled into the Input Data
    //        Register every APB2 clock cycle
    // OUTPUT:
    //        Configure the pin speed mode by programming the respective bits
    //        in the configuration registers.
    //        THIS ALSO DEPENDS ON THE VOLTAGE LEVEL RUNNING
    //        ==============================================
    //        LL_GPIO_SPEED_FREQ_LOW       (0x00000000U)          /*!< Select I/O low output speed    */
    //        LL_GPIO_SPEED_FREQ_MEDIUM    GPIO_OSPEEDR_OSPEED0_0 /*!< Select I/O medium output speed */
    //        LL_GPIO_SPEED_FREQ_HIGH      GPIO_OSPEEDR_OSPEED0_1 /*!< Select I/O fast output speed   */
    //        LL_GPIO_SPEED_FREQ_VERY_HIGH GPIO_OSPEEDR_OSPEED0   /*!< Select I/O high output speed   */

    switch (driveMode)
    {
        case PinMode_Input:
            LL_GPIO_SetPinOutputType(GPIO_port, PinMask, LL_GPIO_MODE_INPUT);
            LL_GPIO_SetPinPull(GPIO_port, PinMask, LL_GPIO_PULL_NO);
            break;

        case PinMode_InputPullDown:
            LL_GPIO_SetPinOutputType(GPIO_port, PinMask, LL_GPIO_MODE_INPUT);
            LL_GPIO_SetPinPull(GPIO_port, PinMask, LL_GPIO_PULL_DOWN);
            break;

        case PinMode_InputPullUp:
            LL_GPIO_SetPinOutputType(GPIO_port, PinMask, LL_GPIO_MODE_INPUT);
            LL_GPIO_SetPinPull(GPIO_port, PinMask, LL_GPIO_PULL_UP);
            break;

        case PinMode_Output:
            LL_GPIO_SetPinOutputType(GPIO_port, PinMask, LL_GPIO_OUTPUT_PUSHPULL);
            break;

        case PinMode_OutputOpenDrain:
            LL_GPIO_SetPinOutputType(GPIO_port, PinMask, LL_GPIO_OUTPUT_OPENDRAIN);
            break;

        default:
            // all other modes are NOT supported
            return false;
    }

    return true;
}

bool CPU_GPIO_DriveModeSupported(GPIO_PIN pinNumber, PinMode driveMode)
{
    (void)pinNumber;
    return true;
}

uint32_t CPU_GPIO_GetPinDebounce(GPIO_PIN pinNumber)
{
    gpio_input_state *ptr = GetInputState(pinNumber);

    if (ptr)
    {
        return ptr->debounceMs;
    }

    return 0;
}
bool CPU_GPIO_SetPinDebounce(GPIO_PIN pinNumber, uint32_t debounceTimeMilliseconds)
{
    gpio_input_state *ptr = GetInputState(pinNumber);
    if (ptr)
    {
        ptr->debounceMs = debounceTimeMilliseconds;
        return true;
    }
    return false;
}
