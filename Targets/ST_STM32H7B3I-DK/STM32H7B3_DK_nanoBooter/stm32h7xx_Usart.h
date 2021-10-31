#pragma once

#include "stm32h7b3xxq.h"


#define CHECK_BIT(REGISTER,BIT_POSITION)  (REGISTER & BIT_POSITION)

#define USART_RESET(USART)  USART->CR1 = 0; USART->CR2 = 0; USART->CR3 = 0

/*
Bit 0 UE: USART enable
    When this bit is cleared, the USART prescalers and outputs are stopped immediately, and all 
    current operations are discarded.The USART configuration is kept, but all the USART_ISR 
    status flags are reset.This bit is set and cleared by software.
    0 : USART prescaler and outputs disabled, low - power mode
    1 : USART enabled
*/    
#define USART_DISABLE(USART)  CLEAR_BIT(USART->CR1, USART_CR1_UE_Pos)
#define USART_ENABLE(USART)   SET_BIT  (USART->CR1, USART_CR1_UE_Pos)

/*
Bit 1 UESM: USART enable in low-power mode 
    When this bit is cleared, the USART cannot wake up the MCU from low-power mode.
    When this bit is set, the USART can wake up the MCU from low-power mode.
    This bit is set and cleared by software.
    0: USART not able to wake up the MCU from low-power mode.
    1: USART able to wake up the MCU from low-power mode
*/
#define USART_LOW_POWER_DISABLE(USART)  CLEAR_BIT(USART->CR1, USART_CR1_UESM_Pos)
#define USART_LOW_POWER_ENABLE(USART)   SET_BIT  (USART->CR1, USART_CR1_UESM_Pos)
 
/*
Bit 2 RE: Receiver enable
    This bit enables the receiver. It is set and cleared by software.
    0: Receiver is disabled
    1: Receiver is enabled and begins searching for a start bit
 */
#define USART_RECEIVER_DISABLE(USART)  CLEAR_BIT(USART->CR1, USART_CR1_RE_Pos)    
#define USART_RECEIVER_ENABLE(USART)   SET_BIT  (USART->CR1, USART_CR1_RE_Pos)    

/*
Bit 3 TE: Transmitter enable
    This bit enables the transmitter. It is set and cleared by software.
    0: Transmitter is disabled
    1: Transmitter is enabled
 */
#define USART_TRANSMITTER_DISABLE(USART)  CLEAR_BIT(USART->CR1, USART_CR1_TE_Pos) 
#define USART_TRANSMITTER_ENABLE(USART)   SET_BIT  (USART->CR1, USART_CR1_TE_Pos)

/*
Bit 4 IDLEIE: IDLE interrupt enable
    This bit is set and cleared by software.
    0: Interrupt inhibited
    1: USART interrupt generated whenever IDLE = 1 in the USART_ISR register
 */
#define USART_IDLE_INTERRUPT_DISABLE(USART)  CLEAR_BIT(USART->CR1, USART_CR1_IDLEIE_Pos)
#define USART_IDLE_INTERRUPT_ENABLE(USART)   SET_BIT  (USART->CR1, USART_CR1_IDLEIE_Pos)

/*
Bit 5 RXFNEIE: RXFIFO not empty interrupt enable
    This bit is set and cleared by software.
        0: Interrupt inhibited
        1: USART interrupt generated whenever ORE = 1 or RXFNE = 1 in the USART_ISR register
 */
#define USART_DISABLE_RXNE_RX_FIFO_INTERRUPT_DISABLE(USART) CLEAR_BIT(USART->CR1, USART_CR1_RXNEIE_RXFNEIE_Pos) 
#define USART_ENABLE_RXNE_RX_FIFO_INTERRUPT_ENABLE(USART)   SET_BIT  (USART->CR1, USART_CR1_RXNEIE_RXFNEIE_Pos)
/*
Bit 6 TCIE: Transmission complete interrupt enable
    This bit is set and cleared by software.
        0: Interrupt inhibited
        1: USART interrupt generated whenever TC = 1 in the USART_ISR register
*/
#define USART_TX_COMPLETE_INTERRUPT_DISABLE(USART)  CLEAR_BIT(USART->CR1, USART_CR1_TCIE_Pos) 
#define USART_TX_COMPLETE_INTERRUPT_ENABLE(USART)   SET_BIT  (USART->CR1, USART_CR1_TCIE_Pos)
/*
Bit 7 TXFNFIE: TXFIFO not full interrupt enable
    This bit is set and cleared by software.
        0: Interrupt inhibited
        1: USART interrupt generated whenever TXFNF =1 in the USART_ISR register
*/                                                                
#define USART_TX_TXE_FIFO_COMPLETE_INTERRUPT_DISABLE(USART)  CLEAR_BIT(USART->CR1, USART_CR1_TXEIE_TXFNFIE_Pos) 
#define USART_TX_TXE_FIFO_COMPLETE_INTERRUPT_ENABLE(USART)   SET_BIT  (USART->CR1, USART_CR1_TXEIE_TXFNFIE_Pos)
/*
Bit 8 PEIE: PE interrupt enable
    This bit is set and cleared by software.
        0: Interrupt inhibited
        1: USART interrupt generated whenever PE = 1 in the USART_ISR register
*/
#define USART_PARITY_INTERRUPT_DISABLE(USART)  CLEAR_BIT(USART->CR1, USART_CR1_PEIE_Pos) 
#define USART_PARITY_INTERRUPT_ENABLE(USART)   SET_BIT  (USART->CR1, USART_CR1_PEIE_Pos)
      
/*
Bit 9 PS: Parity selection
    This bit selects the odd or even parity when the parity generation/detection is enabled (PCE 
    bit set). It is set and cleared by software. The parity is selected after the current byte.
        0: Even parity
        1: Odd parity
    This bitfield can only be written when the USART is disabled (UE = 0).*/

#define USART_PARITY_EVEN(USART) CLEAR_BIT(USART->CR1, USART_CR1_PS_Pos)
#define USART_PARITY_ODD(USART)  SET_BIT  (USART->CR1, USART_CR1_PS_Pos)   
/*
Bit 10 PCE: Parity control enable
    This bit selects the hardware parity control (generation and detection). When the parity 
    control is enabled, the computed parity is inserted at the MSB position (9th bit if M = 1; 8th bit 
    if M = 0) and the parity is checked on the received data. This bit is set and cleared by 
    software. Once it is set, PCE is active after the current byte (in reception and in 
    transmission).
        0: Parity control disabled
        1: Parity control enabled
    This bitfield can only be written when the USART is disabled (UE = 0).
*/                                                                
#define USART_PARITY_NONE(USART)     CLEAR_BIT(USART->CR1, USART_CR1_PCE_Pos)
#define USART_PARITY_ENABLED(USART)  SET_BIT  (USART->CR1, USART_CR1_PCE_Pos)  
/*
Bit 11 WAKE : Receiver wakeup method
    This bit determines the USART wakeup method from Mute mode.It is set or cleared by 
    software.
        0 : Idle line
        1 : Address mark
    This bitfield can only be written when the USART is disabled(UE = 0).
*/                                                                
#define USART_RECEIVER_WAKE_IDLE(USART)     CLEAR_BIT(USART->CR1, USART_CR1_WAKE_Pos)
#define USART_RECEIVER_WAKE_ADDRESS(USART)  SET_BIT  (USART->CR1, USART_CR1_WAKE_Pos)
/*
Bit 12 M0: Word length
Bit 28 M1: Word length
    M[1:0] = ‘10’: 1 start bit, 7 Data bits, n Stop bit
    M[1:0] = ‘00’: 1 start bit, 8 Data bits, n Stop bit
    M[1:0] = ‘01’: 1 start bit, 9 Data bits, n Stop bit
        This bit can only be written when the USART is disabled (UE = 0)
*/
#define USART_7_BIT_WORD_LENGTH(USART)   SET_BIT  (USART->CR1, USART_CR1_M1_Pos); CLEAR_BIT(USART->CR1, USART_CR1_M_Pos);
#define USART_8_BIT_WORD_LENGTH(USART)   CLEAR_BIT(USART->CR1, USART_CR1_M1_Pos); CLEAR_BIT(USART->CR1, USART_CR1_M_Pos);
#define USART_9_BIT_WORD_LENGTH(USART)   CLEAR_BIT(USART->CR1, USART_CR1_M1_Pos); SET_BIT  (USART->CR1, USART_CR1_M_Pos);
/*
Bit 13 MME : Mute mode enable
    This bit enables the USART Mute mode function.When set, the USART can switch between 
    active and Mute mode, as defined by the WAKE bit.It is set and cleared by software.
        0 : Receiver in active mode permanently
        1 : Receiver can switch between Mute mode and active mode
*/
#define USART_MUTE_MODE_DISABLE(USART)  CLEAR_BIT(USART->CR1, USART_CR1_MME_Pos);        
#define USART_MUTE_MODE_ENABLE(USART)   SET_BIT  (USART->CR1, USART_CR1_MME_Pos);
/*
Bit 14 CMIE: Character match interrupt enable
    This bit is set and cleared by software.
        0: Interrupt inhibited
        1: USART interrupt generated when the CMF bit is set in the USART_ISR register.
*/                                                                                                                   
#define USART_MATCH_DISABLE(USART)  CLEAR_BIT(USART->CR1, USART_CR1_CMIE_Pos);       
#define USART_MATCH_ENABLE(USART)   SET_BIT  (USART->CR1, USART_CR1_CMIE_Pos);
/*
Bit 15 OVER8: Oversampling mode
    0: Oversampling by 16
    1: Oversampling by 8
This bit can only be written when the USART is disabled (UE = 0).
Note: In LIN, IrDA and Smartcard modes, this bit must be kept cleared. 
*/                                                                                                                   
#define USART_OVERSAMPLING_16(USART) CLEAR_BIT(USART->CR1, USART_CR1_OVER8_Pos);      
#define USART_OVERSAMPLING_8(USART)  SET_BIT  (USART->CR1, USART_CR1_OVER8_Pos);
 
/*
Bits 20 : 16 DEDT[4 : 0] : Driver Enable deassertion time
    This 5 - bit value defines the time between the end of the last stop bit, in a transmitted 
    message, and the de - activation of the DE(Driver Enable) signal.It is expressed in sample 
    time units(1 / 8 or 1 / 16 bit time, depending on the oversampling rate).
    If the USART_TDR register is written during the DEDT time, the new data is transmitted only 
    when the DEDT and DEAT times have both elapsed.
    This bitfield can only be written when the USART is disabled(UE = 0).
        Note : If the Driver Enable feature is not supported, this bit is reserved and
*/    
    
/* ------------DEDT[4 : 0] = ?*/

/*
Bit 26 RTOIE: Receiver timeout interrupt enable
    This bit is set and cleared by software.
        0: Interrupt inhibited
        1: USART interrupt generated when the RTOF bit is set in the USART_ISR register.
            Note: If the USART does not support the Receiver timeout feature, this bit is reserved and 
            must be kept at reset value.
*/
#define USART_RECEIVER_TIMEOUT_INT_DISABLE(USART) CLEAR_BIT(USART->CR1, USART_CR1_RTOIE_Pos);      
#define USART_RECEIVER_TIMEOUT_INT_ENABLE(USART)  SET_BIT  (USART->CR1, USART_CR1_RTOIE_Pos);
/*
 *Bit 27 EOBIE: End of Block interrupt enable
    This bit is set and cleared by software.
        0: Interrupt inhibited
        1: USART interrupt generated when the EOBF flag is set in the USART_ISR register
            Note: If the USART does not support Smartcard mode, this bit is reserved and must be kept at 
            reset value*/                                                                                                                   
#define USART_END_OF_BLOCK_INT_ENABLE(USART)  SET_BIT  (USART->CR1, USART_CR1_EOBIE_Pos);
#define USART_END_OF_BLOCK_INT_DISABLE(USART) CLEAR_BIT(USART->CR1, USART_CR1_EOBIE_Pos);      
/*
Bit 29 FIFOEN: FIFO mode enable
    This bit is set and cleared by software.
        0: FIFO mode is disabled.
        1: FIFO mode is enabled.
            This bitfield can only be written when the USART is disabled (UE = 0).
            Note: FIFO mode can be used on standard UART communication, in SPI master/slave mode 
            and in Smartcard modes only. It must not be enabled in IrDA and LIN modes.
*/                                                                                                                  
#define USART_FIFO_DISABLE(USART)  CLEAR_BIT(USART->CR1, USART_CR1_FIFOEN_Pos);     
#define USART_FIFO_ENABLE(USART)   SET_BIT  (USART->CR1, USART_CR1_FIFOEN_Pos);
/*
Bit 30 TXFEIE: TXFIFO empty interrupt enable
    This bit is set and cleared by software.
        0: Interrupt inhibited
        1: USART interrupt generated when TXFE = 1 in the USART_ISR register
*/                                                                                                                   
#define USART_TXFIFO_ENABLE(USART)  SET_BIT  (USART->CR1, USART_CR1_TXFEIE_Pos);
#define USART_TXFIFO_DISABLE(USART) CLEAR_BIT(USART->CR1, USART_CR1_TXFEIE_Pos);     
/*
Bit 31 RXFFIE: RXFIFO Full interrupt enable
    This bit is set and cleared by software.
        0: Interrupt inhibited
        1: USART interrupt generated when RXFF = 1 in the USART_ISR register
*/                                                                                                                   
#define USART_RXFIFO_DISABLE(USART) CLEAR_BIT(USART->CR1, USART_CR1_RXFFIE_Pos);     
#define USART_RXFIFO_ENABLE(USART)  SET_BIT  (USART->CR1, USART_CR1_RXFFIE_Pos);
/*
Bit 0 SLVEN: Synchronous Slave mode enable
    When the SLVEN bit is set, the synchronous slave mode is enabled.
        0: Slave mode disabled.
        1: Slave mode enabled.
*/                                                                                                                   
#define USART_SYNCH_SLAVE_DISABLE(USART) CLEAR_BIT(USART->CR2, USART_CR2_SLVEN_Pos);      
#define USART_SYNCH_SLAVE_ENABLE(USART)  SET_BIT  (USART->CR2, USART_CR2_SLVEN_Pos);
/*
Bit 3 DIS_NSS:
    When the DIS_NSS bit is set, the NSS pin input is ignored.
        0: SPI slave selection depends on NSS input pin.
        1: SPI slave is always selected and NSS input pin is ignored      
*/                                                                                                                   
#define USART_NSS_SPI_SLAVE_SELECTION(USART) SET_BIT  (USART->CR2, USART_CR2_DIS_NSS_Pos);
#define USART_NSS_SPI_SLAVE_ALWAYS(USART)    CLEAR_BIT(USART->CR2, USART_CR2_DIS_NSS_Pos);    
/*
Bit 4 ADDM7: 7-bit Address Detection/4-bit Address Detection
    This bit is for selection between 4-bit address detection or 7-bit address detection. 
        0: 4-bit address detection
        1: 7-bit address detection (in 8-bit data mode)
            This bit can only be written when the USART is disabled (UE = 0)
            Note: In 7-bit and 9-bit data modes, the address detection is done on 6-bit and 8-bit address 
            (ADD[5:0] and ADD[7:0]) respectively
*/                                                                                                                   
#define USART_ADDRESS_DETECT_4BIT(USART)  SET_BIT  (USART->CR2, USART_CR2_ADDM7_Pos);
#define USART_ADDRESS_DETECT_7BIT(USART)  CLEAR_BIT(USART->CR2, USART_CR2_ADDM7_Pos);      
/*
Bit 5 LBDL: LIN break detection length
    This bit is for selection between 11 bit or 10 bit break detection.
        0: 10-bit break detection
        1: 11-bit break detection
            This bit can only be written when the USART is disabled (UE = 0).
*/                                                                                                                   
#define LIN_BREAK_DETECT_10BIT(USART)  CLEAR_BIT(USART->CR2, USART_CR2_LBDL_Pos);       
#define LIN_BREAK_DETECT_11BIT(USART)  SET_BIT  (USART->CR2, USART_CR2_LBDL_Pos);
/*
Bit 6 LBDIE: LIN break detection interrupt enable
    Break interrupt mask (break detection using break delimiter).
        0: Interrupt is inhibited
        1: An interrupt is generated whenever LBDF = 1 in the USART_ISR register
*/                                                                                                                   
#define LIN_BREAK_DETECT_INTERRUPT_DISABLE(USART)  CLEAR_BIT(USART->CR2, USART_CR2_LBDIE_Pos);      
#define LIN_BREAK_DETECT_INTERRUPT_ENABLE(USART)   SET_BIT  (USART->CR2, USART_CR2_LBDIE_Pos);
/*
Bit 7 Reserved, must be kept at reset value
*/
/* ---------  */
/*
Bit 8 LBCL: Last bit clock pulse
    This bit is used to select whether the clock pulse associated with the last data bit transmitted (MSB) 
    has to be output on the SCLK pin in synchronous mode. 
        0: The clock pulse of the last data bit is not output to the SCLK pin
        1: The clock pulse of the last data bit is output to the SCLK pin
            Caution: The last bit is the 7th or 8th or 9th data bit transmitted depending on the 7 or 8 or 9 bit 
            format selected by the M bit in the USART_CR1 register.
            This bit can only be written when the USART is disabled (UE = 0)
 */
#define USART_LAST_BIT_CLOCK_PULSE_EXCLUDE(USART)  CLEAR_BIT(USART->CR2, USART_CR2_LBCL_Pos);       
#define USART_LAST_BIT_CLOCK_PULSE_PRESENT(USART)  SET_BIT  (USART->CR2, USART_CR2_LBCL_Pos);
/*
Bit 9 CPHA: Clock phase
    This bit is used to select the phase of the clock output on the SCLK pin in synchronous mode. It 
    works in conjunction with the CPOL bit to produce the desired clock/data relationship (see 
    Figure 544 and Figure 545)
        0: The first clock transition is the first data capture edge
        1: The second clock transition is the first data capture edge
        This bit can only be written when the USART is disabled (UE = 0).
*/                                                                                         
#define USART_CLOCK_PHASE_FIRST(USART)   SET_BIT  (USART->CR2, USART_CR2_LBCL_Pos);
#define USART_CLOCK_PHASE_SECOND(USART)  CLEAR_BIT(USART->CR2, USART_CR2_CPHA_Pos);       
/*
Bit 10 CPOL: Clock polarity
    This bit enables the user to select the polarity of the clock output on the SCLK pin in synchronous 
    mode. It works in conjunction with the CPHA bit to produce the desired clock/data relationship 
        0: Steady low value on SCLK pin outside transmission window
        1: Steady high value on SCLK pin outside transmission window
        This bit can only be written when the USART is disabled (UE = 0)
*/
#define USART_CLOCK_POLARITY_LOW(USART)  CLEAR_BIT(USART->CR2, USART_CR2_CPOL_Pos);
#define USART_CLOCK_POLARITY_HIGH(USART) SET_BIT  (USART->CR2, USART_CR2_CPOL_Pos);
/*
Bit 11 CLKEN: Clock enable
    This bit enables the user to enable the SCLK pin.
        0: SCLK pin disabled
        1: SCLK pin enabled
        This bit can only be written when the USART is disabled (UE = 0).
        Note: If neither synchronous mode nor Smartcard mode is supported, this bit is reserved and must 
        be kept at reset value. Refer to Section 53.4: USART implementation on page 1958.
        In Smartcard mode, in order to provide correctly the SCLK clock to the smartcard, the steps 
        below must be respected:
            UE = 0
            SCEN = 1
            GTPR configuration
            CLKEN= 1
            UE = 1
*/                                                                
#define USART_SCLK_DISABLE(USART) CLEAR_BIT(USART->CR2, USART_CR2_CLKEN_Pos);  
#define USART_SCLK_ENABLE(USART)  SET_BIT  (USART->CR2, USART_CR2_CLKEN_Pos);
/*
Bits 13:12 STOP[1:0]: stop bits
    These bits are used for programming the stop bits.
    00: 1 stop bit
    01: 0.5 stop bit.
    10: 2 stop bits
    11: 1.5 stop bits
        This bitfield can only be written when the USART is disabled (UE = 0)
*/
#define USART_STOP_1_BITS(USART)    CLEAR_BIT(USART->CR2, USART_CR2_STOP_Pos+1);CLEAR_BIT(USART->CR2, USART_CR2_STOP_Pos);      
#define USART_STOP_0_5_BITS(USART)  CLEAR_BIT(USART->CR2, USART_CR2_STOP_Pos+1);SET_BIT  (USART->CR2, USART_CR2_STOP_Pos);      
#define USART_STOP_2_BITS(USART)    SET_BIT  (USART->CR2, USART_CR2_STOP_Pos+1);CLEAR_BIT(USART->CR2, USART_CR2_STOP_Pos);     
#define USART_STOP_1_5_BITS(USART)  SET_BIT  (USART->CR2, USART_CR2_STOP_Pos+1);SET_BIT  (USART->CR2, USART_CR2_STOP_Pos);     
/*
Bit 14 LINEN: LIN mode enable
    This bit is set and cleared by software.
        0: LIN mode disabled
        1: LIN mode enabled
        The LIN mode enables the capability to send LIN synchronous breaks (13 low bits) using the 
        SBKRQ bit in the USART_CR1 register, and to detect LIN Sync breaks.
        This bitfield can only be written when the USART is disabled (UE = 0).
*/
#define LIN_MODE_DISABLE(USART)  CLEAR_BIT(USART->CR2, USART_CR2_LINEN_Pos);      
#define LIN_MODE_ENABLE(USART)   SET_BIT  (USART->CR2, USART_CR2_LINEN_Pos);
/*
Bit 15 SWAP: Swap TX/RX pins
This bit is set and cleared by software.
    0: TX/RX pins are used as defined in standard pinout
    1: The TX and RX pins functions are swapped. This enables to work in the case of a cross-wired 
    connection to another UART. 
    This bitfield can only be written when the USART is disabled (UE = 0).
*/                                                                                                                  
#define USART_TX_PIN_STANDARD(USART) CLEAR_BIT(USART->CR2, USART_CR2_SWAP_Pos);      
#define USART_TX_RX_PIN_SWAP(USART)  SET_BIT  (USART->CR2, USART_CR2_SWAP_Pos);
/*
Bit 16 RXINV: RX pin active level inversion
    This bit is set and cleared by software.
        0: RX pin signal works using the standard logic levels (VDD =1/idle, Gnd = 0/mark) 
        1: RX pin signal values are inverted (VDD =0/mark, Gnd = 1/idle). 
        This enables the use of an external inverter on the RX line. 
        This bitfield can only be written when the USART is disabled (UE = 0)     
*/                                                                                                                  
#define USART_RX_ACTIVE_LEVEL_STANDARD(USART)  CLEAR_BIT(USART->CR2, USART_CR2_RXINV_Pos);     
#define USART_RX_ACTIVE_LEVEL_INVERTED(USART)  SET_BIT  (USART->CR2, USART_CR2_RXINV_Pos);
/*
Bit 17 TXINV: TX pin active level inversion
    This bit is set and cleared by software.
        0: TX pin signal works using the standard logic levels (VDD =1/idle, Gnd = 0/mark) 
        1: TX pin signal values are inverted (VDD =0/mark, Gnd = 1/idle). 
        This enables the use of an external inverter on the TX line. 
        This bitfield can only be written when the USART is disabled (UE = 0)
*/                                                                                                                  
#define USART_TX_ACTIVE_LEVEL_NORMAL(USART)    CLEAR_BIT(USART->CR2, USART_CR2_TXINV_Pos);     
#define USART_TX_ACTIVE_LEVEL_INVERTED(USART)  SET_BIT  (USART->CR2, USART_CR2_TXINV_Pos);
/*
Bit 18 DATAINV: Binary data inversion
    This bit is set and cleared by software.
        0: Logical data from the data register are send/received in positive/direct logic. (1 = H, 0 = L) 
        1: Logical data from the data register are send/received in negative/inverse logic. (1 = L, 0 = H). 
        The parity bit is also inverted.
        This bitfield can only be written when the USART is disabled (UE = 0).
*/                                                                                                                  
#define USART_BINARY_DATA_NORMAL(USART)    CLEAR_BIT(USART->CR2, USART_CR2_DATAINV_Pos);     
#define USART_BINARY_DATA_INVERTED(USART)  SET_BIT  (USART->CR2, USART_CR2_DATAINV_Pos);
/*
Bit 19 MSBFIRST: Most significant bit first
    This bit is set and cleared by software.
    0: data is transmitted/received with data bit 0 first, following the start bit. 
    1: data is transmitted/received with the MSB (bit 7/8) first, following the start bit. 
    This bitfield can only be written when the USART is disabled (UE = 0)
*/                                                                                                 
#define USART_MSB_BIT_FIRST(USART)  CLEAR_BIT(USART->CR2, USART_CR2_MSBFIRST_Pos);     
#define USART_LSB_BIT_FIRST(USART)  SET_BIT  (USART->CR2, USART_CR2_MSBFIRST_Pos);
/*
Bit 20 ABREN: Auto baud rate enable
    This bit is set and cleared by software.
        0: Auto baud rate detection is disabled. 
        1: Auto baud rate detection is enabled. 
        Note: If the USART does not support the auto baud rate feature, this bit is reserved and must be kept 
        at reset value. R
*/                                                                                                 
#define USART_AUTOBAUD_DISABLE(USART)  CLEAR_BIT(USART->CR2, USART_CR2_ABREN_Pos);    
#define USART_AUTOBAUD_ENABLE(USART)   SET_BIT  (USART->CR2, USART_CR2_ABREN_Pos);

/*
Bits 22:21 ABRMOD[1:0]: Auto baud rate mode
These bits are set and cleared by software.
    00: Measurement of the start bit is used to detect the baud rate. 
    01: Falling edge to falling edge measurement (the received frame must start with a single bit = 1 and Frame = Start10xxxxxx)
    10: 0x7F frame detection.
    11: 0x55 frame detection
        This bitfield can only be written when ABREN = 0 or the USART is disabled (UE = 0).
        Note: If DATAINV = 1 and/or MSBFIRST = 1 the patterns must be the same on the line, for example  0xAA for MSBFIRST)
        If the USART does not support the auto baud rate feature, this bit is reserved and must be kept at reset value. 
*/
#define USART_AUTOBAUD_MODE0(USART)  CLEAR_BIT(USART->CR2, USART_CR2_ABRMODE_Pos+1);CLEAR_BIT(USART->CR2, USART_CR2_ABRMODE_Pos);
#define USART_AUTOBAUD_MODE1(USART)  CLEAR_BIT(USART->CR2, USART_CR2_ABRMODE_Pos+1);SET_BIT  (USART->CR2, USART_CR2_ABRMODE_Pos);
#define USART_AUTOBAUD_MODE2(USART)  SET_BIT  (USART->CR2, USART_CR2_ABRMODE_Pos+1);CLEAR_BIT(USART->CR2, USART_CR2_ABRMODE_Pos);    
#define USART_AUTOBAUD_MODE3(USART)  SET_BIT  (USART->CR2, USART_CR2_ABRMODE_Pos+1);SET_BIT  (USART->CR2, USART_CR2_ABRMODE_Pos);    
/*
Bit 23 RTOEN: Receiver timeout enable
    This bit is set and cleared by software.
    0: Receiver timeout feature disabled. 
    1: Receiver timeout feature enabled. 
        When this feature is enabled, the RTOF flag in the USART_ISR register is set if the RX line is idle 
        (no reception) for the duration programmed in the RTOR (receiver timeout register).
        Note: If the USART does not support the Receiver timeout feature, this bit is reserved and must be kept at reset value
*/                                               
#define USART_RECEIVER_TIMEOUT_DISABLE(USART)  CLEAR_BIT(USART->CR2, USART_CR2_RTOEN_Pos);     
#define USART_RECEIVER_TIMEOUT_ENABLE(USART)   SET_BIT  (USART->CR2, USART_CR2_RTOEN_Pos);
/*
Bits 31:24 ADD[7:0]: Address of the USART node ADD[7:4]: 
    These bits give the address of the USART node or a character code to be recognized.
    They are used to wake up the MCU with 7-bit address mark detection in multiprocessor 
    communication during Mute mode or low-power mode. The MSB of the character sent by the 
    transmitter should be equal to 1. They can also be used for character detection during normal 
    reception, Mute mode inactive (for example, end of block detection in ModBus protocol). In this 
    case, the whole received character (8-bit) is compared to the ADD[7:0] value and CMF flag is set on 
    match.
    These bits can only be written when reception is disabled (RE = 0) or the USART is disabled (UE = 0).
    ADD[3:0]:
    These bits give the address of the USART node or a character code to be recognized.
    They are used for wakeup with address mark detection, in multiprocessor communication during 
    Mute mode or low-power mode.
    These bits can only be written when reception is disabled (RE = 0) or the USART is disabled (UE = 0).
*/

#define USART_NODE_ADDRESS  SET_BITS()

/*
Bit 0 EIE: Error interrupt enable
    Error Interrupt Enable Bit is required to enable interrupt generation in case of a framing 
    error, overrun error noise flag or SPI slave underrun error (FE = 1 or ORE = 1 or NE = 1 or UDR = 1 in the USART_ISR register).
    0: Interrupt inhibited
    1: interrupt generated when 
*/
#define USART_ERROR_INTERRUPT_DISABLE(USART) CLEAR_BIT(USART->CR3, USART_CR3_EIE_Pos);       
#define USART_ERROR_INTERRUPT_ENABLE(USART)  SET_BIT  (USART->CR3, USART_CR3_EIE_Pos);
/*
Bit 1 IREN: IrDA mode enable
    This bit is set and cleared by software.
    0: IrDA disabled
    1: IrDA enabled
        This bit can only be written when the USART is disabled (UE = 0).
        Note: If IrDA mode is not supported, this bit is reserved and must be kept at reset value. 
*/                                               
#define IRDA_MODE_DISABLE(USART)  CLEAR_BIT(USART->CR3, USART_CR3_IREN_Pos);      
#define IRDA_MODE_ENABLE(USART)  SET_BIT   (USART->CR3, USART_CR3_IREN_Pos);
/*
Bit 2 IRLP: IrDA low-power
    This bit is used for selecting between normal and low-power IrDA modes
    0: Normal mode
    1: Low-power mode
        This bit can only be written when the USART is disabled (UE = 0).
        Note: If IrDA mode is not supported, this bit is reserved and must be kept at reset value
*/                                               
#define IRDA_POWER_NORMAL(USART) CLEAR_BIT(USART->CR3, USART_CR3_IRLP_Pos);      
#define IRDA_POWER_LOW(USART)    SET_BIT  (USART->CR3, USART_CR3_IRLP_Pos);
/*
Bit 3 HDSEL: Half-duplex selection
    Selection of Single-wire Half-duplex mode 
    0: Half duplex mode is not selected
    1: Half duplex mode is selected 
        This bit can only be written when the USART is disabled (UE = 0).    
*/                                               
#define USART_NORMAL_DUPLEX(USART)  CLEAR_BIT(USART->CR3, USART_CR3_HDSEL_Pos);     
#define USART_HALF_DUPLEX(USART)    SET_BIT  (USART->CR3, USART_CR3_HDSEL_Pos);
/*
Bit 4 NACK: Smartcard NACK enable
    0: NACK transmission in case of parity error is disabled
    1: NACK transmission during parity error is enabled
        This bitfield can only be written when the USART is disabled (UE = 0).
        Note: If the USART does not support Smartcard mode, this bit is reserved and must be kept at reset value. 
*/                                               
#define SMARTCARD_NACK_PE_DISABLE(USART)  CLEAR_BIT(USART->CR3, USART_CR3_NACK_Pos);      
#define SMARTCARD_NACK_PE_ENABLE(USART)   SET_BIT  (USART->CR3, USART_CR3_NACK_Pos);
/*
Bit 5 SCEN: Smartcard mode enable
    This bit is used for enabling Smartcard mode.
    0: Smartcard Mode disabled
    1: Smartcard Mode enabled
        This bitfield can only be written when the USART is disabled (UE = 0).
        Note: If the USART does not support Smartcard mode, this bit is reserved and must be kept at reset value. 
*/                                               
#define SMARTCARD_MODE_DISABLE(USART)  CLEAR_BIT(USART->CR3, USART_CR3_SCEN_Pos);      
#define SMARTCARD_MODE_ENABLE(USART)   SET_BIT  (USART->CR3, USART_CR3_SCEN_Pos);
/*
Bit 6 DMAR: DMA enable receiver
    This bit is set/reset by software
    0: DMA mode is disabled for reception
    1: DMA mode is enabled for reception
*/                                               
#define USART_DMA_RECEIVER_DISABLE(USART)  CLEAR_BIT(USART->CR3, USART_CR3_DMAR_Pos);      
#define USART_DMA_RECEIVER_ENABLE(USART)   SET_BIT  (USART->CR3, USART_CR3_DMAR_Pos);
/*
Bit 7 DMAT: DMA enable transmitter
    This bit is set/reset by software
    1: DMA mode is enabled for transmission
    0: DMA mode is disabled for transmission
*/                                               
#define USART_DMA_TRANSMITTER_DISABLE(USART)  CLEAR_BIT(USART->CR3, USART_CR3_DMAT_Pos);      
#define USART_DMA_TRANSMITTER_ENABLE(USART)   SET_BIT  (USART->CR3, USART_CR3_DMAT_Pos);
/*
Bit 8 RTSE: RTS enable
    0: RTS hardware flow control disabled
    1: RTS output enabled, data is only requested when there is space in the receive buffer. The 
    transmission of data is expected to cease after the current character has been transmitted. 
    The nRTS output is asserted (pulled to 0) when data can be received.
    This bit can only be written when the USART is disabled (UE = 0).
    Note: If the hardware flow control feature is not supported, this bit is reserved and must be kept at reset value
*/
#define USART_RTS_DISABLE(USART) CLEAR_BIT(USART->CR3, USART_CR3_RTSE_Pos);      
#define USART_RTS_ENABLE(USART)  SET_BIT  (USART->CR3, USART_CR3_RTSE_Pos);
/*
Bit 9 CTSE: CTS enable
    0: CTS hardware flow control disabled
    1: CTS mode enabled, data is only transmitted when the nCTS input is asserted (tied to 0). 
    If the nCTS input is deasserted while data is being transmitted, then the transmission is 
    completed before stopping. If data is written into the data register while nCTS is asserted, 
    the transmission is postponed until nCTS is asserted.
    This bit can only be written when the USART is disabled (UE = 0)
    Note: If the hardware flow control feature is not supported, this bit is reserved and must be kept at reset value. 
*/                                               
#define USART_CTS_DISABLE(USART)   CLEAR_BIT(USART->CR3, USART_CR3_CTSE_Pos);      
#define USART_CTS_ENABLE(USART)    SET_BIT  (USART->CR3, USART_CR3_CTSE_Pos);
/*
Bit 10 CTSIE: CTS interrupt enable
    0: Interrupt is inhibited
    1: An interrupt is generated whenever CTSIF = 1 in the USART_ISR register
    Note: If the hardware flow control feature is not supported, this bit is reserved and must be kept at reset value. 
*/                                               
#define USART_CTS_INTERRUPT_DISABLE(USART)  CLEAR_BIT(USART->CR3, USART_CR3_CTSIE_Pos);     
#define USART_CTS_INTERRUPT_ENABLE(USART)   SET_BIT  (USART->CR3, USART_CR3_CTSIE_Pos);
/*
Bit 11 ONEBIT: One sample bit method enable
    This bit enables the user to select the sample method. When the one sample bit method is 
    selected the noise detection flag (NE) is disabled.
    0: Three sample bit method
    1: One sample bit method
        This bit can only be written when the USART is disabled (UE = 0).
*/                                               
#define USART_THREE_SAMPLE_BIT(USART)  CLEAR_BIT(USART->CR3, USART_CR3_ONEBIT_Pos);    
#define USART_ONE_SAMPLE_BIT(USART)    SET_BIT  (USART->CR3, USART_CR3_ONEBIT_Pos);
/*
Bit 12 OVRDIS: Overrun Disable
This bit is used to disable the receive overrun detection. 
    0: Overrun Error Flag, ORE, is set when received data is not read before receiving new data. 
    1: Overrun functionality is disabled. If new data is received while the RXNE flag is still set
    the ORE flag is not set and the new received data overwrites the previous content of the 
    USART_RDR register. When FIFO mode is enabled, the RXFIFO is bypassed and data is 
    written directly in USART_RDR register. Even when FIFO management is enabled, the RXNE flag is to be used.
        This bit can only be written when the USART is disabled (UE = 0).
        Note: This control bit enables checking the communication flow w/o reading the data
*/                                               
#define USART_OVERRUN_ERROR_FLAG_ENABLE(USART)   CLEAR_BIT(USART->CR3, USART_CR3_OVRDIS_Pos);    
#define USART_OVERRUN_ERROR_FLAG_DISABLE(USART)   SET_BIT (USART->CR3, USART_CR3_OVRDIS_Pos);
/*
Bit 13 DDRE: DMA Disable on Reception Error
    0: DMA is not disabled in case of reception error. The corresponding error flag is set but 
    RXNE is kept 0 preventing from overrun. As a consequence, the DMA request is not asserted, 
    so the erroneous data is not transferred (no DMA request), but next correct received data is transferred (used for Smartcard mode).
    1: DMA is disabled following a reception error. The corresponding error flag is set, as well as RXNE. 
    The DMA request is masked until the error flag is cleared. This means that the software must first disable 
    the DMA request (DMAR = 0) or clear RXNE/RXFNE is case FIFO mode is enabled) before clearing the error flag.
        This bit can only be written when the USART is disabled (UE=0).
        Note: The reception errors are: parity error, framing error or noise error
*/                                               
#define USART_DMA_RECEPTION_ERROR_INTERRUPT_ENABLE(USART)   CLEAR_BIT(USART->CR3, USART_CR3_DDRE_Pos);
#define USART_DMA_RECEPTION_ERROR_INTERRUPT_DISABLE(USART)  SET_BIT  (USART->CR3, USART_CR3_DDRE_Pos);      
/*
Bit 14 DEM: Driver enable mode 
    This bit enables the user to activate the external transceiver control, through the DE signal. 
    0: DE function is disabled. 
    1: DE function is enabled. The DE signal is output on the RTS pin.
        This bit can only be written when the USART is disabled (UE = 0).
        Note: If the Driver Enable feature is not supported, this bit is reserved and must be kept at reset value. 
*/                                                                                                 
#define USART_DE_SIGNAL_DISABLE(USART)  CLEAR_BIT(USART->CR3, USART_CR3_DEM_Pos);       
#define USART_DE_SIGNAL_ENABLE(USART)   SET_BIT  (USART->CR3, USART_CR3_DEM_Pos);
/*
Bit 15 DEP: Driver enable polarity selection
    0: DE signal is active high. 
    1: DE signal is active low.
        This bit can only be written when the USART is disabled (UE = 0).
        Note: If the Driver Enable feature is not supported, this bit is reserved and must be kept at reset value. 
*/                                                                                                 
#define USART_DE_SIGNAL_ACTIVE_HIGH(USART) CLEAR_BIT(USART->CR3, USART_CR3_DEP_Pos);       
#define USART_DE_SIGNAL_ACTIVE_LOW(USART)  SET_BIT  (USART->CR3, USART_CR3_DEP_Pos);
/*
Bit 16 Reserved
    Must be kept at reset value.     
*/
/*
Bits 19:17 SCARCNT[2:0]: Smartcard auto-retry count
    This bitfield specifies the number of retries for transmission and reception in Smartcard mode.
    In transmission mode, it specifies the number of automatic retransmission retries, before 
    generating a transmission error (FE bit set).
    In reception mode, it specifies the number or erroneous reception trials, before generating a 
    reception error (RXNE/RXFNE and PE bits set).
    This bitfield must be programmed only when the USART is disabled (UE = 0).
    When the USART is enabled (UE = 1), this bitfield may only be written to 0x0, in order to 
    stop retransmission. 
        0x0: retransmission disabled - No automatic retransmission in transmit mode. 
        0x1 to 0x7: number of automatic retransmission attempts (before signaling error)
        Note: If Smartcard mode is not supported, this bit is reserved and must be kept at reset value.
*/
#define USART_SCARCNT(USART)  (Doco:BITS_17,18,19)
/*
Bits 21:20 WUS[1:0]: Wakeup from low-power mode interrupt flag selection
    This bitfield specifies the event which activates the WUF (Wakeup from low-power mode flag). 
    00: WUF active on address match (as defined by ADD[7:0] and ADDM7)
    01: Reserved.
    10: WUF active on start bit detection
    11: WUF active on RXNE/RXFNE. 
        This bitfield can only be written when the USART is disabled (UE = 0).
        If the USART does not support the wakeup from Stop feature, this bit is reserved and must be kept at reset value
*/
#define USART_WAKEUP_ON_ADDRESS_MATCH(USART)     CLEAR_BIT(USART->CR3, USART_CR3_WUS_Pos);CLEAR_BIT(USART->CR3, USART_CR3_WUS_Pos);
#define USART_WAKEUP_ON_DETECT_START_BIT(USART)  SET_BIT  (USART->CR3, USART_CR3_WUS_Pos);CLEAR_BIT(USART->CR3, USART_CR3_WUS_Pos);
#define USART_WAKEUP_ON_RXNE_RXFNE(USART)        SET_BIT  (USART->CR3, USART_CR3_WUS_Pos);SET_BIT  (USART->CR3, USART_CR3_WUS_Pos);       
/*
Bit 22 WUFIE: Wakeup from low-power mode interrupt enable
    This bit is set and cleared by software.
    0: Interrupt inhibited
    1: USART interrupt generated whenever WUF = 1 in the USART_ISR register
        Note: WUFIE must be set before entering in low-power mode.
        The WUF interrupt is active only in low-power mode.
        If the USART does not support the wakeup from Stop feature, this bit is reserved and must be kept at reset value
*/
#define USART_WAKEUP_DISABLE(USART) CLEAR_BIT(USART->CR3, USART_CR3_WUFIE_Pos);       
#define USART_WAKEUP_ENABLE(USART)  SET_BIT(USART->CR3, USART_CR3_WUFIE_Pos);
/*
Bit 23 TXFTIE: TXFIFO threshold interrupt enable
    This bit is set and cleared by software.
    0: Interrupt inhibited
    1: USART interrupt generated when TXFIFO reaches the threshold programmed in TXFTCFG
*/       
#define USART_TXFIFO_THESHOLD_INTERRUPT_DISABLE(USART)  CLEAR_BIT(USART->CR3, USART_CR3_TXFTIE_Pos);       
#define USART_TXFIFO_THESHOLD_INTERRUPT_ENABLE(USART)   SET_BIT(USART->CR3, USART_CR3_TXFTIE_Pos);
/*
Bit 24 TCBGTIE: Transmission Complete before guard time, interrupt enable
    This bit is set and cleared by software.
    0: Interrupt inhibited
    1: USART interrupt generated whenever TCBGT=1 in the USART_ISR register
        Note: If the USART does not support the Smartcard mode, this bit is reserved and must be kept at reset value. 
*/                                                                                                                  
#define USART_TX_COMPLETE_GUARD_TIME_INTERRUPT_DISABLE(USART) CLEAR_BIT(USART->CR3, USART_CR3_TCBGTIE_Pos);       
#define USART_TX_COMPLETE_GUARD_TIME_INTERRUPT_ENABLE(USART)  SET_BIT(USART->CR3, USART_CR3_TCBGTIE_Pos);
/*
Bits 27:25 RXFTCFG[2:0]: Receive FIFO threshold configuration
    000:Receive FIFO reaches 1/8 of its depth
    001:Receive FIFO reaches 1/4 of its depth
    010:Receive FIFO reaches 1/2 of its depth
    011:Receive FIFO reaches 3/4 of its depth
    100:Receive FIFO reaches 7/8 of its depth
    101:Receive FIFO becomes full
        Remaining combinations: Reserved
*/
#define USART_RXFIFO_THRESHOLD_1_8(USART) MODIFY_REG(USART->CR3, USART_CR3_RXFTCFG ,UART_RXFIFO_THRESHOLD_1_8 )
#define USART_RXFIFO_THRESHOLD_1_4(USART) MODIFY_REG(USART->CR3, USART_CR3_RXFTCFG ,UART_RXFIFO_THRESHOLD_1_4 )
#define USART_RXFIFO_THRESHOLD_1_2(USART) MODIFY_REG(USART->CR3, USART_CR3_RXFTCFG ,UART_RXFIFO_THRESHOLD_1_2 )
#define USART_RXFIFO_THRESHOLD_3_4(USART) MODIFY_REG(USART->CR3, USART_CR3_RXFTCFG ,UART_RXFIFO_THRESHOLD_3_4 )
#define USART_RXFIFO_THRESHOLD_7_8(USART) MODIFY_REG(USART->CR3, USART_CR3_RXFTCFG ,UART_RXFIFO_THRESHOLD_7_8 )
#define USART_RXFIFO_THRESHOLD_8_8(USART) MODIFY_REG(USART->CR3, USART_CR3_RXFTCFG ,UART_RXFIFO_THRESHOLD_8_8 )

/*
Bit 28 RXFTIE: RXFIFO threshold interrupt enable
    This bit is set and cleared by software.
    0: Interrupt inhibited
    1: USART interrupt generated when Receive FIFO reaches the threshold programmed in RXFTCFG
*/
#define USART_RXFIFO_THESHOLD_INTERRUPT_DISABLE(USART) CLEAR_BIT(USART->CR3, USART_CR3_RXFTIE_Pos);       
#define USART_RXFIFO_THESHOLD_INTERRUPT_ENABLE(USART)  SET_BIT(USART->CR3, USART_CR3_RXFTIE_Pos);

/*
Bits 31:29 TXFTCFG[2:0]: TXFIFO threshold configuration
000:TXFIFO reaches 1/8 of its depth
001:TXFIFO reaches 1/4 of its depth
010:TXFIFO reaches 1/2 of its depth
011:TXFIFO reaches 3/4 of its depth
100:TXFIFO reaches 7/8 of its depth
101:TXFIFO becomes empty
Remaining combinations: Reserved
*/
#define USART_TXFIFO_THRESHOLD_1_8(USART) MODIFY_REG(USART->CR3, USART_CR3_TXFTCFG ,UART_TXFIFO_THRESHOLD_1_8 )
#define USART_TXFIFO_THRESHOLD_1_4(USART) MODIFY_REG(USART->CR3, USART_CR3_TXFTCFG ,UART_TXFIFO_THRESHOLD_1_4 )
#define USART_TXFIFO_THRESHOLD_1_2(USART) MODIFY_REG(USART->CR3, USART_CR3_TXFTCFG ,UART_TXFIFO_THRESHOLD_1_2 )
#define USART_TXFIFO_THRESHOLD_3_4(USART) MODIFY_REG(USART->CR3, USART_CR3_TXFTCFG ,UART_TXFIFO_THRESHOLD_3_4 )
#define USART_TXFIFO_THRESHOLD_7_8(USART) MODIFY_REG(USART->CR3, USART_CR3_TXFTCFG ,UART_TXFIFO_THRESHOLD_7_8 )
#define USART_TXFIFO_THRESHOLD_8_8(USART) MODIFY_REG(USART->CR3, USART_CR3_TXFTCFG ,UART_TXFIFO_THRESHOLD_8_8 )


    
  

