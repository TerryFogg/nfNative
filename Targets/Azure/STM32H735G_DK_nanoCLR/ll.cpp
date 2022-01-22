//
//#include "stm32h7xx.h"
//#include "stm32h7xx_ll_bus.h"
//#include "stm32h7xx_ll_i2c.h"
//#include "stm32h7xx_ll_gpio.h"
//#include "stm32h7xx_ll_i2c.h"
//
//
//
//void ll()
//{
//    // Enable clock on the Port the SDA and SCL pins for IC4 are found
//    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD); 
//
//    
//    /* Configure I2C Tx as alternate function */
//    LL_GPIO_InitTypeDef  GPIO_InitStruct;
//    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
//    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
//    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//    GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
//    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//
//    LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_I2C4);
//    
//    LL_I2C_DisableOwnAddress2(I2C4);
//    LL_I2C_DisableGeneralCall(I2C4);
//    LL_I2C_EnableClockStretching(I2C4);
//    
//    
//    LL_I2C_InitTypeDef  I2C_InitStruct;
//    
//    I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
//    I2C_InitStruct.OwnAddress1 = 0;
//    I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
//    I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
//    
//    
//    I2C_InitStruct.Timing  = BUS_I2C4_FREQUENCY;
//    I2C_InitStruct.AnalogFilter = 0;
//    I2C_InitStruct.DigitalFilter = 0;
//    
//    
//    LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
//    
//    
//
//  
//    hI2c->Init.Timing           = timing;
//    hI2c->Init.OwnAddress1      = 0;
//    hI2c->Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
//    hI2c->Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
//    hI2c->Init.OwnAddress2      = 0;
//    hI2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
//    hI2c->Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
//    hI2c->Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
//
//
//    LL_I2C_Init(I2C1, &I2C_InitStruct);
//}
//
//  
//
//
///*
//
//
//
//
//3- The related GPIO pins are configured for I2C
//
//
//
//
//
//
//
//4- The I2C peripheral clock enabled
//
//
//
///* Peripheral clock enable */
//LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
//
//
//
//
//5 - The I2C is configured
//
//
//speed mode selection
//
//Duty cycle selection
//
//Own address selection
//
//Address length selection
//
//
//
//LL_I2C_DisableOwnAddress2(I2C1);
//LL_I2C_DisableGeneralCall(I2C1);
//LL_I2C_EnableClockStretching(I2C1);
//I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
//I2C_InitStruct.ClockSpeed = 400000;
//I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
//I2C_InitStruct.OwnAddress1 = 0;
//I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
//I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
//LL_I2C_SetOwnAddress2(I2C1, 0);
//
//
//6 - The I2C is initialized
//
//
//
//LL_I2C_Init(I2C1, &I2C_InitStruct);
//
//
//7 - Generate a start condition
//
//
//
//LL_I2C_GenerateStartCondition(I2C1);
//while (!LL_I2C_IsActiveFlag_SB(I2C1)) {}
//;
//(void) I2C1->SR1;
//
//
//
//8 - Transmit MMA8452 write address to send register address
//
//
//
//LL_I2C_TransmitData8(I2C1, dev_write_ADDR);
//while (!LL_I2C_IsActiveFlag_ADDR(I2C1)) {}
//;
//LL_I2C_ClearFlag_ADDR(I2C1);
//
//
//
//
//
//9 - Send the registered address
//
//
//
//LL_I2C_TransmitData8(I2C1, (uint8_t)(0x0D));
//while (!LL_I2C_IsActiveFlag_TXE(I2C1)) {}
//;
//
//
//
//
//
//
//10 - generate start condition again
//
//
//
//LL_I2C_GenerateStartCondition(I2C1);
//while (!LL_I2C_IsActiveFlag_SB(I2C1)) {}
//;
//(void) I2C1->SR1;
//
//
//
//11 - send the MMA8452's read address
//
//
//
//LL_I2C_TransmitData8(I2C1, dev_read_ADDR);
//while (!LL_I2C_IsActiveFlag_ADDR(I2C1)) {}
//;
//LL_I2C_ClearFlag_ADDR(I2C1);
//
//
//
//12 - define the read data to a variable
//
//
//
//value = LL_I2C_ReceiveData8(I2C1);
//
//
//
//
//13 - generate a stop condition and send NACK to stop the receiving
//
//
//
//LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
//LL_I2C_GenerateStopCondition(I2C1);
//
//
//
//
//
//
//
//
//
//
//
//
//
//*/