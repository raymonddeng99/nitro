/**
  * @file    sccb_bus.c 
  * @author  Arducam
  * @version V0.1
  * @date    2018.06.22
  * @brief   sccb bus
  */
#include "sccb_bus.h"
#include "I2C.h"
#include "delay.h"
 uint32_t ntime;

/**
  * @brief  init i2c bus
  * @param  None
  * @retval None
  */
void sccb_bus_init(void){
	// Enable GPIO clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	// B11: input
	GPIOB->MODER &= ~(GPIO_MODER_MODE11);

	// B11: push-pull
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT2);

	// B11: Pull-Up, No Pull-Down
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD2);

	// B11: 50 MHz						 
	GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED11);



 	// B10: output mode
	GPIOB->MODER &= ~GPIO_MODER_MODE10;
	GPIOB->MODER |= GPIO_MODER_MODE10_0;
	
	//B10: output type as Push-Pull
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT10;
	
	//B10: as No Pull-Up, No Pull-Down
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD10;
}

uint8_t sccb_bus_write_byte(uint8_t data){
	I2C_SendData(I2C1, 0x60, data, 1);
	return 1;
}

uint8_t sccb_bus_read_byte(void){
	uint8_t Data_Receive;	
	I2C_ReceiveData(I2C1, 0x61, Data_Receive, 1);
	return Data_Receive;
}

