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

uint8_t sccb_bus_write_byte(uint8_t data){
	I2C_SendData(I2C1, 0x60, data, 1);
	return 1;
}

uint8_t sccb_bus_read_byte(void){
	uint8_t Data_Receive;	
	I2C_ReceiveData(I2C1, 0x61, Data_Receive, 1);
	return Data_Receive;
}

