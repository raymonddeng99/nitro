#ifndef _SCCB_BUS_H_
#define _SCCB_BUS_H_

#define I2C_TIM 1

#include <stdint.h>
#include "stm32l476xx.h"
#include "main.h"
#include "I2C.h"

void sccb_bus_init(void);
void sccb_bus_start(void);
void sccb_bus_stop(void);
void sccb_bus_send_noack(void);
void sccb_bus_send_ack(void);
uint8_t sccb_bus_write_byte(uint8_t data);
uint8_t sccb_bus_read_byte(void);


#endif /* _SCCB_BUS_H_ */

/******************* (C) COPYRIGHT 2015 WangBao *****END OF FILE****/
