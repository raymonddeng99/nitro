#include "delay.h"
#include <stdint.h>

static uint8_t  fac_us=0;
static uint16_t fac_ms=0;								    
		    								   
void delay_us(u32 nus){
	
	uint32_t i, j;
	for (i = 0; i < nus; i++) {
		for (j = 0; j < 18; j++) // This is an experimental value.
			(void)i;
	}

}

void delay_ms(u16 nms){
	delay(nms);
} 
