#include <stdio.h>
#include <stdbool.h>
#include "main.h"

void Init_USARTx() {
	UART1_Init();
	UART1_GPIO_Init();
	USART_Init(USART1);
}

void Init_IR_Sensor(){
	// Enable GPIO clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	// Set mode of GPIO B2 to input
	GPIOB->MODER &= ~(GPIO_MODER_MODE2);

	// Configure PB2 output type to push-pull
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT2);

	// Configure PB2 output type as No Pull-Up, No Pull-Down
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD2);
}

void configs(){
	System_Clock_Init(); // Switch System Clock = 80 MHz
	Init_USARTx();
	
	SysTick_Init();
	SPI1_Init();

	//sccb_bus_init();
	//ArduCAM_Init(sensor_model);
}

void check_SPI_interface(){
	uint8_t temp;
	while(1){
		write_reg(ARDUCHIP_TEST1, 0x55);
		temp = read_reg(ARDUCHIP_TEST1);
		if (temp != 0x55){
			printf("ACK CMD SPI interface Error! Value of temp: %d\n", temp);
			delay_ms(1000);
			continue;
		}
		else{
			printf("ACK CMD SPI interface OK!\r\n");
			break;
		}
	}
}

void find_cam_module(){
	uint8_t vid, pid;
	while(1){
		sensor_addr = 0x60;
		wrSensorReg8_8(0xff, 0x01);
		rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
		rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
		if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 )))
			printf("ACK CMD Can't find OV2640 module!\r\n");
		else{
			sensor_model =  OV2640;
			printf("ACK CMD OV2640 detected.\r\n");   
			break;
		}
	}
}

int main(void){
	configs();
	check_SPI_interface();
	// find_cam_module();
	
	/*
	while(1){
		// Currently B2: probably will have to change
		// int motion_detected = ((GPIOB->IDR & GPIO_IDR_ID2) == GPIO_IDR_ID2);

		// Change dis
		if (1){
			printf("Motion detected!\n");
			SingleCapTransfer();
			SendbyUSART1();
		}
	}
	*/
}
