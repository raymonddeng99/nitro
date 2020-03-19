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
}

int main(void){
	configs();

	int num_faces_detected;
	
	while(1){
		int motion_detected = ((GPIOB->IDR & GPIO_IDR_ID2) == GPIO_IDR_ID2);

		// Change dis
		if (1){
			printf("Motion detected!\n");

			uint8_t start = 10;
			uint8_t data_received;

			SPI_Write(SPI1, start, data_received, 1);
		}

		scanf("%d", &num_faces_detected);
		printf("Estimated number of people nearby: %d", num_faces_detected);
	}
}
