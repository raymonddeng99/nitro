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
	
	Init_IR_Sensor();
	
	SysTick_Init();
	SPI1_Init();
}

int main(void){
	configs();
	

	while(1){
		printf("running\n");
		
		int motion_detected = ((GPIOB->IDR & GPIO_IDR_ID2) == GPIO_IDR_ID2);
		
		if (motion_detected){
			printf("Motion detected! State of motion_detected: %d\n", motion_detected);
 
			uint8_t start_code = 0x20;
			uint8_t receiveData;
			//uint8_t num_faces_detected;
		
			//delay(2);
		
			SPI_Write(SPI1, start_code, &receiveData, 1);
		
			//SPI_Read(SPI1, num_faces_detected, 1);
		
			//printf("Estimated number of people nearby: %d\n\n", num_faces_detected);
		}
		
		else{
			uint8_t stop_code = 0x21;
			uint8_t receiveData;
			
			SPI_Write(SPI1, stop_code, &receiveData, 1);
		}
	}
}
