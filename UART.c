#include "UART.h"

void UART1_Init(void) {
	// Enable UART1 clock in peripheral clock register
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	// Select system clock as UART1 clock source in peripheral independent clock config register
	RCC->CCIPR &= ~(RCC_CCIPR_USART1SEL);
	RCC->CCIPR |= RCC_CCIPR_USART1SEL_0;
}

void UART2_Init(void) {
	// Enable UART2 clock in peripheral clock register
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

	// Select system clock as UART2 clock source in peripheral independent clock config register
	RCC->CCIPR &= ~(RCC_CCIPR_USART2SEL);
	RCC->CCIPR |= RCC_CCIPR_USART2SEL_0;
}

void UART1_GPIO_Init(void) {
	// Configure PB6, PB7 to operate as UART TX/RX

	// Enable clock for PB6, PB7
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	
	// Set both GPIO pins to alternative function mode
	GPIOB->MODER &= ~(GPIO_MODER_MODE6);
	GPIOB->MODER &= ~(GPIO_MODER_MODE7);
	GPIOB->MODER |= GPIO_MODER_MODE6_1;
	GPIOB->MODER |= GPIO_MODER_MODE7_1;

	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL6;
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL6_3);
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL7;
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL7_3);

	// Set both pins to very high speed
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED6);
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED7);
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED6;
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED7;

	// Set both pins to have a push-pull output type
	GPIOB->OTYPER |= GPIO_OTYPER_OT6;
	GPIOB->OTYPER |= GPIO_OTYPER_OT7;

	// Configure both GPIO pins to use pull-up resistors for I/O
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD6;
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD7;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD6_0;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD7_0;
}

void UART2_GPIO_Init(void) {
	// Configure PD5, PD6 to operate as UART TX/RX

	// Enable clock for PD5, PD6
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
	
	// Set both GPIO pins to alternative function mode (AF7)
	GPIOD->MODER &= ~(GPIO_MODER_MODE5);
	GPIOD->MODER &= ~(GPIO_MODER_MODE6);
	GPIOD->MODER |= GPIO_MODER_MODE5_1;
	GPIOD->MODER |= GPIO_MODER_MODE6_1;


	GPIOD->AFR[0] |= GPIO_AFRL_AFSEL5;
	GPIOD->AFR[0] &= ~(GPIO_AFRL_AFSEL5_3);
	GPIOD->AFR[0] |= GPIO_AFRL_AFSEL6;
	GPIOD->AFR[0] &= ~(GPIO_AFRL_AFSEL6_3);



	// Set both pins to very high speed
	GPIOD->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5);
	GPIOD->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED6);
	GPIOD->OSPEEDR |= GPIO_OSPEEDR_OSPEED5;
	GPIOD->OSPEEDR |= GPIO_OSPEEDR_OSPEED6;

	// Set both pins to have a push-pull output type
	GPIOD->OTYPER |= GPIO_OTYPER_OT5;
	GPIOD->OTYPER |= GPIO_OTYPER_OT6;

	// Configure both GPIO pins to use pull-up resistors for I/O
	GPIOD->PUPDR &= ~GPIO_PUPDR_PUPD5;
	GPIOD->PUPDR &= ~GPIO_PUPDR_PUPD6;
	GPIOD->PUPDR |= GPIO_PUPDR_PUPD5_0;
	GPIOD->PUPDR |= GPIO_PUPDR_PUPD6_0;
}

void USART_Init(USART_TypeDef* USARTx) {
	USARTx->CR1 &= ~(USART_CR1_UE);

	// In control registers, set:
	//		word length 		:= 8 bits
	USARTx->CR1 &= ~(USART_CR1_M0);
	USARTx->CR1 &= ~(USART_CR1_M1);
	//		oversampling mode 	:= oversampling by 16
	USARTx->CR1 &= ~(USART_CR1_OVER8);
	//		num stop bits 		:= 1
	USARTx->CR2 &= ~(USART_CR2_STOP);


	// Set baud rate := 460800
	// What is frequency of system clock? System clock freq = 80 MHz
	// USART_DIV = f_CLK / TX_RX_Baud_Rate = 80 MHz / 460800 = 174
	USARTx->BRR = 174;



	// In control registers, enable transmitter and receiver
	USARTx->CR1 |= USART_CR1_TE;
	USARTx->CR1 |= USART_CR1_RE;

	// Enable USART in control registers
	USARTx->CR1 |= USART_CR1_UE;
}

void UART1_BulkOut(uint32_t len, uint8_t *p){
	for (uint32_t cnt = 0; cnt != len; cnt++){    
  		// while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
  		USART_Write(USART1, *p, 1);
    	p++;    
	}
}

uint8_t USART_Read (USART_TypeDef * USARTx) {
	// SR_RXNE (Read data register not empty) bit is set by hardware
	while (!(USARTx->ISR & USART_ISR_RXNE));  // Wait until RXNE (RX not empty) bit is set
	// USART resets the RXNE flag automatically after reading DR
	return ((uint8_t)(USARTx->RDR & 0xFF));
	// Reading USART_DR automatically clears the RXNE flag 
}

void USART_Write(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t nBytes) {
	int i;
	// TXE is cleared by a write to the USART_DR register.
	// TXE is set by hardware when the content of the TDR 
	// register has been transferred into the shift register.
	for (i = 0; i < nBytes; i++) {
		while (!(USARTx->ISR & USART_ISR_TXE));   	// wait until TXE (TX empty) bit is set
		// Writing USART_DR automatically clears the TXE flag 	
		USARTx->TDR = buffer[i] & 0xFF;
		USART_Delay(300);
	}
	while (!(USARTx->ISR & USART_ISR_TC));   		  // wait until TC bit is set
	USARTx->ISR &= ~USART_ISR_TC;
}   

void USART_Delay(uint32_t us) {
	uint32_t time = 100*us/7;    
	while(--time);   
}

void USART_IRQHandler(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t * pRx_counter) {
	if(USARTx->ISR & USART_ISR_RXNE) {						// Received data                         
		buffer[*pRx_counter] = USARTx->RDR;         // Reading USART_DR automatically clears the RXNE flag 
		(*pRx_counter)++;  
		if((*pRx_counter) >= BufferSize )  {
			(*pRx_counter) = 0;
		}   
	} else if(USARTx->ISR & USART_ISR_TXE) {
 		//USARTx->ISR &= ~USART_ISR_TXE;            // clear interrupt 
		//Tx1_Counter++;
	} else if(USARTx->ISR & USART_ISR_ORE) {			// Overrun Error
		while(1);
	} else if(USARTx->ISR & USART_ISR_PE) {				// Parity Error
		while(1);
	} else if(USARTx->ISR & USART_ISR_PE) {				// USART_ISR_FE	
		while(1);
	} else if (USARTx->ISR & USART_ISR_NE){ 			// Noise Error Flag
		while(1);     
	}	
}
