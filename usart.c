#include "usart.h"
#include "uart.h"

unsigned char USART1_RecieveData;
unsigned char NewCMD = 0;

void USART1_UART_Init(uint32_t BaudRate){
	/*
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

  	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate = BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  	USART_Init(USART1, &USART_InitStructure);
  	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  	USART_Cmd(USART1, ENABLE);
		*/
}

void UART1_BulkOut(uint32_t len, uint8_t *p){
	for (uint32_t cnt = 0; cnt != len; cnt++){    
  		// while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
  		USART_Write(USART1, *p, 1);
    	p++;    
	}
}

/*
void USART1_IRQHandler(void){
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		USART1_RecieveData = USART_ReceiveData(USART1);
		NewCMD = 1;
	} 
}
*/
