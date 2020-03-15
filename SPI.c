#include "spi.h"

#define BMPIMAGEOFFSET 66
#define pgm_read_byte(x)        (*((char *)x))
const char bmp_header[BMPIMAGEOFFSET] = {
  0x42, 0x4D, 0x36, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x00, 0x28, 0x00,
  0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0x03, 0x00,
  0x00, 0x00, 0x00, 0x58, 0x02, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x1F, 0x00,
  0x00, 0x00
};

uint8_t	*picbuf = 0;
bool receive_OK = false;    
bool USB_sendOK = true;    
bool send_OK = true;
uint32_t sendlen = 0;
uint32_t haveRev = 0;
uint32_t noRev = 0;
uint8_t  EP2_SendFinish = 1;
uint8_t	Buf1[BUFFER_MAX_SIZE]={0}, Buf2[BUFFER_MAX_SIZE]={0};
extern uint16_t NumPackage;

void SPI2_GPIO_Init(){
	// Configure PB[13, 15] to desired settings

	// Enable clock for PB[13, 15]
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	
	// Set all GPIO pins to alternative function mode (AF5)
	GPIOB->MODER &= ~(GPIO_MODER_MODE13);
	GPIOB->MODER &= ~(GPIO_MODER_MODE14);
	GPIOB->MODER &= ~(GPIO_MODER_MODE15);
	GPIOB->MODER |= GPIO_MODER_MODE13_1;
	GPIOB->MODER |= GPIO_MODER_MODE14_1;
	GPIOB->MODER |= GPIO_MODER_MODE15_1;
	
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL13);
	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL13_0;
	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL13_2;
	
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL14);
	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL14_0;
	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL14_2;
	
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL15);
	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL15_0;
	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL15_2;
	

	// Set all pins to very high speed
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED13);
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED14);
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED15);
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED13;
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED14;
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED15;

	// Set all pins to have a push-pull output type
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT13);
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT14);
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT15);

	// Configure all GPIO pins to no pull-up, pull-down
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD13);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD14);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD15);
}

void SPI_Init(void){
	SPI2->CR1 &= ~(SPI_CR1_SPE);
	
	// Enable SPI2 clock in peripheral clock register
	RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;

	// Reset SPI2 by setting appropriate bits in peripheral reset register
	RCC->APB1RSTR1 |= RCC_APB1RSTR1_SPI2RST;

	// Clear bits so SPI2 does not remain in a reset state
	RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_SPI2RST);

	// Configure the serial channel for full duplex communication
	SPI2->CR1 &= ~(SPI_CR1_RXONLY);

	// Configure the communication for 2-line unidirectional data mode
	SPI2->CR1 &= ~(SPI_CR1_BIDIMODE);

	// Disable output in bidirectional mode
	SPI2->CR1 &= ~(SPI_CR1_BIDIOE);

	// Set: 
	//		the data frame for receiving the MSB first when data is transmitted
	SPI2->CR1 &= ~(SPI_CR1_LSBFIRST);

	// 		the data length to 8 bits
	SPI2->CR2 &= ~(SPI_CR2_DS);
	SPI2->CR2 |= SPI_CR2_DS;
	SPI2->CR2 &= ~(SPI_CR2_DS_3);

	//		the frame format to be in SPI Motorola mode
	SPI2->CR2 &= ~(SPI_CR2_FRF);

	// Set: 
	//		the clock polarity to 0
	SPI2->CR1 &= ~(SPI_CR1_CPOL);

	//		the clock phase s.t. the first clock transition is the first data capture edge
	SPI2->CR1 &= ~(SPI_CR1_CPHA);

	// Set the baud rate prescaler to 16
	SPI2->CR1 &= ~(SPI_CR1_BR);
	SPI2->CR1 |= SPI_CR1_BR;
	SPI2->CR1 &= ~(SPI_CR1_BR_2);

	// Disable CRC calculation
	SPI2->CR1 &= ~(SPI_CR1_CRCEN);

	// Set: 
	//		the board to operate in master mode
	SPI2->CR1 |= SPI_CR1_MSTR;

	//		enable software slave management
	SPI2->CR1 |= SPI_CR1_SSM;

	//		enable NSS pulse management
	SPI2->CR2 |= SPI_CR2_NSSP;

	// Set the internal slave bit
	SPI2->CR1 |= SPI_CR1_SSI;

	// Set the FIFO reception threshold to 1/4 (8 bit)
	SPI2->CR2 |= SPI_CR2_FRXTH;

	// Enable SPI 
	SPI2->CR1 |= SPI_CR1_SPE;
}

/*
	DMA provides high-speed transfer between peripherals and memory.
	2 DMA controllers have 14 channels in total, each dedicated to managing memory access requests from >=1 peripherals.
	Each has an arbiter for handling the priority between DMA requests.

	Channel 4, request num 1: SPI2_RX
	Channel 5, request num 1: SPI2_TX
	Channel 6, request num 2: USART1_TX
*/
void DMA_config(){
	// Set peripheral register address in DMA_CPARx register (!!!!!)
	/*
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI2->DR;
  	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART2->DR;
  	*/

	DMA_CPAR4 = (u32)(SPI2_BASE + 0x0C)
	DMA_CPAR5 = (u32)(SPI2_BASE + 0x0C)
	DMA_CPAR6 = (u32)(USART2_BASE + 0x28)

	// Set memory address in DMA_CMARx register
	// Configure total amount of memory to be transferred in DMA_CNDTRx register
	// 			see DMA1_RX, DMA1_SendtoUsart

	// Configure channel priority using PL[1:0] in DMA_CCRx 
	DMA_CCR4 |= DMA_CCR_PL;
	DMA_CCR4 &= ~(DMA_CCR_PL_0);

	DMA_CCR5 |= DMA_CCR_PL;
	DMA_CCR5 &= ~(DMA_CCR_PL_0);

	DMA_CCR6 |= DMA_CCR_PL;

	// Configure:
	//		M2M mode (Arducam disables this)
	DMA_CCR4 &= ~(DMA_CCR_MEM2MEM);
	DMA_CCR5 &= ~(DMA_CCR_MEM2MEM);
	DMA_CCR6 &= ~(DMA_CCR_MEM2MEM);

	//		data transfer direction
	DMA_CCR4 &= ~(DMA_CCR_DIR);
	DMA_CCR5 |= DMA_CCR_DIR;
	DMA_CCR6 |= DMA_CCR_DIR;

	//		circular mode
	DMA_CCR4 &= ~(DMA_CCR_CIRC);
	DMA_CCR5 &= ~(DMA_CCR_CIRC);
	DMA_CCR6 &= ~(DMA_CCR_CIRC);


	//		peripheral and memory incremented mode
	DMA_CCR4 |= DMA_CCR_MINC;
	DMA_CCR4 &= ~(DMA_CCR_PINC);

	DMA_CCR5 |= DMA_CCR_MINC;
	DMA_CCR5 &= ~(DMA_CCR_PINC);

	DMA_CCR6 |= DMA_CCR_MINC;
	DMA_CCR6 &= ~(DMA_CCR_PINC);

	//		peripheral data size
	DMA_CCR4 |= DMA_CCR_PSIZE;
	DMA_CCR4 &= ~(DMA_CCR_PSIZE_0);

	DMA_CCR5 |= DMA_CCR_PSIZE;
	DMA_CCR5 &= ~(DMA_CCR_PSIZE_0);

	DMA_CCR6 |= DMA_CCR_PSIZE;
	DMA_CCR6 &= ~(DMA_CCR_PSIZE_0);

	// 		memory data size
	DMA_CCR4 |= DMA_CCR_MSIZE;
	DMA_CCR4 &= ~(DMA_CCR_MSIZE_0);

	DMA_CCR5 |= DMA_CCR_MSIZE;
	DMA_CCR5 &= ~(DMA_CCR_MSIZE_0);

	DMA_CCR6 |= DMA_CCR_MSIZE;
	DMA_CCR6 &= ~(DMA_CCR_MSIZE_0);

	//		interrupt after half/full transfer in DMA_CCRx register
	DMA_CCR4 |= DMA_CCR_TCIE;
	DMA_CCR5 |= DMA_CCR_TCIE;  
	DMA_CCR6 |= DMA_CCR_TCIE;
}

void NVIC_Config(){
  	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	NVIC_SetPriority(DMA1_Channel4_IRQn, 0);

  	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	NVIC_SetPriority(DMA1_Channel6_IRQn, 0);
}

void SPI2_Init(void){
	SPI2_GPIO_Init();
	SPI_Init();

	SPI2_ReadWriteByte(0xff);

	DMA_Config();
	NVIC_Config();
}
 
void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler){
  	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI2->CR1 &= 0XFFC7;
	SPI2->CR1|= SPI_BaudRatePrescaler;
	SPI2->CR1 |= SPI_CR1_SPE
} 

u8 SPI2_ReadWriteByte(u8 TxData){
	uint8_t rxBuffer[32];
	SPI_Write(SPI2, TxData, rxBuffer, 1)
	return SPI_Read(SPI2, rxBuffer, 1)				    
}


/*
	Only 16 bits (65535) can be transferred at a time; maybe look into circular mode

	"If the channel is configured in non-circular mode, no DMA request is served after the last transfer
	(that is once the num of data items to be transferred has reached zero). In order to reload a new number
	of data items to be transferred into the DMA_CNDTRx register, the DMA channel must be disabled".

	"In circular mode, after the last transfer, the DMA_CNDTRx register is automatically reloaded with the initially
	programmed value. The current internal address registers are reloaded with the base address values from the 
	DMA_CPARx/DMA_CMARx registers".
*/

void DMA1_RX(uint8_t *p , uint32_t len){		
  	CS_LOW();
	set_fifo_burst();

	if (len > 65535){
		printf("DMA1RX: DMA_CNDTR only allows values up to 65535. len: %p:\n", len);
		return;
	}

	/*
	STM32 manual:
		when MSIZE = 10 (32 bit), MA[1:0] bits are ignored.
		Do I left shift address by 2?
	*/

	DMA_CMAR4 = (u32)p << 2;
	DMA_CNDTR4 = (u16)len;
	DMA_CMAR5 = (u32)p << 2;
	DMA_CNDTR5 = (u16)len;

	DMA1_CSELR &= ~(DMA_CSELR_C4S | DMA_CSELR_C5S);
	DMA1_CSELR |= (1 << 12 | 1 << 16);

	DMA2_CSELR &= ~(DMA_CSELR_C6S);
	DMA2_CSELR |= (1 << 21);

	// Active channel by setting ENABLE bit in DMA_CCRx register
	DMA_CCR4 |= DMA_CCR_EN;
	DMA_CCR5 |= DMA_CCR_EN;
}

void DMA1_SendtoUsart(uint8_t *p , uint32_t len){
	if (len > 65535){
		printf("DMA1RX: DMA_CNDTR only allows values up to 65535. len: %p:\n", len);
		return;
	}

	DMA_CMAR6 = (u32)p << 2;
	DMA_CNDTR6 = (u16)len;

	DMA_CCR6 |= DMA_CCR_EN;
}

void SendbyUSART1(void){	
  	uint8_t	*sdbuf;
	haveRev += sendlen;
  	if(haveRev < length){	
		if(picbuf == Buf1){		
			sdbuf = Buf1;
			picbuf = Buf2;	
		}
		else{
			sdbuf = Buf2;
			picbuf = Buf1;
		}
	  	UART1_BulkOut(sendlen,sdbuf);
    	noRev	= length - haveRev;		
		sendlen	= (noRev>=BUFFER_MAX_SIZE) ? BUFFER_MAX_SIZE : noRev;	
		DMA1_RX(picbuf, sendlen);	
	}
	else{
		UART1_BulkOut(sendlen, picbuf);
		send_OK = 1;
	}			 	 					 	 	
}

void SingleCapTransfer(void){
	flush_fifo();
	clear_fifo_flag();
	start_capture(); 
	while(!get_bit(ARDUCHIP_TRIG , CAP_DONE_MASK)){;}
	//printf("ACK CMD capture done\r\n");
	length= read_fifo_length();
	//printf("ACK CMD the length is %d\r\n",length);
	sendlen = (length>=BUFFER_MAX_SIZE) ? BUFFER_MAX_SIZE : length;
	picbuf = Buf1;
	haveRev = 0;
	DMA1_RX(picbuf, sendlen);
}

void DMA4_Channel1_IRQHandler(void){ 	
	if(DMA_GetITStatus(DMA1_IT_TC4)){
		DMA_ClearITPendingBit(DMA1_IT_GL4 | DMA1_IT_TC4 | DMA1_IT_GL5 | DMA1_IT_TC5);

		DMA_CCR4 &= ~(DMA_CCR_EN);
		DMA_CCR5 &= ~(DMA_CCR_EN);

		// SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, DISABLE);
		CS_HIGH();
		receive_OK =1;
	}
}


void DMA6_Channel2_IRQHandler(void){
	if(DMA_GetITStatus(DMA1_IT_TC7)){
		DMA_ClearITPendingBit(DMA1_IT_GL7 | DMA1_IT_TC7);
		DMA_CCR6 &= ~(DMA_CCR_EN);
	}
}
 
void SPI_Write(SPI_TypeDef * SPIx, uint8_t *txBuffer, uint8_t * rxBuffer, int size) {
	volatile uint32_t tmpreg; 
	int i = 0;
	for (i = 0; i < size; i++) {
		while( (SPIx->SR & SPI_SR_TXE ) != SPI_SR_TXE );  // Wait for TXE (Transmit buffer empty)
		*((volatile uint8_t*)&SPIx->DR) = txBuffer[i];
		while((SPIx->SR & SPI_SR_RXNE ) != SPI_SR_RXNE); // Wait for RXNE (Receive buffer not empty)
		rxBuffer[i] = *((__IO uint8_t*)&SPIx->DR);
	}
	while( (SPIx->SR & SPI_SR_BSY) == SPI_SR_BSY ); // Wait for BSY flag cleared
}

void SPI_Read(SPI_TypeDef * SPIx, uint8_t *rxBuffer, int size) {
	int i = 0;
	for (i = 0; i < size; i++) {
		while( (SPIx->SR & SPI_SR_TXE ) != SPI_SR_TXE ); // Wait for TXE (Transmit buffer empty)
		*((volatile uint8_t*)&SPIx->DR) = rxBuffer[i];	
		// The clock is controlled by master. Thus the master has to send a byte
		// data to the slave to start the clock. 
		while((SPIx->SR & SPI_SR_RXNE ) != SPI_SR_RXNE); 
		rxBuffer[i] = *((__IO uint8_t*)&SPIx->DR);
	}
	while( (SPIx->SR & SPI_SR_BSY) == SPI_SR_BSY ); // Wait for BSY flag cleared
}
 

void SPI_Delay(uint32_t us) {
	uint32_t i, j;
	for (i = 0; i < us; i++) {
		for (j = 0; j < 18; j++) // This is an experimental value.
			(void)i;
	}
}

void SPIx_IRQHandler(SPI_TypeDef * SPIx, uint8_t *buffer, uint8_t *counter) {
	if(SPIx->SR & SPI_SR_RXNE) {        //	SPI Busy
		buffer[*counter] = SPIx->DR;   
		// Reading SPI_DR automatically clears the RXNE flag 
		(*counter)++;  
		if( (*counter) >= BufferSize )  {
			(*counter) = 0;
		}  
	}
}
