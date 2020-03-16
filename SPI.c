#include "spi.h"
#include "stm32l476xx.h"

#include <stdbool.h>

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

void SPI1_GPIO_Init(){
	// Configure PE[13, 15] to desired settings

	// Enable clock for PE[13, 15]
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
	
	// Set all GPIO pins to alternative function mode (AF5)
	GPIOE->MODER &= ~(GPIO_MODER_MODE13);
	GPIOE->MODER &= ~(GPIO_MODER_MODE14);
	GPIOE->MODER &= ~(GPIO_MODER_MODE15);
	GPIOE->MODER |= GPIO_MODER_MODE13_1;
	GPIOE->MODER |= GPIO_MODER_MODE14_1;
	GPIOE->MODER |= GPIO_MODER_MODE15_1;
	
	GPIOE->AFR[1] &= ~(GPIO_AFRH_AFSEL13);
	GPIOE->AFR[1] |= GPIO_AFRH_AFSEL13_0;
	GPIOE->AFR[1] |= GPIO_AFRH_AFSEL13_2;
	
	GPIOE->AFR[1] &= ~(GPIO_AFRH_AFSEL14);
	GPIOE->AFR[1] |= GPIO_AFRH_AFSEL14_0;
	GPIOE->AFR[1] |= GPIO_AFRH_AFSEL14_2;
	
	GPIOE->AFR[1] &= ~(GPIO_AFRH_AFSEL15);
	GPIOE->AFR[1] |= GPIO_AFRH_AFSEL15_0;
	GPIOE->AFR[1] |= GPIO_AFRH_AFSEL15_2;
	

	// Set all pins to very high speed
	GPIOE->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED13);
	GPIOE->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED14);
	GPIOE->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED15);
	GPIOE->OSPEEDR |= GPIO_OSPEEDR_OSPEED13;
	GPIOE->OSPEEDR |= GPIO_OSPEEDR_OSPEED14;
	GPIOE->OSPEEDR |= GPIO_OSPEEDR_OSPEED15;

	// Set all pins to have a push-pull output type
	GPIOE->OTYPER &= ~(GPIO_OTYPER_OT13);
	GPIOE->OTYPER &= ~(GPIO_OTYPER_OT14);
	GPIOE->OTYPER &= ~(GPIO_OTYPER_OT15);

	// Configure all GPIO pins to no pull-up, pull-down
	GPIOE->PUPDR &= ~(GPIO_PUPDR_PUPD13);
	GPIOE->PUPDR &= ~(GPIO_PUPDR_PUPD14);
	GPIOE->PUPDR &= ~(GPIO_PUPDR_PUPD15);
}

void SPI_Init(void){
	SPI1->CR1 &= ~(SPI_CR1_SPE);
	
	// Enable SPI1 clock in peripheral clock register
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	// Reset SPI1 by setting appropriate bits in peripheral reset register
	RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;

	// Clear bits so SPI1 does not remain in a reset state
	RCC->APB2RSTR &= ~(RCC_APB2RSTR_SPI1RST);

	// Configure the serial channel for full duplex communication
	SPI1->CR1 &= ~(SPI_CR1_RXONLY);

	// Configure the communication for 2-line unidirectional data mode
	SPI1->CR1 &= ~(SPI_CR1_BIDIMODE);

	// Disable output in bidirectional mode
	//		CHANGE TO: Enable output in bidirectional mode
	SPI1->CR1 |= SPI_CR1_BIDIOE;	

	// Set: 
	//		the data frame for receiving the MSB first when data is transmitted
	SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);

	// 		the data length to 8 bits
	SPI1->CR2 &= ~(SPI_CR2_DS);
	SPI1->CR2 |= SPI_CR2_DS;
	SPI1->CR2 &= ~(SPI_CR2_DS_3);

	//		the frame format to be in SPI Motorola mode
	SPI1->CR2 &= ~(SPI_CR2_FRF);

	// Set: 
	//		the clock polarity to 0
	SPI1->CR1 &= ~(SPI_CR1_CPOL);

	//		the clock phase s.t. the first clock transition is the first data capture edge
	SPI1->CR1 &= ~(SPI_CR1_CPHA);

	// Set the baud rate prescaler to 16
	//		CHANGE TO: Set the baud rate prescaler to 8
	SPI1->CR1 &= ~(SPI_CR1_BR);
	//SPI1->CR1 |= SPI_CR1_BR;
	//SPI1->CR1 &= ~(SPI_CR1_BR_1);
	SPI1->CR1 |= SPI_CR1_BR_1;

	// Disable CRC calculation
	//		CHANGE TO: Enable CRC calculation
	SPI1->CR1 |= (SPI_CR1_CRCEN);

	// Set: 
	//		the board to operate in master mode
	SPI1->CR1 |= SPI_CR1_MSTR;

	//		enable software slave management
	//		CHANGE TO: disable software slave management
	SPI1->CR1 &= ~(SPI_CR1_SSM);

	//		ADD: SSOE = 1 (SSM = 0)
	SPI1->CR2 |= SPI_CR2_SSOE;

	//		enable NSS pulse management
	SPI1->CR2 |= SPI_CR2_NSSP;

	// Disable internal slave bit
	SPI1->CR1 &= ~(SPI_CR1_SSI);

	// Set the FIFO reception threshold to 1/4 (8 bit)
	SPI1->CR2 |= SPI_CR2_FRXTH;

	/*

	Configure CRCL, CRCEN if CRC is needed
	Write to SPI_CRCPR register if needed
		Read about smaller details of CRC: what to do if SPI is off, etc..

		The CRC polynomial (0007h) is the reset value of this register. Another polynomial can be configured as required.

		What does CRC_Poly7 mean??

	SPI1->CR2 |= SPI_CR2_RXDMAEN;
	SPI1->CR2 |= SPI_CR2_TXDMAEN;

	*/

	// Enable SPI 
	SPI1->CR1 |= SPI_CR1_SPE;
}

/*
	Channel 2, request num 1: SPI1_RX
	Channel 3, request num 1: SPI1_TX
	Channel 4, request num 2: USART1_TX
*/
void DMA_config(){
	// Set peripheral register address in DMA->CPARx register (!!!!!)
	/*
	DMA->InitStructure.DMA->PeripheralBaseAddr = (u32)&SPI2->DR;
  	DMA->InitStructure.DMA->PeripheralBaseAddr = (u32)&USART2->DR;
  	*/

	DMA1_Channel2->CPAR = (u32)(SPI1_BASE + 0x0C);
	DMA1_Channel3->CPAR = (u32)(SPI1_BASE + 0x0C);
	DMA1_Channel4->CPAR = (u32)(USART1_BASE + 0x28);

	// Set memory address in DMA->CMARx register
	// Configure total amount of memory to be transferred in DMA->CNDTRx register
	// 			see DMA1_RX, DMA1_SendtoUsart

	// Configure channel priority using PL[1:0] in DMA->CCRx 
	DMA1_Channel2->CCR |= DMA_CCR_PL;
	DMA1_Channel2->CCR &= ~(DMA_CCR_PL_0);

	DMA1_Channel3->CCR |= DMA_CCR_PL;
	DMA1_Channel3->CCR &= ~(DMA_CCR_PL_0);

	DMA1_Channel4->CCR |= DMA_CCR_PL;

	// Configure:
	//		M2M mode
	DMA1_Channel2->CCR &= ~(DMA_CCR_MEM2MEM);
	DMA1_Channel3->CCR &= ~(DMA_CCR_MEM2MEM);
	DMA1_Channel4->CCR &= ~(DMA_CCR_MEM2MEM);

	//		data transfer direction
	DMA1_Channel2->CCR &= ~(DMA_CCR_DIR);
	DMA1_Channel3->CCR |= DMA_CCR_DIR;
	DMA1_Channel4->CCR |= DMA_CCR_DIR;

	//		circular mode
	DMA1_Channel2->CCR &= ~(DMA_CCR_CIRC);
	DMA1_Channel3->CCR &= ~(DMA_CCR_CIRC);
	DMA1_Channel4->CCR &= ~(DMA_CCR_CIRC);


	//		peripheral and memory incremented mode
	DMA1_Channel2->CCR |= DMA_CCR_MINC;
	DMA1_Channel2->CCR &= ~(DMA_CCR_PINC);

	DMA1_Channel3->CCR |= DMA_CCR_MINC;
	DMA1_Channel3->CCR &= ~(DMA_CCR_PINC);

	DMA1_Channel4->CCR |= DMA_CCR_MINC;
	DMA1_Channel4->CCR &= ~(DMA_CCR_PINC);

	//		peripheral data size
	DMA1_Channel2->CCR |= DMA_CCR_PSIZE;
	DMA1_Channel2->CCR &= ~(DMA_CCR_PSIZE_0);

	DMA1_Channel3->CCR |= DMA_CCR_PSIZE;
	DMA1_Channel3->CCR &= ~(DMA_CCR_PSIZE_0);

	DMA1_Channel4->CCR |= DMA_CCR_PSIZE;
	DMA1_Channel4->CCR &= ~(DMA_CCR_PSIZE_0);

	// 		memory data size
	DMA1_Channel2->CCR |= DMA_CCR_MSIZE;
	DMA1_Channel2->CCR &= ~(DMA_CCR_MSIZE_0);

	DMA1_Channel3->CCR |= DMA_CCR_MSIZE;
	DMA1_Channel3->CCR &= ~(DMA_CCR_MSIZE_0);

	DMA1_Channel4->CCR |= DMA_CCR_MSIZE;
	DMA1_Channel4->CCR &= ~(DMA_CCR_MSIZE_0);

	//		interrupt after half/full transfer in DMA->CCRx register
	DMA1_Channel2->CCR |= DMA_CCR_TCIE;
	DMA1_Channel3->CCR |= DMA_CCR_TCIE;  
	DMA1_Channel4->CCR |= DMA_CCR_TCIE;
}

void NVIC_Config(){
  	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	NVIC_SetPriority(DMA1_Channel2_IRQn, 0);

  	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	NVIC_SetPriority(DMA1_Channel4_IRQn, 0);
}

void SPI1_Init(void){
	//RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
	SPI1_GPIO_Init();
	ArduCAM_CS_init();
	SPI_Init();

	SPI1_ReadWriteByte(0xff);

	//DMA_config();
	//NVIC_Config();
}

u8 SPI1_ReadWriteByte(u8 TxData){
	uint8_t rxBuffer;
	SPI_Write(SPI1, &TxData, rxBuffer, 1);
	SPI_Read(SPI1, rxBuffer, 1);
	return rxBuffer;
}


/*
	Only 16 bits (65535) can be transferred at a time; maybe look into circular mode?

	"If the channel is configured in non-circular mode, no DMA request is served after the last transfer
	(that is once the num of data items to be transferred has reached zero). In order to reload a new number
	of data items to be transferred into the DMA->CNDTRx register, the DMA channel must be disabled".

	"In circular mode, after the last transfer, the DMA->CNDTRx register is automatically reloaded with the initially
	programmed value. The current internal address registers are reloaded with the base address values from the 
	DMA->CPARx/DMA->CMARx registers".
*/

void DMA1_RX(uint8_t *p , uint32_t len){		
  	CS_LOW();
	set_fifo_burst();

	if (len > 65535){
		printf("DMA1RX: DMA->CNDTR only allows values up to 65535. len: %p:\n", len);
		return;
	}

	/*
	STM32 manual:
		when MSIZE = 10 (32 bit), MA[1:0] bits are ignored.
		Do I left shift address by 2?
	*/
	
	DMA1_Channel2->CMAR = (u32)p << 2;
	DMA1_Channel2->CNDTR = (u16)len;
	DMA1_Channel3->CMAR = (u32)p << 2;
	DMA1_Channel3->CNDTR = (u16)len;

	/*

	typedef struct{
  		__IO uint32_t CSELR;
	} DMA_Request_TypeDef;

	DMA1_CSELR &= ~(DMA_CSELR_C2S | DMA_CSELR_C3S | DMA_CSELR_C4S);
	DMA1_CSELR |= (1 << 4 | 1 << 8 | 1 << 13);
	
	*/

	// Active channel by setting ENABLE bit in DMA->CCRx register
	DMA1_Channel2->CCR |= DMA_CCR_EN;
	DMA1_Channel3->CCR |= DMA_CCR_EN;
}

void DMA1_SendtoUsart(uint8_t *p , uint32_t len){
	if (len > 65535){
		printf("DMA1RX: DMA->CNDTR only allows values up to 65535. len: %p:\n", len);
		return;
	}

	DMA1_Channel4->CMAR = (u32)p << 2;
	DMA1_Channel4->CNDTR = (u16)len;

	DMA1_Channel4->CCR |= DMA_CCR_EN;
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

void DMA1_Channel2_IRQHandler(void){ 	
	/*
	if(DMA->GetITStatus(DMA1_IT_TC4)){
		DMA->ClearITPendingBit(DMA1_IT_GL4 | DMA1_IT_TC4 | DMA1_IT_GL5 | DMA1_IT_TC5);

		DMA1_Channel4->CCR &= ~(DMA_CCR_EN);
		DMA1_Channel5->CCR &= ~(DMA_CCR_EN);

		// SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, DISABLE);
		CS_HIGH();
		receive_OK =1;
	}
	*/
}


void DMA1_Channel4_IRQHandler(void){
	/*
	if(DMA->GetITStatus(DMA1_IT_TC7)){
		DMA->ClearITPendingBit(DMA1_IT_GL7 | DMA1_IT_TC7);
		DMA->CCR6 &= ~(DMA->CCR_EN);
	}
	*/
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
