#include "SPI.h"
#include "SysTimer.h"

// Note: When the data frame size is 8 bit, "SPIx->DR = byte_data;" works incorrectly. 
// It mistakenly send two bytes out because SPIx->DR has 16 bits. To solve the program,
// we should use "*((volatile uint8_t*)&SPIx->DR) = byte_data";


// LSM303C eCompass (a 3D accelerometer and 3D magnetometer module) SPI Interface: 
//   MAG_DRDY = PC2   MEMS_SCK  = PD1 (SPI2_SCK)   XL_CS  = PE0             
//   MAG_CS   = PC0   MEMS_MOSI = PD4 (SPI2_MOSI)  XL_INT = PE1       
//   MAG_INT  = PC1 
//
// L3GD20 Gyro (three-axis digital output) SPI Interface: 
//   MEMS_SCK  = PD1 (SPI2_SCK)    GYRO_CS   = PD7 (GPIO)
//   MEMS_MOSI = PD4 (SPI2_MOSI)   GYRO_INT1 = PD2
//   MEMS_MISO = PD3 (SPI2_MISO)   GYRO_INT2 = PB8

extern uint8_t Rx1_Counter;
extern uint8_t Rx2_Counter;

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

	// Disable CRC calculation
	SPI1->CR1 &= ~(SPI_CR1_CRCEN);

	// Set: 
	//		the board to operate in slave mode
	SPI1->CR1 &= ~(SPI_CR1_MSTR);

	//		enable software slave management
	//		CHANGE TO: disable software slave management
	SPI1->CR1 &= ~(SPI_CR1_SSM);

	//		enable NSS pulse management
	SPI1->CR2 |= SPI_CR2_NSSP;

	// Disable internal slave bit
	SPI1->CR1 &= ~(SPI_CR1_SSI);

	// Set the FIFO reception threshold to 1/4 (8 bit)
	SPI1->CR2 |= SPI_CR2_FRXTH;

	// Enable SPI 
	SPI1->CR1 |= SPI_CR1_SPE;
}

void SPI1_CS_Init(){
	// Clock already enabled for E pins
	
	// Set PE12 to AF mode
	GPIOE->MODER &= ~(GPIO_MODER_MODE12);
	//GPIOE->MODER |= GPIO_MODER_MODE12_0;
	GPIOE->MODER |= GPIO_MODER_MODE12_1;
	
	// Set PE12 to very high speed
	GPIOE->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED12);
	GPIOE->OSPEEDR |= GPIO_OSPEEDR_OSPEED12;
	
	// Set PE12 to push-pull output type
	GPIOE->OTYPER &= ~(GPIO_OTYPER_OT12);
	
	// Configure PE12 to no pull-up, pull-down
	GPIOE->PUPDR &= ~(GPIO_PUPDR_PUPD12);
}

void SPI1_Init(){
	SPI1_GPIO_Init();
	SPI_Init();
	SPI1_CS_Init();
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
