#ifndef __STM32L476G_DISCOVERY_SPI_H
#define __STM32L476G_DISCOVERY_SPI_H

#include "stm32l476xx.h"
#include "main.h"
#define  BUFFER_MAX_SIZE 4096
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

extern uint32_t sendlen ; 
extern uint32_t haveRev ;
extern uint32_t noRev;
extern uint8_t *picbuf	;
extern bool receive_OK;	
extern bool send_OK ; 
extern uint8_t	Buf1[BUFFER_MAX_SIZE];
extern uint8_t	Buf2[BUFFER_MAX_SIZE];
extern uint8_t  EP2_SendFinish;

#define BufferSize 32

void SPI1_GPIO_Init(void);
void SPI_Init(void);

void DMA_config();
void NVIC_Config();
void SPI1_Init(void);		

u8 SPI1_ReadWriteByte(u8 TxData);
void	DMA1_RX(uint8_t *p , uint32_t len);
void	DMA1_SendtoUsart(uint8_t *p , uint32_t len);

void	SendbyUSART1( void);
void SingleCapTransfer(void);

void DMA4_Channel1_IRQHandler(void);
void DMA6_Channel2_IRQHandler(void);


void SPI_Write(SPI_TypeDef * SPIx, uint8_t *txBuffer, uint8_t * rxBuffer, int size);
void SPI_Read(SPI_TypeDef * SPIx, uint8_t *rxBuffer, int size);
void SPI_Delay(uint32_t us);
void SPIx_IRQHandler(SPI_TypeDef * SPIx, uint8_t *buffer, uint8_t *counter);

#endif 
