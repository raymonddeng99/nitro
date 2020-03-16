#include "ArduCAM.h"
#include "spi.h"
#include "sccb_bus.h"
#include "usart.h"
#include "ov2640_regs.h"

byte sensor_model = 0;
byte sensor_addr = 0;
byte m_fmt = JPEG;
uint32_t length = 0;
uint8_t is_header= false;

void ArduCAM_Init(byte model){
	wrSensorReg8_8(0xff, 0x01);
	wrSensorReg8_8(0x12, 0x80);

	wrSensorRegs8_8(OV2640_JPEG_INIT);
	wrSensorRegs8_8(OV2640_YUV422);
	wrSensorRegs8_8(OV2640_JPEG);
	wrSensorReg8_8(0xff, 0x01);
	wrSensorReg8_8(0x15, 0x00);
	wrSensorRegs8_8(OV2640_320x240_JPEG);
}

void ArduCAM_CS_init(void){
	// Enable GPIO Clocks
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
	
	
	// Set PE12 to output mode
	GPIOE->MODER &= ~GPIO_MODER_MODE12;
	GPIOE->MODER |= GPIO_MODER_MODE12_0;

	//Configure PE12 output speed to very fast
	GPIOE->OSPEEDR |= GPIO_OSPEEDR_OSPEED12;
	
	//Configure PE12 output type as Push-Pull
	GPIOE->OTYPER &= ~GPIO_OTYPER_OT12;
	
	//Configure PE12 output type as No Pull-Up, No Pull-Down
	GPIOE->PUPDR &= ~GPIO_PUPDR_PUPD12;

	CS_HIGH();
}

// CS_PORT = E
// CS_PIN = 12
void CS_HIGH(void){
	GPIOE->ODR |= GPIO_ODR_OD12;		
}

void CS_LOW(void){
	GPIOE->ODR &= ~(GPIO_ODR_OD12);			    
}

void set_format(byte fmt){
	m_fmt = (fmt == BMP)? BMP : JPEG;
}

uint8_t bus_read(int address){
	uint8_t value;
	CS_LOW();
	SPI1_ReadWriteByte(address);
	value = SPI1_ReadWriteByte(0x00);
	CS_HIGH();
	return value;
}

uint8_t bus_write(int address,int value){	
	CS_LOW();
	delay_us(10);
	SPI1_ReadWriteByte(address);
	SPI1_ReadWriteByte(value);
	delay_us(10);
	CS_HIGH();
	return 1;
}

uint8_t read_reg(uint8_t addr){
	uint8_t data = bus_read(addr & 0x7F);
	return data;
}

void write_reg(uint8_t addr, uint8_t data){
	 bus_write(addr | 0x80, data); 
}

uint8_t read_fifo(void){
	uint8_t data;
	data = bus_read(SINGLE_FIFO_READ);
	return data;
}

void set_fifo_burst(){
	SPI1_ReadWriteByte(BURST_FIFO_READ);
}

void flush_fifo(void){
	write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void start_capture(void){
	write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

void clear_fifo_flag(void ){
	write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

uint32_t read_fifo_length(void){
	uint32_t len1,len2,len3,len=0;
	len1 = read_reg(FIFO_SIZE1);
  	len2 = read_reg(FIFO_SIZE2);
  	len3 = read_reg(FIFO_SIZE3) & 0x7f;
  	len = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
		return len;	
}

//Set corresponding bit  
void set_bit(uint8_t addr, uint8_t bit){
	uint8_t temp;
	temp = read_reg(addr);
	write_reg(addr, temp | bit);
}

//Clear corresponding bit 
void clear_bit(uint8_t addr, uint8_t bit){
	uint8_t temp;
	temp = read_reg(addr);
	write_reg(addr, temp & (~bit));
}

//Get corresponding bit status
uint8_t get_bit(uint8_t addr, uint8_t bit){
  uint8_t temp;
  temp = read_reg(addr);
  temp = temp & bit;
  return temp;
}

//Set ArduCAM working mode
//MCU2LCD_MODE: MCU writes the LCD screen GRAM
//CAM2LCD_MODE: Camera takes control of the LCD screen
//LCD2MCU_MODE: MCU read the LCD screen GRAM
void set_mode(uint8_t mode){
  switch (mode){
    case MCU2LCD_MODE:
      write_reg(ARDUCHIP_MODE, MCU2LCD_MODE);
      break;
    case CAM2LCD_MODE:
      write_reg(ARDUCHIP_MODE, CAM2LCD_MODE);
      break;
    case LCD2MCU_MODE:
      write_reg(ARDUCHIP_MODE, LCD2MCU_MODE);
      break;
    default:
      write_reg(ARDUCHIP_MODE, MCU2LCD_MODE);
      break;
  }
}

byte wrSensorReg8_8(int regID, int regDat){
	/*
	delay_us(5);
	sccb_bus_start();                          
	if(sccb_bus_write_byte(sensor_addr) == 0){
		sccb_bus_stop();                        
		return 1;
	}
	delay_us(5);
	if(sccb_bus_write_byte(regID) == 0){
		sccb_bus_stop();                              
		return 2;                                       
	}
	delay_us(5);
	if(sccb_bus_write_byte(regDat)==0){
		sccb_bus_stop();                                 
		return 3;
	}
	sccb_bus_stop();                                    
	return 0;
	*/
}


byte rdSensorReg8_8(uint8_t regID, uint8_t* regDat){
	/*
	delay_us(10);
	
	sccb_bus_start();
	if(sccb_bus_write_byte(sensor_addr) == 0)                 
	{
		sccb_bus_stop();                                
		//goto start;
		return 1;                                        
	}
	delay_us(10);
	if(sccb_bus_write_byte(regID)==0)//ID
	{
		sccb_bus_stop();                                  
		//goto start;
		return 2;                                       
	}
	sccb_bus_stop();                                   
	delay_us(10);	
	sccb_bus_start();
	if(sccb_bus_write_byte(sensor_addr|0x01)==0)                    
	{
		sccb_bus_stop();                                   
		//goto start;
		return 3;                                          
	}
	delay_us(10);
	*regDat = sccb_bus_read_byte();                    
	sccb_bus_send_noack();                                
	sccb_bus_stop();                                      
	return 0;    
*/	
}

//I2C Array Write 8bit address, 8bit data
int wrSensorRegs8_8(const struct sensor_reg reglist[]){
	/*
  int err = 0;
  uint16_t reg_addr = 0;
  uint16_t reg_val = 0;
  const struct sensor_reg *next = reglist;
  while ((reg_addr != 0xff) | (reg_val != 0xff))
  {
    reg_addr = next->reg;
    reg_val = next->val;
    err = wrSensorReg8_8(reg_addr, reg_val);
 //   delay_us(400);
    next++;
  }

  return err;
	*/
}

byte wrSensorReg16_8(int regID, int regDat){
	/*
	sccb_bus_start();
	if(0==sccb_bus_write_byte(sensor_addr))
	{
		sccb_bus_stop();
		return(0);
	}
	delay_us(5);
  if(0==sccb_bus_write_byte(regID>>8))
	{
		sccb_bus_stop();
		return(0);
	}
	delay_us(5);
  if(0==sccb_bus_write_byte(regID))
	{
		sccb_bus_stop();
		return(0);
	}
	delay_us(5);
  if(0==sccb_bus_write_byte(regDat))
	{
		sccb_bus_stop();
		return(0);
	}
  sccb_bus_stop();
	
  return(1);
	*/
}

int wrSensorRegs16_8(const struct sensor_reg reglist[]){
	/*
  int err = 0;

  unsigned int reg_addr;
  unsigned char reg_val;
  const struct sensor_reg *next = reglist;

  while ((reg_addr != 0xffff) | (reg_val != 0xff))
  {
    reg_addr =next->reg;
    reg_val = next->val;
    err = wrSensorReg16_8(reg_addr, reg_val);
    delay_us(600);
    next++;
  }
  return err;
	*/
}


byte rdSensorReg16_8(uint16_t regID, uint8_t* regDat){
	/*
	sccb_bus_start();                  
	if(0==sccb_bus_write_byte(0x78))
	{
		sccb_bus_stop();
		return(0);
	}
	delay_us(20);
	delay_us(20);
  if(0==sccb_bus_write_byte(regID>>8))
	{
		sccb_bus_stop();
		return(0);
	}
	delay_us(20);
  if(0==sccb_bus_write_byte(regID))
	{
		sccb_bus_stop();
		return(0);
	}
	delay_us(20);
	sccb_bus_stop();
	
	delay_us(20);
	
	
	sccb_bus_start();                 
	if(0==sccb_bus_write_byte(0x79))
	{
		sccb_bus_stop();
		return(0);
	}
	delay_us(20);
  *regDat=sccb_bus_read_byte();
  sccb_bus_send_noack();
  sccb_bus_stop();
  return(1);
	*/
}