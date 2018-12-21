/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * 作者    :Xiluna Tech
 * 文件名  :SPI.c
 * 描述    :SPI驱动
 * 官网    :http://xiluna.com/
 * 公总号  :XilunaTech
**********************************************************************************/
#include "SPI1.h"

void SPI1_Configuration(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	/* SCK, MISO and MOSI  PA5=CLK,PA6=MISO,PA7=MOSI*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);
	/*  PB11  */
	GPIO_SetBits(GPIOB, GPIO_Pin_11);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* SPI1 configuration  */
	SPI_Cmd(SPI1, DISABLE);        
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;      
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;     
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;       
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;    
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;      
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;  

	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;     
	SPI_InitStructure.SPI_CRCPolynomial = 7;       
    
	SPI_Init(SPI1, &SPI_InitStructure);	 
	SPI_Cmd(SPI1, ENABLE);
}

uint8_t SPI1_ReadWrite_Byte(uint8_t byte)
{
	
	while((SPI1->SR & SPI_I2S_FLAG_TXE)==RESET);
	SPI1->DR = byte;  
	
	while((SPI1->SR & SPI_I2S_FLAG_RXNE)==RESET);
	return(SPI1->DR);
}


void SPI1_writeReg(u8 reg ,u8 data){
	SPI1_ReadWrite_Byte(reg);
	SPI1_ReadWrite_Byte(data);
}


u8 SPI1_readReg(u8 reg){
	SPI1_ReadWrite_Byte(reg|0x80);
	return SPI1_ReadWrite_Byte(0xff);
}


void SPI1_readRegs(u8 reg, u8 length, u8 *data){
	u8 count = 0;
	SPI1_ReadWrite_Byte(reg|0x80);
	for(count=0;count<length;count++){
		data[count] = SPI1_ReadWrite_Byte(0xff);
	}
}

