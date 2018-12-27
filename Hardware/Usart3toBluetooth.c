#include "Usart3toBluetooth.h"

unsigned char Tx_Buf_Uart3[160];
unsigned char Rx_Buf_Uart3[160];

int Flag_Tx_Uart3_Busy=0;

void Usart3toBluetooth_Init(u32 Bound){	
  
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	
  DMA_InitTypeDef DMA_InitStructure;   
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);  
  
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure);  
	
	
	DMA_DeInit(DMA1_Stream3);  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;   
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);  
	
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Tx_Buf_Uart3;  
	
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  
	  
	DMA_InitStructure.DMA_BufferSize = TX_LEN;  
	 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
	
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
	
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
	
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;  
	 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
	
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
		
	  
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;      
 
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;          

	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;         

	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   
         
	DMA_Init(DMA1_Stream3, &DMA_InitStructure);    

	DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);     


	DMA_DeInit(DMA1_Stream1);  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);  
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Rx_Buf_Uart3;   
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;   
	DMA_InitStructure.DMA_BufferSize = RX_LEN;   
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;   
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;   
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;    
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;    
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;  
  
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;       
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;           
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;           
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   
		
	DMA_Init(DMA1_Stream1, &DMA_InitStructure);    

	DMA_Cmd(DMA1_Stream1,ENABLE);  
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_USART3); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_USART3); 
	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = Bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
	USART_Init(USART3, &USART_InitStructure); 
	
	USART_ITConfig(USART3,USART_IT_TC,DISABLE);  
  USART_ITConfig(USART3,USART_IT_RXNE,DISABLE);  
  USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);  

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;				
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;				
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					
	NVIC_Init(&NVIC_InitStructure);	
	

	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  

	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);

	USART_ITConfig(USART3,USART_IT_TC,DISABLE);  
	USART_ITConfig(USART3,USART_IT_RXNE,DISABLE);  
	USART_ITConfig(USART3,USART_IT_TXE,DISABLE);  
	USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);  
	
	USART_Cmd(USART3, ENABLE);
	
}


void _Uart3_deal_irq_dma_tx(void)  
{  
    if(DMA_GetITStatus(DMA1_Stream3,DMA_IT_TCIF3) != RESET)   
    {  

        DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);  

        DMA_Cmd(DMA1_Stream3,DISABLE);  
			
        USART_ITConfig(USART3,USART_IT_TC,ENABLE);  
    }  
}  
  

  
uint8_t _Uart3_deal_irq_tx_end(void)  
{  
    if(USART_GetITStatus(USART3, USART_IT_TC) != RESET)  
    {  
        USART_ITConfig(USART3,USART_IT_TC,DISABLE);  
        Flag_Tx_Uart3_Busy = 0;      
        return 1;  
    }        
    return 0;  
}  

  
void Uart3_tx(uint8_t *data,uint16_t size)  
{   
    while (Flag_Tx_Uart3_Busy);  
    Flag_Tx_Uart3_Busy = 1;  
    memcpy(Tx_Buf_Uart3,data,size);   
    DMA_SetCurrDataCounter(DMA1_Stream3,size);   
    DMA_Cmd(DMA1_Stream3,ENABLE);  
}  



   
uint8_t _Uart3_deal_irq_rx_end(uint8_t *buf)  
{     
	uint16_t len = 0; 
	
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET) 
	{	
		USART3->SR;  
		USART3->DR;
	
		DMA_Cmd(DMA1_Stream1,DISABLE);
		DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1); 
 
		len = RX_LEN - DMA_GetCurrDataCounter(DMA1_Stream1);  
		memcpy(buf,Rx_Buf_Uart3,len);
			
		DMA_SetCurrDataCounter(DMA1_Stream1,RX_LEN);  

		DMA_Cmd(DMA1_Stream1,ENABLE);  

		return len;  
	}   		
	return 0;  
}  

void inf_Uart3_deal_irq_dma_tx(void)  
{  
    _Uart3_deal_irq_dma_tx();  
}  
  
uint8_t inf_Uart3_deal_irq_tx_end(void)  
{  
    return _Uart3_deal_irq_tx_end();  
}  
  
uint8_t inf_Uart3_deal_irq_rx_end(uint8_t *buf)  
{  
    return _Uart3_deal_irq_rx_end(buf);  
}

void Uart3_dma_tx_irq_handler(void) 
{  
    inf_Uart3_deal_irq_dma_tx();  
}  
      
void Uart3_irq_handler(void)                                
{     
	OS_ERR err; 
	Data_Rx Bluetooth_rx;
	Receive_GroundStation GroundStationData;
	inf_Uart3_deal_irq_tx_end();  
	Bluetooth_rx.len = inf_Uart3_deal_irq_rx_end(Bluetooth_rx.buf);
	memcpy(&GroundStationData,&Bluetooth_rx.buf,Bluetooth_rx.len);
	if (Bluetooth_rx.len != 0){ 
		//更新消息队列
		OSQPost(&messageQueue[GROUND_STATION],&GroundStationData,Bluetooth_rx.len,OS_OPT_POST_FIFO,&err);
	}				
} 


void DMA1_Stream3_IRQHandler(void)   
{  
		OSIntEnter();
		Uart3_dma_tx_irq_handler();
		OSIntExit();
}  
     

  
void USART3_IRQHandler(void)   
{  
		OSIntEnter();
    Uart3_irq_handler();
		OSIntExit();	
} 


