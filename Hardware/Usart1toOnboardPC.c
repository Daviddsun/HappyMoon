#include "Usart1toOnboardPC.h"

unsigned char Tx_Buf_Uart1[160];
unsigned char Rx_Buf_Uart1[64];

int Flag_Tx_Uart1_Busy=0;

void Usart1toOnboardPC_Init(u32 Bound)
{
	//GPIO管脚配置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	
  DMA_InitTypeDef DMA_InitStructure;   
  
  RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  
  
	//DMA中断配置
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure);  
	
	//DMA
	DMA_DeInit(DMA2_Stream7);  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;   

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);  

	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Tx_Buf_Uart1;  

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

          
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);    

	DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);     

	//DMA
	DMA_DeInit(DMA2_Stream5);  
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);  
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Rx_Buf_Uart1;   
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
		
    
	DMA_Init(DMA2_Stream5, &DMA_InitStructure);    

	DMA_Cmd(DMA2_Stream5,ENABLE);  
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);		 //GPIOA
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);		 //USART1

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); 	//GPIOA9 USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); 	//GPIOA10 USART1
	
	//USART1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 ; 	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;							
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;					
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 						
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 							
	GPIO_Init(GPIOA,&GPIO_InitStructure); 										

   //USART1 
	USART_InitStructure.USART_BaudRate = Bound;								
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;			
	USART_InitStructure.USART_StopBits = USART_StopBits_1;					
	USART_InitStructure.USART_Parity = USART_Parity_No;							
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
	USART_Init(USART1, &USART_InitStructure); 											
	
	
	USART_ITConfig(USART1,USART_IT_TC,DISABLE);  
  USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);  
  USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);  

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	
	
 
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  

	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);

	USART_ITConfig(USART1,USART_IT_TC,DISABLE);  
	USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);  
	USART_ITConfig(USART1,USART_IT_TXE,DISABLE);  
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);  
	

	USART_Cmd(USART1, ENABLE);					
}


  
void _Uart1_deal_irq_dma_tx(void)  
{  
    if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7) != RESET)   
    {  

        DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);  

        DMA_Cmd(DMA2_Stream7,DISABLE);  

        USART_ITConfig(USART1,USART_IT_TC,ENABLE);  
    }  
}  
  
  
uint8_t _Uart1_deal_irq_tx_end(void)  
{  
    if(USART_GetITStatus(USART1, USART_IT_TC) != RESET)  
    {  
        USART_ITConfig(USART1,USART_IT_TC,DISABLE);  
        Flag_Tx_Uart1_Busy = 0;  
          
        return 1;  
    }        
    return 0;  
}  


  
void Uart1_tx(uint8_t *data,uint16_t size)  
{  
    while (Flag_Tx_Uart1_Busy);  
    Flag_Tx_Uart1_Busy = 1;  
    memcpy(Tx_Buf_Uart1,data,size);  
    DMA_SetCurrDataCounter(DMA2_Stream7,size);  
    DMA_Cmd(DMA2_Stream7,ENABLE);  
}  



uint8_t _Uart1_deal_irq_rx_end(uint8_t *buf)  
{     
    uint16_t len = 0;  
      
    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  
    {  
        USART1->SR;  
        USART1->DR; 
        DMA_Cmd(DMA2_Stream5,DISABLE);   
        DMA_ClearFlag(DMA2_Stream5,DMA_FLAG_TCIF5);   
        len = RX_LEN - DMA_GetCurrDataCounter(DMA2_Stream5);  
        memcpy(buf,Rx_Buf_Uart1,len);  
          
        DMA_SetCurrDataCounter(DMA2_Stream5,RX_LEN);  

        DMA_Cmd(DMA2_Stream5,ENABLE);  
  
        return len;  
    }   
      
    return 0;  
}  


  
void inf_Uart1_deal_irq_dma_tx(void)  
{  
    _Uart1_deal_irq_dma_tx();  
}  

  
uint8_t inf_Uart1_deal_irq_tx_end(void)  
{  
    return _Uart1_deal_irq_tx_end();  
}  
  
  
uint8_t inf_Uart1_deal_irq_rx_end(uint8_t *buf)  
{  
    return _Uart1_deal_irq_rx_end(buf);  
}

  
void Uart1_dma_tx_irq_handler(void)  
{  
    inf_Uart1_deal_irq_dma_tx();  
}  
Data_Rx OnboardPC;
void Uart1_irq_handler(void)                                
{  
		OS_ERR err;	
    inf_Uart1_deal_irq_tx_end();  
    OnboardPC.len = inf_Uart1_deal_irq_rx_end(OnboardPC.buf);
    if (OnboardPC.len != 0)  
    { 
			//消息队列
			OSQPost(&messageQueue[VISUAL_ODOMETRY],&OnboardPC.buf,OnboardPC.len,OS_OPT_POST_FIFO,&err);
		}	
} 



  
void DMA2_Stream7_IRQHandler(void)  
{  	
		OSIntEnter();
    Uart1_dma_tx_irq_handler(); 
		OSIntExit();	
}  
      
  
void USART1_IRQHandler(void)   
{  
		OSIntEnter();
    Uart1_irq_handler(); 
		OSIntExit();
}  



