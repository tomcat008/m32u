#include "sys.h"
#include "usart.h"	  
#include "save_data.h"

////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif




//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
    if(para_value.debug_mode == 1)
    {
        while((USART2->SR&0X40)==0);//ѭ������,ֱ���������   
          USART2->DR = (u8) ch;      
    }
		else
		{
        while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
          USART1->DR = (u8) ch;   
		}
    
    return ch;
}
#endif 


/*ʹ��microLib�ķ���*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 

//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	

//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	  

void (*usart3_irq_callback) (u8 data); 

uint8_t USART1_TX_BUF[SENDBUFF_SIZE]; 
uint8_t USART2_TX_BUF[SENDBUFF_SIZE]; 


unsigned char USART1_RX_BUF[USART1_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
unsigned char USART2_RX_BUF[USART2_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.


//���ڷ��ͻ����� 	
unsigned char USART3_TX_BUF[USART3_MAX_SEND_LEN]; 	//���ͻ���,���USART2_MAX_SEND_LEN�ֽ�
								//���ʹ���˽���   	  
//���ڽ��ջ����� 	
unsigned char USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				//���ջ���,���USART2_MAX_RECV_LEN���ֽ�.



CIRCULATION_BUFF usart1_rx_cir_buff;
CIRCULATION_BUFF usart2_rx_cir_buff;




void USART_RX_DMA_Config(DMA_Channel_TypeDef* DMAy_Channelx, u32 p_addr, u32 m_addr, u16 len)
{
		DMA_InitTypeDef DMA_InitStructure;
	
	  if(((u32)DMAy_Channelx) < (AHBPERIPH_BASE + 0x0400))
		{
		    /*����DMAʱ��*/
		    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
		}
		else
		{
		    /*����DMAʱ��*/
		    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);	
		}

		/*����DMAԴ���������ݼĴ�����ַ*/
		DMA_InitStructure.DMA_PeripheralBaseAddr = p_addr;	   

		/*�ڴ��ַ(Ҫ����ı�����ָ��)*/
		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)m_addr;

		/*���򣺴����赽�ڴ�*/		
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	

		/*�����СDMA_BufferSize=SENDBUFF_SIZE*/	
		DMA_InitStructure.DMA_BufferSize = len;

		/*�����ַ����*/	    
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 

		/*�ڴ��ַ����*/
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	

		/*�������ݵ�λ*/	
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

		/*�ڴ����ݵ�λ 8bit*/
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 

		/*DMAģʽ������ѭ��*/
	  //DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	 

		/*���ȼ����߼�*/	
		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;  

		/*��ֹ�ڴ浽�ڴ�Ĵ���	*/
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

		/*����DMA1��4ͨ��*/		   
		DMA_Init(DMAy_Channelx, &DMA_InitStructure); 	  
		
		/*ʹ��DMA*/
		DMA_Cmd (DMAy_Channelx,ENABLE);		
}


void USART_TX_DMA_Config(DMA_Channel_TypeDef* DMAy_Channelx, u32 p_addr, u32 m_addr)
{
		DMA_InitTypeDef DMA_InitStructure;
		//NVIC_InitTypeDef NVIC_InitStructure; 
	
	  if(((u32)DMAy_Channelx) < (AHBPERIPH_BASE + 0x0400))
		{
		    /*����DMAʱ��*/
		    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
		}
		else
		{
		    /*����DMAʱ��*/
		    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);	
		}

		/*����DMAԴ���������ݼĴ�����ַ*/
		DMA_InitStructure.DMA_PeripheralBaseAddr = p_addr;	   

		/*�ڴ��ַ(Ҫ����ı�����ָ��)*/
		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)m_addr;

		/*���򣺴��ڴ浽����*/		
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	

		/*�����СDMA_BufferSize=SENDBUFF_SIZE*/	
		DMA_InitStructure.DMA_BufferSize = 0;

		/*�����ַ����*/	    
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 

		/*�ڴ��ַ����*/
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	

		/*�������ݵ�λ*/	
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

		/*�ڴ����ݵ�λ 8bit*/
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 

		/*DMAģʽ��������һ�ξ�ֹͣ*/
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;
		//DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	 

		/*���ȼ�����*/	
		DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  

		/*��ֹ�ڴ浽�ڴ�Ĵ���	*/
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

		/*����DMA1��4ͨ��*/		   
		DMA_Init(DMAy_Channelx, &DMA_InitStructure); 	  
		
		/*ʹ��DMA*/
		DMA_Cmd (DMAy_Channelx,ENABLE);		
}




  
void uart1_init(u32 bound)
{
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
  //USART1_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

  //Usart1 NVIC ����
  //NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�3
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	//NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
  //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	
	/* Write to DMAy Channelx CMAR */
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
	
  USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 
	
	
	usart1_rx_cir_buff.busy = 0;
	usart1_rx_cir_buff.remain = USART1_REC_LEN;
	usart1_rx_cir_buff.buff = USART1_RX_BUF;
	usart1_rx_cir_buff.rptr = USART1_RX_BUF;
	usart1_rx_cir_buff.wptr = USART1_RX_BUF;
	usart1_rx_cir_buff.len  = USART1_REC_LEN;
	
	USART_TX_DMA_Config(USART1_TX_DMA_CH, USART1_DR_Base, (u32)USART1_TX_BUF);
	USART_RX_DMA_Config(USART1_RX_DMA_CH, USART1_DR_Base, (u32)USART1_RX_BUF, USART1_REC_LEN);

}

void uart2_init(USART_PARA para)
{
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
	  USART_InitTypeDef USART_InitStructure;
	  //NVIC_InitTypeDef NVIC_InitStructure;
	   
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 
    
	  //USART2_TX   GPIOA.2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.2
     
    //USART2_RX	  GPIOA.3��ʼ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.3 
    
    //Usart2 NVIC ����
    //NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	  //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	  //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�3
	  //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	  //NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	  
    
    USART_InitStructure.USART_BaudRate = (para.baurd);//���ڲ�����
		
	  if(para.data_bits == 9)
	  {
	      USART_InitStructure.USART_WordLength = USART_WordLength_9b;  
	  }
	  else
	  {
	      USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
	  }
    
	  if(para.stop == 3)
	  {
	      USART_InitStructure.USART_StopBits = USART_StopBits_2;  
	  }
	  else if(para.stop == 2)
	  {
	      USART_InitStructure.USART_StopBits = USART_StopBits_1_5;  
	  }
	  else if(para.stop == 0)
	  {
	      USART_InitStructure.USART_StopBits = USART_StopBits_0_5;  
	  }
	  else
	  {
	      USART_InitStructure.USART_StopBits = USART_StopBits_1;  
	  }
    
	  if(para.parity == 2)
	  {
	      USART_InitStructure.USART_Parity = USART_Parity_Odd;  
	  }
	  else if(para.parity == 1)
	  {
	      USART_InitStructure.USART_Parity = USART_Parity_Even;  
	  }
	  else
	  {
	      USART_InitStructure.USART_Parity = USART_Parity_No;
	  }
    
	  if(para.flow_ctr == 2)
	  {
	      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;  
	  }
	  else if(para.flow_ctr == 1)
	  {
	      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;  
	  }
	  else
	  {
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
	  }
    
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  
    USART_Init(USART2, &USART_InitStructure);   
	  
	  /* Write to DMAy Channelx CMAR */
	  USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
	  USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	  
    USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���1 
	  
	  
	  usart2_rx_cir_buff.busy = 0;
	  usart2_rx_cir_buff.remain = USART2_REC_LEN;
	  usart2_rx_cir_buff.buff = USART2_RX_BUF;
	  usart2_rx_cir_buff.rptr = USART2_RX_BUF;
	  usart2_rx_cir_buff.wptr = USART2_RX_BUF;
	  usart2_rx_cir_buff.len  = USART2_REC_LEN;
	  
	  USART_TX_DMA_Config(USART2_TX_DMA_CH, USART2_DR_Base, (u32)USART2_TX_BUF);
	  USART_RX_DMA_Config(USART2_RX_DMA_CH, USART2_DR_Base, (u32)USART2_RX_BUF, USART2_REC_LEN);

}


void uart3_init(void)//���ڳ�ʼ������
{  

    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);// GPIOAʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
    
    USART_DeInit(USART3);                                //��λ����1
     //USART2_TX   PB10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;            //PA.2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	     //�����������
    GPIO_Init(GPIOB, &GPIO_InitStructure);               //��ʼ��PA2
     
    //USART2_RX	  PB11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;            //PA.3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOB, &GPIO_InitStructure);               //��ʼ��PA3
    
    USART_InitStructure.USART_BaudRate = 115200;                                    //һ������Ϊ9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;                            //����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	               //�շ�ģʽ
     
    USART_Init(USART3, &USART_InitStructure); //��ʼ������	3
    USART_Cmd(USART3, ENABLE);                                         //ʹ�ܴ��� 
    	
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�����ж�   
    
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
		
    USART3_RX_STA=0;		//����
	 
}	





#if 0
void USART1_IRQHandler(void)                	//����1�жϷ������
{
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
		{
		Res =USART_ReceiveData(USART1);	//��ȡ���յ�������
		
		if((USART_RX_STA&0x8000)==0)//����δ���
		{
			if(USART_RX_STA&0x4000)//���յ���0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else USART_RX_STA|=0x8000;	//��������� 
				}
			else //��û�յ�0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
					}		 
				}
			}   		 
     } 
}

#endif


u16 USART3_RX_STA=0;   	 


void USART3_IRQHandler(void)//STM32
{
    u8 res;	    
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//���յ�����
    {
        res =USART_ReceiveData(USART3);		
        if(usart3_irq_callback != NULL)
				{
				    (*usart3_irq_callback)(res);
				}					
        
    }  											 
}   



void usart_send_byte(USART_TypeDef * USARTx, u8 str)
{
    while((USARTx->SR&0X40)==0);
		USARTx->DR = str;  
}


void usart_send_str(USART_TypeDef * USARTx, u8 * str, u16 len)
{
    u16 i = 0;
    
	  for(i=0; i<len; i++)
	  {
		    while((USARTx->SR&0X40)==0);//ѭ������,ֱ���������  
        USARTx->DR = (u8) *str++; 				
		}
}




//����һ��DMA����
void MYDMA_Enable(DMA_Channel_TypeDef*DMA_CHx, unsigned char send_len)
{ 
	DMA_Cmd(DMA_CHx, DISABLE );  //�ر�USART1 TX DMA1 ��ָʾ��ͨ��      
 	DMA_SetCurrDataCounter(DMA_CHx, send_len);//DMAͨ����DMA����Ĵ�С
 	DMA_Cmd(DMA_CHx, ENABLE);  //ʹ��USART1 TX DMA1 ��ָʾ��ͨ�� 
}	  


/*******************************************************************************  
* Function Name  : USART1_dma_send  
* Description    : This function DMA send data.  
* Input          : DMAy_Channelx is dma cm, str is send content, len is long
* Output         : None  
* Return         : None  
*******************************************************************************/  
u16 USART_dma_send(DMA_Channel_TypeDef* DMAy_Channelx, u8 *str, u16 len)
{
    u16 i=0;
    
    if(len > SENDBUFF_SIZE)
    {
        return 1;
    }
    if(DMAy_Channelx == USART1_TX_DMA_CH)
		{
        for(i=0; i<len; i++)
        {
            USART1_TX_BUF[i] = *str++;
		    }
				while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	  }
		else if(DMAy_Channelx == USART2_TX_DMA_CH)
		{
        for(i=0; i<len; i++)
        {
            USART2_TX_BUF[i] = *str++;
		    }
				while((USART2->SR&0X40)==0);//ѭ������,ֱ���������   
	  }	
   
    while(DMA_GetCurrDataCounter(DMAy_Channelx));
    MYDMA_Enable(DMAy_Channelx, len);
	  return 0;
}


u16 circulation_buff_available(DMA_Channel_TypeDef* DMAy_Channelx, CIRCULATION_BUFF cir_buff)
{
	  u16 res = 0;
	  cir_buff.remain = DMA_GetCurrDataCounter(DMAy_Channelx);
	  cir_buff.wptr = cir_buff.buff + (cir_buff.len - cir_buff.remain);
    if(cir_buff.wptr >  cir_buff.rptr)
    {
			  res = (u32)cir_buff.wptr - (u32)cir_buff.rptr;
    }
    else if(cir_buff.wptr <  cir_buff.rptr)
    {
			  res = cir_buff.len - ((u32)cir_buff.rptr - (u32)cir_buff.wptr);
    }
		else
		{
		    res = 0;
		}
		return res;
}

int circulation_buff_read(DMA_Channel_TypeDef* DMAy_Channelx, CIRCULATION_BUFF* cir_buff,unsigned char* read_buff,unsigned int len)
{
	  unsigned int i = 0,rlen = 0;
	
	  cir_buff->remain = DMA_GetCurrDataCounter(DMAy_Channelx);
	  cir_buff->wptr = cir_buff->buff + (cir_buff->len - cir_buff->remain);
	  if(cir_buff->busy == 0)
		{
		    cir_buff->busy = 1;
        for(i=0; i< len; i++)
	      {
			      //if(circulation_buff_available(cir_buff))  
					  if(cir_buff->rptr !=  cir_buff->wptr)
				    {
				        read_buff[i] = *cir_buff->rptr++;
							  rlen++;
							  
		            if(cir_buff->rptr == (cir_buff->buff+cir_buff->len))
		            {
		                cir_buff->rptr = cir_buff->buff;
		            }
				    }
						else
						{
							  cir_buff->busy = 0;
						    return rlen;
						}
		    }
		    cir_buff->busy = 0;
			  return rlen;

		}
		else
		{
		    return CIRCULATION_BUFF_BUSY;
		}

}





