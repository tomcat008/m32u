#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

#define DEBUG(...)                          printf(__VA_ARGS__)


#define CIRCULATION_BUFF_BUSY                -1
#define CIRCULATION_BUFF_INSUFFICIENT        -2



#define USART1_REC_LEN  			               512  	//�����������ֽ��� 200
#define USART2_REC_LEN  			               512  	//�����������ֽ��� 200
	                                           
#define USART1_DR_Base                       0x40013804		// 0x40010000 + 3800 + 0x04 = 0x40013804
                                             
#define USART1_TX_DMA_CH                     DMA1_Channel4
#define USART1_RX_DMA_CH                     DMA1_Channel5
                                             
	                                           
#define USART2_DR_Base                       (USART2_BASE + 4)		// 0x40000000 + 4400 + 0x04 = 0x40013804
                                             
#define USART2_TX_DMA_CH                     DMA1_Channel7
#define USART2_RX_DMA_CH                     DMA1_Channel6




#define DEBUG_USART                          USART1


#define HOSTIF_USART                         USART2
#define HOSTIF_USART_CIR_BUFF                usart2_rx_cir_buff
#define HOSTIF_USART_TX_DMA_CH               USART2_TX_DMA_CH
#define HOSTIF_USART_RX_DMA_CH               USART2_RX_DMA_CH

#define SIM800C_USART                        USART3

#define USART3_MAX_RECV_LEN		              512				//�����ջ����ֽ���
#define USART3_MAX_SEND_LEN		              100				//����ͻ����ֽ���



#define SENDBUFF_SIZE                        256


typedef struct
{
    //0  baurd 1200 2400 4800 9600 14400 19200 38400 57600 115200 (define 9600)
    // 
	  u32 baurd;
    //1  data_bits   8  9(define 8)
	  u8 data_bits;
    //2  parity      0-none   1-Even    2-Odd(defube 0)
	  u8 parity;
    //3  stop        0-0.5  1-1  2-1.5   3-2(define  1)
	  u8 stop;
    //4  flow_ctr    0-none   1-handware   2-sofrware(define 0)
	  u8 flow_ctr;
	  
}USART_PARA;



typedef struct
{

	  unsigned int len;
	  unsigned int remain;
	  unsigned char busy;
	  unsigned char * buff;
	  unsigned char * rptr;
	  unsigned char * wptr;
	  
}CIRCULATION_BUFF;





	  	
extern uint8_t SendBuff[SENDBUFF_SIZE];


extern void (*usart3_irq_callback) (u8 data);

extern unsigned char  USART1_RX_BUF[USART1_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern unsigned char  USART2_RX_BUF[USART2_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 


extern unsigned char  USART3_RX_BUF[USART3_MAX_RECV_LEN]; 		//���ջ���,���USART3_MAX_RECV_LEN�ֽ�
extern unsigned char  USART3_TX_BUF[USART3_MAX_SEND_LEN]; 		//���ͻ���,���USART3_MAX_SEND_LEN�ֽ�
extern u16 USART3_RX_STA;   						//��������״̬

extern CIRCULATION_BUFF usart1_rx_cir_buff;
extern CIRCULATION_BUFF usart2_rx_cir_buff;

extern u16 USART1_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��
extern void uart1_init(u32 bound);
extern void uart2_init(USART_PARA para);
extern void uart3_init(void);//���ڳ�ʼ������

extern void usart_send_str(USART_TypeDef * USARTx, u8 * str, u16 len);
extern void usart_send_byte(USART_TypeDef * USARTx, u8 str);
extern u16 USART_dma_send(DMA_Channel_TypeDef* DMAy_Channelx, u8 *str, u16 len);


extern u16 circulation_buff_available(DMA_Channel_TypeDef* DMAy_Channelx, CIRCULATION_BUFF cir_buff);
extern int circulation_buff_read(DMA_Channel_TypeDef* DMAy_Channelx, CIRCULATION_BUFF* cir_buff,unsigned char* read_buff,unsigned int len);



#endif


