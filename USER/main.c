#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "timer.h"
#include "timer_irq.h"
#include "adc.h"
#include "control.h"
#include "stm32f10x.h"
#include "save_data.h"
#include "hostif.h"
#include "SIM800C.h"



extern  __IO uint16_t Adc1_temp[5];
#ifdef TIM4_CH1
extern u8  TIM4CH1_CAPTURE_STA;		//���벶��״̬		    				
extern u16	TIM4CH1_CAPTURE_VAL;	//���벶��ֵ	
#endif
#ifdef TIM4_CH2
extern u8  TIM4CH2_CAPTURE_STA;		//���벶��״̬		    				
extern u16	TIM4CH2_CAPTURE_VAL;	//���벶��ֵ	
#endif
#if (leg_count==3)  
#ifdef TIM4_CH3
extern u8  TIM4CH3_CAPTURE_STA;		//���벶��״̬		    				
extern u16	TIM4CH3_CAPTURE_VAL;	//���벶��ֵ	
#endif
#endif

void key_run(void);
void usart_run(void);

u8 switch_hand_ctr = 0;

u8 rx_buff[200] = {0};
u16 rx_len = 0;



void disable_jtag()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	*(__IO uint32_t *) (PERIPH_BB_BASE + ((PWR_BASE - PERIPH_BASE) * 32) + (0x08 * 4)) = (uint32_t)ENABLE;
	*(__IO uint32_t *) (PERIPH_BB_BASE + ((BKP_BASE - PERIPH_BASE + 0x30) * 32) + (0x00 * 4)) = (uint32_t)DISABLE;
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);	 
}




u16 receive_len = 0;

int main(void)
{		
	//SCB->VTOR = FLASH_BASE | 0x4000; /* Vector Table Relocation in Internal FLASH. */
	 
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart1_init(115200);	 //���ڳ�ʼ��Ϊ115200
	TIM2_Int_Init(1000,71);
	SIM800C_Init();
	delay_ms(100);
	SIM_PWR_Open();
    read_para_from_flash();

	uart2_init(para_value.usart_para);	 

	//IWDG_Init(6,625);    //���Ƶ��Ϊ256,����ֵΪ625,���ʱ��Ϊ4s	

	
	//define_para_value();
	
	//KEY_Init(); 

	//ADC1_Init();
	//ADC_DMA();
	
    DEBUG("SAVE_DATA len is %d\r\n", sizeof(SAVE_DATA));
    DEBUG("System is start!!!!!\r\n");
    
    while(1) 
		{
		  
			//����ֻ����sim800c������������
			//���͵�sim800c��͸�������� host���պ����д�����
			//host_receive_packet();
			//SIM8XX_User_Receive();
			
			
			if(para_value.word_mode == WOKE_MODE_AT)
			{
					woke_mode_at_main();
			}
			else if(para_value.word_mode == WOKE_MODE_TS)
			{
					woke_mode_ts_main();
			}
			else if(para_value.word_mode == WOKE_MODE_HTTP)
			{
					woke_mode_http_main();
			}
			else if(para_value.word_mode == WOKE_MODE_MODBUS_HTTP)
			{
					woke_mode_modbus_http_main();
			}
		}
}


