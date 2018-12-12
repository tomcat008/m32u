#include "pwr.h"
#include "control.h"
#include "usart.h"
#include "24cxx.h"
#include "mortor.h"
#include "save_data.h"

u8 diaodianceshi_count = 10;

void PVD_Config(void)
{
		NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    EXTI_InitStructure.EXTI_Line = EXTI_Line16;														//PVD�����ж���16
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 									//ʹ���ж�ģʽ
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;							//�½��ش���
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;								//�����ش���
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;			//��ѹ�������½�Խ���趨��ֵʱ�������ж�
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; 														//ʹ���ж���
    EXTI_Init(&EXTI_InitStructure); 																			//��ʼ��

    PWR_PVDCmd(ENABLE); 																									//ʹ��PVD
    PWR_PVDLevelConfig(PWR_PVDLevel_2V9);																	//�趨��ط�ֵ
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	
		NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	
}

void PVD_IRQHandler(void)
{
//		u8 dssdsdfs = 0;
		EXTI_ClearITPendingBit(EXTI_Line16);
		
		if(PWR_GetFlagStatus(PWR_FLAG_PVDO)) 
		{
				mortor_stop();
				
//			if(diaodianceshi_count == 1)
//				AT24CXX_WriteOneByte(14,0);
//			else
			{
				//printf("11111:%d",motor_running);
				if((motor_running == 1) || (diaodian == 1))
					AT24CXX_WriteOneByte(0xff,1);	
				
			}
				


//				printf("%d",dssdsdfs);
////				puts("Has entered the interrupt!!!");
				PWR_ClearFlag(PWR_FLAG_PVDO);
		}
}
 
