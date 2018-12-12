#include "stm32f10x.h"
#include "adc.h"

__IO uint16_t Adc1_temp[5] = {0,0,0,0,0};
void ADC1_Init(void)
{
	ADC_InitTypeDef ADC_InitStructure;//����ṹ��
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;				// ????0
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;			// ????
  GPIO_Init(GPIOA, &GPIO_InitStructure);				//??A??
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//�����ڵ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;//ģ��ת��������ɨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//ģ��ת����������ģʽ��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�涨�������ұ߶���
	ADC_InitStructure.ADC_NbrOfChannel = 4;//
	ADC_Init(ADC1, &ADC_InitStructure);//��������

	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_55Cycles5);
	//����ָ��ADC�Ĺ�����ͨ�����������ǵ�ת��˳���ת��ʱ��
	ADC_DMACmd(ADC1, ENABLE);//ʹ��ָ����ADC��DMA����
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_ResetCalibration(ADC1);//����ָ����ADc��У׼�Ĵ���
	while(ADC_GetResetCalibrationStatus(ADC1));//��ȡADC����У׼�Ĵ�����״̬
	ADC_StartCalibration(ADC1);//��ʼָ��ADC��У׼״̬
	while(ADC_GetCalibrationStatus(ADC1));//��ȡָ����У׼��״̬
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//ʹ��ָ����ADc�����ת������
}
//-----------------//
void ADC_DMA(void)
{
	DMA_InitTypeDef DMA_InitStructure; //����ṹ��
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//??MDA
	
	DMA_DeInit(DMA1_Channel1);//��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;//�������ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&Adc1_temp;//�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//������Ϊ���ݴ������Դ
	DMA_InitStructure.DMA_BufferSize = 4;//����ָ��DMAͨ����DMA����Ĵ�С����λΪ���ݵ�λ
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�����ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//�������ݿ��Ϊ16
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//�ڴ����ݿ��Ϊ16
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//������ѭ������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//ͨ�����и����ȼ�
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//DMAͨ��û������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);//��ʼ��ͨ��1�ļĴ���
	DMA_Cmd(DMA1_Channel1, ENABLE);//ʹ��ͨ��1
}

