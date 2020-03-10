#include "timer.h"
#include "led.h"
#include "usart.h"
#include "can.h"
#include "wdg.h"
//////////////////////////////////////////////////////////////////////////////////	 
//ͨ�ö�ʱ��4�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��4!

//���ʱ�� = �� �Զ�����ֵ��ARR��+ 1 ���� Ԥ��Ƶϵ����PSC��+ 1 �� / ��ʱ��ʱ�ӣ�Tclk��72MHz
////20ms -> (199+1)( 7199+1)/72000000 = 0.02s = 20ms
void TIM4_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //ʱ��ʹ��
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���	

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	
	//TIM_ClockDivision�Ǹı������벶��ʱ�˲��õġ�
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	
	//��������0�������Զ�����ֵ��TIMx_ARR����Ȼ�����´�0��ʼ�������Ҳ���һ������������¼�
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
  //TIM_ClearITPendingBit(TIM4, TIM_IT_Update);//��������ж�����λ
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIMx����  ������ʱ��4
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //ʹ��ָ����TIM4�ж�,��������ж�
							 
}


//��ʱ��4�жϷ������
void TIM4_IRQHandler(void)   //TIM4�ж�
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
		{	 	
			  TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //���TIMx���жϴ�����λ:TIM �ж�Դ 
			
			  Vehicle1_Data_Field[0]=(Velocity>>3)&0x00ff;
			  Vehicle1_Data_Field[1]=((Velocity&0x0007)<<5)|((Yaw_Rate>>8)&0x000F);
			  Vehicle1_Data_Field[2]=Yaw_Rate&0x00FF;
			  Vehicle1_Data_Field[3]=0x9F;
			  Vehicle1_Data_Field[4]=0xFF;
			  Vehicle1_Data_Field[5]=0;
			  Vehicle1_Data_Field[6]=0;
			  Vehicle1_Data_Field[7]=0;
			
				Can_Send_Msg(0x4f0,Vehicle1_Data_Field,8);
				Can_Send_Msg(0x4f1,Vehicle2_Data_Field,8);
				Can_Send_Msg(0x5f2,Vehicle3_Data_Field,8);
				Can_Send_Msg(0x5f3,Vehicle4_Data_Field,8);
				Can_Send_Msg(0x5f4,Vehicle5_Data_Field,8);
				Can_Send_Msg(0x5f5,Vehicle6_Data_Field,8);
				Can_Send_Msg(0x5c0,ESR_Sim1_Data_Field,8);
			
    }
}
