#include "stm32f10x.h"
#include "can.h"
#include "usart.h"
#include "delay.h"
#include "timer.h"
#include "wdg.h"
#include "lcd.h"
/************************************************
 ALIENTEK ս��STM32F103������ʵ��0
 ����ģ��
 ע�⣬�����ֲ��е��½������½�ʹ�õ�main�ļ� 
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/



	//���ײ��״����������Ϣ
	u8 Vehicle1_Data_Field[8]={0x00,0x00,0x00,0x1f,0xff,0x00,0x00,0x00};//0x4f0
	u8 Vehicle2_Data_Field[8]={0x00,0x00,0x00,0x00,0x00,0x00,0xbf,0xec};//0x4f1
	u8 Vehicle3_Data_Field[8]={0xc0,0x04,0x0f,0x78,0x28,0x5c,0x00,0x00};//0x5f2
	u8 Vehicle4_Data_Field[8]={0x00,0x00,0x00,0x00,0x00,0x02,0x01,0x01};//0x5f3
	u8 Vehicle5_Data_Field[8]={0x00,0x47,0x00,0x00,0x00,0x56,0x2e,0x12};//0x5f4
	u8 Vehicle6_Data_Field[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//0x5f5
	u8 ESR_Sim1_Data_Field[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//0x5c0
	
  u16 CAN_TX_SCAN_INDEX=0;//��ȡESR_Status1--CAN_TX_SCAN_INDEX
  
	//��⵽��64���ϰ���ľ���ͽǶ�
	s16 ANGLE[64];
	u16 RANGE[64];
    //���ײ������ϰ�����Ϣ
	u8 ESR_Status_1_Data_Field[8]={0};
	u8 ESR_Status_2_Data_Field[8]={0};
	u8 ESR_Status_4_Data_Field[8]={0};
  u8 ESR_Track_Data_Field[64][8]={0};


u16 len=0;
u16 Velocity=0;
s16 Yaw_Rate=0;
	
u16 USART_RX_STA=0;       //����״̬���
	

u8 USART_RX_BUF[300];     //���ջ���,���USART_REC_LEN���ֽ�.

int main(void)
 {	
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	 delay_init();
	 //IWDG_Init(4,625);    //���Ƶ��Ϊ64,����ֵΪ625,���ʱ��Ϊ1s	
	 uart_init(115200);
	 TIM4_Int_Init(199,7199);//20����
	 //LCD_Init();
	 CAN_Mode_Init();
	
	 //USART_ClearFlag(USART1, USART_FLAG_TC); //�巢�ͽ�����־
	while(1)
	 {
		 u16 num_1=0;//#λ��
		 u16 num_2=0;//,λ��
		 u16 i=0;
		 u16 x=0;
		 s16 y=0;
		 int sign_yaw_rate=1;
		 
		 if(USART_RX_STA&0x8000)
		 {     
			     len=USART_RX_STA&0x3fff;					 			
					 
					 for(i=0;i<len;i++)
					 {
						 if(USART_RX_BUF[i]==0x23) //ȷ��#�ŵ�λ��
							 num_1=i;
						 if(USART_RX_BUF[i]==0x2C) //ȷ�����ŵ�λ��
							 num_2=i;
					 }
					 for(i=num_1+1;i<len;i++)
					 {
						 if(i<num_2) x=x*10+USART_RX_BUF[i]-48;   //0��ASCIIΪ48
						 
						 if(i>num_2)
						 {
							 if(USART_RX_BUF[i]==45)
								 sign_yaw_rate=-1;
							 else y=y*10+sign_yaw_rate*(USART_RX_BUF[i]-48);
						 }
					 }
					 Velocity=x;
					 Yaw_Rate=y;
			 
			 
			 	//��λ��STM32F103ͨ������1���ͼ�⵽��64���ϰ���ľ���ͽǶ���Ϣ����λ��
						for(i=0;i<64;i++) printf("%d,%d;",RANGE[0],ANGLE[0]);
						printf("$");					 
						
						USART_RX_STA=0;
			 
		 }		  

		
	}
	
}
