#include "stm32f10x.h"
#include "can.h"
#include "usart.h"
#include "delay.h"
#include "timer.h"
#include "wdg.h"
#include "lcd.h"
/************************************************
 ALIENTEK 战舰STM32F103开发板实验0
 工程模板
 注意，这是手册中的新建工程章节使用的main文件 
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/



	//毫米波雷达接收数据信息
	u8 Vehicle1_Data_Field[8]={0x00,0x00,0x00,0x1f,0xff,0x00,0x00,0x00};//0x4f0
	u8 Vehicle2_Data_Field[8]={0x00,0x00,0x00,0x00,0x00,0x00,0xbf,0xec};//0x4f1
	u8 Vehicle3_Data_Field[8]={0xc0,0x04,0x0f,0x78,0x28,0x5c,0x00,0x00};//0x5f2
	u8 Vehicle4_Data_Field[8]={0x00,0x00,0x00,0x00,0x00,0x02,0x01,0x01};//0x5f3
	u8 Vehicle5_Data_Field[8]={0x00,0x47,0x00,0x00,0x00,0x56,0x2e,0x12};//0x5f4
	u8 Vehicle6_Data_Field[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//0x5f5
	u8 ESR_Sim1_Data_Field[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//0x5c0
	
  u16 CAN_TX_SCAN_INDEX=0;//读取ESR_Status1--CAN_TX_SCAN_INDEX
  
	//检测到的64个障碍物的距离和角度
	s16 ANGLE[64];
	u16 RANGE[64];
    //毫米波发送障碍物信息
	u8 ESR_Status_1_Data_Field[8]={0};
	u8 ESR_Status_2_Data_Field[8]={0};
	u8 ESR_Status_4_Data_Field[8]={0};
  u8 ESR_Track_Data_Field[64][8]={0};


u16 len=0;
u16 Velocity=0;
s16 Yaw_Rate=0;
	
u16 USART_RX_STA=0;       //接收状态标记
	

u8 USART_RX_BUF[300];     //接收缓冲,最大USART_REC_LEN个字节.

int main(void)
 {	
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	 delay_init();
	 //IWDG_Init(4,625);    //与分频数为64,重载值为625,溢出时间为1s	
	 uart_init(115200);
	 TIM4_Int_Init(199,7199);//20毫秒
	 //LCD_Init();
	 CAN_Mode_Init();
	
	 //USART_ClearFlag(USART1, USART_FLAG_TC); //清发送结束标志
	while(1)
	 {
		 u16 num_1=0;//#位置
		 u16 num_2=0;//,位置
		 u16 i=0;
		 u16 x=0;
		 s16 y=0;
		 int sign_yaw_rate=1;
		 
		 if(USART_RX_STA&0x8000)
		 {     
			     len=USART_RX_STA&0x3fff;					 			
					 
					 for(i=0;i<len;i++)
					 {
						 if(USART_RX_BUF[i]==0x23) //确定#号的位置
							 num_1=i;
						 if(USART_RX_BUF[i]==0x2C) //确定逗号的位置
							 num_2=i;
					 }
					 for(i=num_1+1;i<len;i++)
					 {
						 if(i<num_2) x=x*10+USART_RX_BUF[i]-48;   //0的ASCII为48
						 
						 if(i>num_2)
						 {
							 if(USART_RX_BUF[i]==45)
								 sign_yaw_rate=-1;
							 else y=y*10+sign_yaw_rate*(USART_RX_BUF[i]-48);
						 }
					 }
					 Velocity=x;
					 Yaw_Rate=y;
			 
			 
			 	//下位机STM32F103通过串口1发送检测到的64个障碍物的距离和角度信息给上位机
						for(i=0;i<64;i++) printf("%d,%d;",RANGE[0],ANGLE[0]);
						printf("$");					 
						
						USART_RX_STA=0;
			 
		 }		  

		
	}
	
}
