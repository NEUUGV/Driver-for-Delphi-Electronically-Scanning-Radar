#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//CAN驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/7
//版本：V1.1 
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved	
//********************************************************************************
//V1.1修改说明 20150528
//修正了CAN初始化函数的相关注释，更正了波特率计算公式
////////////////////////////////////////////////////////////////////////////////// 	 
 
//CAN初始化
//tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:时间段2的时间单元.   范围:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:时间段1的时间单元.   范围:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :波特率分频器.范围:1~1024;  tq=(brp)*tpclk1
//波特率=pclk1/(((tbs1+1)+(tbs2+1)+1)*brp);
//mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
//pclk1的时钟在初始化的时候设置为36M,如果设置CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_LoopBack);
//则波特率为:36M/((8+9+1)*4)=500Kbps
//返回值:0,初始化OK;
// 其他,初始化失败;

//CAN总线的波特率是取自于总线APB1（PCLK1），通过函数RCC_PCLK1Config给PCLK1配置频率。设置了以上的四个值之后，
//CAN总线的波特率=PCLK1/((CAN_SJW +CAN_BS1 + CAN_BS2)*CAN_Prescaler)
//假设PCLK1=36MHz、CAN_SJW=1、CAN_BS1=8、CAN_BS2=7、CAN_Prescaler=9
//则CAN总线的波特率=PCLK1/((1 + 8 + 7) * 9) = 36MHz / 16 / 9 = 250Kbits

u8 CAN_Mode_Init(void)
{ 
	GPIO_InitTypeDef 		GPIO_InitStructure; 
	CAN_InitTypeDef        	CAN_InitStructure;
	CAN_FilterInitTypeDef  	CAN_FilterInitStructure;

	NVIC_InitTypeDef  		NVIC_InitStructure;


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	
	
	// 设置CAN1 接收中断的优先级
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

/*
(1)GPIO_Mode_AIN 模拟输入—应用ADC模拟输入，或者低功耗下省电

(2)GPIO_Mode_IN_FLOATING 浮空输入—可以做KEY识别

(3)GPIO_Mode_IPD 下拉输入— IO内部下拉电阻输入

(4)GPIO_Mode_IPU 上拉输入—IO内部上拉电阻输入

(5)GPIO_Mode_Out_OD 开漏输出—IO输出0接GND，IO输出1，悬空，需要外接上拉电阻，才能实现输出高电平。
 当输出为1时，IO口的状态由上拉电阻拉高电平，但由于是开漏输出模式，这样IO口也就可以由外部电路改变为低电平或不变。
 可以读IO输入电平变化，实现C51的IO双向功能。

(6)GPIO_Mode_Out_PP 推挽输出—IO输出0-接GND，IO输出1 -接VCC，读输入值是未知的。

(7)GPIO_Mode_AF_OD 复用开漏输出—片内外设功能（TX1,MOSI,MISO.SCK.SS）。

(8)GPIO_Mode_AF_PP 复用推挽输出—片内外设功能（I2C的SCL,SDA）。
	*/

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//初始化IO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//初始化IO

	//CAN单元设置
	CAN_InitStructure.CAN_TTCM=DISABLE;			//非时间触发通信模式  
	CAN_InitStructure.CAN_ABOM=DISABLE;			//软件离线管理	 
	CAN_InitStructure.CAN_AWUM=DISABLE;			//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART=ENABLE;			//禁止报文自动传送 
	CAN_InitStructure.CAN_RFLM=DISABLE;		 	//报文不锁定,新的覆盖旧的  
	CAN_InitStructure.CAN_TXFP=DISABLE;			//发送优先级由报文标识符决定 
	CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;	        //模式设置： mode:0,普通模式;1,回环模式; 
	//设置波特率
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;				//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=CAN_BS1_3tq; 			//Tbs1=tbs1+1个时间单位CAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;				//Tbs2=tbs2+1个时间单位CAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=12;        //分频系数(Fdiv)为brp+1	
	CAN_Init(CAN1, &CAN_InitStructure);        	//初始化CAN1 

	CAN_FilterInitStructure.CAN_FilterNumber=0;	//过滤器0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 	//标识符屏蔽模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; 	//32位宽 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;	//32位ID 设置32位标识符（ID）的高16位
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;  //设置32位标识符（ID）的低16位
	//设置32位屏蔽位MASK=0x00000000，令带有任意标识符ID的报文都能被接收
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK //设置高16位屏蔽位
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000; // //设置低16位屏蔽位
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//激活过滤器0

	CAN_FilterInit(&CAN_FilterInitStructure);			//滤波器初始化	   

	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);				//FIFO0消息挂号中断允许.		使能CAN1 接收中断 

	return 0;
}   
 

//CAN接收中断服务函数			    
void USB_LP_CAN1_RX0_IRQHandler(void)
{
		int i=0;
	  uint32_t Obstacle_ID=0;  //0~63
    CanRxMsg RxMessage;
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	 
		if(RxMessage.StdId==0x4E0)
		{
			   for(i=0;i<8;i++) ESR_Status_1_Data_Field[i]=RxMessage.Data[i];
			
				 CAN_TX_SCAN_INDEX=(ESR_Status_1_Data_Field[3]<<8)+ESR_Status_1_Data_Field[4];
				
				 Vehicle2_Data_Field[0]=CAN_TX_SCAN_INDEX>>8;		  
				 Vehicle2_Data_Field[1]=CAN_TX_SCAN_INDEX&(0x00ff);    
				 Vehicle2_Data_Field[2]=0x00;		  
				 Vehicle2_Data_Field[3]=0x00;		  
				 Vehicle2_Data_Field[4]=0x00;		  
				 Vehicle2_Data_Field[5]=0x00;		  
				 Vehicle2_Data_Field[6]=0xbf; //令该字节低6位为1，从而声明最多检测64个障碍物
				 Vehicle2_Data_Field[7]=0xec; //令该字节的Bit5为1，从而声明车速有效
		}	
		else if(RxMessage.StdId==0x4E1)
		{
			   for(i=0;i<8;i++) ESR_Status_2_Data_Field[i]=RxMessage.Data[i];
		}
		else if(RxMessage.StdId==0x4E3)
		{
			   for(i=0;i<8;i++) ESR_Status_4_Data_Field[i]=RxMessage.Data[i];
		}			
		else if((RxMessage.StdId>=0x500)&&(RxMessage.StdId<=0x53F))//获取障碍物信息	
		{
					 Obstacle_ID=RxMessage.StdId-0x500;
					 for(i=0;i<8;i++) ESR_Track_Data_Field[Obstacle_ID][i]=RxMessage.Data[i];
					
			     //获取第（Obstacle_ID+1）个障碍物的角度
					 if((ESR_Track_Data_Field[Obstacle_ID][1]&0x10)==0x10) ANGLE[Obstacle_ID]=(((ESR_Track_Data_Field[Obstacle_ID][1]%32)<<5)+(ESR_Track_Data_Field[Obstacle_ID][2]>>3))|0xfc00; //得到负数角度
					 else ANGLE[Obstacle_ID]=(((ESR_Track_Data_Field[Obstacle_ID][1]%32)<<5)+(ESR_Track_Data_Field[Obstacle_ID][2]>>3)); //得到正数角度
					
					 //获取第（Obstacle_ID+1）个障碍物的距离
			     RANGE[Obstacle_ID]=((ESR_Track_Data_Field[Obstacle_ID][2]<<8)+ESR_Track_Data_Field[Obstacle_ID][3])&0x07ff;
					
		}
		
		
}



//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
u8 Can_Send_Msg(uint32_t ID,u8* Msg_Data_Field,u8 Len)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=ID;			// 标准标识符 
	//TxMessage.ExtId=0x12;			// 设置扩展标示符 
	TxMessage.IDE=CAN_Id_Standard; 	// 标准帧
	TxMessage.RTR=CAN_RTR_Data;		// 数据帧
	TxMessage.DLC=Len;				// 要发送的数据长度
	for(i=0;i<Len;i++)
	TxMessage.Data[i]=Msg_Data_Field[i];			          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0; 
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
	if(i>=0XFFF)return 1;
	return 0;	 
}
//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
/*u8 Can_Receive_Msg(u8 *buf,uint32_t *id_std,uint32_t *id_ext)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
	if(RxMessage.StdId==0x217)
	{
	  for(i=0;i<8;i++)
		buf[i]=RxMessage.Data[i]; 
	*id_std=RxMessage.StdId;
	*id_ext=RxMessage.ExtId;//用指针数据不对，改用全局变量
	
		return RxMessage.DLC;	
	}
	else
		return 0;	
}*/














