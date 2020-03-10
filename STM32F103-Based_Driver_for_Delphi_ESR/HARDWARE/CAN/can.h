#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	 
#include "math.h"
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
 
//CAN接收RX0中断使能
#define CAN_RX0_INT_ENABLE	1		//0,不使能;1,使能.								    

//检测到的64个障碍物的角度和距离
  extern s16 ANGLE[64];
	extern u16 RANGE[64];

  extern u8 ESR_Status_1_Data_Field[8];
  extern u8 ESR_Status_2_Data_Field[8];
  extern u8 ESR_Status_4_Data_Field[8];	
  extern u8 ESR_Track_Data_Field[64][8];
	
	extern int flag_usart_rec;
	extern u16 Velocity_Rate;
	extern s16 Yaw;
	
	
	
extern u8 Vehicle1_Data_Field[8];
extern u8 Vehicle2_Data_Field[8];
extern u8 Vehicle3_Data_Field[8];
extern u8 Vehicle4_Data_Field[8];
extern u8 Vehicle5_Data_Field[8];
extern u8 Vehicle6_Data_Field[8];
extern u8 Vehicle1_Data_Field[8];	
extern u8 ESR_Sim1_Data_Field[8];
//Vehicle1

extern	u8 CAN_RX_VEHICLE_SPEED;//来源于采集车速

extern  u8 CAN_RX_STEERING_ANGLE;//偏航角

 //Vehicle2
extern  u16 CAN_TX_SCAN_INDEX;//读取ESR_Status1--CAN_TX_SCAN_INDEX

 //Vehicle3

extern double CAN_RX_RADAR_HEIGHT;//雷达距地面高度


//Vehicle5

extern u16 CAN_RX_DISTANCE_REAR_AXLE;//雷达到后轴的距离
extern double CAN_RX_WHEELBASE;//车辆轴距
	
	
u8 CAN_Mode_Init(void);//CAN初始化
 
u8 Can_Send_Msg(uint32_t ID,u8* Msg_Data_Field,u8 Len);						//发送数据

u8 Can_Receive_Msg(u8 *buf,uint32_t *id_std,uint32_t *id_ext);							//接收数据
#endif

















