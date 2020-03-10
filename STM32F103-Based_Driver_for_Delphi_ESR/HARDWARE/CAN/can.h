#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	 
#include "math.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//CAN���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/7
//�汾��V1.1 
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved	
//********************************************************************************
//V1.1�޸�˵�� 20150528
//������CAN��ʼ�����������ע�ͣ������˲����ʼ��㹫ʽ
////////////////////////////////////////////////////////////////////////////////// 	 
 
//CAN����RX0�ж�ʹ��
#define CAN_RX0_INT_ENABLE	1		//0,��ʹ��;1,ʹ��.								    

//��⵽��64���ϰ���ĽǶȺ;���
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

extern	u8 CAN_RX_VEHICLE_SPEED;//��Դ�ڲɼ�����

extern  u8 CAN_RX_STEERING_ANGLE;//ƫ����

 //Vehicle2
extern  u16 CAN_TX_SCAN_INDEX;//��ȡESR_Status1--CAN_TX_SCAN_INDEX

 //Vehicle3

extern double CAN_RX_RADAR_HEIGHT;//�״�����߶�


//Vehicle5

extern u16 CAN_RX_DISTANCE_REAR_AXLE;//�״ﵽ����ľ���
extern double CAN_RX_WHEELBASE;//�������
	
	
u8 CAN_Mode_Init(void);//CAN��ʼ��
 
u8 Can_Send_Msg(uint32_t ID,u8* Msg_Data_Field,u8 Len);						//��������

u8 Can_Receive_Msg(u8 *buf,uint32_t *id_std,uint32_t *id_ext);							//��������
#endif

















