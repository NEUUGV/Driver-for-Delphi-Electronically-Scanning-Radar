#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"
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
 
//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024;  tq=(brp)*tpclk1
//������=pclk1/(((tbs1+1)+(tbs2+1)+1)*brp);
//mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
//pclk1��ʱ���ڳ�ʼ����ʱ������Ϊ36M,�������CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_LoopBack);
//������Ϊ:36M/((8+9+1)*4)=500Kbps
//����ֵ:0,��ʼ��OK;
// ����,��ʼ��ʧ��;

//CAN���ߵĲ�������ȡ��������APB1��PCLK1����ͨ������RCC_PCLK1Config��PCLK1����Ƶ�ʡ����������ϵ��ĸ�ֵ֮��
//CAN���ߵĲ�����=PCLK1/((CAN_SJW +CAN_BS1 + CAN_BS2)*CAN_Prescaler)
//����PCLK1=36MHz��CAN_SJW=1��CAN_BS1=8��CAN_BS2=7��CAN_Prescaler=9
//��CAN���ߵĲ�����=PCLK1/((1 + 8 + 7) * 9) = 36MHz / 16 / 9 = 250Kbits

u8 CAN_Mode_Init(void)
{ 
	GPIO_InitTypeDef 		GPIO_InitStructure; 
	CAN_InitTypeDef        	CAN_InitStructure;
	CAN_FilterInitTypeDef  	CAN_FilterInitStructure;

	NVIC_InitTypeDef  		NVIC_InitStructure;


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	
	
	// ����CAN1 �����жϵ����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

/*
(1)GPIO_Mode_AIN ģ�����롪Ӧ��ADCģ�����룬���ߵ͹�����ʡ��

(2)GPIO_Mode_IN_FLOATING �������롪������KEYʶ��

(3)GPIO_Mode_IPD �������롪 IO�ڲ�������������

(4)GPIO_Mode_IPU �������롪IO�ڲ�������������

(5)GPIO_Mode_Out_OD ��©�����IO���0��GND��IO���1�����գ���Ҫ����������裬����ʵ������ߵ�ƽ��
 �����Ϊ1ʱ��IO�ڵ�״̬�������������ߵ�ƽ���������ǿ�©���ģʽ������IO��Ҳ�Ϳ������ⲿ��·�ı�Ϊ�͵�ƽ�򲻱䡣
 ���Զ�IO�����ƽ�仯��ʵ��C51��IO˫���ܡ�

(6)GPIO_Mode_Out_PP ���������IO���0-��GND��IO���1 -��VCC��������ֵ��δ֪�ġ�

(7)GPIO_Mode_AF_OD ���ÿ�©�����Ƭ�����蹦�ܣ�TX1,MOSI,MISO.SCK.SS����

(8)GPIO_Mode_AF_PP �������������Ƭ�����蹦�ܣ�I2C��SCL,SDA����
	*/

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//��ʼ��IO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//��ʼ��IO

	//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM=DISABLE;			//��ʱ�䴥��ͨ��ģʽ  
	CAN_InitStructure.CAN_ABOM=DISABLE;			//������߹���	 
	CAN_InitStructure.CAN_AWUM=DISABLE;			//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART=ENABLE;			//��ֹ�����Զ����� 
	CAN_InitStructure.CAN_RFLM=DISABLE;		 	//���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP=DISABLE;			//�������ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;	        //ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ; 
	//���ò�����
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;				//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=CAN_BS1_3tq; 			//Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;				//Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=12;        //��Ƶϵ��(Fdiv)Ϊbrp+1	
	CAN_Init(CAN1, &CAN_InitStructure);        	//��ʼ��CAN1 

	CAN_FilterInitStructure.CAN_FilterNumber=0;	//������0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 	//��ʶ������ģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; 	//32λ�� 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;	//32λID ����32λ��ʶ����ID���ĸ�16λ
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;  //����32λ��ʶ����ID���ĵ�16λ
	//����32λ����λMASK=0x00000000������������ʶ��ID�ı��Ķ��ܱ�����
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK //���ø�16λ����λ
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000; // //���õ�16λ����λ
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//���������0

	CAN_FilterInit(&CAN_FilterInitStructure);			//�˲�����ʼ��	   

	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);				//FIFO0��Ϣ�Һ��ж�����.		ʹ��CAN1 �����ж� 

	return 0;
}   
 

//CAN�����жϷ�����			    
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
				 Vehicle2_Data_Field[6]=0xbf; //����ֽڵ�6λΪ1���Ӷ����������64���ϰ���
				 Vehicle2_Data_Field[7]=0xec; //����ֽڵ�Bit5Ϊ1���Ӷ�����������Ч
		}	
		else if(RxMessage.StdId==0x4E1)
		{
			   for(i=0;i<8;i++) ESR_Status_2_Data_Field[i]=RxMessage.Data[i];
		}
		else if(RxMessage.StdId==0x4E3)
		{
			   for(i=0;i<8;i++) ESR_Status_4_Data_Field[i]=RxMessage.Data[i];
		}			
		else if((RxMessage.StdId>=0x500)&&(RxMessage.StdId<=0x53F))//��ȡ�ϰ�����Ϣ	
		{
					 Obstacle_ID=RxMessage.StdId-0x500;
					 for(i=0;i<8;i++) ESR_Track_Data_Field[Obstacle_ID][i]=RxMessage.Data[i];
					
			     //��ȡ�ڣ�Obstacle_ID+1�����ϰ���ĽǶ�
					 if((ESR_Track_Data_Field[Obstacle_ID][1]&0x10)==0x10) ANGLE[Obstacle_ID]=(((ESR_Track_Data_Field[Obstacle_ID][1]%32)<<5)+(ESR_Track_Data_Field[Obstacle_ID][2]>>3))|0xfc00; //�õ������Ƕ�
					 else ANGLE[Obstacle_ID]=(((ESR_Track_Data_Field[Obstacle_ID][1]%32)<<5)+(ESR_Track_Data_Field[Obstacle_ID][2]>>3)); //�õ������Ƕ�
					
					 //��ȡ�ڣ�Obstacle_ID+1�����ϰ���ľ���
			     RANGE[Obstacle_ID]=((ESR_Track_Data_Field[Obstacle_ID][2]<<8)+ESR_Track_Data_Field[Obstacle_ID][3])&0x07ff;
					
		}
		
		
}



//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
u8 Can_Send_Msg(uint32_t ID,u8* Msg_Data_Field,u8 Len)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=ID;			// ��׼��ʶ�� 
	//TxMessage.ExtId=0x12;			// ������չ��ʾ�� 
	TxMessage.IDE=CAN_Id_Standard; 	// ��׼֡
	TxMessage.RTR=CAN_RTR_Data;		// ����֡
	TxMessage.DLC=Len;				// Ҫ���͵����ݳ���
	for(i=0;i<Len;i++)
	TxMessage.Data[i]=Msg_Data_Field[i];			          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0; 
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
	if(i>=0XFFF)return 1;
	return 0;	 
}
//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;
/*u8 Can_Receive_Msg(u8 *buf,uint32_t *id_std,uint32_t *id_ext)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
	if(RxMessage.StdId==0x217)
	{
	  for(i=0;i<8;i++)
		buf[i]=RxMessage.Data[i]; 
	*id_std=RxMessage.StdId;
	*id_ext=RxMessage.ExtId;//��ָ�����ݲ��ԣ�����ȫ�ֱ���
	
		return RxMessage.DLC;	
	}
	else
		return 0;	
}*/














