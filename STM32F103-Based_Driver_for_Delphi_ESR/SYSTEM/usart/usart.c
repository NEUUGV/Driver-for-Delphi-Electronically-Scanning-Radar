#include "sys.h"
#include "usart.h"	  
#include "math.h"	
#include "can.h"	
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/8/18
//�汾��V1.5
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*ʹ��microLib�ķ���*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	

//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
//u16 USART_RX_STA=0;       //����״̬���	  
  
void uart_init(u32 Baud_Rate){
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
  
	
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
	
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
  //USART1_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

  //Usart1 NVIC����Ƕ�����жϿ������� ����  
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = Baud_Rate;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 

}
void USART1_IRQHandler(void)   //����1�жϷ������
{
	  u8 Res;
	  u16 Buffer_Index;
	
#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();    
#endif
		if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
		{
				Res =USART_ReceiveData(USART1);	//��ȡ���յ�������
				if((USART_RX_STA&0x8000)==0)//����δ���
					{
							if(USART_RX_STA&0x4000)//���յ���0x0d
								{
										if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
										else USART_RX_STA|=0x8000;	//���������
								}
							else //��û�յ�0X0D
								{	
										if(Res==0x0d)USART_RX_STA|=0x4000;
										else
											{
													Buffer_Index=USART_RX_STA&0X3FFF;
												  USART_RX_BUF[Buffer_Index]=Res;
													USART_RX_STA++;
													if(USART_RX_STA>(300-1))USART_RX_STA=0;//���ڽ��յ�����Ч�ֽ����������ջ����С,��λ����״̬���	  
											}		 
								}
					}   		 
     } 
		
		//���ڽ������ݹ��࣬���������������󣬲���������жϣ�ORE��
		if(USART_GetFlagStatus(USART1,USART_FLAG_ORE)==SET)
			{
				USART_ClearFlag(USART1,USART_FLAG_ORE); //������жϱ�־
				USART_ReceiveData(USART1);  // �ӵ�����
			}
		
#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntExit();  											 
#endif
} 
#endif	

