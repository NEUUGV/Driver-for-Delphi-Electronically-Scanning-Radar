#include "timer.h"
#include "led.h"
#include "usart.h"
#include "can.h"
#include "wdg.h"
//////////////////////////////////////////////////////////////////////////////////	 
//通用定时器4中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器4!

//溢出时间 = （ 自动加载值（ARR）+ 1 ）（ 预分频系数（PSC）+ 1 ） / 定时器时钟（Tclk）72MHz
////20ms -> (199+1)( 7199+1)/72000000 = 0.02s = 20ms
void TIM4_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //时钟使能
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器	

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	
	//TIM_ClockDivision是改变做输入捕获时滤波用的。
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	
	//计数器从0计数到自动加载值（TIMx_ARR），然后重新从0开始计数并且产生一个计数器溢出事件
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
  //TIM_ClearITPendingBit(TIM4, TIM_IT_Update);//清除更新中断请求位
	TIM_Cmd(TIM4, ENABLE);  //使能TIMx外设  启动定时器4
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //使能指定的TIM4中断,允许更新中断
							 
}


//定时器4中断服务程序
void TIM4_IRQHandler(void)   //TIM4中断
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{	 	
			  TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源 
			
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
