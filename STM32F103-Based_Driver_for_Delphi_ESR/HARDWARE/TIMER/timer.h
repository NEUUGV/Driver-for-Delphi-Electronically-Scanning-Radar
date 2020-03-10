#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"


void TIM4_Int_Init(u16 arr,u16 psc);
void TIM4_PWM_Init(u16 arr,u16 psc);

void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
int PWM_OUT(int x);

#define FORWARD PGout(2)// PG2
#define BACK PGout(4)// PG4	

extern s16 error_intger;	
extern u8 canbuf_accelerator[8];
void DIR_Init(void);//≥ı ºªØ


extern u16 Velocity;
extern s16 Yaw_Rate;

#endif
