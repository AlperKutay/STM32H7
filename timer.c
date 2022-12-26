#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
void Tim2_Init(void);
void Pin_Init(void);
void TIM2_IRQHandler(void)
{
if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
{
TIM_ClearITPendingBit(TIM2, TIM_IT_Update);//BAYRAGI TEMIZE KESME GELDYSE
GPIO_ToggleBits(GPIOD, GPIO_Pin_13); // PIN13 YANSIN
}
}
int main()
{
Tim2_Init();
Pin_Init();
while(1)
{
}
}
void Tim2_Init(void)
{
TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); //TIMER 2 YOLU AKTIF EDILDI;
TIM_TimeBaseStruct.TIM_Period=1000-1; //1 MS ;
TIM_TimeBaseStruct.TIM_Prescaler=84-1;// 24 MHZ 1 MHZ ILE DÜS
TIM_TimeBaseStruct.TIM_ClockDivision=0;
TIM_TimeBaseStruct.TIM_CounterMode=TIM_CounterMode_Up;
TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);
TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); // IT ENABLE
TIM_Cmd(TIM2,ENABLE); //CNT ENABLE
NVIC_InitTypeDef NVIC_InitStruct;
// TIMER 2 GENEL KESMELER AKTIF EDILICEK;
NVIC_InitStruct.NVIC_IRQChannel=TIM2_IRQn;
NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
NVIC_InitStruct.NVIC_IRQChannelSubPriority=1;
NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
NVIC_Init(&NVIC_InitStruct);
 
}
void Pin_Init(void)
{
GPIO_InitTypeDef GPIO_InitStruct;
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
GPIO_InitStruct.GPIO_Pin=GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;
GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
GPIO_Init(GPIOD,&GPIO_InitStruct);
}
uint16_t EVAL_AUDIO_GetSampleCallBack(void)
{
return 0; // ses çipini kullaniyorsaniz tek sample veriyi burada return ile döndürün.
}
void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size)
{
return; // ses çipini kullaniyorsaniz burada çipe veri aktarimi DMA sona ermis
}