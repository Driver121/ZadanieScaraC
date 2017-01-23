
#include "funkcie.h"
#include "stm32l1xx_tim.h"
#include "stm32l1xx.h"
#include "stm32l1xx_it.h"

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure,TIM_TimeBaseStructure1;
TIM_OCInitTypeDef  TIM_OCInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure1;
NVIC_InitTypeDef NVIC_InitStructure;
TIM_ICInitTypeDef TIM_ICInitStructure;


uint16_t PrescalerValue = 0, timerPeriodValue=0;


/*Táto funkcia ma na staros nastavenie GPIO,èasovaèov a PWM . Tie sme nastavily pod¾a motorov ktoré sme mali dostupné

 */
void pwm_initOutput()
{
	 /* TIMX clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    /* GPIOA clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);


	/*--------------------------------- GPIO nastavenie -------------------------*/
	/* GPIOB nastavenie : Pin 6 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;

	// GPIOA nastavenie: Pin 6 | Pin 7
	GPIO_InitStructure1.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure1.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure1.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_40MHz;


	// Inicializácia štruktúry
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_Init(GPIOA, &GPIO_InitStructure1);

    // Nastavenie Aleternatívnych funckii
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4); //
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3); //
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3); //

	/* Nastavenie predelièky  */
	PrescalerValue = (uint16_t) 16;


	/*Nastavenie èasovaèa pre digitálny motor */
	/*Nastavenie pre digitálny motor nastavenie na 300 Hz */

	TIM_TimeBaseStructure.TIM_Period = 3332;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	/*Nastavenie èasovaèa pre analógový motor */
	/*Nastavenie pre analógový  motor nastavenie na 50 Hz */
	TIM_TimeBaseStructure1.TIM_Period =19999 ;
    TIM_TimeBaseStructure1.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure1.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure1.TIM_CounterMode = TIM_CounterMode_Up;

   /* Inicializácia Timerov */
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM4, ENABLE);  //

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure1);
	TIM_Cmd(TIM3, ENABLE);

	/* PWM1 Mode configuration: Channel1 / Chnnel 2 */

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	/*Nastavenie channelov */
	TIM_OC1Init(TIM4, &TIM_OCInitStructure); //
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    /* TIM3 má dva Channel 1 a Channel 2 */
    TIM_OC1Init(TIM3, &TIM_OCInitStructure); //
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_OC2Init(TIM3, &TIM_OCInitStructure); //
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);


    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_ARRPreloadConfig(TIM3, ENABLE);

	/* TIM4 enable counter */
	TIM_Cmd(TIM4, ENABLE);
	/* TIM5 enable counter */
	TIM_Cmd(TIM3, ENABLE);
}

/* Nastavenie hodnoty pre motor ktorý ovláda motor */
void nastavPero(uint16_t value)
{
	TIM3->CCR1 =value;
}

/* Nastavenie hodnoty pre ve¾ký  motor */
void nastavVelkyMotor(uint16_t value)
{
	TIM4->CCR1 =value;

}

/* Nastavenie hodnoty pre maly motor */
void nastavMalyMotor(uint16_t value)
{
	TIM3->CCR2 =value;

}





