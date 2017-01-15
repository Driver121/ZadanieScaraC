#include "stm32l1xx_tim.h"
#include "stm32l1xx.h"

#ifndef FUNKCIE_H_
#define FUNKCIE_H_


/* funkcie pre riadenie a komunikáciu cez USART 2*/
void initUSART2(void);
void RegisterCallbackUART2(void *callback);
void USART2_IRQHandler(void);
void vykresli(char *);
void pockaj(void);


/* funkcie suvisiace s PWM */
void pwm_initOutput(void);
void nastavPero(uint16_t);
void nastavVelkyMotor(uint16_t);
void nastavMalyMotor(uint16_t);



#endif /* FUNKCIE_H_ */
