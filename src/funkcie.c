/*
 * cv6.c
 *
 *  Created on: Oct 26, 2016
 *      Author: Andrej
 */
#include "funkcie.h"
#include <stddef.h>
#include "stm32l1xx.h"
#include <stdio.h>
#include <string.h>


uint16_t value = 0;


void initUSART2(void)   /// usart 1
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /*Nastavenie hodín pre GPIOA  */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  /*Nastavienie hodín pre usart 2  */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);


    /* Nastavenie Pinov pre Usart pre odoslavanie a prímanie */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    /*Inicializácia GPIO */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;   //  GPIO_Pin_10 | GPIO_Pin_9;

    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //usart configuration
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);   // usart 1


    //configuring interrupts

      NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);


      NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
      //choosing which event should cause interrupt
      USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

      // Enable USART
      USART_Cmd(USART2, ENABLE);

}

void (* gCallback1)(unsigned char) = 0;
void RegisterCallbackUART2(void *callback){
    gCallback1 = callback;
}

char retazicek[30];
char hodnotaPWM1[3];
char hodnotaPWM2[3];

int pocetik = 0;
int hMalyMotor=0;
int hVelkyMotor=0;
int polomer=0;

float xStart=0;
float xFinish=0;
float y=0;
float r_sq=0;


/*
 *  V tejto funkcii riešime prijatie pola v ktorom máme zakodované  o aký pohyb sa jedná a aké su jeho parametre pre jednotlivé motory
 *
 */
void USART2_IRQHandler(void)  //USART1_IRQHandler
{
    uint16_t pom = 0;

        if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
        {
            if (pocetik == 0){          // tato podmienka spravi, ze v pripade ze sa posiela novy prikaz (to je vtedy ked je pocetik = 0)
                retazicek[0] = '\0';    // tak vyresetuj vysledny retazec
            }

            USART_ClearITPendingBit(USART2, USART_IT_RXNE);
            char znacicek = USART_ReceiveData(USART2);

            retazicek[pocetik++] = znacicek;    // tu sa pridavaju znaky do vysledneho retazca

            if(znacicek == '\n'){       // tato podmienka spravi, ze ak zachyti znak enter, tak resetne premennu pocetik - teda
                pocetik = 0;           // to je koniec prikazu a ked pride dalsi prikaz tak ho uz uklada nanovo
                if (gCallback1)
                 {
                    gCallback1(retazicek);
                 }
            }



        }

}


uint16_t hodnota=0;
uint16_t pole[50];
int i=0;

/* V tejto funkcii riešime spracovanie udajov zo G-kodu a pod¾a toho vygenerujem PWM pre motory */

void vykresli(char hodnota[]){
	 if(retazicek[0]=='L')
	 {
	      if(retazicek[3]=='0')   // Nastav pero na pisanie
	                	  {
	                		  nastavPero(1100); // hodnota pre zlozenie pera

	                		//  pockaj();

	                		  hodnotaPWM1[0]=retazicek[5];
	                		  hodnotaPWM1[1]=retazicek[6];
	                		  hodnotaPWM1[2]=retazicek[7];
	                		  hodnotaPWM1[3]=retazicek[8];

	                		  hodnotaPWM2[0]=retazicek[10];
	                		  hodnotaPWM2[1]=retazicek[11];
	                		  hodnotaPWM2[2]=retazicek[12];
	                		  hodnotaPWM2[3]=retazicek[13];

	                		  sscanf(hodnotaPWM1,"%d",&hVelkyMotor);
	                		  sscanf(hodnotaPWM2,"%d",&hMalyMotor);

	                		  nastavVelkyMotor(hVelkyMotor);
	                		  nastavMalyMotor(hMalyMotor);


	                	  }
	                	  else   // Zodvihni pero
	                	  {
	                		  nastavPero(1700); // hodnota pre zdvyhnutie pera
	                	//	  pockaj();
	                		  hodnotaPWM1[0]=retazicek[5];
	                		  hodnotaPWM1[1]=retazicek[6];
	                		  hodnotaPWM1[2]=retazicek[7];
	                		  hodnotaPWM1[3]=retazicek[8];

							  hodnotaPWM2[0]=retazicek[10];
							  hodnotaPWM2[1]=retazicek[11];
							  hodnotaPWM2[2]=retazicek[12];
							  hodnotaPWM2[3]=retazicek[13];

	                		  sscanf(hodnotaPWM1,"%d",&hVelkyMotor);
	                		  sscanf(hodnotaPWM2,"%d",&hMalyMotor);

	                		  nastavVelkyMotor(hVelkyMotor);
	                		  nastavMalyMotor(hMalyMotor);
	                  	  }
	                  }
	 if(retazicek[0]=='K')
	                  {
                           if (retazicek[7]=='0')
                           {
	                	  	   nastavPero(1100); // hodnota pre zlozenie pera


	                	  	    hodnotaPWM1[0]=retazicek[9];
	                	  	    hodnotaPWM1[1]=retazicek[10];
	                	  	    hodnotaPWM1[2]=retazicek[11];
	                	  	    hodnotaPWM1[3]=retazicek[12];

	                	  	    hodnotaPWM2[0]=retazicek[14];
	                	  	    hodnotaPWM2[1]=retazicek[15];
	                	  	    hodnotaPWM2[2]=retazicek[16];
	                	  	    hodnotaPWM2[3 ]=retazicek[17];

	                	  	   sscanf(hodnotaPWM1,"%d",&hVelkyMotor);
	                	  	   sscanf(hodnotaPWM2,"%d",&hMalyMotor);

	                	  	   nastavVelkyMotor(hVelkyMotor);
	                	  	   nastavMalyMotor(hMalyMotor);
                           }
                           else
                           {
                        	   nastavPero(1700); // hodnota zdvihnutie zlozenie pera


								hodnotaPWM1[0]=retazicek[9];
								hodnotaPWM1[1]=retazicek[10];
								hodnotaPWM1[2]=retazicek[11];
								hodnotaPWM1[3]=retazicek[12];

								hodnotaPWM2[0]=retazicek[14];
								hodnotaPWM2[1]=retazicek[15];
								hodnotaPWM2[2]=retazicek[16];
								hodnotaPWM2[3 ]=retazicek[17];

							   sscanf(hodnotaPWM1,"%d",&hVelkyMotor);
							   sscanf(hodnotaPWM2,"%d",&hMalyMotor);

							   nastavVelkyMotor(hVelkyMotor);
							   nastavMalyMotor(hMalyMotor);
                           }




	                  }
}

void pockaj()
{
	int i;
	for (i=0;i<500000;i++)
	{

	}
}



