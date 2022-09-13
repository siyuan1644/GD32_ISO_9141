#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "gd32C10x.h"
#include "string.h"

#define  K_TXD         GPIO_Pin_9  //K��   PA9
#define  K_RXD         GPIO_Pin_10  //K��	 PA10

#define  L_LINE         GPIO_Pin_4

void UART0_Init(uint32_t bound);

void UART1_Init(uint32_t bound);
void UART2_Init(uint32_t bound);
void UART3_Init(uint32_t bound);


void KLIN_Send_ByteOne(uint8_t byte);
uint8_t Send_Frame_KW2000Test(uint8_t cmdaddr[]);//KWP ���Ժ���

uint8_t KWP_CheckBus(void);
uint8_t Send_StKWP2000Frame(uint8_t cmdaddr[]);
uint8_t Send_StKWP2000Value(uint8_t Value);//����һ���ֽ�

uint8_t InitKinSys(uint8_t iType,uint8_t SendAdd,uint16_t iKwpBaudVale);//����ϵͳ
uint8_t SendKwp14230Frame(uint8_t cmdaddr[]);//KWP ����ͺ���
uint8_t SendKwp9141Frame(uint8_t cmdaddr[]);

#endif


