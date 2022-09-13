#include "usart.h"	  
#include "bsp.h"	
#include "timer.h"

/*******************************
��ʼ������1
bound ������


*******************************/
//void UART1_Init(uint32_t bound)
//{
//	    /* enable USART, GPIOA  clock */
//    rcu_periph_clock_enable(RCU_GPIOA);
//    rcu_periph_clock_enable(RCU_AF);
//	    /* enable USART clock */
//    rcu_periph_clock_enable(RCU_USART0);

//    /* connect port to USARTx_Tx */
//    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

//    /* connect port to USARTx_Rx */
//    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

//    /* USART configure */
//    usart_deinit(USART0);
//    usart_baudrate_set(USART0, 115200U);
//    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
//    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
//    usart_enable(USART0);
//	
//	
//		nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
//    nvic_irq_enable(USART0_IRQn, 0, 1);
//	
//}

/*******************************
��ʼ������0
bound ������

GD32C103 ʵ���ǽӵ��˴���1 ��
*******************************/
void UART1_Init(uint32_t bound)
{
	 /* enable USART, GPIOA  clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART1);
    rcu_periph_clock_enable(RCU_AF);
    
		nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(USART1_IRQn, 0, 1);
	

    /* connect port to USART1_Tx */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    /* connect port to USART1_Rx */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3);

   
    /* USART1 and USART2 baudrate configuration */
    usart_baudrate_set(USART1, bound);//������
    
    /* configure USART word length */
    usart_word_length_set(USART1, USART_WL_8BIT);//8λ���ݸ�ʽ

    /* configure USART stop bits */
    usart_stop_bit_set(USART1, USART_STB_1BIT);//ֹͣλ

    /* configure USART transmitter */
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);//ʹ�ܷ���

    /* configure USART receiver */
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);//ʹ�ܽ���

    /* enable USART */
    usart_enable(USART1);

    /* enable the USART interrupt */
    usart_interrupt_enable(USART1, USART_INT_RBNE);

}

//����һ���ֽ�
void KLIN_Send_ByteOne(uint8_t byte)
{
		iKwpLastSendValue=byte;
    usart_data_transmit(USART1, (uint8_t)byte);
    while(RESET == usart_flag_get(USART1, USART_FLAG_TBE));	
}
/***********************************************
����KWP ����
***********************************************/
uint8_t CountKwpDataLong(uint8_t *cmdaddr)
{
		uint8_t Slong=0;  
	uint8_t date1=0,date2=0;
	date1=cmdaddr[0]&0xf0;
	date2=cmdaddr[0]&0x0f;
	//iNissanKwpFLag=0;
  if(date1==0x80) //8X֡ 
	{
		if(date2==0)// 80֡
			Slong=cmdaddr[3]+5; //����
    else		        //8X֡
			Slong=cmdaddr[0]-0x80+4; //����
	}	
	else if(date1>0x80)//
	{
		if(date2==0)// 80֡
			Slong=cmdaddr[3]+5; //����
    else		        //CX֡
			Slong=cmdaddr[0]-0xC0+4; //����
	}
	else if(date1<0x40)//NISSAN KWP
	{
			Slong=cmdaddr[0]+2; //����
		 // iNissanKwpFLag=0x80;//�ղ�kwp
	}
	return Slong;
}
// �ۼӺ�У��
uint8_t SumDat(uint8_t *cmdaddr,uint8_t Slong)
{
	uint8_t Sidx=0;
	uint16_t Sum=0;
	for(Sidx = 0; Sidx <(Slong-1); Sidx ++)
	{ 
		Sum=Sum+cmdaddr[Sidx];
	}
	return (Sum&0x00ff);
}

/***********************************************
����:KWP ��������ͺ���
����:
����:�ɹ�>0;ʧ��=0
***********************************************/
uint8_t Send_Frame_KW2000Test(uint8_t cmdaddr[])
{
	
	uint8_t Sidx=0,Slong=0;  
//	uint16_t i=0;
	
	Slong=CountKwpDataLong(cmdaddr);	
	cmdaddr[Slong-1]=SumDat(cmdaddr,Slong);
	for(Sidx = 0; Sidx <Slong; Sidx ++)
	{ 
	  KLIN_Send_ByteOne(cmdaddr[Sidx]); 
		Delay_ms(5);
	 }
	 
	 return (Slong-3);
}



//------------------------------------------------------------------------------
// Funtion: KWP �ж����߿���״̬ PA3
// Info   : none  �ȴ��Ϳ���>30MS
//------------------------------------------------------------------------------
uint8_t KWP_CheckBus(void)
{
    uint32_t time;
   // SetDelay( 200000 );  //200ms
    time = 0;
	//if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_1))//�͵�ƽ
		GetTimer6Cnt();//����˼�����
		return 1;
//	
//		while( 1)//�ߵ�ƽʵ��
//	 {
//				time = GetTimer6CntEx();
//				if(time>40*1000) return 1;//EOF 
//		}
	
//    // �ȴ�һ���Ϳ���
//    while(1)
//    {
//			if(iPartValue!=2&&iPartValue!=3&&iPartValue!=4) break;//��K
//			
//			
//        time = GetTimer6CntEx();
//			  if(time>(200*1000)) break;//��ʱ��
//				if(SET == gpio_input_bit_get(GPIOA,GPIO_PIN_3))
//				{
//					GetTimer6Cnt();//����˼�����
//					 // һֱ�ȴ�һ����
//					while( SET == gpio_input_bit_get(GPIOA,GPIO_PIN_3))//�ߵ�ƽʵ��
//					{
//						 time = GetTimer6CntEx();
//						 if(time>30*1000) return 1;//EOF 
//					}
//					GetTimer6Cnt();//����˼�����
//				}

//    }
	
	
    return 1;
}


/***********************************************
����:��׼ KWP 9141  ����ͺ���
����:
����:��һ֡��ƫ�Ƶ�ַ
***********************************************/
uint8_t Send_StISO9141Frame(uint8_t cmdaddr[])
{
	uint8_t Sidx=0,Slong=0;  
	uint16_t i=0;
	uint32_t tmp=0;

	
	
	Slong=cmdaddr[0];
	
	
	
	cmdaddr[Slong-1+1]=SumDat(cmdaddr+1,Slong);
	//cmdaddr[Slong-1]=sum;//�ۼӺ����һ���ֽ�
	
	//USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);//���ж�
	    /* enable the USART interrupt */
  usart_interrupt_disable(USART1, USART_INT_RBNE);
	usart_receive_config(USART1, USART_RECEIVE_DISABLE);//�رս���
	usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE);//����
	
	Delay_ms(20);
	for(Sidx=0; Sidx <Slong; Sidx ++)
	{ 
	  KLIN_Send_ByteOne(cmdaddr[Sidx+1]); 
		if(Sidx==(Slong-1))
		{
			Delay_us(20);
			usart_data_receive(USART1);
			usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE);//����
		}
		Delay_ms(5);
	 }

	 
	 usart_interrupt_enable(USART1, USART_INT_RBNE); 
	 usart_receive_config(USART1, USART_RECEIVE_ENABLE);//ʹ�ܽ���

	
	 
	 return Slong+1;
	
}


/***********************************************
iReFlag=0 ֻ������ 
����:��׼ KWP 2000  ����ͺ���
����:
����:��һ֡��ƫ�Ƶ�ַ
***********************************************/
uint8_t Send_StKWP2000Frame(uint8_t cmdaddr[])
{
	uint8_t Sidx=0,Slong=0;  
	uint16_t i=0;
	uint32_t tmp=0;

	
	if(iPartValue==4)//9141
	{
		Slong=Send_StISO9141Frame(cmdaddr);
		return Slong;
	}
	
	Slong=CountKwpDataLong(cmdaddr);
	
	
	
	cmdaddr[Slong-1]=SumDat(cmdaddr,Slong);
	//cmdaddr[Slong-1]=sum;//�ۼӺ����һ���ֽ�
	
	//USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);//���ж�
	    /* enable the USART interrupt */
  usart_interrupt_disable(USART1, USART_INT_RBNE);
	usart_receive_config(USART1, USART_RECEIVE_DISABLE);//�رս���
	usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE);//����
	
	Delay_ms(20);
	for(Sidx=0; Sidx <Slong; Sidx ++)
	{ 
	  KLIN_Send_ByteOne(cmdaddr[Sidx]); 
		if(Sidx==(Slong-1))
		{
			Delay_us(20);
			usart_data_receive(USART1);
			usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE);//����
		}
		Delay_ms(5);
	 }

	 
	 usart_interrupt_enable(USART1, USART_INT_RBNE); 
	 usart_receive_config(USART1, USART_RECEIVE_ENABLE);//ʹ�ܽ���

	
	 
	 return Slong;
	
}


/***********************************************
iReFlag=0 ֻ������ 
����:
����:
����:��һ֡��ƫ�Ƶ�ַ
***********************************************/
uint8_t Send_StKWP2000Value(uint8_t Value)
{


	
	//cmdaddr[Slong-1]=sum;//�ۼӺ����һ���ֽ�
	
	//USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);//���ж�
	    /* enable the USART interrupt */
  usart_interrupt_disable(USART1, USART_INT_RBNE);
	usart_receive_config(USART1, USART_RECEIVE_DISABLE);//�رս���
	usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE);//����
	
//	Delay_ms(20);
//	for(Sidx=0; Sidx <Slong; Sidx ++)
//	{ 
//	  KLIN_Send_ByteOne(cmdaddr[Sidx]); 
//		if(Sidx==(Slong-1))
//		{
//			Delay_us(20);
//			usart_data_receive(USART1);
//			usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE);//����
//		}
//		Delay_ms(5);
//	 }
	  Delay_ms(5); 	
	 KLIN_Send_ByteOne(Value);
	 Delay_ms(1); 	
	 //			Delay_us(20);
	usart_data_receive(USART1);
	usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE);//����
	Delay_us(20);
	usart_interrupt_enable(USART1, USART_INT_RBNE); 
	 usart_receive_config(USART1, USART_RECEIVE_ENABLE);//ʹ�ܽ���

	
	 
	 return 1;
	
}


//�ߵ͵�ƽ����
void Low_High_ms(u16 low,u16 high)
{
     rcu_periph_clock_enable(RCU_GPIOA);
    /* configure LED2 GPIO port */ 
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_2);
    /* reset LED2 GPIO pin */
	//tja1027 оƬ���ƽŽ�PB11 ,PB11=1ʱʹ��оƬ
    gpio_bit_set(GPIOB,GPIO_PIN_11);	//PB11=1
	
	  gpio_bit_set(GPIOA,GPIO_PIN_2);	//PA2=2  ����
		Delay_ms(300);//300ms �ߵ�ƽ
	  //��������
	  gpio_bit_reset(GPIOA,GPIO_PIN_2);	
	  Delay_ms(low);
		gpio_bit_set(GPIOA,GPIO_PIN_2);	
	  Delay_ms(high);
}
/*************************************
����:���͵�ַ�뺯��
�ɹ�����:1 ʧ��0
*************************************/
u8 SendAddFrame(u8 add)
{
	  u8 i=0;
     rcu_periph_clock_enable(RCU_GPIOA);
    /* configure LED2 GPIO port */ 
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_2);
    /* reset LED2 GPIO pin */
	//tja1027 оƬ���ƽŽ�PB11 ,PB11=1ʱʹ��оƬ
    gpio_bit_set(GPIOB,GPIO_PIN_11);	//PB11=1
	
	  gpio_bit_set(GPIOA,GPIO_PIN_2);//����
	  Delay_ms(1000);
	  Delay_ms(1000);
		Delay_ms(1000);//����3�����ϸߵ�ƽ
		//GPIO_SetBits(GPIOA,K_TXD);
    gpio_bit_reset(GPIOA,GPIO_PIN_2);//ֹͣλ
    Delay_ms(200);		
		for(i=0;i<8;i++)//�͵��� 
		{
			
			if(add&0x01)gpio_bit_set(GPIOA,GPIO_PIN_2);
			else     	gpio_bit_reset(GPIOA,GPIO_PIN_2);
			add=add>>1;	
		//	if(i<7)
				Delay_ms(200);
	  	
    }
		gpio_bit_set(GPIOA,GPIO_PIN_2);

		//��ʼ�� 10416
   return 1;
}


//����һ���ֽ�����
//byte ���յ�������
//delayMs ��ʱ����
//0  ��ʱ
//1  ��ȷ����
u8 KLIN_Recieve_Byte(u8 *byte,u16 delayMs)
{
	u32 delay=0;

	u32 delayMs1=delayMs*1000;
	GetTimer6Cnt();
	iKRxdValue=0;
	iKRxdFlag=0x00;//���־λ
	while(1)
	{
		Delay_us(1);
		//delay++;
		delay=GetTimer6CntEx();
		if(delay>delayMs1) 
		{
			return 0;//��ʱ��
		}
//		if(SET==usart_flag_get(USART1, USART_FLAG_RT))
//		{
//			iKRxdValue=usart_data_receive(USART1);
//			break;
//		}
		
		if(iKRxdFlag==1)
		{
			break;
		}
		
	}
	
	 *byte = iKRxdValue; 
		return 1;

}

//����ϵͳ
// 0 ����ʧ��
// 1 ����ɹ�
u8 AddrWakeUpOneEx(u8 iAddValue,uint16_t iKwpBaudVale)
{  
	u8 TimeOutFlag=0;
	u8 err=0;
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
//	USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);  //��ֹ�����ж�
	
	usart_interrupt_disable(USART1, USART_INT_RBNE);
	usart_receive_config(USART1, USART_RECEIVE_DISABLE);//�رս���
	
	if(SendAddFrame(iAddValue))
	{    
		UART1_Init(iKwpBaudVale); 
		
	  if(KLIN_Recieve_Byte(&err,1000))//55  300MS��ʱ    55
	  {
	     if(KLIN_Recieve_Byte(&err,50))//K1
	     {			
				 if(KLIN_Recieve_Byte(&err,50))//k2
				 {
						 Delay_ms(30);
						 KLIN_Send_ByteOne(~err);
						 if(KLIN_Recieve_Byte(&err,60 +60)) //���յ�ַ��ȡ�� 
						 {
							  Delay_ms(50);
							 Delay_ms(500);
								TimeOutFlag=1;
						 }
				 }
		 }
	  }
	}
	else
	{
		TimeOutFlag = 0;
	}
//	
	
	//UART2_Init(iKwpBaudVale); 
	return TimeOutFlag;
} 


//���� K�� 
//iType=0 �ߵ͵�ƽ 25/25��iType=1 ��ַ�� ����
//iKwpBaudVale �����Ĳ�����
//�ɹ� 1
//ʧ�� 0
uint8_t InitKinSys(uint8_t iType,uint8_t SendAdd,uint16_t iKwpBaudVale)
{
	uint8_t iRet=0;
	if(iType==0)
	{
	//	UART2_Init(iKwpBaudVale); 		
		Low_High_ms(25,25);
	  UART1_Init(iKwpBaudVale); 		
		iRet=1;
	}
	else if(iType==1)
	{
		iRet=AddrWakeUpOneEx(SendAdd,iKwpBaudVale);//��ַ�� ����
	}
	
	
	
	return iRet;
}


/***********************************************

����:��׼ KWP 2000  ����ͺ���
����:cmdaddr = 83 F1 11 C1 EF 8F C4
����:
����: 
***********************************************/
uint8_t SendKwp14230Frame(uint8_t cmdaddr[])
{
	uint8_t Sidx=0,Slong=0;  
	uint16_t i=0;
	uint32_t tmp=0;

	
	Slong=CountKwpDataLong(cmdaddr);//����֡ͷ�Զ����㳤��

	cmdaddr[Slong-1]=SumDat(cmdaddr,Slong);
	//cmdaddr[Slong-1]=sum;//�ۼӺ����һ���ֽ�
	
	//USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);//���ж�
	    /* enable the USART interrupt */
  usart_interrupt_disable(USART1, USART_INT_RBNE);
	usart_receive_config(USART1, USART_RECEIVE_DISABLE);//�رս���
	usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE);//����
	
	Delay_ms(20);
	for(Sidx=0; Sidx <Slong; Sidx ++)
	{ 
	  KLIN_Send_ByteOne(cmdaddr[Sidx]); 
		if(Sidx==(Slong-1))
		{
			Delay_us(20);
			usart_data_receive(USART1);
			usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE);//����
		}
		Delay_ms(5);
	 }

	 
	 usart_interrupt_enable(USART1, USART_INT_RBNE); 
	 usart_receive_config(USART1, USART_RECEIVE_ENABLE);//ʹ�ܽ���

	
	 
	 return Slong;
	
}

/***********************************************

����:��׼ 9141 ����ͺ���
����:cmdaddr = 06 68 6A F1 09 02 CE
����:
����: 
***********************************************/
uint8_t SendKwp9141Frame(uint8_t cmdaddr[])
{
	uint8_t Sidx=0,Slong=0;  
	uint16_t i=0;
	uint32_t tmp=0;
	
	Slong=cmdaddr[0];//���㳤��	
	cmdaddr[Slong]=SumDat(cmdaddr+1,Slong);

	//cmdaddr[Slong-1]=sum;//�ۼӺ����һ���ֽ�
	
	//USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);//���ж�
	    /* enable the USART interrupt */
  usart_interrupt_disable(USART1, USART_INT_RBNE);
	usart_receive_config(USART1, USART_RECEIVE_DISABLE);//�رս���
	usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE);//����
	
	Delay_ms(20);
	for(Sidx=0; Sidx <Slong; Sidx ++)
	{ 
	  KLIN_Send_ByteOne(cmdaddr[Sidx]); 
		if(Sidx==(Slong-1))
		{
			Delay_us(20);
			usart_data_receive(USART1);
			usart_interrupt_flag_clear(USART1, USART_INT_FLAG_RBNE);//����
		}
		Delay_ms(5);
	 }

	 
	 usart_interrupt_enable(USART1, USART_INT_RBNE); 
	 usart_receive_config(USART1, USART_RECEIVE_ENABLE);//ʹ�ܽ���

	
	 
	 return Slong;
	
}
