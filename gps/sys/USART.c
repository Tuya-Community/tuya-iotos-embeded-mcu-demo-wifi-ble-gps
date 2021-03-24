#include "USART.h"
#include "TIM.h"

void creat_sq(Queue* SQ)//���ö��в���
{
	SQ->front=0;
	SQ->rear=0;
	SQ->size=quene_main_buf_total;
}
void front_inc(Queue* SQ)//����ָ��
{
	SQ->front++;
	if(SQ->front==SQ->size)
	{
		SQ->front=0;
	}
}
void rear_inc(Queue* SQ)//��βָ��
{
	SQ->rear++;
	if(SQ->rear==SQ->size)
	{
		SQ->rear=0;
	}
}



uint8_t USART_Send(USART_TypeDef * MY_usart,uint8_t *data,uint16_t len)
{
	uint8_t ret=1;
	uint16_t timeout=0x8000;
	while(len>0)
	{
		timeout=0x8000;
		MY_usart->TDR = *data;
		while((MY_usart->ISR&1<<6)!=1<<6)//�ȴ��������
		{
			timeout--;
			if( 0 == timeout )
			{
				ret = 1;
				break;
			}
		}
		data++;
		len--;
	}
	if( 0 != timeout )
	{
		ret = 0;
	}
	return ret;
}

Queue Q_Main;
struct quene_buf_type1 q_mian_buf[quene_main_buf_total];
uint8_t USART_MAIN_BUF[USART_MAIN_BUF_LEN];
uint8_t F_TASK_USART_MAIN=0;
void Configure_USART_MAIN(uint32_t bound) //TX PC4, RX PC5 USART1
{
	RCC->APBRSTR2 &=~(1<<14);//�ָ�����1
	RCC->IOPENR |= 1<<2;//ʹ��GPIOCʱ��
	GPIOC->MODER &=~(3<<8|3<<10);
	GPIOC->MODER |=2<<8|2<<10;//����ģʽ
	GPIOC->AFR[0] &=~(0xf<<16|0xf<<20);
	GPIOC->AFR[0] |=1<<16|1<<20;//ѡ���ù���AF1
	RCC->APBENR2 |=1<<14;//ʹ�ܴ���1ʱ��	
	USART_MAIN->BRR = 16000000 / bound; 
	USART_MAIN->CR1 |= 1<<0|1<<2|1<<3|1<<5;//����ʹ�ܣ�ʹ�ܽ��գ�ʹ�ܷ���,
	while((USART_MAIN->ISR & 1<<6) != 1<<6)//������ɱ�־λ
	{ 
		break;/* add time out here for a robust application */
	}	
	NVIC_SetPriority(USART1_IRQn, 1);
	NVIC_EnableIRQ(USART1_IRQn);
	creat_sq(&Q_Main);
	F_TASK_USART_MAIN=0;//�����ʱ����ʼ��ʱ���󴥷� ���������������Ϊ���ճ�ʱ��ʱ������ʱ�Ѿ��󴥷�һ��
}
void USART1_IRQHandler(void)
{
	if((USART_MAIN->ISR & 1<<5) == 1<<5)//���ռĴ������ݲ�Ϊ��
	{
		uart_receive_input((unsigned char)(USART_MAIN->RDR));
	}
	else
	{
		//NVIC_DisableIRQ(USART1_IRQn); /* Disable USART1_IRQn */
	}	
	if((USART_MAIN->ISR & (1<<3)) == (1<<3))//ORE
	{
		USART_MAIN->ICR =1<<3;
	}
}



Queue Q_Side;
struct quene_buf_type2 q_side_buf[quene_side_buf_total];
uint8_t USART_VICE_BUF[USART_VICE_BUF_LEN];
uint8_t F_TASK_USART_VICE=0;
void Configure_USART_VICE(uint32_t bound) //TX PD5, RX PD6 USART2	
{
	RCC->APBRSTR1 &=~(1<<17);//�ָ�����2
	RCC->IOPENR |= 1<<3;//ʹ��GPIODʱ��
	GPIOD->MODER &=~(3<<10|3<<12);
	GPIOD->MODER |=2<<10|2<<12;//����ģʽ
	GPIOD->AFR[0] &=~(0xf<<20|0xf<<24);
	GPIOD->AFR[0] |=0<<20|0<<24;//ѡ���ù���AF0
	RCC->APBENR1 |=1<<17;//ʹ�ܴ���3ʱ��
	
	USART_VICE->BRR = 16000000 / bound; 
	USART_VICE->CR1 |= 1<<0|1<<2|1<<3|1<<5;//����ʹ�ܣ�ʹ�ܽ��գ�ʹ�ܷ���,
	while((USART_VICE->ISR & 1<<6) != 1<<6)//������ɱ�־λ
	{ 
		break;/* add time out here for a robust application */
	}	
	NVIC_SetPriority(USART2_IRQn, 1);
	NVIC_EnableIRQ(USART2_IRQn);	
	creat_sq(&Q_Side);
	F_TASK_USART_VICE=0;//�����ʱ����ʼ��ʱ���󴥷�
}


#define USART_FH_len 6	//����֡ͷ����
char USART_FH[USART_FH_len]={'$','G','N','G','G','A'};//����֡ͷ
#define USART_FT_len 2	//����֡β����
uint8_t USART_FT[USART_FT_len]={0X0D,0X0A};//����֡β
uint8_t GPS_COMMA=',';
uint8_t GPS_STOP='*';
gps_t GNGGA_DATA[16];
void USART2_IRQHandler(void)
{
	static uint8_t chartoreceive = 0;
	static uint16_t count=0;
	static uint8_t degree=0;	
	static uint8_t count_comma=0;
	if((USART_VICE->ISR & 1<<5) == 1<<5)//���ռĴ������ݲ�Ϊ��
	{
		chartoreceive = (uint8_t)(USART_VICE->RDR);
		if(degree==USART_FH_len)//�Ѿ�����֡ͷ
		{
			q_side_buf[Q_Side.rear].data[count]=chartoreceive;		
			count++;
			if(chartoreceive==USART_FT[degree-USART_FH_len])//ͻȻ����֡β�ĵ�һ֡
			{
				degree++;
			}
			//����Ϊ���ݽ���
			if((chartoreceive==GPS_COMMA)||(chartoreceive==GPS_STOP))
			{
				count_comma++;
			}	
			else
			{
				GNGGA_DATA[count_comma].data[GNGGA_DATA[count_comma].len]=chartoreceive;
				GNGGA_DATA[count_comma].len++;
			}
			//����λ���ݽ���			
		}
		else
		{
			if(degree+1<USART_FH_len+1)//���֡ͷ   //��û��֡ͷʱ���ж� degree_dr< 0ʱ���������ᾯ�棬���+1
			{
				if(chartoreceive==USART_FH[degree])//����֡ͷ��һ֡
				{
					q_side_buf[Q_Side.rear].data[count]=chartoreceive;//��¼֡ͷ������
					count++;
					degree++;
				}
				else
				{
					if(chartoreceive==USART_FH[0])//֡ͷ����
					{
						count=1;
						degree=1;
					}
					else
					{
						count=0;
						degree=0;
					}
				}
			}
			else if((degree>USART_FH_len)&&(degree<USART_FH_len+USART_FT_len))//����֡β�ĵ�һ֡�󣬿�ʼ���֡β������֡
			{
				q_side_buf[Q_Side.rear].data[count]=chartoreceive;//��¼���ݶλ�֡β����
				count++;	
				if(chartoreceive==USART_FT[degree-USART_FH_len])//��⵽֡β������֡
				{
					degree++;
				}
				else
				{
					if(chartoreceive==USART_FT[0])//֡β����
					{
						degree=USART_FH_len+1;
					}
					else
					{
						degree=USART_FH_len;
					}
				}
			}
		}
		if(degree==USART_FH_len+USART_FT_len)//�Ѿ�����֡ͷ+֡β
		{
			q_side_buf[Q_Side.rear].length=count;
			rear_inc(&Q_Side);
			F_TASK_USART_VICE++;//���յ�����������һ�����ݺ󣬴�������		
			count=0;
			degree=0;	
			GNGGA_DATA[count_comma].len--;//��0x0D��ȥ
			count_comma=0;				
		}		
	}
	else
	{
		//NVIC_DisableIRQ(USART2_IRQn); 
	}	
	if((USART_VICE->ISR & (1<<3)) == (1<<3))//ORE
	{
		USART_VICE->ICR =1<<3;
	}
}
uint16_t char_to_hex( uint8_t* ch,uint8_t len)
{
	uint8_t value = 0,i;
	uint16_t sum=0;
	for(i=0;i<len;i++)
	{
		if(*ch >= 0x30 && *ch <=0x39)
		{
			value = *ch - 0x30;
		}
		else if(*ch >= 0x41 && *ch <= 0x46)
		{
			value = *ch - 0x37;
		}
		else if(*ch >= 0x61 && *ch <= 0x66)
		{
			value = *ch - 0x57;
		}
		sum=sum*16+value;
		ch++;
	}
	return sum;
}
void TASK_USART_VICE(void)//USART_VICE   USART_MAIN
{
	uint8_t i=1,xor_hex=0;
	
	for(i=1;q_side_buf[Q_Side.front].data[i]!='*';i++)
	{
		xor_hex^=q_side_buf[Q_Side.front].data[i];
	}
	memset(q_side_buf[Q_Side.front].data,0,q_side_buf[Q_Side.front].length);
	q_side_buf[Q_Side.front].length=0;
	front_inc(&Q_Side);	

	if(xor_hex==char_to_hex(GNGGA_DATA[15].data,2))//У��
	{
		GNGGA_DATA[2].data[GNGGA_DATA[2].len]=GNGGA_DATA[3].data[0];
		GNGGA_DATA[2].len++;
		mcu_dp_string_update(DPID_LATITUDE,GNGGA_DATA[2].data,GNGGA_DATA[2].len);
		
		GNGGA_DATA[4].data[GNGGA_DATA[4].len]=GNGGA_DATA[5].data[0];
		GNGGA_DATA[4].len++;
		mcu_dp_string_update(DPID_LONGITUDE,GNGGA_DATA[4].data,GNGGA_DATA[4].len);
		
		for(i=1;i<16;i++)
		{
			memset(GNGGA_DATA[i].data,0,GNGGA_DATA[i].len);
			GNGGA_DATA[i].len=0;
		}
	}

}

