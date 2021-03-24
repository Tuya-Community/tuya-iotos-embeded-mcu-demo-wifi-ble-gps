#include "USART.h"
#include "TIM.h"

void creat_sq(Queue* SQ)//设置队列参数
{
	SQ->front=0;
	SQ->rear=0;
	SQ->size=quene_main_buf_total;
}
void front_inc(Queue* SQ)//队首指针
{
	SQ->front++;
	if(SQ->front==SQ->size)
	{
		SQ->front=0;
	}
}
void rear_inc(Queue* SQ)//队尾指针
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
		while((MY_usart->ISR&1<<6)!=1<<6)//等待发送完毕
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
	RCC->APBRSTR2 &=~(1<<14);//恢复串口1
	RCC->IOPENR |= 1<<2;//使能GPIOC时钟
	GPIOC->MODER &=~(3<<8|3<<10);
	GPIOC->MODER |=2<<8|2<<10;//复用模式
	GPIOC->AFR[0] &=~(0xf<<16|0xf<<20);
	GPIOC->AFR[0] |=1<<16|1<<20;//选择复用功能AF1
	RCC->APBENR2 |=1<<14;//使能串口1时钟	
	USART_MAIN->BRR = 16000000 / bound; 
	USART_MAIN->CR1 |= 1<<0|1<<2|1<<3|1<<5;//串口使能，使能接收，使能发送,
	while((USART_MAIN->ISR & 1<<6) != 1<<6)//发送完成标志位
	{ 
		break;/* add time out here for a robust application */
	}	
	NVIC_SetPriority(USART1_IRQn, 1);
	NVIC_EnableIRQ(USART1_IRQn);
	creat_sq(&Q_Main);
	F_TASK_USART_MAIN=0;//清除定时器初始化时的误触发 必须清除！！！因为接收超时定时器开启时已经误触发一次
}
void USART1_IRQHandler(void)
{
	if((USART_MAIN->ISR & 1<<5) == 1<<5)//接收寄存器数据不为空
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
	RCC->APBRSTR1 &=~(1<<17);//恢复串口2
	RCC->IOPENR |= 1<<3;//使能GPIOD时钟
	GPIOD->MODER &=~(3<<10|3<<12);
	GPIOD->MODER |=2<<10|2<<12;//复用模式
	GPIOD->AFR[0] &=~(0xf<<20|0xf<<24);
	GPIOD->AFR[0] |=0<<20|0<<24;//选择复用功能AF0
	RCC->APBENR1 |=1<<17;//使能串口3时钟
	
	USART_VICE->BRR = 16000000 / bound; 
	USART_VICE->CR1 |= 1<<0|1<<2|1<<3|1<<5;//串口使能，使能接收，使能发送,
	while((USART_VICE->ISR & 1<<6) != 1<<6)//发送完成标志位
	{ 
		break;/* add time out here for a robust application */
	}	
	NVIC_SetPriority(USART2_IRQn, 1);
	NVIC_EnableIRQ(USART2_IRQn);	
	creat_sq(&Q_Side);
	F_TASK_USART_VICE=0;//清除定时器初始化时的误触发
}


#define USART_FH_len 6	//定义帧头长度
char USART_FH[USART_FH_len]={'$','G','N','G','G','A'};//定义帧头
#define USART_FT_len 2	//定义帧尾长度
uint8_t USART_FT[USART_FT_len]={0X0D,0X0A};//定义帧尾
uint8_t GPS_COMMA=',';
uint8_t GPS_STOP='*';
gps_t GNGGA_DATA[16];
void USART2_IRQHandler(void)
{
	static uint8_t chartoreceive = 0;
	static uint16_t count=0;
	static uint8_t degree=0;	
	static uint8_t count_comma=0;
	if((USART_VICE->ISR & 1<<5) == 1<<5)//接收寄存器数据不为空
	{
		chartoreceive = (uint8_t)(USART_VICE->RDR);
		if(degree==USART_FH_len)//已经满足帧头
		{
			q_side_buf[Q_Side.rear].data[count]=chartoreceive;		
			count++;
			if(chartoreceive==USART_FT[degree-USART_FH_len])//突然等于帧尾的第一帧
			{
				degree++;
			}
			//以下为数据解析
			if((chartoreceive==GPS_COMMA)||(chartoreceive==GPS_STOP))
			{
				count_comma++;
			}	
			else
			{
				GNGGA_DATA[count_comma].data[GNGGA_DATA[count_comma].len]=chartoreceive;
				GNGGA_DATA[count_comma].len++;
			}
			//以上位数据解析			
		}
		else
		{
			if(degree+1<USART_FH_len+1)//检测帧头   //当没有帧头时，判断 degree_dr< 0时，编译器会警告，因此+1
			{
				if(chartoreceive==USART_FH[degree])//满足帧头的一帧
				{
					q_side_buf[Q_Side.rear].data[count]=chartoreceive;//记录帧头段内容
					count++;
					degree++;
				}
				else
				{
					if(chartoreceive==USART_FH[0])//帧头误判
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
			else if((degree>USART_FH_len)&&(degree<USART_FH_len+USART_FT_len))//满足帧尾的第一帧后，开始检测帧尾的其他帧
			{
				q_side_buf[Q_Side.rear].data[count]=chartoreceive;//记录数据段或帧尾内容
				count++;	
				if(chartoreceive==USART_FT[degree-USART_FH_len])//检测到帧尾的其他帧
				{
					degree++;
				}
				else
				{
					if(chartoreceive==USART_FT[0])//帧尾误判
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
		if(degree==USART_FH_len+USART_FT_len)//已经满足帧头+帧尾
		{
			q_side_buf[Q_Side.rear].length=count;
			rear_inc(&Q_Side);
			F_TASK_USART_VICE++;//接收到满足条件的一包数据后，触发任务		
			count=0;
			degree=0;	
			GNGGA_DATA[count_comma].len--;//把0x0D舍去
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

	if(xor_hex==char_to_hex(GNGGA_DATA[15].data,2))//校验
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

