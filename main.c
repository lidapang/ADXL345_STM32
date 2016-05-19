/*
>File name: main.c
>Authot : zyx
>Mail : zhang-yx10@foxmail.com
>Create Time : Thursday , May 19, 2016
*/
#include "stm32f10x.h"
#include "stdio.h"

/*USART*/
#define USART2_GPIO              GPIOA
#define USART2_CLK               RCC_APB1Periph_USART2
#define USART2_GPIO_CLK          RCC_APB2Periph_GPIOA
#define USART2_RxPin             GPIO_Pin_3
#define USART2_TxPin             GPIO_Pin_2
#define USART2_IRQn              USART2_IRQn
#define USART2_IRQHandler        USART2_IRQHandler

/* SPI2  AXDL345 */
//PB12 PB13 PB14 PB15
//NSS  SCK  MISO MOSI 
#define SPI2_GPIO                 GPIOB
#define SPI2_CLK                  RCC_APB1Periph_SPI2
#define SPI2_GPIO_CLK             RCC_APB2Periph_GPIOB 
#define SPI2_NSS_Pin              GPIO_Pin_12
#define SPI2_SCK_Pin              GPIO_Pin_13
#define SPI2_MISO_Pin             GPIO_Pin_14
#define SPI2_MOSI_Pin             GPIO_Pin_15

/* ADXL345 */
#define OFSX 0x1E
#define OFSY 0x1F
#define OFSZ 0x20
#define POWER_CTL 0x2D
#define INT_ENABLE 0x2E
#define INT_MAP 0x2F
#define ACT_INACT_CTL 0x27
#define DATA_FORMAT 0x31
#define BW_RATE 0x2C
#define THRESH_ACT 0x24
#define DATA_FORMAT 0x31
#define DATAX0 0x32
#define DATAX1 0x33
#define DATAY0 0x34
#define DATAY1 0x35
#define DATAZ0 0x36
#define DATAZ1 0x37


void RCC_Configuration(void);			
void GPIO_Configuration(void);				
void NVIC_Configuration(void);		
void SPI_Congfiguration(void);
void USART_Configuration(void);
void EXTI0_Congfiguration(void);


/*ADXL345 function*/
void ADXL345_configuration(void);
void Axis_Data_Collect_345(void); //数据采集函数
uint8_t ADXL345_read_byte(uint8_t add);
void ADXL345_write_byte(uint8_t add,uint8_t val);
void ADXL345_AUTO_Adjust(char *xval,char *yval,char *zval);     //自动校准
void ADXL345_RD_Avval(short *x,short *y,short *z); //读平均值
void read_id(void);
void readall(void);

uint16_t RATE=0X02;  //0X02 0.39HZ ; 0x08 25hz; 0X0A 1OO,0X0E 1600HZ
void delay(void);
uint8_t accl_return[6];
uint16_t x,y,z;// 存放X,Y,Z轴的数据
int val[6];

int main()
{
  /* 系统时钟初始化 */
  RCC_Configuration();
  /* 中断管理器初始化 */
  NVIC_Configuration();
  /* GPIO初始化*/
  GPIO_Configuration();
  /* USART初始化*/
	USART_Configuration();
	/*SPI初始化*/
	SPI_Congfiguration();
  /*外部中断初始化*/
	EXTI0_Congfiguration();
	
	printf("ADXL345 system start\r\n");
	read_id();
	ADXL345_configuration();//adxl345配置
	readall();
	while(1){}
}
/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		 //开启GPIO B的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		 //开启GPIO A的时钟
	
	/*使能串口GPIO时钟*/
  RCC_APB2PeriphClockCmd(USART2_GPIO_CLK, ENABLE);
  /* Enable USART1 Clock */
  /*使能串口2时钟*/
  RCC_APB1PeriphClockCmd(USART2_CLK, ENABLE); 
	
	/*使能SPI2时钟*/
	RCC_APB1PeriphClockCmd(SPI2_CLK, ENABLE);
	/*使能SPI2 GPIO时钟*/	
  RCC_APB2PeriphClockCmd(SPI2_GPIO_CLK,ENABLE);

  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

/*******************************************************************************
* Function Name  : NVIC_Configuration EXTI0_Config EXTI8_Config
* Description    : Configures the nested vectored interrupt controller.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
  /* Enable CAN1 RX0 interrupt IRQ channel */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
	/*PB0  ADXL345 IRQ input */
	/* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}
/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;  //定义GPIO结构体
	//初始化 LED  PB3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				     //LED管脚 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;			 //将PB3 配置为通用推挽输出  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //IO口翻转速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //将上述设置赋予GPIOB
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);   //关键一步！！
	
	/*串口2 TX管脚配置*/
  /* Configure USART2 Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = USART2_TxPin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(USART2_GPIO, &GPIO_InitStructure); 
	

  /*串口2 RX管脚配置*/
  /* Configure USART2 Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = USART2_RxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(USART2_GPIO, &GPIO_InitStructure);


  /*SPI2 引脚配置 Master模式*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = SPI2_SCK_Pin | SPI2_MOSI_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(SPI2_GPIO, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =SPI2_MISO_Pin; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(SPI2_GPIO, &GPIO_InitStructure);
  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin=SPI2_NSS_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(SPI2_GPIO, &GPIO_InitStructure);

}
/*******************************************************************************
* Function Name  : USART_Configuration
* Description    : Configures the USART.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART_Configuration(void)
{   
	USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

  USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;	//USART2发送/接收模式
  USART_Init(USART2, &USART_InitStructure);
  USART_Cmd(USART2, ENABLE);
	
}
/*******************************************************************************
* Function Name  : SPI_Congfiguration
* Description    : Configures the SPI.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_Congfiguration(void)
{ 
	SPI_InitTypeDef   SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStructure.SPI_CPOL=SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA=SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;//SPI_BaudRatePrescaler_256;  
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);
}
/*******************************************************************************
* Function Name  : EXTI0_Congfiguration
* Description    : Configures the EXTI0.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI0_Congfiguration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;  
	EXTI_InitTypeDef   EXTI_InitStructure;
  //PB0 ADXL345 IRQ ;
  /* Configure PB.00 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* Connect EXTI0 Line to PB.00 pin */
	//change to pb0
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}


void delay(void)
{
   int i,j;
   for(j=0;j<100;j++)
   for(i=0;i<3000;i++);
}

void delay_m(void)
{
  int i;
  for(i=0;i<500;i++); 
}

void ADXL345_configuration(void)
{
	delay();
	ADXL345_write_byte(DATA_FORMAT,0x08); //数据通信格式;设置为自检功能禁用,4线制SPI接口,低电平中断输出配合上升沿触发,13位全分辨率,输出数据右对齐,2g量程
	printf("format %2X\r\n",ADXL345_read_byte(0x31));
  
//   ADXL345_write_byte(0x1E,0x00); //X 偏移量 根据测试传感器的状态写入pdf29页 0X00 (15.6mg/LSB)
//   ADXL345_write_byte(0x1F,0x00); //Y 偏移量 根据测试传感器的状态写入pdf29页 0X00 (15.6mg/LSB)
  ADXL345_write_byte(OFSZ,0x08); //Z 偏移量 根据测试传感器的状态写入pdf29页 0X00 (15.6mg/LSB)
//   ADXL345_write_byte(0x21,0x00);  //敲击延时0:禁用; (1.25ms/LSB)
//   ADXL345_write_byte(0x22,0x00);  //检测第一次敲击后的延时0:禁用; (1.25ms/LSB)
//   ADXL345_write_byte(0x23,0x00);  //敲击窗口0:禁用; (1.25ms/LSB)
//   ADXL345_write_byte(0x24,0x01);  //保存检测活动阀值; (62.5mg/LSB)
//   ADXL345_write_byte(0x25,0x01);  //保存检测静止阀值; (62.5mg/LSB)
//   ADXL345_write_byte(0x26,0x2B);  //检测活动时间阀值; (1s/LSB)
  ADXL345_write_byte(ACT_INACT_CTL,0x70);  //Axis enable control for activity and inactivity detection
//   ADXL345_write_byte(0x28,0x09);  //自由落体检测推荐阀值; (62.5mg/LSB)
//   ADXL345_write_byte(0x29,0xFF);  //自由落体检测时间阀值,设置为最大时间; (5ms/LSB)
//   ADXL345_write_byte(0x2A,0x80);  //
//   ADXL345_read_byte(0x2B);    //只读寄存器,状态读取
  ADXL345_write_byte(BW_RATE,RATE); //速率设定为 0x08 25hz， 0X0A 1OO,0X0E 1600HZ
   
  
//	ADXL345_read_byte(0x31);    //只读寄存器,状态读取
//	ADXL345_write_byte(0x38,0x00);  //FIFO模式设定,Stream模式，触发连接INT1,31级样本缓冲
//	ADXL345_read_byte(0x39);    //只读寄存器,状态读取
	
	//before powerctl and dataready
	ADXL345_write_byte(POWER_CTL,0x08); //选择电源模式关闭自动休眠,休眠,唤醒功能参考pdf24页
	printf("format %2X\r\n",ADXL345_read_byte(0x2D));
  
	ADXL345_write_byte(INT_ENABLE,0x80);  //使能 DATA_READY
	printf("int_enable %2X\r\n",ADXL345_read_byte(0x2E));
	
	ADXL345_write_byte(INT_MAP,0x00);//中断映射控制，映射到int1管脚
	printf("int_map %2X\r\n",ADXL345_read_byte(0x2F));

	printf("adxl config ok\r\n");
}

void Axis_Data_Collect_345(void)//数据采集函数
{
 //		依次读取6个数据寄存器，注意延迟
	accl_return[0]=ADXL345_read_byte(DATAX0);//DATAX0
	accl_return[1]=ADXL345_read_byte(DATAX1);//DATAX1
	accl_return[2]=ADXL345_read_byte(DATAY0);//DATAY0
	accl_return[3]=ADXL345_read_byte(DATAY1);//DATAY1
	accl_return[4]=ADXL345_read_byte(DATAZ0);//DATAZ0
	accl_return[5]=ADXL345_read_byte(DATAZ1);//DATAZ1
	
	x = (accl_return[1] << 8) + accl_return[0];	
	printf("X=%02x\t",x);
	y = (accl_return[3] << 8) + accl_return[2];
	printf("Y=%02x\t",y);
	z = (accl_return[5] << 8) + accl_return[4];
	printf("Z=%02x\r\n",z);

}	


uint8_t ADXL345_read_byte(uint8_t add)//spi读函数
{ 
		u8 temp;
		delay_m();
    GPIO_ResetBits(SPI2_GPIO ,SPI2_NSS_Pin);
    
    while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
    SPI_I2S_SendData(SPI2,(add|0x80)<<8|0x00);//参见adxl345发送序列，读操作第一位msb为高，而后是地址
    while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE)==RESET);
    temp=SPI_I2S_ReceiveData(SPI2)&0xff;
    GPIO_SetBits(SPI2_GPIO ,SPI2_NSS_Pin);
   
    return temp;
}

void ADXL345_write_byte(uint8_t add,uint8_t val)//spi写函数
{	u8 temp;
	
	delay_m();
  GPIO_ResetBits(SPI2_GPIO ,SPI2_NSS_Pin);
  while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET);
  SPI_I2S_SendData(SPI2,add<<8|val);//将地址左移8位
  while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE)==RESET);
  temp=SPI_I2S_ReceiveData(SPI2)&0xff;
  GPIO_SetBits(SPI2_GPIO ,SPI2_NSS_Pin);
	SPI_I2S_ReceiveData(SPI2)&0xff;
  if(temp){printf("writeok\r\n");}
}

void read_id(void)//读取device id函数
{	u16 id;	

		ADXL345_read_byte(0x00);

		id=ADXL345_read_byte(0x00);

	printf("DEVICE ID  :%02X\r\n",id);
}	

void readall(void)//读取数据寄存器函数
{	

	val[0]=ADXL345_read_byte(0x32);
	val[1]=ADXL345_read_byte(0x33);
	val[2]=ADXL345_read_byte(0x34);
	val[3]=ADXL345_read_byte(0x35);
	val[4]=ADXL345_read_byte(0x36);
	val[5]=ADXL345_read_byte(0x37);

}

void ADXL345_AUTO_Adjust(char *xval,char *yval,char *zval)
{
	short tx,ty,tz;
	u8 i;
	short offx=0,offy=0,offz=0;

	//=====???=====
	ADXL345_write_byte(POWER_CTL,0x00);     //???????.
	ADXL345_write_byte(INT_ENABLE,0x00);    //DATA_READY ????
	delay();
	ADXL345_write_byte(DATA_FORMAT,0X2F); //???????,13?????,???????,16g??
	ADXL345_write_byte(BW_RATE,0x0A);  //???????100Hz


	ADXL345_write_byte(THRESH_ACT,0x01);    //????????187.5mg?????
	// ADXL345_write_byte(DUR,0xF0);       //????10*625uS
	ADXL345_write_byte(ACT_INACT_CTL,0xEE); //??X?Y?Z???Activity?Inactivity??
	ADXL345_write_byte(INT_MAP,0x00);       //???????1?
	ADXL345_write_byte(INT_ENABLE,0x80);    //DATA_READY????
	ADXL345_write_byte(POWER_CTL,0x28);     //????,????

	//=====================================
	ADXL345_write_byte(OFSX,0x00);
	ADXL345_write_byte(OFSY,0x00);
	ADXL345_write_byte(OFSZ,0x05);
	delay();

	for(i=0;i<10;i++)
	{
		ADXL345_RD_Avval(&tx,&ty,&tz);
		offx+=tx;
		offy+=ty;
		offz+=tz;
	}   
	offx/=10;
	offy/=10;
	offz/=10;
	*xval=-offx/4;
	*yval=-offy/4;
	*zval=-(offz-256)/4;  
	ADXL345_write_byte(OFSX,*xval);
	ADXL345_write_byte(OFSY,*yval);
	ADXL345_write_byte(OFSZ,*zval);

}
void ADXL345_RD_Avval(short *x,short *y,short *z)
{
       short tx=0,ty=0,tz=0;        
       u8 i;  
       for(i=0;i<10;i++)
       {
              Axis_Data_Collect_345();
              delay();
              tx+=(short)*x; ty+=(short)*y; tz+=(short)*z;      
       }
       *x=tx/10; *y=ty/10; *z=tz/10;
} 




#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
