/*
>File name: main.c
>Authot : zyx
>Mail : zhang-yx10@foxmail.com
>Create Time : Thursday , May 19, 2016
*/
#include "stm32f10x.h"

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
// PB0 ADXL345 IRQ ; PB8 ADIS16240 IRQ
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
