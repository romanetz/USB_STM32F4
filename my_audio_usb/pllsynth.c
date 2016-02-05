/* PLL synteza SAA1057, LCD display */


#include "main.h"

#define PLL_SPI SPI2
#define PLL_SPI_CLK RCC_APB1Periph_SPI2
#define PLL_SPI_SCK_GPIO_CLK RCC_AHB1Periph_GPIOB
#define PLL_SPI_MOSI_GPIO_CLK RCC_AHB1Periph_GPIOB
#define PLL_SPI_CS_GPIO_CLK RCC_AHB1Periph_GPIOB

#define PLL_SPI_MOSI_SOURCE GPIO_PinSource15
#define PLL_SPI_MOSI_PIN GPIO_Pin_15

#define PLL_SPI_SCK_SOURCE GPIO_PinSource10
#define PLL_SPI_SCK_PIN GPIO_Pin_10

#define PLL_SPI_CS_PIN GPIO_Pin_8

#define PLL_SPI_SCK_AF GPIO_AF_SPI2
#define PLL_SPI_MOSI_AF GPIO_AF_SPI2
#define PLL_SPI_CS_GPIO_PORT GPIOB
#define PLL_SPI_MOSI_GPIO_PORT GPIOB
#define PLL_SPI_SCK_GPIO_PORT GPIOB



void InitDDSPorts(void)
{
	 GPIO_InitTypeDef GPIO_InitStructure;

	 /* Enable SCK, MOSI and MISO GPIO clocks */
		  RCC_AHB1PeriphClockCmd(PLL_SPI_SCK_GPIO_CLK | PLL_SPI_MOSI_GPIO_CLK, ENABLE);

		  /* Enable CS  GPIO clock */
		  RCC_AHB1PeriphClockCmd(PLL_SPI_CS_GPIO_CLK | RCC_AHB1Periph_GPIOC, ENABLE);

		  /* Enable INT1 GPIO clock */
		  //RCC_AHB1PeriphClockCmd(LIS302DL_SPI_INT1_GPIO_CLK, ENABLE);

		  /* Enable INT2 GPIO clock */
		  //RCC_AHB1PeriphClockCmd(LIS302DL_SPI_INT2_GPIO_CLK, ENABLE);

		  //GPIO_PinAFConfig(LIS302DL_SPI_MISO_GPIO_PORT, LIS302DL_SPI_MISO_SOURCE, LIS302DL_SPI_MISO_AF);

		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

		  /* SPI SCK pin configuration as a GPIO for reset purpose*/
		  GPIO_InitStructure.GPIO_Pin = PLL_SPI_SCK_PIN;
		  GPIO_Init(PLL_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

		  /* SPI MISO pin configuration */
		  //GPIO_InitStructure.GPIO_Pin = LIS302DL_SPI_MISO_PIN;
		  //GPIO_Init(LIS302DL_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

		  /* Configure GPIO PIN for PLL Chip select */
		  GPIO_InitStructure.GPIO_Pin = PLL_SPI_CS_PIN;
		  GPIO_Init(PLL_SPI_CS_GPIO_PORT, &GPIO_InitStructure);

		  /* Configure GPIO PIN for DDS Reset select */
		  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
 		  GPIO_Init(GPIOC, &GPIO_InitStructure);

 		  /* Reset pulse */

 		  Delayms(1);
 		  //GPIO_ResetBits(GPIOC, GPIO_Pin_11);
 		  GPIO_SetBits(GPIOC, GPIO_Pin_11);

 		  /* SCK pulse */
 /*		  GPIO_SetBits(PLL_SPI_SCK_GPIO_PORT, PLL_SPI_SCK_PIN);
 		  Delay(1);
 		  GPIO_ResetBits(PLL_SPI_SCK_GPIO_PORT, PLL_SPI_SCK_PIN);
*/
 		  /* Freq_update pulse */
/* 		  GPIO_SetBits(PLL_SPI_CS_GPIO_PORT, PLL_SPI_CS_PIN);
 		  Delay(1);
 		  GPIO_ResetBits(PLL_SPI_CS_GPIO_PORT, PLL_SPI_CS_PIN);*/

		  GPIO_PinAFConfig(PLL_SPI_SCK_GPIO_PORT, PLL_SPI_SCK_SOURCE, PLL_SPI_SCK_AF);
		  GPIO_PinAFConfig(PLL_SPI_MOSI_GPIO_PORT, PLL_SPI_MOSI_SOURCE, PLL_SPI_MOSI_AF);

		  /* Reinitialize SPI SCK as SPI AF port */
		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

		  /* SPI  MOSI pin configuration */
		  GPIO_InitStructure.GPIO_Pin = PLL_SPI_MOSI_PIN;
		  GPIO_Init(PLL_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

		  /* SPI SCK pin configuration */
		  GPIO_InitStructure.GPIO_Pin = PLL_SPI_SCK_PIN;
		  GPIO_Init(PLL_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

		  /* Configure GPIO PINs to detect Interrupts */
		 /* GPIO_InitStructure.GPIO_Pin = LIS302DL_SPI_INT1_PIN;
		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		  GPIO_Init(LIS302DL_SPI_INT1_GPIO_PORT, &GPIO_InitStructure);

		  GPIO_InitStructure.GPIO_Pin = LIS302DL_SPI_INT2_PIN;
		  GPIO_Init(LIS302DL_SPI_INT2_GPIO_PORT, &GPIO_InitStructure);*/

}

void InitDDSSPI(uint16_t prescaler)
{
    SPI_InitTypeDef PLL_SPI_InitStruct;
    /* Enable the SPI periph clock*/
	RCC_APB1PeriphClockCmd(PLL_SPI_CLK, ENABLE);
	/*Init PLL SPI */
	SPI_I2S_DeInit(PLL_SPI);
	PLL_SPI_InitStruct.SPI_Direction=SPI_Direction_1Line_Tx;
	PLL_SPI_InitStruct.SPI_Mode=SPI_Mode_Master;
	PLL_SPI_InitStruct.SPI_DataSize=SPI_DataSize_8b;
	PLL_SPI_InitStruct.SPI_CPOL=SPI_CPOL_Low;
	PLL_SPI_InitStruct.SPI_CPHA=SPI_CPHA_1Edge;
	PLL_SPI_InitStruct.SPI_NSS=SPI_NSS_Soft;
	PLL_SPI_InitStruct.SPI_FirstBit=SPI_FirstBit_MSB;
	PLL_SPI_InitStruct.SPI_BaudRatePrescaler=prescaler;
	SPI_Init(PLL_SPI, &PLL_SPI_InitStruct);
	SPI_Cmd(PLL_SPI,ENABLE);
}

void PLLSendFreq(uint16_t pll_div)
{
uint16_t tmp,tmp2;
tmp=pll_div&0xffff;
tmp=(tmp>272)? tmp:272;
while(SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_BSY));
while(!SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_TXE));
SPI_I2S_SendData(PLL_SPI,0x28);//IN1 word
while(SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_BSY));
Delayms(1);
GPIO_SetBits(PLL_SPI_CS_GPIO_PORT, PLL_SPI_CS_PIN); //Set PLL CS
Delayms(1);
while(!SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_TXE));
SPI_I2S_SendData(PLL_SPI,(tmp&0xff));//LSB of frequency word
while(!SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_TXE));
SPI_I2S_SendData(PLL_SPI,((uint8_t *)&tmp)[1]);//MSB of frequency word
while(!SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_TXE));
SPI_I2S_SendData(PLL_SPI,0x12); //FM mode, prediv/2 100 kHz ref frequency 4.5M XTAL
while(SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_BSY)); //Wait till the end of transmission
Delayms(1);
GPIO_ResetBits(PLL_SPI_CS_GPIO_PORT, PLL_SPI_CS_PIN);
}

void PLLSendCtrlWord(uint32_t ctrl_word)
{uint16_t tmp,tmp2;
while(SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_BSY));
while(!SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_TXE));
SPI_I2S_SendData(PLL_SPI,0x29);//IN2 word
while(SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_BSY));
Delayms(1);
GPIO_SetBits(PLL_SPI_CS_GPIO_PORT, PLL_SPI_CS_PIN); //Set PLL CS
Delayms(1);
while(!SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_TXE));
SPI_I2S_SendData(PLL_SPI,((uint8_t *)&ctrl_word)[0]);//
while(!SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_TXE));
SPI_I2S_SendData(PLL_SPI,((uint8_t *)&ctrl_word)[1]);//DO=unlock, dead zone DZD
while(!SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_TXE));
SPI_I2S_SendData(PLL_SPI,((uint8_t *)&ctrl_word)[2]);
while(SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_BSY)); //Wait till the end of transmission
Delayms(1);
GPIO_ResetBits(PLL_SPI_CS_GPIO_PORT, PLL_SPI_CS_PIN);
}

#define CFR1 0x0
#define CFR2 0x1
#define FTW0 0x4
#define AUTO_OSK	(1)
#define OSKEN		(1 << 1)
#define LOAD_ARR	(1 << 2)
#define AUTO_SYNC	(1 << 7)
#define LSB_FST		(1)
#define SDIO_IPT	(1 << 1)
#define CLR_PHA		(1 << 2)
#define SINE_OPT	(1 << 4)
#define ACLR_PHA	(1 << 5)
#define VCO_RANGE	(1 << 2)
#define CRS_OPT		(1 << 1)
#define nSYNC_OPT	(1 << 1)
#define HMANU_SYNC	(1 << 2)
#define HSPD_SYNC	(1 << 3)
#define XTAL_FREQ	80000
#define PLL_MULT	9
uint32_t FTW;

uint8_t cfr[5];

void ad9951_init(void)
{
	int ret;
	uint64_t tmp64;
	uint8_t i;
	uint32_t carrier=101200;

	cfr[0] = CFR1;
	cfr[1] = 0;
	cfr[2] = 0;
	cfr[3] = SDIO_IPT;
	cfr[4] = nSYNC_OPT;
	for (i=0;i<5;i++)
		{while(SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_BSY));
		SPI_I2S_SendData(PLL_SPI,cfr[i]);};

	while(SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_BSY));
	GPIO_SetBits(PLL_SPI_CS_GPIO_PORT, PLL_SPI_CS_PIN); //Set PLL CS

	GPIO_ResetBits(PLL_SPI_CS_GPIO_PORT, PLL_SPI_CS_PIN);

	cfr[0] = CFR2;
	cfr[1] = 0;
	cfr[2] = 0;
	cfr[3] = (PLL_MULT<<3)|7;
	for (i=0;i<4;i++)
		{while(SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_BSY));
		SPI_I2S_SendData(PLL_SPI,cfr[i]);};

	while(SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_BSY));
	GPIO_SetBits(PLL_SPI_CS_GPIO_PORT, PLL_SPI_CS_PIN); //Set PLL CS

	GPIO_ResetBits(PLL_SPI_CS_GPIO_PORT, PLL_SPI_CS_PIN);
#define MAX32 4294967296UL

	tmp64=((MAX32)*(uint64_t)carrier)/(XTAL_FREQ*PLL_MULT);
	FTW=(uint32_t)tmp64&0xffffffff;
	TIM2->CCR4=__REV(FTW);
	cfr[0]=FTW0;
	//*(uint32_t *)(&cfr[1])=*(uint32_t *)(&TIM2->CCR4);
	cfr[1]=((FTW>>24)&0xFF);
	cfr[2]=((FTW>>16)&0xFF);
	cfr[3]=((FTW>>8)&0xFF);
	cfr[4]=(FTW&0xFF);
		for (i=0;i<5;i++)
		{while(SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_BSY));
		SPI_I2S_SendData(PLL_SPI,cfr[i]);};

		while(SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_BSY));
	GPIO_SetBits(PLL_SPI_CS_GPIO_PORT, PLL_SPI_CS_PIN); //Set PLL CS

	GPIO_ResetBits(PLL_SPI_CS_GPIO_PORT, PLL_SPI_CS_PIN);
}

void ad9951_sendfreq(uint32_t freq)
{
	uint8_t i;
	uint32_t tmp;
	FTW=freq*10737;
	cfr[0]=FTW0;
	tmp=__REV(FTW);
	*(uint32_t *)(&cfr[1])=*(uint32_t *)(&tmp);

	for (i=0;i<5;i++)
	{while(SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_BSY));
		SPI_I2S_SendData(PLL_SPI,cfr[i]);};

	while(SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_BSY));
	GPIO_SetBits(PLL_SPI_CS_GPIO_PORT, PLL_SPI_CS_PIN); //Set PLL CS

	GPIO_ResetBits(PLL_SPI_CS_GPIO_PORT, PLL_SPI_CS_PIN);

}
/*
#define DDS_CLOCK 125000000
void DDSSendFreq(uint32_t freq)
{uint8_t i;

uint32_t FTW= ((uint64_t)freq*4294967295)/DDS_CLOCK;
TIM2->CCR4=0x45A1CAC0;
//TIM2->CCR4=FTW;
//uint32_t FTW= 0x45A1CAC0;//freq*4294967295/DDS_CLOCK;
for (i=0;i<4;i++)
{while(((PLL_SPI->SR)&SPI_I2S_FLAG_BSY ));
//while(!SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_TXE));
SPI_I2S_SendData(PLL_SPI,FTW&0xFF);//IN1 word
FTW>>=8;
};
//while(SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_BSY));
//while(!SPI_I2S_GetFlagStatus(PLL_SPI, SPI_I2S_FLAG_TXE));
while(((PLL_SPI->SR)&SPI_I2S_FLAG_BSY ));
SPI_I2S_SendData(PLL_SPI,0);//IN1 word

GPIO_SetBits(PLL_SPI_CS_GPIO_PORT, PLL_SPI_CS_PIN); //Set PLL CS
Delay(1);
GPIO_ResetBits(PLL_SPI_CS_GPIO_PORT, PLL_SPI_CS_PIN);
}
*/
/*
void DDS_DMA_config(uint32_t * buf, uint8_t * buf1, uint16_t bufsize)
{

  DMA_InitTypeDef DMA_InitStructure;

  //DMA to send 0x04 command to ad9951
  /* DMA1_Stream0 channel2 configuration - DMA requestor is TIM4.CH1.OC**************************************/
  /* Enable DMA1_Stream0 */
/*  DMA_Cmd(DMA1_Stream0, DISABLE);
  DMA_DeInit(DMA1_Stream0);
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) buf;
  DMA_InitStructure.DMA_BufferSize = bufsize;
  DMA_InitStructure.DMA_PeripheralBaseAddr = &(SPI2->DR); //Only channel1 is loaded
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single ;
  DMA_Init(DMA1_Stream0, &DMA_InitStructure);

  //DMA to send FTW to ad9951
  /* DMA1_Stream3 channel2 configuration - DMA requestor is TIM4.CH2.OC**************************************/
 /* DMA_DeInit(DMA1_Stream3);
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) buf;
  DMA_InitStructure.DMA_BufferSize = bufsize*4;
  DMA_InitStructure.DMA_PeripheralBaseAddr = &(SPI2->DR); //Only channel1 is loaded
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
  DMA_Init(DMA1_Stream3, &DMA_InitStructure);
  //DMA_FlowControllerConfig(DMA1_Stream3, DMA_FlowCtrl_Memory);*/
/*DMA_ITConfig(DMA1_Stream0,DMA_IT_TC|DMA_IT_HT,ENABLE);

  NVIC_EnableIRQ(DMA1_Stream0_IRQn);

  /* Enable DMA1_Stream0 */
/*DMA_Cmd(DMA1_Stream0, ENABLE);

  /* Enable DMA1_Stream3 */
 // DMA_Cmd(DMA1_Stream3, ENABLE);
/*
}*/

//DMA IRQ handler
//it is called every half if buffer is sent
void DMA1_Stream0_IRQHandler(void)
{
   if(DMA_GetITStatus(DMA1_Stream0,DMA_IT_HTIF0))	 /* DMA1 通道5 半传输中断 */
   {
DMA_ClearITPendingBit(DMA1_Stream0,DMA_IT_HTIF0);  /* DMA1 通道5 全局中断 */
	//	 dds_buffer_fill = 1;
   }
   if(DMA_GetITStatus(DMA1_Stream0,DMA_IT_TCIF0))  /* DMA1 通道5 传输完成中断 */
   {
DMA_ClearITPendingBit(DMA1_Stream0,DMA_IT_TCIF0); /* DMA1 通道5 全局中断 */

		// dds_buffer_fill = 2;
   }
}
