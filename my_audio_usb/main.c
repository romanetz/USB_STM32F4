/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4_discovery_audio_codec.h"
#include "usbd_audio_core.h"
#include "usbd_conf.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include <sys/stat.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment = 4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;
s16 Audio[192] __attribute__ ((aligned(4)));//384 samples=768 bytes. in order LRLR...
s16 Audio2[192] __attribute__ ((aligned(4)));//384 samples=768 bytes. in order LRLR...
extern float MPX_buf[4608] __attribute__ ((aligned(4)));	//4608*4 bytes MPX signal buffer
extern uint16_t DAC_buf[4608] __attribute__ ((aligned(4)));		//4608*4 bytes
extern uint32_t DDS_buf[4608] __attribute__ ((aligned(4)));		//4608 bytes
extern const s16 LUT[16384];
extern volatile uint8_t audio_buffer_fill;
volatile uint8_t * xbuf;
uint32_t fs=USBD_AUDIO_FREQ;
uint8_t mult_factor=8;
extern volatile uint8_t mpx_buff_fill=0;
uint16_t PrescalerValue = 0;
uint16_t bufsize=AUDIO_OUT_PACKET/2;
__IO uint8_t UserButtonPressed = 0;
extern float b;
extern float d;
extern float a;
extern uint8_t use_preemph;
extern uint32_t dphase;
extern float dev;
__IO uint32_t TimingDelay;
//extern uint8_t * xbuf;
uint8_t Buffer[6];

/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
volatile uint32_t mmm,nnn;
RCC_ClocksTypeDef RCC_Clocks;

uint16_t fleft=440;
uint16_t fright=261;
uint8_t test[10];
float *var;
int main(void)
{
	uint16_t i;
	uint32_t f0;
	uint8_t textbuf[30];
  SystemInit();

  
  /* Initialize LEDs and User_Button on STM32F4-Discovery --------------------*/
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
  
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);

  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
  InitDDSPorts();
  InitDDSSPI(SPI_BaudRatePrescaler_2);
  TMR2_Config(fs*mult_factor/2, RCC_Clocks.PCLK1_Frequency);
  ad9951_init();
  f0=__REV(TIM2->CCR4);

  //SetI2SMCLK(fs);
  EVAL_AUDIO_Init(OUTPUT_DEVICE_HEADPHONE, 90, fs);

 //EVAL_AUDIO_Play(Audio,384);
 DAC_Config();
 Audio_DAC_play(DAC_buf,mult_factor*bufsize);
 TMR4_Config(DDS_buf,mult_factor*bufsize);
 TMR3_Config();
 mpx_init(mult_factor,bufsize/2);

 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

 USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &AUDIO_cb, &USR_cb);
  while(1)
  {

//while(mpx_buff_fill==0);
	      	//DDS is synchronous with DAC
	  if (mpx_buff_fill==1)
	  {
		  //if (UserButtonPressed&2)
		  //Sin_Gen (&Audio2[0],fs,fleft,fright,bufsize/2);
		  MPX_Gen (fs,mult_factor,&Audio2[0],MPX_buf,bufsize/2);
	      DAC_normalise(&MPX_buf[0],&DAC_buf[0],mult_factor*bufsize/2);
	      FM_MPX(f0,&MPX_buf[0],mult_factor*bufsize/2, &DDS_buf[0]);
	      mpx_buff_fill=0;

	  };
	  if (mpx_buff_fill==2)
	  {
		  //if (UserButtonPressed&2)
		  //Sin_Gen (&Audio2[96],fs,fleft,fright,bufsize/2);
		  MPX_Gen (fs,mult_factor,&Audio2[bufsize],&MPX_buf[mult_factor*bufsize/2],bufsize/2);
	  	  DAC_normalise(&MPX_buf[mult_factor*bufsize/2],&DAC_buf[mult_factor*bufsize/2],mult_factor*bufsize/2);
	      FM_MPX(f0,&MPX_buf[mult_factor*bufsize/2],mult_factor*bufsize/2, &DDS_buf[mult_factor*bufsize/2]);
	  	  mpx_buff_fill=0;

	  };

if (VCP_get_char(&test[0]))
  {STM_EVAL_LEDToggle(LED6);
	switch (test[0])
  {
  case 'S': d+=0.01; sprintf(textbuf,"+S %f\n\r",d); VCP_send_str(textbuf); break;
  case 's': d-=0.01; sprintf(textbuf,"-S %f\n\r",d); VCP_send_str(textbuf); break;
  case 'M': b+=0.01; sprintf(textbuf,"+M %f\n\r",b); VCP_send_str(textbuf); break;
  case 'm': b-=0.01; sprintf(textbuf,"-M %f\n\r",b); VCP_send_str(textbuf); break;
  case 'F': f0+=596523; sprintf(textbuf,"+Carrier frequency %d\n\r",f0); VCP_send_str(textbuf); break;
  case 'f': f0-=596523; sprintf(textbuf,"-Carrier frequency %d\n\r",f0); VCP_send_str(textbuf); break;
  case 'P': a+=0.001; sprintf(textbuf,"+Pilot level %f\n\r",a); VCP_send_str(textbuf); break;
  case 'p': a-=0.001; sprintf(textbuf,"-Pilot level %f\n\r",a); VCP_send_str(textbuf); break;
  case 'T': dphase+=1184; sprintf(textbuf,"+Pilot freq %u\n\r",dphase); VCP_send_str(textbuf); break;
  case 't': dphase-=1184; sprintf(textbuf,"-Pilot freq %u\n\r",dphase); VCP_send_str(textbuf); break;
  case 'D': dev+=5965.23235/32768.0; sprintf(textbuf,"+Carrier dev %f\n\r",a); VCP_send_str(textbuf); break;
  case 'd': dev-=5965.23235/32768.0; sprintf(textbuf,"-Carrier dev %f\n\r",a); VCP_send_str(textbuf); break;
 // case 'E': use_preemph=1; sprintf(textbuf,"Preemphasis on\n\r"); VCP_send_str(textbuf); break;
//  case 'e': use_preemph=0; sprintf(textbuf,"Preemphasis off\n\r"); VCP_send_str(textbuf); break;
  case ' ': sprintf(textbuf,"current: S: %f M: %f\n\r",d,b); VCP_send_str(textbuf); break;
  };
memset(test,0,30);
  };
  }; //while(1)
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
  */
void Delayms(__IO uint32_t nTime)
{
  TimingDelay = nTime;
  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}

void EVAL_AUDIO_TransferComplete_CallBack(uint32_t *pBuffer, uint32_t Size)
{
  	audio_buffer_fill=2;
	update_audio_buf();
	//Sin_Gen (&Audio[96],fs,fleft,fright,48);
	memcpy((uint8_t*)Audio+AUDIO_OUT_PACKET,xbuf,AUDIO_OUT_PACKET);
  	//if (!(UserButtonPressed&2))
  	memcpy((uint8_t*)Audio2+AUDIO_OUT_PACKET,(uint8_t*)Audio+AUDIO_OUT_PACKET,AUDIO_OUT_PACKET);
  	audio_buffer_fill=0;
}

void EVAL_AUDIO_HalfTransfer_CallBack(uint32_t *pBuffer, uint32_t Size)
{
	audio_buffer_fill=1;
	update_audio_buf();
	//Sin_Gen (&Audio[0],fs,fleft,fright,48);
	memcpy(Audio,xbuf,AUDIO_OUT_PACKET);
	//if (!(UserButtonPressed&2))
	memcpy(Audio2,Audio,AUDIO_OUT_PACKET);
	audio_buffer_fill=0;
}
/**
  * @brief  This function handles main DAC interrupt.
  * @param  None
  * @retval 0 if correct communication, else wrong communication
  */


void Audio_MAL_DAC_IRQHandler(void)
{
	  if (DMA_GetFlagStatus(AUDIO_DAC_DMA_STREAM, AUDIO_DAC_DMA_FLAG_TC) != RESET)
		{mpx_buff_fill=2;
		 DMA_ClearFlag(AUDIO_DAC_DMA_STREAM, AUDIO_DAC_DMA_FLAG_TC);};
	  if (DMA_GetFlagStatus(AUDIO_DAC_DMA_STREAM, AUDIO_DAC_DMA_FLAG_HT) != RESET)
			{mpx_buff_fill=1;DMA_ClearFlag(AUDIO_DAC_DMA_STREAM, AUDIO_DAC_DMA_FLAG_HT);};

}

   int _write (int fd, char *pBuffer, int size)
    {
       VCP_send_buffer(pBuffer, size);
       return 1;
    }


   int _read (int fd, char *pBuffer, int size)
    {
	   int done = VCP_get_string((uint8_t *) pBuffer);
	   if (done)
	                 return done;
    }

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
