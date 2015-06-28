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
#include "usbd_usr.h"
#include "usbd_desc.h"




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
extern const s16 LUT[16384];
extern volatile uint8_t audio_buffer_fill;
volatile uint8_t * xbuf;
uint32_t fs=48000;
uint8_t mult_factor=8;
extern volatile uint8_t mpx_buff_fill=0;
uint16_t PrescalerValue = 0;
uint16_t bufsize=96;
__IO uint8_t UserButtonPressed = 0;

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

int main(void)
{
  uint16_t i;
  uint16_t fleft=440;
  uint16_t fright=261;
uint32_t f0;
  uint32_t phL1=0,phR1=0,phL2=0,phR2=0;
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
  //PLLSendFreq(freq_khz/(50*2));
  TMR2_Config(fs*mult_factor/2, RCC_Clocks.PCLK1_Frequency);
  //DDSSendFreq(freq);
  ad9951_init();
  f0=__REV(TIM2->CCR4);
  //ad9951_sendfreq(freq);
  //PLLSendCtrlWord(0x0032F0);
  EVAL_AUDIO_Init(OUTPUT_DEVICE_HEADPHONE, 70, fs);
  //SetI2SMCLK(fs);
  //Sin_Gen (Audio,fs,fleft,fright,96);
  //Sin_Gen (&Audio2[0],fs,fleft,fright,96);
  //MPX_Gen (fs,mult_factor,Audio,MPX_buf,192);
  //EVAL_AUDIO_Play(Audio,384);
  DAC_Config();
 Audio_DAC_play(DAC_buf,mult_factor*bufsize);
 TMR4_Config(DDS_buf,mult_factor*bufsize);
 TMR3_Config();
 mpx_init(mult_factor,bufsize/2);
 TMR6_Config(1000000UL, RCC_Clocks.PCLK1_Frequency*2);

 USBD_Init(&USB_OTG_dev,
            USB_OTG_FS_CORE_ID,
            &USR_desc,
            &AUDIO_cb,
            &USR_cb);
  while(1)
  {

	    if (audio_buffer_fill==1)
	      			{
		  //Sin_Gen (&Audio[0],fs,fleft,fright,48);
	    	memcpy(Audio,xbuf,192); //moved to codec routines
	    	//if (!(UserButtonPressed&2))
	    	memcpy(Audio2,Audio,192);
		    audio_buffer_fill=0;
	      			};
	      	if (audio_buffer_fill==2)
	      		  	{
	     		//Sin_Gen (&Audio[96],fs,fleft,fright,48);
	      		memcpy((uint8_t*)Audio+192,xbuf,192);// moved to codec routines
	      		//if (!(UserButtonPressed&2))
	      		memcpy((uint8_t*)Audio2+192,(uint8_t*)Audio+192,192);
	      		audio_buffer_fill=0;
	      	    	};
	      	//while (audio_buffer_fill==0);

	/*if (audio_buffer_fill==1)
	 	  	      			{memcpy(Audio,xbuf,192);audio_buffer_fill=0;
	 	  	      			};
	 	   	if (audio_buffer_fill==2)
	 	  	      	    	{memcpy((uint8_t*)Audio+192,xbuf,192);audio_buffer_fill=0;
	 	  	      	    	};

*/
//while(mpx_buff_fill==0);
	      	//DDS is synchronous with DAC
	  if (mpx_buff_fill==1)
	  {
		 // if (UserButtonPressed&2)
		 // Sin_Gen (&Audio2[0],fs,fleft,fright,48);

		  MPX_Gen (fs,mult_factor,&Audio2[0],MPX_buf,bufsize/2);
	      DAC_normalise(&MPX_buf[0],&DAC_buf[0],mult_factor*bufsize/2);
	      FM_MPX(f0,&MPX_buf[0],mult_factor*bufsize/2, &DDS_buf[0]);

	      mpx_buff_fill=0;
	  };
	  if (mpx_buff_fill==2)
	  {
		//  if (UserButtonPressed&2)
		//  Sin_Gen (&Audio2[96],fs,fleft,fright,48);

		  MPX_Gen (fs,mult_factor,&Audio2[bufsize],&MPX_buf[mult_factor*bufsize/2],bufsize/2);
	  	  DAC_normalise(&MPX_buf[mult_factor*bufsize/2],&DAC_buf[mult_factor*bufsize/2],mult_factor*bufsize/2);
	      FM_MPX(f0,&MPX_buf[mult_factor*bufsize/2],mult_factor*bufsize/2, &DDS_buf[mult_factor*bufsize/2]);
	  	  mpx_buff_fill=0;
	  };
  }

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




/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
