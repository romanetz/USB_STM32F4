/**
  ******************************************************************************
  * @file    usbd_audio_core.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-July-2011
  * @brief   This file provides the high layer firmware functions to manage the 
  *          following functionalities of the USB Audio Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as Audio Streaming Device
  *           - Audio Streaming data transfer
  *           - AudioControl requests management
  *           - Error management
  *           
  *  @verbatim
  *      
  *          ===================================================================      
  *                                Audio Class Driver Description
  *          =================================================================== 
  *           This driver manages the Audio Class 1.0 following the "USB Device Class Definition for
  *           Audio Devices V1.0 Mar 18, 98".
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Standard AC Interface Descriptor management
  *             - 1 Audio Streaming Interface (with single channel, PCM, Stereo mode)
  *             - 1 Audio Streaming Endpoint
  *             - 1 Audio Terminal Input (1 channel)
  *             - Audio Class-Specific AC Interfaces
  *             - Audio Class-Specific AS Interfaces
  *             - AudioControl Requests: only SET_CUR and GET_CUR requests are supported (for Mute)
  *             - Audio Feature Unit (limited to Mute control)
  *             - Audio Synchronization type: Asynchronous/Explicit feedback
  *             - Single fixed audio sampling rate (configurable in usbd_conf.h file)
  *          
  *           @note
  *            The Audio Class 1.0 is based on USB Specification 1.0 and thus supports only
  *            Low and Full speed modes and does not allow High Speed transfers.
  *            Please refer to "USB Device Class Definition for Audio Devices V1.0 Mar 18, 98"
  *            for more details.
  * 
  *           These aspects may be enriched or modified for a specific user application.
  *          
  *            This driver doesn't implement the following aspects of the specification 
  *            (but it is possible to manage these features with some modifications on this driver):
  *             - AudioControl Endpoint management
  *             - AudioControl requsests other than SET_CUR and GET_CUR
  *             - Abstraction layer for AudioControl requests (only Mute functionality is managed)
  *             - Audio Synchronization type: Adaptive
  *             - Audio Compression modules and interfaces
  *             - MIDI interfaces and modules
  *             - Mixer/Selector/Processing/Extension Units (Feature unit is limited to Mute control)
  *             - Any other application-specific modules
  *             - Multiple and Variable audio sampling rates
  *             - Out Streaming Endpoint/Interface (microphone)
  *      
  *  @endverbatim
  *                                  
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

#include "usbd_audio_core.h"
#include "usbd_audio_out_if.h"
/* Includes ------------------------------------------------------------------*/
#include "usbd_usr.h"
#include "usb_regs.h"
#include "usbd_ioreq.h"
extern s16 Audio[192] __attribute__ ((aligned(4)));
extern s16 Audio2[192] __attribute__ ((aligned(4)));
extern volatile uint8_t audio_buffer_fill;
static volatile uint8_t flag=0;
static volatile uint8_t dpid;
volatile int32_t gap,corr,oldgap,dgap,tmpxx;
volatile int32_t maxgap, mingap;
int8_t shift = 0;
static volatile uint32_t  usbd_cdc_AltSet  = 0;

volatile uint32_t counter=0,old_counter=0;
__ALIGN_BEGIN uint8_t CmdBuff[VIRTUAL_COM_PORT_INT_SIZE ] __ALIGN_END ;
__ALIGN_BEGIN uint8_t USB_Rx_Buffer   [VIRTUAL_COM_PORT_DATA_SIZE] __ALIGN_END ;
#define APP_TX_BUF_SIZE 2048
__ALIGN_BEGIN uint8_t APP_Rx_Buffer   [APP_RX_DATA_SIZE] __ALIGN_END ;
__ALIGN_BEGIN uint8_t APP_Tx_Buffer   [APP_TX_BUF_SIZE] __ALIGN_END ;

uint32_t USB_Tx_length=0;
uint32_t USB_Tx_ptr=0;
uint8_t  USB_Tx_State = 0;

uint32_t APP_Rx_ptr_in=0;    /* Increment this pointer or roll it back to
                                     start address when writing received data
                                     in the buffer APP_Rx_Buffer. */

uint32_t APP_Rx_ptr_out = 0;
uint32_t APP_Rx_length  = 0;

static uint32_t cdcCmd = 0xFF;
static uint32_t cdcLen = 0;


//uint8_t APP_Tx_Buffer[APP_TX_BUF_SIZE];
uint32_t APP_tx_ptr_head;
uint32_t APP_tx_ptr_tail;
/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */


/** @defgroup usbd_audio 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup usbd_audio_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup usbd_audio_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup usbd_audio_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup usbd_audio_Private_FunctionPrototypes
  * @{
  */

/*********************************************
   AUDIO Device library callbacks
 *********************************************/
static uint8_t  usbd_audio_Init       (void  *pdev, uint8_t cfgidx);
static uint8_t  usbd_audio_DeInit     (void  *pdev, uint8_t cfgidx);
static uint8_t  usbd_audio_Setup      (void  *pdev, USB_SETUP_REQ *req);
static uint8_t  usbd_audio_EP0_RxReady(void *pdev);
static uint8_t  usbd_audio_DataIn     (void *pdev, uint8_t epnum);
static uint8_t  usbd_audio_DataOut    (void *pdev, uint8_t epnum);
static uint8_t  usbd_audio_SOF        (void *pdev);
static uint8_t  usbd_audio_OUT_Incplt (void  *pdev);
static uint8_t  usbd_audio_IN_Incplt (void  *pdev);

/*********************************************
   AUDIO Requests management functions
 *********************************************/
static void AUDIO_Req_GetCurrent(void *pdev, USB_SETUP_REQ *req);
static void AUDIO_Req_SetCurrent(void *pdev, USB_SETUP_REQ *req);
static uint8_t  *USBD_audio_GetCfgDesc (uint8_t speed, uint16_t *length);
/**
  * @}
  */ 

/** @defgroup usbd_audio_Private_Variables
  * @{
  */ 
/* Main Buffer for Audio Data Out transfers and its relative pointers */
uint8_t  IsocOutBuff [TOTAL_OUT_BUF_SIZE] __attribute__ ((aligned(4)));
uint8_t* IsocOutWrPtr = IsocOutBuff;
uint8_t* IsocOutRdPtr = IsocOutBuff;

/* Main Buffer for Audio Control Rrequests transfers and its relative variables */
uint8_t  AudioCtl[64];
uint8_t  AudioCtlCmd = 0;
uint32_t AudioCtlLen = 0;
uint8_t  AudioCtlUnit = 0;
#define SOF_RATE 0x5
/*
#ifndef sync
#define sync
#endif
*/
static uint32_t PlayFlag = 0;
static volatile  uint16_t SOF_num=0;
static volatile uint32_t  usbd_audio_AltSet = 0;
static uint8_t usbd_audio_CfgDesc[AUDIO_CONFIG_DESC_SIZE];
uint8_t tmpbuf[AUDIO_OUT_PACKET+16] __attribute__ ((aligned(4)));
static volatile uint16_t rest;
static volatile uint16_t max_length;

CDC_IF_Prop_TypeDef APP_FOPS =
{
  VCP_Init,
  VCP_DeInit,
  VCP_Ctrl,
  VCP_DataTx,
  VCP_DataRx
};

/* Private variables ---------------------------------------------------------*/
LINE_CODING linecoding = {
		115200, /* baud rate*/
		0x00, /* stop bits-1*/
		0x00, /* parity - none*/
		0x08 /* nb. of bits 8*/
};
/**
* @brief  USBD_USR_DeviceDisonnected
*         Displays the message on LCD on device disconnection Event
* @param  None
* @retval Staus
*/
void USBD_USR_DeviceDisconnected (void)
{
	  //usbd_audio_DeInit(pdev,1);
}


/* AUDIO interface class callbacks structure */
USBD_Class_cb_TypeDef  AUDIO_cb = 
{
  usbd_audio_Init,
  usbd_audio_DeInit,
 /*Cintrol EP*/
  usbd_audio_Setup,
  NULL, /* EP0_TxSent */
  usbd_audio_EP0_RxReady,
 /*Class-specific EP */
  usbd_audio_DataIn,
  usbd_audio_DataOut,
  usbd_audio_SOF,
  usbd_audio_IN_Incplt,
  usbd_audio_OUT_Incplt,   
  USBD_audio_GetCfgDesc,
#ifdef USB_OTG_HS_CORE  
  USBD_audio_GetCfgDesc, /* use same config as per FS */
#endif    
};

#ifdef shit
uint8_t audio_desc[]={
CONFIG_ONLY_DESC_SIZE,     /*  Configuration Descriptor Size - always 9 bytes*/
  USB_CONFIG_DESCRIPTOR,     /* "Configuration" type of descriptor */
  CONFIG_DESC_SIZE, 0x00,    /*  Total length of the Configuration descriptor */
  0x02,                      /*  NumInterfaces */
  0x01,                      /*  Configuration Value */
  0,                         /*  Configuration Description String Index*/
  (BUS_POWERED | SELF_POWERED),
  /* S08/CFv1 are both self powered (its compulsory to set bus powered)*/
  /* Attributes.support RemoteWakeup and self power */
  0x32,                   /*  Current draw from bus */

  /* AUDIO CONTROL INTERFACE DESCRIPTOR */
  IFACE_ONLY_DESC_SIZE,      /* Size of this descriptor */
  USB_IFACE_DESCRIPTOR,      /* INTERFACE descriptor */
  0x00,                      /* Index of this interface */
  0x00,                      /* Index of this setting */
  0x00,                      /* 0 endpoint */
  USB_DEVICE_CLASS_AUDIO,    /* AUDIO */
  USB_SUBCLASS_AUDIOCONTROL, /* AUDIO_CONTROL */
  0x00,                      /* Unused */
  0x00,                      /* Unused */

  /* Interface Header Audio Class Descriptor */
  /* Audio class-specific interface header */
  HEADER_ONLY_DESC_SIZE,           /* bLength (9) */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType (CS_INTERFACE) */
  AUDIO_CONTROL_HEADER,            /* bDescriptorSubtype (HEADER) */
  0x00,0x01,                       /* bcdADC (1.0) */
  0x2B,0x00,                       /* wTotalLength (43) */
  0x01,                            /* bInCollection (1 streaming interface) */
  0x01,                            /* baInterfaceNr (interface 1 is stream) */

  /* Input Terminal Audio Class Descriptor */
  /* Audio class-specific input terminal */
  INPUT_TERMINAL_ONLY_DESC_SIZE,   /* bLength (12) */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType (CS_INTERFACE) */
  AUDIO_CONTROL_INPUT_TERMINAL,    /* bDescriptorSubtype (INPUT_TERMINAL) */
  0x01,                            /* bTerminalID (1) */
  0x01,0x01,                       /* wTerminalType (radio receiver) */
  0x00,                            /* bAssocTerminal (none) */
  NB_CHANNELS,                     /* bNrChannels (2) */
  0x00,0x00,                       /* wChannelConfig (left, right) */
  0x00,                            /* iChannelNames (none) */
  0x00,                            /* iTerminal (none) */

  /* Feature Unit Audio Class Descriptor */
  /* Audio class-specific feature unit */
  FEATURE_UNIT_ONLY_DESC_SIZE,     /* bLength (9) */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType (CS_INTERFACE) */
  AUDIO_CONTROL_FEATURE_UNIT,      /* bDescriptorSubtype (FEATURE_UNIT) */
  0x02,                            /* bUnitID (2) */
  0x01,                            /* bSourceID (input terminal 1) */
  0x01,                            /* bControlSize (1 bytes) */
  (
      AUDIO_AUTOMATIC_GAIN_CONTROL |  /* Master controls */
      AUDIO_TREBLE_CONTROL         |  /* Controls enabled: AGC, TREBLE, BASS, VOLUME, MUTE */
      AUDIO_BASS_CONTROL           |
      AUDIO_VOLUME_CONTROL         |
      AUDIO_MUTE_CONTROL
  ),
  0x00,                            /* Channel 0 controls */
  0x00,                            /* iFeature */

  /* Output Terminal Audio Class Descriptor */
  /* Audio class-specific output terminal */
  OUTPUT_TERMINAL_ONLY_DESC_SIZE,  /* bLength (9) */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType (CS_INTERFACE) */
  AUDIO_CONTROL_OUTPUT_TERMINAL,   /* bDescriptorSubtype (OUTPUT_TERMINAL) */
  0x03,                            /* bTerminalID (3) */
  0x01,0x03,                       /* wTerminalType (USB streaming) */
  0x00,                            /* bAssocTerminal (none) */
  0x02,                            /* bSourceID (feature unit 2) */
  0x00,                            /* iTerminal (none) */

  /* Audio Stream Interface Descriptor(Interface 1, Alternate Setting 0) */
  /* USB speaker standard AS interface descriptor - audio streaming operational
  (Interface 1, Alternate Setting 0) */
  IFACE_ONLY_DESC_SIZE,            /* bLength (9) */
  USB_IFACE_DESCRIPTOR,            /* bDescriptorType (CS_INTERFACE) */
  0x01,                            /* interface Number: 1 */
  0x00,                            /* Alternate Setting: 0 */
  0x00,                            /* not used (Zero Bandwidth) */
  USB_DEVICE_CLASS_AUDIO,          /* USB DEVICE CLASS AUDIO */
  USB_SUBCLASS_AUDIOSTREAM,        /* AUDIO SUBCLASS AUDIOSTREAMING */
  0x00,                            /* AUDIO PROTOCOL UNDEFINED */
  0x00,                            /* Unused */

  /* Alternate Audio Interface Descriptor(Interface 1, Alternate Setting 1) */
  /* USB speaker standard AS interface descriptor - audio streaming operational
  (Interface 1, Alternate Setting 1) */
  IFACE_ONLY_DESC_SIZE,            /* bLength (9) */
  USB_IFACE_DESCRIPTOR,            /* bDescriptorType (CS_INTERFACE) */
  0x01,                            /* interface Number: 1 */
  0x01,                            /* Alternate Setting: 1 */
  0x02,                            /* TWO Endpoints DATA and Feedback endpoints */
  USB_DEVICE_CLASS_AUDIO,          /* USB DEVICE CLASS AUDIO */
  USB_SUBCLASS_AUDIOSTREAM,        /* AUDIO SUBCLASS AUDIOSTREAMING */
  0x00,                            /* AUDIO PROTOCOL UNDEFINED */
  0x00,                            /* Unused */

  /* Audio Stream Audio Class Descriptor */
  /* USB speaker standard General AS interface descriptor */
  AUDIO_STREAMING_IFACE_DESC_SIZE, /* bLength (7) */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType (CS_INTERFACE) */
  AUDIO_STREAMING_GENERAL,         /* GENERAL subtype  */
  0x01,                            /* Unit ID of output terminal */
  INTERFACE_DELAY,                 /* Interface delay */
  PCM_FORMAT,0x00,                 /* PCM format */

  /* Format Type Audio Descriptor */
  /* USB speaker audio type I format interface descriptor */
  AUDIO_INTERFACE_DESC_TYPE_I_SIZE, 	/* bLength (11) */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE, 	/* bDescriptorType (CS_INTERFACE) */
  AUDIO_STREAMING_FORMAT_TYPE,     	/* DescriptorSubtype: AUDIO STREAMING FORMAT TYPE */
  AUDIO_FORMAT_TYPE_I,             	/* Format Type: Type I */
  NB_CHANNELS,                       	/* Number of Channels: one channel */
  SUB_FRAME_SIZE,                            	/* SubFrame Size: one byte per audio subframe */
  BIT_RESOLUTION,                            	/* Bit Resolution: 8 bits per sample */
  0x01,                            	/* One frequency supported */
  0x40,0x1F,0x00,                  	/* 8 kHz */
  //0x80,0xBB,0x00,					/* 48 kHz */

  /* Isochronous Endpoint Descriptor */
  AUDIO_STREAMING_ENDP_DESC_SIZE, // bLength
  USB_AUDIO_DESCRIPTOR,		    // bDescriptorType
  AUDIO_ENDPOINT_GENERAL,		    // bDescriptorSubType
  0x80,		                    // bmAttributes
  0x00,		                    // bLockDelayUnits
  0x00,0x00,	                    // wLockDelay

  /* EP1. Isochronous Endpoint Audio Class Descriptor */
  ENDP_ONLY_DESC_SIZE,        // bLength
  USB_ENDPOINT_DESCRIPTOR,    // endpoint descriptor type
  EP01_OUT,                   // host->dev, ep 1
  0x05,                       // iso + async + data
  0x30,0x00,                  /* wMaxPacketSize(96) */
  //0xC4,0x00,                /* wMaxPacketSize(196) */
  0x01,                       // interval polling(2^x ms)
  0x00,                       // bRefresh
  EP02_IN,                    // bSyncAddress

  // EP2. EP descriptor (standard) - feedback endpoint
  ENDP_ONLY_DESC_SIZE,        // bLength
  USB_ENDPOINT_DESCRIPTOR,    // bDescriptorType
  EP02_IN,                    // dev->host, ep 2
  0x15,                       // iso + asynch + feedback
  0x03,0x00,                  // wMaxPacketSize
  0x01,                       // interval polling(2^x ms)
  0x05,                       // bRefresh(32ms)
  0x00,                       // bSyncAddress
  };
#endif
/* USB AUDIO device Configuration Descriptor */

#ifdef sync_old
static uint8_t usbd_audio_CfgDesc[AUDIO_CONFIG_DESC_SIZE] =
{
  /* Configuration 1 */
  USB_CONFIGUARTION_DESC_SIZE,          /* bLength */
  USB_CONFIGURATION_DESCRIPTOR_TYPE,    /* bDescriptorType */
  LOBYTE(AUDIO_CONFIG_DESC_SIZE),       /* wTotalLength  118 bytes*/
  HIBYTE(AUDIO_CONFIG_DESC_SIZE),      
  0x02,                                 /* bNumInterfaces */
  0x01,                                 /* bConfigurationValue */
  0x00,                                 /* iConfiguration */
  0x80,                                 /* bmAttributes  BUS Powred*/
  0x80,                                 /* bMaxPower = 256 mA*/
  /* 09 byte*/
  
  /* USB Speaker Standard interface descriptor */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */
  0x00,                                 /* bInterfaceNumber */
  0x00,                                 /* bAlternateSetting */
  0x00,                                 /* bNumEndpoints */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOCONTROL,          /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/
  
  /* USB Speaker Class-specific AC Interface Descriptor */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_HEADER,                 /* bDescriptorSubtype */
  0x00,          /* 1.00 */             /* bcdADC */
  0x01,
  AUDIO_INTERFACE_DESC_SIZE+AUDIO_INPUT_TERMINAL_DESC_SIZE+AUDIO_FEATURE_UNIT_DESC_SZ+AUDIO_OUTPUT_TERMINAL_DESC_SIZE,   /* wTotalLength = 40*/
  0x00,
  0x01,                                 /* bInCollection */
  0x01,                                 /* baInterfaceNr */
  /* 09 byte*/
  
  /* USB Speaker Input Terminal Descriptor - host sends audio here*/
  AUDIO_INPUT_TERMINAL_DESC_SIZE,       /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_INPUT_TERMINAL,         /* bDescriptorSubtype */
  0x01,                                 /* bTerminalID */
  0x01,                                 /* wTerminalType AUDIO_TERMINAL_USB_STREAMING   0x0101 */
  0x01,
  0x00,                                 /* bAssocTerminal */
  0x02,                                 /* bNrChannels */
  0x03,                                 /* wChannelConfig 0x0003  FL,FR */
  0x00,
  0x00,                                 /* iChannelNames */
  0x00,                                 /* iTerminal */
  /* 12 byte*/
  
  /* USB Speaker Audio Feature Unit Descriptor - audio is processed here */
  AUDIO_FEATURE_UNIT_DESC_SZ,           /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_FEATURE_UNIT,           /* bDescriptorSubtype */
  AUDIO_OUT_STREAMING_CTRL,             /* bUnitID */
  0x01,                                 /* bSourceID */
  0x01,                                 /* bControlSize */
  AUDIO_CONTROL_MUTE,                   /* bmaControls(0) */
  0x00,                                 /* bmaControls(1) */
  0x00,									/* bmaControls(2) */
  0x00,                                 /* iTerminal */
  /* 10 byte*/
  
  /*USB Speaker Output Terminal Descriptor */
  AUDIO_OUTPUT_TERMINAL_DESC_SIZE,      /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_OUTPUT_TERMINAL,        /* bDescriptorSubtype */
  0x03,                                 /* bTerminalID */
  0x01,                                 /* wTerminalType  0x0301*/
  0x03,
  0x00,                                 /* bAssocTerminal */
  0x02,                                 /* bSourceID */
  0x00,                                 /* iTerminal */
  /* 09 byte*/
  
  /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero Bandwith */
  /* Interface 1, Alternate Setting 0                                             */
  AUDIO_INTERFACE_DESC_SIZE,  			/* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */
  0x01,                                 /* bInterfaceNumber */
  0x00,                                 /* bAlternateSetting */
  0x00,                                 /* bNumEndpoints */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/
  
  /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Operational */
  /* Interface 1, Alternate Setting 1                                           */
  AUDIO_INTERFACE_DESC_SIZE,  			/* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */
  0x01,                                 /* bInterfaceNumber */
  0x01,                                 /* bAlternateSetting */
  0x02,                                 /* bNumEndpoints - Audio Out and Feedback enpoint*/
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/
  
  /* USB Speaker Audio Streaming Interface Descriptor */
  AUDIO_STREAMING_INTERFACE_DESC_SIZE,  /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_STREAMING_GENERAL,              /* bDescriptorSubtype */
  0x01,                                 /* bTerminalLink */
  0x01,                                 /* bDelay */
  0x01,                                 /* wFormatTag AUDIO_FORMAT_PCM  0x0001*/
  0x00,
  /* 07 byte*/
  
  /* USB Speaker Audio Type III Format Interface Descriptor */
  AUDIO_FORMAT_TYPE_I_DESC_SZ,          /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_STREAMING_FORMAT_TYPE,          /* bDescriptorSubtype */
  AUDIO_FORMAT_TYPE_I,                  /* bFormatType */
  0x02,                                 /* bNrChannels */
  0x02,                                 /* bSubFrameSize :  2 Bytes per frame (16bits) */
  16,                                   /* bBitResolution (16-bits per sample) */ 
  0x01,                                 /* bSamFreqType only one frequency supported */ 
  SAMPLE_FREQ(USBD_AUDIO_FREQ),         /* Audio sampling frequency coded on 3 bytes */
  /* 11 byte*/

  /* Endpoint 1 - Standard Descriptor */
    AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
    USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */
    AUDIO_OUT_EP,                         /* bEndpointAddress 1 out endpoint*/
    0x05,							      /* bmAttributes */
    AUDIO_OUT_PACKET+16,0,    			  /* wMaxPacketSize in Bytes ((Freq(Samples)+1)*2(Stereo)*2(HalfWord)) */
    0x01,                                 /* bInterval */
    0x00,                                 /* bRefresh */
    AUDIO_IN_EP,                          /* bSynchAddress */
    /* 09 byte*/

    /* Endpoint - Audio Streaming (Class-specific) Descriptor*/
     AUDIO_STREAMING_ENDPOINT_DESC_SIZE,   /* bLength */
     AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
     AUDIO_ENDPOINT_GENERAL,               /* bDescriptor */
     0x00,                                 /* bmAttributes */
     0x00,                                 /* bLockDelayUnits */
     0x00,                                 /* wLockDelay */
     0x00,
     /* 07 byte*/

  /* Endpoint 2 - Standard Descriptor */
  AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
  USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */
  AUDIO_IN_EP,                         /* bEndpointAddress 2 in endpoint*/
  0x11,        						   /* bmAttributes */
  3,0,   						    /* wMaxPacketSize in Bytes 4 */
  1,								/* bInterval 1ms*/
  SOF_RATE,							/* bRefresh 2ms*/
  0x00,                             /* bSynchAddress */
  /* 09 byte*/
} ;
#endif

//typedef USB_CONFIGURATION_DESCRIPTOR_t

#ifdef sync
static uint8_t usbd_audio_CfgDesc[AUDIO_CONFIG_DESC_SIZE] =
{
/* Configuration 1 */
USB_CONFIGUARTION_DESC_SIZE,          /* bLength */
USB_CONFIGURATION_DESCRIPTOR_TYPE,    /* bDescriptorType */
LOBYTE(AUDIO_CONFIG_DESC_SIZE),       /* wTotalLength  118 bytes*/
HIBYTE(AUDIO_CONFIG_DESC_SIZE),
0x04,                                 /* bNumInterfaces */
0x01,                                 /* bConfigurationValue */
0x00,                                 /* iConfiguration */
0x80,                                 /* bmAttributes  BUS Powred*/
0xF0,                                 /* bMaxPower = 500 mA*/
/* 09 byte*/

/* UAC IAD */
INTERFACE_ASSOC_DESC_SIZE,	// bLength: Interface Descriptor size
USB_INTERFACE_ASSOC_DESCRIPTOR_TYPE,	// bDescriptorType: IAD
0x00,	// bFirstInterface
0x02,	// bInterfaceCount
USB_DEVICE_CLASS_AUDIO,	// bFunctionClass: Audio
0x00,	// bFunctionSubClass
0x00,	// bFunctionProtocol
0x00,	// Interface string index

/* USB Speaker Standard interface descriptor */
AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */
0x00,                                 /* bInterfaceNumber */
0x00,                                 /* bAlternateSetting */
0x00,                                 /* bNumEndpoints */
USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
AUDIO_SUBCLASS_AUDIOCONTROL,          /* bInterfaceSubClass */
AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
0x00,                                 /* iInterface */
/* 09 byte*/

/* USB Speaker Class-specific AC Interface Descriptor */
AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
AUDIO_CONTROL_HEADER,                 /* bDescriptorSubtype */
0x00,          /* 1.00 */             /* bcdADC */
0x01,
AUDIO_INTERFACE_DESC_SIZE+AUDIO_INPUT_TERMINAL_DESC_SIZE+AUDIO_FEATURE_UNIT_DESC_SZ+AUDIO_OUTPUT_TERMINAL_DESC_SIZE,   /* wTotalLength = 40*/
0x00,
0x01,                                 /* bInCollection */
0x01,                                 /* baInterfaceNr */
/* 09 byte*/

/* USB Speaker Input Terminal Descriptor - host sends audio here*/
AUDIO_INPUT_TERMINAL_DESC_SIZE,       /* bLength */
AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
AUDIO_CONTROL_INPUT_TERMINAL,         /* bDescriptorSubtype */
0x01,                                 /* bTerminalID */
0x01,                                 /* wTerminalType AUDIO_TERMINAL_USB_STREAMING   0x0101 */
0x01,
0x00,                                 /* bAssocTerminal */
0x02,                                 /* bNrChannels */
0x03,                                 /* wChannelConfig 0x0003  FL,FR */
0x00,
0x00,                                 /* iChannelNames */
0x00,                                 /* iTerminal */
/* 12 byte*/

/* USB Speaker Audio Feature Unit Descriptor - audio is processed here */
AUDIO_FEATURE_UNIT_DESC_SZ,           /* bLength */
AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
AUDIO_CONTROL_FEATURE_UNIT,           /* bDescriptorSubtype */
AUDIO_OUT_STREAMING_CTRL,             /* bUnitID */
0x01,                                 /* bSourceID */
0x01,                                 /* bControlSize */
AUDIO_CONTROL_MUTE,                   /* bmaControls(0) */
0x00,                                 /* bmaControls(1) */
0x00,									/* bmaControls(2) */
0x00,                                 /* iTerminal */
/* 10 byte*/

/*USB Speaker Output Terminal Descriptor */
AUDIO_OUTPUT_TERMINAL_DESC_SIZE,      /* bLength */
AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
AUDIO_CONTROL_OUTPUT_TERMINAL,        /* bDescriptorSubtype */
0x03,                                 /* bTerminalID */
0x01,                                 /* wTerminalType  0x0301*/
0x03,
0x00,                                 /* bAssocTerminal */
0x02,                                 /* bSourceID */
0x00,                                 /* iTerminal */
/* 09 byte*/

/* USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero Bandwith */
/* Interface 1, Alternate Setting 0                                             */
AUDIO_INTERFACE_DESC_SIZE,  			/* bLength */
USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */
0x01,                                 /* bInterfaceNumber */
0x00,                                 /* bAlternateSetting */
0x00,                                 /* bNumEndpoints */
USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
0x00,                                 /* iInterface */
/* 09 byte*/

/* USB Speaker Standard AS Interface Descriptor - Audio Streaming Operational */
/* Interface 1, Alternate Setting 1                                           */
AUDIO_INTERFACE_DESC_SIZE,  			/* bLength */
USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */
0x01,                                 /* bInterfaceNumber */
0x01,                                 /* bAlternateSetting */
0x02,                                 /* bNumEndpoints - Audio Out and Feedback enpoint*/
USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
0x00,                                 /* iInterface */
/* 09 byte*/

/* USB Speaker Audio Streaming Interface Descriptor */
AUDIO_STREAMING_INTERFACE_DESC_SIZE,  /* bLength */
AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
AUDIO_STREAMING_GENERAL,              /* bDescriptorSubtype */
0x01,                                 /* bTerminalLink */
0x04,                                 /* bDelay */
0x01,                                 /* wFormatTag AUDIO_FORMAT_PCM  0x0001*/
0x00,
/* 07 byte*/

/* USB Speaker Audio Type III Format Interface Descriptor */
AUDIO_FORMAT_TYPE_I_DESC_SZ,          /* bLength */
AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
AUDIO_STREAMING_FORMAT_TYPE,          /* bDescriptorSubtype */
AUDIO_FORMAT_TYPE_I,                  /* bFormatType */
0x02,                                 /* bNrChannels */
0x02,                                 /* bSubFrameSize :  2 Bytes per frame (16bits) */
16,                                   /* bBitResolution (16-bits per sample) */
0x01,                                 /* bSamFreqType only one frequency supported */
SAMPLE_FREQ(USBD_AUDIO_FREQ),         /* Audio sampling frequency coded on 3 bytes */
/* 11 byte*/

/* Endpoint 1 - Standard Descriptor */
AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */
AUDIO_OUT_EP,                         /* bEndpointAddress 1 out endpoint*/
0x05,							      /* bmAttributes */
AUDIO_OUT_PACKET+16,0,    			  /* wMaxPacketSize in Bytes ((Freq(Samples)+1)*2(Stereo)*2(HalfWord)) */
0x01,                                 /* bInterval */
0x01,                                 /* bRefresh */
AUDIO_IN_EP,                          /* bSynchAddress */
/* 09 byte*/

/* Endpoint - Audio Streaming (Class-specific) Descriptor*/
AUDIO_STREAMING_ENDPOINT_DESC_SIZE,   /* bLength */
AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
AUDIO_ENDPOINT_GENERAL,               /* bDescriptor */
0x00,                                 /* bmAttributes */
0x00,                                 /* bLockDelayUnits */
0x32,                                 /* wLockDelay */
0x00,
/* 07 byte*/

/* Endpoint 2 - Standard Descriptor */
AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */
AUDIO_IN_EP,                         /* bEndpointAddress 2 in endpoint*/
0x11,        						   /* bmAttributes */
3,0,   						    /* wMaxPacketSize in Bytes 4 */
1,								/* bInterval 1ms*/
SOF_RATE,							/* bRefresh 64ms*/
0x00,                             /* bSynchAddress */
/* 09 byte*/

/* CDC IAD */
0x08,	// bLength: Interface Descriptor size
0x0B,	// bDescriptorType: IAD
0x02,	// bFirstInterface
0x02,	// bInterfaceCount
0x02,	// bFunctionClass: CDC
0x02,	// bFunctionSubClass
0x01,	// bFunctionProtocol
0x02,	// iFunction

/*Interface Descriptor*/
0x09,   /* bLength: Interface Descriptor size */
USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: Interface */  /* Interface descriptor type */
0x02,   /* bInterfaceNumber: Number of Interface */
0x00,   /* bAlternateSetting: Alternate setting */
0x01,   /* bNumEndpoints: One endpoints used */
0x02,   /* bInterfaceClass: Communication Interface Class */
0x02,   /* bInterfaceSubClass: Abstract Control Model */
0x01,   /* bInterfaceProtocol: Common AT commands */
0x00,   /* iInterface: */

/*Header Functional Descriptor*/
0x05,   /* bLength: Endpoint Descriptor size */
CDC_INTERFACE_DESCRIPTOR_TYPE,   /* bDescriptorType: CS_INTERFACE */
0x00,   /* bDescriptorSubtype: Header Func Desc */
0x10,   /* bcdCDC: spec release number */
0x01,

/*Call Managment Functional Descriptor*/
0x05,   /* bFunctionLength */
0x24,   /* bDescriptorType: CS_INTERFACE */
0x01,   /* bDescriptorSubtype: Call Management Func Desc */
0x00,   /* bmCapabilities: D0+D1 */
0x03,   /* bDataInterface: 3 */

/*ACM Functional Descriptor*/
0x04,   /* bFunctionLength */
0x24,   /* bDescriptorType: CS_INTERFACE */
0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
0x02,   /* bmCapabilities */

/*Union Functional Descriptor*/
0x05,   /* bFunctionLength */
0x24,   /* bDescriptorType: CS_INTERFACE */
0x06,   /* bDescriptorSubtype: Union func desc */
0x02,   /* bMasterInterface: Communication class interface */
0x03,   /* bSlaveInterface0: Data Class Interface */

/*Endpoint 3 Descriptor*/
0x07,  									 /* bLength: Endpoint Descriptor size */
USB_ENDPOINT_DESCRIPTOR_TYPE, 			/* bDescriptorType: Endpoint */
CDC_STATE_IN_EP,   						/* bEndpointAddress: (IN3) */
USB_ENDPOINT_TYPE_INTERRUPT,   			/* bmAttributes: Interrupt */
VIRTUAL_COM_PORT_INT_SIZE,      			/* wMaxPacketSize: */
0x00,
0xFF,   									/* bInterval: */

/*Data class interface descriptor*/
0x09,   /* bLength: Endpoint Descriptor size */
USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: */
0x03,   /* bInterfaceNumber: Number of Interface */
0x00,   /* bAlternateSetting: Alternate setting */
0x02,   /* bNumEndpoints: Two endpoints used */
0x0A,   /* bInterfaceClass: CDC */
0x00,   /* bInterfaceSubClass: */
0x00,   /* bInterfaceProtocol: */
0x00,   /* iInterface: */

/*Endpoint 2 Descriptor*/
0x07,   									/* bLength: Endpoint Descriptor size */
USB_ENDPOINT_DESCRIPTOR_TYPE,   			/* bDescriptorType: Endpoint */
CDC_OUT_DATA_EP,   						/* bEndpointAddress: (OUT2) */
USB_ENDPOINT_TYPE_BULK,   				/* bmAttributes: Bulk */
VIRTUAL_COM_PORT_DATA_SIZE,             	/* wMaxPacketSize: */
0x00,
0x00,   									/* bInterval: ignore for Bulk transfer */

/*Endpoint 2 Descriptor*/
0x07,   /* bLength: Endpoint Descriptor size */
USB_ENDPOINT_DESCRIPTOR_TYPE,   			/* bDescriptorType: Endpoint */
CDC_IN_DATA_EP,   						/* bEndpointAddress: (IN2) */
USB_ENDPOINT_TYPE_BULK,   				/* bmAttributes: Bulk */
VIRTUAL_COM_PORT_DATA_SIZE,             	/* wMaxPacketSize: */
0x00,
0x00    									/* bInterval */
};

#else
/* USB AUDIO device Configuration Descriptor */
static uint8_t usbd_audio_CfgDesc[AUDIO_CONFIG_DESC_SIZE] =
{
  /* Configuration 1 */
  0x09,                                 /* bLength */
  USB_CONFIGURATION_DESCRIPTOR_TYPE,    /* bDescriptorType */
  LOBYTE(AUDIO_CONFIG_DESC_SIZE),       /* wTotalLength  109 bytes*/
  HIBYTE(AUDIO_CONFIG_DESC_SIZE),
  0x02,                                 /* bNumInterfaces */
  0x01,                                 /* bConfigurationValue */
  0x00,                                 /* iConfiguration */
  0xC0,                                 /* bmAttributes  BUS Powred*/
  0x32,                                 /* bMaxPower = 100 mA*/
  /* 09 byte*/

  /* USB Speaker Standard interface descriptor */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */
  0x00,                                 /* bInterfaceNumber */
  0x00,                                 /* bAlternateSetting */
  0x00,                                 /* bNumEndpoints */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOCONTROL,          /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/

  /* USB Speaker Class-specific AC Interface Descriptor */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_HEADER,                 /* bDescriptorSubtype */
  0x00,          /* 1.00 */             /* bcdADC */
  0x01,
  0x27,                                 /* wTotalLength = 39*/
  0x00,
  0x01,                                 /* bInCollection */
  0x01,                                 /* baInterfaceNr */
  /* 09 byte*/

  /* USB Speaker Input Terminal Descriptor */
  AUDIO_INPUT_TERMINAL_DESC_SIZE,       /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_INPUT_TERMINAL,         /* bDescriptorSubtype */
  0x01,                                 /* bTerminalID */
  0x01,                                 /* wTerminalType AUDIO_TERMINAL_USB_STREAMING   0x0101 */
  0x01,
  0x00,                                 /* bAssocTerminal */
  0x01,                                 /* bNrChannels */
  0x00,                                 /* wChannelConfig 0x0000  Mono */
  0x00,
  0x00,                                 /* iChannelNames */
  0x00,                                 /* iTerminal */
  /* 12 byte*/

  /* USB Speaker Audio Feature Unit Descriptor */
  0x09,                                 /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_FEATURE_UNIT,           /* bDescriptorSubtype */
  AUDIO_OUT_STREAMING_CTRL,             /* bUnitID */
  0x01,                                 /* bSourceID */
  0x01,                                 /* bControlSize */
  AUDIO_CONTROL_MUTE,                   /* bmaControls(0) */
  0x00,                                 /* bmaControls(1) */
  0x00,                                 /* iTerminal */
  /* 09 byte*/

  /*USB Speaker Output Terminal Descriptor */
  0x09,      /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_OUTPUT_TERMINAL,        /* bDescriptorSubtype */
  0x03,                                 /* bTerminalID */
  0x01,                                 /* wTerminalType  0x0301*/
  0x03,
  0x00,                                 /* bAssocTerminal */
  0x02,                                 /* bSourceID */
  0x00,                                 /* iTerminal */
  /* 09 byte*/

  /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero Bandwith */
  /* Interface 1, Alternate Setting 0                                             */
  AUDIO_INTERFACE_DESC_SIZE,  /* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */
  0x01,                                 /* bInterfaceNumber */
  0x00,                                 /* bAlternateSetting */
  0x00,                                 /* bNumEndpoints */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/

  /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Operational */
  /* Interface 1, Alternate Setting 1                                           */
  AUDIO_INTERFACE_DESC_SIZE,  /* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */
  0x01,                                 /* bInterfaceNumber */
  0x01,                                 /* bAlternateSetting */
  0x01,                                 /* bNumEndpoints */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/

  /* USB Speaker Audio Streaming Interface Descriptor */
  AUDIO_STREAMING_INTERFACE_DESC_SIZE,  /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_STREAMING_GENERAL,              /* bDescriptorSubtype */
  0x01,                                 /* bTerminalLink */
  0x01,                                 /* bDelay */
  0x01,                                 /* wFormatTag AUDIO_FORMAT_PCM  0x0001*/
  0x00,
  /* 07 byte*/

  /* USB Speaker Audio Type III Format Interface Descriptor */
  0x0B,                                 /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_STREAMING_FORMAT_TYPE,          /* bDescriptorSubtype */
  AUDIO_FORMAT_TYPE_III,                /* bFormatType */
  0x02,                                 /* bNrChannels */
  0x02,                                 /* bSubFrameSize :  2 Bytes per frame (16bits) */
  16,                                   /* bBitResolution (16-bits per sample) */
  0x01,                                 /* bSamFreqType only one frequency supported */
  SAMPLE_FREQ(USBD_AUDIO_FREQ),         /* Audio sampling frequency coded on 3 bytes */
  /* 11 byte*/

  /* Endpoint 1 - Standard Descriptor */
  AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
  USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */
  AUDIO_OUT_EP,                         /* bEndpointAddress 1 out endpoint*/
  USB_ENDPOINT_TYPE_ISOCHRONOUS,        /* bmAttributes */
  AUDIO_PACKET_SZE(USBD_AUDIO_FREQ),    /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
  0x01,                                 /* bInterval */
  0x00,                                 /* bRefresh */
  0x00,                                 /* bSynchAddress */
  /* 09 byte*/

  /* Endpoint - Audio Streaming Descriptor*/
  AUDIO_STREAMING_ENDPOINT_DESC_SIZE,   /* bLength */
  AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
  AUDIO_ENDPOINT_GENERAL,               /* bDescriptor */
  0x00,                                 /* bmAttributes */
  0x00,                                 /* bLockDelayUnits */
  0x00,                                 /* wLockDelay */
  0x00,
  /* 07 byte*/
} ;
#endif

#ifdef composite

static uint8_t usbd_audio_CfgDesc[COMPOSITE_CONFIG_DESC_SIZE] =
{
  /* Configuration 1 */
  USB_CONFIGUARTION_DESC_SIZE,          /* bLength */
  USB_CONFIGURATION_DESCRIPTOR_TYPE,    /* bDescriptorType */
  LOBYTE(COMPOSITE_CONFIG_DESC_SIZE),       /* wTotalLength  118 bytes*/
  HIBYTE(COMPOSITE_CONFIG_DESC_SIZE),
  0x05,                                 /* bNumInterfaces 1 AudioControl, 1 CDC control, 2 AudioStreaming, */
  0x01,                                 /* bConfigurationValue */
  0x00,                                 /* iConfiguration */
  0x80,                                 /* bmAttributes  BUS Powred*/
  0xF0,                                 /* bMaxPower = 500 mA*/
  /* 09 byte*/

  /* USB Speaker Standard interface descriptor */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */
  0x00,                                 /* bInterfaceNumber */
  0x00,                                 /* bAlternateSetting */
  0x00,                                 /* bNumEndpoints */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOCONTROL,          /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/

  /* USB Speaker Class-specific AC Interface Descriptor */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_HEADER,                 /* bDescriptorSubtype */
  0x00,          /* 1.00 */             /* bcdADC */
  0x01,
  0x27,                                 /* wTotalLength = 39*/
  0x00,
  0x01,                                 /* bInCollection */
  0x01,                                 /* baInterfaceNr */
  /* 09 byte*/

  /* USB Speaker Input Terminal Descriptor - host sends audio here*/
  AUDIO_INPUT_TERMINAL_DESC_SIZE,       /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_INPUT_TERMINAL,         /* bDescriptorSubtype */
  0x01,                                 /* bTerminalID */
  0x01,                                 /* wTerminalType AUDIO_TERMINAL_USB_STREAMING   0x0101 */
  0x01,
  0x00,                                 /* bAssocTerminal */
  0x01,                                 /* bNrChannels */
  0x00,                                 /* wChannelConfig 0x0000  Mono */
  0x00,
  0x00,                                 /* iChannelNames */
  0x00,                                 /* iTerminal */
  /* 12 byte*/

  /* USB Speaker Audio Feature Unit Descriptor - audio is processed here */
  AUDIO_FEATURE_UNIT_DESC_SZ,           /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_FEATURE_UNIT,           /* bDescriptorSubtype */
  AUDIO_OUT_STREAMING_CTRL,             /* bUnitID */
  0x01,                                 /* bSourceID */
  0x01,                                 /* bControlSize */
  AUDIO_CONTROL_MUTE,                   /* bmaControls(0) */
  0x00,                                 /* bmaControls(1) */
  0x00,                                 /* iTerminal */
  /* 09 byte*/

  /*USB Speaker Output Terminal Descriptor */
  AUDIO_OUTPUT_TERMINAL_DESC_SIZE,      /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_OUTPUT_TERMINAL,        /* bDescriptorSubtype */
  0x03,                                 /* bTerminalID */
  0x01,                                 /* wTerminalType  0x0301*/
  0x03,
  0x00,                                 /* bAssocTerminal */
  0x02,                                 /* bSourceID */
  0x00,                                 /* iTerminal */
  /* 09 byte*/

  /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero Bandwith */
  /* Interface 1, Alternate Setting 0                                             */
  AUDIO_INTERFACE_DESC_SIZE,  			/* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */
  0x01,                                 /* bInterfaceNumber */
  0x00,                                 /* bAlternateSetting */
  0x00,                                 /* bNumEndpoints */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/

  /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Operational */
  /* Interface 1, Alternate Setting 1                                           */
  AUDIO_INTERFACE_DESC_SIZE,  			/* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */
  0x01,                                 /* bInterfaceNumber */
  0x01,                                 /* bAlternateSetting */
  0x02,                                 /* bNumEndpoints - Audio Out and Feedback enpoint*/
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/

  /* USB Speaker Audio Streaming Interface Descriptor */
  AUDIO_STREAMING_INTERFACE_DESC_SIZE,  /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_STREAMING_GENERAL,              /* bDescriptorSubtype */
  0x01,                                 /* bTerminalLink */
  0x01,                                 /* bDelay */
  0x01,                                 /* wFormatTag AUDIO_FORMAT_PCM  0x0001*/
  0x00,
  /* 07 byte*/

  /* USB Speaker Audio Type III Format Interface Descriptor */
  AUDIO_FORMAT_TYPE_III_DESC_SZ,        /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_STREAMING_FORMAT_TYPE,          /* bDescriptorSubtype */
  AUDIO_FORMAT_TYPE_III,                /* bFormatType */
  0x02,                                 /* bNrChannels */
  0x02,                                 /* bSubFrameSize :  2 Bytes per frame (16bits) */
  16,                                   /* bBitResolution (16-bits per sample) */
  0x01,                                 /* bSamFreqType only one frequency supported */
  SAMPLE_FREQ(USBD_AUDIO_FREQ),         /* Audio sampling frequency coded on 3 bytes */
  /* 11 byte*/

  /* Endpoint 1 - Standard Descriptor */
    AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
    USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */
    AUDIO_OUT_EP,                         /* bEndpointAddress 1 out endpoint*/
    0x05,							        /* bmAttributes */
    196,0,    /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
    0x01,                                 /* bInterval */
    0x01,                                 /* bRefresh */
    AUDIO_IN_EP,                                 /* bSynchAddress */
    /* 09 byte*/

    /* Endpoint - Audio Streaming (Class-specific) Descriptor*/
     AUDIO_STREAMING_ENDPOINT_DESC_SIZE,   /* bLength */
     AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
     AUDIO_ENDPOINT_GENERAL,               /* bDescriptor */
     0x00,                                 /* bmAttributes */
     0x00,                                 /* bLockDelayUnits */
     0x00,                                 /* wLockDelay */
     0x00,
     /* 07 byte*/

  /* Endpoint FB - Standard Descriptor */
  AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
  USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */
  AUDIO_IN_EP,                         /* bEndpointAddress 2 in endpoint*/
  0x11,//was 0x15        						   /* bmAttributes */
  3,0,   						    /* wMaxPacketSize in Bytes 3 */
  1,//was 1                                 /* bInterval 1ms*/
  SOF_RATE,                                 /* bRefresh 2ms*/
  0x00,                                 /* bSynchAddress */
  /* 09 byte*/


} ;
#endif

/** @defgroup usbd_audio_Private_Functions
  * @{
  */ 

uint32_t feedback_data __attribute__ ((aligned(4))) = 0x0C0000;//(47<<14)+((991<<14)/1000); //was 0x0BFFC6
uint32_t accum __attribute__ ((aligned(4))) = 0x0C0000;//(47<<14)+((991<<14)/1000); //was 0x0BFFC6
//uint32_t FB_rate = (USBD_AUDIO_FREQ == 44100)? (44 << 14) + ((1 << 14) / 10) : (USBD_AUDIO_FREQ / 1000) << 14;

//
// Sampling Frequency Control
//  bmRequestType: 00100010B (SET_), 10100010B (GET_)  Type:class, Recipient:endpoint
//  bRequest     : GET_ and SET_ (CUR, MIN, MAX, RES)
//  wValue       : high byte: SAMPLING_FREQ_CONTROL (0x01), low byte: 0
//  wIndex       ; high byte: 0, lowbyte: target endpoint address (including direction bit)
//  wLength      : length of frequency value ( 3 bytes )
//  Data         : sampling frequency in Hz, 3 bytes, little-endian
//

#define SAMPLING_FREQ_CONTROL       0x01
#define SAMPLING_FREQ_CONTROL_WLEN  3

#define PUT_SAMPLING_FREQ( buf, value ) buf[0] = ((value) >>  0) & 0x000000FF;\
                                        buf[1] = ((value) >>  8) & 0x000000FF;\
                                        buf[2] = ((value) >> 16) & 0x000000FF

/**
* @brief  usbd_audio_Init
*         Initilaizes the AUDIO interface.
* @param  pdev: device instance
* @param  cfgidx: Configuration index
* @retval status
*/
static uint8_t  usbd_audio_Init (void  *pdev, 
                                 uint8_t cfgidx)
{  
  /* Open EP OUT */
  DCD_EP_Open(pdev,
              AUDIO_OUT_EP,
              AUDIO_OUT_PACKET+16,
              USB_OTG_EP_ISOC);

  /* Open EP IN */
  DCD_EP_Open(pdev,
		  	  AUDIO_IN_EP,
              3,
              USB_OTG_EP_ISOC);
  DCD_EP_Flush(pdev,AUDIO_IN_EP);

  DCD_EP_Open(pdev,
		  	  CDC_STATE_IN_EP,
		  	  VIRTUAL_COM_PORT_INT_SIZE,
              USB_OTG_EP_INT);
  DCD_EP_Open(pdev,
              CDC_IN_DATA_EP,
              VIRTUAL_COM_PORT_DATA_SIZE,
              USB_OTG_EP_BULK);
  DCD_EP_Open(pdev,
		  	  CDC_OUT_DATA_EP,
		  	  VIRTUAL_COM_PORT_DATA_SIZE,
              USB_OTG_EP_BULK);
  DCD_EP_PrepareRx(pdev,CDC_OUT_DATA_EP,(uint8_t*)(USB_Rx_Buffer),VIRTUAL_COM_PORT_DATA_SIZE);

  /* Initialize the Audio output Hardware layer */
  if (AUDIO_OUT_fops.Init(USBD_AUDIO_FREQ, DEFAULT_VOLUME, 0) != USBD_OK)
  {
    return USBD_FAIL;
  }
  flag=1;
  return USBD_OK;
}

/**
* @brief  usbd_audio_Init
*         DeInitializes the AUDIO layer.
* @param  pdev: device instance
* @param  cfgidx: Configuration index
* @retval status
*/
static uint8_t  usbd_audio_DeInit (void  *pdev, 
                                   uint8_t cfgidx)
{ 
  DCD_EP_Close (pdev , AUDIO_OUT_EP);
  DCD_EP_Close (pdev , AUDIO_IN_EP);
  DCD_EP_Close (pdev , CDC_STATE_IN_EP);
  DCD_EP_Close (pdev , CDC_IN_DATA_EP);
  DCD_EP_Close (pdev , CDC_OUT_DATA_EP);
  flag=0;
  
  /* DeInitialize the Audio output Hardware layer */
  if (AUDIO_OUT_fops.DeInit(0) != USBD_OK)
  {
    return USBD_FAIL;
  }
  
  return USBD_OK;
}

/**
  * @brief  usbd_audio_Setup
  *         Handles the Audio control request parsing.
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  usbd_audio_Setup (void  *pdev, 
                                  USB_SETUP_REQ *req)
{
  uint16_t len;
  uint8_t  *pbuf;
  uint32_t tmp;
  
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    /* AUDIO Class Requests -------------------------------*/
  case USB_REQ_TYPE_CLASS :    
    switch (req->bRequest)
    {
    case AUDIO_REQ_GET_CUR:
      AUDIO_Req_GetCurrent(pdev, req);
      break;
      
    case AUDIO_REQ_SET_CUR:
      AUDIO_Req_SetCurrent(pdev, req);   
      break;

    case CDC_REQ_SET_LINE_CODING:
    	/* Set the value of the current command to be processed */
    	cdcCmd = req->bRequest;
    	cdcLen = req->wLength;
    	/* Prepare the reception of the buffer over EP0
    	 Next step: the received data will be managed in usbd_cdc_EP0_TxSent()
    	 function. */
    	USBD_CtlPrepareRx (pdev,CmdBuff, req->wLength);
    	break;

    case CDC_REQ_GET_LINE_CODING:
        /* Get the data to be sent to Host from interface layer */
        APP_FOPS.pIf_Ctrl(req->bRequest, CmdBuff, req->wLength);
        /* Send the data to the host */
        USBD_CtlSendData (pdev,
                          CmdBuff,
                          req->wLength);
    	break;

    case CDC_REQ_SET_CONTROL_LINE_STATE:
    	return USBD_OK;

    default:
      USBD_CtlError (pdev, req);
      return USBD_FAIL;
    }
    break;
    
    /* Standard Requests -------------------------------*/
  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_DESCRIPTOR: 
      if( (req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE)
      {
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
        pbuf = usbd_audio_Desc;   
#else
        pbuf = usbd_audio_CfgDesc + 18;
#endif 
        len = MIN(USB_AUDIO_DESC_SIZ , req->wLength);
      }
      
      USBD_CtlSendData (pdev, 
                        pbuf,
                        len);
      break;
      
    case USB_REQ_GET_INTERFACE :
      if ((uint8_t)(req->wIndex)==1) //Audio streaming interface
    	USBD_CtlSendData (pdev,
                        (uint8_t *)&usbd_audio_AltSet,
                        1);
      else if ((uint8_t)(req->wIndex)==3) { //CDC data interface
    	  USBD_CtlSendData (pdev,(uint8_t *)&usbd_cdc_AltSet,1);
	};
      break;
      
    case USB_REQ_SET_INTERFACE :

    	if ((uint8_t)(req->wValue) < AUDIO_TOTAL_IF_NUM && (uint8_t)(req->wIndex)==1) //Alt Setting for audio
      {
        usbd_audio_AltSet = (uint8_t)(req->wValue);
        if (usbd_audio_AltSet == 1)
        {SOF_num=0; };
        if (usbd_audio_AltSet == 0)
        {PlayFlag=0; memset(Audio2,0,384); memset(Audio,0,384);};
        flag=0;
        DCD_EP_Flush(pdev,AUDIO_IN_EP);
      }
      else if ((uint8_t)(req->wIndex)!=1) //Alt Setting for CDC
    	      {
    			usbd_cdc_AltSet = (uint8_t)(req->wValue);
    	      }
      else
      {
        /* Call the error management function (command will be nacked */
        USBD_CtlError (pdev, req);
      }
      break;
    }
  }
  return USBD_OK;
}

/**
  * @brief  usbd_audio_EP0_RxReady
  *         Handles audio control requests data.
  * @param  pdev: device device instance
  * @retval status
  */

/* Prepare the reception of the buffer over EP0 */
/*USBD_CtlPrepareRx (pdev,
                   AudioCtl,
                   req->wLength);

/* Set the global variables indicating current request and its length
to the function usbd_audio_EP0_RxReady() which will process the request */
/*AudioCtlCmd = AUDIO_REQ_SET_CUR;     /* Set the request value */
/*AudioCtlLen = req->wLength;          /* Set the request data length */
/*AudioCtlUnit = HIBYTE(req->wIndex);  /* Set the request target unit */

static uint8_t  usbd_audio_EP0_RxReady (void  *pdev)
{ 
  uint32_t temp;
	/* Check if an AudioControl request has been issued */
  if (AudioCtlCmd == AUDIO_REQ_SET_CUR)
  {/* In this driver, to simplify code, only SET_CUR request is managed */
    /* Check for which addressed unit the AudioControl request has been issued */

	if (AudioCtlUnit == AUDIO_OUT_STREAMING_CTRL)
    {/* In this driver, to simplify code, only one unit is manage */
      /* Call the audio interface mute function */
      AUDIO_OUT_fops.MuteCtl(AudioCtl[0]);
      
      /* Reset the AudioCtlCmd variable to prevent re-entering this function */
      AudioCtlCmd = 0;
      AudioCtlLen = 0;
    }
  }
  if (AudioCtlCmd == AUDIO_REQ_GET_CUR)
    {
  	  temp=48000;
     AudioCtl[0]=(uint8_t)(temp&0xff);
     AudioCtl[1]=(uint8_t)((temp>>8)&0xff);
     AudioCtl[2]=(uint8_t)((temp>>16)&0xff);
    /* Reset the AudioCtlCmd variable to prevent re-entering this function */
    AudioCtlCmd = 0;
    AudioCtlLen = 0;
    }

  if (cdcCmd != NO_CMD)
  {
    /* Process the data */
    APP_FOPS.pIf_Ctrl(cdcCmd, CmdBuff, cdcLen);

    /* Reset the command variable to default value */
    cdcCmd = NO_CMD;
  }
  return USBD_OK;
}

/**
  * @brief  usbd_audio_DataIn
  *         Handles the audio IN data stage.
  * @param  pdev: instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  usbd_audio_DataIn (void *pdev, uint8_t epnum)
{
	if (epnum == (AUDIO_IN_EP&0x7f))
	{
		SOF_num=0;
		DCD_EP_Tx (pdev, AUDIO_IN_EP, (uint8_t *) &feedback_data, 3);
	};
	if (epnum==(CDC_IN_DATA_EP&0x7f))
	{  if (USB_Tx_State == 1)
	  {
	    if (APP_Rx_length == 0)
	    {
	      USB_Tx_State = 0;
	    }
	    else
	    {
	      if (APP_Rx_length > CDC_DATA_IN_PACKET_SIZE){
	        USB_Tx_ptr = APP_Rx_ptr_out;
	        USB_Tx_length = CDC_DATA_IN_PACKET_SIZE;

	        APP_Rx_ptr_out += CDC_DATA_IN_PACKET_SIZE;
	        APP_Rx_length -= CDC_DATA_IN_PACKET_SIZE;
	      }
	      else
	      {
	        USB_Tx_ptr = APP_Rx_ptr_out;
	        USB_Tx_length = APP_Rx_length;

	        APP_Rx_ptr_out += APP_Rx_length;
	        APP_Rx_length = 0;
	      }

	      /* Prepare the available data buffer to be sent on IN endpoint */
	      DCD_EP_Tx (pdev,CDC_IN_DATA_EP,(uint8_t*)&APP_Rx_Buffer[USB_Tx_ptr],USB_Tx_length);
	    }
	  }
	};

	return USBD_OK;
}

/**
  * @brief  usbd_audio_DataOut
  *         Handles the Audio Out data stage.
  * @param  pdev: instance
  * @param  epnum: endpoint number
  * @retval status
  */


static uint8_t  usbd_audio_DataOut (void *pdev, uint8_t epnum)
{     
	  uint32_t curr_length,curr_pos,rest;

	if (epnum == AUDIO_OUT_EP)
  {
	  curr_length=USBD_GetRxCount (pdev,epnum);
	  curr_pos=(IsocOutWrPtr-IsocOutBuff);
	  rest=TOTAL_OUT_BUF_SIZE-curr_pos;
	  //monitor sample rate conversion
	  if (curr_length<AUDIO_OUT_PACKET) {STM_EVAL_LEDToggle(LED3);};
	  if (curr_length>AUDIO_OUT_PACKET) {STM_EVAL_LEDToggle(LED5);};

	  if (rest<curr_length)
	  {
	  if (rest>0)
	  {memcpy((uint8_t*)IsocOutWrPtr,tmpbuf,rest);
	  IsocOutWrPtr = IsocOutBuff;
	  curr_length-=rest;
	  };
	  if ((curr_length)>0)
	  {memcpy((uint8_t*)IsocOutWrPtr,(uint8_t *)(&tmpbuf[0]+rest),curr_length);
	  IsocOutWrPtr+=curr_length;};
	  }
	  else
	  {
	  if (curr_length>0)
	  {memcpy((uint8_t*)IsocOutWrPtr,tmpbuf,curr_length);
	  // Increment the Buffer pointer
	  IsocOutWrPtr += curr_length;};
  	  }
	  //roll it back when all buffers are full
	  if (IsocOutWrPtr >= (IsocOutBuff + (TOTAL_OUT_BUF_SIZE)))
		  IsocOutWrPtr = IsocOutBuff;
    /* Toggle the frame index */
    ((USB_OTG_CORE_HANDLE*)pdev)->dev.out_ep[epnum].even_odd_frame =
      (((USB_OTG_CORE_HANDLE*)pdev)->dev.out_ep[epnum].even_odd_frame)? 0:1;
	   DCD_EP_PrepareRx(pdev,
	                     AUDIO_OUT_EP,
	                     (uint8_t*)tmpbuf,
	                     AUDIO_OUT_PACKET+16);
    /* Trigger the start of streaming only when half buffer is full */
    if ((PlayFlag == 0) && (IsocOutWrPtr >= (IsocOutBuff + TOTAL_OUT_BUF_SIZE/2)))
    {
      /* Enable start of Streaming */
      PlayFlag = 1;
      AUDIO_OUT_fops.AudioCmd((uint8_t*)(IsocOutRdPtr),  /* Samples buffer pointer */
                          AUDIO_OUT_PACKET,          /* Number of samples in Bytes */
                          AUDIO_CMD_PLAY);           /* Command to be processed */
    }
    counter+=1;
  };
	if (epnum==CDC_OUT_DATA_EP)
	{  curr_length=USBD_GetRxCount (pdev,epnum);
	  /* USB data will be immediately processed, this allow next USB traffic being
	     NAKed till the end of the application Xfer */
	  APP_FOPS.pIf_DataRx(USB_Rx_Buffer, curr_length);

	  /* Prepare Out endpoint to receive next packet */
	  DCD_EP_PrepareRx(pdev,CDC_OUT_DATA_EP,(uint8_t*)(USB_Rx_Buffer),VIRTUAL_COM_PORT_DATA_SIZE);
	};

  return USBD_OK;
}

/**
  * @brief  usbd_audio_SOF
  *         Handles the SOF event (data buffer update and synchronization).
  * @param  pdev: instance
  * @param  epnum: endpoint number
  * @retval status
  */

static uint8_t  usbd_audio_SOF (void *pdev)
{     uint8_t res;
static uint16_t n;
USB_OTG_DSTS_TypeDef  FS_DSTS;
static uint32_t FrameCount = 0;

if ((counter-old_counter)==0)
		PlayFlag=0;
		old_counter=counter;
  /* Check if there are available data in stream buffer.
    In this function, a single variable (PlayFlag) is used to avoid software delays.
    The play operation must be executed as soon as possible after the SOF detection. */
if (usbd_audio_AltSet==1)
{
	shift=0;
	if (PlayFlag==1)
	{
	gap=(IsocOutWrPtr-IsocOutRdPtr);
	tmpxx=(DMA1_Stream7->NDTR)%96;
	if (tmpxx==0) tmpxx+=96;
	if (gap<0) gap+=(TOTAL_OUT_BUF_SIZE);
	shift=-(gap+tmpxx*2-(TOTAL_OUT_BUF_SIZE/2))>>3;
	};

	accum+=(TIM2->CCR1);
	if (shift!=0) accum+=shift;
	SOF_num++;
	if (SOF_num==(1<<SOF_RATE))
		{if (SOF_RATE>6)
			{feedback_data+=accum>>(SOF_RATE-6);}
		else
			{feedback_data+=accum<<(6-SOF_RATE);};
		feedback_data>>=1;
		SOF_num=0;
		accum=0;
		//flag=0;
	}

	if ((!flag))
		{
					{//feedback_data=722534;
					DCD_EP_Tx (pdev, AUDIO_IN_EP, (uint8_t *) &feedback_data, 3);
					flag=1;
				};
			};

  }

if (FrameCount++ == CDC_IN_FRAME_INTERVAL)
{
  Handle_USBAsynchXfer(pdev);

	/* Reset the frame counter */
  FrameCount = 0;

  /* Check the data to be sent through IN pipe */

}

return USBD_OK;
}

/**
  * @brief  usbd_audio_OUT_Incplt
  *         Handles the iso out incomplete event.
  * @param  pdev: instance
  * @retval status
  */
static uint8_t  usbd_audio_OUT_Incplt (void  *pdev)
{
  return USBD_OK;
}

static uint8_t  usbd_audio_IN_Incplt (void  *pdev)
{
	DCD_EP_Flush(pdev,AUDIO_IN_EP);
	STM_EVAL_LEDToggle(LED6);
	//feedback_data=0X0C0077;
	DCD_EP_Tx (pdev, AUDIO_IN_EP, (uint8_t *) &feedback_data, 3);
	return USBD_OK;
}



/******************************************************************************
     AUDIO Class requests management
******************************************************************************/
/**
  * @brief  AUDIO_Req_GetCurrent
  *         Handles the GET_CUR Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_Req_GetCurrent(void *pdev, USB_SETUP_REQ *req)
{  
  /* Send the current mute state */
  USBD_CtlSendData (pdev, 
                    AudioCtl,
                    req->wLength);
}

/**
  * @brief  AUDIO_Req_SetCurrent
  *         Handles the SET_CUR Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_Req_SetCurrent(void *pdev, USB_SETUP_REQ *req)
{ 
  if (req->wLength)
  {
    /* Prepare the reception of the buffer over EP0 */
    USBD_CtlPrepareRx (pdev, 
                       AudioCtl,
                       req->wLength);
    
    /* Set the global variables indicating current request and its length 
    to the function usbd_audio_EP0_RxReady() which will process the request */
    AudioCtlCmd = AUDIO_REQ_SET_CUR;     /* Set the request value */
    AudioCtlLen = req->wLength;          /* Set the request data length */
    AudioCtlUnit = HIBYTE(req->wIndex);  /* Set the request target unit */
  }
}

/**
  * @brief  USBD_audio_GetCfgDesc 
  *         Returns configuration descriptor.
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_audio_GetCfgDesc (uint8_t speed, uint16_t *length)
{
  *length = sizeof (usbd_audio_CfgDesc);
  return usbd_audio_CfgDesc;
}

void update_audio_buf(void)
{uint8_t res;
/* Start playing received packet */

if(PlayFlag == 1)
{
	res=AUDIO_OUT_fops.AudioCmd((uint8_t*)(IsocOutRdPtr),  /* Samples buffer pointer */
	                    AUDIO_OUT_PACKET,          /* Number of samples in Bytes */
	                    AUDIO_CMD_PLAY);           /* Command to be processed */
	IsocOutRdPtr += AUDIO_OUT_PACKET;
	if (IsocOutRdPtr >= (IsocOutBuff + (TOTAL_OUT_BUF_SIZE)))
	{/* Roll back to the start of buffer */
	IsocOutRdPtr = IsocOutBuff;
	}
}
/*else {
/*	res=AUDIO_OUT_fops.AudioCmd((uint8_t*)(IsocOutRdPtr),  /* Samples buffer pointer */
//	                    AUDIO_OUT_PACKET,          /* Number of samples in Bytes */
	//                    AUDIO_CMD_PAUSE);           /* Command to be processed */
//}

}

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  VCP_Init
  *         Initializes the Media on the STM32
  * @param  None
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
static uint16_t VCP_Init(void)
{
  return USBD_OK;
}

/**
  * @brief  VCP_DeInit
  *         DeInitializes the Media on the STM32
  * @param  None
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
static uint16_t VCP_DeInit(void)
{
  return USBD_OK;
}


/**
  * @brief  VCP_Ctrl
  *         Manage the CDC class requests
  * @param  Cmd: Command code
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
static uint16_t VCP_Ctrl (uint32_t Cmd, uint8_t* Buf, uint32_t Len)
{
  switch (Cmd)
  {
  case SEND_ENCAPSULATED_COMMAND:
    /* Not  needed for this driver */
    break;

  case GET_ENCAPSULATED_RESPONSE:
    /* Not  needed for this driver */
    break;

  case SET_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case GET_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case CLEAR_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case SET_LINE_CODING://команда настроить контроллер
	  linecoding.bitrate=*(uint32_t *) &Buf[0];
	  linecoding.format =Buf[4] ;
	  linecoding.paritytype = Buf[5];
	  linecoding.datatype = Buf[6];
	  //эта команда вызвываетс€, если на компьютере мен€ютс€ настройки порта
    break;

  case GET_LINE_CODING://команда считать настройки из контроллера
	    Buf[0] = (uint8_t) (linecoding.bitrate);
	  	Buf[1] = (uint8_t) (linecoding.bitrate >> 8);
	  	Buf[2] = (uint8_t) (linecoding.bitrate >> 16);
	  	Buf[3] = (uint8_t) (linecoding.bitrate >> 24);
	  	Buf[4] = linecoding.format;
	  	Buf[5] = linecoding.paritytype;
	  	Buf[6] = linecoding.datatype;
	  break;

  case SET_CONTROL_LINE_STATE:
    /* Not  needed for this driver */
    break;

  case SEND_BREAK:
    /* Not  needed for this driver */
    break;

  default:
    break;
  }

  return USBD_OK;
}


/**
 * @brief  putchar
 *         Sends one char over the USB serial link.
 * @param  buf: char to be sent
 * @retval none
 */

void VCP_put_char(uint8_t buf) {
	VCP_DataTx(&buf, 1);
}

void VCP_send_str(uint8_t* buf) {
	uint32_t i = 0;
	while (*(buf + i)) {
		i++;
	}
	VCP_DataTx(buf, i);
}

void VCP_send_buffer(uint8_t* buf, int len) {
	VCP_DataTx(buf, len);
}

/**
 * @brief  VCP_DataTx
 *         CDC received data to be send over USB IN endpoint are managed in
 *         this function.
 * @param  Buf: Buffer of data to be sent
 * @param  Len: Number of data to be sent (in bytes)
 * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
 */
static uint16_t VCP_DataTx(uint8_t* Buf, uint32_t Len) {
	uint32_t i = 0;
	while (i < Len) {
		APP_Rx_Buffer[APP_Rx_ptr_in] = *(Buf + i);
		APP_Rx_ptr_in++;
		i++;
		/* To avoid buffer overflow */
		if (APP_Rx_ptr_in == APP_RX_DATA_SIZE) {
			APP_Rx_ptr_in = 0;
		}
	}

	return USBD_OK;
}

/**
 * @brief  VCP_DataRx
 *         Data received over USB OUT endpoint are sent over CDC interface
 *         through this function.
 *
 *         @note
 *         This function will block any OUT packet reception on USB endpoint
 *         until exiting this function. If you exit this function before transfer
 *         is complete on CDC interface (ie. using DMA controller) it will result
 *         in receiving more data while previous ones are still not sent.
 *
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
 */


static uint16_t VCP_DataRx(uint8_t* Buf, uint32_t Len) {
	uint32_t i;
	for (i = 0; i < Len; i++) {
		APP_Tx_Buffer[APP_tx_ptr_head] = *(Buf + i);
		APP_tx_ptr_head++;
		if (APP_tx_ptr_head == APP_TX_BUF_SIZE)
			APP_tx_ptr_head = 0;

		if (APP_tx_ptr_head == APP_tx_ptr_tail)
			return USBD_FAIL;
	}

	return USBD_OK;
}

int VCP_get_char(uint8_t *buf) {
	if (APP_tx_ptr_head == APP_tx_ptr_tail)
		return 0;

	*buf = APP_Tx_Buffer[APP_tx_ptr_tail];
	APP_tx_ptr_tail++;
	if (APP_tx_ptr_tail == APP_TX_BUF_SIZE)
		APP_tx_ptr_tail = 0;

	return 1;
}

int VCP_get_string(uint8_t *buf) {
	if (APP_tx_ptr_head == APP_tx_ptr_tail)
		return 0;

	while (!APP_Tx_Buffer[APP_tx_ptr_tail]
			|| APP_Tx_Buffer[APP_tx_ptr_tail] == '\n'
			|| APP_Tx_Buffer[APP_tx_ptr_tail] == '\r') {
		APP_tx_ptr_tail++;
		if (APP_tx_ptr_tail == APP_TX_BUF_SIZE)
			APP_tx_ptr_tail = 0;
		if (APP_tx_ptr_head == APP_tx_ptr_tail)
			return 0;
	}

	int i = 0;
	do {
		*(buf + i) = APP_Tx_Buffer[i + APP_tx_ptr_tail];
		i++;

		if ((APP_tx_ptr_tail + i) == APP_TX_BUF_SIZE)
			i = -APP_tx_ptr_tail;
		if (APP_tx_ptr_head == (APP_tx_ptr_tail + i))
			return 0;

	} while (APP_Tx_Buffer[APP_tx_ptr_tail + i]
			&& APP_Tx_Buffer[APP_tx_ptr_tail + i] != '\n'
			&& APP_Tx_Buffer[APP_tx_ptr_tail + i] != '\r');

	*(buf + i) = 0;
	APP_tx_ptr_tail += i;
	if (APP_tx_ptr_tail >= APP_TX_BUF_SIZE)
		APP_tx_ptr_tail -= APP_TX_BUF_SIZE;
	return i;
}

static void Handle_USBAsynchXfer (void *pdev)
{
  uint16_t USB_Tx_ptr;
  uint16_t USB_Tx_length;

  if(USB_Tx_State != 1)
  {
    if (APP_Rx_ptr_out == APP_RX_DATA_SIZE)
    {
      APP_Rx_ptr_out = 0;
    }

    if(APP_Rx_ptr_out == APP_Rx_ptr_in)
    {
      USB_Tx_State = 0;
      return;
    }

    if(APP_Rx_ptr_out > APP_Rx_ptr_in) /* rollback */
    {
      APP_Rx_length = APP_RX_DATA_SIZE - APP_Rx_ptr_out;

    }
    else
    {
      APP_Rx_length = APP_Rx_ptr_in - APP_Rx_ptr_out;

    }
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
     APP_Rx_length &= ~0x03;
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

    if (APP_Rx_length > VIRTUAL_COM_PORT_DATA_SIZE)
    {
      USB_Tx_ptr = APP_Rx_ptr_out;
      USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;

      APP_Rx_ptr_out += VIRTUAL_COM_PORT_DATA_SIZE;
      APP_Rx_length -= VIRTUAL_COM_PORT_DATA_SIZE;
    }
    else
    {
      USB_Tx_ptr = APP_Rx_ptr_out;
      USB_Tx_length = APP_Rx_length;

      APP_Rx_ptr_out += APP_Rx_length;
      APP_Rx_length = 0;
    }
    USB_Tx_State = 1;
    DCD_EP_Tx (pdev,CDC_IN_DATA_EP,(uint8_t*)&APP_Rx_Buffer[USB_Tx_ptr],USB_Tx_length);
  }
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
