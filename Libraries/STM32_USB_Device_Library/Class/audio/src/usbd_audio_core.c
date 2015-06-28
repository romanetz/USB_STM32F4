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
#include "usbd_ioreq.h"
extern volatile uint8_t audio_buffer_fill;
static volatile uint8_t flag=0;
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
uint8_t  IsocOutBuff [TOTAL_OUT_BUF_SIZE * 2] __attribute__ ((aligned(4))); //48 bytes
uint8_t* IsocOutWrPtr = IsocOutBuff;
uint8_t* IsocOutRdPtr = IsocOutBuff;

/* Main Buffer for Audio Control Rrequests transfers and its relative variables */
uint8_t  AudioCtl[64];
uint8_t  AudioCtlCmd = 0;
uint32_t AudioCtlLen = 0;
uint8_t  AudioCtlUnit = 0;
#define SOF_RATE 0x1
static uint32_t PlayFlag = 0;
static volatile  uint16_t SOF_num=0;
static volatile uint32_t  usbd_audio_AltSet = 0;
static uint8_t usbd_audio_CfgDesc[AUDIO_CONFIG_DESC_SIZE];

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
/*#ifndef sync
#define sync
#endif*/
#ifdef sync
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
    196,0,    							/* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
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

  /* Endpoint 2 - Standard Descriptor */
  AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
  USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */
  AUDIO_IN_EP,                         /* bEndpointAddress 2 in endpoint*/
  0x11,//was 0x15        						   /* bmAttributes */
  3,0,   						    /* wMaxPacketSize in Bytes 4 */
  SOF_RATE,
  SOF_RATE,
  //SOF_RATE,//was SOF_RATE                                 /* bInterval 1ms*/
  //SOF_RATE,                                 /* bRefresh 2ms*/
  0x00,                                 /* bSynchAddress */
  /* 09 byte*/

} ;
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

  /* Initialize the Audio output Hardware layer */
  if (AUDIO_OUT_fops.Init(USBD_AUDIO_FREQ, DEFAULT_VOLUME, 0) != USBD_OK)
  {
    return USBD_FAIL;
  }
  /* Prepare Out endpoint to receive audio data */
  //first time when we start
  DCD_EP_PrepareRx(pdev,
                   AUDIO_OUT_EP,
                   (uint8_t*)IsocOutBuff,                        
                   AUDIO_OUT_PACKET);  
  IsocOutWrPtr+=AUDIO_OUT_PACKET;
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
  DCD_EP_Flush (pdev , AUDIO_OUT_EP);
  DCD_EP_Flush (pdev , AUDIO_IN_EP);
  DCD_EP_Close (pdev , AUDIO_OUT_EP);
  DCD_EP_Close (pdev , AUDIO_IN_EP);
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
  USB_OTG_GRXFSTS_TypeDef FS_DSTS;
  USB_OTG_DEPCTL_TypeDef EP81_Ctrl;
  
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
      USBD_CtlSendData (pdev,
                        (uint8_t *)&usbd_audio_AltSet,
                        1);
      break;
      
    case USB_REQ_SET_INTERFACE :
      if ((uint8_t)(req->wValue) < AUDIO_TOTAL_IF_NUM)
      {
        usbd_audio_AltSet = (uint8_t)(req->wValue);
        if (usbd_audio_AltSet == 1)
        {SOF_num=0;
        flag=0;
        //FS_DSTS.d32=USB_OTG_READ_REG32(((USB_OTG_CORE_HANDLE *)pdev)->regs.DREGS->DSTS);
        DCD_EP_Flush(pdev,AUDIO_IN_EP);
        //DCD_EP_Tx (pdev, AUDIO_IN_EP, (uint8_t *) &feedback_data, 3);
        };
        if (usbd_audio_AltSet == 0)
        {flag=0;
        	DCD_EP_Flush(pdev,AUDIO_IN_EP);
        }
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
		flag=0;
		SOF_num=0;
	}
	return USBD_OK;
}

/**
  * @brief  usbd_audio_DataOut
  *         Handles the Audio Out data stage.
  * @param  pdev: instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t tmpbuf[AUDIO_OUT_PACKET+4] __attribute__ ((aligned(4)));

static volatile uint16_t rest;
static volatile uint16_t max_length;

static uint8_t  usbd_audio_DataOut (void *pdev, uint8_t epnum)
{     
	  uint16_t curr_length;
	  uint8_t flag;
	  uint16_t curr_pos,rest;
	  static uint16_t tmp;

	if (epnum == AUDIO_OUT_EP)
  {
	  curr_length=USBD_GetRxCount (pdev,epnum);
	   DCD_EP_PrepareRx(pdev,
	                     AUDIO_OUT_EP,
	                     (uint8_t*)tmpbuf,
	                     curr_length);
	  curr_pos=(IsocOutWrPtr-IsocOutBuff);
	  rest=(AUDIO_OUT_PACKET * OUT_PACKET_NUM)-curr_pos;
	  //if (curr_length<AUDIO_OUT_PACKET) {STM_EVAL_LEDToggle(LED3);};
	  //if (curr_length>AUDIO_OUT_PACKET) {STM_EVAL_LEDToggle(LED4);};
	  if (rest<curr_length)
	  { //STM_EVAL_LEDToggle(LED4);
	  if (rest>0)
	  {memcpy((uint8_t*)IsocOutWrPtr,tmpbuf,rest);
	  IsocOutWrPtr = IsocOutBuff;};
	  if ((curr_length-rest)>0)
	  {memcpy((uint8_t*)IsocOutWrPtr,tmpbuf+rest,curr_length-rest);
	  IsocOutWrPtr+=curr_length-rest;};
	  }
	  else
	  {
	  if (curr_length>0)
	  {memcpy((uint8_t*)IsocOutWrPtr,tmpbuf,curr_length);
	  // Increment the Buffer pointer
	  IsocOutWrPtr += curr_length;};
  	  }
	  //roll it back when all buffers are full
	  if (IsocOutWrPtr >= (IsocOutBuff + (AUDIO_OUT_PACKET * OUT_PACKET_NUM)))
		  IsocOutWrPtr = IsocOutBuff;
    /* Toggle the frame index */
    ((USB_OTG_CORE_HANDLE*)pdev)->dev.out_ep[epnum].even_odd_frame =
      (((USB_OTG_CORE_HANDLE*)pdev)->dev.out_ep[epnum].even_odd_frame)? 0:1;
    //max_length=(AUDIO_OUT_PACKET * OUT_PACKET_NUM)-(IsocOutWrPtr-IsocOutBuff);
    //max_length=(max_length>(AUDIO_OUT_PACKET+4))?(AUDIO_OUT_PACKET+4):max_length;
    //if ((max_length<AUDIO_OUT_PACKET)&&(max_length>0))   STM_EVAL_LEDToggle(LED4);
    /* Prepare Out endpoint to receive next audio packet */
   /* DCD_EP_PrepareRx(pdev,
                     AUDIO_OUT_EP,
                     (uint8_t*)tmpbuf,
                     196);
*/
    /* Trigger the start of streaming only when half buffer is full */
    if ((PlayFlag == 0) && (IsocOutWrPtr >= (IsocOutBuff + ((AUDIO_OUT_PACKET * OUT_PACKET_NUM) / 2))))
    {
      /* Enable start of Streaming */
      PlayFlag = 1;
      //DCD_EP_Tx(pdev,AUDIO_IN_EP,(uint8_t*)&feedback_data,3);
    }
  }

  return USBD_OK;
}

/**
  * @brief  usbd_audio_SOF
  *         Handles the SOF event (data buffer update and synchronization).
  * @param  pdev: instance
  * @param  epnum: endpoint number
  * @retval status
  */
static volatile int32_t gap,corr,oldgap,dgap,tmpxx;

#define FB_RATE_DELTA (1<<12)
#define FB_RATE_DELTA_NUM 2

static uint8_t  usbd_audio_SOF (void *pdev)
{     uint8_t res;
static uint16_t n;

  /* Check if there are available data in stream buffer.
    In this function, a single variable (PlayFlag) is used to avoid software delays.
    The play operation must be executed as soon as possible after the SOF detection. */
if (usbd_audio_AltSet==1)
{
	if (PlayFlag)
  {
	oldgap=gap;
	gap=(IsocOutWrPtr-IsocOutRdPtr);
	tmpxx=(DMA1_Stream7->NDTR)%96;
	if (gap<0) gap+=(AUDIO_OUT_PACKET * OUT_PACKET_NUM);
	dgap=gap-oldgap;
	if ((dgap<1024)&&(dgap>-1024))
	corr=dgap;
	else corr=0;
			/* Start playing received packet */
    res=AUDIO_OUT_fops.AudioCmd((uint8_t*)(IsocOutRdPtr),  /* Samples buffer pointer */
                            AUDIO_OUT_PACKET,          /* Number of samples in Bytes */
                            AUDIO_CMD_PLAY);           /* Command to be processed */
    /* Increment the Buffer pointer or roll it back when all buffers all full */  
	IsocOutRdPtr += AUDIO_OUT_PACKET; //48 samples*4bytes/sample=192 bytes
    if (IsocOutRdPtr >= (IsocOutBuff + (AUDIO_OUT_PACKET * OUT_PACKET_NUM)))
    {/* Roll back to the start of buffer */
      IsocOutRdPtr = IsocOutBuff;
    }
  };
		//if (PlayFlag)

		/*	if (gap>(AUDIO_OUT_PACKET * OUT_PACKET_NUM/2))
			feedback_data+=0x10; else
				if (gap<(AUDIO_OUT_PACKET * OUT_PACKET_NUM/2))
					feedback_data-=0x10;*/
	accum+=(TIM2->CCR1);
			SOF_num++;
			if ((!flag))
							{
								if (SOF_RATE>6)
								{feedback_data=accum>>(SOF_RATE-6);}
								else
								{feedback_data=accum<<(6-SOF_RATE);};
								//feedback_data-=corr;
								//if (SOF_num==(1<<SOF_RATE)-1) DCD_EP_Flush (pdev, AUDIO_IN_EP);
								DCD_EP_Tx (pdev, AUDIO_IN_EP, (uint8_t *) &feedback_data, 3);
								accum=0;
								flag=1;
								SOF_num=0;
							};

    //else
    	/* Increment to the next sub-buffer */

    /* If all available buffers have been consumed, stop playing */
    if (IsocOutRdPtr == IsocOutWrPtr)
    {    
      // Pause the audio stream
      AUDIO_OUT_fops.AudioCmd((uint8_t*)(IsocOutBuff),   // Samples buffer pointer
                              AUDIO_OUT_PACKET,          // Number of samples in Bytes
                              AUDIO_CMD_PAUSE);          // Command to be processed
      
      //
      // Stop entering play loop
      PlayFlag = 0;
      // Reset buffer pointers
     // IsocOutRdPtr = IsocOutBuff;
      //IsocOutWrPtr = IsocOutBuff;
    }

//#ifdef sync

//#endif

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
	if ((SOF_num>((1<<SOF_RATE)-1))&&flag)
	   {flag=0;
	   	DCD_EP_Flush(pdev,AUDIO_IN_EP);
	   	STM_EVAL_LEDToggle(LED4);
	   };
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
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
