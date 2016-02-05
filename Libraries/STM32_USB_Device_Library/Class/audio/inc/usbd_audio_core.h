/**
  ******************************************************************************
  * @file    usbd_audio_core.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    22-July-2011
  * @brief   header file for the usbd_audio_core.c file.
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

#ifndef __USB_AUDIO_CORE_H_
#define __USB_AUDIO_CORE_H_

#include "usbd_ioreq.h"
#include "usbd_req.h"
#include "usbd_desc.h"




/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup usbd_audio
  * @brief This file is the Header file for USBD_audio.c
  * @{
  */ 


/** @defgroup usbd_audio_Exported_Defines
  * @{
  */ 

/* AudioFreq * DataSize (2 bytes) * NumChannels (Stereo: 2) */
#define AUDIO_OUT_PACKET                              (uint32_t)(((USBD_AUDIO_FREQ * 2 * 2) /1000)) 

/* Number of sub-packets in the audio transfer buffer. You can modify this value but always make sure
  that it is an even number and higher than 3 */
#define OUT_PACKET_NUM                                   8
/* Total size of the audio transfer buffer */
#define TOTAL_OUT_BUF_SIZE                           ((uint32_t)(AUDIO_OUT_PACKET * OUT_PACKET_NUM))

void update_audio_buf(void);

#define CDC_OUT_DATA_EP								  0x02
#define CDC_IN_DATA_EP								  0x82
#define CDC_STATE_IN_EP								  0x83

#define CDC_REQ_SET_LINE_CODING						  0x20
#define CDC_REQ_GET_LINE_CODING						  0x21
#define CDC_REQ_SET_CONTROL_LINE_STATE				  0x22

#define CDC_INTERFACE_DESCRIPTOR_TYPE                 0x24
#define CDC_ENDPOINT_DESCRIPTOR_TYPE                  0x25
#define VIRTUAL_COM_PORT_INT_SIZE 					  0x0A
#define VIRTUAL_COM_PORT_DATA_SIZE					  0x40
#define APP_RX_DATA_SIZE							  2048

#define USB_INTERFACE_ASSOC_DESCRIPTOR_TYPE			  0x0B
#define INTERFACE_ASSOC_DESC_SIZE					  0x08

#define AUDIO_INTERFACE_DESC_SIZE                     9
#define USB_AUDIO_DESC_SIZ                            0x09
#define AUDIO_STANDARD_ENDPOINT_DESC_SIZE             0x09
#define AUDIO_STREAMING_ENDPOINT_DESC_SIZE            0x07

#define AUDIO_DESCRIPTOR_TYPE                         0x21
#define USB_DEVICE_CLASS_AUDIO                        0x01
#define AUDIO_SUBCLASS_AUDIOCONTROL                   0x01
#define AUDIO_SUBCLASS_AUDIOSTREAMING                 0x02
#define AUDIO_PROTOCOL_UNDEFINED                      0x00
#define AUDIO_STREAMING_GENERAL                       0x01
#define AUDIO_STREAMING_FORMAT_TYPE                   0x02

/* Audio Descriptor Types */
#define AUDIO_INTERFACE_DESCRIPTOR_TYPE               0x24
#define AUDIO_ENDPOINT_DESCRIPTOR_TYPE                0x25

/* Audio Control Interface Descriptor Subtypes */
#define AUDIO_CONTROL_HEADER                          0x01
#define AUDIO_CONTROL_INPUT_TERMINAL                  0x02
#define AUDIO_CONTROL_OUTPUT_TERMINAL                 0x03
#define AUDIO_CONTROL_FEATURE_UNIT                    0x06

#define AUDIO_INPUT_TERMINAL_DESC_SIZE                0x0C
#define AUDIO_OUTPUT_TERMINAL_DESC_SIZE               0x09
#define AUDIO_STREAMING_INTERFACE_DESC_SIZE           0x07

#define AUDIO_CONTROL_MUTE                            0x0001

#define AUDIO_FORMAT_TYPE_I                           0x01
#define AUDIO_FORMAT_TYPE_III                         0x03

#define USB_ENDPOINT_TYPE_ASYNC_FEEDBACK  			  0x15
#define USB_ENDPOINT_TYPE_ISOCHRONOUS                 0x01
#define USB_ENDPOINT_TYPE_BULK		                  0x02
#define USB_ENDPOINT_TYPE_INTERRUPT	                  0x03
#define AUDIO_ENDPOINT_GENERAL                        0x01

#define AUDIO_REQ_GET_CUR                             0x81
#define AUDIO_REQ_SET_CUR                             0x01

#define AUDIO_OUT_STREAMING_CTRL                      0x02

#define USB_CONFIGUARTION_DESC_SIZE					  0x09
#define AUDIO_OUTPUT_TERMINAL_DESC_SIZE 			  0x09
#define AUDIO_FEATURE_UNIT_DESC_SZ					  0x0A
#define AUDIO_FORMAT_TYPE_III_DESC_SZ				  0x0B
#define AUDIO_FORMAT_TYPE_I_DESC_SZ				      0x0B

#define sync
#ifdef sync
#define AUDIO_CONFIG_DESC_SIZE						  193
//119
#else
#define AUDIO_CONFIG_DESC_SIZE						  109
#endif

/*
#define AUDIO_CONFIG_DESC_SIZE					\
 	USB_CONFIGUARTION_DESC_SIZE         + \
    AUDIO_INTERFACE_DESC_SIZE           + \
    AUDIO_INTERFACE_DESC_SIZE		    + \
    AUDIO_INPUT_TERMINAL_DESC_SIZE      + \
    AUDIO_FEATURE_UNIT_DESC_SZ          + \
    AUDIO_OUTPUT_TERMINAL_DESC_SIZE     + \
    AUDIO_INTERFACE_DESC_SIZE           + \
    AUDIO_INTERFACE_DESC_SIZE           + \
    AUDIO_STREAMING_INTERFACE_DESC_SIZE + \
    AUDIO_FORMAT_TYPE_III_DESC_SZ       + \
    AUDIO_STANDARD_ENDPOINT_DESC_SIZE   + \
    AUDIO_STREAMING_ENDPOINT_DESC_SIZE  + \
    AUDIO_STANDARD_ENDPOINT_DESC_SIZE
*/
/**
  * @}
  */ 


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
typedef struct _Audio_Fops
{
    uint8_t  (*Init)         (uint32_t  AudioFreq, uint32_t Volume, uint32_t options);
    uint8_t  (*DeInit)       (uint32_t options);
    uint8_t  (*AudioCmd)     (uint8_t* pbuf, uint32_t size, uint8_t cmd);
    uint8_t  (*VolumeCtl)    (uint8_t vol);
    uint8_t  (*MuteCtl)      (uint8_t cmd);
    uint8_t  (*PeriodicTC)   (uint8_t cmd);
    uint8_t  (*GetState)     (void);
}AUDIO_FOPS_TypeDef;
/**
  * @}
  */ 



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */ 
#define AUDIO_PACKET_SZE(frq)          (uint8_t)(((frq * 2 * 2)/1000+4) & 0xFF), \
                                       (uint8_t)((((frq * 2 * 2)/1000+4) >> 8) & 0xFF)
#define SAMPLE_FREQ(frq)               (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))
/**
  * @}
  */ 

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */ 

extern USBD_Class_cb_TypeDef  AUDIO_cb;

/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
/**
  * @}
  */ 

#define CDC_DATA_IN_PACKET_SIZE                 0x40
#define CDC_IN_FRAME_INTERVAL					5
#define CDC_DATA_OUT_PACKET_SIZE				0x40
//*(uint16_t *)(((USB_OTG_CORE_HANDLE *)pdev)->dev.pConfig_descriptor + 64)

/*---------------------------------------------------------------------*/
/*  CDC definitions                                                    */
/*---------------------------------------------------------------------*/

/**************************************************/
/* CDC Requests                                   */
/**************************************************/
#define SEND_ENCAPSULATED_COMMAND               0x00
#define GET_ENCAPSULATED_RESPONSE               0x01
#define SET_COMM_FEATURE                        0x02
#define GET_COMM_FEATURE                        0x03
#define CLEAR_COMM_FEATURE                      0x04
#define SET_LINE_CODING                         0x20
#define GET_LINE_CODING                         0x21
#define SET_CONTROL_LINE_STATE                  0x22
#define SEND_BREAK                              0x23
#define NO_CMD                                  0xFF

/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
typedef struct _CDC_IF_PROP
{
  uint16_t (*pIf_Init)     (void);
  uint16_t (*pIf_DeInit)   (void);
  uint16_t (*pIf_Ctrl)     (uint32_t Cmd, uint8_t* Buf, uint32_t Len);
  uint16_t (*pIf_DataTx)   (uint8_t* Buf, uint32_t Len);
  uint16_t (*pIf_DataRx)   (uint8_t* Buf, uint32_t Len);
}
CDC_IF_Prop_TypeDef;

/* Exported typef ------------------------------------------------------------*/
/* The following structures groups all needed parameters to be configured for the
   ComPort. These parameters can modified on the fly by the host through CDC class
   command class requests. */
typedef struct
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
}LINE_CODING;

/* Exported constants --------------------------------------------------------*/

#define DEFAULT_CONFIG                  0
#define OTHER_CONFIG                    1

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
static uint16_t VCP_Init     (void);
static uint16_t VCP_DeInit   (void);
static uint16_t VCP_Ctrl     (uint32_t Cmd, uint8_t* Buf, uint32_t Len);
static uint16_t VCP_DataTx   (uint8_t* Buf, uint32_t Len);
static uint16_t VCP_DataRx   (uint8_t* Buf, uint32_t Len);
void VCP_send_buffer(uint8_t* buf, int len);
void VCP_send_str(uint8_t* buf);
void VCP_put_char(uint8_t buf);
int VCP_get_char(uint8_t *buf);
int VCP_get_string(uint8_t *buf);
static void Handle_USBAsynchXfer (void *pdev);
#endif  // __USB_AUDIO_CORE_H_
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
