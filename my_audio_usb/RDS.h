// Header file of RDS USB coder ATm32 software 
//
// =====================================================
//  Description
//    This file include common definitions for RDS USB software  
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
//    
//=======================================
// Author : G.Laroche www.g.laroche.free.fr
//=======================================
// History:
//  19/05/07    V 1.0
//       original version 
//=======================================

#ifndef RDS_H
#define RDS_H

// =====================================================
/* TEST version */
#ifndef RDS_TI_VERSION
#define RDS_TI_VERSION 1
#endif
// =====================================================

/* comment the following line if hardware biphase is used (not define RDS_SOFT_BIPHASE)*/
#define RDS_SOFT_BIPHASE 1 

/* common define */
#ifndef true
#define true 1
#endif

#ifndef false
#define false 0
#endif

/* Common RDS constant */
#define RDS_SIZE_FRAME       13          /* 13 bytes for a frame*/
#define RDS_SIZE_PSNAME      8          /* 8 bytes for PS NAME */
#define RDS_SIZE_RADIOTEXT   64          /* 64 bytes for Radio text group 2A */
#define RDS_SIZE_RADIOTEXT_B 32          /* 32 bytes for Radio text group 2B */
#define RDS_SIZE_GROUPS      16          /* 16 groups A and 16 groups B --> 32 groups total */
#define RDS_SIZE_TYPE_GROUPS 2           /* 2 groups type : A and B */
#define RDS_SIZE_AFTAB       25          /* 25 Alternat freq max for AF functionality */

#define RDS_OFFSET_A         0xFC         /* Offset A for checkword computing block 1   */
#define RDS_OFFSET_B         0x198        /* Offset B for checkword computing block 2   */
#define RDS_OFFSET_C         0x168        /* Offset C for checkword computing block 3   */
#define RDS_OFFSET_CPRIME    0x350        /* Offset C' for checkword computing block 3B */
#define RDS_OFFSET_D         0x1B4        /* Offset D for checkword computing block 4   */

#define RDS_GROUP_TYPE_0A    0x00         /* group type code  */
#define RDS_GROUP_TYPE_0B    0x01         /* group type code  */
#define RDS_GROUP_TYPE_1A    0x02         /* group type code  */
#define RDS_GROUP_TYPE_1B    0x03         /* group type code  */
#define RDS_GROUP_TYPE_2A    0x04         /* group type code  */
#define RDS_GROUP_TYPE_2B    0x05         /* group type code  */
#define RDS_GROUP_TYPE_3A    0x06         /* group type code  */
#define RDS_GROUP_TYPE_3B    0x07         /* group type code  */
#define RDS_GROUP_TYPE_4A    0x08         /* group type code  */
#define RDS_GROUP_TYPE_4B    0x09         /* group type code  */
#define RDS_GROUP_TYPE_5A    0x0A         /* group type code  */
#define RDS_GROUP_TYPE_5B    0x0B         /* group type code  */
#define RDS_GROUP_TYPE_6A    0x0B         /* group type code  */
#define RDS_GROUP_TYPE_6B    0x0D         /* group type code  */
#define RDS_GROUP_TYPE_7A    0x0E         /* group type code  */
#define RDS_GROUP_TYPE_7B    0x0F         /* group type code  */
#define RDS_GROUP_TYPE_8A    0x10         /* group type code  */
#define RDS_GROUP_TYPE_8B    0x11         /* group type code  */
#define RDS_GROUP_TYPE_9A    0x12         /* group type code  */
#define RDS_GROUP_TYPE_9B    0x13         /* group type code  */
#define RDS_GROUP_TYPE_10A   0x14         /* group type code  */
#define RDS_GROUP_TYPE_10B   0x15         /* group type code  */
#define RDS_GROUP_TYPE_11A   0x16         /* group type code  */
#define RDS_GROUP_TYPE_11B   0x17         /* group type code  */
#define RDS_GROUP_TYPE_12A   0x18         /* group type code  */
#define RDS_GROUP_TYPE_12B   0x19         /* group type code  */
#define RDS_GROUP_TYPE_13A   0x1A         /* group type code  */
#define RDS_GROUP_TYPE_13B   0x1B         /* group type code  */
#define RDS_GROUP_TYPE_14A   0x1C         /* group type code  */
#define RDS_GROUP_TYPE_14B   0x1D         /* group type code  */
#define RDS_GROUP_TYPE_15A   0x1E         /* group type code  */
#define RDS_GROUP_TYPE_15B   0x1F         /* group type code  */

#define RDS_GROUP_TYPE_SELECT 0x01        /* select 1st bit to determine type 0A/0B according b0 */
#define RDS_GROUP_TYPE_SHIFT  1           /* 1 right shift to determine group number from RDS_GROUP_TYPE_define */

#define RDS_AF_NOAF          224          /* code for no AF  */
#define RDS_AF_FILLER_CODE   205          /* filler code for AF  */
#define RDS_AF_FIRST_AF      1            /* code for Freq 87.6 MHz and step frequency by 100KHz */
#define RDS_AF_FIRST_AF_FRQ  875          /* first freq 87.6 MHz for AF in 100KHz lsb equal to 1 */
#define RDS_RTXT_ENDLINE     '\r'


#endif
