// RDS Groups generate header functions of RDS software 
//
// =====================================================
//  Description
//    This file include constantes, definitions and  prototypes functions
//    for RDS groups generating
//    
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
//=======================================
// Author : G.Laroche www.g.laroche.free.fr
//=======================================
// History:
// 29/05/07    V1.0
//       create 
//=======================================

#ifndef RDS_GROUPGEN_H
#define RDS_GROUPGEN_H

/* include application */
#include "RDS.h"

/* definition of constants  */

/* local type */
typedef enum {
	E_RDS_BUF_STATE_FREE, /* buffer free */
	E_RDS_BUF_STATE_IN_USED, /* buffer in used  by handler RDS clock*/
	E_RDS_BUF_STATE_IN_PREPARATION, /* buffer in preparation by RDS generate function */
	E_RDS_BUF_STATE_READY /* buffer ready to sending data */
} T_E_RDS_BUFF_STATE;

/* initialisation */
void RDSGEN_Init(void);
/* generating 0A group */
void RDSGEN_Gen0A(unsigned char BufGroup[RDS_SIZE_FRAME]);
/* generating 2A group */
void RDSGEN_Gen2A(unsigned char BufGroup[RDS_SIZE_FRAME]);
/*select next data and return it */
//unsigned char RDSGEN_ReadNextData(void);
/* compute RDS CRC from 16b data and 10b offset g(x) = x10 + x8 + x7 + x5 + x4 + x3 + 1*/
uint16_t RDSGEN_ComputeCRC(uint16_t Data, uint16_t Offset);

/* set data and checkword bock1 in buffer */
void RDSGEN_SetBlock1(uint16_t Data, uint16_t CheckWord, uint8_t Buffer[RDS_SIZE_FRAME]);
/* set data and checkword bock2 in buffer */
void RDSGEN_SetBlock2(uint16_t Data, uint16_t CheckWord, uint8_t Buffer[RDS_SIZE_FRAME]);
/* set data and checkword bock3 in buffer */
void RDSGEN_SetBlock3(uint16_t Data, uint16_t CheckWord, uint8_t Buffer[RDS_SIZE_FRAME]);
/* set data and checkword bock4 in buffer */
void RDSGEN_SetBlock4(uint16_t Data, uint16_t CheckWord, uint8_t Buffer[RDS_SIZE_FRAME]);

/* Check buffer free and fill it if OK */
void RDSGEN_CheckFreeBuf(void);
/* return next group type to generate */
unsigned char RDSGEN_SelectNextGroup(void);
extern void DoPS_String(void);

extern unsigned char RDSGEN_ReadNextData(void);
#endif
