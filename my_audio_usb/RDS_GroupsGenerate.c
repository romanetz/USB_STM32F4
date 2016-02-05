//    RDS groups generating function of RDS software 
//
// =====================================================
//  Description
//    This file include initialisation and management functions
//       of RDS groups generating 
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
// 22/05/07    V1.0
//       creation 
//       derivative from PC utilities in C++ , files :
//             GenRDS.cpp   09/2000   author: g.laroche
//             BlocRDS.cpp  08/2000   author: g.laroche
//             GroupRDS.cpp 08/2000   author: g.laroche
// 11/06/07    V1.1
//       comment unused functionnality to limit binary size for ATmega8 version compatibility 
//=======================================

#include "main.h"
#include "RDS_GroupsGenerate.h"
#include "RDS.h"

/* local constant */

/* static variable  */
static unsigned char RDS_BufferGroup1[RDS_SIZE_FRAME]; /* first buffer group  */
static unsigned char RDS_BufferGroup2[RDS_SIZE_FRAME]; /* second buffer group */
static T_E_RDS_BUFF_STATE RDS_Buf1State; /* buffer 1 state */
static T_E_RDS_BUFF_STATE RDS_Buf2State; /* buffer 2 state */

static unsigned char PSNameIndex; /* current Index for PS Name */

static unsigned char RTxtIndex; /* current Index for RadioText */

static unsigned char AFIndex; /* current Index for AF */

static uint16_t CurrentPICode; /* current PICode */
static uint16_t CurrentPICodeCRCBlock1; /* current CRC of PICode for bloc1 */

static unsigned char * ptCharToSend; /*  current index pointer of next char to send */

static unsigned char SelectGroup; /* selected group generate, used to select the next */

/* global data */
unsigned char CurrentPSName[RDS_SIZE_PSNAME + 1]; /* current PS Name */
unsigned char CurrentRTxt[RDS_SIZE_RADIOTEXT]; /* current RadioText ended by 0x0d '\r' , unused data should be set to \r for optimisation */
unsigned char CurrentTxtAB; /* current Text A/B , each change clear line text */
unsigned short CurrentTP; /* current TP */
unsigned short CurrentPTY; /* current PTY */
unsigned char CurrentTA; /* current TA */
unsigned char CurrentMS; /* current MS */
unsigned char CurrentDI; /* current DI */

const char EEP_get_PSName[] = "RDS test";
const char EEP_get_Radiotext[] = "Ovo je radio tekst proba. STM32F407&AD9951 software FM modulator";
const uint16_t EEP_get_PICode = 0xF123;

char EEP_get_TP = 1;
char EEP_get_PTY = 1;
char EEP_get_TA = 1;
char EEP_get_MS = 1;
char EEP_get_DI = 1;

/* externam data */
extern unsigned char RDSDumpBuf;

/*  */
void RDSGEN_Init(void) {
	unsigned char TmpIndex;
	/* initialise PSNAME */
	for (TmpIndex = 0; TmpIndex < RDS_SIZE_PSNAME; TmpIndex++) {
		CurrentPSName[TmpIndex] = EEP_get_PSName[TmpIndex];
	}
	/* initialise Radiotext */
	for (TmpIndex = 0; TmpIndex < RDS_SIZE_RADIOTEXT; TmpIndex++) {
		CurrentRTxt[TmpIndex] = EEP_get_Radiotext[TmpIndex];
	}

	/* initialise CurrentPICode for optimisation time process */
	CurrentPICode = EEP_get_PICode;
	/* initialise CurrentPICodeCRCBlock1 for optimisation process */
	CurrentPICodeCRCBlock1 = RDSGEN_ComputeCRC(CurrentPICode, RDS_OFFSET_A);
	/* initialise CurrentDataBloc2 for optimisation time */
	/* TP as b10 */
	/* PTY as b5 to b9 */
	CurrentTP = EEP_get_TP << 10;
	CurrentPTY = EEP_get_PTY << 5;
	CurrentTA = EEP_get_TA; /* it's not necessary to shift TA, because testing with hardware input */
	CurrentMS = EEP_get_MS << 3;
	CurrentDI = EEP_get_DI;

	RTxtIndex = 0;
	PSNameIndex = 0;
	ptCharToSend = RDS_BufferGroup1;
	AFIndex = 0;
	CurrentTxtAB = 0;

	/*  generate first and second buffer before start clock*/
	RDSGEN_Gen0A(RDS_BufferGroup1);
	RDS_Buf1State = E_RDS_BUF_STATE_IN_USED;

	RDSGEN_Gen0A(RDS_BufferGroup2);
	RDS_Buf2State = E_RDS_BUF_STATE_READY;

	SelectGroup = 0;

}

/* generating 0A group */
void RDSGEN_Gen0A(unsigned char BufGroup[RDS_SIZE_FRAME]) {
	unsigned char TempI;
	uint16_t TempData;
	uint16_t TempCRC;

	/* clear buffer */
	for (TempI = 0; TempI < RDS_SIZE_FRAME; TempI++) {
		BufGroup[TempI] = 0;
	}

	/* bloc1 */
	RDSGEN_SetBlock1(CurrentPICode, CurrentPICodeCRCBlock1, BufGroup);

	/* Bloc 2 */
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+                
// |                                   TempData                    |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+                
// |b15|b14|b13|b12|b11|b10|b9 |b8 |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |b3 |b2 |b1 |b0 |b0 |b0 |b4 |b3 |b2 |b1 |b0 |b0 |b0 |b2 |b1 |b0 |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |      Group type   |TP |   PTY             |TA |MS |DI | PSName|
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
	/* CurrentPTY , CurrentTP  and CurrentMS  previously shift left */

	TempData = CurrentPTY | (RDS_GROUP_TYPE_0A << 11) | CurrentTP;
	TempData |= CurrentMS;
	if ((CurrentTA != 0) || (EEP_get_TA != 0)) {
		//RDSIO_SetTALed(1);
		TempData |= 0x10;
	} else {
		//RDSIO_SetTALed(0);
	}
	TempData |= (PSNameIndex >> 1); /* PSNameIndex /2 */
	switch (PSNameIndex >> 1) {
	case 0:
		TempData |= (CurrentDI & 0x08) << 3; /* select b3 bit and shift left */
		break;
	case 1:
		TempData |= (CurrentDI & 0x04) << 3; /* select b2 bit and shift left */
		break;
	case 2:
		TempData |= (CurrentDI & 0x02) << 3; /* select b1 bit and shift left */
		break;
	case 3:
		TempData |= (CurrentDI & 0x01) << 3; /* select b0 bit and shift left */
		break;
	}
	TempCRC = RDSGEN_ComputeCRC(TempData, RDS_OFFSET_B);
	RDSGEN_SetBlock2(TempData, TempCRC, BufGroup);

	/* Bloc 3 */

	/* AF code will be implemented in following version */
	//if (EEP_get_NbAF() == 0) {
	if (0) {
		TempData = RDS_AF_NOAF << 8; /* NoAF code */
		TempData |= RDS_AF_FILLER_CODE; /* Filler code */
	}
//	} else {
//		/* if begin of AF list, start with number of AF , followed by first AF */
//		if (AFIndex == 0) {
//			TempData = EEP_get_NbAF() << 8;
//			TempData |= EEP_get_AF(AFIndex++);
//		} else {
//			/* next AF */
//			TempData = EEP_get_AF(AFIndex++) << 8;
//			/* check if end of list , for filler code */
//			if (AFIndex == EEP_get_NbAF()) {
//				TempData |= RDS_AF_FILLER_CODE; /* Filler code */
//			}
//		}
//		/* test if End of AF list for restart */
//		if (AFIndex == EEP_get_NbAF()) {
//			AFIndex = 0; /* restart AF liste */
//		}
//	}
	TempCRC = RDSGEN_ComputeCRC(TempData, RDS_OFFSET_C);
	RDSGEN_SetBlock3(TempData, TempCRC, BufGroup);

	/* bloc 4 */
	TempData = (CurrentPSName[PSNameIndex++]) << 8;
	TempData |= (CurrentPSName[PSNameIndex++]);
	if (PSNameIndex >= RDS_SIZE_PSNAME) {
		PSNameIndex = 0;
	}
	TempCRC = RDSGEN_ComputeCRC(TempData, RDS_OFFSET_D);
	RDSGEN_SetBlock4(TempData, TempCRC, BufGroup);
	/* debug */
//   RS_DebugPrintfStrPGM(PSTR("0A "));
}

/* generating 2A group */
void RDSGEN_Gen2A(unsigned char BufGroup[RDS_SIZE_FRAME]) {
	unsigned char TempI;
	uint16_t TempData;
	uint16_t TempCRC;

	/* clear buffer */
	for (TempI = 0; TempI < RDS_SIZE_FRAME; TempI++) {
		BufGroup[TempI] = 0;
	}

	/* bloc1 */
	RDSGEN_SetBlock1(CurrentPICode, CurrentPICodeCRCBlock1, BufGroup);

	/* Bloc 2 */
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+                
// |                                   TempData                    |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+                
// |b15|b14|b13|b12|b11|b10|b9 |b8 |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |b3 |b2 |b1 |b0 |b0 |b0 |b4 |b3 |b2 |b1 |b0 |b0 |b3 |b2 |b1 |b0 |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |   Group type  |B0 |TP |   PTY             |TAB|  TxtSeg       |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
	/* Every change of TextA/B clear line of reciever */
	/* Line should be ended by '\r'(0x0D), for optimisation unused char in memory should be set to '\r' */
	/* the new line char '\n' (0x0A) clear line of reciever */

	/* CurrentPTY , CurrentTP  and CurrentMS  previously shift left */

	TempData = CurrentPTY | (RDS_GROUP_TYPE_2A << 11) | CurrentTP;
	if (CurrentTxtAB != 0) {
		TempData |= 0x10;
	}
	TempData |= (RTxtIndex >> 2); /*  radio text index /4, 4 char for each group send */
	TempCRC = RDSGEN_ComputeCRC(TempData, RDS_OFFSET_B);
	RDSGEN_SetBlock2(TempData, TempCRC, BufGroup);

	/* block 3 */
	TempData = CurrentRTxt[RTxtIndex] << 8;
	RTxtIndex++;
	TempData |= CurrentRTxt[RTxtIndex];
	RTxtIndex++;
	TempCRC = RDSGEN_ComputeCRC(TempData, RDS_OFFSET_C);
	RDSGEN_SetBlock3(TempData, TempCRC, BufGroup);

	/* block 4 */
	TempData = CurrentRTxt[RTxtIndex] << 8;
	RTxtIndex++;
	TempData |= CurrentRTxt[RTxtIndex];
	/* test end of line or end of array */
	/* unused char previously set to \r, so juste check las char of current group */
	if ((RTxtIndex == (RDS_SIZE_RADIOTEXT - 1)) || (CurrentRTxt[RTxtIndex] == RDS_RTXT_ENDLINE)) {
		/* restart to begin line */
		RTxtIndex = 0;
	} else {
		/* select next char */
		RTxtIndex++;
	}
	TempCRC = RDSGEN_ComputeCRC(TempData, RDS_OFFSET_D);
	RDSGEN_SetBlock4(TempData, TempCRC, BufGroup);

}

/* This function read next char and switch to second buffer if necessary */
/* function called by Interrupt, buffer state flag must be in critical section before writing it */

unsigned char RDSGEN_ReadNextData(void) {
	unsigned char NextChar;

	/* read new char */
	NextChar = *ptCharToSend;
	/* inc ptCharToSend */
	ptCharToSend++;
	/* test if end of table */

	/* test buffer used */
	if (RDS_Buf1State == E_RDS_BUF_STATE_IN_USED) {
		/* test end of buffer */
		if (ptCharToSend == &RDS_BufferGroup1[RDS_SIZE_FRAME]) {
			/* if buffer2 is ready -> use it*/
			if (RDS_Buf2State == E_RDS_BUF_STATE_READY) {
				ptCharToSend = &RDS_BufferGroup2[0];
				RDS_Buf2State = E_RDS_BUF_STATE_IN_USED;
				RDS_Buf1State = E_RDS_BUF_STATE_FREE;
			} else {
				/* error no buffer ready !!!!!*/
				/* resend last buffer to wait new data available */
				ptCharToSend = &RDS_BufferGroup1[0];
				//RS_PrintfStrPGM(PSTR("ERR:0 BUF\r\n"));
			}
		}
	} else {
		/* Buffer2 is used */
		/* test end of buffer */
		if (ptCharToSend == &RDS_BufferGroup2[RDS_SIZE_FRAME]) {
			/* if buffer1 is ready -> use it*/
			if (RDS_Buf1State == E_RDS_BUF_STATE_READY) {
				ptCharToSend = &RDS_BufferGroup1[0];
				RDS_Buf1State = E_RDS_BUF_STATE_IN_USED;
				RDS_Buf2State = E_RDS_BUF_STATE_FREE;
			} else {
				/* error no buffer ready !!!!!*/
				/* resend last buffer to wait new data available */
				ptCharToSend = &RDS_BufferGroup2[0];
				//RS_PrintfStrPGM(PSTR("ERR:0 BUF\r\n"));
			}
		}
	}
	return (NextChar);
}

/* compute RDS CRC from 16b data and 10b offset g(x) = x10 + x8 + x7 + x5 + x4 + x3 + 1*/
uint16_t RDSGEN_ComputeCRC(uint16_t Data, uint16_t Offset) {
// Word data used is a 16bits size 
// check word is a 10bits size
//
//   g(x) = x10 + x8 + x7 + x5 + x4 + x3 + 1
// 
//   +---------------+--------+--------+------------+--------+-------------^
//   |               |        |        |            |        |             |
//   +->b1->b2->b3->XOR->b4->XOR->b5->XOR->b6->b7->XOR->b8->XOR->b9->b10->XOR  +->Output
//                                                                         ^   |  MSB first
//                                                                         |   |
//   --------------------------------------------------------------------->+---+
	/* local define */
#define RDS_SELECT_B0  0x0001
#define RDS_SELECT_B3  0x0008
#define RDS_SELECT_B4  0x0010
#define RDS_SELECT_B5  0x0020
#define RDS_SELECT_B7  0x0080
#define RDS_SELECT_B8  0x0100
#define RDS_SELECT_B10 0x0400
#define RDS_SELECT_B11 0x0800
#define RDS_SELECT_B15 0x8000
#define RDS_SELECT_10BITS 0x03FF
#define RDS_NBR_BCL 16
#define RDS_NBR_BCL_CTRL 10

	uint16_t ResultCRC = 0; /* current result , only first 10b used */
	unsigned char LoopIndex; /* index for loop */

	/* main loop  of CRC processing */
	for (LoopIndex = 0; LoopIndex < RDS_NBR_BCL; LoopIndex++) {
		/* logical shift left of Result CRC */
		ResultCRC = ResultCRC << 1;

		/*IF MSB of data = 1 */
		/* THEN */
		if ((Data & RDS_SELECT_B15) != 0) {
			/* Result = XOR with used bits b10 "g(x)=x10" */
			ResultCRC ^= RDS_SELECT_B10;
			/* ENDIF */
		}
		/* IF (b10 of result =1) */
		/* THEN */
		if ((ResultCRC & RDS_SELECT_B10) != 0) {
			/* Result = XOR with used bits in "x8+x7+x5+x4+x3+1" ---> (b8,b7,b5,b4,b3,b0) */
			ResultCRC ^= (RDS_SELECT_B8 | RDS_SELECT_B7 | RDS_SELECT_B5 | RDS_SELECT_B4 | RDS_SELECT_B3 | RDS_SELECT_B0);
			/* ENDIF */
		}
		/* logical shift left of input data */
		Data = Data << 1;
	} /* END loop */

	/* adding offset with XOR and reset unused bits */
	ResultCRC = (ResultCRC ^ Offset) & RDS_SELECT_10BITS;

	/*return result CRC */
	return (ResultCRC);
}

/* set data and checkword bock1 in buffer */
void RDSGEN_SetBlock1(uint16_t Data, uint16_t CheckWord, uint8_t Buffer[RDS_SIZE_FRAME]) {
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+                
// |         Buffer [0]            |            Buffer[1]          |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+                
// |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |b15|b14|b13|b12|b11|b10|b9 |b8 |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |                              Data    block1                   |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+

// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+                
// |         Buffer [2]            |            Buffer[3]          |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+                
// |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |b9 |b8 |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |                       |  
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |                Checkword   block1     |                       |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
	Buffer[0] = (uint8_t)(Data >> 8); /* higher byte of data */
	Buffer[1] = (uint8_t) Data; /* lower byte of data */
	Buffer[2] = (uint8_t)(CheckWord >> 2); /* higher byte of checkword */
	Buffer[3] |= (uint8_t)(CheckWord << 6); /* lower byte of checkword */

}

/* set data and checkword bock2 in buffer */
void RDSGEN_SetBlock2(uint16_t Data, uint16_t CheckWord, uint8_t Buffer[RDS_SIZE_FRAME]) {
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+                
// |         Buffer [3]            |            Buffer[4]          |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+                
// |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |       |b15|b14|b13|b12|b11|b10|b9 |b8 |b7 |b6 |b5 |b4 |b3 |b2 |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |       |                      Data   block2                    |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+

// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+                
// |         Buffer [5]            |            Buffer[6]          |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+                
// |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |b1 |b0 |b9 |b8 |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |               |                      |  
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |  Data |                Checkword  block2      |               |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+

	Buffer[3] |= (uint8_t)(Data >> 10);
	Buffer[4] = (uint8_t)(Data >> 2);
	Buffer[5] = (((uint8_t) Data) << 6) | ((uint8_t)(CheckWord >> 4));
	Buffer[6] |= ((uint8_t) CheckWord) << 4;

}

/* set data and checkword bock3 in buffer */
void RDSGEN_SetBlock3(uint16_t Data, uint16_t CheckWord, uint8_t Buffer[RDS_SIZE_FRAME]) {
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+                
// |         Buffer [6]            |            Buffer[7]          |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+                
// |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |               |b15|b14|b13|b12|b11|b10|b9 |b8 |b7 |b6 |b5 |b4 |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |               |              Data   block3                    |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+

// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+                
// |         Buffer [8]            |            Buffer[9]          |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+                
// |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |b3 |b2 |b1 |b0 |b9 |b8 |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |       |                      |  
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |  Data  block3 |        Checkword  block3              |       |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+

	Buffer[6] |= (uint8_t)(Data >> 12);
	Buffer[7] = (uint8_t)(Data >> 4);
	Buffer[8] = (((uint8_t) Data) << 4) | ((uint8_t)(CheckWord >> 6));
	Buffer[9] |= ((uint8_t) CheckWord) << 2;

}

/* set data and checkword bock4 in buffer */
void RDSGEN_SetBlock4(uint16_t Data, uint16_t CheckWord, uint8_t Buffer[RDS_SIZE_FRAME]) {
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+                
// |         Buffer [9]            |            Buffer[10]         |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+                
// |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |                       |b15|b14|b13|b12|b11|b10|b9 |b8 |b7 |b6 |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |                       |      Data   block4                    |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+

// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+                
// |         Buffer [11]           |            Buffer[12]         |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+                
// |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |b5 |b4 |b3 |b2 |b1 |b0 |b9 |b8 |b7 |b6 |b5 |b4 |b3 |b2 |b1 |b0 |                      |  
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |  Data  block4         |    Checkword  block4                  |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+

	Buffer[9] |= (uint8_t)(Data >> 14);
	Buffer[10] = (uint8_t)(Data >> 6);
	Buffer[11] = (((uint8_t) Data) << 2) | ((uint8_t)(CheckWord >> 8)); /* higher bit of checkword */
	Buffer[12] = (uint8_t) CheckWord; /* lower byte of checkword */

}

/* Check buffer free and fill it if OK */
void RDSGEN_CheckFreeBuf(void) {
	/* temporary buffer  pointer */
	unsigned char * ptBuffer = &RDS_BufferGroup1[0];
	unsigned char BufferAvailable; /* set to !=0 if buffer free */

	BufferAvailable = 0;

	if (RDS_Buf1State == E_RDS_BUF_STATE_FREE) {
		/* filling buffer */
		RDS_Buf1State = E_RDS_BUF_STATE_IN_PREPARATION;
		ptBuffer = &RDS_BufferGroup1[0];
		BufferAvailable = 1;
	}
	if (RDS_Buf2State == E_RDS_BUF_STATE_FREE) {
		/* filling buffer */
		RDS_Buf2State = E_RDS_BUF_STATE_IN_PREPARATION;
		ptBuffer = &RDS_BufferGroup2[0];
		BufferAvailable = 2;
	}

	if (BufferAvailable != 0) {
//       RDSIO_SetActLed(1);    /* for time measure debug */
		switch (RDSGEN_SelectNextGroup()) {
		case RDS_GROUP_TYPE_0A:
			RDSGEN_Gen0A(ptBuffer);
			break;
		case RDS_GROUP_TYPE_2A:
			RDSGEN_Gen2A(ptBuffer);
			break;
		}
		switch (BufferAvailable) {
		case 1: {
			RDS_Buf1State = E_RDS_BUF_STATE_READY;
//  comment following line for ATmega8 compatibility (binary size)    
//              
//                if (RDSDumpBuf!=0)
//                {
//                 /* Debug function for polling RS transmit, to avoid dead lock on full RS buffer */
// 	             RS_DebugPrintfStrPGM(PSTR("BufferGroup1= "));
//                 for(BufferAvailable=0;BufferAvailable!=RDS_SIZE_FRAME;BufferAvailable++)
//                 { 
//                       RS_DebugPrintfUi(RDS_BufferGroup1[BufferAvailable],1);
//  	                   RS_DebugPrintfStrPGM(PSTR(" "));
//                 }
//  	             RS_DebugPrintfStrPGM(PSTR("\r\n"));
//                }               
//               
			break;
		}
		case 2: {
			RDS_Buf2State = E_RDS_BUF_STATE_READY;
//  comment following line for ATmega8 compatibility (binary size)    
//                if (RDSDumpBuf!=0)
//                {
//                    /* Debug function for polling RS transmit, to avoid dead lock on full RS buffer */
//    	            RS_DebugPrintfStrPGM(PSTR("BufferGroup2= "));
//                    for(BufferAvailable=0;BufferAvailable!=RDS_SIZE_FRAME;BufferAvailable++)
//                    {
//                       RS_DebugPrintfUi(RDS_BufferGroup2[BufferAvailable],1);
//  	                   RS_DebugPrintfStrPGM(PSTR(" "));
//                    }
//	                RS_DebugPrintfStrPGM(PSTR("\r\n"));
//
//                 }

			break;
		}
		}
//       RDSIO_SetActLed(0);    /* for time measure debug */
		//RDSIO_ToggleActLed(); /* toggle Activity LED on each group generate ---> ~6Hz */
	} /* endif test on buffer available */

}

/* return next group type to generate */
unsigned char RDSGEN_SelectNextGroup(void) {
	/* for first version only group 0A and 2A could be sent, 0A is always enable */
	/* a more complex algo will be implemented in next version */
	unsigned char TmpGroupRate;

	TmpGroupRate = 8; //EEP_get_GroupRate((RDS_GROUP_TYPE_2A & RDS_GROUP_TYPE_SELECT), (RDS_GROUP_TYPE_2A >> RDS_GROUP_TYPE_SHIFT));

	if (TmpGroupRate == 0) {
		/* 2A group disable , so return GROUP0A */
		return (RDS_GROUP_TYPE_0A);
	} else {
		/* 2A group enable */
		/* incrementing SelectGroup counter */
		SelectGroup++;
		if (SelectGroup > TmpGroupRate) {
			SelectGroup = 0;
			return (RDS_GROUP_TYPE_2A);
		} else {
			return (RDS_GROUP_TYPE_0A);
		}
	}
}

void DoPS_String(void) {
	static uint8_t tmpPsIndex = 0;

	switch (tmpPsIndex % 4) {
	case 0:
		sprintf(CurrentPSName, "RDS test");
		break;
	case 1:
		sprintf(CurrentPSName, "RDS DEMO");
		break;
	case 2:
		sprintf(CurrentPSName, "DDS GEN ");
		break;
	case 3:
		sprintf(CurrentPSName, "--TEST--");
		break;
	}
	tmpPsIndex++;
}
