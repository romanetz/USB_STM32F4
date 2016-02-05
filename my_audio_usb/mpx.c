#include "stm32f4xx_dac.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4_discovery_audio_codec.h"
#include "main.h"
#include "mpx.h"
#include "arm_math.h"

#define RDS
#define Zljko
//#define romanetz
#define RDS_PI 0x7EB0
#ifdef RDS
#include "ps_demo.h"
#include "rds00_wav.h"
#include "rds01_wav.h"
#include "rds10_wav.h"
#include "rds11_wav.h"
#endif

extern __IO uint8_t UserButtonPressed;

//#define DAC_DHR12L1_ADDRESS    0x4000740C
//#define DAC_DHR12LD_ADDRESS    0x40007424
static volatile uint32_t phase38;
static volatile uint32_t phase19;
static volatile uint32_t phase57;
static volatile uint32_t phaseRDS;

static volatile uint32_t phaseFM;
float xx_L[3] __attribute__ ((aligned(4), section("RAM2")));
float xx_R[3] __attribute__ ((aligned(4), section("RAM2")));
float yy_L[3] __attribute__ ((aligned(4), section("RAM2")));
float yy_R[3] __attribute__ ((aligned(4), section("RAM2")));
extern __IO uint8_t UserButtonPressed;
float preemph_iir_coeefsA[3] __attribute__ ((aligned(4), section("RAM2")));;
float preemph_iir_coeefsB[3] __attribute__ ((aligned(4), section("RAM2")));;

#define LUTSIZE	1024
#define MPX_MODE
#define pi_1024 3217
arm_fir_interpolate_instance_f32 mpx_interp_L __attribute__ ((aligned(4), section("RAM2")));
arm_fir_interpolate_instance_f32 mpx_interp_R __attribute__ ((aligned(4), section("RAM2")));
arm_fir_interpolate_instance_f32 * interp_ptr_L=&mpx_interp_L;
arm_fir_interpolate_instance_f32 * interp_ptr_R=&mpx_interp_R;

arm_fir_instance_f32 fir_L __attribute__ ((aligned(4), section("RAM2")));
arm_fir_instance_f32 fir_R __attribute__ ((aligned(4), section("RAM2")));
arm_fir_instance_f32 * fir_ptr_L=&fir_L;
arm_fir_instance_f32 * fir_ptr_R=&fir_R;

float Lbuf[48] __attribute__ ((aligned(4), section("RAM2")));
float Lbuf2[48] __attribute__ ((aligned(4), section("RAM2")));
float L_interp_buf[384] __attribute__ ((aligned(4),section("RAM2")));
float Rbuf[48] __attribute__ ((aligned(4),section("RAM2")));
float Rbuf2[48] __attribute__ ((aligned(4),section("RAM2")));
float R_interp_buf[384] __attribute__ ((aligned(4),section("RAM2")));
float interp_coeffs_ram[interp_coeff_size] __attribute__ ((aligned(4),section("RAM2")));
float fir_coeffs_ram[interp_coeff_size] __attribute__ ((aligned(4),section("RAM2")));

float MPX_buf[768] __attribute__ ((aligned(4)));		// bytes
uint16_t DAC_buf[768] __attribute__ ((aligned(4)));		//4608 bytes
s16 FB_buf[768] __attribute__ ((aligned(4)));		//1536 bytes
uint32_t DDS_buf[768] __attribute__ ((aligned(4)));		//4608 bytes

#ifndef old_interp
float32_t interp_state_L[48+320/8-1] __attribute__ ((aligned(4),section("RAM2")));
float32_t interp_state_R[48+320/8-1] __attribute__ ((aligned(4),section("RAM2")));
#else
float32_t interp_state_L[48+192/8-1] __attribute__ ((aligned(4),section("RAM2")));
float32_t interp_state_R[48+192/8-1] __attribute__ ((aligned(4),section("RAM2")));
#endif
float32_t fir_state_L[48+100-1] __attribute__ ((aligned(4),section("RAM2")));
float32_t fir_state_R[48+100-1] __attribute__ ((aligned(4),section("RAM2")));
volatile uint8_t audio_buffer_fill;
volatile uint8_t FM_buffer_fill;

void MPX_Gen (uint32_t samplerate,uint8_t multiplier,s16 * inbuf, float * outbuf, uint16_t size);
void DAC_normalise(float * inbuf,uint16_t * outbuf,s16 * fbbuf, uint16_t size);
void FM_MPX(uint32_t f0code,float * inbuf,uint16_t size, uint32_t * outbuf);

//uint32_t RDS_data[4][4]=\
{0xF00002c8,0x094c0274,0xF0000164,0x2A520000, /* [0][0..3] */ \
0xF00002c8,0x09490090,0xF0000164,0x44530160, /* [1][0..3] */  \
0xF00002c8,0x094a025B,0xF0000164,0x383702A7, /* [2][0..3] */  \
0xF00002c8,0x094c0274,0xF0000164,0x2A520000}; /* [3][0..3] */

/////////////////////////////////////////////////////////////////////////////////////////
//RDS
//#define _USE_BASEBAND
#define SEND_RDS_DATA	   		0
#define SEND_RDS_CRC		    1

static volatile struct {
	uint8_t State;
	int32_t RDSbaseBand;
	uint32_t phase57;
	uint32_t phase1187_5;
	uint32_t oldphase1187_5;
	uint8_t numberOfSentWords;
	uint8_t numberOfBitsToSend;
	uint16_t CurrentBit;
	uint16_t PrevBit;
	uint16_t OutputBuffer[2];
} m_RDS;

static volatile uint32_t phase57, phase1187_5 = 0, oldphase1187_5 = 0;
float m_volRDS;

/////////////////////////////////////////////////////////////////////////////////////////


uint32_t RDS_data[16] __attribute__ ((aligned(4),section("RAM2")));

const uint32_t RDS_init[16]={
0xF1230218,
0x0C0001B1,
0xF12301B4,
0x5244028A,
0xF1230218,
0x0C010008,
0xF12301B4,
0x532003FB,
0xF1230218,
0x0C0202C3,
0xF12301B4,
0x74650093,
0xF1230218,
0x0C070027,
0xF12301B4,
0x73740081
};


// Generator matrix (10 bits)
uint16_t  G[] = { 0x1B9, 0x372, 0x35D, 0x303,
                                0x3BF, 0x2C7, 0x037, 0x06E,
                                0x0DC, 0x1B8, 0x370, 0x359,
                                0x30B, 0x3AF, 0x2E7, 0x077      };

//      Offset words (10 bits)

uint16_t  O[] = { 0x0FC,  // A : Block 0
                                0x198,  // B: Block 1
                                0x168,  // C : Block 2 (Groups version A)
                                0x350,  // C' : Block 2 (Groups version B)
                                0x1B4,  // D : Block 3
                                0x000 };// E

uint32_t  RdsBlock(uint16_t wData,int16_t iBlock)
{
int16_t i;
uint16_t wCrc;
uint32_t dwBlock;
wCrc = 0;
for (i = 15; i >=0; i--)
if (wData & (1 << i))
wCrc ^= G[i];
wCrc ^= O[iBlock];
dwBlock = ((uint32_t)wData) << 16 | (wCrc<<6);
return dwBlock;
}

void mpx_init(uint8_t L,uint32_t bsize)
{int o,i,j;
 int32_t tmp;

	for (o=0; o<interp_coeff_size; o++)
{interp_coeffs_ram[o]=interp_coeffs[o];};

	for (o=0; o<100; o++)
fir_coeffs_ram[o]=fir_coeffs[o];
	//(numTaps/L)+blockSize-1
	for (o=0; o<(48+interp_coeff_size/8-1-1); o++)
	{interp_state_L[o]=0;
	interp_state_R[o]=0;
	};

	for (o=0; o<147; o++)
	{fir_state_L[o]=0;
	fir_state_R[o]=0;};

	arm_fir_interpolate_init_f32(interp_ptr_L,
		 L,
		 interp_coeff_size,
		 (float32_t*) &interp_coeffs_ram[0],
		 (float32_t*) &interp_state_L[0],
		  bsize);
arm_fir_interpolate_init_f32(interp_ptr_R,
		 L,
		  interp_coeff_size,
		  (float32_t*) &interp_coeffs_ram[0],
		  (float32_t*) &interp_state_R[0],
		  bsize);

arm_fir_init_f32(
  fir_ptr_L,
  100,
  (float32_t*) &fir_coeffs_ram[0],
  (float32_t*) &fir_state_L[0],
  bsize);

arm_fir_init_f32(
  fir_ptr_R,
  100,
 (float32_t*) &fir_coeffs_ram[0],
 (float32_t*)  &fir_state_R[0],
  bsize);

preemph_iir_coeefsA[0]= 3.200000000000000;
preemph_iir_coeefsA[1]=-2.0139149;
preemph_iir_coeefsA[2]= 0;
preemph_iir_coeefsB[0]=1;
preemph_iir_coeefsB[1]=	0.21809103944149166;
preemph_iir_coeefsB[2]=	0;

for (j=0;j<16;j++)
	{tmp=RDS_init[j];
	RDS_data[j]=(tmp&0xffff0000)|((tmp&0x0000ffff)<<6);}

/////////////////////////////////////////////////////////////////////////////////////////
//RDS init
	m_RDS.State = SEND_RDS_CRC;
	m_RDS.RDSbaseBand = 0x00;
	m_RDS.phase1187_5 = 0;
	m_RDS.oldphase1187_5 = 0;
	m_RDS.numberOfSentWords = 0;
	m_RDS.numberOfBitsToSend = 0;
	m_RDS.CurrentBit = 0;
	m_RDS.PrevBit = 3;

#ifndef _USE_BASEBAND
	m_volRDS = 0.08; //0.02
#else
			m_volRDS = 1;
#endif

};

//MPX signal is described as:
//MPX(t)=0.5*(L(t)+R(t))+0.5*(L(t)-R(t))*sin(38000*2*pi*t)+0.08*sin(19000*2*pi*t)
//buf is a pointer to array of audio data

int32_t Volume_curr=99;

float a=0.08;
float b=0.45;
float c=1/32768.0;
float d=0.08;
/*
Numerator:
 1 A0
-0.88482666015625 A1
 0.29278564453125 A2
Denominator:
1 B0
0.08721923828125 B1
0.0989990234375 B2
*/
uint32_t dphase=19000*11184;
uint32_t dphase_11875 = 13281962;

#ifdef RDS
static volatile int32_t sin57khz,sinRDS;
static volatile uint8_t RDS_nClockPeriod=0;
static volatile uint16_t RDS_block_num,RDS_symbol_num;
static volatile uint32_t RDS_symbol_mask=0x80000000;
static volatile uint8_t RDS_symbol_prev,RDS_symbol;
static volatile int16_t RDS_signal;
#endif


#ifdef romanetz

void MPX_Gen (uint32_t samplerate,uint8_t multiplier,s16 * inbuf, float * outbuf, uint16_t size)
{int temp=0;
static volatile int32_t sin38khz,sin19khz,sinFM;

uint16_t cnt=0;
int32_t MPX=0;
float mpx_float,pilot_float,dsb_float,RDS_carrier,L1,R1;
float * L_buf=&L_interp_buf[0];
float * R_buf=&R_interp_buf[0];
uint8_t cnt2;
int32_t L,R;
int16_t snd[6];
//dphase=(uint32_t)((19000*2147483648.0*2)/(samplerate*multiplier));

while (cnt<size)
{
//demux
L=*(s16 *)inbuf++;
R=*(s16 *)inbuf++;
//preemphasis IIR filter
xx_L[2]=xx_L[1];
xx_L[1]=xx_L[0];
xx_L[0]=(float)L;
yy_L[2]=yy_L[1];
yy_L[1]=yy_L[0];
yy_L[0]=xx_L[0]*preemph_iir_coeefsA[0]+xx_L[1]*preemph_iir_coeefsA[1]-yy_L[1]*preemph_iir_coeefsB[1];//+xx_L[2]*preemph_iir_coeefsA[2]-yy_L[1]*preemph_iir_coeefsB[1]-yy_L[2]*preemph_iir_coeefsB[2];

xx_R[2]=xx_R[1];
xx_R[1]=xx_R[0];
xx_R[0]=(float)R;
yy_R[2]=yy_R[1];
yy_R[1]=yy_R[0];
yy_R[0]=xx_R[0]*preemph_iir_coeefsA[0]+xx_R[1]*preemph_iir_coeefsA[1]-yy_R[1]*preemph_iir_coeefsB[1];//+xx_R[2]*preemph_iir_coeefsA[2]-yy_R[1]*preemph_iir_coeefsB[1]-yy_R[2]*preemph_iir_coeefsB[2];
Lbuf[cnt]=yy_L[0];
Rbuf[cnt]=yy_R[0];
cnt++;
};

//15k filter @ Fs=48kHz
arm_fir_f32(fir_ptr_L,(float32_t *)&Lbuf[0],(float32_t *)&Lbuf2[0],size);
arm_fir_f32(fir_ptr_R,(float32_t *)&Rbuf[0],(float32_t *)&Rbuf2[0],size);

//interpolate input signals to 384 kHz sample rate
//interpolation polyphase filter
arm_fir_interpolate_f32(interp_ptr_L,(float32_t *)&Lbuf2[0],(float32_t *)&L_interp_buf[0],size);
arm_fir_interpolate_f32(interp_ptr_R,(float32_t *)&Rbuf2[0],(float32_t *)&R_interp_buf[0],size);

cnt=0;
while (cnt<size)
{
for (cnt2=0;cnt2<multiplier;cnt2++)
{
#ifdef integer_mpx
snd[0]=(s16)L_interp_buf[cnt*multiplier+cnt2];//L
snd[1]=(s16)R_interp_buf[cnt*multiplier+cnt2];//R
#endif

phase38+=dphase*2;//38 kHz subcarrier delta phase
phase19+=dphase;//19 kHz pilot-tone

#ifdef RDS
phase57+=dphase*3;//57 kHz subcarrier
phaseRDS+=dphase/4; //4750 Hz RDS symbol clock*4
phase1187_5+=dphase/16; //1187.5 Hz RDS symbol clock
#endif

//instant sinusoidal values
sin38khz=LUT[phase38>>18];
sin19khz=LUT[phase19>>18];

#ifdef RDS
sin57khz=LUT[phase57>>18]; //57k subcarrier for RDS
RDS_carrier=(float)sin57khz*c;
#endif

#ifndef integer_mpx
dsb_float=(float)sin38khz*c;//normalization 38 kHz NCO output to -1..1

#ifdef RDS
if (phaseRDS<=(dphase/4)) //new period of 4750 Hz has begun
	{
	switch (RDS_nClockPeriod)
	{
	case 0:
	{//RDS_symbol_prev=RDS_symbol;
	//RDS_symbol=((RDS_data[RDS_block_num*4+RDS_symbol_num]&RDS_symbol_mask)>0)^RDS_symbol_prev;
	//RDS_symbol|=(RDS_symbol_prev&1)<<1;
	//RDS_signal=(RDS_symbol==1)?32767:-32768;
	//only msb

	RDS_symbol = (RDS_data[RDS_block_num*4+RDS_symbol_num]&RDS_symbol_mask)>0;

		//diff encoding
	RDS_symbol = RDS_symbol ^ RDS_symbol_prev;

		if (RDS_symbol & 0x01) {
			RDS_symbol_prev = 3; //0b00000011
		} else {
			RDS_symbol_prev = 0; //0b00000011
		}

	}; break;
	/*case 1:
	{RDS_signal=(RDS_symbol==1)?32767:-32768;};break;
	case 2:{RDS_signal=(RDS_symbol==1)?-32768:32767; //inverted
			}; break;
	case 3:
	{RDS_signal=(RDS_symbol==1)?-32768:32767;}; break; */
	};
	RDS_nClockPeriod++;
	RDS_symbol_mask>>=1;
	if (!(RDS_symbol_mask&0xFFFFFFC)) RDS_symbol_num++;
	if (RDS_symbol_num>3)
		{RDS_block_num++;
		RDS_symbol_num=0;
		RDS_symbol_mask=0x80000000;
		};
	//0xFFFFFFC - 26
	if (RDS_block_num>3)
			{RDS_block_num=0;
			RDS_symbol_num=0;
			RDS_symbol_mask=0x80000000;
			};
	RDS_nClockPeriod&=3; //0, 1, 2, 3
	}
else
{//RDS_signal=0;
}
//pick right waveform
switch (RDS_symbol) {
case 0x0000:
	RDS_signal = RDS_00[phase1187_5 >> 18];
	break;
case 0x0001:
	RDS_signal = RDS_01[phase1187_5 >> 18];
	break;
case 0x0002:
	RDS_signal = RDS_10[phase1187_5 >> 18];
	break;
case 0x0003:
	RDS_signal = RDS_11[phase1187_5 >> 18];
	break;
}
//Filter_RDS(RDS_signal,RDS_filtered_signal,RDS_filterstate);
#endif

L1=*L_buf++;
R1=*R_buf++;
mpx_float=(float)sin19khz*a+((L1-R1)*dsb_float+(L1+R1))*b;

#ifdef RDS
mpx_float+=RDS_carrier*(float)RDS_signal*d; //DSB modulation of RDS signal
#endif
	    //mpx_float=(float)sin19khz*a+\
		(Lbuf[cnt]-Rbuf[cnt])*b*dsb_float+\
	    (Lbuf[cnt]+Rbuf[cnt])*b;
#endif

#ifdef integer_mpx
snd[4]=sin38khz;
snd[5]=-sin38khz;
MPX=0;
/*
 *  res = __smlald(val1,val2,val3);
 *  p1 = val1[15:0] x val2[15:0]
                                       p2 = val1[31:16] x val2[31:16]
                                       sum = p1 + p2 + val3[63:32][31:0]
                                       res[63:32] = sum[63:32]
                                       res[31:0] = sum[31:0]
                                     */
//DSB part
MPX=__SMLAD(*(uint32_t *)&snd[0],*(uint32_t *)&snd[4],MPX);
MPX/=16384;
//baseband part
MPX=__SMLAD(*(uint32_t *)&snd[0],0x00010001U,MPX);
//pilot-tone
MPX+=(sin19khz>>4);
#endif
#ifndef integer_mpx
*(outbuf++)=mpx_float;
#endif
#ifdef integer_mpx
MPX=(s16)mpx_float;
*(outbuf++)=MPX;
#endif
};
cnt++;
};
}

#endif

#ifdef Zljko
void MPX_Gen(uint32_t samplerate, uint8_t multiplier, s16 * inbuf, float * outbuf, uint16_t size) {

	static volatile int32_t sin38khz, sin19khz, sin57khz;

	uint16_t cnt = 0;
	float mpx_float, dsb_float;
	int32_t L, R;

//demux and interpolate input signals to 384 kHz sample rate
	while (cnt < size) {
		L = *(s16 *) inbuf++;
		R = *(s16 *) inbuf++;

		#ifdef old_interp
		xx_L[2] = xx_L[1];
		xx_L[1] = xx_L[0];
		xx_L[0] = (float) L;
		yy_L[2] = yy_L[1];
		yy_L[1] = yy_L[0];
		yy_L[0] = xx_L[0] * preemph_iir_coeefsA[0] + xx_L[1] * preemph_iir_coeefsA[1] - yy_L[1] * preemph_iir_coeefsB[1];
		//+xx_L[2]*preemph_iir_coeefsA[2]-yy_L[1]*preemph_iir_coeefsB[1]-yy_L[2]*preemph_iir_coeefsB[2];
		xx_R[2] = xx_R[1];
		xx_R[1] = xx_R[0];
		xx_R[0] = (float) R;
		yy_R[2] = yy_R[1];
		yy_R[1] = yy_R[0];
		yy_R[0] = xx_R[0] * preemph_iir_coeefsA[0] + xx_R[1] * preemph_iir_coeefsA[1] - yy_R[1] * preemph_iir_coeefsB[1];
		//+xx_R[2]*preemph_iir_coeefsA[2]-yy_R[1]*preemph_iir_coeefsB[1]-yy_R[2]*preemph_iir_coeefsB[2];

		Lbuf[cnt] = yy_L[0];
		Rbuf[cnt] = yy_R[0];
		#else
		Lbuf[cnt] = (float) L;
		Rbuf[cnt] = (float) R;
		#endif

		cnt++;
	};

//15k filter
#ifdef old_interp
	arm_fir_f32(fir_ptr_L, (float32_t *) &Lbuf[0], (float32_t *) &Lbuf2[0], size);
	arm_fir_f32(fir_ptr_R, (float32_t *) &Rbuf[0], (float32_t *) &Rbuf2[0], size);

//interpolation polyphase filter
	arm_fir_interpolate_f32(interp_ptr_L, (float32_t *) &Lbuf2[0], (float32_t *) &L_interp_buf[0], size);
	arm_fir_interpolate_f32(interp_ptr_R, (float32_t *) &Rbuf2[0], (float32_t *) &R_interp_buf[0], size);
#else
	//interpolation polyphase filter
	arm_fir_interpolate_f32(interp_ptr_L, (float32_t *) &Lbuf[0], (float32_t *) &L_interp_buf[0], size);
	arm_fir_interpolate_f32(interp_ptr_R, (float32_t *) &Rbuf[0], (float32_t *) &R_interp_buf[0], size);

#endif
	cnt = 0;

	while (cnt < (size * multiplier)) {
#ifdef RDS
/////////////////////////////////////////////////////////////////////////////////////////
//RDS
		phase1187_5 += dphase_11875;
		if (phase1187_5 < dphase_11875) { //overflow -> next bit

			if (m_RDS.numberOfBitsToSend == 0) {
				switch (m_RDS.State) {
				case SEND_RDS_DATA:
					m_RDS.State = SEND_RDS_CRC;
					m_RDS.OutputBuffer[SEND_RDS_CRC] = m_RDS.OutputBuffer[SEND_RDS_CRC] << 6;
					m_RDS.numberOfBitsToSend = 10;
					break;
				case SEND_RDS_CRC:
					if (m_RDS.numberOfSentWords == 32) {
						m_RDS.numberOfSentWords = 0;
					}
					m_RDS.OutputBuffer[SEND_RDS_DATA] = PSovi[m_RDS.numberOfSentWords];
					m_RDS.OutputBuffer[SEND_RDS_CRC] = PSovi[m_RDS.numberOfSentWords + 1];
					m_RDS.numberOfSentWords += 2;
					m_RDS.State = SEND_RDS_DATA;
					m_RDS.numberOfBitsToSend = 16;
					break;
				}
//				printf("broj poslatih reci: %d\n",m_RDS.numberOfSentWords);
//				printf("DATA: 0x%X\n",m_RDS.RDSblock[SEND_RDS_DATA]);
//				printf("CRC: 0x%X\n",m_RDS.RDSblock[SEND_RDS_CRC]);
//				printf("\n");
			}

			//only msb
			m_RDS.CurrentBit = m_RDS.OutputBuffer[m_RDS.State] & 0x8000;

			//this part can be simplified, here is in this shape because of easier comparation with my avr asm project...
			if (m_RDS.CurrentBit) {
				m_RDS.CurrentBit = 1; //0b00000001
			} else {
				m_RDS.CurrentBit = 0; //0b00000001
			}

			//diff encoding
			m_RDS.CurrentBit = m_RDS.CurrentBit ^ m_RDS.PrevBit;

			if (m_RDS.CurrentBit & 0x01) {
				m_RDS.PrevBit = 3; //0b00000011
			} else {
				m_RDS.PrevBit = 0; //0b00000011
			}

			//prepare for the next bit
			m_RDS.OutputBuffer[m_RDS.State] = m_RDS.OutputBuffer[m_RDS.State] << 1;
			m_RDS.numberOfBitsToSend--;

		}

		//pick right waveform
		switch (m_RDS.CurrentBit) {
		case 0x0000:
			m_RDS.RDSbaseBand = RDS_00[phase1187_5 >> 18];
			break;
		case 0x0001:
			m_RDS.RDSbaseBand = RDS_01[phase1187_5 >> 18];
			break;
		case 0x0002:
			m_RDS.RDSbaseBand = RDS_10[phase1187_5 >> 18];
			break;
		case 0x0003:
			m_RDS.RDSbaseBand = RDS_11[phase1187_5 >> 18];
			break;
		}
#ifdef _USE_BASEBAND //for using with RDSspy sw.
		m_RDS.RDSbaseBand = ABS(m_RDS.RDSbaseBand);
#endif
#endif
		// instantaneous values of the sine waves
		phase38 += dphase * 2;
		phase19 += dphase;
		sin38khz = LUT[phase38 >> 18];
		sin19khz = LUT[phase19 >> 18];

		//RDS (should be optimized a lot...)
#ifndef _USE_BASEBAND
		phase57 += dphase * 3;
		sin57khz = LUT[phase57 >> 18] ;
#else
		sin57khz = 1;
#endif
/////////////////////////////////////////////////////////////////////////////////////////

		dsb_float = sin38khz * c;
#ifdef RDS
		mpx_float = (float) sin19khz * a + (L_interp_buf[cnt] - R_interp_buf[cnt]) * b * dsb_float + (L_interp_buf[cnt] + R_interp_buf[cnt]) * b\
				+ (float) m_RDS.RDSbaseBand * (float) sin57khz * c*m_volRDS;
#else
		mpx_float = (float) sin19khz * a + (L_interp_buf[cnt] - R_interp_buf[cnt]) * b * dsb_float + (L_interp_buf[cnt] + R_interp_buf[cnt]) * b;

		#endif
		//no 8 x over
		/*
		 mpx_float = (float) sin19khz * a + (Lbuf2[cnt / multiplier] - Rbuf2[cnt / multiplier]) * b * dsb_float\
		+(Lbuf2[cnt / multiplier] + Rbuf2[cnt / multiplier]) * b + (float)RDSbaseBand * sin57khz * m_volRDS;
		 */
		*(outbuf++) = mpx_float;
		cnt++;
	};
}
#endif

float dev=447335/32768.0; //75000*2^32/(80E+06*9)

void FM_MPX(uint32_t f0code,float * inbuf,uint16_t size, uint32_t * outbuf)
{int32_t tmp;
uint16_t cnt=0;
while (cnt<size)
{tmp=(int32_t)(*(inbuf++)*dev);
	*(outbuf++)=__REV(tmp+f0code);
	cnt++;};
}

#define __SSAT(ARG1,ARG2) \
({                          \
  uint32_t __RES, __ARG1 = (ARG1); \
  __ASM ("ssat %0, %1, %2" : "=r" (__RES) :  "I" (ARG2), "r" (__ARG1) ); \
  __RES; \
 })

void DAC_normalise(float * inbuf,uint16_t * outbuf,s16 * fbbuf, uint16_t size)
{
uint16_t cnt;
static volatile int32_t FMsig;
float tmp2;
	int32_t tmp;
	int32_t percent=100;
	int32_t shift=0;
for (cnt=0;cnt<size;cnt++)
	{
		tmp2=(*inbuf++);
		tmp=(int32_t)tmp2*Volume_curr/percent+shift;
		*fbbuf++=__SSAT(tmp,16);
		//*(uint16_t*)buf++=*(uint16_t*)buf*Volume_curr/100+0x8000;
		*(uint16_t*)outbuf++=__USAT(tmp,16);
		}
	}

static volatile uint32_t phL,phR;
//size in pair of samples
void Sin_Gen (s16 * inbuf,uint32_t sampling_freq,uint32_t freq_left,uint32_t freq_right, uint16_t size)
{uint16_t cnt=0;
uint16_t tmpL,tmpR;
uint32_t dphaseL;
uint32_t dphaseR;

dphaseL=89478*freq_left;//(2147483648.0*freq_left*2/sampling_freq);
dphaseR=89478*freq_right;//(2147483648.0*freq_right*2/sampling_freq);
while (cnt<size)
{	phL+=dphaseL;
	phR+=dphaseR;
//eight LSBs of phase are used to interpolate sine values
*(inbuf++)=LUT[phL>>18]/6;
*(inbuf++)=LUT[phR>>18]/6;
cnt++;
};
}
