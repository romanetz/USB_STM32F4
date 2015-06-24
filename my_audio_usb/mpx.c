#include "stm32f4xx_dac.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4_discovery_audio_codec.h"
#include "main.h"
#include "mpx.h"
#include "arm_math.h"
extern __IO uint8_t UserButtonPressed;
//Define output device
#define OUTPUT_INTERNAL_DAC
//#define DAC_DHR12L1_ADDRESS    0x4000740C
//#define DAC_DHR12LD_ADDRESS    0x40007424
static volatile uint32_t phase;
static volatile uint32_t phase19;
static volatile uint32_t phaseFM;
#define LUTSIZE	1024
#define MPX_MODE
#define pi_1024 3217
arm_fir_interpolate_instance_f32 mpx_interp_L __attribute__ ((aligned(8)));
arm_fir_interpolate_instance_f32 mpx_interp_R __attribute__ ((aligned(8)));
arm_fir_interpolate_instance_f32 * interp_ptr_L=&mpx_interp_L;
arm_fir_interpolate_instance_f32 * interp_ptr_R=&mpx_interp_R;
#define interp_coeff_size 192
float Lbuf[48] __attribute__ ((aligned(4), section("RAM2")));
float L_interp_buf[384] __attribute__ ((aligned(4),section("RAM2")));
float Rbuf[48] __attribute__ ((aligned(4),section("RAM2")));
float R_interp_buf[384] __attribute__ ((aligned(4),section("RAM2")));
float interp_coeffs_ram[interp_coeff_size] __attribute__ ((aligned(4),section("RAM2")));

volatile s16 MPX_buf[4608] __attribute__ ((aligned(4)));		//4608 bytes
volatile uint16_t DAC_buf[4608] __attribute__ ((aligned(4)));		//4608 bytes
volatile uint32_t DDS_buf[4608] __attribute__ ((aligned(4)));		//4608 bytes
float32_t interp_state_L[48+192/8-1] __attribute__ ((aligned(8)));
float32_t interp_state_R[48+192/8-1] __attribute__ ((aligned(8)));
volatile uint8_t audio_buffer_fill;
volatile uint8_t FM_buffer_fill;
void MPX_Gen (uint32_t samplerate,uint8_t multiplier,s16 * inbuf, s16 * outbuf, uint16_t size);

void DAC_normalise(s16 * inbuf,uint16_t * outbuf, uint16_t size);
void FM_MPX(uint32_t f0code,s16 * inbuf,uint16_t size, uint32_t * outbuf);

const float32_t interp_coeffs[interp_coeff_size]={
		0,
		-6.10352E-05,
		-6.10352E-05,
		-0.00012207,
		-0.000183105,
		-0.000244141,
		-0.000183105,
		-0.00012207,
		0,
		0.000183105,
		0.000427246,
		0.000610352,
		0.000793457,
		0.000793457,
		0.000732422,
		0.000427246,
		0,
		-0.000549316,
		-0.001159668,
		-0.001708984,
		-0.00201416,
		-0.002075195,
		-0.00177002,
		-0.001037598,
		0,
		0.001281738,
		0.002624512,
		0.00378418,
		0.004455566,
		0.004516602,
		0.003723145,
		0.002197266,
		0,
		-0.002624512,
		-0.005187988,
		-0.007385254,
		-0.008605957,
		-0.008605957,
		-0.007080078,
		-0.004150391,
		0,
		0.004760742,
		0.009460449,
		0.013244629,
		0.015319824,
		0.015136719,
		0.012390137,
		0.007141113,
		0,
		-0.008117676,
		-0.016052246,
		-0.022277832,
		-0.025695801,
		-0.02520752,
		-0.020507813,
		-0.011779785,
		0,
		0.013305664,
		0.026062012,
		0.036132813,
		0.041442871,
		0.040649414,
		0.032958984,
		0.018920898,
		0,
		-0.021240234,
		-0.041564941,
		-0.057617188,
		-0.066162109,
		-0.064819336,
		-0.05267334,
		-0.030273438,
		0,
		0.034240723,
		0.067321777,
		0.093811035,
		0.108398438,
		0.107116699,
		0.08782959,
		0.051025391,
		0,
		-0.059387207,
		-0.118896484,
		-0.169128418,
		-0.200317383,
		-0.203796387,
		-0.173034668,
		-0.104858398,
		0,
		0.136535645,
		0.295837402,
		0.465881348,
		0.632568359,
		0.781433105,
		0.898864746,
		0.974121094,
		1,
		0.974121094,
		0.898864746,
		0.781433105,
		0.632568359,
		0.465881348,
		0.295837402,
		0.136535645,
		0,
		-0.104858398,
		-0.173034668,
		-0.203796387,
		-0.200317383,
		-0.169128418,
		-0.118896484,
		-0.059387207,
		0,
		0.051025391,
		0.08782959,
		0.107116699,
		0.108398438,
		0.093811035,
		0.067321777,
		0.034240723,
		0,
		-0.030273438,
		-0.05267334,
		-0.064819336,
		-0.066162109,
		-0.057617188,
		-0.041564941,
		-0.021240234,
		0,
		0.018920898,
		0.032958984,
		0.040649414,
		0.041442871,
		0.036132813,
		0.026062012,
		0.013305664,
		0,
		-0.011779785,
		-0.020507813,
		-0.02520752,
		-0.025695801,
		-0.022277832,
		-0.016052246,
		-0.008117676,
		0,
		0.007141113,
		0.012390137,
		0.015136719,
		0.015319824,
		0.013244629,
		0.009460449,
		0.004760742,
		0,
		-0.004150391,
		-0.007080078,
		-0.008605957,
		-0.008605957,
		-0.007385254,
		-0.005187988,
		-0.002624512,
		0,
		0.002197266,
		0.003723145,
		0.004516602,
		0.004455566,
		0.00378418,
		0.002624512,
		0.001281738,
		0,
		-0.001037598,
		-0.00177002,
		-0.002075195,
		-0.00201416,
		-0.001708984,
		-0.001159668,
		-0.000549316,
		0,
		0.000427246,
		0.000732422,
		0.000793457,
		0.000793457,
		0.000610352,
		0.000427246,
		0.000183105,
		0,
		-0.00012207,
		-0.000183105,
		-0.000244141,
		-0.000183105,
		-0.00012207,
		-6.10352E-05,
		-6.10352E-05
};

void mpx_init(uint8_t L,uint32_t bsize)
{int o;
	for (o=0; o<interp_coeff_size; o++)
interp_coeffs_ram[o]=interp_coeffs[o];
	for (o=0; o<70; o++)
	{interp_state_L[o]=0;
	interp_state_R[o]=0;};
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
};


//MPX signal is described as:
//MPX(t)=0.5*(L(t)+R(t))+0.5*(L(t)-R(t))*sin(38000*2*pi*t)+0.08*sin(19000*2*pi*t)
//buf is a pointer to array of audio data

uint8_t Volume_curr=80;

float a=0.08;
float b=0.45;
float c=1/32768.0;
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
float xx_L[3] ,xx_R[3] ,yy_L[3] ,yy_R[3] ;
float preemph_iir_coeefsA[3] ={1,-0.88482666015625, 0.29278564453125};
float preemph_iir_coeefsB[3] ={1,0.08721923828125, 0.0989990234375};
void MPX_Gen (uint32_t samplerate,uint8_t multiplier,s16 * inbuf, s16 * outbuf, uint16_t size)
{int temp=0;
static volatile int32_t sin38khz,sin19khz,sinFM;
uint16_t cnt=0;
int32_t MPX=0;
float mpx_float,pilot_float,dsb_float;
uint8_t cnt2;
int32_t L,R;
int16_t snd[6];
uint32_t dphase;
uint32_t dphaseFM;
//dphase=(uint32_t)((19000*2147483648.0*2)/(samplerate*multiplier));
dphase=(uint32_t)(19000*11184);
//demux and interpolate input signals to 384 kHz sample rate
while (cnt<size)
{
L=*(s16 *)inbuf++;
R=*(s16 *)inbuf++;

xx_L[2]=xx_L[1];
xx_L[1]=xx_L[0];
xx_L[0]=(float)L;
yy_L[2]=yy_L[1];
yy_L[1]=yy_L[0];
yy_L[0]=xx_L[0]+xx_L[1]*preemph_iir_coeefsA[1]+xx_L[2]*preemph_iir_coeefsA[2]-yy_L[1]*preemph_iir_coeefsB[1]-yy_L[2]*preemph_iir_coeefsB[2];

xx_R[2]=xx_R[1];
xx_R[1]=xx_R[0];
xx_R[0]=(float)R;
yy_R[2]=yy_R[1];
yy_R[1]=yy_R[0];
yy_R[0]=xx_R[0]+xx_R[1]*preemph_iir_coeefsA[1]+xx_R[2]*preemph_iir_coeefsA[2]-yy_R[1]*preemph_iir_coeefsB[1]-yy_R[2]*preemph_iir_coeefsB[2];

Lbuf[cnt]=yy_L[0];
Rbuf[cnt]=yy_R[0];

cnt++;
};
arm_fir_interpolate_f32(interp_ptr_L,(float32_t *)&Lbuf[0],(float32_t *)&L_interp_buf[0],size);
arm_fir_interpolate_f32(interp_ptr_R,(float32_t *)&Rbuf[0],(float32_t *)&R_interp_buf[0],size);
cnt=0;
while (cnt<size)
{
for (cnt2=0;cnt2<multiplier;cnt2++)
{
#ifdef integer_mpx
snd[0]=(s16)L_interp_buf[cnt*multiplier+cnt2];//L
snd[1]=(s16)R_interp_buf[cnt*multiplier+cnt2];//R
#endif
phase+=dphase*2;
phase19+=dphase;
//мгновенные значения синусоид
sin38khz=LUT[phase>>18];
sin19khz=LUT[phase19>>18];
#ifndef integer_mpx
dsb_float=sin38khz*c;
mpx_float=(float)sin19khz*a+\
		(L_interp_buf[cnt*multiplier+cnt2]-R_interp_buf[cnt*multiplier+cnt2])*b*dsb_float+\
	    (L_interp_buf[cnt*multiplier+cnt2]+R_interp_buf[cnt*multiplier+cnt2])*b;
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
MPX=(s16)mpx_float;
#endif integer_mpx
*(outbuf++)=MPX;
};
cnt++;
};
}



void FM_MPX(uint32_t f0code,s16 * inbuf,uint16_t size, uint32_t * outbuf)
{int32_t tmp;
uint16_t cnt=0;
while (cnt<size)
{tmp=*(inbuf++);
	*(outbuf++)=__REV(tmp*30+f0code);
	cnt++;};
}

void DAC_normalise(s16 * inbuf,uint16_t * outbuf, uint16_t size)
{
uint16_t cnt;
	int32_t tmp;
for (cnt=0;cnt<size;cnt++)
	{
		tmp=(*inbuf)*Volume_curr/100;
		//*(uint16_t*)buf++=*(uint16_t*)buf*Volume_curr/100+0x8000;
		*(uint16_t*)outbuf=0x8000-*(uint16_t*)&tmp;
		inbuf++;
		outbuf++;
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
