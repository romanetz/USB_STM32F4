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
uint32_t DDS_buf[768] __attribute__ ((aligned(4)));		//4608 bytes

float32_t interp_state_L[48+192/8-1] __attribute__ ((aligned(4),section("RAM2")));
float32_t interp_state_R[48+192/8-1] __attribute__ ((aligned(4),section("RAM2")));
float32_t fir_state_L[48+100-1] __attribute__ ((aligned(4),section("RAM2")));
float32_t fir_state_R[48+100-1] __attribute__ ((aligned(4),section("RAM2")));
volatile uint8_t audio_buffer_fill;
volatile uint8_t FM_buffer_fill;

void MPX_Gen (uint32_t samplerate,uint8_t multiplier,s16 * inbuf, float * outbuf, uint16_t size);
void DAC_normalise(float * inbuf,uint16_t * outbuf, uint16_t size);
void FM_MPX(uint32_t f0code,float * inbuf,uint16_t size, uint32_t * outbuf);



void mpx_init(uint8_t L,uint32_t bsize)
{int o;
	for (o=0; o<interp_coeff_size; o++)
{interp_coeffs_ram[o]=interp_coeffs[o];};
	for (o=0; o<100; o++)
fir_coeffs_ram[o]=fir_coeffs[o];
	for (o=0; o<70; o++)
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
};




//MPX signal is described as:
//MPX(t)=0.5*(L(t)+R(t))+0.5*(L(t)-R(t))*sin(38000*2*pi*t)+0.08*sin(19000*2*pi*t)
//buf is a pointer to array of audio data

int32_t Volume_curr=80;

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


void MPX_Gen (uint32_t samplerate,uint8_t multiplier,s16 * inbuf, float * outbuf, uint16_t size)
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

//15k filter
arm_fir_f32(fir_ptr_L,(float32_t *)&Lbuf[0],(float32_t *)&Lbuf2[0],size);
arm_fir_f32(fir_ptr_R,(float32_t *)&Rbuf[0],(float32_t *)&Rbuf2[0],size);

//preemphasis
/*
cnt=0;
while (cnt<size)
{
Lbuf[cnt]=yy_L[0];//
Rbuf[cnt]=yy_R[0];//
cnt++;
};
*/
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
int d=30;
void FM_MPX(uint32_t f0code,float * inbuf,uint16_t size, uint32_t * outbuf)
{int32_t tmp;
uint16_t cnt=0;
while (cnt<size)
{tmp=*(inbuf++);
	*(outbuf++)=__REV(tmp*d+f0code);
	cnt++;};
}

#define __SSAT(ARG1,ARG2) \
({                          \
  uint32_t __RES, __ARG1 = (ARG1); \
  __ASM ("ssat %0, %1, %2" : "=r" (__RES) :  "I" (ARG2), "r" (__ARG1) ); \
  __RES; \
 })

void DAC_normalise(float * inbuf,uint16_t * outbuf, uint16_t size)
{
uint16_t cnt;
float tmp2;
	int32_t tmp;
	int32_t percent=100;
	int32_t shift=32768;
for (cnt=0;cnt<size;cnt++)
	{
		tmp2=(*inbuf++);
		tmp=(int32_t)tmp2*Volume_curr/percent+shift;
		//*(uint16_t*)buf++=*(uint16_t*)buf*Volume_curr/100+0x8000;
		//*(uint16_t*)outbuf++=__USAT(tmp,16);
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
