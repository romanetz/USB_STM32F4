#ifndef RESAMPLE_H
#define RESAMPLE_H


#define  MIN(a, b)      (((a) < (b)) ? (a) : (b))

extern void InitResample(void);
extern void ResampleProccess(int32_t *srcL, int32_t *srcR, int32_t *dstL, int32_t *dstR) ;

#endif
