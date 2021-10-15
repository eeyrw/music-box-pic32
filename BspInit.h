#ifndef __BSP_INIT_H__
#define __BSP_INIT_H__

#define BUFFER_LENGTH 64
extern short buffer_a[BUFFER_LENGTH];
extern short buffer_b[BUFFER_LENGTH];

extern volatile unsigned char bufferAFull;
extern volatile unsigned char bufferBFull;

extern void InitializeSystem(void);
extern void init_i2s1();
extern void i2s_init_DMA(void);
extern void delay_ms(unsigned int count);

#endif
