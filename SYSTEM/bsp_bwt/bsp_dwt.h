#ifndef BSP_DWT_H
#define BSP_DWT_H

#include "main.h"

void DWT_reset_tick(void);
void DWT_init(void);
void DWT_delay_us(uint32_t us);
void DWT_delay_ms(uint32_t ms);
uint32_t DWT_get_time(void);
float DTW_Time_Difference_ms(void);


#endif

