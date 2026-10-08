#ifndef MP_TIME_H
#define MP_TIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint64_t mp_time_now_us(void);
uint32_t mp_time_now_ms(void);

void mp_sleep_ms(uint32_t ms);
void mp_sleep_ticks(uint32_t ticks);

void mp_delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif