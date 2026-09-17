/*
 * File      : my_math.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2026, RTT Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
* 2016-07-01     zoujiachi   	the first version
 * 2026-09-16     JiaVerso      the first version
 * 2026-09-16     JiaVerso      compile and commit.
 */

#ifndef __MY_MATH_H__
#define __MY_MATH_H__

#include <rtthread.h>
#include <rtdevice.h>

typedef struct{
	float x;
	float y;
}Vector2f_t;

typedef struct{
	float x;
	float y;
	float z;
}Vector3f_t;

float fast_invSqrt(float number);
uint16_t math_crc16(uint16_t crc,const void * data,uint16_t len);
void math_itoa(int32_t val,char * str);
const char * math_afromi(int32_t val);

void Vector3_Set(float vector[3], float x, float y, float z);
void Vector3_Normalize(float result[3], const float vector[3]);
void Vector3_CrossProduct(float result[3], const float vector1[3], const float vector2[3]);
float Vector3_DotProduct(const float vector1[3], const float vector2[3]);
float Vector3_Length(const float vector[3]);
void Vector2_Normalize(float result[2], float vector[2]);
float Vector2_DotProduct(const float vector1[2], const float vector2[2]);

uint8_t constrain(float *val, float min_val, float max_val);
float constrain_float(float amt, float low, float high);
uint32_t constrain_uint32(uint32_t amt, uint32_t low, uint32_t high);

#endif
