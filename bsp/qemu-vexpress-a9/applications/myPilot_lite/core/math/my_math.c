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

#include <string.h>
#include <math.h>
#include "my_math.h"

// 快速计算“平方根的倒数”。
float fast_invSqrt(float number)
{
    float x = number;

	// 第一步：将 x 当作整数，进行移位和减法操作，变成近似的结果
	uint32_t i;
	memcpy(&i, &x, sizeof i);
	i = 0x5f3759dfu - (i >> 1);
	memcpy(&x, &i, sizeof x);

	// 第二步：对 x 进行牛顿迭代，使之更接近结果
	const float xhalf = number * 0.5f;
	x = x * (1.5f - (xhalf * x * x));

	return x;
}

// 计算CRC-16-CCITT。
uint16_t math_crc16(uint16_t crc,const void * data,uint16_t len)
{
    // 半字节查表
    const static uint16_t crc_tab[16] =
    {
        0x0000 , 0x1021 , 0x2042 , 0x3063 , 0x4084 , 0x50A5 , 0x60C6 , 0x70E7 ,
        0x8108 , 0x9129 , 0xA14A , 0xB16B , 0xC18C , 0xD1AD , 0xE1CE , 0xF1EF
    };
    uint8_t h_crc;
    const uint8_t * ptr = (const uint8_t *)data;
    //
    while(len --)
    {
        h_crc = (uint8_t)(crc >> 12);
        crc <<= 4;
        crc ^= crc_tab[h_crc ^ ((*ptr) >> 4)];
        //
        h_crc = crc >> 12;
        crc <<= 4;
        crc ^= crc_tab[h_crc ^ ((*ptr) & 0x0F)];
        //
        ptr ++;
    }
    //
    return crc;
}

// 整数转字符串。
void math_itoa(int32_t val,char * str)
{
    char buf[16];
    buf[15] = '\0';
    uint8_t index = 16;
    int nagative = 0;

    if(val == 0)
    {
        // 先考虑val=0的情况。
        str[0] = '0';
        str[1] = '\0';
        return;
    }
    // 考虑val<0的情况。
    if(val < 0)
    {
        val = -val;
        nagative = 1;
    }
    // 处理整数部分
    while(val > 0 && index != 0)
    {
        index--;
        buf[index] = val % 10 + '0';
        val /= 10;
    }

    // 复制结果。
    int i_str = 0;
    if(nagative)
        str[i_str++] = '-';
    for(int i_buf=index;i_buf<16;i_buf++)
        str[i_str++] = buf[i_buf];
    str[i_str++] = '\0';
    //
    return;
}

// 整数转字符串。
// 不可重入，内存空间由函数管理。
const char * math_afromi(int32_t val)
{
    // int32_t最多只有10位，16字节足够。
    static char buffer[16]; 
    //
    math_itoa(val,buffer);
    //
    return buffer;
}

void Vector3_Set(float vector[3], float x, float y, float z)
{
	vector[0] = x;
	vector[1] = y;
	vector[2] = z;
}

// 三维向量归一化
void Vector3_Normalize(float result[3], const float vector[3])
{
    
    float norm_sq = vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2];
    
    // 零向量
    if (norm_sq < 1.0e-12f)
    {
        result[0] = 0.0f;
        result[1] = 0.0f;
        result[2] = 0.0f;
        return;
    }

	float rsqrt = fast_invSqrt(norm_sq);
    result[0] = vector[0] * rsqrt;
    result[1] = vector[1] * rsqrt;
	result[2] = vector[2] * rsqrt;
}

// 
void Vector3_CrossProduct(float result[3], const float vector1[3], const float vector2[3])
{
	result[0] = vector1[1]*vector2[2] - vector1[2]*vector2[1];
	result[1] = vector1[2]*vector2[0] - vector1[0]*vector2[2];
	result[2] = vector1[0]*vector2[1] - vector1[1]*vector2[0];
}

float Vector3_DotProduct(const float vector1[3], const float vector2[3])
{
	return vector1[0]*vector2[0] + vector1[1]*vector2[1] + vector1[2]*vector2[2];
}

float Vector3_Length(const float vector[3])
{
	return sqrt(vector[0]*vector[0]+vector[1]*vector[1]+vector[2]*vector[2]);
}

// 二维向量归一化
void Vector2_Normalize(float result[2], float vector[2])
{
    float norm_sq = vector[0] * vector[0] + vector[1] * vector[1];

    // 零向量
    if (norm_sq < 1.0e-12f)
    {
        result[0] = 0.0f;
        result[1] = 0.0f;
        return;
    }

	float rsqrt = fast_invSqrt(norm_sq);
	result[0] = vector[0] * rsqrt;
    result[1] = vector[1] * rsqrt;
}

float Vector2_DotProduct(const float vector1[2], const float vector2[2])
{
	return vector1[0]*vector2[0] + vector1[1]*vector2[1];
}

// 限幅
uint8_t constrain(float *val, float min_val, float max_val)
{

    if (val == NULL)
    {
        return 3; // 空指针
    }

	if(*val > max_val){
		*val = max_val;
		return 1;
	}
	if(*val < min_val){
		*val = min_val;
		return 2;
	}
	
	return 0;
}

float constrain_float(float amt, float low, float high)
{
	if (!isfinite(amt))
    {
        return low;
    }
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

uint32_t constrain_uint32(uint32_t amt, uint32_t low, uint32_t high)
{
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
