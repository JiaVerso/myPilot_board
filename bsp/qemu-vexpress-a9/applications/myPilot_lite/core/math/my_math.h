/*
 * File      : my_math.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2026, RoboMaster-C-Board-INS-Example
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-09-16     JiaVerso      the first version
 * 2026-10-22     JiaVerso      compile and commit.
 */

#ifndef __MY_MATH_H__
#define __MY_MATH_H__

#include <stdbool.h>
#include <stdint.h>

#include "<arm_math.h>"


typedef struct{
	float x;
	float y;
}Vector2f_t;

typedef struct{
	float x;
	float y;
	float z;
}Vector3f_t;

typedef arm_matrix_instance_f32 math_matrixf_t;
typedef arm_status math_status_t;

#define MY_MATH_OK ARM_MATH_SUCCESS
#define MY_MATH_ARGUMENT_ERROR ARM_MATH_ARGUMENT_ERROR
#define MY_MATH_SIZE_MISMATCH ARM_MATH_SIZE_MISMATCH
#define MY_MATH_SINGULAR ARM_MATH_SINGULAR

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

/* The EKF depends on this interface, not directly on CMSIS-DSP. */
math_status_t my_matrix_init(math_matrixf_t *matrix,
                                  uint16_t rows,
                                  uint16_t columns,
                                  float *data);
math_status_t my_matrix_dimensions(math_matrixf_t *matrix,
                                           uint16_t rows,
                                           uint16_t columns);
 /* 矩阵加法 */
math_status_t my_matrix_add(const math_matrixf_t *left,
                                 const math_matrixf_t *right,
                                 math_matrixf_t *result);
 /* 矩阵减法 */
math_status_t my_matrix_subtract(const math_matrixf_t *left,
                                      const math_matrixf_t *right,
                                      math_matrixf_t *result);
 /* 矩阵乘法 */
math_status_t my_matrix_multiply(const math_matrixf_t *left,
                                      const math_matrixf_t *right,
                                      math_matrixf_t *result);
 /* 矩阵转置 */
math_status_t my_matrix_transpose(const math_matrixf_t *source,
                                       math_matrixf_t *result);
 /* 矩阵求逆 */                                     
math_status_t my_matrix_inverse(const math_matrixf_t *source,
                                     math_matrixf_t *result);

#endif