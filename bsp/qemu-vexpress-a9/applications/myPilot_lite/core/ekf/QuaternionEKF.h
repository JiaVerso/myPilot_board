#ifndef QUATERNION_EKF_INTEGRATED_H
#define QUATERNION_EKF_INTEGRATED_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "my_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The application may redirect these macros to a deterministic memory pool.
 * Kalman_Filter_Init() allocates its working matrices at runtime.
 */
#ifndef user_malloc
#define user_malloc malloc
#endif

#ifndef user_free
#define user_free free
#endif

#define QEKF_INSTANCE_MAGIC 0x51454B46UL

typedef struct kalman_filter_s
{
    float *FilteredValue;		/* 滤波输出的数组 */
    float *MeasuredVector;		/* 原始测量值 */
    float *ControlVector;

    uint8_t xhatSize;
    uint8_t uSize;
    uint8_t zSize;
    uint8_t Initialized;

    uint8_t UseAutoAdjustment;
    uint8_t MeasurementValidNum;

    uint8_t *MeasurementMap;
    float *MeasurementDegree;
    float *MatR_DiagonalElements;
    float *StateMinVariance;
    uint8_t *temp;

    /* Allow callbacks to recover their owning EKF instance. */
    void *UserData;

    uint8_t SkipEq1;
    uint8_t SkipEq2;
    uint8_t SkipEq3;
    uint8_t SkipEq4;
    uint8_t SkipEq5;

    math_matrixf_t xhat;
    math_matrixf_t xhatminus;
    math_matrixf_t u;
    math_matrixf_t z;
    math_matrixf_t P;
    math_matrixf_t Pminus;
    math_matrixf_t F;
    math_matrixf_t FT;
    math_matrixf_t B;
    math_matrixf_t H;
    math_matrixf_t HT;
    math_matrixf_t Q;
    math_matrixf_t R;
    math_matrixf_t K;
    math_matrixf_t S;
    math_matrixf_t temp_matrix;
    math_matrixf_t temp_matrix1;
    math_matrixf_t temp_vector;
    math_matrixf_t temp_vector1;

    int8_t MatStatus;

    void (*User_Func0_f)(struct kalman_filter_s *kf);
    void (*User_Func1_f)(struct kalman_filter_s *kf);
    void (*User_Func2_f)(struct kalman_filter_s *kf);
    void (*User_Func3_f)(struct kalman_filter_s *kf);
    void (*User_Func4_f)(struct kalman_filter_s *kf);
    void (*User_Func5_f)(struct kalman_filter_s *kf);
    tvoid (*User_Func6_f)(struct kalman_filter_s *kf);

    float *xhat_data;
    float *xhatminus_data;
    float *u_data;
    float *z_data;
    float *P_data;
    float *Pminus_data;
    float *F_data;
    float *FT_data;
    float *B_data;
    float *H_data;
    float *HT_data;
    float *Q_data;
    float *R_data;
    float *K_data;
    float *S_data;
    float *temp_matrix_data;
    float *temp_matrix_data1;
    float *temp_vector_data;
    float *temp_vector_data1;
} kalman_filter_t;

/*
 * Sensor callback contract:
 *   context        user-provided context
 *   sensor_index   gyro or accelerometer instance index
 *   data           output x/y/z in SI units
 *   return         0 on success, non-zero on failure
 */
typedef int8_t (*QEKF_SensorReadFn)(void *context,
                                    uint8_t sensor_index,
                                    float data[3]);

typedef struct QEKF_INS_s {
    uint32_t InstanceMagic;
    uint8_t Initialized;
    uint8_t InstanceIndex;

    struct kalman_filter_s IMU_QuaternionEKF;

    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint32_t ErrorCount;
    uint32_t UpdateCount;

    float q[4];
    float GyroBias[3];

    float Gyro[3];
    float Accel[3];
    float OrientationCosine[3];

    float accLPFcoef;
    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;

    float Q1;
    float Q2;
    float R;

    float dt;
    math_matrixf_t ChiSquare;
    float ChiSquare_Data[1];
    float ChiSquareTestThreshold;
    float lambda;

    int16_t YawRoundCount;
    float YawAngleLast;

    /* Per-instance diagnostic snapshots, avoiding shared global matrices. */
    float P_snapshot[36];
    float K_snapshot[18];
    float H_snapshot[18];

    /* Per-instance sensors and adapter callbacks. */
    uint8_t GyroInstance;
    uint8_t AccelInstance;
    QEKF_SensorReadFn ReadGyro;
    QEKF_SensorReadFn ReadAccel;
    void *SensorContext;
} QEKF_INS_t;

/* Original API, operating on QEKF_INS. */
void IMU_QuaternionEKF_Init(float process_noise1,
                            float process_noise2,
                            float measure_noise,
                            float lambda,
                            float lpf);

void IMU_QuaternionEKF_Update(float gx,
                              float gy,
                              float gz,
                              float ax,
                              float ay,
                              float az,
                              float dt);

/* Multi-instance API. */
bool IMU_QuaternionEKF_InitInstance(QEKF_INS_t *qe,
                                    uint8_t instance_index,
                                    float process_noise1,
                                    float process_noise2,
                                    float measure_noise,
                                    float lambda,
                                    float lpf);

void IMU_QuaternionEKF_DeinitInstance(QEKF_INS_t *qe);

bool IMU_QuaternionEKF_UpdateInstance(QEKF_INS_t *qe,
                                      float gx,
                                      float gy,
                                      float gz,
                                      float ax,
                                      float ay,
                                      float az,
                                      float dt);

/*
 * Associates an EKF instance with specific gyro and accelerometer indices.
 * ReadGyro/ReadAccel may be NULL when only explicit IMU updates are used.
 */
bool IMU_QuaternionEKF_SetSensorProvider(QEKF_INS_t *qe,
                                         uint8_t gyro_instance,
                                         uint8_t accel_instance,
                                         QEKF_SensorReadFn read_gyro,
                                         QEKF_SensorReadFn read_accel,
                                         void *sensor_context);

/*
 * Reads the configured gyro and accelerometer instances and performs one EKF
 * update. This is the multi-IMU integration entry point.
 */
bool IMU_QuaternionEKF_UpdateFromSensorsInstance(QEKF_INS_t *qe, float dt);

#ifdef __cplusplus
}
#endif

#endif
