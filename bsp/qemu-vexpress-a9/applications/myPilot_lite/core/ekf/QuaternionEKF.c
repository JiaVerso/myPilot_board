#include "QuaternionEKF.h"

#include <math.h>
#include <string.h>

 /* 状态转移初始单位阵 */
static const float IMU_QuaternionEKF_F[36] = {
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
};

 /* 估计协方差矩阵 */
static const float IMU_QuaternionEKF_P[36] = {
    100000.0f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f,
    0.1f, 100000.0f, 0.1f, 0.1f, 0.1f, 0.1f,
    0.1f, 0.1f, 100000.0f, 0.1f, 0.1f, 0.1f,
    0.1f, 0.1f, 0.1f, 100000.0f, 0.1f, 0.1f,
    0.1f, 0.1f, 0.1f, 0.1f, 100.0f, 0.1f,
    0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 100.0f
};

static void Kalman_Filter_Deinit(kalman_filter_t *kf);
static void H_K_R_Adjustment(kalman_filter_t *kf);
static void IMU_QuaternionEKF_Observe(kalman_filter_t *kf);
static void IMU_QuaternionEKF_F_Linearization_P_Fading(kalman_filter_t *kf);
static void IMU_QuaternionEKF_SetH(kalman_filter_t *kf);
static void IMU_QuaternionEKF_xhatUpdate(kalman_filter_t *kf);
static void IMU_QuaternionEKF_P_PostProcess(kalman_filter_t *kf);

static QEKF_INS_t *qe_from_kf(KalmanFilter_t *kf)
{
    return (kf != NULL) ? (QEKF_INS_t *)kf->UserData : NULL;
}

 /* 初始化内存堆 */ 
static void *kf_alloc(size_t count, size_t element_size)
{
    const size_t bytes = count * element_size;
    void *memory = user_malloc(bytes);
    if (memory != NULL) {
        memset(memory, 0, bytes);
    }
    return memory;
}

static void qe_sanitize_covariance(kalman_filter_t *kf)
{
    if (kf == NULL || !kf->Initialized) {
        return;
    }

	/* 维护协方差矩阵的对称性( [P{ij} = P{ji} ) */
    for (uint8_t i = 0; i < kf->xhatSize; ++i) {
        for (uint8_t j = i + 1; j < kf->xhatSize; ++j) {
            const uint16_t ij = (uint16_t)(i * kf->xhatSize + j);
            const uint16_t ji = (uint16_t)(j * kf->xhatSize + i);
            const float average =
                0.5f * (kf->P_data[ij] + kf->P_data[ji]);
            kf->P_data[ij] = average;
            kf->P_data[ji] = average;
        }
    }

	/* 约束对角线方差 */
    for (uint8_t i = 0; i < kf->xhatSize; ++i) {
        const uint16_t diagonal = (uint16_t)(i * kf->xhatSize + i);
        float value = kf->P_data[diagonal];
        if (!isfinite(value)) {
            value = (i < 4) ? 100000.0f : 100.0f;
        }
        if (value < 1.0e-10f) {
            value = 1.0e-10f;
        }
        if (i < 4 && value > 1.0e6f) {
            value = 1.0e6f;
        } else if (i >= 4 && value > 10000.0f) {
            value = 10000.0f;
        }
        kf->P_data[diagonal] = value;
    }

	/* 约束协方差 */
    for (uint8_t i = 0; i < kf->xhatSize; ++i) {
        const uint16_t dii = (uint16_t)(i * kf->xhatSize + i);
        for (uint8_t j = i + 1; j < kf->xhatSize; ++j) {
            const uint16_t ij = (uint16_t)(i * kf->xhatSize + j);
            const uint16_t ji = (uint16_t)(j * kf->xhatSize + i);
            const uint16_t djj = (uint16_t)(j * kf->xhatSize + j);
            const float limit =
                0.999f * sqrtf(kf->P_data[dii] * kf->P_data[djj]);
            float value = kf->P_data[ij];
            if (!isfinite(value)) {
                value = 0.0f;
            }
            if (value > limit) {
                value = limit;
            } else if (value < -limit) {
                value = -limit;
            }
            kf->P_data[ij] = value;
            kf->P_data[ji] = value;
        }
    }
}

/*
 * Integrated generic linear Kalman framework.
 */
void Kalman_Filter_Init(kalman_filter_t *kf,
                        uint8_t xhatSize,
                        uint8_t uSize,
                        uint8_t zSize)
{
    if (kf == NULL || xhatSize == 0 || zSize == 0) {
        return;
    }

    memset(kf, 0, sizeof(*kf));
    kf->xhatSize = xhatSize;
    kf->uSize = uSize;
    kf->zSize = zSize;
    kf->MatStatus = MY_MATH_ARGUMENT_ERROR;

    kf->MeasurementMap = (uint8_t *)kf_alloc(zSize, sizeof(uint8_t));
    kf->MeasurementDegree = (float *)kf_alloc(zSize, sizeof(float));
    kf->MatR_DiagonalElements = (float *)kf_alloc(zSize, sizeof(float));
    kf->StateMinVariance = (float *)kf_alloc(xhatSize, sizeof(float));
    kf->temp = (uint8_t *)kf_alloc(zSize, sizeof(uint8_t));

    kf->FilteredValue = (float *)kf_alloc(xhatSize, sizeof(float));
    kf->MeasuredVector = (float *)kf_alloc(zSize, sizeof(float));
    kf->ControlVector = (uSize > 0) ?
                        (float *)kf_alloc(uSize, sizeof(float)) : NULL;

    kf->xhat_data = (float *)kf_alloc(xhatSize, sizeof(float));
    kf->xhatminus_data = (float *)kf_alloc(xhatSize, sizeof(float));
    kf->u_data = (uSize > 0) ?
                 (float *)kf_alloc(uSize, sizeof(float)) : NULL;
    kf->z_data = (float *)kf_alloc(zSize, sizeof(float));

    kf->P_data = (float *)kf_alloc(xhatSize * xhatSize, sizeof(float));
    kf->Pminus_data = (float *)kf_alloc(xhatSize * xhatSize, sizeof(float));
    kf->F_data = (float *)kf_alloc(xhatSize * xhatSize, sizeof(float));
    kf->FT_data = (float *)kf_alloc(xhatSize * xhatSize, sizeof(float));
    kf->B_data = (uSize > 0) ?
                 (float *)kf_alloc(xhatSize * uSize, sizeof(float)) : NULL;

    kf->H_data = (float *)kf_alloc(zSize * xhatSize, sizeof(float));
    kf->HT_data = (float *)kf_alloc(xhatSize * zSize, sizeof(float));
    kf->Q_data = (float *)kf_alloc(xhatSize * xhatSize, sizeof(float));
    kf->R_data = (float *)kf_alloc(zSize * zSize, sizeof(float));
    kf->K_data = (float *)kf_alloc(xhatSize * zSize, sizeof(float));

    kf->S_data = (float *)kf_alloc(zSize * zSize, sizeof(float));
    kf->temp_matrix_data =
        (float *)kf_alloc(xhatSize * xhatSize, sizeof(float));
    kf->temp_matrix_data1 =
        (float *)kf_alloc(xhatSize * xhatSize, sizeof(float));
    kf->temp_vector_data = (float *)kf_alloc(xhatSize, sizeof(float));
    kf->temp_vector_data1 = (float *)kf_alloc(xhatSize, sizeof(float));

    if (kf->MeasurementMap == NULL ||
        kf->MeasurementDegree == NULL ||
        kf->MatR_DiagonalElements == NULL ||
        kf->StateMinVariance == NULL ||
        kf->temp == NULL ||
        kf->FilteredValue == NULL ||
        kf->MeasuredVector == NULL ||
        (uSize > 0 && (kf->ControlVector == NULL || kf->u_data == NULL ||
                       kf->B_data == NULL)) ||
        kf->xhat_data == NULL ||
        kf->xhatminus_data == NULL ||
        kf->z_data == NULL ||
        kf->P_data == NULL ||
        kf->Pminus_data == NULL ||
        kf->F_data == NULL ||
        kf->FT_data == NULL ||
        kf->H_data == NULL ||
        kf->HT_data == NULL ||
        kf->Q_data == NULL ||
        kf->R_data == NULL ||
        kf->K_data == NULL ||
        kf->S_data == NULL ||
        kf->temp_matrix_data == NULL ||
        kf->temp_matrix_data1 == NULL ||
        kf->temp_vector_data == NULL ||
        kf->temp_vector_data1 == NULL) {
        Kalman_Filter_Deinit(kf);
        return;
    }

	/* 多态 */
    (void)Matrix_Init(&kf->xhat, xhatSize, 1, kf->xhat_data);
    (void)Matrix_Init(&kf->xhatminus, xhatSize, 1, kf->xhatminus_data);
    if (uSize > 0) {
        (void)Matrix_Init(&kf->u, uSize, 1, kf->u_data);
        (void)Matrix_Init(&kf->B, xhatSize, uSize, kf->B_data);
    }
    (void)Matrix_Init(&kf->z, zSize, 1, kf->z_data);
    (void)Matrix_Init(&kf->P, xhatSize, xhatSize, kf->P_data);
    (void)Matrix_Init(&kf->Pminus, xhatSize, xhatSize, kf->Pminus_data);
    (void)Matrix_Init(&kf->F, xhatSize, xhatSize, kf->F_data);
    (void)Matrix_Init(&kf->FT, xhatSize, xhatSize, kf->FT_data);
    (void)Matrix_Init(&kf->H, zSize, xhatSize, kf->H_data);
    (void)Matrix_Init(&kf->HT, xhatSize, zSize, kf->HT_data);
    (void)Matrix_Init(&kf->Q, xhatSize, xhatSize, kf->Q_data);
    (void)Matrix_Init(&kf->R, zSize, zSize, kf->R_data);
    (void)Matrix_Init(&kf->K, xhatSize, zSize, kf->K_data);
    (void)Matrix_Init(&kf->S, zSize, zSize, kf->S_data);
    (void)Matrix_Init(&kf->temp_matrix, xhatSize, xhatSize,
                      kf->temp_matrix_data);
    (void)Matrix_Init(&kf->temp_matrix1, xhatSize, xhatSize,
                      kf->temp_matrix_data1);
    (void)Matrix_Init(&kf->temp_vector, xhatSize, 1,
                      kf->temp_vector_data);
    (void)Matrix_Init(&kf->temp_vector1, xhatSize, 1,
                      kf->temp_vector_data1);

    kf->SkipEq1 = FALSE;
    kf->SkipEq2 = FALSE;
    kf->SkipEq3 = FALSE;
    kf->SkipEq4 = FALSE;
    kf->SkipEq5 = FALSE;
    kf->Initialized = TRUE;
    kf->MatStatus = MY_MATH_OK;
}

 /* 回收已分配内存 */
static void Kalman_Filter_Deinit(kalman_filter_t *kf)
{
    if (kf == NULL) {
        return;
    }

    user_free(kf->MeasurementMap);
    user_free(kf->MeasurementDegree);
    user_free(kf->MatR_DiagonalElements);
    user_free(kf->StateMinVariance);
    user_free(kf->temp);
    user_free(kf->FilteredValue);
    user_free(kf->MeasuredVector);
    user_free(kf->ControlVector);
    user_free(kf->xhat_data);
    user_free(kf->xhatminus_data);
    user_free(kf->u_data);
    user_free(kf->z_data);
    user_free(kf->P_data);
    user_free(kf->Pminus_data);
    user_free(kf->F_data);
    user_free(kf->FT_data);
    user_free(kf->B_data);
    user_free(kf->H_data);
    user_free(kf->HT_data);
    user_free(kf->Q_data);
    user_free(kf->R_data);
    user_free(kf->K_data);
    user_free(kf->S_data);
    user_free(kf->temp_matrix_data);
    user_free(kf->temp_matrix_data1);
    user_free(kf->temp_vector_data);
    user_free(kf->temp_vector_data1);

    memset(kf, 0, sizeof(*kf));
    kf->MatStatus = MY_MATH_ARGUMENT_ERROR;
}

void Kalman_Filter_Measure(kalman_filter_t *kf)
{
    if (kf == NULL || !kf->Initialized) {
        return;
    }

    if (kf->UseAutoAdjustment != 0) {
        H_K_R_Adjustment(kf);
    } else {
        memcpy(kf->z_data, kf->MeasuredVector,
               sizeof(float) * kf->zSize);
        memset(kf->MeasuredVector, 0, sizeof(float) * kf->zSize);
    }

    if (kf->uSize > 0) {
        memcpy(kf->u_data, kf->ControlVector,
               sizeof(float) * kf->uSize);
    }
}

void Kalman_Filter_xhatMinusUpdate(kalman_filter_t *kf)
{
    if (kf == NULL || !kf->Initialized || kf->SkipEq1) {
        return;
    }

    if (kf->uSize > 0) {
        kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat,
                                        &kf->temp_vector);
        kf->MatStatus = Matrix_Multiply(&kf->B, &kf->u,
                                        &kf->temp_vector1);
        kf->MatStatus = Matrix_Add(&kf->temp_vector, &kf->temp_vector1,
                                   &kf->xhatminus);
    } else {
        kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat,
                                        &kf->xhatminus);
    }
}

void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf)
{
    if (kf == NULL || !kf->Initialized || kf->SkipEq2) {
        return;
    }

    kf->MatStatus = Matrix_Transpose(&kf->F, &kf->FT);
    kf->MatStatus = Matrix_Multiply(&kf->F, &kf->P, &kf->Pminus);
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->FT,
                                    &kf->temp_matrix);
    kf->MatStatus = Matrix_Add(&kf->temp_matrix, &kf->Q, &kf->Pminus);
}

void Kalman_Filter_SetK(KalmanFilter_t *kf)
{
    if (kf == NULL || !kf->Initialized || kf->SkipEq3) {
        return;
    }

    (void)MyMath_Matrix_SetDimensions(&kf->temp_matrix,
                                      kf->H.numRows,
                                      kf->Pminus.numCols);
    (void)MyMath_Matrix_SetDimensions(&kf->temp_matrix1,
                                      kf->temp_matrix.numRows,
                                      kf->HT.numCols);

    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT);
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus,
                                    &kf->temp_matrix);
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT,
                                    &kf->temp_matrix1);
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S);
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);
    if (kf->MatStatus != MY_MATH_OK) {
        return;
    }
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT,
                                    &kf->temp_matrix);
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1,
                                    &kf->K);
}

void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf)
{
    if (kf == NULL || !kf->Initialized || kf->SkipEq4) {
        return;
    }

    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus,
                                    &kf->temp_vector);
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector,
                                    &kf->temp_vector1);
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1,
                                    &kf->temp_vector);
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector,
                               &kf->xhat);
}

void Kalman_Filter_P_Update(KalmanFilter_t *kf)
{
    if (kf == NULL || !kf->Initialized || kf->SkipEq5) {
        return;
    }

    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->H,
                                    &kf->temp_matrix);
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->Pminus,
                                    &kf->temp_matrix1);
    kf->MatStatus = Matrix_Subtract(&kf->Pminus, &kf->temp_matrix1,
                                    &kf->P);
}

float *Kalman_Filter_Update(KalmanFilter_t *kf)
{
    if (kf == NULL || !kf->Initialized) {
        return NULL;
    }

    Kalman_Filter_Measure(kf);
    if (kf->User_Func0_f != NULL) {
        kf->User_Func0_f(kf);
    }

    Kalman_Filter_xhatMinusUpdate(kf);
    if (kf->User_Func1_f != NULL) {
        kf->User_Func1_f(kf);
    }

    Kalman_Filter_PminusUpdate(kf);
    if (kf->User_Func2_f != NULL) {
        kf->User_Func2_f(kf);
    }

    if (kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0) {
        Kalman_Filter_SetK(kf);
        if (kf->User_Func3_f != NULL) {
            kf->User_Func3_f(kf);
        }

        Kalman_Filter_xhatUpdate(kf);
        if (kf->User_Func4_f != NULL) {
            kf->User_Func4_f(kf);
        }

        Kalman_Filter_P_Update(kf);
    } else {
        memcpy(kf->xhat_data, kf->xhatminus_data,
               sizeof(float) * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data,
               sizeof(float) * kf->xhatSize * kf->xhatSize);
    }

    if (kf->User_Func5_f != NULL) {
        kf->User_Func5_f(kf);
    }

    for (uint8_t i = 0; i < kf->xhatSize; ++i) {
        const uint16_t diagonal = (uint16_t)(i * kf->xhatSize + i);
        if (kf->P_data[diagonal] < kf->StateMinVariance[i]) {
            kf->P_data[diagonal] = kf->StateMinVariance[i];
        }
    }

    memcpy(kf->FilteredValue, kf->xhat_data,
           sizeof(float) * kf->xhatSize);

    if (kf->User_Func6_f != NULL) {
        kf->User_Func6_f(kf);
    }
    return kf->FilteredValue;
}

static void H_K_R_Adjustment(KalmanFilter_t *kf)
{
    if (kf == NULL || !kf->Initialized || kf->UseAutoAdjustment == 0) {
        return;
    }

    kf->MeasurementValidNum = 0;
    memcpy(kf->z_data, kf->MeasuredVector,
           sizeof(float) * kf->zSize);
    memset(kf->MeasuredVector, 0, sizeof(float) * kf->zSize);
    memset(kf->R_data, 0, sizeof(float) * kf->zSize * kf->zSize);
    memset(kf->H_data, 0, sizeof(float) * kf->zSize * kf->xhatSize);

    for (uint8_t i = 0; i < kf->zSize; ++i) {
        if (kf->z_data[i] != 0.0f) {
            kf->z_data[kf->MeasurementValidNum] = kf->z_data[i];
            kf->temp[kf->MeasurementValidNum] = i;
            kf->H_data[kf->xhatSize * kf->MeasurementValidNum +
                       kf->MeasurementMap[i] - 1] =
                kf->MeasurementDegree[i];
            ++kf->MeasurementValidNum;
        }
    }

    for (uint8_t i = 0; i < kf->MeasurementValidNum; ++i) {
        kf->R_data[i * kf->MeasurementValidNum + i] =
            kf->MatR_DiagonalElements[kf->temp[i]];
    }

    (void)MyMath_Matrix_SetDimensions(&kf->H,
                                      kf->MeasurementValidNum,
                                      kf->H.numCols);
    (void)MyMath_Matrix_SetDimensions(&kf->HT,
                                      kf->HT.numRows,
                                      kf->MeasurementValidNum);
    (void)MyMath_Matrix_SetDimensions(&kf->R,
                                      kf->MeasurementValidNum,
                                      kf->MeasurementValidNum);
    (void)MyMath_Matrix_SetDimensions(&kf->K,
                                      kf->K.numRows,
                                      kf->MeasurementValidNum);
    (void)MyMath_Matrix_SetDimensions(&kf->z,
                                      kf->MeasurementValidNum,
                                      kf->z.numCols);
}

static bool qe_input_finite(float gx, float gy, float gz,
                            float ax, float ay, float az)
{
    return isfinite(gx) && isfinite(gy) && isfinite(gz) &&
           isfinite(ax) && isfinite(ay) && isfinite(az);
}

bool IMU_QuaternionEKF_InitInstance(QEKF_INS_t *qe,
                                    uint8_t instance_index,
                                    float process_noise1,
                                    float process_noise2,
                                    float measure_noise,
                                    float lambda,
                                    float lpf)
{
    if (qe == NULL || process_noise1 <= 0.0f ||
        process_noise2 <= 0.0f || measure_noise <= 0.0f ||
        !isfinite(process_noise1) || !isfinite(process_noise2) ||
        !isfinite(measure_noise) || !isfinite(lambda) ||
        !isfinite(lpf) || lpf < 0.0f) {
        return false;
    }

    if (qe->InstanceMagic == QEKF_INSTANCE_MAGIC) {
        IMU_QuaternionEKF_DeinitInstance(qe);
    }
    memset(qe, 0, sizeof(*qe));

    qe->InstanceIndex = instance_index;
    qe->Q1 = process_noise1;
    qe->Q2 = process_noise2;
    qe->R = measure_noise;
    qe->ChiSquareTestThreshold = 1.0e-8f;
    qe->lambda = (lambda > 1.0f) ? 1.0f : lambda;
    if (qe->lambda <= 0.0f) {
        qe->lambda = 1.0f;
    }
    qe->accLPFcoef = lpf;

    Kalman_Filter_Init(&qe->IMU_QuaternionEKF, 6, 0, 3);
    if (!qe->IMU_QuaternionEKF.Initialized) {
        memset(qe, 0, sizeof(*qe));
        return false;
    }

    qe->IMU_QuaternionEKF.UserData = qe;
    Matrix_Init(&qe->ChiSquare, 1, 1, qe->ChiSquare_Data);

    qe->IMU_QuaternionEKF.xhat_data[0] = 1.0f;
    qe->IMU_QuaternionEKF.xhat_data[1] = 0.0f;
    qe->IMU_QuaternionEKF.xhat_data[2] = 0.0f;
    qe->IMU_QuaternionEKF.xhat_data[3] = 0.0f;

    qe->IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
    qe->IMU_QuaternionEKF.User_Func1_f =
        IMU_QuaternionEKF_F_Linearization_P_Fading;
    qe->IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
    qe->IMU_QuaternionEKF.User_Func3_f =
        IMU_QuaternionEKF_xhatUpdate;
    qe->IMU_QuaternionEKF.User_Func5_f =
        IMU_QuaternionEKF_P_PostProcess;

    qe->IMU_QuaternionEKF.SkipEq3 = TRUE;
    qe->IMU_QuaternionEKF.SkipEq4 = TRUE;

    memcpy(qe->IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F,
           sizeof(IMU_QuaternionEKF_F));
    memcpy(qe->IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P,
           sizeof(IMU_QuaternionEKF_P));

    qe->Initialized = TRUE;
    qe->InstanceMagic = QEKF_INSTANCE_MAGIC;
    return true;
}

void IMU_QuaternionEKF_Init(float process_noise1,
                            float process_noise2,
                            float measure_noise,
                            float lambda,
                            float lpf)
{
    (void)IMU_QuaternionEKF_InitInstance(&QEKF_INS, 0,
                                        process_noise1,
                                        process_noise2,
                                        measure_noise,
                                        lambda,
                                        lpf);
}

void IMU_QuaternionEKF_DeinitInstance(QEKF_INS_t *qe)
{
    if (qe == NULL || qe->InstanceMagic != QEKF_INSTANCE_MAGIC) {
        return;
    }

    Kalman_Filter_Deinit(&qe->IMU_QuaternionEKF);
    memset(qe, 0, sizeof(*qe));
}

bool IMU_QuaternionEKF_SetSensorProvider(QEKF_INS_t *qe,
                                         uint8_t gyro_instance,
                                         uint8_t accel_instance,
                                         QEKF_SensorReadFn read_gyro,
                                         QEKF_SensorReadFn read_accel,
                                         void *sensor_context)
{
    if (qe == NULL || !qe->Initialized ||
        read_gyro == NULL || read_accel == NULL) {
        return false;
    }

    qe->GyroInstance = gyro_instance;
    qe->AccelInstance = accel_instance;
    qe->ReadGyro = read_gyro;
    qe->ReadAccel = read_accel;
    qe->SensorContext = sensor_context;
    return true;
}

bool IMU_QuaternionEKF_UpdateFromSensorsInstance(QEKF_INS_t *qe, float dt)
{
    if (qe == NULL || !qe->Initialized ||
        qe->ReadGyro == NULL || qe->ReadAccel == NULL) {
        return false;
    }

    float gyro[3];
    float accel[3];
    if (qe->ReadGyro(qe->SensorContext, qe->GyroInstance, gyro) != 0 ||
        qe->ReadAccel(qe->SensorContext, qe->AccelInstance, accel) != 0) {
        return false;
    }

    return IMU_QuaternionEKF_UpdateInstance(qe,
                                           gyro[0], gyro[1], gyro[2],
                                           accel[0], accel[1], accel[2],
                                           dt);
}

bool IMU_QuaternionEKF_UpdateInstance(QEKF_INS_t *qe,
                                      float gx,
                                      float gy,
                                      float gz,
                                      float ax,
                                      float ay,
                                      float az,
                                      float dt)
{
    if (qe == NULL || !isfinite(dt) || dt <= 0.0f ||
        !qe_input_finite(gx, gy, gz, ax, ay, az)) {
        return false;
    }

    if (!qe->Initialized) {
        if (!IMU_QuaternionEKF_InitInstance(qe, qe->InstanceIndex,
                                            10.0f, 0.001f,
                                            1000000.0f * 10.0f,
                                            1.0f, 0.0f)) {
            return false;
        }
    }

    float halfgxdt;
    float halfgydt;
    float halfgzdt;

    qe->dt = dt;

    qe->Gyro[0] = gx - qe->GyroBias[0];
    qe->Gyro[1] = gy - qe->GyroBias[1];
    qe->Gyro[2] = gz - qe->GyroBias[2];

    halfgxdt = 0.5f * qe->Gyro[0] * dt;
    halfgydt = 0.5f * qe->Gyro[1] * dt;
    halfgzdt = 0.5f * qe->Gyro[2] * dt;

    memcpy(qe->IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F,
           sizeof(IMU_QuaternionEKF_F));

    qe->IMU_QuaternionEKF.F_data[1] = -halfgxdt;
    qe->IMU_QuaternionEKF.F_data[2] = -halfgydt;
    qe->IMU_QuaternionEKF.F_data[3] = -halfgzdt;

    qe->IMU_QuaternionEKF.F_data[6] = halfgxdt;
    qe->IMU_QuaternionEKF.F_data[8] = halfgzdt;
    qe->IMU_QuaternionEKF.F_data[9] = -halfgydt;

    qe->IMU_QuaternionEKF.F_data[12] = halfgydt;
    qe->IMU_QuaternionEKF.F_data[13] = -halfgzdt;
    qe->IMU_QuaternionEKF.F_data[15] = halfgxdt;

    qe->IMU_QuaternionEKF.F_data[18] = halfgzdt;
    qe->IMU_QuaternionEKF.F_data[19] = halfgydt;
    qe->IMU_QuaternionEKF.F_data[20] = -halfgxdt;

    if (qe->UpdateCount == 0) {
        qe->Accel[0] = ax;
        qe->Accel[1] = ay;
        qe->Accel[2] = az;
    }

    const float alpha =
        dt / (dt + qe->accLPFcoef);
    qe->Accel[0] = qe->Accel[0] * (1.0f - alpha) + ax * alpha;
    qe->Accel[1] = qe->Accel[1] * (1.0f - alpha) + ay * alpha;
    qe->Accel[2] = qe->Accel[2] * (1.0f - alpha) + az * alpha;

    const float accelNormSquared =
        qe->Accel[0] * qe->Accel[0] +
        qe->Accel[1] * qe->Accel[1] +
        qe->Accel[2] * qe->Accel[2];
    if (!(accelNormSquared > 1.0e-12f)) {
        return false;
    }

    Vector3_Normalize(qe->IMU_QuaternionEKF.MeasuredVector, qe->Accel);

    qe->gyro_norm = Vector3_Length(qe->Gyro);
    qe->accl_norm = Vector3_Length(qe->Accel);

    qe->StableFlag =
        (qe->gyro_norm < 0.3f &&
         qe->accl_norm > 9.8f - 0.5f &&
         qe->accl_norm < 9.8f + 0.5f) ? 1 : 0;

    memset(qe->IMU_QuaternionEKF.Q_data, 0,
           sizeof(float) * qe->IMU_QuaternionEKF.xhatSize *
           qe->IMU_QuaternionEKF.xhatSize);
    memset(qe->IMU_QuaternionEKF.R_data, 0,
           sizeof(float) * qe->IMU_QuaternionEKF.zSize *
           qe->IMU_QuaternionEKF.zSize);

    qe->IMU_QuaternionEKF.Q_data[0] = qe->Q1 * dt;
    qe->IMU_QuaternionEKF.Q_data[7] = qe->Q1 * dt;
    qe->IMU_QuaternionEKF.Q_data[14] = qe->Q1 * dt;
    qe->IMU_QuaternionEKF.Q_data[21] = qe->Q1 * dt;
    qe->IMU_QuaternionEKF.Q_data[28] = qe->Q2 * dt;
    qe->IMU_QuaternionEKF.Q_data[35] = qe->Q2 * dt;

    qe->IMU_QuaternionEKF.R_data[0] = qe->R;
    qe->IMU_QuaternionEKF.R_data[4] = qe->R;
    qe->IMU_QuaternionEKF.R_data[8] = qe->R;

    if (Kalman_Filter_Update(&qe->IMU_QuaternionEKF) == NULL) {
        return false;
    }

    for (uint8_t i = 0; i < 4; ++i) {
        qe->q[i] = qe->IMU_QuaternionEKF.FilteredValue[i];
    }
    qe->GyroBias[0] = qe->IMU_QuaternionEKF.FilteredValue[4];
    qe->GyroBias[1] = qe->IMU_QuaternionEKF.FilteredValue[5];
    qe->GyroBias[2] = 0.0f;

    qe->Yaw = atan2f(
        2.0f * (qe->q[0] * qe->q[3] + qe->q[1] * qe->q[2]),
        2.0f * (qe->q[0] * qe->q[0] + qe->q[1] * qe->q[1]) - 1.0f
    ) * 57.295779513f;

    qe->Pitch = atan2f(
        2.0f * (qe->q[0] * qe->q[1] + qe->q[2] * qe->q[3]),
        2.0f * (qe->q[0] * qe->q[0] + qe->q[3] * qe->q[3]) - 1.0f
    ) * 57.295779513f;

    qe->Roll = asinf(
        -2.0f * (qe->q[1] * qe->q[3] - qe->q[0] * qe->q[2])
    ) * 57.295779513f;

    if (qe->Yaw - qe->YawAngleLast > 180.0f) {
        --qe->YawRoundCount;
    } else if (qe->Yaw - qe->YawAngleLast < -180.0f) {
        ++qe->YawRoundCount;
    }
    qe->YawTotalAngle =
        360.0f * (float)qe->YawRoundCount + qe->Yaw;
    qe->YawAngleLast = qe->Yaw;
    ++qe->UpdateCount;

    return qe->IMU_QuaternionEKF.MatStatus == MY_MATH_OK;
}

void IMU_QuaternionEKF_Update(float gx,
                              float gy,
                              float gz,
                              float ax,
                              float ay,
                              float az,
                              float dt)
{
    (void)IMU_QuaternionEKF_UpdateInstance(&QEKF_INS,
                                           gx, gy, gz,
                                           ax, ay, az, dt);
}

static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf)
{
    QEKF_INS_t *qe = qe_from_kf(kf);
    if (qe == NULL) {
        return;
    }

    float q0 = kf->xhatminus_data[0];
    float q1 = kf->xhatminus_data[1];
    float q2 = kf->xhatminus_data[2];
    float q3 = kf->xhatminus_data[3];

    const float normSquared =
        q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    if (!(normSquared > 1.0e-12f)) {
        return;
    }

    const float qInvNorm = fast_invSqrt(normSquared);
    for (uint8_t i = 0; i < 4; ++i) {
        kf->xhatminus_data[i] *= qInvNorm;
    }
    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    kf->F_data[4] = q1 * qe->dt / 2.0f;
    kf->F_data[5] = q2 * qe->dt / 2.0f;

    kf->F_data[10] = -q0 * qe->dt / 2.0f;
    kf->F_data[11] = q3 * qe->dt / 2.0f;

    kf->F_data[16] = -q3 * qe->dt / 2.0f;
    kf->F_data[17] = -q0 * qe->dt / 2.0f;

    kf->F_data[22] = q2 * qe->dt / 2.0f;
    kf->F_data[23] = -q1 * qe->dt / 2.0f;

    kf->P_data[28] /= qe->lambda;
    kf->P_data[35] /= qe->lambda;

    if (kf->P_data[28] > 10000.0f) {
        kf->P_data[28] = 10000.0f;
    }
    if (kf->P_data[35] > 10000.0f) {
        kf->P_data[35] = 10000.0f;
    }
}

static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf)
{
    if (kf == NULL || !kf->Initialized) {
        return;
    }

    const float doubleq0 = 2.0f * kf->xhatminus_data[0];
    const float doubleq1 = 2.0f * kf->xhatminus_data[1];
    const float doubleq2 = 2.0f * kf->xhatminus_data[2];
    const float doubleq3 = 2.0f * kf->xhatminus_data[3];

    qe_sanitize_covariance(kf);

    memset(kf->H_data, 0,
           sizeof(float) * kf->zSize * kf->xhatSize);

    kf->H_data[0] = -doubleq2;
    kf->H_data[1] = doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] = doubleq1;

    kf->H_data[6] = doubleq1;
    kf->H_data[7] = doubleq0;
    kf->H_data[8] = doubleq3;
    kf->H_data[9] = doubleq2;

    kf->H_data[12] = doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] = doubleq3;
}

static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf)
{
    QEKF_INS_t *qe = qe_from_kf(kf);
    if (qe == NULL || kf == NULL || !kf->Initialized) {
        return;
    }

    (void)MyMath_Matrix_SetDimensions(&kf->temp_matrix,
                                      kf->H.numRows,
                                      kf->Pminus.numCols);
    (void)MyMath_Matrix_SetDimensions(&kf->temp_matrix1,
                                      kf->temp_matrix.numRows,
                                      kf->HT.numCols);

    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT);
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus,
                                    &kf->temp_matrix);
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT,
                                    &kf->temp_matrix1);
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S);
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);

    if (kf->MatStatus != MY_MATH_OK) {
        memcpy(kf->xhat_data, kf->xhatminus_data,
               sizeof(float) * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data,
               sizeof(float) * kf->xhatSize * kf->xhatSize);
        kf->SkipEq5 = TRUE;
        return;
    }

    const float q0 = kf->xhatminus_data[0];
    const float q1 = kf->xhatminus_data[1];
    const float q2 = kf->xhatminus_data[2];
    const float q3 = kf->xhatminus_data[3];

    (void)MyMath_Matrix_SetDimensions(&kf->temp_vector,
                                      kf->H.numRows, 1);
    kf->temp_vector_data[0] = 2.0f * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2.0f * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] =
        q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    for (uint8_t i = 0; i < 3; ++i) {
        qe->OrientationCosine[i] =
            acosf(fabsf(kf->temp_vector_data[i]));
    }

    (void)MyMath_Matrix_SetDimensions(&kf->temp_vector1,
                                      kf->z.numRows, 1);
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector,
                                    &kf->temp_vector1);

    (void)MyMath_Matrix_SetDimensions(&kf->temp_matrix,
                                      kf->temp_vector1.numRows, 1);
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1,
                                    &kf->temp_vector1,
                                    &kf->temp_matrix);

    (void)MyMath_Matrix_SetDimensions(&kf->temp_vector,
                                      1, kf->temp_vector1.numRows);
    kf->MatStatus = Matrix_Transpose(&kf->temp_vector1,
                                     &kf->temp_vector);
    kf->MatStatus = Matrix_Multiply(&kf->temp_vector,
                                    &kf->temp_matrix,
                                    &qe->ChiSquare);

    if (qe->ChiSquare_Data[0] <
        0.5f * qe->ChiSquareTestThreshold) {
        qe->ConvergeFlag = 1;
    }

    if (qe->ChiSquare_Data[0] > qe->ChiSquareTestThreshold &&
        qe->ConvergeFlag) {
        if (qe->StableFlag) {
            ++qe->ErrorCount;
        } else {
            qe->ErrorCount = 0;
        }

        if (qe->ErrorCount > 50U) {
            qe->ConvergeFlag = 0;
            kf->SkipEq5 = FALSE;
        } else {
            memcpy(kf->xhat_data, kf->xhatminus_data,
                   sizeof(float) * kf->xhatSize);
            memcpy(kf->P_data, kf->Pminus_data,
                   sizeof(float) * kf->xhatSize * kf->xhatSize);
            kf->SkipEq5 = TRUE;
            return;
        }
    } else {
        if (qe->ChiSquare_Data[0] >
                0.1f * qe->ChiSquareTestThreshold &&
            qe->ConvergeFlag) {
            qe->AdaptiveGainScale =
                (qe->ChiSquareTestThreshold -
                 qe->ChiSquare_Data[0]) /
                (0.9f * qe->ChiSquareTestThreshold);
        } else {
            qe->AdaptiveGainScale = 1.0f;
        }
        qe->ErrorCount = 0;
        kf->SkipEq5 = FALSE;
    }

    (void)MyMath_Matrix_SetDimensions(&kf->temp_matrix,
                                      kf->Pminus.numRows,
                                      kf->HT.numCols);
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT,
                                    &kf->temp_matrix);
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix,
                                    &kf->temp_matrix1,
                                    &kf->K);

    for (uint16_t i = 0;
         i < (uint16_t)(kf->K.numRows * kf->K.numCols); ++i) {
        kf->K_data[i] *= qe->AdaptiveGainScale;
    }

    for (uint8_t i = 4; i < 6; ++i) {
        for (uint8_t j = 0; j < 3; ++j) {
            kf->K_data[i * 3 + j] *=
                qe->OrientationCosine[i - 4] / 1.5707963f;
        }
    }

    (void)MyMath_Matrix_SetDimensions(&kf->temp_vector,
                                      kf->K.numRows, 1);
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1,
                                    &kf->temp_vector);

    if (qe->ConvergeFlag) {
        for (uint8_t i = 4; i < 6; ++i) {
            const float limit = 1.0e-2f * qe->dt;
            if (kf->temp_vector_data[i] > limit) {
                kf->temp_vector_data[i] = limit;
            } else if (kf->temp_vector_data[i] < -limit) {
                kf->temp_vector_data[i] = -limit;
            }
        }
    }

    kf->temp_vector_data[3] = 0.0f;
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector,
                               &kf->xhat);

    const float qnorm2 =
        kf->xhat_data[0] * kf->xhat_data[0] +
        kf->xhat_data[1] * kf->xhat_data[1] +
        kf->xhat_data[2] * kf->xhat_data[2] +
        kf->xhat_data[3] * kf->xhat_data[3];
    if (qnorm2 > 1.0e-12f) {
        const float invNorm = fast_invSqrt(qnorm2);
        for (uint8_t i = 0; i < 4; ++i) {
            kf->xhat_data[i] *= invNorm;
        }
    }
}

static void IMU_QuaternionEKF_P_PostProcess(KalmanFilter_t *kf)
{
    if (kf == NULL || !kf->Initialized) {
        return;
    }

    qe_sanitize_covariance(kf);

    const float qnorm2 =
        kf->xhat_data[0] * kf->xhat_data[0] +
        kf->xhat_data[1] * kf->xhat_data[1] +
        kf->xhat_data[2] * kf->xhat_data[2] +
        kf->xhat_data[3] * kf->xhat_data[3];
    if (qnorm2 > 1.0e-12f && isfinite(qnorm2)) {
        const float invNorm = fast_invSqrt(qnorm2);
        for (uint8_t i = 0; i < 4; ++i) {
            kf->xhat_data[i] *= invNorm;
        }
    }
}

static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
    QEKF_INS_t *qe = qe_from_kf(kf);
    if (qe == NULL || kf == NULL) {
        return;
    }

    memcpy(qe->P_snapshot, kf->P_data, sizeof(qe->P_snapshot));
    memcpy(qe->K_snapshot, kf->K_data, sizeof(qe->K_snapshot));
    memcpy(qe->H_snapshot, kf->H_data, sizeof(qe->H_snapshot));
}

