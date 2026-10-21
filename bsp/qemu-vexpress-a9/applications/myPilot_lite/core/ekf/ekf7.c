#include "quaternion_ekf7.h"

#include <math.h>
#include <string.h>

#define QEK7_N QEK7_STATE_SIZE
#define QEK7_EPS 1.0e-9f
#define QEK7_GRAVITY 9.80665f
#define QEK7_PI 3.14159265358979323846f

static bool finite3(const float v[3])
{
    return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static float clampf_local(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static bool normalize3(const float in[3], float out[3])
{
    const float n2 = in[0] * in[0] + in[1] * in[1] + in[2] * in[2];
    if (!isfinite(n2) || n2 < QEK7_EPS) {
        return false;
    }

    const float inv_n = 1.0f / sqrtf(n2);
    out[0] = in[0] * inv_n;
    out[1] = in[1] * inv_n;
    out[2] = in[2] * inv_n;
    return finite3(out);
}

static bool normalize_quaternion(float q[4])
{
    const float n2 = q[0] * q[0] + q[1] * q[1] +
                     q[2] * q[2] + q[3] * q[3];
    if (!isfinite(n2) || n2 < QEK7_EPS) {
        return false;
    }

    const float inv_n = 1.0f / sqrtf(n2);
    q[0] *= inv_n;
    q[1] *= inv_n;
    q[2] *= inv_n;
    q[3] *= inv_n;
    return true;
}

static void quaternion_multiply(const float a[4],
                                const float b[4],
                                float out[4])
{
    out[0] = a[0] * b[0] - a[1] * b[1] -
             a[2] * b[2] - a[3] * b[3];
    out[1] = a[0] * b[1] + a[1] * b[0] +
             a[2] * b[3] - a[3] * b[2];
    out[2] = a[0] * b[2] - a[1] * b[3] +
             a[2] * b[0] + a[3] * b[1];
    out[3] = a[0] * b[3] + a[1] * b[2] -
             a[2] * b[1] + a[3] * b[0];
}

static bool quaternion_from_rotation_vector(const float rv[3], float q[4])
{
    const float angle = sqrtf(rv[0] * rv[0] +
                              rv[1] * rv[1] +
                              rv[2] * rv[2]);
    if (!isfinite(angle)) {
        return false;
    }

    if (angle < 1.0e-7f) {
        q[0] = 1.0f;
        q[1] = 0.5f * rv[0];
        q[2] = 0.5f * rv[1];
        q[3] = 0.5f * rv[2];
    } else {
        const float half = 0.5f * angle;
        const float s = sinf(half) / angle;
        q[0] = cosf(half);
        q[1] = rv[0] * s;
        q[2] = rv[1] * s;
        q[3] = rv[2] * s;
    }

    return normalize_quaternion(q);
}

static void rotate_body_to_nav(const float q[4],
                               const float v_body[3],
                               float v_nav[3])
{
    const float qw = q[0];
    const float qx = q[1];
    const float qy = q[2];
    const float qz = q[3];

    const float qw2 = qw * qw;
    const float qx2 = qx * qx;
    const float qy2 = qy * qy;
    const float qz2 = qz * qz;

    v_nav[0] = (qw2 + qx2 - qy2 - qz2) * v_body[0] +
               2.0f * (qx * qy - qw * qz) * v_body[1] +
               2.0f * (qx * qz + qw * qy) * v_body[2];

    v_nav[1] = 2.0f * (qx * qy + qw * qz) * v_body[0] +
               (qw2 - qx2 + qy2 - qz2) * v_body[1] +
               2.0f * (qy * qz - qw * qx) * v_body[2];

    v_nav[2] = 2.0f * (qx * qz - qw * qy) * v_body[0] +
               2.0f * (qy * qz + qw * qx) * v_body[1] +
               (qw2 - qx2 - qy2 + qz2) * v_body[2];
}

/*
 * Jacobian of R(q) * v_body with respect to the four quaternion elements.
 * The result is a 3x4 matrix in row-major order.
 */
static void rotate_jacobian(const float q[4],
                            const float v[3],
                            float H[3][4])
{
    const float qw = q[0];
    const float qx = q[1];
    const float qy = q[2];
    const float qz = q[3];
    const float vx = v[0];
    const float vy = v[1];
    const float vz = v[2];

    H[0][0] = 2.0f * (qw * vx - qz * vy + qy * vz);
    H[0][1] = 2.0f * (qx * vx + qy * vy + qz * vz);
    H[0][2] = 2.0f * (-qy * vx + qx * vy + qw * vz);
    H[0][3] = 2.0f * (-qz * vx - qw * vy + qx * vz);

    H[1][0] = 2.0f * (qz * vx + qw * vy - qx * vz);
    H[1][1] = 2.0f * (qy * vx - qx * vy - qw * vz);
    H[1][2] = 2.0f * (qx * vx + qy * vy + qz * vz);
    H[1][3] = 2.0f * (qw * vx - qz * vy + qy * vz);

    H[2][0] = 2.0f * (-qy * vx + qx * vy + qw * vz);
    H[2][1] = 2.0f * (qz * vx + qw * vy - qx * vz);
    H[2][2] = 2.0f * (-qw * vx + qz * vy - qy * vz);
    H[2][3] = 2.0f * (qx * vx + qy * vy + qz * vz);
}

static void symmetrize_and_limit_covariance(qekf7_t *ekf)
{
    float (*P)[QEK7_N] = ekf->P;

    for (int i = 0; i < QEK7_N; ++i) {
        for (int j = i + 1; j < QEK7_N; ++j) {
            const float average = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = average;
            P[j][i] = average;
        }
    }

    for (int i = 0; i < QEK7_N; ++i) {
        const bool is_quaternion = (i < 4);
        const float p_min = is_quaternion ?
                            ekf->cfg.p_quat_min :
                            ekf->cfg.p_bias_min;
        const float p_max = is_quaternion ?
                            ekf->cfg.p_quat_max :
                            ekf->cfg.p_bias_max;

        if (!isfinite(P[i][i])) {
            P[i][i] = p_max;
        }
        P[i][i] = clampf_local(P[i][i], p_min, p_max);
    }

    /*
     * Enforce |Pij| <= sqrt(Pii * Pjj), which is required for a valid
     * covariance matrix. This also limits damage from numerical drift.
     */
    for (int i = 0; i < QEK7_N; ++i) {
        for (int j = i + 1; j < QEK7_N; ++j) {
            const float limit = 0.999f * sqrtf(P[i][i] * P[j][j]);
            float value = P[i][j];
            if (!isfinite(value)) {
                value = 0.0f;
            }
            value = clampf_local(value, -limit, limit);
            P[i][j] = value;
            P[j][i] = value;
        }
    }
}

static void clamp_gyro_bias(qekf7_t *ekf)
{
    const float limit = ekf->cfg.gyro_bias_limit_rad_s;
    for (int i = 0; i < 3; ++i) {
        if (!isfinite(ekf->gyro_bias[i])) {
            ekf->gyro_bias[i] = 0.0f;
        }
        ekf->gyro_bias[i] = clampf_local(ekf->gyro_bias[i], -limit, limit);
    }
}

static void multiply_7x7(float A[QEK7_N][QEK7_N],
                         float B[QEK7_N][QEK7_N],
                         float C[QEK7_N][QEK7_N])
{
    for (int i = 0; i < QEK7_N; ++i) {
        for (int j = 0; j < QEK7_N; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < QEK7_N; ++k) {
                sum += A[i][k] * B[k][j];
            }
            C[i][j] = sum;
        }
    }
}

static void transpose_7x7(float A[QEK7_N][QEK7_N],
                          float T[QEK7_N][QEK7_N])
{
    for (int i = 0; i < QEK7_N; ++i) {
        for (int j = 0; j < QEK7_N; ++j) {
            T[i][j] = A[j][i];
        }
    }
}

static bool scalar_axis_nis(const qekf7_t *ekf,
                            const float v_body[3],
                            int nav_axis,
                            float reference_value,
                            float measurement_var,
                            float *nis_out)
{
    float predicted[3];
    float J[3][4];

    rotate_body_to_nav(ekf->q, v_body, predicted);
    rotate_jacobian(ekf->q, v_body, J);

    float H[QEK7_N] = {0};
    for (int i = 0; i < 4; ++i) {
        H[i] = J[nav_axis][i];
    }

    float PHt[QEK7_N];
    for (int i = 0; i < QEK7_N; ++i) {
        float sum = 0.0f;
        for (int j = 0; j < QEK7_N; ++j) {
            sum += ekf->P[i][j] * H[j];
        }
        PHt[i] = sum;
    }

    float S = measurement_var;
    for (int i = 0; i < QEK7_N; ++i) {
        S += H[i] * PHt[i];
    }

    const float innovation = reference_value - predicted[nav_axis];
    if (!isfinite(S) || S <= QEK7_EPS || !isfinite(innovation)) {
        *nis_out = INFINITY;
        return false;
    }

    *nis_out = innovation * innovation / S;
    return isfinite(*nis_out);
}

static bool scalar_axis_update(qekf7_t *ekf,
                               const float v_body[3],
                               int nav_axis,
                               float reference_value,
                               float measurement_var,
                               float gate_sigma)
{
    float predicted[3];
    float J[3][4];

    rotate_body_to_nav(ekf->q, v_body, predicted);
    rotate_jacobian(ekf->q, v_body, J);

    float H[QEK7_N] = {0};
    for (int i = 0; i < 4; ++i) {
        H[i] = J[nav_axis][i];
    }

    float PHt[QEK7_N];
    for (int i = 0; i < QEK7_N; ++i) {
        float sum = 0.0f;
        for (int j = 0; j < QEK7_N; ++j) {
            sum += ekf->P[i][j] * H[j];
        }
        PHt[i] = sum;
    }

    float S = measurement_var;
    for (int i = 0; i < QEK7_N; ++i) {
        S += H[i] * PHt[i];
    }

    if (!isfinite(S) || S <= QEK7_EPS) {
        return false;
    }

    const float innovation = reference_value - predicted[nav_axis];
    const float nis = innovation * innovation / S;
    const float gate_squared = gate_sigma * gate_sigma;
    if (!isfinite(nis) || nis > gate_squared) {
        return false;
    }

    const float inv_s = 1.0f / S;
    float K[QEK7_N];
    for (int i = 0; i < QEK7_N; ++i) {
        K[i] = PHt[i] * inv_s;
    }

    for (int i = 0; i < QEK7_N; ++i) {
        const float delta = K[i] * innovation;
        if (i < 4) {
            ekf->q[i] += delta;
        } else {
            ekf->gyro_bias[i - 4] += delta;
        }
    }

    if (!normalize_quaternion(ekf->q)) {
        return false;
    }

    /*
     * For the scalar optimal gain:
     * P_new = P - K * S * K^T
     * This preserves symmetry by construction.
     */
    for (int i = 0; i < QEK7_N; ++i) {
        for (int j = i; j < QEK7_N; ++j) {
            const float value = ekf->P[i][j] - K[i] * S * K[j];
            ekf->P[i][j] = value;
            ekf->P[j][i] = value;
        }
    }

    clamp_gyro_bias(ekf);
    symmetrize_and_limit_covariance(ekf);
    return true;
}

static float quaternion_dot(const float a[4], const float b[4])
{
    return a[0] * b[0] + a[1] * b[1] +
           a[2] * b[2] + a[3] * b[3];
}

static void quaternion_slerp(const float a[4],
                             const float b[4],
                             float alpha,
                             float out[4])
{
    alpha = clampf_local(alpha, 0.0f, 1.0f);

    float target[4] = {b[0], b[1], b[2], b[3]};
    float dot = quaternion_dot(a, target);

    if (dot < 0.0f) {
        for (int i = 0; i < 4; ++i) {
            target[i] = -target[i];
        }
        dot = -dot;
    }

    dot = clampf_local(dot, -1.0f, 1.0f);
    float wa;
    float wb;

    if (dot > 0.9995f) {
        wa = 1.0f - alpha;
        wb = alpha;
    } else {
        const float theta = acosf(dot);
        const float sin_theta = sinf(theta);
        if (fabsf(sin_theta) < QEK7_EPS) {
            wa = 1.0f - alpha;
            wb = alpha;
        } else {
            wa = sinf((1.0f - alpha) * theta) / sin_theta;
            wb = sinf(alpha * theta) / sin_theta;
        }
    }

    for (int i = 0; i < 4; ++i) {
        out[i] = wa * a[i] + wb * target[i];
    }
    normalize_quaternion(out);
}

void qekf7_config_default(qekf7_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));
    cfg->gyro_noise_density = 0.01f;
    cfg->gyro_bias_random_walk = 1.0e-4f;
    cfg->accel_noise = 0.03f;
    cfg->mag_noise = 0.05f;
    cfg->accel_gate_sigma = 5.0f;
    cfg->mag_gate_sigma = 5.0f;
    cfg->accel_norm_min_g = 0.75f;
    cfg->accel_norm_max_g = 1.25f;
    cfg->mag_norm_min = 0.0f;
    cfg->mag_norm_max = 0.0f;
    cfg->initial_attitude_std_rad = 0.35f;
    cfg->initial_bias_std_rad_s = 0.05f;
    cfg->gyro_bias_limit_rad_s = 0.5f;
    cfg->p_quat_min = 1.0e-10f;
    cfg->p_quat_max = 0.05f;
    cfg->p_bias_min = 1.0e-12f;
    cfg->p_bias_max = 1.0f;

    cfg->accel_ref_ned[0] = 0.0f;
    cfg->accel_ref_ned[1] = 0.0f;
    cfg->accel_ref_ned[2] = -1.0f;
    cfg->mag_ref_ned[0] = 1.0f;
    cfg->mag_ref_ned[1] = 0.0f;
    cfg->mag_ref_ned[2] = 0.0f;
    cfg->output_smooth_tau_s = 0.05f;
}

bool qekf7_init(qekf7_t *ekf,
                const qekf7_config_t *cfg,
                const float q_init[4],
                const float gyro_bias_init[3])
{
    if (ekf == NULL || cfg == NULL || q_init == NULL) {
        return false;
    }

    memset(ekf, 0, sizeof(*ekf));
    ekf->cfg = *cfg;

    float accel_ref[3];
    float mag_ref[3];
    if (!normalize3(cfg->accel_ref_ned, accel_ref) ||
        !normalize3(cfg->mag_ref_ned, mag_ref)) {
        return false;
    }
    memcpy(ekf->cfg.accel_ref_ned, accel_ref, sizeof(accel_ref));
    memcpy(ekf->cfg.mag_ref_ned, mag_ref, sizeof(mag_ref));

    if (!isfinite(cfg->gyro_noise_density) ||
        !isfinite(cfg->gyro_bias_random_walk) ||
        cfg->gyro_noise_density <= 0.0f ||
        cfg->gyro_bias_random_walk <= 0.0f ||
        !isfinite(cfg->accel_noise) ||
        !isfinite(cfg->mag_noise) ||
        cfg->accel_noise <= 0.0f ||
        cfg->mag_noise <= 0.0f ||
        !isfinite(cfg->accel_gate_sigma) ||
        !isfinite(cfg->mag_gate_sigma) ||
        cfg->accel_gate_sigma <= 0.0f ||
        cfg->mag_gate_sigma <= 0.0f) {
        return false;
    }

    if (cfg->gyro_bias_limit_rad_s <= 0.0f ||
        cfg->p_quat_min <= 0.0f ||
        cfg->p_quat_max <= cfg->p_quat_min ||
        cfg->p_bias_min <= 0.0f ||
        cfg->p_bias_max <= cfg->p_bias_min ||
        cfg->initial_attitude_std_rad <= 0.0f ||
        cfg->initial_bias_std_rad_s <= 0.0f) {
        return false;
    }

    ekf->initialized = true;
    return qekf7_reset(ekf, q_init, gyro_bias_init);
}

bool qekf7_reset(qekf7_t *ekf,
                 const float q_init[4],
                 const float gyro_bias_init[3])
{
    if (ekf == NULL || q_init == NULL || !ekf->initialized) {
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        if (!isfinite(q_init[i])) {
            return false;
        }
        ekf->q[i] = q_init[i];
    }
    if (!normalize_quaternion(ekf->q)) {
        return false;
    }

    for (int i = 0; i < 3; ++i) {
        ekf->gyro_bias[i] = gyro_bias_init != NULL ?
                            gyro_bias_init[i] : 0.0f;
        if (!isfinite(ekf->gyro_bias[i])) {
            ekf->gyro_bias[i] = 0.0f;
        }
    }

    memset(ekf->P, 0, sizeof(ekf->P));

    /*
     * A quaternion component error is roughly half the corresponding
     * local rotation-angle error.
     */
    const float quat_variance =
        0.25f * ekf->cfg.initial_attitude_std_rad *
        ekf->cfg.initial_attitude_std_rad;
    const float bias_variance =
        ekf->cfg.initial_bias_std_rad_s *
        ekf->cfg.initial_bias_std_rad_s;

    for (int i = 0; i < 4; ++i) {
        ekf->P[i][i] = quat_variance;
    }
    for (int i = 4; i < QEK7_N; ++i) {
        ekf->P[i][i] = bias_variance;
    }

    clamp_gyro_bias(ekf);
    symmetrize_and_limit_covariance(ekf);

    memcpy(ekf->q_output, ekf->q, sizeof(ekf->q));
    memset(&ekf->status, 0, sizeof(ekf->status));
    return true;
}

bool qekf7_predict(qekf7_t *ekf, const float gyro_rad_s[3], float dt)
{
    if (ekf == NULL || gyro_rad_s == NULL || !ekf->initialized ||
        !finite3(gyro_rad_s) || !isfinite(dt) || dt <= 0.0f) {
        return false;
    }

    /*
     * Avoid a single corrupted timestamp from producing a huge rotation.
     * Callers should normally constrain dt to a system-specific maximum.
     */
    dt = clampf_local(dt, 1.0e-5f, 0.1f);

    const float wx = gyro_rad_s[0] - ekf->gyro_bias[0];
    const float wy = gyro_rad_s[1] - ekf->gyro_bias[1];
    const float wz = gyro_rad_s[2] - ekf->gyro_bias[2];

    const float rotation_vector[3] = {
        wx * dt,
        wy * dt,
        wz * dt
    };

    float delta_q[4];
    if (!quaternion_from_rotation_vector(rotation_vector, delta_q)) {
        return false;
    }

    float q_predicted[4];
    quaternion_multiply(ekf->q, delta_q, q_predicted);
    if (!normalize_quaternion(q_predicted)) {
        return false;
    }

    const float qw = ekf->q[0];
    const float qx = ekf->q[1];
    const float qy = ekf->q[2];
    const float qz = ekf->q[3];

    float F[QEK7_N][QEK7_N] = {{0}};
    for (int i = 0; i < QEK7_N; ++i) {
        F[i][i] = 1.0f;
    }

    /* d(q_dot)/d(q) * dt */
    F[0][1] = -0.5f * wx * dt;
    F[0][2] = -0.5f * wy * dt;
    F[0][3] = -0.5f * wz * dt;

    F[1][0] = 0.5f * wx * dt;
    F[1][2] = 0.5f * wz * dt;
    F[1][3] = -0.5f * wy * dt;

    F[2][0] = 0.5f * wy * dt;
    F[2][1] = -0.5f * wz * dt;
    F[2][3] = 0.5f * wx * dt;

    F[3][0] = 0.5f * wz * dt;
    F[3][1] = 0.5f * wy * dt;
    F[3][2] = -0.5f * wx * dt;

    /* d(q_dot)/d(gyro_bias) * dt, with omega = gyro - bias. */
    F[0][4] = 0.5f * qx * dt;
    F[0][5] = 0.5f * qy * dt;
    F[0][6] = 0.5f * qz * dt;

    F[1][4] = -0.5f * qw * dt;
    F[1][5] = 0.5f * qz * dt;
    F[1][6] = -0.5f * qy * dt;

    F[2][4] = -0.5f * qz * dt;
    F[2][5] = -0.5f * qw * dt;
    F[2][6] = 0.5f * qx * dt;

    F[3][4] = 0.5f * qy * dt;
    F[3][5] = -0.5f * qx * dt;
    F[3][6] = -0.5f * qw * dt;

    float FP[QEK7_N][QEK7_N];
    float FT[QEK7_N][QEK7_N];
    float FPFT[QEK7_N][QEK7_N];

    multiply_7x7(F, ekf->P, FP);
    transpose_7x7(F, FT);
    multiply_7x7(FP, FT, FPFT);

    /*
     * Q is expressed as variance accumulated over this prediction step.
     * For gyro white noise integrated into angle:
     *   variance(angle) = sigma_g^2 * dt
     * and a quaternion component is approximately half that angle error.
     */
    const float q_quat =
        0.25f * ekf->cfg.gyro_noise_density *
        ekf->cfg.gyro_noise_density * dt;
    const float q_bias =
        ekf->cfg.gyro_bias_random_walk *
        ekf->cfg.gyro_bias_random_walk * dt;

    for (int i = 0; i < 4; ++i) {
        FPFT[i][i] += q_quat;
    }
    for (int i = 4; i < QEK7_N; ++i) {
        FPFT[i][i] += q_bias;
    }

    memcpy(ekf->q, q_predicted, sizeof(ekf->q));
    memcpy(ekf->P, FPFT, sizeof(ekf->P));

    clamp_gyro_bias(ekf);
    symmetrize_and_limit_covariance(ekf);
    return true;
}

bool qekf7_update_accel(qekf7_t *ekf, const float accel_m_s2[3])
{
    if (ekf == NULL) {
        return false;
    }

    ekf->status.accel_accepted = false;
    ekf->status.accel_rejected = false;

    if (accel_m_s2 == NULL || !ekf->initialized || !finite3(accel_m_s2)) {
        ekf->status.accel_rejected = true;
        return false;
    }

    const float norm = sqrtf(accel_m_s2[0] * accel_m_s2[0] +
                             accel_m_s2[1] * accel_m_s2[1] +
                             accel_m_s2[2] * accel_m_s2[2]);
    if (!isfinite(norm) || norm < QEK7_EPS) {
        ekf->status.accel_rejected = true;
        return false;
    }

    const float norm_g = norm / QEK7_GRAVITY;
    if (norm_g < ekf->cfg.accel_norm_min_g ||
        norm_g > ekf->cfg.accel_norm_max_g) {
        ekf->status.accel_rejected = true;
        return false;
    }

    float accel_norm[3];
    if (!normalize3(accel_m_s2, accel_norm)) {
        ekf->status.accel_rejected = true;
        return false;
    }

    const float var = ekf->cfg.accel_noise * ekf->cfg.accel_noise;
    const float gate = ekf->cfg.accel_gate_sigma;

    /*
     * For gravity-like references, only the two navigation-frame horizontal
     * components carry independent tilt information. The Z component is
     * dependent on the unit-norm constraint and is intentionally not fused.
     */
    const bool nis0_ok =
        scalar_axis_nis(ekf, accel_norm, 0,
                        ekf->cfg.accel_ref_ned[0], var,
                        &ekf->status.accel_nis[0]);
    const bool nis1_ok =
        scalar_axis_nis(ekf, accel_norm, 1,
                        ekf->cfg.accel_ref_ned[1], var,
                        &ekf->status.accel_nis[1]);

    const float gate_squared = gate * gate;
    if (!nis0_ok || !nis1_ok ||
        ekf->status.accel_nis[0] > gate_squared ||
        ekf->status.accel_nis[1] > gate_squared) {
        ekf->status.accel_rejected = true;
        return false;
    }

    const bool accepted0 =
        scalar_axis_update(ekf, accel_norm, 0,
                           ekf->cfg.accel_ref_ned[0], var, gate);
    const bool accepted1 =
        scalar_axis_update(ekf, accel_norm, 1,
                           ekf->cfg.accel_ref_ned[1], var, gate);

    ekf->status.accel_accepted = accepted0 && accepted1;
    ekf->status.accel_rejected = !ekf->status.accel_accepted;
    return ekf->status.accel_accepted;
}

bool qekf7_update_mag(qekf7_t *ekf, const float mag[3])
{
    if (ekf == NULL) {
        return false;
    }

    ekf->status.mag_accepted = false;
    ekf->status.mag_rejected = false;

    if (mag == NULL || !ekf->initialized || !finite3(mag)) {
        ekf->status.mag_rejected = true;
        return false;
    }

    const float norm = sqrtf(mag[0] * mag[0] +
                             mag[1] * mag[1] +
                             mag[2] * mag[2]);
    if (!isfinite(norm) || norm < QEK7_EPS) {
        ekf->status.mag_rejected = true;
        return false;
    }

    if ((ekf->cfg.mag_norm_min > 0.0f ||
         ekf->cfg.mag_norm_max > 0.0f) &&
        (norm < ekf->cfg.mag_norm_min ||
         (ekf->cfg.mag_norm_max > 0.0f &&
          norm > ekf->cfg.mag_norm_max))) {
        ekf->status.mag_rejected = true;
        return false;
    }

    float mag_norm[3];
    if (!normalize3(mag, mag_norm)) {
        ekf->status.mag_rejected = true;
        return false;
    }

    const float var = ekf->cfg.mag_noise * ekf->cfg.mag_noise;
    const float gate = ekf->cfg.mag_gate_sigma;

    /*
     * Magnetometer X/Y observations constrain heading. The vertical
     * component is not fused directly because it is strongly affected by
     * local field variation and is not required for attitude stabilization.
     */
    const bool nis0_ok =
        scalar_axis_nis(ekf, mag_norm, 0,
                        ekf->cfg.mag_ref_ned[0], var,
                        &ekf->status.mag_nis[0]);
    const bool nis1_ok =
        scalar_axis_nis(ekf, mag_norm, 1,
                        ekf->cfg.mag_ref_ned[1], var,
                        &ekf->status.mag_nis[1]);

    const float gate_squared = gate * gate;
    if (!nis0_ok || !nis1_ok ||
        ekf->status.mag_nis[0] > gate_squared ||
        ekf->status.mag_nis[1] > gate_squared) {
        ekf->status.mag_rejected = true;
        return false;
    }

    const bool accepted0 =
        scalar_axis_update(ekf, mag_norm, 0,
                           ekf->cfg.mag_ref_ned[0], var, gate);
    const bool accepted1 =
        scalar_axis_update(ekf, mag_norm, 1,
                           ekf->cfg.mag_ref_ned[1], var, gate);

    ekf->status.mag_accepted = accepted0 && accepted1;
    ekf->status.mag_rejected = !ekf->status.mag_accepted;
    return ekf->status.mag_accepted;
}

void qekf7_publish_output(qekf7_t *ekf, float dt)
{
    if (ekf == NULL || !ekf->initialized) {
        return;
    }

    if (!(dt > 0.0f) || !isfinite(dt) ||
        ekf->cfg.output_smooth_tau_s <= 0.0f) {
        memcpy(ekf->q_output, ekf->q, sizeof(ekf->q_output));
        return;
    }

    const float alpha =
        1.0f - expf(-dt / ekf->cfg.output_smooth_tau_s);
    quaternion_slerp(ekf->q_output, ekf->q, alpha, ekf->q_output);
}

void qekf7_get_quaternion(const qekf7_t *ekf, float q_out[4])
{
    if (ekf == NULL || q_out == NULL || !ekf->initialized) {
        return;
    }
    memcpy(q_out, ekf->q, sizeof(ekf->q));
}

void qekf7_get_output_quaternion(const qekf7_t *ekf, float q_out[4])
{
    if (ekf == NULL || q_out == NULL || !ekf->initialized) {
        return;
    }
    memcpy(q_out, ekf->q_output, sizeof(ekf->q_output));
}

void qekf7_get_gyro_bias(const qekf7_t *ekf, float bias_out[3])
{
    if (ekf == NULL || bias_out == NULL || !ekf->initialized) {
        return;
    }
    memcpy(bias_out, ekf->gyro_bias, sizeof(ekf->gyro_bias));
}

void qekf7_get_euler(const qekf7_t *ekf,
                     float *roll,
                     float *pitch,
                     float *yaw)
{
    if (ekf == NULL || !ekf->initialized) {
        return;
    }

    const float qw = ekf->q_output[0];
    const float qx = ekf->q_output[1];
    const float qy = ekf->q_output[2];
    const float qz = ekf->q_output[3];

    const float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    const float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    const float roll_value = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (qw * qy - qz * qx);
    sinp = clampf_local(sinp, -1.0f, 1.0f);
    const float pitch_value = asinf(sinp);

    const float siny_cosp = 2.0f * (qw * qz + qx * qy);
    const float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    const float yaw_value = atan2f(siny_cosp, cosy_cosp);

    if (roll != NULL) {
        *roll = roll_value;
    }
    if (pitch != NULL) {
        *pitch = pitch_value;
    }
    if (yaw != NULL) {
        *yaw = yaw_value;
    }
}
