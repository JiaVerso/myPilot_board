#ifndef QUATERNION_EKF7_H
#define QUATERNION_EKF7_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define QEK7_STATE_SIZE 7

typedef struct {
    /*
     * Continuous gyro white-noise density in rad/s/sqrt(Hz).
     * Typical MEMS values are roughly 0.001 .. 0.05.
     */
    float gyro_noise_density;

    /*
     * Gyro bias random-walk density in rad/s/sqrt(s).
     * Typical MEMS values are roughly 1e-5 .. 1e-3.
     */
    float gyro_bias_random_walk;

    /*
     * Measurement noise is expressed in normalized-vector units.
     * Both accelerometer and magnetometer inputs are normalized internally.
     * A value of 0.03 therefore means roughly 3% direction noise.
     */
    float accel_noise;
    float mag_noise;

    /* Per-axis innovation gates, in standard deviations. */
    float accel_gate_sigma;
    float mag_gate_sigma;

    /*
     * Static acceleration validity window in g. Samples outside this window
     * are rejected, which prevents linear acceleration from corrupting tilt.
     */
    float accel_norm_min_g;
    float accel_norm_max_g;

    /*
     * Optional magnetometer magnitude window in the input unit.
     * Set both limits to zero to disable this check.
     */
    float mag_norm_min;
    float mag_norm_max;

    /*
     * Initial 1-sigma uncertainty.
     */
    float initial_attitude_std_rad;
    float initial_bias_std_rad_s;

    /* Hard limit applied to the estimated gyro bias. */
    float gyro_bias_limit_rad_s;

    /*
     * Covariance diagonal limits. These are numerical safety limits, not
     * primary tuning parameters.
     */
    float p_quat_min;
    float p_quat_max;
    float p_bias_min;
    float p_bias_max;

    /*
     * Reference directions in the navigation frame. They are normalized by
     * qekf7_init().
     *
     * Quaternion convention:
     *   navigation-to-body? No: q rotates a BODY vector into NAVIGATION.
     *   Therefore: v_nav = R(q) * v_body.
     *
     * At rest, a commonly used NED accelerometer reference is (0, 0, -1),
     * because the accelerometer measures specific force opposite gravity.
     * Verify the sign for the actual sensor convention.
     *
     * mag_ref_ned should be the local magnetic field direction in the same
     * navigation frame. Only the X/Y components are used for heading.
     */
    float accel_ref_ned[3];
    float mag_ref_ned[3];

    /*
     * Optional output smoothing. The published quaternion is SLERP-filtered
     * toward the filter state with this time constant.
     *
     * 0 disables smoothing. 0.02 .. 0.1 s is usually a reasonable range.
     */
    float output_smooth_tau_s;
} qekf7_config_t;

typedef struct {
    float accel_nis[2];
    float mag_nis[2];
    bool accel_accepted;
    bool mag_accepted;
    bool accel_rejected;
    bool mag_rejected;
} qekf7_status_t;

typedef struct {
    /* q[0..3] = qw, qx, qy, qz. Body-to-navigation rotation. */
    float q[4];
    /* Body-frame gyro bias in rad/s. */
    float gyro_bias[3];

    /*
     * Direct 7x7 covariance:
     * index 0..3: quaternion elements
     * index 4..6: gyro bias
     */
    float P[QEK7_STATE_SIZE][QEK7_STATE_SIZE];

    /* Smoothed quaternion exposed to the control loop. */
    float q_output[4];

    qekf7_config_t cfg;
    qekf7_status_t status;
    bool initialized;
} qekf7_t;

/* Fills cfg with conservative defaults. */
void qekf7_config_default(qekf7_config_t *cfg);

/*
 * Initializes the filter.
 *
 * q_init must rotate body vectors into the navigation frame and is normalized.
 * gyro_bias_init may be NULL, in which case zero bias is used.
 */
bool qekf7_init(qekf7_t *ekf,
                const qekf7_config_t *cfg,
                const float q_init[4],
                const float gyro_bias_init[3]);

/* Resets the state while keeping the current configuration. */
bool qekf7_reset(qekf7_t *ekf,
                 const float q_init[4],
                 const float gyro_bias_init[3]);

/*
 * IMU prediction step.
 *
 * gyro_rad_s is the raw calibrated gyro rate in body axes.
 * dt is the actual integration interval in seconds.
 */
bool qekf7_predict(qekf7_t *ekf, const float gyro_rad_s[3], float dt);

/*
 * Accelerometer correction.
 *
 * accel_m_s2 is the calibrated body-frame accelerometer vector in m/s^2.
 * The innovation gate and the norm window are applied automatically.
 */
bool qekf7_update_accel(qekf7_t *ekf, const float accel_m_s2[3]);

/*
 * Magnetometer correction.
 *
 * mag is the calibrated body-frame magnetic vector. Its unit is arbitrary,
 * but all samples must use the same unit.
 */
bool qekf7_update_mag(qekf7_t *ekf, const float mag[3]);

/*
 * Advances the optional SLERP output smoother. Call after all measurement
 * updates for the current IMU cycle.
 */
void qekf7_publish_output(qekf7_t *ekf, float dt);

/* Returns the internal filter quaternion. */
void qekf7_get_quaternion(const qekf7_t *ekf, float q_out[4]);

/* Returns the smoothed quaternion intended for external consumers. */
void qekf7_get_output_quaternion(const qekf7_t *ekf, float q_out[4]);

/* Returns body-frame gyro bias in rad/s. */
void qekf7_get_gyro_bias(const qekf7_t *ekf, float bias_out[3]);

/*
 * Converts the smoothed output quaternion to ZYX Euler angles:
 * roll, pitch, yaw in radians.
 */
void qekf7_get_euler(const qekf7_t *ekf,
                     float *roll,
                     float *pitch,
                     float *yaw);

#ifdef __cplusplus
}
#endif

#endif
