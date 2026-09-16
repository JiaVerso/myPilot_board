#ifndef MYPILOT_LITE_FLIGHT_TYPES_H
#define MYPILOT_LITE_FLIGHT_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float x;
    float y;
    float z;
} flight_vector3f_t;

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} flight_quaternion_t;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} flight_euler_t;

typedef struct
{
    uint64_t timestamp_us;
    flight_vector3f_t gyro_rad_s;
    flight_vector3f_t accel_m_s2;
    flight_vector3f_t mag_gauss;
    float temperature_c;
    uint32_t sequence;
    bool gyro_valid;
    bool accel_valid;
    bool mag_valid;
} flight_imu_sample_t;

typedef struct
{
    uint64_t timestamp_us;
    float throttle;
    float roll_target_rad;
    float pitch_target_rad;
    float yaw_rate_target_rad_s;
    bool armed;
} flight_rc_command_t;

typedef struct
{
    uint64_t timestamp_us;
    float motor[4];
    bool enabled;
} flight_motor_output_t;

#endif
