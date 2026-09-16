#include "types.h"

#include <string.h>

void virtual_imu_reset(flight_imu_sample_t *sample)
{
    if (sample == 0)
    {
        return;
    }

    memset(sample, 0, sizeof(*sample));
    sample->accel_m_s2.z = -9.80665f;
    sample->mag_gauss.x = 1.0f;
    sample->temperature_c = 25.0f;
    sample->gyro_valid = true;
    sample->accel_valid = true;
    sample->mag_valid = true;
}
