#include <stdbool.h>
#include <stddef.h>

#include "types.h"

typedef struct
{
    const flight_imu_sample_t *samples;
    size_t sample_count;
    size_t next_index;
} sensor_replay_t;

void sensor_replay_init(sensor_replay_t *replay,
                        const flight_imu_sample_t *samples,
                        size_t sample_count)
{
    if (replay == 0)
    {
        return;
    }

    replay->samples = samples;
    replay->sample_count = sample_count;
    replay->next_index = 0;
}

bool sensor_replay_next(sensor_replay_t *replay,
                        flight_imu_sample_t *sample)
{
    if (replay == 0 || sample == 0 || replay->samples == 0 ||
        replay->next_index >= replay->sample_count)
    {
        return false;
    }

    *sample = replay->samples[replay->next_index++];
    return true;
}
