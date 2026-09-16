#include "types.h"

#include <string.h>

static flight_motor_output_t last_output;

void virtual_motor_write(const flight_motor_output_t *output)
{
    if (output != 0)
    {
        last_output = *output;
    }
}

void virtual_motor_reset(void)
{
    memset(&last_output, 0, sizeof(last_output));
}

flight_motor_output_t virtual_motor_get_last_output(void)
{
    return last_output;
}
