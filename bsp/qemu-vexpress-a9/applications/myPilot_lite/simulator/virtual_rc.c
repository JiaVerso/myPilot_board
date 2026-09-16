#include "types.h"

#include <string.h>

void virtual_rc_reset(flight_rc_command_t *command)
{
    if (command == 0)
    {
        return;
    }

    memset(command, 0, sizeof(*command));
    command->armed = false;
}
