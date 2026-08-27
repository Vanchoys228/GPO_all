#ifndef YOUBOT_WEB_CONTROLLER_WEBOTS_SIMULATION_H
#define YOUBOT_WEB_CONTROLLER_WEBOTS_SIMULATION_H

#include <stddef.h>

#include <webots/supervisor.h>

int controller_webots_simulation_format_limit_wall(
    char *buffer,
    size_t buffer_size,
    const char *def_name,
    double ax,
    double ay,
    double bx,
    double by,
    double wall_thickness,
    double wall_height);
void controller_webots_simulation_remove_nodes(char defs[][64], int *count);

#endif
