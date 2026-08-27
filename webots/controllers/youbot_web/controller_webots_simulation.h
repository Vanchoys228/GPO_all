#ifndef YOUBOT_WEB_CONTROLLER_WEBOTS_SIMULATION_H
#define YOUBOT_WEB_CONTROLLER_WEBOTS_SIMULATION_H

#include <stddef.h>

#include "controller_types.h"

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
int controller_webots_simulation_format_surface_zone(
    char *buffer,
    size_t buffer_size,
    const SurfaceZone *zone,
    int zone_index);
void controller_webots_simulation_remove_nodes(char defs[][64], int *count);

#endif
