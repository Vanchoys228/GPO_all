#ifndef YOUBOT_WEB_CONTROLLER_WEBOTS_SIMULATION_FORMAT_H
#define YOUBOT_WEB_CONTROLLER_WEBOTS_SIMULATION_FORMAT_H

#include <stddef.h>

#include "controller_types.h"

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
int controller_webots_simulation_format_runtime_obstacle(
    char *buffer,
    size_t buffer_size,
    const RuntimeCommand *command,
    double min_x,
    double max_x,
    double min_y,
    double max_y);

#endif
