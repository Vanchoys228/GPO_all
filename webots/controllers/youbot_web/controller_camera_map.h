#ifndef YOUBOT_WEB_CONTROLLER_CAMERA_MAP_H
#define YOUBOT_WEB_CONTROLLER_CAMERA_MAP_H

#include "controller_types.h"

int controller_camera_map_append_obstacle(
    MapCell *obstacles,
    int *obstacle_count,
    int obstacle_capacity,
    MapCell *free_cells,
    int *free_count,
    double cell_size,
    double epsilon,
    double x,
    double y,
    int confidence_boost);
int controller_camera_map_append_free(
    const MapCell *obstacles,
    int obstacle_count,
    MapCell *free_cells,
    int *free_count,
    int free_capacity,
    double cell_size,
    double epsilon,
    double x,
    double y,
    int confidence_boost);

#endif
