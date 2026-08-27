#ifndef YOUBOT_WEB_CONTROLLER_OBSTACLE_MAP_H
#define YOUBOT_WEB_CONTROLLER_OBSTACLE_MAP_H

#include "controller_types.h"

int controller_obstacle_map_write(
    const char *json_path,
    const char *json_temp_path,
    const char *csv_path,
    const char *csv_temp_path,
    double cell_size,
    const MapCell *cells,
    int cell_count);
void controller_obstacle_map_clear_files(
    const char *json_path,
    const char *json_temp_path,
    const char *csv_path,
    const char *csv_temp_path);

#endif
