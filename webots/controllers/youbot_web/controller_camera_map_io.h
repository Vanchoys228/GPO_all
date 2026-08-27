#ifndef YOUBOT_WEB_CONTROLLER_CAMERA_MAP_IO_H
#define YOUBOT_WEB_CONTROLLER_CAMERA_MAP_IO_H

#include "controller_types.h"

int controller_camera_map_io_write(
    const char *json_path,
    const char *json_temp_path,
    const char *csv_path,
    const char *csv_temp_path,
    double cell_size,
    const MapCell *obstacles,
    int obstacle_count,
    const MapCell *free_cells,
    int free_count);
void controller_camera_map_io_clear_files(
    const char *json_path,
    const char *json_temp_path,
    const char *csv_path,
    const char *csv_temp_path);

#endif
