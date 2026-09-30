#ifndef YOUBOT_WEB_CONTROLLER_LIDAR_SCAN_H
#define YOUBOT_WEB_CONTROLLER_LIDAR_SCAN_H

#include "controller_types.h"

typedef struct {
  int sample_stride;
  double min_trace_range;
  double max_trace_range;
  double range_jump_tolerance;
  double sensor_local_x;
  double sensor_local_y;
  double front_sector_rad;
  double center_sector_rad;
  double front_corner_min_rad;
  double front_corner_max_rad;
  double near_robot_ignore_radius;
  double snap_step;
  double trace_spacing;
} ControllerLidarScanConfig;

typedef struct {
  int hit_count;
  int front_hit_count;
  double front_min_range;
  double center_min_range;
  double left_front_min_range;
  double right_front_min_range;
  double left_min_range;
  double right_min_range;
} ControllerLidarScanStats;

void controller_lidar_scan_capture(
    const ControllerLidarScanConfig *config,
    const float *ranges,
    int resolution,
    double fov,
    double sensor_max_range,
    double robot_x,
    double robot_y,
    double heading,
    double now_time,
    ObstacleTracePoint *trace,
    int *trace_count,
    int trace_capacity,
    ControllerLidarScanStats *stats);

#endif
