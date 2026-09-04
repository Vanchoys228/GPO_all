#ifndef YOUBOT_WEB_CONTROLLER_LIDAR_RUNTIME_H
#define YOUBOT_WEB_CONTROLLER_LIDAR_RUNTIME_H

#include "controller_avoidance.h"

void compute_lidar_obstacle_context(
    LidarObstacleContext *context,
    double target_beam_angle,
    double preferred_turn_sign);
void capture_lidar_trace(void);
void merge_trace_into_map(double now_time);

#endif
