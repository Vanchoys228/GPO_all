#ifndef YOUBOT_WEB_CONTROLLER_LIDAR_MATH_H
#define YOUBOT_WEB_CONTROLLER_LIDAR_MATH_H

#include "controller_types.h"

typedef struct {
  double effective_max_range;
  double track_caution_range;
  double avoid_stop_range;
  double gap_min_range;
  double front_sector_rad;
  double center_sector_rad;
  double front_corner_min_rad;
  double front_corner_max_rad;
  double sigma;
} ControllerLidarContextConfig;

double controller_lidar_range_pressure(
    double range,
    double clear_range,
    double blocked_range);
int controller_lidar_hit_is_consistent(
    const float *ranges,
    int resolution,
    int index,
    double range,
    double effective_max_range,
    int sample_stride,
    double min_trace_range,
    double range_jump_tolerance);
double controller_lidar_trace_confidence(
    const ObstacleTracePoint *point,
    double now_time,
    double ttl_seconds);
void controller_lidar_context_init(
    LidarObstacleContext *context,
    double effective_max_range);
void controller_lidar_context_observe(
    LidarObstacleContext *context,
    const ControllerLidarContextConfig *config,
    double sensed_range,
    double beam_angle,
    int obstacle_hit,
    int expected_zone_wall,
    double target_beam_angle,
    double preferred_turn_sign);

#endif
