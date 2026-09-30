#ifndef YOUBOT_WEB_CONTROLLER_LIDAR_TRACE_H
#define YOUBOT_WEB_CONTROLLER_LIDAR_TRACE_H

#include "controller_types.h"

void controller_lidar_trace_prune(
    ObstacleTracePoint *trace, int *trace_count, double now_time, double ttl_seconds);
void controller_lidar_trace_append(
    ObstacleTracePoint *trace,
    int *trace_count,
    int trace_capacity,
    double spacing,
    double x,
    double y,
    double now_time);
void controller_lidar_trace_merge_into_map(
    const ObstacleTracePoint *trace,
    int trace_count,
    double now_time,
    double max_age_seconds,
    int min_hit_count,
    MapCell *map,
    int *map_count,
    int map_capacity,
    double cell_size,
    double epsilon,
    int *map_dirty);

#endif
