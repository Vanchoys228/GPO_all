#ifndef YOUBOT_WEB_CONTROLLER_MAPPING_RUNTIME_H
#define YOUBOT_WEB_CONTROLLER_MAPPING_RUNTIME_H

#include "controller_mapping_store.h"

typedef struct {
  ControllerMappingStorePaths paths;
  double map_cell_size;
  double camera_cell_size;
} ControllerMappingRuntimeConfig;

typedef struct {
  ControllerMappingStore store;
  ControllerMappingRuntimeConfig config;
} ControllerMappingRuntime;

typedef struct {
  MapCell *persistent_map;
  int *persistent_count;
  int *persistent_dirty;
  MapCell *camera_map;
  int *camera_count;
  MapCell *camera_free_map;
  int *camera_free_count;
  int *camera_dirty;
} ControllerMappingRuntimeMaps;

void controller_mapping_runtime_init(
    ControllerMappingRuntime *runtime,
    const ControllerMappingRuntimeConfig *config);
ControllerMappingRuntimeMaps controller_mapping_runtime_maps(
    ControllerMappingRuntime *runtime);
void controller_mapping_runtime_clear(ControllerMappingRuntime *runtime);
int controller_mapping_runtime_write(ControllerMappingRuntime *runtime, int due);
void controller_mapping_runtime_merge_trace(
    ControllerMappingRuntime *runtime,
    const ObstacleTracePoint *trace,
    int trace_count,
    double now_time,
    double max_age_seconds,
    int min_hit_count,
    double epsilon);

#endif
