#ifndef YOUBOT_WEB_CONTROLLER_WEBOTS_CAMERA_MAP_SYNC_H
#define YOUBOT_WEB_CONTROLLER_WEBOTS_CAMERA_MAP_SYNC_H

#include "controller_camera.h"
#include "controller_types.h"

typedef struct {
  MapCell *obstacles;
  int *obstacle_count;
  int obstacle_capacity;
  MapCell *free_cells;
  int *free_count;
  int free_capacity;
  double cell_size;
  double epsilon;
  ControllerCameraMapGeometryConfig geometry;
  ControllerCameraPose pose;
} ControllerWebotsCameraMapSyncContext;

int controller_webots_camera_map_sync_free_ray(
    ControllerWebotsCameraMapSyncContext *context,
    double relative_angle,
    double range,
    int confidence_boost);
int controller_webots_camera_map_sync_observation(
    ControllerWebotsCameraMapSyncContext *context,
    double relative_angle,
    double range,
    int confidence_boost);

#endif
