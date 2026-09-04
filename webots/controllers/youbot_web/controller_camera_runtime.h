#ifndef YOUBOT_WEB_CONTROLLER_CAMERA_RUNTIME_H
#define YOUBOT_WEB_CONTROLLER_CAMERA_RUNTIME_H

#include "controller_perception_runtime.h"
#include "controller_runtime.h"
#include "controller_webots_pose.h"
#include "controller_webots_sensors.h"

typedef struct {
  double (*current_time)(void *context);
  void (*read_pose)(void *context, double *x, double *y, double *heading);
  void (*merge_free_ray)(void *context, double angle, double range, int confidence);
  void (*merge_observation)(void *context, double angle, double range, int confidence);
  void *context;
} ControllerCameraRuntimeOperations;

typedef struct {
  int time_step_ms;
  int capture_interval;
  int write_interval;
  int frame_width;
  int frame_height;
  int max_virtual_clusters;
  double default_fov;
  double min_obstacle_score;
  double range_fallback;
  double range_search_window;
  double min_trace_range;
  double max_trace_range;
  double track_caution_range;
  double avoid_stop_range;
  double free_ray_min_range;
  const char *bmp_path;
  const char *bmp_temp_path;
  const char *jpeg_path;
  const char *jpeg_temp_path;
} ControllerCameraRuntimeConfig;

typedef struct {
  ControllerWebotsSensors *sensors;
  ControllerPerceptionRuntime *perception;
  ControllerRuntime *controller;
  ControllerWebotsPose *pose;
  int *step_counter;
  ControllerCameraRuntimeConfig config;
  ControllerCameraRuntimeOperations operations;
} ControllerCameraRuntime;

void controller_camera_runtime_init(
    ControllerCameraRuntime *runtime,
    ControllerWebotsSensors *sensors,
    ControllerPerceptionRuntime *perception,
    ControllerRuntime *controller,
    ControllerWebotsPose *pose,
    int *step_counter,
    const ControllerCameraRuntimeConfig *config,
    const ControllerCameraRuntimeOperations *operations);
void controller_camera_runtime_configure_sensors(ControllerCameraRuntime *runtime);
void controller_camera_runtime_capture(ControllerCameraRuntime *runtime);
void controller_camera_runtime_publish(ControllerCameraRuntime *runtime);
void merge_camera_free_ray_into_map(
    double relative_angle, double range, int confidence_boost);
void merge_camera_observation_into_map(
    double relative_angle, double range, int confidence_boost);

#endif
