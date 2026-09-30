#ifndef YOUBOT_WEB_CONTROLLER_PERCEPTION_RUNTIME_H
#define YOUBOT_WEB_CONTROLLER_PERCEPTION_RUNTIME_H

#include "controller_lidar_scan.h"
#include "controller_types.h"

#define CONTROLLER_PERCEPTION_MAX_TRACE_POINTS 520

typedef struct {
  char file_name[64];
  char mime_type[32];
  int sequence;
  double time;
} ControllerCameraFrameMetadata;

typedef struct {
  int available;
  int resolution;
  double fov;
  double max_range;
  ControllerLidarScanStats stats;
} ControllerPerceptionLidarState;

typedef struct {
  int available;
  int virtual_mode;
  int width;
  int height;
  double fov;
  ControllerCameraFrameMetadata frame;
  int obstacle_visible;
  double obstacle_score;
  double obstacle_center_offset;
  double obstacle_angle;
  double obstacle_range;
  int detection_count;
  int obstacle_update_step;
} ControllerPerceptionCameraState;

typedef struct {
  ControllerPerceptionLidarState lidar;
  ControllerPerceptionCameraState camera;
  ObstacleTracePoint trace[CONTROLLER_PERCEPTION_MAX_TRACE_POINTS];
  int trace_count;
} ControllerPerceptionRuntime;

typedef struct {
  int lidar_available;
  int lidar_resolution;
  double lidar_fov;
  double lidar_max_range;
  int camera_available;
  int camera_virtual_mode;
  int camera_width;
  int camera_height;
  double camera_fov;
} ControllerPerceptionSensorMetadata;

typedef struct {
  int visible;
  double score;
  double center_offset;
  double angle;
  double range;
  int detection_count;
  int update_step;
} ControllerPerceptionCameraObservation;

typedef struct {
  const ControllerPerceptionLidarState *lidar;
  const ControllerPerceptionCameraState *camera;
  const ObstacleTracePoint *trace;
  int trace_count;
} ControllerPerceptionFrame;

typedef struct {
  int requested;
  int virtual_mode;
  int width;
  int height;
} ControllerCameraPublicationRequest;

void controller_perception_runtime_init(ControllerPerceptionRuntime *runtime);
void controller_perception_runtime_configure(
    ControllerPerceptionRuntime *runtime,
    const ControllerPerceptionSensorMetadata *metadata);
void controller_perception_runtime_reset_camera_observation(
    ControllerPerceptionRuntime *runtime, int update_step);
void controller_perception_runtime_update_camera(
    ControllerPerceptionRuntime *runtime,
    const ControllerPerceptionCameraObservation *observation);
void controller_perception_runtime_capture_lidar(
    ControllerPerceptionRuntime *runtime,
    const ControllerLidarScanConfig *config,
    const float *ranges,
    double robot_x,
    double robot_y,
    double heading,
    double now_time,
    double trace_ttl_seconds);
ControllerPerceptionFrame controller_perception_runtime_frame(
    const ControllerPerceptionRuntime *runtime);
ControllerCameraPublicationRequest controller_perception_runtime_camera_publication_request(
    const ControllerPerceptionRuntime *runtime, int due);

#endif
