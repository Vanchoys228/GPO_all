#ifndef YOUBOT_WEB_CONTROLLER_WEBOTS_ADAPTER_H
#define YOUBOT_WEB_CONTROLLER_WEBOTS_ADAPTER_H

#include "controller_drive.h"
#include "controller_webots_sensors.h"
#include "controller_webots_pose.h"

typedef void (*ControllerWebotsAdapterDriveFn)(
    void *context,
    const ControllerDriveConfig *config,
    double vx,
    double vy,
    double omega);

typedef struct {
  ControllerDriveConfig drive_config;
  ControllerWebotsAdapterDriveFn drive;
  void *context;
} ControllerWebotsAdapter;

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
} ControllerWebotsAdapterSensorState;

ControllerDriveConfig controller_webots_adapter_drive_config(
    double wheel_radius,
    double wheel_base_longitudinal,
    double wheel_base_lateral,
    double max_wheel_speed_rad_s,
    double acceleration_limit_rad_s2,
    double deceleration_limit_rad_s2,
    double time_step_seconds);
void controller_webots_adapter_init(
    ControllerWebotsAdapter *adapter,
    const ControllerDriveConfig *config,
    ControllerWebotsAdapterDriveFn drive,
    void *context);
void controller_webots_adapter_apply_velocity(
    ControllerWebotsAdapter *adapter, double vx, double vy, double omega);
void controller_webots_adapter_stop(ControllerWebotsAdapter *adapter);
ControllerWebotsAdapterSensorState controller_webots_adapter_init_sensors(
    ControllerWebotsSensors *sensors,
    int time_step,
    int camera_capture_interval,
    int virtual_camera_width,
    int virtual_camera_height,
    double fallback_camera_fov);
void controller_webots_adapter_init_pose(ControllerWebotsPose *pose);
int controller_webots_adapter_reset_pose(
    ControllerWebotsPose *pose, double x, double y, double height, double heading);
int controller_webots_adapter_read_pose(
    const ControllerWebotsPose *pose, double *x, double *y, double *heading);

#endif
