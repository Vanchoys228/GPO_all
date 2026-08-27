#ifndef YOUBOT_WEB_CONTROLLER_NAVIGATION_TRACKING_H
#define YOUBOT_WEB_CONTROLLER_NAVIGATION_TRACKING_H

#include "controller_types.h"

typedef enum {
  CONTROLLER_NAVIGATION_TRACKING_FINAL_ALIGN = 0,
  CONTROLLER_NAVIGATION_TRACKING_TURN = 1,
  CONTROLLER_NAVIGATION_TRACKING_DRIVE = 2,
} ControllerNavigationTrackingAction;

typedef enum {
  CONTROLLER_NAVIGATION_STATUS_ALIGN_FINAL = 0,
  CONTROLLER_NAVIGATION_STATUS_TURN_PATH = 1,
  CONTROLLER_NAVIGATION_STATUS_TURN_LIDAR = 2,
  CONTROLLER_NAVIGATION_STATUS_TRACK_PATH = 3,
  CONTROLLER_NAVIGATION_STATUS_TRACK_LIDAR = 4,
  CONTROLLER_NAVIGATION_STATUS_TRACK_ZONE = 5,
} ControllerNavigationTrackingStatus;

typedef struct {
  double final_align_distance;
  double heading_tolerance_rad;
  double track_slow_radius;
  double turn_exit_error_rad;
  double track_reenter_turn_rad;
  double turn_enter_error_rad;
  double turn_heading_gain;
  double track_heading_gain;
  double final_align_gain;
  double track_min_linear_speed;
  double min_angular_command;
  double position_tolerance;
} ControllerNavigationTrackingConfig;

typedef struct {
  NavigationMode mode;
  int is_final_waypoint;
  int target_has_heading;
  double target_heading;
  double current_heading;
  double distance_to_target;
  double heading_error_to_target;
  double updated_heading_error;
  int route_relaxed_mode;
  int avoidance_active;
  int lidar_hard_priority;
  double lidar_priority_turn_sign;
  double lidar_heading_bias;
  double lidar_speed_scale;
  double expected_wall_speed_scale;
  int lidar_caution_active;
  int expected_zone_wall_slowdown;
  int expected_zone_wall_ahead;
  double runtime_linear_speed_limit;
  double runtime_angular_speed_limit;
} ControllerNavigationTrackingInput;

typedef struct {
  NavigationMode mode;
  ControllerNavigationTrackingAction action;
  ControllerNavigationTrackingStatus status;
  double linear_speed;
  double angular_speed;
} ControllerNavigationTrackingOutput;

int controller_navigation_tracking_prepare(
    const ControllerNavigationTrackingConfig *config,
    const ControllerNavigationTrackingInput *input,
    ControllerNavigationTrackingOutput *output);
void controller_navigation_tracking_compute(
    const ControllerNavigationTrackingConfig *config,
    const ControllerNavigationTrackingInput *input,
    ControllerNavigationTrackingOutput *output);

#endif
