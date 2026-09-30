#ifndef YOUBOT_WEB_CONTROLLER_CONTROL_CONFIG_H
#define YOUBOT_WEB_CONTROLLER_CONTROL_CONFIG_H

#include "controller_avoidance.h"
#include "controller_avoidance_lifecycle.h"
#include "controller_avoidance_start.h"
#include "controller_lifecycle.h"
#include "controller_navigation_lidar.h"
#include "controller_navigation_perception.h"
#include "controller_navigation_tracking.h"
#include "controller_runtime.h"

typedef struct {
  double wheel_radius;
  double wheel_base_longitudinal;
  double wheel_base_lateral;
  double max_wheel_speed_rad_s;
  double acceleration_limit_rad_s2;
  double deceleration_limit_rad_s2;
} ControllerDriveSettings;

typedef struct {
  ControllerRuntimeNavigationConfig runtime;
  ControllerNavigationTrackingConfig tracking;
  ControllerNavigationLidarConfig lidar;
} ControllerNavigationConfig;

typedef struct {
  ControllerNavigationPerceptionConfig navigation;
} ControllerPerceptionConfig;

typedef struct {
  double grid_cell_m;
  int max_boundary_points;
  double contour_offset_m;
  double interior_offset_m;
  double strip_m;
  double min_contour_step_m;
  double max_contour_step_m;
  double min_strip_length_m;
  double rdp_epsilon_m;
  double map_obstacle_clearance_m;
  double max_extent_x_m;
  double max_extent_y_m;
  double obstacle_scan_radius_m;
  int obstacle_scan_points;
  int obstacle_scan_cooldown_steps;
  double obstacle_scan_min_repeat_distance_m;
  int avoidance_replan_steps;
  int avoidance_no_progress_steps;
  int avoidance_max_steps;
  double avoidance_loop_radius_m;
  double avoidance_progress_epsilon_m;
  double avoidance_orbit_heading_rad;
  int replan_cooldown_steps;
  int escape_scan_ahead;
  double escape_min_target_distance_m;
  double escape_obstacle_clearance_m;
  double escape_segment_clearance_m;
} ControllerMappingConfig;

typedef struct {
  ControllerAvoidanceStartConfig start;
  ControllerAvoidanceProgressConfig progress;
  ControllerAvoidanceLifecycleConfig lifecycle;
  ControllerAvoidanceCommandConfig command;
} ControllerAvoidanceConfig;

typedef struct {
  ControllerLifecycleScheduleConfig lifecycle;
} ControllerScheduleConfig;

typedef struct {
  int time_step_ms;
  ControllerDriveSettings drive;
  ControllerNavigationConfig navigation;
  ControllerPerceptionConfig perception;
  ControllerMappingConfig mapping;
  ControllerAvoidanceConfig avoidance;
  ControllerScheduleConfig schedule;
} ControllerControlConfig;

ControllerControlConfig controller_control_config_default(void);

#endif
