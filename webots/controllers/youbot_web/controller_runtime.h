#ifndef YOUBOT_WEB_CONTROLLER_RUNTIME_H
#define YOUBOT_WEB_CONTROLLER_RUNTIME_H

#include "controller_avoidance.h"
#include "controller_navigation_context.h"
#include "controller_survey_state.h"
#include "controller_types.h"

typedef struct {
  RouteData route;
  ZoneData limit_zones;
  SurfaceZoneData surface_zones;
  int current_waypoint_index;
  int route_finished;
  NavigationMode navigation_mode;
  int navigation_waypoint_index;
  double navigation_segment_start_x;
  double navigation_segment_start_z;
  double distance_to_target;
  ControllerNavigationPoseHistory navigation_pose_history;
  ControllerAvoidanceState avoidance;
  double lidar_priority_turn_sign;
  int lidar_priority_hold_steps;
  ControllerMappingSurveyState mapping_survey;
} ControllerRuntime;

typedef struct {
  double x;
  double z;
  double heading;
  int manual_relocation_detected;
} ControllerRuntimeSensorFrame;

typedef enum {
  CONTROLLER_RUNTIME_MOTION_STOP = 0,
  CONTROLLER_RUNTIME_MOTION_CALCULATE = 1,
} ControllerRuntimeMotionKind;

typedef struct {
  ControllerRuntimeMotionKind command;
} ControllerRuntimeMotionCommand;

typedef enum {
  CONTROLLER_RUNTIME_FRAME_WAIT_FOR_ROUTE = 0,
  CONTROLLER_RUNTIME_FRAME_STOP_FINISHED = 1,
  CONTROLLER_RUNTIME_FRAME_READY = 2,
} ControllerRuntimeFrameDecision;

typedef struct {
  ControllerRuntimeMotionCommand motion;
} ControllerRuntimeFrameResult;

void controller_runtime_init(ControllerRuntime *runtime);
ControllerRuntimeFrameDecision controller_runtime_process_sensor_frame(
    ControllerRuntime *runtime,
    const ControllerRuntimeSensorFrame *frame,
    ControllerRuntimeFrameResult *result);
ControllerRuntimeMotionCommand controller_runtime_calculate_motion_command(
    const ControllerRuntime *runtime);

#endif
