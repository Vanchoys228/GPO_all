#ifndef YOUBOT_WEB_CONTROLLER_AVOIDANCE_START_H
#define YOUBOT_WEB_CONTROLLER_AVOIDANCE_START_H

#include "controller_avoidance.h"

typedef struct {
  double switch_margin;
  int initial_hold_steps;
  int priority_hold_steps;
  ControllerAvoidanceDetourConfig detour;
} ControllerAvoidanceStartConfig;

typedef struct {
  const LidarObstacleContext *lidar_context;
  const ControllerAvoidanceDetection *detection;
  double x;
  double z;
  double heading;
  double target_distance;
  double priority_turn_sign;
  double camera_turn_sign;
  double heading_error;
} ControllerAvoidanceStartInput;

typedef struct {
  double priority_turn_sign;
  int priority_hold_steps;
} ControllerAvoidanceStartOutput;

int controller_avoidance_start(
    ControllerAvoidanceState *state,
    const ControllerAvoidanceStartInput *input,
    const ControllerAvoidanceStartConfig *config,
    ControllerAvoidanceStartOutput *output);

#endif
