#ifndef YOUBOT_WEB_CONTROLLER_AVOIDANCE_LIFECYCLE_H
#define YOUBOT_WEB_CONTROLLER_AVOIDANCE_LIFECYCLE_H

#include "controller_avoidance.h"

typedef struct {
  int min_contour_steps;
  double leave_progress;
  double leave_heading_rad;
  double avoid_stop_range;
  double orbit_heading_rad;
  int no_progress_steps;
  int replan_steps;
  int max_steps;
  double loop_radius;
  int free_space_recovery_steps;
  int clear_steps;
} ControllerAvoidanceLifecycleConfig;

typedef struct {
  int mapping_survey;
  int replan_cooldown_steps;
  double x;
  double z;
  double target_distance;
  double heading_error;
  double near_front_range;
  double center_obstacle_range;
} ControllerAvoidanceLifecycleInput;

typedef struct {
  int can_rejoin_route;
  int orbiting_same_object;
  int should_handle_mapping_loop;
  int can_recover_free_space;
} ControllerAvoidanceLifecycleDecision;

void controller_avoidance_lifecycle_evaluate(
    const ControllerAvoidanceState *state,
    const ControllerAvoidanceLifecycleConfig *config,
    const ControllerAvoidanceLifecycleInput *input,
    ControllerAvoidanceLifecycleDecision *decision);
void controller_avoidance_lifecycle_finish(
    ControllerAvoidanceState *state, double start_x, double start_z);

#endif
