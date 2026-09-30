#ifndef YOUBOT_WEB_CONTROLLER_INPUT_ORCHESTRATION_H
#define YOUBOT_WEB_CONTROLLER_INPUT_ORCHESTRATION_H

typedef enum {
  CONTROLLER_INPUT_MOTION_RELOAD_UNCHANGED,
  CONTROLLER_INPUT_MOTION_RELOAD_CHANGED,
} ControllerInputMotionReloadResult;

typedef struct {
  ControllerInputMotionReloadResult (*reload_motion)(void *context);
  void (*reload_runtime_command)(void *context);
  void (*reload_limit_zones)(void *context);
  void (*reload_surface_zones)(void *context);
  void (*reload_route)(void *context);
  void (*set_status)(void *context, const char *status);
} ControllerInputOrchestrationOperations;

typedef struct {
  const ControllerInputOrchestrationOperations *operations;
  void *operations_context;
  int zone_reload_interval;
  int route_reload_interval;
  int runtime_command_reload_interval;
} ControllerInputOrchestration;

void controller_input_orchestration_init(
    ControllerInputOrchestration *input,
    const ControllerInputOrchestrationOperations *operations,
    void *operations_context,
    int zone_reload_interval,
    int route_reload_interval,
    int runtime_command_reload_interval);
void controller_input_orchestration_reload_motion(
    ControllerInputOrchestration *input,
    int step_counter);
void controller_input_orchestration_reload_runtime_command(
    ControllerInputOrchestration *input,
    int step_counter);
void controller_input_orchestration_reload_zones(
    ControllerInputOrchestration *input,
    int step_counter);
void controller_input_orchestration_reload_surface_zones(
    ControllerInputOrchestration *input,
    int step_counter);
void controller_input_orchestration_reload_route(
    ControllerInputOrchestration *input,
    int step_counter);

#endif
