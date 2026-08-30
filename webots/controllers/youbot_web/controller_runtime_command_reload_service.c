#include "controller_runtime_command_reload_service.h"

#include "controller_io.h"

#include <string.h>

void controller_runtime_command_reload_service_init(
    ControllerRuntimeCommandReloadService *service,
    const char *command_path,
    const char *route_path,
    ControllerRuntimeCommandLimits limits,
    ControllerRuntime *runtime,
    const ControllerSurveyIntegrationOps *survey_ops,
    void (*spawn_obstacle)(const RuntimeCommand *command)) {
  if (!service) return;
  memset(service, 0, sizeof(*service));
  service->command_path = command_path;
  service->route_path = route_path;
  service->limits = limits;
  service->runtime = runtime;
  if (survey_ops) service->survey_ops = *survey_ops;
  service->spawn_obstacle = spawn_obstacle;
  service->last_modified = -1;
  service->last_processed_id = -1;
}

ControllerRuntimeCommandReloadResult controller_runtime_command_reload_service_run(
    ControllerRuntimeCommandReloadService *service,
    int step_counter,
    int reload_interval) {
  if (!service || !service->runtime || !service->command_path || reload_interval <= 0 ||
      (step_counter % reload_interval) != 0) {
    return CONTROLLER_RUNTIME_COMMAND_RELOAD_NOT_DUE;
  }

  const long long mtime = get_file_mtime(service->command_path);
  if (mtime < 0) return CONTROLLER_RUNTIME_COMMAND_RELOAD_MISSING;
  if (mtime == service->last_modified) return CONTROLLER_RUNTIME_COMMAND_RELOAD_UNCHANGED;

  RuntimeCommand command = {0};
  if (!controller_runtime_command_load_file(service->command_path, &service->limits, &command)) {
    service->last_modified = mtime;
    return CONTROLLER_RUNTIME_COMMAND_RELOAD_INVALID;
  }

  service->last_modified = mtime;
  if (command.id <= service->last_processed_id) {
    return CONTROLLER_RUNTIME_COMMAND_RELOAD_DUPLICATE;
  }
  service->last_processed_id = command.id;

  if (command.has_start_mapping_survey) {
    controller_survey_integration_start(
        service->route_path,
        &command,
        &service->runtime->route,
        &service->runtime->current_waypoint_index,
        &service->runtime->route_finished,
        &service->runtime->mapping_survey,
        &service->survey_ops);
    return CONTROLLER_RUNTIME_COMMAND_RELOAD_APPLIED;
  }

  if (service->spawn_obstacle) service->spawn_obstacle(&command);
  if (service->survey_ops.clear_error) service->survey_ops.clear_error();
  if (service->survey_ops.set_status) service->survey_ops.set_status("runtime_obstacle_spawned");
  return CONTROLLER_RUNTIME_COMMAND_RELOAD_APPLIED;
}
