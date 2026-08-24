#ifndef YOUBOT_WEB_CONTROLLER_RUNTIME_COMMAND_H
#define YOUBOT_WEB_CONTROLLER_RUNTIME_COMMAND_H

#include "controller_types.h"

typedef struct {
  double default_min_x;
  double default_max_x;
  double default_min_y;
  double default_max_y;
  double max_extent_x;
  double max_extent_y;
} ControllerRuntimeCommandLimits;

int controller_runtime_command_load_file(
    const char *path,
    const ControllerRuntimeCommandLimits *limits,
    RuntimeCommand *command);

#endif
