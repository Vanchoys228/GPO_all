#ifndef YOUBOT_WEB_CONTROLLER_INPUT_RUNTIME_H
#define YOUBOT_WEB_CONTROLLER_INPUT_RUNTIME_H

#include "controller_input_orchestration.h"
#include "controller_types.h"

void set_error(const char *message);
void clear_error(void);
void set_status(const char *status);

void spawn_runtime_obstacle(const RuntimeCommand *command);
ControllerInputMotionReloadResult reload_motion_profile_input(void *context);
void reload_runtime_command_input(void *context);
void reload_limit_zones_input(void *context);
void reload_surface_zones_input(void *context);
void reload_route_input(void *context);
void set_input_status(void *context, const char *status);
void maybe_reload_runtime_command(void);
void maybe_reload_motion_profile(void);
void maybe_reload_zones(void);
void maybe_reload_surface_zones(void);
int load_route(RouteData *route);
void maybe_reload_route(void);

#endif
