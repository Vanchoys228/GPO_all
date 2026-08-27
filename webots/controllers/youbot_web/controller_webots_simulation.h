#ifndef YOUBOT_WEB_CONTROLLER_WEBOTS_SIMULATION_H
#define YOUBOT_WEB_CONTROLLER_WEBOTS_SIMULATION_H

#include <stddef.h>

#include "controller_types.h"

#include <webots/supervisor.h>

#define CONTROLLER_WEBOTS_SIMULATION_MAX_NODE_REGISTRY 512

typedef struct {
  char defs[CONTROLLER_WEBOTS_SIMULATION_MAX_NODE_REGISTRY][64];
  int count;
} ControllerWebotsSimulationNodeRegistry;

int controller_webots_simulation_registry_track(
    ControllerWebotsSimulationNodeRegistry *registry,
    int capacity,
    const char *def_name);
void controller_webots_simulation_registry_forget(
    ControllerWebotsSimulationNodeRegistry *registry,
    int index);
void controller_webots_simulation_registry_remove_at(
    ControllerWebotsSimulationNodeRegistry *registry,
    int index);
void controller_webots_simulation_registry_remove_all(
    ControllerWebotsSimulationNodeRegistry *registry);
void controller_webots_simulation_sync_limit_zones(
    WbFieldRef root_children_field,
    ControllerWebotsSimulationNodeRegistry *registry,
    const ZoneData *zones,
    int capacity,
    double wall_thickness,
    double wall_height);
void controller_webots_simulation_sync_surface_zones(
    WbFieldRef root_children_field,
    ControllerWebotsSimulationNodeRegistry *registry,
    const SurfaceZoneData *zones,
    int capacity);
void controller_webots_simulation_spawn_runtime_obstacle(
    WbFieldRef root_children_field,
    ControllerWebotsSimulationNodeRegistry *registry,
    const RuntimeCommand *command,
    int capacity,
    double min_x,
    double max_x,
    double min_y,
    double max_y);

int controller_webots_simulation_format_limit_wall(
    char *buffer,
    size_t buffer_size,
    const char *def_name,
    double ax,
    double ay,
    double bx,
    double by,
    double wall_thickness,
    double wall_height);
int controller_webots_simulation_format_surface_zone(
    char *buffer,
    size_t buffer_size,
    const SurfaceZone *zone,
    int zone_index);
int controller_webots_simulation_format_runtime_obstacle(
    char *buffer,
    size_t buffer_size,
    const RuntimeCommand *command,
    double min_x,
    double max_x,
    double min_y,
    double max_y);

#endif
