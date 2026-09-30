#ifndef YOUBOT_WEB_CONTROLLER_WEBOTS_ZONE_SYNC_H
#define YOUBOT_WEB_CONTROLLER_WEBOTS_ZONE_SYNC_H

#include "controller_types.h"
#include "controller_webots_simulation.h"

typedef struct {
  WbFieldRef root_children_field;
  ControllerWebotsSimulationNodeRegistry *limit_zone_registry;
  ControllerWebotsSimulationNodeRegistry *surface_zone_registry;
  ControllerWebotsSimulationNodeRegistry *runtime_obstacle_registry;
  int max_limit_zone_nodes;
  int max_surface_zone_nodes;
  int max_runtime_obstacle_nodes;
  double wall_thickness;
  double wall_height;
  double min_x;
  double max_x;
  double min_y;
  double max_y;
} ControllerWebotsZoneSyncContext;

void controller_webots_zone_sync_limit_zones(
    const ControllerWebotsZoneSyncContext *context, const ZoneData *zones);
void controller_webots_zone_sync_surface_zones(
    const ControllerWebotsZoneSyncContext *context, const SurfaceZoneData *zones);
void controller_webots_zone_sync_spawn_obstacle(
    const ControllerWebotsZoneSyncContext *context, const RuntimeCommand *command);
void controller_webots_zone_sync_remove_all(const ControllerWebotsZoneSyncContext *context);

#endif
