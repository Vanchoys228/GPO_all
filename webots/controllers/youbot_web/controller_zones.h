#ifndef YOUBOT_WEB_CONTROLLER_ZONES_H
#define YOUBOT_WEB_CONTROLLER_ZONES_H

#include "controller_types.h"

typedef enum {
  CONTROLLER_ZONE_LOAD_OK = 0,
  CONTROLLER_ZONE_LOAD_INVALID_HEADER = 1,
  CONTROLLER_ZONE_LOAD_INVALID_ENTRY = 2,
  CONTROLLER_ZONE_LOAD_UNEXPECTED_END = 3,
  CONTROLLER_ZONE_LOAD_INVALID_POINT = 4,
  CONTROLLER_ZONE_LOAD_NO_DATA = 5,
} ControllerZoneLoadResult;

ControllerZoneLoadResult controller_limit_zones_load_file(
    const char *path,
    ZoneData *zones);
ControllerZoneLoadResult controller_surface_zones_load_file(
    const char *path,
    SurfaceZoneData *zones);
int controller_zone_data_equal(const ZoneData *left, const ZoneData *right);
int controller_surface_zone_data_equal(
    const SurfaceZoneData *left,
    const SurfaceZoneData *right);

#endif
