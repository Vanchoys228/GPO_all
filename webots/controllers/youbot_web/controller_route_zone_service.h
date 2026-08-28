#ifndef YOUBOT_WEB_CONTROLLER_ROUTE_ZONE_SERVICE_H
#define YOUBOT_WEB_CONTROLLER_ROUTE_ZONE_SERVICE_H

#include "controller_types.h"

typedef enum {
  CONTROLLER_ROUTE_ZONE_STATUS_NOT_CHECKED = 0,
  CONTROLLER_ROUTE_ZONE_STATUS_UNCHANGED = 1,
  CONTROLLER_ROUTE_ZONE_STATUS_OK = 2,
  CONTROLLER_ROUTE_ZONE_STATUS_ROUTE_CANNOT_OPEN = 3,
  CONTROLLER_ROUTE_ZONE_STATUS_ROUTE_EMPTY = 4,
  CONTROLLER_ROUTE_ZONE_STATUS_ZONE_INVALID_HEADER = 5,
  CONTROLLER_ROUTE_ZONE_STATUS_ZONE_INVALID_ENTRY = 6,
  CONTROLLER_ROUTE_ZONE_STATUS_ZONE_UNEXPECTED_END = 7,
  CONTROLLER_ROUTE_ZONE_STATUS_ZONE_INVALID_POINT = 8,
  CONTROLLER_ROUTE_ZONE_STATUS_INVALID_ARGUMENT = 9,
} ControllerRouteZoneStatus;

typedef struct {
  const char *route_path;
  const char *limit_zones_path;
  const char *surface_zones_path;
} ControllerRouteZoneServicePaths;

typedef struct {
  long long route_last_checked;
  long long limit_zones_last_checked;
  long long surface_zones_last_checked;
} ControllerRouteZoneService;

typedef struct {
  int reload_route;
  int reload_limit_zones;
  int reload_surface_zones;
} ControllerRouteZoneServiceReloadRequest;

typedef struct {
  ControllerRouteZoneStatus route_status;
  ControllerRouteZoneStatus limit_zones_status;
  ControllerRouteZoneStatus surface_zones_status;
  int route_changed;
  int limit_zones_changed;
  int surface_zones_changed;
  RouteData route;
  ZoneData limit_zones;
  SurfaceZoneData surface_zones;
} ControllerRouteZoneServiceResult;

void controller_route_zone_service_init(ControllerRouteZoneService *service);
ControllerRouteZoneServiceResult controller_route_zone_service_reload(
    ControllerRouteZoneService *service,
    const ControllerRouteZoneServicePaths *paths,
    const ControllerRouteZoneServiceReloadRequest *request,
    const RouteData *current_route,
    const ZoneData *current_limit_zones,
    const SurfaceZoneData *current_surface_zones);

#endif
