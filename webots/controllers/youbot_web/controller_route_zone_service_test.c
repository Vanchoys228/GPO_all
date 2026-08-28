#include "controller_route_zone_service.h"

#include <stdio.h>
#include <string.h>

static int write_text_file(const char *path, const char *contents) {
  FILE *file = fopen(path, "w");
  if (!file) return 0;
  fputs(contents, file);
  return fclose(file) == 0;
}

int main(void) {
  const char *route_path = "controller_route_zone_service_route.csv";
  const char *limit_path = "controller_route_zone_service_limit_zones.txt";
  const char *surface_path = "controller_route_zone_service_surface_zones.txt";
  const ControllerRouteZoneServicePaths paths = {
      route_path,
      limit_path,
      surface_path,
  };
  const ControllerRouteZoneServiceReloadRequest request = {1, 1, 1};
  ControllerRouteZoneService service;
  RouteData route = {0};
  ZoneData limit_zones = {0};
  SurfaceZoneData surface_zones = {0};

  if (!write_text_file(route_path, "x,z\n1.0,2.0\n")) return 1;
  if (!write_text_file(limit_path, "zone_count 1\nzone 3\n0 0\n1 0\n0 1\n")) return 2;
  if (!write_text_file(surface_path,
                       "surface_zone_count 1\nsurface_zone 3 grass north\n0 0\n1 0\n0 1\n")) {
    return 3;
  }

  controller_route_zone_service_init(&service);
  const ControllerRouteZoneServiceResult result =
      controller_route_zone_service_reload(
          &service, &paths, &request, &route, &limit_zones, &surface_zones);

  remove(route_path);
  remove(limit_path);
  remove(surface_path);

  if (result.route_status != CONTROLLER_ROUTE_ZONE_STATUS_OK ||
      result.limit_zones_status != CONTROLLER_ROUTE_ZONE_STATUS_OK ||
      result.surface_zones_status != CONTROLLER_ROUTE_ZONE_STATUS_OK) {
    return 4;
  }
  if (!result.route_changed || !result.limit_zones_changed ||
      !result.surface_zones_changed) {
    return 5;
  }
  if (result.route.count != 1 || result.limit_zones.count != 1 ||
      result.surface_zones.count != 1 ||
      strcmp(result.surface_zones.zones[0].surface_key, "grass") != 0) {
    return 6;
  }
  return 0;
}
