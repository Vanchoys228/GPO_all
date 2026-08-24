#include "controller_route.h"

#include "controller_io.h"

#include <stdio.h>
#include <string.h>

#define CONTROLLER_ROUTE_PI 3.14159265358979323846

ControllerRouteLoadResult controller_route_load_file(
    const char *path,
    RouteData *route) {
  if (!path || !route) return CONTROLLER_ROUTE_LOAD_CANNOT_OPEN;

  FILE *file = fopen(path, "r");
  if (!file) return CONTROLLER_ROUTE_LOAD_CANNOT_OPEN;

  char line[256];
  int count = 0;
  while (fgets(line, sizeof(line), file)) {
    double x = 0.0;
    double z = 0.0;
    double heading_deg = 0.0;

    if (line[0] == '\n' || line[0] == '\r' || line[0] == '#') continue;
    if ((line[0] >= 'A' && line[0] <= 'z') ||
        strncmp(line, "x,", 2) == 0 ||
        strncmp(line, "x ", 2) == 0) {
      continue;
    }

    if (sscanf(line, " %lf , %lf , %lf", &x, &z, &heading_deg) == 3) {
      if (count < MAX_WAYPOINTS) {
        route->waypoints[count].x = x;
        route->waypoints[count].z = z;
        route->waypoints[count].heading_rad =
            heading_deg * CONTROLLER_ROUTE_PI / 180.0;
        route->waypoints[count].has_heading = 1;
        ++count;
      }
      continue;
    }

    if (sscanf(line, " %lf , %lf", &x, &z) == 2 && count < MAX_WAYPOINTS) {
      route->waypoints[count].x = x;
      route->waypoints[count].z = z;
      route->waypoints[count].heading_rad = 0.0;
      route->waypoints[count].has_heading = 0;
      ++count;
    }
  }

  fclose(file);
  if (count == 0) return CONTROLLER_ROUTE_LOAD_EMPTY;

  route->count = count;
  route->last_modified = get_file_mtime(path);
  return CONTROLLER_ROUTE_LOAD_OK;
}
