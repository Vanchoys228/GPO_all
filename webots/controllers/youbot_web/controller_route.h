#ifndef YOUBOT_WEB_CONTROLLER_ROUTE_H
#define YOUBOT_WEB_CONTROLLER_ROUTE_H

#include "controller_types.h"

typedef enum {
  CONTROLLER_ROUTE_LOAD_OK = 0,
  CONTROLLER_ROUTE_LOAD_CANNOT_OPEN = 1,
  CONTROLLER_ROUTE_LOAD_EMPTY = 2,
} ControllerRouteLoadResult;

ControllerRouteLoadResult controller_route_load_file(
    const char *path,
    RouteData *route);

#endif
