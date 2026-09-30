#ifndef YOUBOT_WEB_CONTROLLER_ZONE_GEOMETRY_H
#define YOUBOT_WEB_CONTROLLER_ZONE_GEOMETRY_H

#include "controller_types.h"

int controller_zone_geometry_point_in(double x, double y, const LimitZone *zone);
int controller_zone_geometry_point_near(
    double x, double y, const LimitZone *zone, double clearance);
int controller_zone_geometry_point_near_boundary(
    double x, double y, const LimitZone *zone, double tolerance);
int controller_zone_geometry_segment_blocked(
    const ZoneData *zones,
    double ax,
    double ay,
    double bx,
    double by,
    double clearance,
    int skip_zone_index);
double controller_zone_geometry_signed_area(const LimitZone *zone);
int controller_zone_geometry_find_room(
    const ZoneData *zones, double robot_x, double robot_y);

#endif
