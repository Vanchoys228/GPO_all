#include "controller_webots_simulation.h"

#include <math.h>
#include <stdio.h>

int controller_webots_simulation_format_limit_wall(
    char *buffer,
    size_t buffer_size,
    const char *def_name,
    double ax,
    double ay,
    double bx,
    double by,
    double wall_thickness,
    double wall_height) {
  if (!buffer || buffer_size == 0 || !def_name) return 0;

  const double dx = bx - ax;
  const double dy = by - ay;
  const double length = hypot(dx, dy);
  if (length <= 0.05) return 0;

  const int written = snprintf(
      buffer,
      buffer_size,
      "DEF %s Solid { "
      "translation %.6f %.6f %.6f "
      "rotation 0 0 1 %.6f "
      "name \"dynamic_zone_wall\" "
      "children [ "
      "Shape { "
      "appearance PBRAppearance { baseColor 0.1176 0.4510 0.9725 roughness 1 metalness 0 transparency 0.45 } "
      "geometry Box { size %.6f %.6f %.6f } "
      "} "
      "] "
      "boundingObject Box { size %.6f %.6f %.6f } "
      "locked TRUE "
      "}",
      def_name,
      (ax + bx) * 0.5,
      (ay + by) * 0.5,
      wall_height * 0.5,
      atan2(dy, dx),
      length,
      wall_thickness,
      wall_height,
      length,
      wall_thickness,
      wall_height);
  return written >= 0 && (size_t)written < buffer_size ? written : 0;
}

void controller_webots_simulation_remove_nodes(char defs[][64], int *count) {
  if (!defs || !count) return;
  for (int i = 0; i < *count; ++i) {
    if (defs[i][0] != '\0') {
      WbNodeRef node = wb_supervisor_node_get_from_def(defs[i]);
      if (node) wb_supervisor_node_remove(node);
    }
    defs[i][0] = '\0';
  }
  *count = 0;
}
