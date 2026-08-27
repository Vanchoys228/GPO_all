#include <assert.h>
#include <stddef.h>
#include <string.h>

#include "controller_types.h"

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

int main(void) {
  char node_string[1024] = {0};
  const int written = controller_webots_simulation_format_limit_wall(
      node_string,
      sizeof(node_string),
      "WEB_LIMIT_2_3",
      1.0,
      2.0,
      4.0,
      6.0,
      0.08,
      0.45);

  assert(written > 0);
  assert(strstr(node_string, "DEF WEB_LIMIT_2_3 Solid {") != NULL);
  assert(strstr(node_string, "translation 2.500000 4.000000 0.225000") != NULL);
  assert(strstr(node_string, "rotation 0 0 1 0.927295") != NULL);
  assert(strstr(node_string, "name \"dynamic_zone_wall\"") != NULL);
  assert(strstr(node_string, "baseColor 0.1176 0.4510 0.9725") != NULL);
  assert(strstr(node_string, "geometry Box { size 5.000000 0.080000 0.450000 }") != NULL);
  assert(strstr(node_string, "boundingObject Box { size 5.000000 0.080000 0.450000 }") != NULL);
  assert(strstr(node_string, "locked TRUE") != NULL);

  SurfaceZone surface = {0};
  strcpy(surface.surface_key, "rough");
  surface.point_count = 3;
  surface.points[0].x = 0.0;
  surface.points[0].y = 0.0;
  surface.points[1].x = 1.0;
  surface.points[1].y = 0.0;
  surface.points[2].x = 0.0;
  surface.points[2].y = 1.0;
  memset(node_string, 0, sizeof(node_string));
  const int surface_written = controller_webots_simulation_format_surface_zone(
      node_string, sizeof(node_string), &surface, 4);

  assert(surface_written > 0);
  assert(strstr(node_string, "DEF WEB_SURFACE_4 Pose {") != NULL);
  assert(strstr(node_string, "baseColor 0.9300 0.5400 0.0800") != NULL);
  assert(strstr(node_string, "point [ 0.000000 0.000000 0.016000, 1.000000 0.000000 0.016000, 0.000000 1.000000 0.016000,  ]") != NULL);
  assert(strstr(node_string, "coordIndex [ 0 1 2 -1 ]") != NULL);
  return 0;
}
