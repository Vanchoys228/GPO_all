#include <assert.h>
#include <stddef.h>
#include <string.h>

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
  return 0;
}
