#include "controller_webots_simulation.h"

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
