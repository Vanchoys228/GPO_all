#include "controller_webots_object_adapter.h"

#include <cmath>
#include <cstring>

int controller_webots_object_adapter_init(
    ControllerWebotsObjectAdapter *adapter,
    const char *def_name) {
  if (!adapter || !def_name) return 0;
  std::memset(adapter, 0, sizeof(*adapter));
  adapter->node = wb_supervisor_node_get_from_def(def_name);
  if (!adapter->node) return 0;
  adapter->translation = wb_supervisor_node_get_field(adapter->node, "translation");
  return adapter->translation != 0;
}

int controller_webots_object_adapter_position(
    const ControllerWebotsObjectAdapter *adapter,
    double *x,
    double *y,
    double *z) {
  if (!adapter || !adapter->translation) return 0;
  const double *value = wb_supervisor_field_get_sf_vec3f(adapter->translation);
  if (!value) return 0;
  if (x) *x = value[0];
  if (y) *y = value[1];
  if (z) *z = value[2];
  return 1;
}

void controller_webots_object_adapter_attach(ControllerWebotsObjectAdapter *adapter) {
  if (adapter) adapter->attached = 1;
}

void controller_webots_object_adapter_update(
    ControllerWebotsObjectAdapter *adapter,
    double robot_x,
    double robot_y,
    double heading,
    double forward_offset,
    double height) {
  if (!adapter || !adapter->attached || !adapter->translation) return;
  const double value[3] = {
      robot_x + std::cos(heading) * forward_offset,
      robot_y + std::sin(heading) * forward_offset,
      height,
  };
  wb_supervisor_field_set_sf_vec3f(adapter->translation, value);
}

void controller_webots_object_adapter_detach(ControllerWebotsObjectAdapter *adapter) {
  if (!adapter || !adapter->node) return;
  adapter->attached = 0;
  wb_supervisor_node_reset_physics(adapter->node);
}
