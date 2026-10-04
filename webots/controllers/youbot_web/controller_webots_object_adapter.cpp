#include "controller_webots_object_adapter.h"

#include <cmath>
#include <cstring>

namespace {
constexpr double kMaximumPositionStep = 0.006;

double approach(double current, double target) {
  const double delta = target - current;
  if (std::fabs(delta) <= kMaximumPositionStep) return target;
  return current + std::copysign(kMaximumPositionStep, delta);
}
}

int controller_webots_object_adapter_init(
    ControllerWebotsObjectAdapter *adapter,
    const char *def_name) {
  if (!adapter || !def_name) return 0;
  std::memset(adapter, 0, sizeof(*adapter));
  adapter->node = wb_supervisor_node_get_from_def(def_name);
  if (!adapter->node) return 0;
  adapter->translation = wb_supervisor_node_get_field(adapter->node, "translation");
  adapter->rotation = wb_supervisor_node_get_field(adapter->node, "rotation");
  return adapter->translation != 0 && adapter->rotation != 0;
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
  if (adapter) {
    adapter->attached = 1;
    adapter->on_platform = 0;
    adapter->pose_initialized = 0;
  }
}

void controller_webots_object_adapter_store_on_platform(
    ControllerWebotsObjectAdapter *adapter) {
  if (adapter) {
    adapter->attached = 1;
    adapter->on_platform = 1;
  }
}

void controller_webots_object_adapter_take_from_platform(
    ControllerWebotsObjectAdapter *adapter) {
  if (adapter) {
    adapter->attached = 1;
    adapter->on_platform = 0;
  }
}

void controller_webots_object_adapter_update(
    ControllerWebotsObjectAdapter *adapter,
    double robot_x,
    double robot_y,
    double heading,
    double forward_offset,
    double height,
    double platform_offset,
    double platform_height) {
  if (!adapter || !adapter->attached || !adapter->translation) return;
  const double offset = adapter->on_platform ? platform_offset : forward_offset;
  const double target[3] = {
      robot_x + std::cos(heading) * offset,
      robot_y + std::sin(heading) * offset,
      adapter->on_platform ? platform_height : height,
  };
  if (!adapter->pose_initialized) {
    const double *current = wb_supervisor_field_get_sf_vec3f(adapter->translation);
    if (!current) return;
    std::memcpy(adapter->position, current, sizeof(adapter->position));
    adapter->pose_initialized = 1;
  }
  for (int axis = 0; axis < 3; ++axis)
    adapter->position[axis] = approach(adapter->position[axis], target[axis]);

  const double rotation[4] = {0.0, 0.0, 1.0, heading};
  const double velocity[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  wb_supervisor_field_set_sf_vec3f(adapter->translation, adapter->position);
  wb_supervisor_field_set_sf_rotation(adapter->rotation, rotation);
  wb_supervisor_node_set_velocity(adapter->node, velocity);
}

void controller_webots_object_adapter_detach(ControllerWebotsObjectAdapter *adapter) {
  if (!adapter || !adapter->node) return;
  adapter->attached = 0;
  adapter->on_platform = 0;
  adapter->pose_initialized = 0;
  wb_supervisor_node_reset_physics(adapter->node);
}
