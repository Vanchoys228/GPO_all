#ifndef CONTROLLER_WEBOTS_OBJECT_ADAPTER_H
#define CONTROLLER_WEBOTS_OBJECT_ADAPTER_H

#include <webots/supervisor.h>

typedef struct {
  WbNodeRef node;
  WbFieldRef translation;
  int attached;
  double height;
} ControllerWebotsObjectAdapter;

int controller_webots_object_adapter_init(
    ControllerWebotsObjectAdapter *adapter,
    const char *def_name);
int controller_webots_object_adapter_position(
    const ControllerWebotsObjectAdapter *adapter,
    double *x,
    double *y,
    double *z);
void controller_webots_object_adapter_attach(ControllerWebotsObjectAdapter *adapter);
void controller_webots_object_adapter_update(
    ControllerWebotsObjectAdapter *adapter,
    double robot_x,
    double robot_y,
    double heading,
    double forward_offset,
    double height);
void controller_webots_object_adapter_detach(ControllerWebotsObjectAdapter *adapter);

#endif
