#ifndef YOUBOT_WEB_CONTROLLER_WEBOTS_POSE_H
#define YOUBOT_WEB_CONTROLLER_WEBOTS_POSE_H

#include <webots/supervisor.h>

typedef struct {
  WbNodeRef self_node;
  WbFieldRef translation_field;
  WbFieldRef rotation_field;
  WbFieldRef root_children_field;
} ControllerWebotsPose;

void controller_webots_pose_init(ControllerWebotsPose *pose);
int controller_webots_pose_is_ready(const ControllerWebotsPose *pose);
int controller_webots_pose_reset(
    ControllerWebotsPose *pose, double x, double y, double height, double heading);
int controller_webots_pose_read(
    const ControllerWebotsPose *pose, double *x, double *y, double *heading);

#endif
