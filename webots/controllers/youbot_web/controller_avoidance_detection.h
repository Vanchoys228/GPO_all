#ifndef YOUBOT_WEB_CONTROLLER_AVOIDANCE_DETECTION_H
#define YOUBOT_WEB_CONTROLLER_AVOIDANCE_DETECTION_H

#include "controller_avoidance.h"

void controller_avoidance_detect(
    const LidarObstacleContext *context,
    int lidar_available,
    int camera_visual_front_obstacle,
    const ControllerAvoidanceDetectionConfig *config,
    ControllerAvoidanceDetection *detection);

#endif
