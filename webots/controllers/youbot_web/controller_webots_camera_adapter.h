#ifndef YOUBOT_WEB_CONTROLLER_WEBOTS_CAMERA_ADAPTER_H
#define YOUBOT_WEB_CONTROLLER_WEBOTS_CAMERA_ADAPTER_H

#include "controller_perception_runtime.h"

typedef ControllerCameraFrameMetadata ControllerWebotsCameraFrameMetadata;

int controller_webots_camera_adapter_publish_frame(
    const char *temp_path,
    const char *final_path,
    const char *file_name,
    const char *mime_type,
    double time,
    ControllerWebotsCameraFrameMetadata *metadata);
void controller_webots_camera_adapter_remove_frames(
    const char *bmp_path,
    const char *bmp_temp_path,
    const char *jpeg_path,
    const char *jpeg_temp_path);

#endif
