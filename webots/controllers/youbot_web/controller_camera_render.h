#ifndef YOUBOT_WEB_CONTROLLER_CAMERA_RENDER_H
#define YOUBOT_WEB_CONTROLLER_CAMERA_RENDER_H

void controller_camera_render_set_pixel(
    unsigned char *pixels, int width, int height, int x, int y, int r, int g, int b);
void controller_camera_render_rect(
    unsigned char *pixels, int width, int height,
    int min_x, int min_y, int max_x, int max_y, int r, int g, int b);
void controller_camera_render_line(
    unsigned char *pixels, int width, int height,
    int x0, int y0, int x1, int y1, int r, int g, int b);
void controller_camera_render_background(unsigned char *pixels, int width, int height);
void controller_camera_render_reticle(unsigned char *pixels, int width, int height);
void controller_camera_render_box(
    unsigned char *pixels, int width, int height,
    int center_x, int bottom_y, int box_width, int box_height, double danger);

#endif
