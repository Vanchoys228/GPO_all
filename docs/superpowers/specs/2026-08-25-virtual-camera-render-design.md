# Virtual Camera Render Design

Extract pure RGB raster operations from `youbot_web.c` into
`controller_camera_render.c/.h`: clamped pixel writes, clipped rectangles,
Bresenham lines, the existing perspective background, and obstacle boxes.
Dimensions are explicit. Lidar-derived scene construction, route overlay inputs,
Webots state, and BMP/JPEG file I/O remain in the controller. All formulas and
draw order stay unchanged for the existing 320x180 frame.
