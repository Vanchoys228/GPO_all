# Camera Map Geometry Design

## Goal

Extract pure world-coordinate calculations for camera obstacle and free-space
observations from `youbot_web.c` without changing map storage or runtime behavior.

## Boundary

`controller_camera.c/.h` receives an explicit robot pose, sensor offset, relative
angle, range, and geometry thresholds. It returns either one obstacle point or a
bounded sequence of free-ray points. The module has no Webots, global-state,
mapping-array, confidence, dirty-flag, allocation, or file-I/O dependency.

`youbot_web.c` continues to read pose, provide the current constants, append
returned points to existing map arrays, merge confidence, and serialize JSON/CSV.

## Preserved behavior

The extraction preserves `world_angle = heading - relative_angle`, the existing
sensor-origin transform, finite/range rejection, free-ray margin and clamp,
`floor(usable_range / step)`, iteration beginning at step 2, and the near-robot
filter. The adapter uses a fixed local output buffer larger than the maximum
number of points allowed by the existing range and step constants.

## Testing and success criteria

Standalone tests cover forward and rotated poses, sensor offsets, invalid
obstacle ranges, free-ray point order/count, margin/clamping, and invalid inputs.
All existing project checks must remain green, and Docker remains untouched.
