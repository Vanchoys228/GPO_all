# Controller Step Orchestration Design

Create `controller_step.c/.h` to execute one controller step through explicit
callbacks and the lifecycle task mask. Preserve the original order exactly:
zone reloads, route, motion, runtime command, lidar capture, trace merge, map
write, camera perception, camera frame, camera-map write, navigation, avoidance
metrics, telemetry. Webots and application state remain behind adapter callbacks.
This also fixes the lifecycle integration regression that placed camera-map write
before camera perception/frame on common due steps.
