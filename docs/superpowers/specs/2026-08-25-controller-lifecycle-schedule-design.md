# Controller Lifecycle Schedule Design

Create a Webots-independent `controller_lifecycle.c/.h` that converts a step
counter plus explicit intervals into a task mask for zone, route, motion,
runtime-command, map, camera-capture, and camera-frame work. Step zero remains due
for every positive interval. The existing task functions retain their internal
guards, file checks, and side effects, preserving startup and cleanup behavior.
