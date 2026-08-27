# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

### Generic Makefile.include for Webots controllers, physics plugins, robot
### window libraries, remote control libraries and other libraries
### to be used with GNU make
###
### Platforms: Windows, macOS, Linux
### Languages: C, C++
###
### Authors: Olivier Michel, Yvan Bourquin, Fabien Rohrer
###          Edmund Ronald, Sergei Poskriakov
###
###-----------------------------------------------------------------------------
###
### This file is meant to be included from the Makefile files located in the
### Webots projects subdirectories. It is possible to set a number of variables
### to customize the build process, i.e., add source files, compilation flags,
### include paths, libraries, etc. These variables should be set in your local
### Makefile just before including this Makefile.include. This Makefile.include
### should never be modified.
###
### Here is a description of the variables you may set in your local Makefile:
###
### ---- C Sources ----
### if your program uses several C source files:
### C_SOURCES = my_plugin.c my_clever_algo.c my_graphics.c
###
### ---- C++ Sources ----
### if your program uses several C++ source files:
### CXX_SOURCES = my_plugin.cc my_clever_algo.cpp my_graphics.c++
###
### ---- Compilation options ----
### if special compilation flags are necessary:
### CFLAGS = -Wno-unused-result
###
### ---- Linked libraries ----
### if your program needs additional libraries:
### INCLUDE = -I"/my_library_path/include"
### LIBRARIES = -L"/path/to/my/library" -lmy_library -lmy_other_library
###
### ---- Linking options ----
### if special linking flags are needed:
### LFLAGS = -s
###
### ---- Webots included libraries ----
### if you want to use the Webots C API in your C++ controller program:
### USE_C_API = true
###
### ---- Debug mode ----
### if you want to display the gcc command line for compilation and link, as
### well as the rm command details used for cleaning:
### VERBOSE = 1
###
###-----------------------------------------------------------------------------

C_SOURCES = \
  youbot_web.c \
  controller_avoidance.c \
  controller_avoidance_start.c \
  controller_avoidance_lifecycle.c \
  controller_avoidance_recovery.c \
  controller_avoidance_service.c \
  controller_avoidance_presentation.c \
  controller_camera.c \
  controller_camera_map.c \
  controller_camera_map_io.c \
  controller_camera_render.c \
  controller_camera_fusion.c \
  controller_camera_virtual.c \
  controller_drive.c \
  controller_io.c \
  controller_lidar_math.c \
  controller_lidar_scan.c \
  controller_lidar_trace.c \
  controller_lifecycle.c \
  controller_math.c \
  controller_mapping_route_io.c \
  controller_mapping_obstacles.c \
  controller_mapping_scan.c \
  controller_mapping_scan_service.c \
  controller_motion_profile.c \
  controller_navigation_context.c \
  controller_navigation_lidar.c \
  controller_navigation_motion_service.c \
  controller_navigation_perception.c \
  controller_navigation_presentation.c \
  controller_navigation_route.c \
  controller_navigation_session.c \
  controller_navigation_service.c \
  controller_navigation_tracking.c \
  controller_navigation_zone_guard.c \
  controller_obstacle_map.c \
  controller_paths.c \
  controller_route.c \
  controller_runtime_command.c \
  controller_step.c \
  controller_survey_contour.c \
  controller_survey_coverage.c \
  controller_survey_generator.c \
  controller_survey_grid.c \
  controller_survey_geometry.c \
  controller_survey_integration.c \
  controller_survey_lifecycle.c \
  controller_survey_route_builder.c \
  controller_survey_state.c \
  controller_telemetry.c \
  controller_telemetry_service.c \
  controller_types.c \
  controller_webots_devices.c \
  controller_webots_adapter.c \
  controller_webots_camera_adapter.c \
  controller_webots_pose.c \
  controller_webots_sensors.c \
  controller_webots_simulation.c \
  controller_zone_geometry.c \
  controller_zones.c

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
