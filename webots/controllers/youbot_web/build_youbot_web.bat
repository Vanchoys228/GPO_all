@echo off
setlocal

set "VCVARS="
if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" (
  set "VCVARS=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
)
if not defined VCVARS if exist "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" (
  set "VCVARS=C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
)

if not defined VCVARS (
  echo Failed to locate vcvars64.bat. Install Visual Studio 2022 with Desktop development with C++.
  exit /b 1
)

call "%VCVARS%" >nul
if errorlevel 1 exit /b 1

if not defined WEBOTS_HOME (
  if exist "D:\DS\Programs\Webots\include\controller\c\webots\robot.h" (
    set "WEBOTS_HOME=D:\DS\Programs\Webots"
  ) else if exist "C:\Program Files\Webots\include\controller\c\webots\robot.h" (
    set "WEBOTS_HOME=C:\Program Files\Webots"
  )
)

if not exist "%WEBOTS_HOME%\include\controller\c\webots\robot.h" (
  echo Failed to locate Webots headers in "%WEBOTS_HOME%".
  echo Set WEBOTS_HOME to your Webots installation directory and rerun the build.
  exit /b 1
)

set "CONTROLLER_DIR=%~dp0"
pushd "%CONTROLLER_DIR%"
if not exist "build" mkdir "build"
if not exist "build\obj" mkdir "build\obj"

cl /nologo /std:c11 /O2 /I"%WEBOTS_HOME%\include\controller\c" ^
  "youbot_web.c" ^
  "controller_avoidance_command.c" ^
  "controller_avoidance_detection.c" ^
  "controller_avoidance_state.c" ^
  "controller_avoidance_start.c" ^
  "controller_avoidance_lifecycle.c" ^
  "controller_avoidance_recovery.c" ^
  "controller_avoidance_service.c" ^
  "controller_avoidance_presentation.c" ^
  "controller_camera.c" ^
  "controller_camera_map.c" ^
  "controller_camera_map_io.c" ^
  "controller_camera_render.c" ^
  "controller_camera_fusion.c" ^
  "controller_camera_virtual.c" ^
  "controller_drive.c" ^
  "controller_io.c" ^
  "controller_lidar_math.c" ^
  "controller_lidar_scan.c" ^
  "controller_lidar_trace.c" ^
  "controller_lifecycle.c" ^
  "controller_math.c" ^
  "controller_mapping_route_io.c" ^
  "controller_mapping_obstacles.c" ^
  "controller_mapping_scan.c" ^
  "controller_mapping_scan_service.c" ^
  "controller_mapping_scan_transition.c" ^
  "controller_mapping_survey_escape.c" ^
  "controller_mapping_survey_safety.c" ^
  "controller_mapping_store.c" ^
  "controller_motion_profile.c" ^
  "controller_navigation_context.c" ^
  "controller_navigation_adapter.c" ^
  "controller_navigation_state.c" ^
  "controller_navigation_metrics.c" ^
  "controller_navigation_lidar.c" ^
  "controller_navigation_motion_service.c" ^
  "controller_navigation_perception.c" ^
  "controller_navigation_presentation.c" ^
  "controller_navigation_route.c" ^
  "controller_navigation_session.c" ^
  "controller_navigation_service.c" ^
  "controller_navigation_tracking.c" ^
  "controller_navigation_zone_guard.c" ^
  "controller_obstacle_map.c" ^
  "controller_paths.c" ^
  "controller_route.c" ^
  "controller_route_zone_service.c" ^
  "controller_runtime.c" ^
  "controller_runtime_command.c" ^
  "controller_survey_contour.c" ^
  "controller_survey_coverage.c" ^
  "controller_survey_coverage_bounds.c" ^
  "controller_survey_generator.c" ^
  "controller_survey_grid.c" ^
  "controller_survey_geometry.c" ^
  "controller_survey_intervals.c" ^
  "controller_survey_integration.c" ^
  "controller_survey_lifecycle.c" ^
  "controller_survey_route_builder.c" ^
  "controller_survey_state.c" ^
  "controller_step.c" ^
  "controller_telemetry.c" ^
  "controller_telemetry_service.c" ^
  "controller_types.c" ^
  "controller_webots_devices.c" ^
  "controller_webots_adapter.c" ^
  "controller_webots_camera_adapter.c" ^
  "controller_webots_pose.c" ^
  "controller_webots_sensors.c" ^
  "controller_webots_simulation_format.c" ^
  "controller_webots_simulation_registry.c" ^
  "controller_webots_simulation.c" ^
  "controller_zones.c" ^
  "controller_zone_geometry.c" ^
  /Fe:"youbot_web.exe" ^
  /Fo"build\obj\\" ^
  /link /LIBPATH:"%WEBOTS_HOME%\lib\controller" Controller.lib
if errorlevel 1 (
  popd
  exit /b 1
)

echo Built: "%CONTROLLER_DIR%youbot_web.exe"
popd
