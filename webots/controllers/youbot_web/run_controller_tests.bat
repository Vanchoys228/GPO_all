@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "VCVARS="
if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" set "VCVARS=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
if not defined VCVARS if exist "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" set "VCVARS=C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
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
  echo Set WEBOTS_HOME to your Webots installation directory and rerun the tests.
  exit /b 1
)

set "WEBOTS_INCLUDE=%WEBOTS_HOME%\include\controller\c"
set "WEBOTS_LIBRARY=%WEBOTS_HOME%\lib\controller"
set "CONTROLLER_DIR=%~dp0"
set "OUTPUT_DIR=%CONTROLLER_DIR%build\tests"
if not exist "%OUTPUT_DIR%" mkdir "%OUTPUT_DIR%"

set SOURCES=controller_avoidance_start.c controller_avoidance_lifecycle.c controller_avoidance_recovery.c controller_avoidance_service.c controller_avoidance_presentation.c controller_camera.c controller_camera_map.c controller_camera_map_io.c controller_camera_render.c controller_camera_fusion.c controller_camera_virtual.c controller_drive.c controller_io.c controller_lidar_math.c controller_lidar_scan.c controller_lidar_trace.c controller_lifecycle.c controller_mapping_obstacles.c controller_mapping_route_io.c controller_mapping_scan.c controller_mapping_scan_service.c controller_mapping_store.c controller_mapping_survey_escape.c controller_math.c controller_motion_profile.c controller_navigation_context.c controller_navigation_lidar.c controller_navigation_perception.c controller_navigation_presentation.c controller_navigation_route.c controller_navigation_session.c controller_navigation_service.c controller_navigation_tracking.c controller_navigation_zone_guard.c controller_obstacle_map.c controller_paths.c controller_route.c controller_route_zone_service.c controller_runtime.c controller_runtime_command.c controller_step.c controller_survey_contour.c controller_survey_coverage.c controller_survey_generator.c controller_survey_grid.c controller_survey_grid_navigation.c controller_survey_offset_contour.c controller_survey_geometry.c controller_survey_integration.c controller_survey_lifecycle.c controller_survey_route_builder.c controller_survey_state.c controller_telemetry.c controller_telemetry_service.c controller_types.c controller_webots_devices.c controller_webots_adapter.c controller_webots_pose.c controller_webots_sensors.c controller_webots_simulation.c controller_zone_geometry.c controller_zones.c
set SOURCES=%SOURCES% controller_webots_camera_adapter.c
set SOURCES=%SOURCES% controller_navigation_motion_service.c
set SOURCES=%SOURCES% controller_survey_route_primitives.c
set SOURCES=%SOURCES% controller_survey_contour_path.c
set SOURCES=%SOURCES% controller_survey_coverage_intervals.c
set SOURCES=%SOURCES% controller_camera_geometry.c
set SOURCES=%SOURCES% controller_webots_zone_sync.c
set SOURCES=%SOURCES% controller_webots_motion_state.c
set SOURCES=%SOURCES% controller_mapping_survey_runtime_safety.c
set SOURCES=%SOURCES% controller_webots_navigation_state.c
set SOURCES=%SOURCES% controller_webots_camera_range.c
set SOURCES=%SOURCES% controller_webots_camera_perception.c
set SOURCES=%SOURCES% controller_webots_camera_map_sync.c
set SOURCES=%SOURCES% controller_mapping_survey_safety.c
set SOURCES=%SOURCES% controller_navigation_adapter.c
set SOURCES=%SOURCES% controller_navigation_state.c
set SOURCES=%SOURCES% controller_navigation_metrics.c
set SOURCES=%SOURCES% controller_mapping_scan_transition.c
set SOURCES=%SOURCES% controller_webots_simulation_format.c
set SOURCES=%SOURCES% controller_webots_simulation_registry.c
set SOURCES=%SOURCES% controller_avoidance_command.c
set SOURCES=%SOURCES% controller_avoidance_detection.c
set SOURCES=%SOURCES% controller_avoidance_state.c
set SOURCES=%SOURCES% controller_survey_intervals.c
set SOURCES=%SOURCES% controller_survey_coverage_bounds.c
set /a PASSED=0
set "TEST_PATTERN=controller_*_test.c"
if defined CONTROLLER_TEST_FILTER set "TEST_PATTERN=%CONTROLLER_TEST_FILTER%"

pushd "%CONTROLLER_DIR%"
for %%F in (!TEST_PATTERN!) do (
  echo [webots-test] %%F
  cl /nologo /std:c11 /O2 /I"%WEBOTS_INCLUDE%" "%%F" !SOURCES! /Fe:"%OUTPUT_DIR%\%%~nF.exe" /Fo:"%OUTPUT_DIR%\\" /link /LIBPATH:"%WEBOTS_LIBRARY%" Controller.lib >nul
  if errorlevel 1 (
    echo [webots-test] compile failed: %%F
    popd
    exit /b 1
  )
  "%OUTPUT_DIR%\%%~nF.exe"
  if errorlevel 1 (
    echo [webots-test] failed: %%F
    popd
    exit /b 1
  )
  set /a PASSED+=1
)
popd

echo [webots-test] !PASSED! tests passed
exit /b 0
