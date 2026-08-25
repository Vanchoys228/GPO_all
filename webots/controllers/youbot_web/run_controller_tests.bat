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

set "CONTROLLER_DIR=%~dp0"
set "OUTPUT_DIR=%CONTROLLER_DIR%build\tests"
if not exist "%OUTPUT_DIR%" mkdir "%OUTPUT_DIR%"

set SOURCES=controller_avoidance.c controller_camera.c controller_camera_map.c controller_camera_render.c controller_drive.c controller_io.c controller_lidar_math.c controller_lifecycle.c controller_math.c controller_motion_profile.c controller_paths.c controller_route.c controller_runtime_command.c controller_step.c controller_survey_geometry.c controller_survey_state.c controller_telemetry.c controller_types.c controller_zones.c
set /a PASSED=0

pushd "%CONTROLLER_DIR%"
for %%F in (controller_*_test.c) do (
  echo [webots-test] %%F
  cl /nologo /std:c11 /O2 "%%F" !SOURCES! /Fe:"%OUTPUT_DIR%\%%~nF.exe" /Fo:"%OUTPUT_DIR%\\" >nul
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
