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
if not exist "%CONTROLLER_DIR%build" mkdir "%CONTROLLER_DIR%build"
if not exist "%CONTROLLER_DIR%build\obj" mkdir "%CONTROLLER_DIR%build\obj"

cl /nologo /std:c11 /O2 /I"%WEBOTS_HOME%\include\controller\c" ^
  "%CONTROLLER_DIR%youbot_web.c" ^
  "%CONTROLLER_DIR%controller_avoidance.c" ^
  "%CONTROLLER_DIR%controller_camera.c" ^
  "%CONTROLLER_DIR%controller_camera_map.c" ^
  "%CONTROLLER_DIR%controller_camera_render.c" ^
  "%CONTROLLER_DIR%controller_drive.c" ^
  "%CONTROLLER_DIR%controller_io.c" ^
  "%CONTROLLER_DIR%controller_lidar_math.c" ^
  "%CONTROLLER_DIR%controller_lifecycle.c" ^
  "%CONTROLLER_DIR%controller_math.c" ^
  "%CONTROLLER_DIR%controller_motion_profile.c" ^
  "%CONTROLLER_DIR%controller_route.c" ^
  "%CONTROLLER_DIR%controller_runtime_command.c" ^
  "%CONTROLLER_DIR%controller_survey_geometry.c" ^
  "%CONTROLLER_DIR%controller_survey_state.c" ^
  "%CONTROLLER_DIR%controller_step.c" ^
  "%CONTROLLER_DIR%controller_telemetry.c" ^
  "%CONTROLLER_DIR%controller_types.c" ^
  "%CONTROLLER_DIR%controller_zones.c" ^
  /Fe:"%CONTROLLER_DIR%youbot_web.exe" ^
  /Fo"%CONTROLLER_DIR%build\obj\\" ^
  /link /LIBPATH:"%WEBOTS_HOME%\lib\controller" Controller.lib
if errorlevel 1 exit /b 1

echo Built: "%CONTROLLER_DIR%youbot_web.exe"
