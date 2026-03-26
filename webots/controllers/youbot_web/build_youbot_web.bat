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

set "WEBOTS_HOME=C:\Program Files\Webots"
if not exist "%WEBOTS_HOME%\include\controller\c\webots\robot.h" (
  echo Failed to locate Webots headers in "%WEBOTS_HOME%".
  exit /b 1
)

set "CONTROLLER_DIR=%~dp0"
if not exist "%CONTROLLER_DIR%build" mkdir "%CONTROLLER_DIR%build"
if not exist "%CONTROLLER_DIR%build\obj" mkdir "%CONTROLLER_DIR%build\obj"

cl /nologo /std:c11 /O2 /I"%WEBOTS_HOME%\include\controller\c" ^
  "%CONTROLLER_DIR%youbot_web.c" ^
  /Fe:"%CONTROLLER_DIR%youbot_web.exe" ^
  /Fo"%CONTROLLER_DIR%build\obj\\" ^
  /link /LIBPATH:"%WEBOTS_HOME%\lib\controller" Controller.lib
if errorlevel 1 exit /b 1

echo Built: "%CONTROLLER_DIR%youbot_web.exe"
