@echo off
setlocal EnableDelayedExpansion

set "SCRIPT_DIR=%~dp0"
set "BUILD_DIR=%SCRIPT_DIR%build"
set "OBJ_DIR=%BUILD_DIR%\obj"
set "SOURCE_MANIFEST=%SCRIPT_DIR%route_solver_sources.txt"

set "VCVARS="
if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" set "VCVARS=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
if not defined VCVARS if exist "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" set "VCVARS=C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
if not defined VCVARS (
  echo Failed to locate vcvars64.bat. Install Visual Studio 2022 with Desktop development with C++.
  exit /b 1
)

call "%VCVARS%" >nul
if errorlevel 1 exit /b 1
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"
if not exist "%OBJ_DIR%" mkdir "%OBJ_DIR%"

set "CLFLAGS=/nologo /std:c++20 /EHsc /O2 /W4"
set "OBJECTS="
for /f "usebackq delims=" %%S in ("%SOURCE_MANIFEST%") do (
  echo Compiling %%~nxS
  cl %CLFLAGS% /I"%SCRIPT_DIR%include" /c "%SCRIPT_DIR%%%S" /Fo"%OBJ_DIR%\%%~nS.obj"
  if errorlevel 1 exit /b 1
  set "OBJECTS=!OBJECTS! "%OBJ_DIR%\%%~nS.obj""
)

echo Compiling gpo_route_solver.cpp
cl %CLFLAGS% /I"%SCRIPT_DIR%include" /c "%SCRIPT_DIR%apps\gpo_route_solver.cpp" /Fo"%OBJ_DIR%\gpo_route_solver.obj"
if errorlevel 1 exit /b 1

echo Linking gpo_route_solver.exe
cl /nologo !OBJECTS! "%OBJ_DIR%\gpo_route_solver.obj" /Fe:"%BUILD_DIR%\gpo_route_solver.exe"
if errorlevel 1 exit /b 1
echo Build finished: "%BUILD_DIR%\gpo_route_solver.exe"
