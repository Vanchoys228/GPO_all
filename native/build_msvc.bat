@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "BUILD_DIR=%SCRIPT_DIR%build"
set "OBJ_DIR=%BUILD_DIR%\obj"

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

if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"
if not exist "%OBJ_DIR%" mkdir "%OBJ_DIR%"

set "CLFLAGS=/nologo /std:c++20 /EHsc /O2"
set "LDFLAGS=/nologo"

set "SOURCES=%SCRIPT_DIR%src\common.cpp %SCRIPT_DIR%src\ga_tabu.cpp %SCRIPT_DIR%src\annealing.cpp %SCRIPT_DIR%src\scatter.cpp %SCRIPT_DIR%src\cuckoo.cpp %SCRIPT_DIR%apps\gpo_route_solver.cpp"

for %%F in (%SOURCES%) do (
  echo Compiling %%~nxF
  cl %CLFLAGS% /I"%SCRIPT_DIR%include" /c "%%~F" /Fo"%OBJ_DIR%\%%~nF.obj"
  if errorlevel 1 exit /b 1
)

set "OBJECTS=%OBJ_DIR%\common.obj %OBJ_DIR%\ga_tabu.obj %OBJ_DIR%\annealing.obj %OBJ_DIR%\scatter.obj %OBJ_DIR%\cuckoo.obj %OBJ_DIR%\gpo_route_solver.obj"
echo Linking gpo_route_solver.exe
cl %LDFLAGS% %OBJECTS% /Fe:"%BUILD_DIR%\gpo_route_solver.exe"
if errorlevel 1 exit /b 1

echo Build finished: "%BUILD_DIR%\gpo_route_solver.exe"
