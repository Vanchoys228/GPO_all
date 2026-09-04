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
set "PATH=%WEBOTS_LIBRARY%;%PATH%"
set "CONTROLLER_DIR=%~dp0"
set "OUTPUT_DIR=%CONTROLLER_DIR%build\tests"
if not exist "%OUTPUT_DIR%" mkdir "%OUTPUT_DIR%"

set "SOURCES="
for /f "usebackq delims=" %%S in ("%CONTROLLER_DIR%controller_sources.txt") do (
  if /I not "%%S"=="youbot_web.c" set "SOURCES=!SOURCES! %%S"
)
set /a PASSED=0
set "TEST_PATTERN=controller_*_test.c"
if defined CONTROLLER_TEST_FILTER set "TEST_PATTERN=%CONTROLLER_TEST_FILTER%"

pushd "%CONTROLLER_DIR%"
for %%F in (!TEST_PATTERN!) do (
  echo [webots-test] %%F
  cl /nologo /std:c11 /O2 /I"%WEBOTS_INCLUDE%" "%%F" !SOURCES! /Fe:"%OUTPUT_DIR%\%%~nF.exe" /Fo:"%OUTPUT_DIR%\\" /link /LIBPATH:"%WEBOTS_LIBRARY%" Controller.lib >nul
  if errorlevel 1 (
    echo [webots-test] compile failed: %%F
    goto :test_failed
  )
  "%OUTPUT_DIR%\%%~nF.exe"
  set "TEST_EXIT=!ERRORLEVEL!"
  if not "!TEST_EXIT!"=="0" (
    echo [webots-test] failed: %%F
    goto :test_failed
  )
  set /a PASSED+=1
)
popd

echo [webots-test] !PASSED! tests passed
exit /b 0

:test_failed
popd
exit /b 1
