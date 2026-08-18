:: This batch script builds the Maya plugin for multiple Maya versions.
@chcp 65001
@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "EXIT_CODE=0"
pushd "%~dp0" >nul
if errorlevel 1 (
    echo Failed to switch to script directory: %~dp0
    exit /b 1
)

:: ============================================
:: Configuration

set "SOURCE_DIR=src"
set "DEFAULT_DEVKIT_ROOT=D:\MayaDev"

if defined DEVKIT_ROOT (
    echo Using DEVKIT_ROOT from environment: %DEVKIT_ROOT%
) else (
    set "DEVKIT_ROOT=%DEFAULT_DEVKIT_ROOT%"
    echo DEVKIT_ROOT is not set; using default: %DEVKIT_ROOT%
)

call :SelectGenerator
if errorlevel 1 (
    set "EXIT_CODE=1"
    goto :Finish
)


:: ============================================
:: Main build loop for different Maya versions.

set "BUILD_FAILED=0"

call :Build 2027 "Autodesk_Maya_2027*_DEVKIT_Windows" v143 20
if errorlevel 1 set "BUILD_FAILED=1"

call :Build 2026 "Autodesk_Maya_2026*_DEVKIT_Windows" v143 17
if errorlevel 1 set "BUILD_FAILED=1"

call :Build 2025 "Autodesk_Maya_2025*_DEVKIT_Windows" v143 17
if errorlevel 1 set "BUILD_FAILED=1"

call :Build 2024 "Autodesk_Maya_2024*_DEVKIT_Windows" v143 17
if errorlevel 1 set "BUILD_FAILED=1"

call :Build 2023 "Autodesk_Maya_2023*_DEVKIT_Windows" v142
if errorlevel 1 set "BUILD_FAILED=1"

call :Build 2022 "Autodesk_Maya_2022*_DEVKIT_Windows" v142
if errorlevel 1 set "BUILD_FAILED=1"

set "EXIT_CODE=%BUILD_FAILED%"
goto :Finish


:: ============================================
:: Function: Select a CMake Visual Studio generator via vswhere.exe.

:SelectGenerator
set "GENERATOR="
set "GENERATOR_COUNT=0"

call :FindVsWhere
if not defined VSWHERE (
    echo.
    echo [ERROR] vswhere.exe was not found.
    echo Install Visual Studio Installer, or add vswhere.exe to PATH.
    exit /b 1
)

set "VSWHERE_OUTPUT=%TEMP%\boneDynamicsNode_vswhere_%RANDOM%.txt"
"%VSWHERE%" -all -prerelease -products * -requires Microsoft.Component.MSBuild Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationVersion > "%VSWHERE_OUTPUT%"
if errorlevel 1 (
    del "%VSWHERE_OUTPUT%" >nul 2>nul
    echo.
    echo [ERROR] Failed to query Visual Studio installations via vswhere.exe.
    exit /b 1
)

for /f "usebackq tokens=1 delims=." %%V in ("%VSWHERE_OUTPUT%") do (
    call :AddGenerator "%%V"
)
del "%VSWHERE_OUTPUT%" >nul 2>nul

if %GENERATOR_COUNT% EQU 0 (
    echo.
    echo [ERROR] No Visual Studio installation with C++ build tools was found.
    echo Install the "Desktop development with C++" workload and try again.
    exit /b 1
)

:GeneratorMenu
echo.
echo Available Visual Studio CMake generators:
for /l %%I in (1,1,%GENERATOR_COUNT%) do (
    echo   %%I^) !GENERATOR_%%I!
)
echo.
set "GENERATOR_CHOICE="
set /p "GENERATOR_CHOICE=Select generator [1-%GENERATOR_COUNT%]: "

if not defined GENERATOR_CHOICE (
    echo Please enter a number from 1 to %GENERATOR_COUNT%.
    goto :GeneratorMenu
)

for /f "delims=0123456789" %%A in ("!GENERATOR_CHOICE!") do (
    echo Please enter a number from 1 to %GENERATOR_COUNT%.
    goto :GeneratorMenu
)

set "GENERATOR="
for /l %%I in (1,1,%GENERATOR_COUNT%) do (
    if "!GENERATOR_CHOICE!"=="%%I" set "GENERATOR=!GENERATOR_%%I!"
)

if not defined GENERATOR (
    echo Please enter a number from 1 to %GENERATOR_COUNT%.
    goto :GeneratorMenu
)

echo Selected generator: %GENERATOR%
exit /b 0


:: ============================================
:: Function: Find vswhere.exe.

:FindVsWhere
set "VSWHERE="

if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" (
    set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
    exit /b 0
)

if exist "%ProgramFiles%\Microsoft Visual Studio\Installer\vswhere.exe" (
    set "VSWHERE=%ProgramFiles%\Microsoft Visual Studio\Installer\vswhere.exe"
    exit /b 0
)

for /f "delims=" %%P in ('where vswhere.exe 2^>nul') do (
    set "VSWHERE=%%P"
    exit /b 0
)

exit /b 0


:: ============================================
:: Function: Add a supported CMake Visual Studio generator.

:AddGenerator
set "VS_MAJOR=%~1"
set "CMAKE_GENERATOR="

if "%VS_MAJOR%"=="18" set "CMAKE_GENERATOR=Visual Studio 18 2026"
if "%VS_MAJOR%"=="17" set "CMAKE_GENERATOR=Visual Studio 17 2022"
if "%VS_MAJOR%"=="16" set "CMAKE_GENERATOR=Visual Studio 16 2019"
if "%VS_MAJOR%"=="15" set "CMAKE_GENERATOR=Visual Studio 15 2017"

if not defined CMAKE_GENERATOR exit /b 0

for /l %%I in (1,1,%GENERATOR_COUNT%) do (
    if /i "!GENERATOR_%%I!"=="%CMAKE_GENERATOR%" exit /b 0
)

set /a GENERATOR_COUNT+=1
set "GENERATOR_!GENERATOR_COUNT!=%CMAKE_GENERATOR%"
exit /b 0


:: ============================================
:: Function: Build the plugin for a specific Maya version.

:Build
set "MAYA_VERSION=%~1"
set "DEVKIT_PATTERN=%~2"
set "MSVC=%~3"
set "CXX_STANDARD_VER=%~4"
set "DIRNAME=build/win64_%MAYA_VERSION%"

call :FindDevkit "%DEVKIT_PATTERN%"

if not defined DEVKIT_LOCATION (
    echo.
    echo [ERROR] Maya %MAYA_VERSION% Devkit was not found under: %DEVKIT_ROOT%
    echo Pattern: %DEVKIT_PATTERN%
    exit /b 1
)

echo.
echo ========================================
echo Maya %MAYA_VERSION%
echo GENERATOR=%GENERATOR%
echo DEVKIT_LOCATION=%DEVKIT_LOCATION%
echo BUILD_DIR=%DIRNAME%
echo TOOLSET=%MSVC%
if defined CXX_STANDARD_VER echo CXX_STANDARD_VER=%CXX_STANDARD_VER%
echo ========================================

if defined CXX_STANDARD_VER (
    cmake -S "%SOURCE_DIR%" -B "%DIRNAME%" -G "%GENERATOR%" -A x64 -T "%MSVC%" -DCXX_STANDARD_VER=%CXX_STANDARD_VER%
) else (
    cmake -S "%SOURCE_DIR%" -B "%DIRNAME%" -G "%GENERATOR%" -A x64 -T "%MSVC%"
)
if errorlevel 1 exit /b 1

cmake --build "%DIRNAME%"
if errorlevel 1 exit /b 1

exit /b 0


:: ============================================
:: Function: Find the Maya Devkit location based on the provided pattern.

:FindDevkit
set "DEVKIT_LOCATION="

for /d %%D in ("%DEVKIT_ROOT%\%~1") do (
    if exist "%%~fD\devkitBase\include\maya\MFnPlugin.h" (
        set "DEVKIT_LOCATION=%%~fD\devkitBase"
        goto :eof
    )
)

exit /b 0


:Finish
popd >nul
exit /b %EXIT_CODE%
