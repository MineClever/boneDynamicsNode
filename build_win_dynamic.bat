:: This script builds the plugin for multiple Maya versions.

@echo off
chcp 65001 >nul
setlocal EnableExtensions EnableDelayedExpansion

set "EXIT_CODE=0"
pushd "%~dp0" >nul
if errorlevel 1 (
    echo [ERROR] Failed to switch to the script directory: %~dp0
    exit /b 1
)

:: ============================================
:: Configuration

set "SOURCE_DIR=src"
set "DEFAULT_DEVKIT_ROOT=D:\MayaDev"

if defined DEVKIT_ROOT (
    echo Using DEVKIT_ROOT from the environment: !DEVKIT_ROOT!
) else (
    set "DEVKIT_ROOT=%DEFAULT_DEVKIT_ROOT%"
    echo DEVKIT_ROOT is not set; using the default path: !DEVKIT_ROOT!
)

call :SelectBuildTarget
if errorlevel 2 goto :Finish
if errorlevel 1 (
    set "EXIT_CODE=1"
    goto :Finish
)

call :SelectGenerator
if errorlevel 2 goto :Finish
if errorlevel 1 (
    set "EXIT_CODE=1"
    goto :Finish
)

if /i "%BUILD_MODE%"=="installed-maya" (
    call :BuildSelectedMaya
    if errorlevel 1 set "EXIT_CODE=1"
) else if /i "%BUILD_MODE%"=="all-devkits" (
    call :BuildAllDevkits
    if errorlevel 1 set "EXIT_CODE=1"
) else (
    echo [ERROR] Unknown build mode: %BUILD_MODE%
    set "EXIT_CODE=1"
)

goto :Finish


:: ============================================
:: Read installed Maya versions from the registry and select a build target.

:SelectBuildTarget
set "BUILD_MODE="
set "SELECTED_MAYA_VERSION="
set "SELECTED_MAYA_LOCATION="
set "INSTALLED_MAYA_COUNT=0"

call :FindInstalledMaya
set /a "ALL_DEVKITS_CHOICE=INSTALLED_MAYA_COUNT + 1"

:BuildTargetMenu
echo.
echo Available build targets:
echo ----------------------------------------
for /l %%I in (1,1,!INSTALLED_MAYA_COUNT!) do (
    echo   [%%I] Maya !INSTALLED_MAYA_VERSION_%%I!: !INSTALLED_MAYA_LOCATION_%%I!
)
echo   [!ALL_DEVKITS_CHOICE!] Build all supported Devkits under DEVKIT_ROOT
if !INSTALLED_MAYA_COUNT! EQU 0 (
    echo       ^(No Maya installations were found in the registry; Devkits can still be scanned.^)
)
echo   [0] Cancel
echo ----------------------------------------
echo.
set "TARGET_CHOICE="
set /p "TARGET_CHOICE=Select build target (Enter = [!ALL_DEVKITS_CHOICE!]): "
if errorlevel 1 exit /b 2

if "!TARGET_CHOICE!"=="" set "TARGET_CHOICE=!ALL_DEVKITS_CHOICE!"
if "!TARGET_CHOICE!"=="0" exit /b 2

if "!TARGET_CHOICE!"=="!ALL_DEVKITS_CHOICE!" (
    set "BUILD_MODE=all-devkits"
    exit /b 0
)

for /l %%I in (1,1,!INSTALLED_MAYA_COUNT!) do (
    if "!TARGET_CHOICE!"=="%%I" (
        set "BUILD_MODE=installed-maya"
        set "SELECTED_MAYA_VERSION=!INSTALLED_MAYA_VERSION_%%I!"
        set "SELECTED_MAYA_LOCATION=!INSTALLED_MAYA_LOCATION_%%I!"
    )
)

if defined BUILD_MODE (
    echo Selected Maya !SELECTED_MAYA_VERSION!.
    exit /b 0
)

echo [ERROR] Enter a number from the menu.
goto :BuildTargetMenu


:: ============================================
:: Find installed Maya versions in the Windows registry.

:FindInstalledMaya
for /f "delims=" %%V in ('powershell -NoProfile -Command "$k=Get-ChildItem HKLM:\SOFTWARE\Autodesk\Maya -EA 0;$r=@();foreach($x in $k){if($x.PSChildName.Length -eq 4){$r+=[int]$x.PSChildName}};[Array]::Sort($r);[Array]::Reverse($r);$r" 2^>nul ^<nul') do (
    call :GetMayaInstallLocation "%%V"
    if defined MAYA_INSTALL_LOCATION (
        set /a "INSTALLED_MAYA_COUNT+=1"
        set "INSTALLED_MAYA_VERSION_!INSTALLED_MAYA_COUNT!=%%V"
        set "INSTALLED_MAYA_LOCATION_!INSTALLED_MAYA_COUNT!=!MAYA_INSTALL_LOCATION!"
    )
)
exit /b 0


:: ============================================
:: Read the installation directory for a Maya version.

:GetMayaInstallLocation
set "MAYA_INSTALL_LOCATION="
for /f "delims=" %%P in ('powershell -NoProfile -Command "$p=Get-ItemProperty HKLM:\SOFTWARE\Autodesk\Maya\%~1\Setup\InstallPath -EA 0;$p.MAYA_INSTALL_LOCATION" 2^>nul ^<nul') do (
    set "MAYA_INSTALL_LOCATION=%%P"
)

if "!MAYA_INSTALL_LOCATION:~-1!"=="\" set "MAYA_INSTALL_LOCATION=!MAYA_INSTALL_LOCATION:~0,-1!"
exit /b 0


:: ============================================
:: Select a CMake Visual Studio generator via vswhere.exe.

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
    echo [ERROR] Failed to query Visual Studio installations with vswhere.exe.
    exit /b 1
)

for /f "usebackq tokens=1 delims=." %%V in ("%VSWHERE_OUTPUT%") do (
    call :AddGenerator "%%V"
)
del "%VSWHERE_OUTPUT%" >nul 2>nul

if !GENERATOR_COUNT! EQU 0 (
    echo.
    echo [ERROR] No supported Visual Studio installation with C++ build tools was found.
    echo Install the "Desktop development with C++" workload and try again.
    exit /b 1
)

:GeneratorMenu
echo.
echo Available Visual Studio CMake generators:
for /l %%I in (1,1,!GENERATOR_COUNT!) do (
    echo   [%%I] !GENERATOR_%%I!
)
echo   [0] Cancel
echo.
set "GENERATOR_CHOICE="
set /p "GENERATOR_CHOICE=Select generator: "
if errorlevel 1 exit /b 2

if "!GENERATOR_CHOICE!"=="0" exit /b 2

for /l %%I in (1,1,!GENERATOR_COUNT!) do (
    if "!GENERATOR_CHOICE!"=="%%I" set "GENERATOR=!GENERATOR_%%I!"
)

if defined GENERATOR (
    echo Selected generator: !GENERATOR!
    exit /b 0
)

echo [ERROR] Enter a number from the menu.
goto :GeneratorMenu


:: ============================================
:: Find vswhere.exe.

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
:: Map a Visual Studio major version to a CMake generator.

:AddGenerator
set "VS_MAJOR=%~1"
set "CMAKE_GENERATOR="

if "%VS_MAJOR%"=="18" set "CMAKE_GENERATOR=Visual Studio 18 2026"
if "%VS_MAJOR%"=="17" set "CMAKE_GENERATOR=Visual Studio 17 2022"
if "%VS_MAJOR%"=="16" set "CMAKE_GENERATOR=Visual Studio 16 2019"
if "%VS_MAJOR%"=="15" set "CMAKE_GENERATOR=Visual Studio 15 2017"

if not defined CMAKE_GENERATOR exit /b 0

for /l %%I in (1,1,!GENERATOR_COUNT!) do (
    if /i "!GENERATOR_%%I!"=="%CMAKE_GENERATOR%" exit /b 0
)

set /a "GENERATOR_COUNT+=1"
set "GENERATOR_!GENERATOR_COUNT!=%CMAKE_GENERATOR%"
exit /b 0


:: ============================================
:: Build for the registry-selected Maya and link its installed libraries.

:BuildSelectedMaya
call :FindDevkitForVersion "%SELECTED_MAYA_VERSION%"
if not defined DEVKIT_LOCATION (
    echo.
    echo [ERROR] No Maya %SELECTED_MAYA_VERSION% Devkit was found under DEVKIT_ROOT.
    echo Searched: %DEVKIT_ROOT%\Autodesk_Maya_%SELECTED_MAYA_VERSION%*_DEVKIT_Windows\devkitBase
    exit /b 1
)

set "MAYA_LIBRARY_DIR=%SELECTED_MAYA_LOCATION%\lib"
if not exist "%MAYA_LIBRARY_DIR%\OpenMaya.lib" (
    echo.
    echo [ERROR] OpenMaya.lib was not found in the selected Maya installation:
    echo %MAYA_LIBRARY_DIR%\OpenMaya.lib
    exit /b 1
)

call :Build "%SELECTED_MAYA_VERSION%" "%DEVKIT_LOCATION%" "%MAYA_LIBRARY_DIR%"
exit /b %ERRORLEVEL%


:: ============================================
:: Scan DEVKIT_ROOT and build every supported Devkit.

:BuildAllDevkits
set "FOUND_DEVKIT=0"
set "BUILD_FAILED=0"

for /d %%D in ("%DEVKIT_ROOT%\Autodesk_Maya_*_DEVKIT_Windows") do (
    if exist "%%~fD\devkitBase\include\maya\MFnPlugin.h" (
        set "FOUND_DEVKIT=1"
        for %%N in ("%%~fD") do set "DEVKIT_NAME=%%~nxN"
        set "DEVKIT_MAYA_VERSION="
        for /f "tokens=3 delims=_" %%V in ("!DEVKIT_NAME!") do set "DEVKIT_MAYA_VERSION=%%V"

        call :Build "!DEVKIT_MAYA_VERSION!" "%%~fD\devkitBase" ""
        set "BUILD_RESULT=!ERRORLEVEL!"
        if "!BUILD_RESULT!"=="1" set "BUILD_FAILED=1"
    )
)

if "!FOUND_DEVKIT!"=="0" (
    echo.
    echo [ERROR] No valid devkitBase was found under DEVKIT_ROOT.
    echo Searched: %DEVKIT_ROOT%\Autodesk_Maya_*_DEVKIT_Windows\devkitBase
    exit /b 1
)

exit /b !BUILD_FAILED!


:: ============================================
:: Set the compiler toolset and C++ standard for a Maya version.
:: Return 1 when the version is outside the supported 2022-2027 range.

:GetBuildSettings
set "MSVC="
set "CXX_STANDARD_VER="

if "%~1"=="2027" (
    set "MSVC=v143"
    set "CXX_STANDARD_VER=20"
)
if "%~1"=="2026" (
    set "MSVC=v143"
    set "CXX_STANDARD_VER=17"
)
if "%~1"=="2025" (
    set "MSVC=v143"
    set "CXX_STANDARD_VER=17"
)
if "%~1"=="2024" (
    set "MSVC=v143"
    set "CXX_STANDARD_VER=17"
)
if "%~1"=="2023" (
    set "MSVC=v142"
    set "CXX_STANDARD_VER=17"
)
if "%~1"=="2022" (
    set "MSVC=v142"
    set "CXX_STANDARD_VER=17"
)
if "%~1"=="2019" (
    set "MSVC=v142"
    set "CXX_STANDARD_VER=17"
)

if not defined MSVC exit /b 1
exit /b 0


:: ============================================
:: Build one Maya version with the specified Devkit.
:: Pass the optional third argument to CMake as the Maya library directory.

:Build
set "MAYA_VERSION=%~1"
set "DEVKIT_LOCATION=%~2"
set "MAYA_LIBRARY_DIR=%~3"
set "DIRNAME=build/win64_%MAYA_VERSION%"

call :GetBuildSettings "%MAYA_VERSION%"
if errorlevel 1 (
    echo.
    echo [SKIP] Maya %MAYA_VERSION% is outside this project's supported range (2022-2027).
    exit /b 2
)

if not exist "%DEVKIT_LOCATION%\include\maya\MFnPlugin.h" (
    echo.
    echo [ERROR] Invalid DEVKIT_LOCATION: %DEVKIT_LOCATION%
    exit /b 1
)

if defined MAYA_LIBRARY_DIR (
    if not exist "%MAYA_LIBRARY_DIR%\OpenMaya.lib" (
        echo.
        echo [ERROR] Maya library directory does not contain OpenMaya.lib: %MAYA_LIBRARY_DIR%
        exit /b 1
    )
)

echo.
echo ========================================
echo Maya %MAYA_VERSION%
echo GENERATOR=%GENERATOR%
echo DEVKIT_LOCATION=%DEVKIT_LOCATION%
if defined MAYA_LIBRARY_DIR (
    echo LIBRARY_DIR=%MAYA_LIBRARY_DIR% ^(Maya installation directory^)
) else (
    echo LIBRARY_DIR=Resolved automatically from the Devkit
)
echo BUILD_DIR=%DIRNAME%
echo TOOLSET=%MSVC%
echo CXX_STANDARD_VER=%CXX_STANDARD_VER%
echo ========================================

if defined MAYA_LIBRARY_DIR (
    cmake -S "%SOURCE_DIR%" -B "%DIRNAME%" -G "%GENERATOR%" -A x64 -T "%MSVC%" -DCXX_STANDARD_VER=%CXX_STANDARD_VER% "-DDEVKIT_LIBRARY_DIR:PATH=%MAYA_LIBRARY_DIR%"
) else (
    cmake -S "%SOURCE_DIR%" -B "%DIRNAME%" -G "%GENERATOR%" -A x64 -T "%MSVC%" -DCXX_STANDARD_VER=%CXX_STANDARD_VER% -UDEVKIT_LIBRARY_DIR
)
if errorlevel 1 exit /b 1

cmake --build "%DIRNAME%"
if errorlevel 1 exit /b 1

exit /b 0


:: ============================================
:: Find the Devkit for a Maya version under DEVKIT_ROOT.

:FindDevkitForVersion
set "DEVKIT_LOCATION="

for /d %%D in ("%DEVKIT_ROOT%\Autodesk_Maya_%~1*_DEVKIT_Windows") do (
    if exist "%%~fD\devkitBase\include\maya\MFnPlugin.h" (
        set "DEVKIT_LOCATION=%%~fD\devkitBase"
        goto :eof
    )
)

exit /b 0


:Finish
popd >nul
exit /b %EXIT_CODE%
