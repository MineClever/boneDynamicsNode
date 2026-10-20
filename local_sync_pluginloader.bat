@echo off
setlocal EnableExtensions
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0local_sync_pluginloader.ps1" %*
set "SYNC_EXIT_CODE=%ERRORLEVEL%"
echo.
if not "%SYNC_EXIT_CODE%"=="0" echo [ERROR] Sync failed with exit code %SYNC_EXIT_CODE%.
pause
exit /b %SYNC_EXIT_CODE%
