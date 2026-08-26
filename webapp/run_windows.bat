@echo off
REM GPS_Localize Viewer - Windows launcher (VIEW ONLY, cannot control the robot)
REM Double-click this file. It uses exe\GPS_Localize_Viewer.exe if it is there,
REM otherwise it falls back to a Python installation.
setlocal
cd /d "%~dp0"

if exist "exe\GPS_Localize_Viewer.exe" (
    echo Starting GPS_Localize Viewer...
    "exe\GPS_Localize_Viewer.exe" %*
    goto :end
)

where python >nul 2>&1
if %errorlevel%==0 (
    echo Starting GPS_Localize Viewer with Python...
    python viewer_server.py %*
    goto :end
)

echo.
echo   Neither exe\GPS_Localize_Viewer.exe nor Python was found.
echo   Either build the exe once with build_exe.bat, or install Python 3
echo   from https://www.python.org/downloads/ and run this file again.
echo.

:end
pause
