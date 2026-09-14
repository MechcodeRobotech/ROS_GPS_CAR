@echo off
REM Build the standalone Windows viewer: webapp\exe\GPS_Localize_Viewer.exe
REM Needs Python 3 with pip. Run once; after that only run_windows.bat is needed.
setlocal
cd /d "%~dp0"

set PKG=..\gps_localize_ws\src\gps_localize

python -m pip install --upgrade pyinstaller pyyaml || goto :fail

python -m PyInstaller ^
  --noconfirm --clean --onefile ^
  --name GPS_Localize_Viewer ^
  --distpath exe --workpath build --specpath build ^
  --paths "%PKG%" ^
  --add-data "%PKG%\web;web" ^
  --add-data "%PKG%\config;config" ^
  --hidden-import gps_localize.configstore ^
  --hidden-import gps_localize.httpd ^
  --hidden-import yaml ^
  viewer_server.py || goto :fail

echo.
echo   Built exe\GPS_Localize_Viewer.exe
echo   Copy the whole "webapp" folder to any Windows PC - no Python needed there.
echo.
pause
exit /b 0

:fail
echo.
echo   BUILD FAILED - check the messages above.
echo.
pause
exit /b 1
