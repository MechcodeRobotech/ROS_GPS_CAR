@echo off
REM One click: test everything this Windows machine can test.
REM   test_windows.bat            full run
REM   test_windows.bat --quick    skip the firmware build
REM   test_windows.bat --up       also build and start the Docker stack
cd /d "%~dp0"
python tools\test_windows.py %*
echo.
pause
