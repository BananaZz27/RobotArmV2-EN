@echo off
cd /d "%~dp0src"
"..\.venv\Scripts\python.exe" calib_app.py
if errorlevel 1 pause
