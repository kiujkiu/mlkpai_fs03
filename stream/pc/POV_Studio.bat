@echo off
rem POV Studio 启动器 (Windows Python 3.12)
chcp 65001 >nul
cd /d "%~dp0"
set "PY=%LOCALAPPDATA%\Programs\Python\Python312\python.exe"
if not exist "%PY%" (
    echo [POV Studio] 找不到 %PY%
    pause
    exit /b 1
)
"%PY%" pov_studio.py %*
if errorlevel 1 pause
