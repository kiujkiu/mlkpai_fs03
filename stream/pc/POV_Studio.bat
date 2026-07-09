@echo off
chcp 65001 >nul
cd /d D:\claude_workspace\pov3d\mlkpai_fs03\stream\pc
"%LOCALAPPDATA%\Programs\Python\Python312\python.exe" pov_studio.py
if errorlevel 1 pause
