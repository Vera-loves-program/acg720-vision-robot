@echo off
setlocal
cd /d "%~dp0.."
if exist ".venv\Scripts\python.exe" goto environment_ready
where py >nul 2>nul
if not errorlevel 1 (
    set "VR_PY=py -3"
) else (
    where python >nul 2>nul
    if errorlevel 1 goto no_python
    set "VR_PY=python"
)
%VR_PY% -c "import sys; sys.exit(sys.version_info < (3,10))"
if errorlevel 1 goto no_python
%VR_PY% -m venv .venv
if errorlevel 1 goto failed
:environment_ready
if /i "%~1"=="probe" (
    .venv\Scripts\python.exe pc\udp_probe.py --bind 192.168.10.3 --seconds 10
    goto finished
)
.venv\Scripts\python.exe -c "import cv2, numpy" >nul 2>nul
if not errorlevel 1 goto view
.venv\Scripts\python.exe -m pip install -i https://mirrors.aliyun.com/pypi/simple/ -r pc\requirements.txt
if errorlevel 1 goto failed
:view
.venv\Scripts\python.exe pc\udp_video_viewer.py --bind 192.168.10.3
:finished
set "VR_EXIT=%ERRORLEVEL%"
pause
exit /b %VR_EXIT%
:no_python
echo Install Python 3.10 or newer for Windows from https://www.python.org/downloads/windows/
echo Then reopen your terminal and try again.
pause
exit /b 1
:failed
echo Setup failed. Read the error above and the connection guide in docs.
pause
exit /b 1
