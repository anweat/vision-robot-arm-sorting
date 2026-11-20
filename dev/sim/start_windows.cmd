@echo off
setlocal EnableExtensions EnableDelayedExpansion
rem ============================================================================================
rem  Windows one-click start: simulator console (port 8771, A4 map with two bins) + gateway web
rem  page (port 8093), then the browser. Double-click it; close the two windows to stop.
rem  Uses only what is in the repository (detector model, block faces and arm meshes are committed).
rem    start_windows.cmd            start and open the browser
rem    start_windows.cmd nobrowser  start only
rem  Needs Python with uv (if missing:  python -m pip install uv ). The first run lets uv download
rem  opencv / numpy / onnxruntime (~70 MB). Simulation only: not a statement about the real arm.
rem  This file is ASCII on purpose: cmd misreads UTF-8 text in batch files.
rem ============================================================================================

cd /d "%~dp0..\.."
set "SIM_PORT=8771"
set "WEB_PORT=8093"
set "ARM_MAP=map_a4_v4"
set "PYTHONUTF8=1"
if not exist "data\web" mkdir "data\web"

rem ---- 1. app\data must point at data: a symlink in git, checked out on Windows as a small text file ----
fsutil reparsepoint query "app\data" >nul 2>&1
if errorlevel 1 (
    if exist "app\data\" (
        echo app\data is a plain folder, not a link to data. Move its contents into data\ and delete it first.
        pause
        exit /b 1
    )
    if exist "app\data" del /f /q "app\data"
    mklink /J "app\data" "data" >nul
    if errorlevel 1 (
        echo Could not create the app\data junction.
        pause
        exit /b 1
    )
    git update-index --skip-worktree app/data >nul 2>&1
    echo app\data is now a junction to data.
)

rem ---- 2. a command that runs uv (the first python on PATH may not have it when double-clicked) ----
set "UVCMD="
for %%P in ("%USERPROFILE%\anaconda3\python.exe" "%USERPROFILE%\miniconda3\python.exe" "%LOCALAPPDATA%\Programs\Python\Python314\python.exe" "%LOCALAPPDATA%\Programs\Python\Python313\python.exe") do (
    if not defined UVCMD if exist "%%~P" (
        "%%~P" -m uv --version >nul 2>&1 && set "UVCMD=%%~sP -m uv"
    )
)
if not defined UVCMD (
    where uv >nul 2>&1 && set "UVCMD=uv"
)
if not defined UVCMD (
    python -m uv --version >nul 2>&1 && set "UVCMD=python -m uv"
)
if not defined UVCMD (
    py -m uv --version >nul 2>&1 && set "UVCMD=py -m uv"
)
if not defined UVCMD (
    echo uv not found. Install it first:  python -m pip install uv
    pause
    exit /b 1
)
echo Using: %UVCMD%
set "WITH=--no-project --with opencv-python<5 --with numpy --with onnxruntime"

rem ---- 3. two windows: simulator console, gateway ----
start "arm-sim (8771)" cmd /k %UVCMD% run --no-project --with "opencv-python<5" --with numpy --with onnxruntime dev/sim/sim_server.py --port %SIM_PORT% --speed 4 --map map_a4_v4 --bins bins_acrylic_v2_pair
start "arm-gateway (8093)" cmd /k %UVCMD% run --no-project python app/src/web/serve.py --console http://127.0.0.1:%SIM_PORT% --log data/web/log.jsonl --port %WEB_PORT% --flow-python "%UVCMD% run !WITH! python" --pipeline-args "--bins bins_acrylic_v2_pair"

if /i "%~1"=="nobrowser" goto :done

rem ---- 4. wait for the simulator, then open the page ----
echo Waiting for the simulator (the first run downloads its packages and can take a few minutes)...
for /l %%i in (1,1,300) do (
    curl -s -o nul http://127.0.0.1:%SIM_PORT%/api/status && goto :open
    ping -n 3 127.0.0.1 >nul
)
echo The simulator did not come up within 10 minutes: see the "arm-sim (8771)" window.
pause
exit /b 1

:open
start "" http://127.0.0.1:%WEB_PORT%/
:done
echo Started: page http://127.0.0.1:%WEB_PORT%/  simulator http://127.0.0.1:%SIM_PORT%/sim/  (close the two windows to stop)
endlocal
