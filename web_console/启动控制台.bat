@echo off
title ArmPi Pro 任务控制台
cd /d "%~dp0"

echo ============================================
echo    ArmPi Pro 任务控制台  -  一键启动
echo ============================================
echo.

set "PY=C://Users//Administrator//.workbuddy//binaries//python//envs//default//Scripts//python.exe"

if not exist "%PY%" goto nopython

netstat -ano | findstr ":8000" | findstr "LISTENING" >nul 2>&1
if not errorlevel 1 goto already

echo [1/2] 正在启动本地代理 127.0.0.1:8000 ...
start "armpi-proxy" /min "%PY%" "proxy_server(1).py"

set N=0
:waitloop
netstat -ano | findstr ":8000" | findstr "LISTENING" >nul 2>&1
if not errorlevel 1 goto ready
set /a N=N+1
if %N% GEQ 15 goto waitfail
ping -n 2 -w 1000 127.0.0.1 >nul
goto waitloop

:waitfail
echo [警告] 代理 15 秒内未监听 8000 端口，仍继续打开页面。
goto open

:already
echo [1/2] 本地代理已经在运行 ^(端口 8000^)，跳过启动。
goto open

:ready
echo [1/2] 代理已就绪。

:open
echo [2/2] 打开浏览器 http://127.0.0.1:8000
start "" "http://127.0.0.1:8000"
echo.
echo ------------------------------------------------------------
echo  页面里要做的事：
echo    1. 小车 IP 填当前地址（例如 10.120.150.178） -^> 点「连接」
echo    2. 画面源选「深度相机RGB /ascamera_hp60c/rgb0/image」
echo.
echo  小车 IP 又变了？在这台电脑上执行  ping raspberrypi  可查。
echo  小车也重启过？相机节点要手动拉起 —— 完整步骤见  开机恢复.md
echo ------------------------------------------------------------
echo.
echo  关闭本窗口不会停掉代理；要停代理，关掉最小化的 armpi-proxy 窗口。
timeout /t 10 /nobreak >nul
exit /b 0

:nopython
echo [错误] 找不到 Python：
echo   %PY%
echo.
echo 请用记事本打开本文件，把 PY= 那一行改成你机器上真实的 python.exe 路径。
echo.
pause
exit /b 1
