@echo off
rem Robocar dashboard in the browser: double-click, or run  tools\dashboard\dashboard.cmd
rem Needs Python 3.8+ on PATH (python or py). Ctrl+C in this window quits.
where python >nul 2>nul
if %errorlevel%==0 (
    python "%~dp0tel_web.py" %*
) else (
    py -3 "%~dp0tel_web.py" %*
)
