@echo off
rem Start the grant tracker on this PC and open it in the browser.
rem Already running (127.0.0.1:8765 answers)? Then only open the browser.
rem Otherwise refresh the Dashboard data (--quick: no posters, no codec probes)
rem and start the server, which opens the browser once its port is bound
rem (the old launcher opened it first, so the first load could fail).
cd /d "%~dp0.."
set PY=C:\Users\natha\.conda\envs\vla-real\python.exe
if not exist "%PY%" set PY=python
set PORT=8765
"%PY%" -c "import socket,sys; s=socket.socket(); s.settimeout(0.5); sys.exit(0 if s.connect_ex(('127.0.0.1', %PORT%)) == 0 else 1)"
if %errorlevel%==0 (
  echo The tracker is already running: opening http://127.0.0.1:%PORT%/
  start "" http://127.0.0.1:%PORT%/
  exit /b 0
)
"%PY%" tools\build_dashboard.py --quick
"%PY%" tracker\serve.py --port %PORT% --open
