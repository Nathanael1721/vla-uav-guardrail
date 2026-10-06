@echo off
rem Start the grant tracker on this PC and open it in the browser.
cd /d "%~dp0.."
set PY=C:\Users\natha\.conda\envs\vla-real\python.exe
if not exist "%PY%" set PY=python
start "" http://127.0.0.1:8765/
"%PY%" tracker\serve.py --port 8765
