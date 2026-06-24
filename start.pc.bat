@echo off
cd /d "%~dp0"

echo ==============================
echo HPVC PC HMI Start
echo ==============================

echo Start React HMI...
python start_pc.py

echo ==============================
echo Stopped.
echo ==============================

pause