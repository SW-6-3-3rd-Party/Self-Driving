#!/bin/sh
set -eu

cd "/home/taegonkim/Desktop/MIDDLE"
pkill -f '[.]venv/bin/python3 app.py' 2>/dev/null || true
pkill -f '[p]ython3 app.py' 2>/dev/null || true

exec ./.venv/bin/python3 app.py \
  --camera /dev/video0 \
  --udp-host 192.168.202.98 \
  --udp-port 5005 \
  --udp-source-port 5006 \
  --left-trigger 23 \
  --left-echo 24 \
  --right-trigger 17 \
  --right-echo 27
