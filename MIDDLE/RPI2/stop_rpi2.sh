#!/bin/sh

pkill -f '[.]venv/bin/python3 app.py' 2>/dev/null || true
pkill -f '[p]ython3 app.py' 2>/dev/null || true
