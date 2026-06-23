from flask import Flask, jsonify, request
import json
import threading
from pathlib import Path

from ota_manager import OTAManager


app = Flask(__name__)

vehicle_status = {
    "current_mode": "MANUAL",
    "drive_command": "STOP",
    "steering_command": "STRAIGHT",
    "target_speed": 0.0,
    "turn_signal": "OFF",
    "turn_request": 0,
    "lkas_active": False,
    "lca_active": False,
    "acc_active": False,
    "aeb_active": False
}

manager = OTAManager()
update_thread = None
is_running = False
lock = threading.Lock()

VERSION_PATH = Path("config/device_versions.json")


@app.after_request
def add_cors_headers(response):
    response.headers["Access-Control-Allow-Origin"] = "*"
    response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type"
    return response


def run_update():
    global is_running

    try:
        manager.start_update()
    finally:
        with lock:
            is_running = False


@app.route("/ota/status", methods=["GET"])
def get_ota_status():
    with lock:
        return jsonify({
            "state": manager.state.value,
            "progress": manager.progress,
            "error_code": manager.error_code,
            "running": is_running
        })


@app.route("/ota/start", methods=["POST"])
def start_ota():
    global manager
    global update_thread
    global is_running

    with lock:
        if is_running:
            return jsonify({
                "result": "rejected",
                "reason": "OTA already running"
            }), 409

        manager = OTAManager()
        is_running = True

        update_thread = threading.Thread(target=run_update)
        update_thread.start()

        return jsonify({
            "result": "accepted",
            "message": "OTA update started"
        })


@app.route("/version", methods=["GET"])
def get_versions():
    if not VERSION_PATH.exists():
        return jsonify({
            "error": "version file not found"
        }), 404

    with open(VERSION_PATH, "r", encoding="utf-8") as f:
        versions = json.load(f)

    return jsonify(versions)


@app.route("/control/manual", methods=["POST"])
def manual_control():
    data = request.get_json(silent=True) or {}

    drive_command = data.get("drive_command", "STOP")
    steering_command = data.get("steering_command", "STRAIGHT")
    target_speed = data.get("target_speed", 0.0)

    vehicle_status["current_mode"] = "MANUAL"
    vehicle_status["drive_command"] = drive_command
    vehicle_status["steering_command"] = steering_command
    vehicle_status["target_speed"] = target_speed

    print(
        f"[MANUAL CONTROL] drive={drive_command}, "
        f"steering={steering_command}, speed={target_speed}"
    )

    return jsonify({
        "result": "accepted",
        "drive_command": drive_command,
        "steering_command": steering_command,
        "target_speed": target_speed
    })


@app.route("/control/turn-signal", methods=["POST"])
def turn_signal_control():
    data = request.get_json(silent=True) or {}

    turn_signal = data.get("turn_signal", "OFF")
    vehicle_status["turn_signal"] = turn_signal

    if turn_signal == "LEFT":
        turn_request = 1
    elif turn_signal == "RIGHT":
        turn_request = 2
    else:
        turn_request = 0

    vehicle_status["turn_request"] = turn_request

    print(f"[TURN SIGNAL] signal={turn_signal}, turn_request={turn_request}")

    return jsonify({
        "result": "accepted",
        "turn_signal": turn_signal,
        "turn_request": turn_request
    })


@app.route("/vehicle/status", methods=["GET"])
def get_vehicle_status():
    return jsonify(vehicle_status)


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8000)
