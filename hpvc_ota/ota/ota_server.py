from flask import Flask, jsonify, request
import json
import threading
import time

from ota_manager import OTAManager
from config_paths import BASE_DIR

from control.control_arbitration import ControlArbitrator
from control.command_sender import CommandSender
from control.control_loop import HPVCControlLoop

from comm.udp_receiver_manager import UDPReceiverManager
from features.feature_manager import FeatureManager
from mqtt_ota_bridge import MQTTOTABridge


app = Flask(__name__)

VERSION_PATH = BASE_DIR / "config" / "device_versions.json"

ALLOWED_DRIVE_COMMANDS = {"FORWARD", "REVERSE", "STOP"}
ALLOWED_STEERING_COMMANDS = {"LEFT", "RIGHT", "STRAIGHT"}
ALLOWED_TURN_SIGNALS = {"OFF", "LEFT", "RIGHT"}
MAX_TARGET_SPEED = 0.5


vehicle_status = {
    "current_mode": "MANUAL",
    "drive_command": "STOP",
    "steering_command": "STRAIGHT",
    "target_speed": 0.0,
    "turn_signal": "OFF",
    "turn_request": 0,

    # Feature enable flags
    "lkas_active": False,
    "lca_active": False,
    "acc_active": False,
    "aeb_active": False,

    "last_pc_command_time": None,

    "last_final_command": None,
    "last_front_zone_command": None,
    "last_rear_zone_command": None,
    "last_feature_request": None,
    "control_loop_count": 0,
}

status_lock = threading.Lock()

manager = OTAManager()
mqtt_ota_bridge = MQTTOTABridge(manager)

udp_receiver_manager = UDPReceiverManager()
feature_manager = FeatureManager()
arbitrator = ControlArbitrator()
command_sender = CommandSender()

control_loop = HPVCControlLoop(
    vehicle_status=vehicle_status,
    status_lock=status_lock,
    udp_receiver_manager=udp_receiver_manager,
    feature_manager=feature_manager,
    arbitrator=arbitrator,
    command_sender=command_sender,
    period_sec=0.05,
)


@app.after_request
def add_cors_headers(response):
    response.headers["Access-Control-Allow-Origin"] = "*"
    response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type"
    return response


def validate_manual_request(data: dict):
    drive_command = data.get("drive_command", "STOP")
    steering_command = data.get("steering_command", "STRAIGHT")
    target_speed = data.get("target_speed", 0.0)

    if drive_command not in ALLOWED_DRIVE_COMMANDS:
        return False, f"invalid drive_command: {drive_command}"

    if steering_command not in ALLOWED_STEERING_COMMANDS:
        return False, f"invalid steering_command: {steering_command}"

    try:
        target_speed = float(target_speed)
    except (TypeError, ValueError):
        return False, "target_speed must be number"

    if target_speed < 0.0 or target_speed > MAX_TARGET_SPEED:
        return False, f"target_speed out of range: {target_speed}"

    if drive_command == "STOP" and target_speed != 0.0:
        return False, "target_speed must be 0.0 when drive_command is STOP"

    return True, None


def validate_turn_signal_request(data: dict):
    turn_signal = data.get("turn_signal", "OFF")

    if turn_signal not in ALLOWED_TURN_SIGNALS:
        return False, f"invalid turn_signal: {turn_signal}"

    return True, None


@app.route("/ota/status", methods=["GET"])
def get_ota_status():
    return jsonify(manager.get_status())


@app.route("/ota/start", methods=["POST"])
def start_ota_deprecated():
    return jsonify({
        "result": "deprecated",
        "message": "OTA start is now handled by MQTT topic hpvc/ota/job",
        "status": manager.get_status(),
    }), 410


@app.route("/version", methods=["GET"])
def get_versions():
    if not VERSION_PATH.exists():
        return jsonify({
            "error": "version file not found",
        }), 404

    with open(VERSION_PATH, "r", encoding="utf-8") as f:
        versions = json.load(f)

    mqtt_ota_bridge.publish_version(versions)

    return jsonify(versions)


@app.route("/control/manual", methods=["POST"])
def manual_control():
    data = request.get_json(silent=True) or {}

    valid, reason = validate_manual_request(data)
    if not valid:
        return jsonify({
            "result": "rejected",
            "reason": reason,
        }), 400

    drive_command = data.get("drive_command", "STOP")
    steering_command = data.get("steering_command", "STRAIGHT")
    target_speed = float(data.get("target_speed", 0.0))

    with status_lock:
        vehicle_status["drive_command"] = drive_command
        vehicle_status["steering_command"] = steering_command
        vehicle_status["target_speed"] = target_speed
        vehicle_status["last_pc_command_time"] = time.time()

    print(
        f"[MANUAL CONTROL] updated "
        f"drive={drive_command}, steering={steering_command}, speed={target_speed}"
    )

    return jsonify({
        "result": "accepted",
        "drive_command": drive_command,
        "steering_command": steering_command,
        "target_speed": target_speed,
    })


@app.route("/control/turn-signal", methods=["POST"])
def turn_signal_control():
    data = request.get_json(silent=True) or {}

    valid, reason = validate_turn_signal_request(data)
    if not valid:
        return jsonify({
            "result": "rejected",
            "reason": reason,
        }), 400

    turn_signal = data.get("turn_signal", "OFF")

    if turn_signal == "LEFT":
        turn_request = 1
    elif turn_signal == "RIGHT":
        turn_request = 2
    else:
        turn_request = 0

    with status_lock:
        vehicle_status["turn_signal"] = turn_signal
        vehicle_status["turn_request"] = turn_request

    print(f"[TURN SIGNAL] updated signal={turn_signal}, turn_request={turn_request}")

    return jsonify({
        "result": "accepted",
        "turn_signal": turn_signal,
        "turn_request": turn_request,
    })


@app.route("/features/enable", methods=["POST"])
def set_feature_flags():
    data = request.get_json(silent=True) or {}

    with status_lock:
        for key in ("lkas_active", "lca_active", "acc_active", "aeb_active"):
            if key in data:
                vehicle_status[key] = bool(data[key])

        result = {
            "lkas_active": vehicle_status["lkas_active"],
            "lca_active": vehicle_status["lca_active"],
            "acc_active": vehicle_status["acc_active"],
            "aeb_active": vehicle_status["aeb_active"],
        }

    print(f"[FEATURE FLAGS] {result}")

    return jsonify({
        "result": "accepted",
        "features": result,
    })


@app.route("/features/status", methods=["GET"])
def get_feature_status():
    with status_lock:
        feature_flags = {
            "lkas_active": vehicle_status["lkas_active"],
            "lca_active": vehicle_status["lca_active"],
            "acc_active": vehicle_status["acc_active"],
            "aeb_active": vehicle_status["aeb_active"],
            "last_feature_request": vehicle_status["last_feature_request"],
        }

    feature_debug = feature_manager.get_debug_status()

    return jsonify({
        "feature_flags": feature_flags,
        "feature_debug": feature_debug,
    })


@app.route("/vehicle/status", methods=["GET"])
def get_vehicle_status():
    with status_lock:
        status = dict(vehicle_status)

    status["ecu_comm_status"] = udp_receiver_manager.get_status()
    status["feature_debug"] = feature_manager.get_debug_status()
    status["ota_status"] = manager.get_status()

    return jsonify(status)


@app.route("/comm/status", methods=["GET"])
def get_comm_status():
    return jsonify(udp_receiver_manager.get_status())


if __name__ == "__main__":
    udp_receiver_manager.start()
    control_loop.start()
    mqtt_ota_bridge.start()

    try:
        app.run(host="0.0.0.0", port=8000)
    finally:
        mqtt_ota_bridge.stop()
        control_loop.stop()
        udp_receiver_manager.stop()
