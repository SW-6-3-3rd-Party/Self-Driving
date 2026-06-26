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

# HPVC가 이 버전 이상이면 AEB를 안전 기능으로 자동 활성화한다.
# 프로젝트 시나리오:
# - OTA 전: LKAS/LCA 사용 가능, ACC/AEB 비활성
# - OTA 후: AEB 자동 활성, ACC는 PC 버튼으로 활성
AEB_AUTO_ENABLE_MIN_VERSION = "2.0.1"


def parse_version(version: str):
    """
    '2.0.1' 같은 semantic version 문자열을 비교 가능한 tuple로 변환한다.
    잘못된 값이면 (0, 0, 0)을 반환한다.
    """
    try:
        parts = [int(part) for part in str(version).strip().split(".")]
    except (TypeError, ValueError):
        return (0, 0, 0)

    while len(parts) < 3:
        parts.append(0)

    return tuple(parts[:3])


def get_hpvc_version():
    """
    config/device_versions.json에서 HPVC 버전을 읽는다.
    읽기 실패 시 '0.0.0'을 반환한다.
    """
    if not VERSION_PATH.exists():
        return "0.0.0"

    try:
        with open(VERSION_PATH, "r", encoding="utf-8") as f:
            versions = json.load(f)

        return str(versions.get("HPVC", "0.0.0"))
    except Exception as e:
        print(f"[VERSION] failed to read HPVC version: {e}")
        return "0.0.0"


def is_aeb_auto_enabled_by_version():
    """
    HPVC 버전이 기준 버전 이상이면 AEB 자동 활성화 대상이다.
    """
    hpvc_version = get_hpvc_version()
    return parse_version(hpvc_version) >= parse_version(AEB_AUTO_ENABLE_MIN_VERSION)


def get_initial_aeb_active():
    """
    서버 시작 시점의 AEB 초기값.
    OTA 후 버전이면 true, OTA 전 버전이면 false.
    """
    return is_aeb_auto_enabled_by_version()


vehicle_status = {
    "current_mode": "MANUAL",
    "drive_command": "STOP",
    "steering_command": "STRAIGHT",
    "target_speed": 0.0,
    "turn_signal": "OFF",
    "turn_request": 0,

    # Feature enable flags
    #
    # 주의:
    # - PC 화면에서 LKAS 버튼은 LKAS + LCA를 함께 ON/OFF 한다.
    # - ACC 버튼은 ACC만 독립적으로 ON/OFF 한다.
    # - AEB는 HPVC 버전이 AEB_AUTO_ENABLE_MIN_VERSION 이상이면 자동 ON 된다.
    # - 아래 *_active 필드는 기존 HPVC control loop / feature_manager와의 호환을 위해 유지한다.
    # - API 응답 기준값은 feature_enable_flags를 사용한다.
    "lkas_active": False,
    "lca_active": False,
    "acc_active": False,
    "aeb_active": get_initial_aeb_active(),

    "last_pc_command_time": None,

    "last_final_command": None,
    "last_front_zone_command": None,
    "last_rear_zone_command": None,
    "last_feature_request": None,
    "last_feature_flag_update": None,
    "control_loop_count": 0,
}

# PC 버튼으로 설정한 기능 enable 상태의 기준 저장소.
# control loop가 vehicle_status["*_active"]를 feature output으로 덮어쓰더라도
# PC 화면과 /features/status는 이 값을 기준으로 표시한다.
feature_enable_flags = {
    "lkas_active": False,
    "lca_active": False,
    "acc_active": False,
    "aeb_active": get_initial_aeb_active(),
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


def apply_aeb_auto_activation_policy():
    """
    OTA 이후 HPVC 버전이 기준 이상이면 AEB를 자동 활성화한다.
    AEB는 PC 버튼으로 켜고 끄는 기능이 아니라 안전 기능이므로,
    버전 조건을 만족하면 외부 요청으로 OFF 되지 않도록 true로 유지한다.
    """
    if is_aeb_auto_enabled_by_version():
        feature_enable_flags["aeb_active"] = True


def sync_feature_flags_to_vehicle_status():
    """
    기존 control loop / feature_manager와의 호환을 위해
    API 기준 feature_enable_flags를 vehicle_status에도 반영한다.
    """
    apply_aeb_auto_activation_policy()

    vehicle_status["lkas_active"] = feature_enable_flags["lkas_active"]
    vehicle_status["lca_active"] = feature_enable_flags["lca_active"]
    vehicle_status["acc_active"] = feature_enable_flags["acc_active"]
    vehicle_status["aeb_active"] = feature_enable_flags["aeb_active"]


def get_feature_flags_snapshot():
    return {
        "lkas_active": feature_enable_flags["lkas_active"],
        "lca_active": feature_enable_flags["lca_active"],
        "acc_active": feature_enable_flags["acc_active"],
        "aeb_active": feature_enable_flags["aeb_active"],
        "aeb_auto_enabled": is_aeb_auto_enabled_by_version(),
        "aeb_auto_min_version": AEB_AUTO_ENABLE_MIN_VERSION,
        "hpvc_version": get_hpvc_version(),
        "last_feature_flag_update": vehicle_status.get("last_feature_flag_update"),
        "last_feature_request": vehicle_status.get("last_feature_request"),
    }


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
@app.route("/features", methods=["POST"])
def set_feature_flags():
    data = request.get_json(silent=True) or {}

    with status_lock:
        # LKAS는 LCA와 묶어서 제어한다.
        # 요청에 lkas_active 또는 lkas_enabled가 들어온 경우에만 변경한다.
        if "lkas_active" in data:
            lkas_active = bool(data["lkas_active"])
            feature_enable_flags["lkas_active"] = lkas_active
            feature_enable_flags["lca_active"] = lkas_active

        if "lkas_enabled" in data:
            lkas_active = bool(data["lkas_enabled"])
            feature_enable_flags["lkas_active"] = lkas_active
            feature_enable_flags["lca_active"] = lkas_active

        # LCA 단독 제어는 허용하지 않는다.
        # 외부에서 lca_active/lca_enabled가 들어와도 LKAS 상태를 따른다.
        if (
            ("lca_active" in data or "lca_enabled" in data)
            and "lkas_active" not in data
            and "lkas_enabled" not in data
        ):
            feature_enable_flags["lca_active"] = feature_enable_flags["lkas_active"]

        # ACC는 LKAS/LCA와 독립적으로 제어한다.
        # 요청에 acc_active 또는 acc_enabled가 들어온 경우에만 변경한다.
        if "acc_active" in data:
            feature_enable_flags["acc_active"] = bool(data["acc_active"])

        if "acc_enabled" in data:
            feature_enable_flags["acc_active"] = bool(data["acc_enabled"])

        # AEB는 OTA 이후 자동 활성화되는 안전 기능이다.
        # 버전 조건을 만족하면 외부 요청으로 OFF 되지 않는다.
        if "aeb_active" in data or "aeb_enabled" in data:
            requested_aeb = bool(data.get("aeb_active", data.get("aeb_enabled", False)))

            if is_aeb_auto_enabled_by_version():
                feature_enable_flags["aeb_active"] = True
            else:
                feature_enable_flags["aeb_active"] = requested_aeb

        vehicle_status["last_feature_flag_update"] = {
            "applied_at": time.time(),
            "raw_request": data,
        }

        # 기존 control loop / feature_manager와의 호환을 위해 vehicle_status에도 반영
        sync_feature_flags_to_vehicle_status()

        result = get_feature_flags_snapshot()

    print(f"[FEATURE FLAGS] {result}")

    return jsonify({
        "result": "accepted",
        "features": result,
        "feature_flags": result,
    })


@app.route("/features", methods=["GET"])
@app.route("/features/status", methods=["GET"])
def get_feature_status():
    with status_lock:
        # 조회 시에도 한 번 더 동기화하여,
        # control loop가 vehicle_status["*_active"]를 덮어쓴 경우를 보정한다.
        sync_feature_flags_to_vehicle_status()
        feature_flags = get_feature_flags_snapshot()

    feature_debug = feature_manager.get_debug_status()

    return jsonify({
        "feature_flags": feature_flags,
        "features": feature_flags,
        "feature_debug": feature_debug,
    })


@app.route("/vehicle/status", methods=["GET"])
def get_vehicle_status():
    with status_lock:
        sync_feature_flags_to_vehicle_status()
        status = dict(vehicle_status)
        status["feature_flags"] = get_feature_flags_snapshot()

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
