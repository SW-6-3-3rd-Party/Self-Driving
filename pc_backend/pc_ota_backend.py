from flask import Flask, jsonify, request, send_from_directory
import hashlib
import json
import os
import threading
from datetime import datetime
from pathlib import Path

import paho.mqtt.client as mqtt


app = Flask(__name__)

# =========================
# PC OTA Backend 설정
# =========================

BASE_DIR = Path(__file__).resolve().parents[1]
PACKAGES_DIR = BASE_DIR / "packages"

# HPVC가 접근할 PC Wi-Fi IP
PC_ARTIFACT_HOST = os.environ.get("PC_ARTIFACT_HOST", "192.168.137.1")

# Flask Backend + Artifact HTTP Server port
PC_BACKEND_PORT = int(os.environ.get("PC_BACKEND_PORT", "8080"))

# PC 내부 Mosquitto Broker
MQTT_BROKER_HOST = os.environ.get("MQTT_BROKER_HOST", "192.168.137.1")
MQTT_BROKER_PORT = int(os.environ.get("MQTT_BROKER_PORT", "1883"))

TOPIC_JOB = "hpvc/ota/job"
TOPIC_STATUS = "hpvc/ota/status"
TOPIC_RESULT = "hpvc/ota/result"
TOPIC_VERSION = "hpvc/ota/version"
TOPIC_HEARTBEAT = "hpvc/ota/heartbeat"

PACKAGES_DIR.mkdir(parents=True, exist_ok=True)

mqtt_client = None
mqtt_connected = False

state_lock = threading.Lock()

latest_status = {
    "state": "UNKNOWN",
    "progress": 0,
    "running": False,
    "source": "pc_ota_backend",
}

latest_result = None
latest_version = None
latest_heartbeat = None
latest_job = None


# =========================
# 공통 유틸
# =========================

def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()

    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(chunk)

    return digest.hexdigest()


def default_filename(target: str, target_version: str) -> str:
    if target == "HPVC":
        return f"hpvc_{target_version}.zip"

    if target == "FRONT_ZONE":
        return f"front_zone_{target_version}.bin"

    if target == "REAR_ZONE":
        return f"rear_zone_{target_version}.bin"

    if target == "CENTER_RPI":
        return f"center_rpi_{target_version}.zip"

    return f"{target.lower()}_{target_version}.zip"


def make_job_id(target: str) -> str:
    now = datetime.now().strftime("%Y%m%d-%H%M%S")
    return f"ota-{target.lower()}-{now}"


def make_artifact_url(filename: str) -> str:
    return f"http://{PC_ARTIFACT_HOST}:{PC_BACKEND_PORT}/packages/{filename}"


def publish_ota_job(job: dict):
    global mqtt_connected

    if mqtt_client is None or not mqtt_connected:
        raise RuntimeError("MQTT broker is not connected")

    payload = json.dumps(job)

    info = mqtt_client.publish(
        TOPIC_JOB,
        payload,
        qos=1,
        retain=False,
    )

    info.wait_for_publish()

    if info.rc != mqtt.MQTT_ERR_SUCCESS:
        raise RuntimeError(f"MQTT publish failed rc={info.rc}")


# =========================
# MQTT Callback
# =========================

def on_connect(client, userdata, flags, rc):
    global mqtt_connected

    if rc == 0:
        mqtt_connected = True
        print("[PC OTA BACKEND] MQTT connected")

        client.subscribe(TOPIC_STATUS, qos=1)
        client.subscribe(TOPIC_RESULT, qos=1)
        client.subscribe(TOPIC_VERSION, qos=1)
        client.subscribe(TOPIC_HEARTBEAT, qos=1)

        print("[PC OTA BACKEND] subscribed OTA status/result/version/heartbeat topics")

    else:
        mqtt_connected = False
        print(f"[PC OTA BACKEND] MQTT connect failed rc={rc}")


def on_disconnect(client, userdata, rc):
    global mqtt_connected
    mqtt_connected = False
    print(f"[PC OTA BACKEND] MQTT disconnected rc={rc}")


def on_message(client, userdata, msg):
    global latest_status, latest_result, latest_version, latest_heartbeat

    topic = msg.topic
    payload_text = msg.payload.decode("utf-8")

    try:
        payload = json.loads(payload_text)
    except json.JSONDecodeError:
        payload = {
            "raw": payload_text,
        }

    with state_lock:
        if topic == TOPIC_STATUS:
            latest_status = payload

        elif topic == TOPIC_RESULT:
            latest_result = payload

        elif topic == TOPIC_VERSION:
            latest_version = payload

        elif topic == TOPIC_HEARTBEAT:
            latest_heartbeat = payload

    print(f"[PC OTA BACKEND] MQTT RX topic={topic}, payload={payload_text}")


def start_mqtt_client():
    global mqtt_client

    mqtt_client = mqtt.Client(client_id="pc-ota-backend")
    mqtt_client.on_connect = on_connect
    mqtt_client.on_disconnect = on_disconnect
    mqtt_client.on_message = on_message

    print(
        f"[PC OTA BACKEND] connecting MQTT broker "
        f"{MQTT_BROKER_HOST}:{MQTT_BROKER_PORT}"
    )

    mqtt_client.connect(
        MQTT_BROKER_HOST,
        MQTT_BROKER_PORT,
        30,
    )

    mqtt_client.loop_start()


# =========================
# CORS
# =========================

@app.after_request
def add_cors_headers(response):
    response.headers["Access-Control-Allow-Origin"] = "*"
    response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type"
    return response


# =========================
# Artifact HTTP Server
# =========================

@app.route("/packages/<path:filename>", methods=["GET"])
def serve_package(filename):
    return send_from_directory(PACKAGES_DIR, filename, as_attachment=False)


# =========================
# PC OTA Backend API
# =========================

@app.route("/api/ota/health", methods=["GET"])
def health():
    return jsonify({
        "pc_ota_backend": "running",
        "mqtt_connected": mqtt_connected,
        "mqtt_broker": f"{MQTT_BROKER_HOST}:{MQTT_BROKER_PORT}",
        "artifact_base_url": f"http://{PC_ARTIFACT_HOST}:{PC_BACKEND_PORT}/packages",
        "packages_dir": str(PACKAGES_DIR),
    })


@app.route("/api/ota/packages", methods=["GET"])
def list_packages():
    files = []

    for path in sorted(PACKAGES_DIR.glob("*")):
        if path.is_file():
            files.append({
                "filename": path.name,
                "size": path.stat().st_size,
                "sha256": sha256_file(path),
                "artifact_url": make_artifact_url(path.name),
            })

    return jsonify({
        "packages": files,
    })


@app.route("/api/ota/start", methods=["POST"])
def start_ota():
    global latest_job

    data = request.get_json(silent=True)
    if data is None:
        if request.form:
            data = request.form.to_dict()
        elif request.args:
            data = request.args.to_dict()
        else:
            raw_body = request.get_data(as_text=True)
            if raw_body.strip():
                return jsonify({
                    "result": "rejected",
                    "reason": "invalid JSON body",
                    "raw_body": raw_body,
                    "hint": (
                        "Use Invoke-RestMethod, or send query parameters such as "
                        "/api/ota/start?target=REAR_ZONE&target_version=2.0.1"
                    ),
                }), 400
            data = {}

    target = data.get("target", "HPVC")
    target_version = data.get("target_version", "2.0.0")
    filename = data.get("filename") or default_filename(target, target_version)
    job_id = data.get("job_id") or make_job_id(target)
    rollback_enabled = bool(data.get("rollback_enabled", True))

    package_path = PACKAGES_DIR / filename

    if not package_path.exists():
        return jsonify({
            "result": "rejected",
            "reason": f"package not found: {package_path}",
            "hint": "Put the OTA package file into the packages directory.",
        }), 404

    artifact_url = make_artifact_url(filename)
    sha256 = sha256_file(package_path)

    job = {
        "job_id": job_id,
        "target": target,
        "target_version": target_version,
        "artifact_url": artifact_url,
        "sha256": sha256,
        "size": package_path.stat().st_size,
        "rollback_enabled": rollback_enabled,
    }

    try:
        publish_ota_job(job)

    except Exception as exc:
        return jsonify({
            "result": "failed",
            "reason": str(exc),
            "job": job,
        }), 500

    with state_lock:
        latest_job = job

    return jsonify({
        "result": "published",
        "topic": TOPIC_JOB,
        "job": job,
    })


@app.route("/api/ota/status", methods=["GET"])
def get_ota_status():
    with state_lock:
        return jsonify({
            "mqtt_connected": mqtt_connected,
            "latest_job": latest_job,
            "latest_status": latest_status,
            "latest_result": latest_result,
            "latest_version": latest_version,
            "latest_heartbeat": latest_heartbeat,
        })


@app.route("/api/ota/result", methods=["GET"])
def get_ota_result():
    with state_lock:
        return jsonify({
            "latest_result": latest_result,
        })


@app.route("/api/ota/version", methods=["GET"])
def get_ota_version():
    with state_lock:
        return jsonify({
            "latest_version": latest_version,
        })


if __name__ == "__main__":
    start_mqtt_client()

    print(f"[PC OTA BACKEND] packages_dir={PACKAGES_DIR}")
    print(f"[PC OTA BACKEND] artifact URL base=http://{PC_ARTIFACT_HOST}:{PC_BACKEND_PORT}/packages")
    print(f"[PC OTA BACKEND] HTTP API running on 0.0.0.0:{PC_BACKEND_PORT}")

    app.run(
        host="0.0.0.0",
        port=PC_BACKEND_PORT,
        debug=False,
    )
