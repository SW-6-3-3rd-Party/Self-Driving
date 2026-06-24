import json
import threading
import time

import paho.mqtt.client as mqtt

from config_paths import CONFIG_DIR


class MQTTOTABridge:
    def __init__(self, ota_manager):
        self.ota_manager = ota_manager

        self.config = self._load_config()
        self.enabled = bool(self.config.get("enable_mqtt", False))

        self.broker_host = self.config.get("broker_host", "127.0.0.1")
        self.broker_port = int(self.config.get("broker_port", 1883))
        self.client_id = self.config.get("client_id", "hpvc-ota-agent")
        self.keepalive = int(self.config.get("keepalive", 30))
        self.topics = self.config.get("topics", {})

        self.topic_job = self.topics.get("job", "hpvc/ota/job")
        self.topic_status = self.topics.get("status", "hpvc/ota/status")
        self.topic_result = self.topics.get("result", "hpvc/ota/result")
        self.topic_version = self.topics.get("version", "hpvc/ota/version")
        self.topic_heartbeat = self.topics.get(
            "heartbeat",
            "hpvc/ota/heartbeat",
        )

        self.client = None
        self.running = False
        self.heartbeat_thread = None

    def _load_config(self) -> dict:
        config_path = CONFIG_DIR / "mqtt_config.json"

        if not config_path.exists():
            return {
                "enable_mqtt": False,
            }

        with open(config_path, "r", encoding="utf-8") as f:
            return json.load(f)

    def start(self):
        if not self.enabled:
            print("[MQTT OTA] disabled")
            return

        self.client = mqtt.Client(client_id=self.client_id)

        self.client.will_set(
            self.topic_heartbeat,
            payload=json.dumps({
                "client_id": self.client_id,
                "status": "offline",
            }),
            qos=1,
            retain=True,
        )

        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect

        print(
            f"[MQTT OTA] connecting broker={self.broker_host}:{self.broker_port}"
        )

        try:
            self.client.connect(
                self.broker_host,
                self.broker_port,
                self.keepalive,
            )
        except OSError as exc:
            print(f"[MQTT OTA] broker connection failed: {exc}")
            print("[MQTT OTA] server will continue without MQTT OTA")
            return

        self.running = True
        self.client.loop_start()

        self.heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            daemon=True,
        )
        self.heartbeat_thread.start()

    def stop(self):
        self.running = False

        if self.client:
            self.publish_heartbeat("offline")
            self.client.loop_stop()
            self.client.disconnect()

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        rc_value = getattr(rc, "value", rc)

        if rc_value == 0:
            print("[MQTT OTA] connected")

            client.subscribe(self.topic_job, qos=1)

            print(f"[MQTT OTA] subscribed topic={self.topic_job}")

            self.publish_heartbeat("online")
            self.publish_status(self.ota_manager.get_status())

        else:
            print(f"[MQTT OTA] connect failed rc={rc}")

    def _on_disconnect(self, client, userdata, rc, properties=None):
        print(f"[MQTT OTA] disconnected rc={rc}")

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        payload_text = msg.payload.decode("utf-8")

        print(f"[MQTT OTA] message topic={topic}, payload={payload_text}")

        if topic == self.topic_job:
            self._handle_ota_job(payload_text)

    def _handle_ota_job(self, payload_text: str):
        try:
            job = json.loads(payload_text)

            if self.ota_manager.running:
                self.publish_result({
                    "result": "REJECTED",
                    "reason": "OTA already running",
                    "job": job,
                })
                return

            thread = threading.Thread(
                target=self._run_ota_job,
                args=(job,),
                daemon=True,
            )
            thread.start()

        except Exception as exc:
            self.publish_result({
                "result": "REJECTED",
                "reason": str(exc),
                "raw_payload": payload_text,
            })

    def _run_ota_job(self, job: dict):
        result = self.ota_manager.start_update(
            job=job,
            status_callback=self.publish_status,
        )

        self.publish_result(result)
        self.publish_status(self.ota_manager.get_status())

    def publish_status(self, status: dict):
        self._publish_json(
            self.topic_status,
            status,
            qos=1,
            retain=True,
        )

    def publish_result(self, result: dict):
        self._publish_json(
            self.topic_result,
            result,
            qos=1,
            retain=False,
        )

    def publish_version(self, version: dict):
        self._publish_json(
            self.topic_version,
            version,
            qos=1,
            retain=True,
        )

    def publish_heartbeat(self, status: str):
        self._publish_json(
            self.topic_heartbeat,
            {
                "client_id": self.client_id,
                "status": status,
                "timestamp": time.time(),
            },
            qos=1,
            retain=True,
        )

    def _heartbeat_loop(self):
        while self.running:
            self.publish_heartbeat("online")
            time.sleep(2.0)

    def _publish_json(
        self,
        topic: str,
        payload: dict,
        qos: int = 0,
        retain: bool = False,
    ):
        if not self.client:
            return

        try:
            self.client.publish(
                topic,
                json.dumps(payload),
                qos=qos,
                retain=retain,
            )

        except Exception as exc:
            print(f"[MQTT OTA] publish failed topic={topic}, error={exc}")
