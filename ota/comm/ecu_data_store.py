import time
from threading import Lock


class ECUDataStore:
    def __init__(self):
        self.lock = Lock()

        self.data = {
            "center": None,
            "front": None,
            "rear": None,
            "heartbeat": {},
            "last_received": {
                "center": None,
                "front": None,
                "rear": None,
                "heartbeat": {},
            },
            "packet_count": {
                "center": 0,
                "front": 0,
                "rear": 0,
                "heartbeat": 0,
            },
            "fault_count": {
                "crc_error": 0,
                "parse_error": 0,
                "unknown_msg": 0,
            },
        }

    def update_center(self, header: dict, payload: dict):
        with self.lock:
            self.data["center"] = {
                "header": header,
                "payload": payload,
            }
            self.data["last_received"]["center"] = time.time()
            self.data["packet_count"]["center"] += 1

    def update_front(self, header: dict, payload: dict):
        with self.lock:
            self.data["front"] = {
                "header": header,
                "payload": payload,
            }
            self.data["last_received"]["front"] = time.time()
            self.data["packet_count"]["front"] += 1

    def update_rear(self, header: dict, payload: dict):
        with self.lock:
            self.data["rear"] = {
                "header": header,
                "payload": payload,
            }
            self.data["last_received"]["rear"] = time.time()
            self.data["packet_count"]["rear"] += 1

    def update_heartbeat(self, header: dict, payload: dict):
        device_id = str(header.get("device_id"))

        with self.lock:
            self.data["heartbeat"][device_id] = {
                "header": header,
                "payload": payload,
            }
            self.data["last_received"]["heartbeat"][device_id] = time.time()
            self.data["packet_count"]["heartbeat"] += 1

    def increment_fault(self, fault_name: str):
        with self.lock:
            if fault_name not in self.data["fault_count"]:
                self.data["fault_count"][fault_name] = 0
            self.data["fault_count"][fault_name] += 1

    def get_snapshot(self) -> dict:
        with self.lock:
            now = time.time()

            snapshot = {
                "center": self.data["center"],
                "front": self.data["front"],
                "rear": self.data["rear"],
                "heartbeat": self.data["heartbeat"],
                "packet_count": self.data["packet_count"],
                "fault_count": self.data["fault_count"],
                "link_status": {},
            }

            snapshot["link_status"]["center"] = self._make_link_status(
                self.data["last_received"]["center"],
                now,
                timeout_sec=0.150,
            )

            snapshot["link_status"]["front"] = self._make_link_status(
                self.data["last_received"]["front"],
                now,
                timeout_sec=0.100,
            )

            snapshot["link_status"]["rear"] = self._make_link_status(
                self.data["last_received"]["rear"],
                now,
                timeout_sec=0.150,
            )

            return snapshot

    def _make_link_status(self, last_received, now, timeout_sec: float):
        if last_received is None:
            return {
                "received": False,
                "timeout": True,
                "age_ms": None,
            }

        age_sec = now - last_received

        return {
            "received": True,
            "timeout": age_sec > timeout_sec,
            "age_ms": int(age_sec * 1000),
        }
