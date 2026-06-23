from enum import Enum
from package_validator import PackageValidator
from update_stager import UpdateStager
from version_manager import VersionManager


class OTAState(Enum):
    IDLE = "IDLE"
    PACKAGE_RECEIVED = "PACKAGE_RECEIVED"
    VERIFYING = "VERIFYING"
    READY_TO_UPDATE = "READY_TO_UPDATE"
    STAGING = "STAGING"
    DISTRIBUTING = "DISTRIBUTING"
    INSTALLING = "INSTALLING"
    VALIDATING = "VALIDATING"
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"


class OTAManager:
    def __init__(self):
        self.state = OTAState.IDLE
        self.progress = 0
        self.error_code = None
        self.verified_dir = None

    def set_state(self, state: OTAState, progress: int):
        self.state = state
        self.progress = progress
        print(f"[OTA] state={self.state.value}, progress={self.progress}%")

    def fail_update(self, error_code: str):
        self.error_code = error_code
        self.set_state(OTAState.FAILED, self.progress)
        print(f"[OTA ERROR] {self.error_code}")

    def start_update(self):
        try:
            self.set_state(OTAState.PACKAGE_RECEIVED, 10)

            self.set_state(OTAState.VERIFYING, 30)

            version_manager = VersionManager(
                version_path="config/device_versions.json",
                manifest_path="updates/incoming/manifest.json"
            )
            version_manager.check_update_required()

            validator = PackageValidator(
                manifest_path="updates/incoming/manifest.json",
                package_dir="updates/incoming"
            )
            validator.validate_package()

            self.set_state(OTAState.STAGING, 45)
            stager = UpdateStager(
                manifest_path="updates/incoming/manifest.json",
                package_dir="updates/incoming",
                verified_root="updates/verified"
            )
            self.verified_dir = stager.stage()

            self.set_state(OTAState.READY_TO_UPDATE, 50)
            print(f"[OTA] verified package path={self.verified_dir}")

            self.set_state(OTAState.DISTRIBUTING, 70)
            print("[MOCK FOTA] FRONT_ZONE update success")
            print("[MOCK FOTA] REAR_ZONE update success")

            self.set_state(OTAState.INSTALLING, 90)
            print("[MOCK SOTA] HPVC app update success")

            self.set_state(OTAState.VALIDATING, 95)
            version_manager.apply_target_versions_mock()
            version_manager.validate_updated_versions()

            self.set_state(OTAState.SUCCESS, 100)

        except Exception as e:
            self.fail_update(str(e))


if __name__ == "__main__":
    ota_manager = OTAManager()
    ota_manager.start_update()
