class UDSDoIPClient:
    """
    현업 확장용 skeleton.

    실제 차량에서는 HPVC가 UDS Client / DoIP Client,
    Zone ECU가 UDS Server / DoIP Server 역할을 한다.

    현재 프로젝트에서는 TCP 기반 FOTA 구조를 사용하고,
    이 파일은 UDS over DoIP 확장 지점으로 둔다.
    """

    def __init__(self, ip: str, port: int = 13400):
        self.ip = ip
        self.port = port

    def enter_programming_session(self):
        raise NotImplementedError("UDS DiagnosticSessionControl not implemented")

    def security_access(self):
        raise NotImplementedError("UDS SecurityAccess not implemented")

    def request_download(self):
        raise NotImplementedError("UDS RequestDownload not implemented")

    def transfer_data(self):
        raise NotImplementedError("UDS TransferData not implemented")

    def request_transfer_exit(self):
        raise NotImplementedError("UDS RequestTransferExit not implemented")

    def routine_control_verify(self):
        raise NotImplementedError("UDS RoutineControl not implemented")

    def ecu_reset(self):
        raise NotImplementedError("UDS ECUReset not implemented")
