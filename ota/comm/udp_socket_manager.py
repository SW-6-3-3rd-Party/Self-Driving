import socket


class UDPSocketManager:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, packet: bytes, ip: str, port: int):
        self.sock.sendto(packet, (ip, port))
