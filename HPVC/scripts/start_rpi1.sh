#!/bin/bash
set -e

sudo ip link set eth0 up
sudo ip addr flush dev eth0
sudo ip addr add 192.168.10.1/24 dev eth0
sudo ip route replace 192.168.10.0/24 dev eth0 src 192.168.10.1
sudo ip neigh flush dev eth0 || true
sudo ip neigh replace 192.168.10.11 lladdr 00:00:0c:11:11:11 dev eth0 nud permanent

sudo pkill -f '^./RPI1Deployment\.elf$' || true

cd ~/MATLAB_ws/R2026a/Users/taegon/Documents/MATLAB/Examples/R2026a/autonomous_control/RCCarAutonomousSystem

sudo nohup ./RPI1Deployment.elf > /tmp/RPI1Deployment.out 2>/tmp/RPI1Deployment.err &

sleep 2

sudo ip link set eth0 up
sudo ip addr replace 192.168.10.1/24 dev eth0
sudo ip route replace 192.168.10.0/24 dev eth0 src 192.168.10.1
sudo ip neigh replace 192.168.10.11 lladdr 00:00:0c:11:11:11 dev eth0 nud permanent

echo "=== eth0 ==="
ip -br addr show eth0
ip link show eth0

echo "=== route to TC375 ==="
ip route get 192.168.10.11 || true
ip neigh show 192.168.10.11 dev eth0 || true

echo "=== RPI1Deployment ==="
pgrep -af RPI1Deployment || true

echo "=== UDP ports ==="
ss -lunp | grep -E '5005|5101|5102|5011' || true
