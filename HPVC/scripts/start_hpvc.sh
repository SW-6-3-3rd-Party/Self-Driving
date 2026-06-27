#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." 2>/dev/null && pwd || true)"

sudo ip link set eth0 up
sudo ip addr flush dev eth0
sudo ip addr add 192.168.10.1/24 dev eth0
sudo ip route replace 192.168.10.0/24 dev eth0 src 192.168.10.1
sudo ip neigh flush dev eth0 || true
sudo ip neigh replace 192.168.10.11 lladdr 02:37:50:ae:b0:01 dev eth0 nud permanent

sudo pkill -f '^./HPVCDeployment\.elf$' || true
sudo pkill -f 'python3 -m HPVC.hpvc_aeb' || true

cd ~/MATLAB_ws/R2026a/Users/taegon/Documents/MATLAB/Examples/R2026a/autonomous_control/RCCarAutonomousSystem

sudo nohup ./HPVCDeployment.elf > /tmp/HPVCDeployment.out 2>/tmp/HPVCDeployment.err &

if [ "${START_HPVC_AEB:-0}" = "1" ]; then
    AEB_ROOT="${HPVC_AEB_ROOT:-$REPO_ROOT}"
    if [ -f "$AEB_ROOT/HPVC/hpvc_aeb.py" ]; then
        cd "$AEB_ROOT"
        sudo nohup python3 -m HPVC.hpvc_aeb ${HPVC_AEB_ARGS:-} \
            > /tmp/hpvc_aeb.out 2>/tmp/hpvc_aeb.err &
    else
        echo "START_HPVC_AEB=1 but HPVC_AEB_ROOT does not point to this repository."
    fi
fi

sleep 2

sudo ip link set eth0 up
sudo ip addr replace 192.168.10.1/24 dev eth0
sudo ip route replace 192.168.10.0/24 dev eth0 src 192.168.10.1
sudo ip neigh replace 192.168.10.11 lladdr 02:37:50:ae:b0:01 dev eth0 nud permanent

echo "=== eth0 ==="
ip -br addr show eth0
ip link show eth0

echo "=== route to TC375 ==="
ip route get 192.168.10.11 || true
ip neigh show 192.168.10.11 dev eth0 || true

echo "=== HPVCDeployment ==="
pgrep -af HPVCDeployment || true

echo "=== UDP ports ==="
ss -lunp | grep -E '5005|5101|5102|5011|5013' || true
