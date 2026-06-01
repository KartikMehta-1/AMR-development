#!/usr/bin/env bash
set -euo pipefail

ORIN_HOST="${ORIN_HOST:-orin}"
ORIN_CONTAINER_NAME="${AMR_ORIN_HW_CONTAINER:-amr_orin_hw}"
NUC_IP="${NUC_IP:-192.168.1.8}"
ORIN_IP="${ORIN_IP:-192.168.1.20}"
ORIN_TO_NUC_PORT="${ORIN_TO_NUC_PORT:-$((30000 + RANDOM % 10000))}"
NUC_TO_ORIN_PORT="${NUC_TO_ORIN_PORT:-$((40000 + RANDOM % 10000))}"

tmp_dir="$(mktemp -d)"
cleanup() {
  rm -rf "${tmp_dir}"
}
trap cleanup EXIT

echo "Testing UDP Orin -> NUC (${ORIN_IP} -> ${NUC_IP}:${ORIN_TO_NUC_PORT})"
python3 - <<PY >"${tmp_dir}/nuc_recv.out" 2>&1 &
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("0.0.0.0", ${ORIN_TO_NUC_PORT}))
open("${tmp_dir}/nuc_recv.ready", "w").close()
s.settimeout(8)
try:
    data, addr = s.recvfrom(2048)
    print("PASS", data.decode(errors="replace"), addr, flush=True)
except Exception as exc:
    print("FAIL", repr(exc), flush=True)
PY
recv_pid=$!
for _ in $(seq 1 50); do
  [ -f "${tmp_dir}/nuc_recv.ready" ] && break
  sleep 0.1
done
ssh -n "${ORIN_HOST}" "docker exec '${ORIN_CONTAINER_NAME}' python3 -c \"import socket; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(('${ORIN_IP}', 0)); s.sendto(b'amr orin to nuc udp probe', ('${NUC_IP}', ${ORIN_TO_NUC_PORT})); print('sent from', s.getsockname(), flush=True)\""
wait "${recv_pid}" || true
cat "${tmp_dir}/nuc_recv.out"

echo
echo "Testing UDP NUC -> Orin (${NUC_IP} -> ${ORIN_IP}:${NUC_TO_ORIN_PORT})"
ssh -n "${ORIN_HOST}" "docker exec '${ORIN_CONTAINER_NAME}' bash -lc \"cat > /tmp/amr_udp_recv_orin.py <<'PY'
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('0.0.0.0', ${NUC_TO_ORIN_PORT}))
s.settimeout(8)
try:
    data, addr = s.recvfrom(2048)
    print('PASS', data.decode(errors='replace'), addr, flush=True)
except Exception as exc:
    print('FAIL', repr(exc), flush=True)
PY
nohup python3 /tmp/amr_udp_recv_orin.py >/tmp/amr_udp_recv_orin.out 2>&1 &\""
sleep 1
python3 - <<PY
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.sendto(b'amr nuc to orin udp probe', ("${ORIN_IP}", ${NUC_TO_ORIN_PORT}))
print("sent")
PY
sleep 9
ssh -n "${ORIN_HOST}" "docker exec '${ORIN_CONTAINER_NAME}' cat /tmp/amr_udp_recv_orin.out"
