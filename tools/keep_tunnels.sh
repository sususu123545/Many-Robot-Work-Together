#!/usr/bin/env bash
# Keep the two local forwards the tools rely on alive:
#   localhost:19091 -> robot 127.0.0.1:9091   (rosbridge, bound to loopback on the Pi)
#   localhost:19099 -> robot 127.0.0.1:8099   (web_video_server snapshots)
# The link dropped once mid-measurement (2026-11-03 21:47) and every tool in
# tools/ hardcodes 19091, so this reconnects instead of dying with the shell.
set -u
cd "$(dirname "$0")/.." || exit 1
CREDS=tools/runtime/arm_creds.json
export ARMPI_SSH_PASSWORD="$(python -c "import json;print(json.load(open('$CREDS'))['password'])")"
HOST="${1:-100.102.57.19}"

start() {
  local_port="$1"; remote_port="$2"; log="$3"
  python tools/ssh_tunnel.py --host "$HOST" --user pi \
    --local-port "$local_port" --remote-host 127.0.0.1 --remote-port "$remote_port" \
    >> "$log" 2>&1 &
  echo $!
}

while true; do
  p91=$(start 19091 9091 tools/runtime/tunnel19091.log)
  p99=$(start 19099 8099 tools/runtime/tunnel19099.log)
  echo "$(date '+%H:%M:%S') tunnels up (19091 pid $p91, 19099 pid $p99)"
  # Block until either one dies; the relay exits when the SSH channel breaks.
  while kill -0 "$p91" 2>/dev/null && kill -0 "$p99" 2>/dev/null; do
    sleep 5
  done
  kill "$p91" "$p99" 2>/dev/null
  wait "$p91" "$p99" 2>/dev/null
  echo "$(date '+%H:%M:%S') tunnel dropped, reconnecting in 3 s"
  sleep 3
done
