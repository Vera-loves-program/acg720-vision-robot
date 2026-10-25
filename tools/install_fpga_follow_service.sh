#!/bin/sh
# Install an explicit vision-only boot service. No target is selected, no
# motor/gimbal is authorized on boot. Network carrier may appear later.
set -eu
follow_root=/home/orangepi/acg720_vision
if [ "$(id -un)" != orangepi ]; then echo 'Run as orangepi, without sudo prefix.' >&2; exit 1; fi
test -x /home/orangepi/.venvs/acg720-detection/bin/python
test -f "$follow_root/pc/visual_task.py"
follow_unit=$(mktemp)
trap 'rm -f -- "$follow_unit"' EXIT HUP INT TERM
cat > "$follow_unit" <<'UNIT'
[Unit]
Description=ACG720 v22 dog/face vision and135deg pose for FPGA LCD
After=network.target
StartLimitIntervalSec=30
StartLimitBurst=5

[Service]
Type=simple
User=orangepi
WorkingDirectory=/home/orangepi/acg720_vision
Environment=PYTHONUNBUFFERED=1
ExecStart=/home/orangepi/.venvs/acg720-detection/bin/python /home/orangepi/acg720_vision/client.py gimbal --camera /dev/video0 --width 640 --height 480 --fps 15 --fpga --standalone --initial-pan 1200 --initial-tilt-angle 135
Restart=on-failure
RestartSec=2
KillMode=control-group
TimeoutStopSec=15
StandardOutput=append:/home/orangepi/acg720_vision/follow-console.log
StandardError=inherit

[Install]
WantedBy=multi-user.target
UNIT
sudo systemctl stop acg720-follow.service 2>/dev/null || true
follow_pids=$(pgrep -f '[/]home/orangepi/acg720_vision/pc/usb_gimbal.py( |$)' || true)
for follow_pid in $follow_pids; do kill -TERM "$follow_pid"; done
follow_wait=0
while pgrep -f '[/]home/orangepi/acg720_vision/pc/usb_gimbal.py( |$)' >/dev/null; do
    follow_wait=$((follow_wait+1))
    if [ "$follow_wait" -ge 150 ]; then echo 'Old vision process did not exit; install stopped.' >&2; exit 1; fi
    sleep .1
done
if ss -lntu | grep -Eq ':8080\b|:6102\b'; then
    echo '8080/6102 occupied by another process; install stopped.' >&2; ss -lntup; exit 1
fi
sudo install -o root -g root -m 0644 "$follow_unit" /etc/systemd/system/acg720-follow.service
sudo systemctl daemon-reload
sudo systemctl enable --now acg720-follow.service
sleep 2
sudo systemctl --no-pager --full status acg720-follow.service
grep -E 'FOLLOW runtime:|FOLLOW prepared:|VISION engines:' "$follow_root/follow-console.log" | tail -n 3 || true
echo 'Boot service installed. Use start_fpga_follow.sh --restart for later updates.'
