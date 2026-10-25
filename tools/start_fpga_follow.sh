#!/bin/sh
# One launcher owns restart, the actual camera process and its two ports.
# It never selects a target or authorizes a wheel movement.
set -eu
follow_root=/home/orangepi/acg720_vision
follow_python=/home/orangepi/.venvs/acg720-detection/bin/python
follow_mode=${1:-start}
follow_log_summary() {
    # Show the actually running source version and calibrated PWM mapping,
    # not just the launch argument from an older service installation.
    grep -E 'FOLLOW runtime:|FOLLOW prepared:' "$follow_root/follow-console.log" 2>/dev/null | tail -n 2 || true
    tail -n 10 "$follow_root/follow-console.log" 2>/dev/null || true
}
case "$follow_mode" in start|--restart|--status) ;; *) echo 'Use start, --restart or --status'; exit 2;; esac
if systemctl is-enabled --quiet acg720-follow.service 2>/dev/null; then
    if [ "$follow_mode" = --restart ] && ! systemctl show -p ExecStart --value acg720-follow.service | grep -q -- '--initial-tilt-angle 135'; then
        # Upgrade only our existing service's launch pose. No calibration or
        # models are rewritten by the installer.
        exec sh "$follow_root/tools/install_fpga_follow_service.sh"
    fi
    case "$follow_mode" in
        --status) systemctl --no-pager --full status acg720-follow.service; follow_log_summary; exit 0;;
        --restart) sudo systemctl restart acg720-follow.service;;
        start) sudo systemctl start acg720-follow.service;;
    esac
    sleep 2
    systemctl --no-pager --full status acg720-follow.service
    follow_log_summary
    exit 0
fi
follow_pids=$(pgrep -f '[/]home/orangepi/acg720_vision/pc/usb_gimbal.py( |$)' || true)
if [ "$follow_mode" = --status ]; then
    pgrep -af '[/]home/orangepi/acg720_vision/pc/usb_gimbal.py( |$)' || true
    ss -lntup | grep -E ':8080\b|:6102\b' || true
    follow_log_summary
    exit 0
fi
if [ -n "$follow_pids" ]; then
    if [ "$follow_mode" != --restart ]; then
        echo "Already running: $follow_pids (use --restart to load an update)"
        exit 0
    fi
    for follow_pid in $follow_pids; do kill -TERM "$follow_pid" 2>/dev/null || true; done
    follow_wait=0
    while pgrep -f '[/]home/orangepi/acg720_vision/pc/usb_gimbal.py( |$)' >/dev/null; do
        follow_wait=$((follow_wait+1))
        if [ "$follow_wait" -ge 100 ]; then echo 'Old camera process has not exited; no duplicate started.'; exit 1; fi
        sleep .1
    done
fi
if ss -lntu | grep -Eq ':8080\b|:6102\b'; then
    echo '8080 or 6102 is occupied by another process. No duplicate started.'
    ss -lntup | grep -E ':8080\b|:6102\b' || true
    exit 1
fi
test -x "$follow_python" || { echo "Missing runtime: $follow_python"; exit 1; }
cd "$follow_root"
if [ -f follow-console.log ]; then cp follow-console.log follow-console.previous.log; fi
nohup "$follow_python" client.py gimbal --camera /dev/video0 --width 640 --height 480 --fps 15 \
    --fpga --standalone --initial-pan 1200 --initial-tilt-angle 135 > follow-console.log 2>&1 < /dev/null &
follow_pid=$!
sleep 2
if ! kill -0 "$follow_pid" 2>/dev/null; then cat follow-console.log; exit 1; fi
echo "Vision process running: $follow_pid; targets and wheels are not started by this script."
follow_log_summary
