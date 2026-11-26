#!/bin/sh
set -eu

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
    printf 'Usage: sh scripts/probe_camera.sh [/dev/videoN]\n'
    exit 0
fi
if [ "$#" -gt 1 ]; then
    printf 'Usage: sh scripts/probe_camera.sh [/dev/videoN]\n' >&2
    exit 2
fi
command -v v4l2-ctl >/dev/null || {
    printf 'v4l2-ctl is required (v4l-utils).\n' >&2
    exit 1
}

printf '\nBoard\n'
if [ -r /proc/device-tree/model ]; then
    tr -d '\000' </proc/device-tree/model
    printf '\n'
fi
uname -rm

printf '\nV4L2 nodes\n'
v4l2-ctl --list-devices || true

printf '\nMedia graphs\n'
if command -v media-ctl >/dev/null; then
    for graph in /dev/media*; do
        [ -c "$graph" ] || continue
        media-ctl -d "$graph" -p || true
    done
else
    printf 'media-ctl is unavailable. Install it to trace the OV13850 entity.\n'
fi

device=${1:-}
if [ -z "$device" ]; then
    count=0
    for node in /sys/class/video4linux/video*; do
        [ -r "$node/name" ] || continue
        name=$(cat "$node/name")
        [ "$name" = rkisp_mainpath ] || continue
        device="/dev/${node##*/}"
        count=$((count + 1))
        printf 'Main path candidate: %s\n' "$device"
    done
    if [ "$count" -ne 1 ]; then
        printf 'Use the OV13850 media graph to choose a node and rerun with /dev/videoN.\n' >&2
        exit 1
    fi
fi

printf '\nCapture format and capabilities on %s\n' "$device"
v4l2-ctl -d "$device" --all
v4l2-ctl -d "$device" --list-formats-ext
