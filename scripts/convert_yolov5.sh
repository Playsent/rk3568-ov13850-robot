#!/bin/sh
set -eu

if [ "$#" -ne 2 ]; then
    printf 'Usage: sh scripts/convert_yolov5.sh /path/to/rknn_model_zoo /path/to/yolov5s_relu.onnx\n' >&2
    exit 2
fi
project=$(CDPATH='' cd -- "$(dirname -- "$0")/.." && pwd)
zoo=$(CDPATH='' cd -- "$1" && pwd)
onnx_dir=$(CDPATH='' cd -- "$(dirname -- "$2")" && pwd)
onnx="$onnx_dir/$(basename -- "$2")"
output="$project/model/yolov5s_rk3568.rknn"
[ -r "$onnx" ] || { printf 'Missing ONNX: %s\n' "$onnx" >&2; exit 1; }
cd "$zoo/examples/yolov5/python"
test -f convert.py
test -f ../../../datasets/COCO/coco_subset_20.txt
temp=$(mktemp -d "$project/model/.export.XXXXXX")
trap 'rm -f -- "$temp/model.rknn"; rmdir -- "$temp"' EXIT
python3 convert.py "$onnx" rk3568 i8 "$temp/model.rknn"
test -s "$temp/model.rknn"
mv -- "$temp/model.rknn" "$output"
printf 'Exported %s\n' "$output"
