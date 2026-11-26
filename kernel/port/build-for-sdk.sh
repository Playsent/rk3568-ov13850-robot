#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-only
set -eu

if [ "$#" -ne 2 ]; then
    printf 'Usage: sh build-for-sdk.sh /path/to/linux-6.1.141 /path/to/build-dir\n' >&2
    exit 2
fi

port=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)
kernel=$(CDPATH='' cd -- "$1" && pwd)
mkdir -p -- "$2"
build=$(CDPATH='' cd -- "$2" && pwd)
compiler=${CROSS_COMPILE:-}
if [ -z "$compiler" ] && [ "$(uname -m)" != aarch64 ]; then
    compiler=aarch64-linux-gnu-
fi

if [ "$(make -s --no-print-directory -C "$kernel" kernelversion)" != 6.1.141 ]; then
    printf 'Expected the ATK/Rockchip Linux 6.1.141 source tree.\n' >&2
    exit 1
fi
command -v "${compiler}gcc" >/dev/null || {
    printf 'Cannot find %sgcc. Set CROSS_COMPILE to the SDK toolchain prefix.\n' "$compiler" >&2
    exit 1
}

for patch in "$port/ov13850-atk-rk3568.patch" "$port/rtl8852bs-sdk-out-of-tree.patch"; do
    if git -C "$kernel" apply --reverse --check "$patch" 2>/dev/null; then
        printf 'Already applied: %s\n' "${patch##*/}"
    else
        git -C "$kernel" apply --check "$patch"
        git -C "$kernel" apply "$patch"
    fi
done

if [ ! -f "$build/.config" ]; then
    cp -- "$port/rk3568-stock.config" "$build/.config"
fi
make -C "$kernel" O="$build" ARCH=arm64 CROSS_COMPILE="$compiler" olddefconfig
grep -qx 'CONFIG_VIDEO_OV13850=y' "$build/.config" || {
    printf 'The selected configuration needs CONFIG_VIDEO_OV13850=y.\n' >&2
    exit 1
}
make -C "$kernel" O="$build" ARCH=arm64 CROSS_COMPILE="$compiler" \
    -j"${JOBS:-$(nproc)}" Image modules
cp -- "$port/atk-dlrk3568-ov13850.dtb" "$build/atk-dlrk3568-ov13850.dtb"
printf 'Image: %s/arch/arm64/boot/Image\nDTB: %s/atk-dlrk3568-ov13850.dtb\n' "$build" "$build"
