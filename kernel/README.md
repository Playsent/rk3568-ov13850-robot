# 在 ATK-DLRK3568 上练习 OV13850 驱动移植

先用现有镜像作为对照，再让同一颗 OV13850 通过新的设备树匹配串，绑定你编译的练习驱动。这样每次只切换一个变量，可以判断失败出在驱动编译、设备树匹配，还是 Sensor 到 ISP 的数据链路。

## 已在你的板上确认

| 项目 | 板上结果 |
| --- | --- |
| 板卡与系统 | ATK-DLRK3568，Ubuntu 24.04，AArch64，内核 6.1.141 |
| Sensor | I²C 设备 `4-0010`，驱动识别到 `0x00d850` 和 revision `0xb2` |
| 实际设备树 | `/i2c@fe5d0000/ov13850@10`，compatible 为 `ovti,ov13850` |
| 驱动编译方式 | `CONFIG_VIDEO_OV13850=y`，`CONFIG_MODULES=y` |
| ISP 输出 | `/dev/video0` 对应 `rkisp_mainpath`，MPLANE API，单内存平面，NV12 Full Range |
| 采集基线 | 1280×720，stride 1280，sizeimage 1,382,400 字节，60 帧测试显示 30.05 fps |

`y` 表示原驱动在内核 Image 中。练习使用一个带独立名字和匹配串的驱动，并让实验 DTB 选择它。两套驱动可以编入同一个实验内核，便于用原设备树和实验设备树做 A/B 验证。

## 1. 准备匹配的源码

在 Linux 编译机或 Ubuntu 虚拟机上准备与当前镜像对应的正点原子 Linux 6.1 SDK。核对 SDK 的发行版本和 ATK-DLRK3568 板级配置，`6.1.141` 只是内核版本号。以编译机现有的厂商交叉工具链为准。MobaXterm 用来读板端结果和传文件。
[正点原子 RK3568 SDK 下载入口](https://www.openedv.com/docs/boards/arm-linux/RK3568Linux.html)

在板上导出运行配置，再复制到编译机。

```sh
zcat /proc/config.gz > ~/rk3568-stock.config
media-ctl -p -d /dev/media0 > ~/rk3568-stock-media.txt
```

打开 SDK 中的 `drivers/media/i2c/ov13850.c`。沿 `of_match_table → ov13850_probe → __ov13850_power_on → ov13850_check_sensor_id → v4l2_async_register_subdev_sensor` 阅读。找到 `s_stream` 回调及其调用的模式表写入函数，核对 `probe` 成功和开始输出图像分别发生在哪里。
[瑞芯微 Linux 6.1 OV13850 驱动](https://github.com/rockchip-linux/kernel/blob/develop-6.1/drivers/media/i2c/ov13850.c)

## 2. 生成并编译练习驱动

在编译机操作 SDK 的实验副本。首先把下面第一个变量改成你自己的路径，并在本工程根目录运行。

```bash
lab_kernel=/path/to/alientek-sdk-lab/kernel
lab_patch="$PWD/out/ov13850-portlab.patch"
mkdir -p out
python3 kernel/prepare_ov13850_lab.py --kernel "$lab_kernel" --output "$lab_patch"
git -C "$lab_kernel" apply --check "$lab_patch"
git -C "$lab_kernel" apply "$lab_patch"
```

补丁增加 `drivers/media/i2c/ov13850_portlab.c` 和对应的 Kconfig、Makefile 条目。打开生成的 C 文件，找这三处改动。

1. OF 匹配串为 `ovti,ov13850-portlab`。它决定新驱动能绑定哪个设备树节点。
2. I²C 驱动名和 I²C ID 为 `ov13850-portlab`。启动后可通过 sysfs 检查实际绑定的驱动。
3. `probe` 完成供电、Chip ID 验证和 V4L2 subdev 注册后打印 `OV13850 portlab: sensor and V4L2 subdev registered`。Sensor 的寄存器模式表、供电顺序和 subdev 名仍取自这份 SDK。

可以先阅读 [瑞芯微 6.1 参考补丁](ov13850-portlab-rockchip-6.1-reference.patch)。实际编译使用上面针对正点原子 SDK 生成的补丁。

用厂商 SDK 的板级内核构建方式编译。下面是标准 Kbuild 的实验内核构建方式，工具链前缀、配置文件位置和额外构建选项需与该 SDK 对应。内核和模块使用同一份输出目录。

```bash
lab_build="$PWD/out/kernel-build"
export ARCH=arm64
export CROSS_COMPILE=/path/to/toolchain/bin/aarch64-linux-gnu-
mkdir -p "$lab_build"
cp /path/to/rk3568-stock.config "$lab_build/.config"
"$lab_kernel/scripts/config" --file "$lab_build/.config" \
  -e VIDEO_OV13850 -e VIDEO_OV13850_PORTLAB
make -C "$lab_kernel" O="$lab_build" olddefconfig
grep '^CONFIG_VIDEO_OV13850' "$lab_build/.config"
make -C "$lab_kernel" O="$lab_build" -j"$(nproc)" Image dtbs modules
```

应看到两个配置都为 `y`。这里把练习驱动也编入 Image，使启动阶段的匹配实验不依赖原镜像的模块目录或模块 ABI。SDK 构建脚本若重新加载 defconfig，要把这两个配置加入实验板级配置后再构建。

## 3. 用实测节点生成实验 DTB

随附的 [设备树 overlay](atk-dlrk3568-ov13850-portlab.dts) 使用板端已确认的绝对路径，只覆盖 sensor 的 `compatible` 属性。它通过离线合并生成实验 DTB，板上启动前就可以核对输出。

```bash
stock_dtb=/path/to/the-exact-stock-board.dtb
sensor=/i2c@fe5d0000/ov13850@10
fdtget -t s "$stock_dtb" "$sensor" compatible
# 上一条应输出 ovti,ov13850。若路径不匹配，先核对是否选错板级 DTB。
dtc -@ -I dts -O dtb -o out/ov13850-portlab.dtbo \
  kernel/atk-dlrk3568-ov13850-portlab.dts
fdtoverlay -i "$stock_dtb" -o out/atk-dlrk3568-portlab.dtb out/ov13850-portlab.dtbo
fdtget -t s out/atk-dlrk3568-portlab.dtb "$sensor" compatible
```

最后应输出 `ovti,ov13850-portlab`。留存能启动的原厂镜像和启动介质。按该版 SDK 的启动布局，把实验 Image、DTB 和对应模块装入单独的测试启动项或测试镜像。若厂商布局使用打包的 `boot.img`，在实验副本里用该 SDK 的打包入口组合 Image 和 DTB。

## 4. 两次启动，三项验收

**第一次用实验 Image + 原有 DTB。** 这次验证新增驱动编入内核后，原节点仍绑定 `ov13850` 并正常出帧。**第二次用同一个实验 Image + 实验 DTB。** 以下命令先确认新绑定，再验证媒体拓扑和出帧。

```sh
readlink -f /sys/bus/i2c/devices/4-0010/driver
tr '\000' '\n' </sys/bus/i2c/devices/4-0010/of_node/compatible
sudo dmesg | grep -E 'ov13850-portlab|OV13850 portlab|dphy0'
media-ctl -d /dev/media0 -p
timeout 12 v4l2-ctl -d /dev/video0 \
  --set-fmt-video=width=1280,height=720,pixelformat=NV12 \
  --get-fmt-video --stream-mmap=8 --stream-count=60 --stream-to=/dev/null
```

| 验收点 | 通过条件 | 若失败先看 |
| --- | --- | --- |
| 驱动匹配 | sysfs 指向 `ov13850-portlab`，运行设备树是新 compatible | 启动实际加载的 DTB、`CONFIG_VIDEO_OV13850_PORTLAB` |
| 驱动与媒体注册 | 新的 probe 完成日志出现，图中 OV13850 与 D-PHY、RKISP 链路可见 | 电源/时钟、Chip ID、endpoint 与 async 注册日志 |
| 同配置出帧 | 60 帧命令退出正常，1280×720 NV12 与原基线一致 | ISP 采集节点、相机程序占用、`s_stream`、CSI 错误日志 |

你在现有镜像看到的 dummy regulator 和 sleep pinstate 提示可以作为对照。当前出帧已经成功。检查这些属性时先查对应板级原理图和实际电源连接，再改设备树电源配置。

通过后，可以在练习驱动的 `s_stream` 启停成功分支加一条 `dev_dbg`，输出启停状态和当前模式，再重复一次 60 帧采集。这个实验能直观看到 V4L2 的 `STREAMON/STREAMOFF` 如何到达 Sensor 驱动。

下一轮可练习板级 DTSI 拆分。在 SDK 中定位这个 sensor 的实际节点，逐一对应驱动读取的 `xvclk`、GPIO、电源、模组信息与 endpoint 属性。把相机节点及相关 endpoint 放入单独的 DTSI，由实验板级 DTS 引用。对比拆分前后 DTB 的反编译结果和媒体拓扑，确认属性与链接一致之后再逐项调整。
