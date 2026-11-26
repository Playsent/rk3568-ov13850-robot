# ATK-DLRK3568 OV13850 驱动移植

目标板为正点原子 ATK-DLRK3568，运行 AArch64 Linux 6.1.141。移植依据为本机的 `hdmi_ubuntu24.04.4_update.img` V1.1，发布日期 2026-11-05，与板上记录对应。

| 文件 | 内容 |
| --- | --- |
| [ov13850.c](ov13850.c) | 完整 sensor 驱动。支持 6.1 的 V4L2 subdev、I²C、runtime PM 和模式配置。 |
| [ov13850-atk-rk3568.patch](ov13850-atk-rk3568.patch) | 在 SDK 的 `drivers/media/i2c/ov13850.c` 上集成该驱动。沿用 `CONFIG_VIDEO_OV13850=y` 和 `ovti,ov13850`。 |
| [rtl8852bs-sdk-out-of-tree.patch](rtl8852bs-sdk-out-of-tree.patch) | 修复 SDK 在独立输出目录构建时 RTL8852BS 找不到 Rockchip 平台参数的问题，保留原厂配置中的 Wi-Fi 模块。 |
| [atk-dlrk3568-ov13850-camera.dts](atk-dlrk3568-ov13850-camera.dts) | 从原厂 DTB 核对的相机板级配置，包含 OV13850、VM149C、XCLK、RESET/PWDN 和 D-PHY endpoint。 |
| `atk-dlrk3568-ov13850-camera.dtbo`、`atk-dlrk3568-ov13850.dtb` | 已编译的 overlay 和基于原厂 DTB 合并的板级 DTB。 |
| [rk3568-stock.config](rk3568-stock.config) | 从原厂内核 Image 提取的完整构建配置。 |
| [build-for-sdk.sh](build-for-sdk.sh) | 在 Linux SDK 实验副本中应用补丁，并构建 Image 和模块。 |
| [Kbuild](Kbuild) | 用准备好的同款内核构建目录进行外部模块编译。此方式用于验证编译与链接，板上仍以内建驱动集成为部署方式。 |

## 核对结果

- I²C 控制器为 `/i2c@fe5d0000`，Sensor 地址为 `0x10`，实测 Chip ID `0x00d850`、revision `0xb2`。
- 模组为 `ZC-OV13850R2A-V1`。设备树保留四条物理走线，Sensor 寄存器配置仍启用两条，链路频率为 300 MHz。两个初始化表和两个模式表的全部 467 个寄存器条目，已和原厂内核二进制逐项匹配。
- 驱动读取 CSI-2 endpoint，保留原厂寄存器时序。修正 revision 表在多实例间共享的问题，GPIO 或时钟提供方尚未就绪时保留 probe defer，供电失败时回收已打开的电源 GPIO。
- overlay 已用设备树编译器编译，并与原厂 DTB 合并。合并后的 1045 个设备树节点及属性逐项一致。未给固定供电的板卡添加推测的电源控制项。
- OV13850 驱动使用固定版内核头文件和固件中提取的配置成功交叉编译为 ARM64 目标文件。此项验证了驱动源码与 6.1.141 内核 API 的编译兼容性。
- 原厂系统已在 `/dev/video0` 完成 1280×720 NV12 的 60 帧测试，显示 30.05 fps。新内核的测试以同一配置为验收基线。

驱动基线固定为 [Rockchip Linux 6.1.141 的 b4ef083dc0c3 提交](https://github.com/rockchip-linux/kernel/tree/b4ef083dc0c3608e744deabb43dc6b781aadbe6e)。其中 `ov13850.c` 的 SHA-256 为 `0df2a5436e8a68865c0318d9e8c782eafbf41744ed4e2023ebf9bff38784cf8c`。

## 编译和上板验收

已核对正点原子 SDK V1.2.0（20261105），其内核目录为 `kernel-6.1`，归档中的内核提交为 `ad4a8d6011db82a4706e82c15f83427575a450df`。该版本的 OV13850 原始源码通过补丁应用检查。

编译脚本接收正点原子 6.1.141 内核源码目录和一个构建目录。使用与当前固件配套的 SDK 工具链。厂商 B 盘是对应 SDK 的分发入口。
[正点原子 RK3568 资料下载](https://www.openedv.com/docs/boards/arm-linux/RK3568Linux.html)

```sh
CROSS_COMPILE=/path/to/aarch64-none-linux-gnu- \
  sh kernel/port/build-for-sdk.sh /path/to/sdk/kernel-6.1 /path/to/kernel-build
```

构建输出的 Image 位于 `arch/arm64/boot/Image`，板级 DTB 位于构建目录根目录。模块与同一份 Image 一起构建。按该版 SDK 的启动镜像打包流程使用这些产物。带新版驱动的内核启动时会出现 `ATK OV13850: 2 active CSI-2 lanes, V4L2 subdev ready`，再用原采集命令复测。

2026-11-23，测试镜像在 ATK-DLRK3568 上启动并通过 OV13850 采集验收。实测内核为 `6.1.141`，I²C `4-0010` 的 OV13850 报告两条活动 CSI-2 通道，V4L2 子设备已就绪。`/dev/video0` 以 `1280×720 NV12` 连续采集 60 帧，显示 `30.05 fps`，`capture_status=0`，与原厂基线一致。 [烧录和回退说明](../../out/atk-dlrk3568-ov13850-6.1.141/README.md)
