#!/usr/bin/env python3
"""Create an OV13850 porting patch from the supplied Rockchip BSP tree."""

import argparse
import difflib
from pathlib import Path
import re


I2C = Path("drivers/media/i2c")
STOCK = "ov13850"
LAB = "ov13850-portlab"
LAB_OBJECT = "ov13850_portlab"
CONFIG = "VIDEO_OV13850"
LAB_CONFIG = "VIDEO_OV13850_PORTLAB"


def replace_once(text, pattern, replacement, label):
    changed, count = re.subn(pattern, lambda _: replacement, text, flags=re.MULTILINE)
    if count != 1:
        raise ValueError(f"{label}: expected one matching location, found {count}")
    return changed


def make_driver(source):
    driver = replace_once(
        source, r'^(\s*\{\s*\.compatible\s*=\s*)"ovti,ov13850"(\s*\},\s*)$',
        f'\t{{ .compatible = "ovti,{LAB}" }},', "OV13850 OF match table",
    )
    driver = replace_once(
        driver, r'^\s*\{\s*"(?:ovti,)?ov13850"\s*,\s*0\s*\},[^\S\n]*$',
        f'\t{{ "{LAB}", 0 }},', "OV13850 I2C ID table",
    )
    driver = replace_once(
        driver, r'^[ \t]*\.name\s*=\s*(?:OV13850_NAME|"ov13850"),[ \t]*$',
        f'\t\t.name = "{LAB}",', "OV13850 I2C driver name",
    )
    probe = re.search(
        r'^static int ov13850_probe\b.*?(?=^static (?:int|void) ov13850_remove\b)',
        driver, flags=re.MULTILINE | re.DOTALL,
    )
    if probe is None:
        raise ValueError("could not locate the OV13850 probe and remove functions")
    traced = replace_once(
        probe.group(), r'^\tpm_runtime_idle\(dev\);[ \t]*$',
        '\tpm_runtime_idle(dev);\n'
        '\tdev_info(dev, "OV13850 portlab: sensor and V4L2 subdev registered\\n");',
        "OV13850 probe success path",
    )
    return driver[:probe.start()] + traced + driver[probe.end():]


def make_kconfig(source):
    match = re.search(
        rf'^config {CONFIG}\n(?:(?!^(?:menuconfig|config|endmenu|endif|if)\b).)*',
        source, flags=re.MULTILINE | re.DOTALL,
    )
    if match is None or LAB_CONFIG in source:
        raise ValueError("expected a stock VIDEO_OV13850 Kconfig stanza")
    stanza = match.group()
    settings, separator, _ = stanza.partition("\n\thelp\n")
    if not separator:
        raise ValueError("cannot locate help in the stock OV13850 Kconfig stanza")
    settings = settings.replace(f"config {CONFIG}", f"config {LAB_CONFIG}", 1)
    settings = replace_once(
        settings, r'^[ \t]*tristate\s+"[^"]+"[ \t]*$',
        '\ttristate "OmniVision OV13850 porting exercise"', "OV13850 Kconfig prompt",
    )
    lab = (settings + "\n\thelp\n"
           "\t  OV13850 sensor driver used by the RK3568 porting exercise.\n"
           "\t  This driver matches the ovti,ov13850-portlab device-tree node.\n\n")
    return source[:match.end()] + lab + source[match.end():]


def make_patch(path, before, after, *, new=False):
    name = path.as_posix()
    intro = f"diff --git a/{name} b/{name}\n"
    if new:
        intro += "new file mode 100644\n"
    return intro + "".join(difflib.unified_diff(
        before.splitlines(keepends=True), after.splitlines(keepends=True),
        fromfile="/dev/null" if new else f"a/{name}", tofile=f"b/{name}",
    ))


def generate(kernel):
    driver = kernel / I2C / f"{STOCK}.c"
    lab_driver = I2C / f"{LAB_OBJECT}.c"
    if (kernel / lab_driver).exists():
        raise ValueError(f"the SDK already contains {lab_driver}")
    config = (kernel / I2C / "Kconfig").read_text(encoding="utf-8")
    makefile = (kernel / I2C / "Makefile").read_text(encoding="utf-8")
    if LAB_CONFIG in makefile:
        raise ValueError("the SDK Makefile already contains the practice driver")
    updated_makefile = replace_once(
        makefile, rf'^obj-\$\(CONFIG_{CONFIG}\)[ \t]*\+=[ \t]*{STOCK}\.o[ \t]*$',
        f"obj-$(CONFIG_{CONFIG}) += {STOCK}.o\nobj-$(CONFIG_{LAB_CONFIG}) += {LAB_OBJECT}.o",
        "OV13850 Makefile entry",
    )
    lab = make_driver(driver.read_text(encoding="utf-8"))
    return "".join((
        make_patch(I2C / "Kconfig", config, make_kconfig(config)),
        make_patch(I2C / "Makefile", makefile, updated_makefile),
        make_patch(lab_driver, "", lab, new=True),
    ))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kernel", required=True, type=Path, help="Rockchip Linux 6.1 kernel directory")
    parser.add_argument("--output", required=True, type=Path, help="new .patch file to create")
    args = parser.parse_args()
    try:
        patch = generate(args.kernel)
        with args.output.open("x", encoding="utf-8", newline="\n") as output:
            output.write(patch)
    except (OSError, ValueError) as error:
        parser.exit(1, f"Cannot prepare OV13850 porting patch: {error}\n")
    print(f"Created {args.output}. Use git apply --check in the target SDK first.")


if __name__ == "__main__":
    main()
