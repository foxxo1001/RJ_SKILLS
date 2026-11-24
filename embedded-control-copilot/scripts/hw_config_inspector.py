#!/usr/bin/env python3
"""
hw_config_inspector.py
Zero-dependency Python 3 hardware/IDE configuration inspector for:
  - STM32CubeMX (.ioc) & Keil MDK (.uvprojx)
  - TI CCS / SysConfig (.syscfg + ti_msp_dl_config.h)
  - ESP-IDF (sdkconfig + CMakeLists.txt)
  - Renesas RASC / FSP (configuration.xml + bsp_cfg.h)
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

SKIP_WALK_DIRS = {".git", "build", "objects", "listings", "managed_components", "__pycache__"}


def read_text_safely(path: Path) -> str:
    raw = path.read_bytes()
    for enc in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return raw.decode(enc)
        except UnicodeDecodeError:
            continue
    return raw.decode("utf-8", errors="replace")


def parse_stm32_ioc(path: Path) -> Dict[str, Any]:
    kv: Dict[str, str] = {}
    for line in read_text_safely(path).splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        kv[k.strip()] = v.strip()

    timers: Dict[str, Dict[str, str]] = {}
    uarts: Dict[str, Dict[str, str]] = {}
    dmas: List[str] = []
    gpio_labels: Dict[str, str] = {}
    nvic_irqs: List[str] = []
    warnings: List[str] = []

    for k, v in kv.items():
        mt = re.match(r"^(TIM\d+)\.(Prescaler|Period|Channel-[^=]+|Mode|EncoderMode)$", k)
        if mt:
            timers.setdefault(mt.group(1), {})[mt.group(2)] = v
        mu = re.match(r"^((?:U?S?ART|LPUART)\d+)\.(BaudRate|VirtualMode)$", k)
        if mu:
            uarts.setdefault(mu.group(1), {})[mu.group(2)] = v
        if k.startswith("Dma.") and ".Mode" in k:
            dmas.append(f"{k}={v}")
        if k.endswith(".GPIO_Label"):
            gpio_labels[k.replace(".GPIO_Label", "")] = v
        if k.startswith("NVIC.") and k != "NVIC.PriorityGroup" and ":true:" in v:
            nvic_irqs.append(f"{k.replace('NVIC.', '')} -> {v}")

    prio_grp = kv.get("NVIC.PriorityGroup", "unspecified")
    has_freertos = any("FREERTOS" in v for k, v in kv.items() if k.startswith("Mcu.IP"))
    if has_freertos and "4" not in prio_grp and prio_grp != "unspecified":
        warnings.append(f"启用了 FreeRTOS 但 NVIC 优先级分组为 {prio_grp}，强烈建议改为 NVIC_PRIORITYGROUP_4。")

    return {
        "file": str(path),
        "mcu": kv.get("Mcu.UserName") or kv.get("Mcu.Name", "unknown"),
        "clocks_hz": {
            "SYSCLK": kv.get("RCC.SYSCLKFreq_Value", "?"),
            "HCLK": kv.get("RCC.HCLKFreq_Value", "?"),
            "APB1_TIM": kv.get("RCC.APB1TimFreq_Value", "?"),
            "APB2_TIM": kv.get("RCC.APB2TimFreq_Value", "?"),
        },
        "freertos": has_freertos,
        "nvic_priority_group": prio_grp,
        "timers": timers,
        "uarts": uarts,
        "dma_modes": dmas[:20],
        "gpio_labels": gpio_labels,
        "enabled_irqs_count": len(nvic_irqs),
        "warnings": warnings,
    }


def parse_keil_uvprojx(path: Path) -> Dict[str, Any]:
    warnings: List[str] = []
    try:
        root = ET.fromstring(read_text_safely(path))
    except ET.ParseError as exc:
        return {"file": str(path), "error": f"XML parse error: {exc}"}

    def find_text(tag: str) -> str:
        el = root.find(f".//{tag}")
        return el.text.strip() if el is not None and el.text else ""

    device = find_text("Device")
    uac6 = find_text("uAC6")
    use_ulib = find_text("useUlib")
    optim = find_text("Optim")
    fpu = find_text("TargetCommonOption/Cpu") or find_text("Cpu")
    compiler = "ARMClang (AC6)" if uac6 == "1" else "ARMCC (AC5)"

    if use_ulib != "1":
        warnings.append("Keil 未勾选 Use MicroLIB (useUlib=0)：若使用 printf() 请确保已禁用半主机并实现 _sys_exit()，否则脱机运行必卡死。")
    if uac6 == "1":
        warnings.append("使用 ARMClang (AC6) 编译器：优化较激进，跨中断变量必须严格添加 volatile。")

    return {
        "file": str(path),
        "device": device,
        "compiler": compiler,
        "use_microlib": use_ulib == "1",
        "optimization_code": optim,
        "cpu_spec": fpu[:120],
        "warnings": warnings,
    }


def parse_ti_syscfg(path: Path, header_path: Path | None) -> Dict[str, Any]:
    text = read_text_safely(path)
    device_m = re.search(r"--device\s+\"([^\"]+)\"", text)
    product_m = re.search(r"--product\s+\"([^\"]+)\"", text)

    modules: Dict[str, List[str]] = {}
    for m in re.finditer(r'scripting\.addModule\("([^"]+)"\)', text):
        mod = m.group(1).split("/")[-1]
        modules.setdefault(mod, [])

    instances: List[Dict[str, str]] = []
    inst_vars: Dict[str, str] = {}
    for m in re.finditer(r'const\s+([A-Za-z0-9_]+)\s*=\s*([A-Za-z0-9_]+)\.addInstance\(', text):
        inst_vars[m.group(1)] = m.group(2)

    inst_props: Dict[str, Dict[str, str]] = {}
    for m in re.finditer(r'^([A-Za-z0-9_]+)\.([A-Za-z0-9_.$]+)\s*=\s*([^;]+);', text, re.MULTILINE):
        var, prop, val = m.group(1), m.group(2), m.group(3).strip().strip('"')
        inst_props.setdefault(var, {})[prop] = val

    warnings: List[str] = []
    for var, props in inst_props.items():
        name = props.get("$name", var)
        entry = {"var": var, "module": inst_vars.get(var, ""), "name": name}
        for key in ("timerCount", "timerPeriod", "pwmMode", "clockDivider", "targetBaudRate", "interrupts"):
            if key in props:
                entry[key] = props[key]
        instances.append(entry)
        if "PWM" in inst_vars.get(var, "").upper() or "pwm" in var.lower():
            pwm_mode = props.get("pwmMode", "EDGE_ALIGN")
            if "UP" not in pwm_mode.upper():
                warnings.append(
                    f"TI MSPM0 PWM 实例 '{name}' 默认为向下计数模式 ({pwm_mode})，设置占空比时注意 CC = Period - DutyCounts 反向陷阱！"
                )

    macros: List[str] = []
    if header_path and header_path.exists():
        htext = read_text_safely(header_path)
        macros = re.findall(r"#define\s+([A-Z0-9_]+(?:_INST|_INST_INT_IRQN|_PORT|_PIN))\b", htext)[:40]

    return {
        "file": str(path),
        "device": device_m.group(1) if device_m else "unknown",
        "product": product_m.group(1) if product_m else "unknown",
        "modules": sorted(modules.keys()),
        "instances": instances[:30],
        "config_header": str(header_path) if header_path else None,
        "extracted_macros": macros,
        "warnings": warnings,
    }


def parse_esp_idf(sdkconfig_path: Path) -> Dict[str, Any]:
    kv: Dict[str, str] = {}
    for line in read_text_safely(sdkconfig_path).splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        kv[k.strip()] = v.strip().strip('"')

    warnings: List[str] = []
    hz_str = kv.get("CONFIG_FREERTOS_HZ", "100")
    try:
        hz = int(hz_str)
        if hz <= 100:
            warnings.append(
                f"CONFIG_FREERTOS_HZ={hz} (Tick=10ms) 过粗！控制类任务请提高到 1000Hz 或改用 esp_timer / GPTimer 驱动控制环。"
            )
    except ValueError:
        hz = -1

    opt = "unknown"
    for k in (
        "CONFIG_COMPILER_OPTIMIZATION_DEBUG",
        "CONFIG_COMPILER_OPTIMIZATION_SIZE",
        "CONFIG_COMPILER_OPTIMIZATION_PERF",
    ):
        if kv.get(k) == "y":
            opt = k.replace("CONFIG_COMPILER_OPTIMIZATION_", "")

    return {
        "file": str(sdkconfig_path),
        "target": kv.get("CONFIG_IDF_TARGET", "unknown"),
        "cpu_freq_mhz": kv.get("CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ", "?"),
        "freertos_hz": hz_str,
        "unicore": kv.get("CONFIG_FREERTOS_UNICORE", "n") == "y",
        "task_wdt_timeout_s": kv.get("CONFIG_ESP_TASK_WDT_TIMEOUT_S", "?"),
        "optimization": opt,
        "warnings": warnings,
    }


def parse_renesas_fsp(xml_path: Path) -> Dict[str, Any]:
    warnings: List[str] = []
    try:
        root = ET.fromstring(read_text_safely(xml_path))
    except ET.ParseError as exc:
        return {"file": str(xml_path), "error": f"XML parse error: {exc}"}

    device = "unknown"
    for el in root.iter():
        if el.tag.endswith("device") and el.text and el.text.strip():
            device = el.text.strip()
            break
        if el.tag.endswith("raBspConfiguration"):
            pass

    props: Dict[str, str] = {}
    for prop in root.iter():
        if prop.tag.endswith("property"):
            pid = prop.attrib.get("id", "")
            pval = prop.attrib.get("value", "")
            if pid:
                props[pid] = pval

    main_stack_str = props.get("config.bsp.common.main", "")
    heap_str = props.get("config.bsp.common.heap", "")
    if main_stack_str:
        try:
            stack_bytes = int(main_stack_str, 0)
            if stack_bytes < 0x1000:
                warnings.append(
                    f"瑞萨 FSP Main Stack Size 仅为 {main_stack_str} ({stack_bytes} bytes < 4KB)！跑运动学/浮点/snprintf 极易触发栈监控 NMI/HardFault，请在 RASC 中调大至 >= 0x1000。"
                )
        except ValueError:
            pass

    stacks: List[Dict[str, str]] = []
    for mod in root.iter():
        if mod.tag.endswith("module"):
            mid = mod.attrib.get("id", "")
            if any(k in mid for k in ("r_gpt", "r_sci_uart", "r_can", "r_adc", "r_icu", "r_iic", "r_spi")):
                m_props = {
                    p.attrib.get("id", ""): p.attrib.get("value", "")
                    for p in mod
                    if p.tag.endswith("property")
                }
                stacks.append({
                    "driver": mid.split(".")[-1],
                    "name": m_props.get("module.driver.timer.name")
                    or m_props.get("module.driver.uart.name")
                    or m_props.get("module.driver.adc.name")
                    or m_props.get("module.driver.external_irq.name")
                    or "unnamed",
                    "callback": m_props.get("module.driver.timer.p_callback")
                    or m_props.get("module.driver.uart.p_callback")
                    or "",
                })

    return {
        "file": str(xml_path),
        "device": device,
        "main_stack_size": main_stack_str or "unknown",
        "heap_size": heap_str or "unknown",
        "fsp_stacks": stacks[:25],
        "warnings": warnings,
    }


def inspect_workspace(workspace: Path) -> Dict[str, Any]:
    report: Dict[str, Any] = {
        "workspace": str(workspace),
        "stm32_ioc": [],
        "keil_uvprojx": [],
        "ti_syscfg": [],
        "esp_idf": [],
        "renesas_fsp": [],
    }

    ti_header: Path | None = None
    syscfg_paths: List[Path] = []

    for root, dirs, filenames in os.walk(workspace):
        dirs[:] = [d for d in dirs if d.lower() not in SKIP_WALK_DIRS]
        for fn in filenames:
            p = Path(root) / fn
            low = fn.lower()
            if low.endswith(".ioc"):
                report["stm32_ioc"].append(parse_stm32_ioc(p))
            elif low.endswith(".uvprojx"):
                report["keil_uvprojx"].append(parse_keil_uvprojx(p))
            elif low.endswith(".syscfg"):
                syscfg_paths.append(p)
            elif low == "ti_msp_dl_config.h":
                ti_header = p
            elif low == "sdkconfig":
                report["esp_idf"].append(parse_esp_idf(p))
            elif low == "configuration.xml":
                report["renesas_fsp"].append(parse_renesas_fsp(p))

    for sp in syscfg_paths:
        report["ti_syscfg"].append(parse_ti_syscfg(sp, ti_header))

    return report


def format_markdown(report: Dict[str, Any]) -> str:
    lines = [f"# Hardware & IDE Configuration Inspection", f"- **Workspace**: `{report['workspace']}`", ""]
    found_any = False

    for item in report["stm32_ioc"]:
        found_any = True
        lines.append(f"## STM32CubeMX (`{item['file']}`)")
        lines.append(f"- **MCU**: `{item['mcu']}` | **FreeRTOS**: `{item['freertos']}` | **NVIC Group**: `{item['nvic_priority_group']}`")
        c = item["clocks_hz"]
        lines.append(f"- **Clocks (Hz)**: SYSCLK=`{c['SYSCLK']}`, HCLK=`{c['HCLK']}`, APB1_TIM=`{c['APB1_TIM']}`, APB2_TIM=`{c['APB2_TIM']}`")
        if item["timers"]:
            lines.append(f"- **Timers**: `{json.dumps(item['timers'], ensure_ascii=False)}`")
        if item["uarts"]:
            lines.append(f"- **UARTs**: `{json.dumps(item['uarts'], ensure_ascii=False)}`")
        if item["dma_modes"]:
            lines.append(f"- **DMA Modes**: `{', '.join(item['dma_modes'][:10])}`")
        for w in item["warnings"]:
            lines.append(f"- ⚠️ **Warning**: {w}")
        lines.append("")

    for item in report["keil_uvprojx"]:
        found_any = True
        lines.append(f"## Keil MDK (`{item['file']}`)")
        if "error" in item:
            lines.append(f"- Error: {item['error']}")
        else:
            lines.append(f"- **Device**: `{item['device']}` | **Compiler**: `{item['compiler']}` | **MicroLIB**: `{item['use_microlib']}`")
            for w in item["warnings"]:
                lines.append(f"- ⚠️ **Warning**: {w}")
        lines.append("")

    for item in report["ti_syscfg"]:
        found_any = True
        lines.append(f"## TI MSPM0 SysConfig (`{item['file']}`)")
        lines.append(f"- **Device**: `{item['device']}` | **Modules**: `{', '.join(item['modules'])}`")
        if item["instances"]:
            inst_names = [f"{x['name']}({x['module']})" for x in item["instances"]]
            lines.append(f"- **Instances**: `{', '.join(inst_names)}`")
        if item["extracted_macros"]:
            lines.append(f"- **Header Macros (`ti_msp_dl_config.h`)**: `{', '.join(item['extracted_macros'][:15])}`")
        for w in item["warnings"]:
            lines.append(f"- ⚠️ **Warning**: {w}")
        lines.append("")

    for item in report["esp_idf"]:
        found_any = True
        lines.append(f"## ESP-IDF (`{item['file']}`)")
        lines.append(
            f"- **Target**: `{item['target']}` | **CPU**: `{item['cpu_freq_mhz']} MHz` | "
            f"**FreeRTOS HZ**: `{item['freertos_hz']}` | **Opt**: `{item['optimization']}`"
        )
        for w in item["warnings"]:
            lines.append(f"- ⚠️ **Warning**: {w}")
        lines.append("")

    for item in report["renesas_fsp"]:
        found_any = True
        lines.append(f"## Renesas RASC / FSP (`{item['file']}`)")
        if "error" in item:
            lines.append(f"- Error: {item['error']}")
        else:
            lines.append(
                f"- **Device**: `{item['device']}` | **Main Stack**: `{item['main_stack_size']}` | "
                f"**Heap**: `{item['heap_size']}`"
            )
            if item["fsp_stacks"]:
                s_list = [f"{s['name']}({s['driver']})" for s in item["fsp_stacks"]]
                lines.append(f"- **Configured FSP Drivers**: `{', '.join(s_list)}`")
            for w in item["warnings"]:
                lines.append(f"- ⚠️ **Warning**: {w}")
        lines.append("")

    if not found_any:
        lines.append("ℹ️ 未在当前工作区发现 `.ioc` / `.uvprojx` / `.syscfg` / `sdkconfig` / `configuration.xml` 工程配置文件。")

    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description="Embedded Hardware & IDE Config Inspector")
    parser.add_argument("workspace", help="Project workspace directory")
    parser.add_argument("--json", action="store_true", help="Output structured JSON")
    args = parser.parse_args()

    ws = Path(args.workspace)
    if not ws.exists():
        print(f"Error: workspace not found: {ws}", file=sys.stderr)
        return 2

    report = inspect_workspace(ws)
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        print(format_markdown(report))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
