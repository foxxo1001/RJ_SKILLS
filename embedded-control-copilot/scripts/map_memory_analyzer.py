#!/usr/bin/env python3
"""
map_memory_analyzer.py
Zero-dependency Python 3 linker .map analyzer for Keil MDK (ARMCC/ARMClang) and GCC / ESP-IDF GNU ld.
Detects Flash/RAM hotspots, accidental double-precision soft-float bloat, dynamic heap usage, and small stacks.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

SOFT_DOUBLE_SYMBOLS = (
    "__aeabi_dadd",
    "__aeabi_dsub",
    "__aeabi_dmul",
    "__aeabi_ddiv",
    "__aeabi_d2f",
    "__aeabi_f2d",
    "__aeabi_d2iz",
    "__aeabi_cdcmple",
    "__adddf3",
    "__subdf3",
    "__muldf3",
    "__divdf3",
    "_printf_fp",
    "_dtoa_r",
    "__hardfp_sin",
    "__hardfp_cos",
    "__hardfp_atan2",
)

HEAP_SYMBOLS = ("malloc", "_malloc_r", "calloc", "realloc", "pvPortMalloc")


def read_text_safely(path: Path) -> str:
    raw = path.read_bytes()
    for enc in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return raw.decode(enc)
        except UnicodeDecodeError:
            continue
    return raw.decode("utf-8", errors="replace")


def parse_keil_map(text: str, top_n: int) -> Dict[str, Any]:
    totals: Dict[str, int] = {}
    for label, key in (
        (r"Total RO\s+Size\s*\([^)]*\)\s*=?\s*(\d+)", "ro_bytes"),
        (r"Total RW\s+Size\s*\([^)]*\)\s*=?\s*(\d+)", "rw_bytes"),
        (r"Total ROM\s+Size\s*\([^)]*\)\s*=?\s*(\d+)", "rom_bytes"),
    ):
        m = re.search(label, text)
        if m:
            totals[key] = int(m.group(1))

    # Parse per-object summary lines:
    # Code (inc. data)   RO Data    RW Data    ZI Data      Debug   Object Name
    objects: List[Dict[str, Any]] = []
    obj_re = re.compile(
        r"^\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+([A-Za-z0-9_.\-/\\]+\.(?:o|obj))\s*$",
        re.MULTILINE,
    )
    for m in obj_re.finditer(text):
        code_b = int(m.group(1))
        ro_b = int(m.group(3))
        rw_b = int(m.group(4))
        zi_b = int(m.group(5))
        objects.append({
            "name": m.group(7),
            "flash_bytes": code_b + ro_b + rw_b,
            "ram_bytes": rw_b + zi_b,
            "code": code_b,
            "ro_data": ro_b,
            "rw_data": rw_b,
            "zi_data": zi_b,
        })

    stack_bytes = None
    m_stack = re.search(r"\bSTACK\b.*?0x([0-9a-fA-F]+)\s+0x([0-9a-fA-F]+)", text)
    if m_stack:
        stack_bytes = int(m_stack.group(2), 16)
    else:
        m_stack2 = re.search(r"0x[0-9a-fA-F]+\s+0x([0-9a-fA-F]+)\s+Zero\s+RW\s+\d+\s+STACK", text)
        if m_stack2:
            stack_bytes = int(m_stack2.group(1), 16)

    top_flash = sorted(objects, key=lambda x: x["flash_bytes"], reverse=True)[:top_n]
    top_ram = sorted(objects, key=lambda x: x["ram_bytes"], reverse=True)[:top_n]

    return {
        "format": "keil_arm",
        "totals": totals,
        "stack_bytes": stack_bytes,
        "top_flash": top_flash,
        "top_ram": top_ram,
    }


def parse_gcc_map(text: str, top_n: int) -> Dict[str, Any]:
    regions: List[Dict[str, Any]] = []
    in_mem_cfg = False
    for line in text.splitlines():
        if line.strip().startswith("Memory Configuration"):
            in_mem_cfg = True
            continue
        if in_mem_cfg:
            if line.strip().startswith("Linker script"):
                in_mem_cfg = False
                continue
            m = re.match(r"^([A-Za-z0-9_]+)\s+0x([0-9a-fA-F]+)\s+0x([0-9a-fA-F]+)", line.strip())
            if m and m.group(1) != "*default*":
                regions.append({
                    "name": m.group(1),
                    "origin": f"0x{m.group(2)}",
                    "length_bytes": int(m.group(3), 16),
                })

    sec_totals: Dict[str, int] = {}
    obj_sizes: Dict[str, Tuple[int, int]] = {}  # obj -> (flash, ram)

    sec_re = re.compile(
        r"^\s*(\.(?:text|rodata|data|bss|iram0\.text|dram0\.data|dram0\.bss|ccmram)[A-Za-z0-9_.$]*)\s+"
        r"0x[0-9a-fA-F]+\s+0x([0-9a-fA-F]+)\s+(.+)$",
        re.MULTILINE,
    )
    for m in sec_re.finditer(text):
        sec = m.group(1)
        sz = int(m.group(2), 16)
        obj = m.group(3).strip()
        if sz == 0:
            continue
        root_sec = "." + sec.split(".")[1]
        sec_totals[root_sec] = sec_totals.get(root_sec, 0) + sz

        is_ram = any(k in sec for k in (".data", ".bss", "dram0", "ccmram"))
        is_flash = any(k in sec for k in (".text", ".rodata", ".data", "iram0"))
        old_f, old_r = obj_sizes.get(obj, (0, 0))
        obj_sizes[obj] = (old_f + (sz if is_flash else 0), old_r + (sz if is_ram else 0))

    stack_bytes = None
    m_st = re.search(r"_Min_Stack_Size\s*=\s*0x([0-9a-fA-F]+)", text)
    if m_st:
        stack_bytes = int(m_st.group(1), 16)
    else:
        m_st2 = re.search(r"\.stack\s+0x[0-9a-fA-F]+\s+0x([0-9a-fA-F]+)", text)
        if m_st2:
            stack_bytes = int(m_st2.group(1), 16)

    objs = [{"name": k, "flash_bytes": v[0], "ram_bytes": v[1]} for k, v in obj_sizes.items()]
    top_flash = sorted(objs, key=lambda x: x["flash_bytes"], reverse=True)[:top_n]
    top_ram = sorted(objs, key=lambda x: x["ram_bytes"], reverse=True)[:top_n]

    flash_sum = sec_totals.get(".text", 0) + sec_totals.get(".rodata", 0) + sec_totals.get(".data", 0) + sec_totals.get(".iram0", 0)
    ram_sum = sec_totals.get(".data", 0) + sec_totals.get(".bss", 0) + sec_totals.get(".dram0", 0)

    return {
        "format": "gnu_ld_gcc",
        "regions": regions,
        "section_totals": sec_totals,
        "totals": {"rom_bytes": flash_sum, "rw_bytes": ram_sum},
        "stack_bytes": stack_bytes,
        "top_flash": top_flash,
        "top_ram": top_ram,
    }


def analyze_map(path: Path, top_n: int) -> Dict[str, Any]:
    text = read_text_safely(path)
    is_keil = ("ARM Linker" in text) or ("Total RO  Size" in text) or ("Image Symbol Table" in text)
    parsed = parse_keil_map(text, top_n) if is_keil else parse_gcc_map(text, top_n)

    soft_doubles = [sym for sym in SOFT_DOUBLE_SYMBOLS if re.search(rf"\b{re.escape(sym)}\b", text)]
    heap_hits = [sym for sym in HEAP_SYMBOLS if re.search(rf"\b{re.escape(sym)}\b", text)]

    warnings: List[str] = []
    if soft_doubles:
        warnings.append(
            f"检测到软双精度浮点库符号被链接 ({', '.join(soft_doubles[:6])})！说明代码中混入了 double 字面量或双精度 math/printf，会严重拖慢实时控制环并浪费 Flash。"
        )
    if heap_hits:
        warnings.append(
            f"检测到动态内存分配符号 ({', '.join(heap_hits)})：电赛控制代码强烈建议零 malloc 静态预分配以防堆碎片死机。"
        )
    if parsed.get("stack_bytes") is not None and parsed["stack_bytes"] < 2048:
        warnings.append(
            f"主栈空间偏小 ({parsed['stack_bytes']} bytes < 2048)：若含姿态解算、运动学或 printf，建议扩大至 >= 4096 bytes。"
        )

    parsed["file"] = str(path)
    parsed["soft_double_symbols"] = soft_doubles
    parsed["heap_symbols"] = heap_hits
    parsed["warnings"] = warnings
    return parsed


def format_markdown(data: Dict[str, Any]) -> str:
    t = data.get("totals", {})
    lines = [
        "# Linker Map Memory Analysis",
        f"- **File**: `{data['file']}` (`{data['format']}`)",
        f"- **Estimated Flash / ROM**: `{t.get('rom_bytes', '?')}` bytes (`{t.get('rom_bytes', 0) / 1024:.2f} KB`)",
        f"- **Estimated RAM (RW+ZI/BSS)**: `{t.get('rw_bytes', '?')}` bytes (`{t.get('rw_bytes', 0) / 1024:.2f} KB`)",
    ]
    if data.get("stack_bytes") is not None:
        lines.append(f"- **Configured Stack Size**: `{data['stack_bytes']}` bytes")
    lines.append("")

    if data["warnings"]:
        lines.append("## ⚠️ Contest Risk Diagnostics")
        for w in data["warnings"]:
            lines.append(f"- {w}")
        lines.append("")
    else:
        lines.append("✅ **未发现软双精度污染、动态堆风险或栈过小隐患。**\n")

    if data.get("top_flash"):
        lines.append("## Top Flash Consumers")
        lines.append("| Module | Flash (B) | RAM (B) |")
        lines.append("| :--- | ---: | ---: |")
        for item in data["top_flash"][:10]:
            lines.append(f"| `{item['name']}` | {item['flash_bytes']} | {item['ram_bytes']} |")
        lines.append("")

    if data.get("top_ram"):
        lines.append("## Top RAM Consumers")
        lines.append("| Module | RAM (B) | Flash (B) |")
        lines.append("| :--- | ---: | ---: |")
        for item in data["top_ram"][:10]:
            if item["ram_bytes"] > 0:
                lines.append(f"| `{item['name']}` | {item['ram_bytes']} | {item['flash_bytes']} |")
        lines.append("")

    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description="Keil/GCC Linker .map Memory & Float-Bloat Analyzer")
    parser.add_argument("map_file", help="Path to .map file")
    parser.add_argument("--top", type=int, default=15, help="Top N modules to display")
    parser.add_argument("--json", action="store_true", help="Output structured JSON")
    args = parser.parse_args()

    p = Path(args.map_file)
    if not p.exists():
        print(f"Error: map file not found: {p}", file=sys.stderr)
        return 2

    data = analyze_map(p, args.top)
    if args.json:
        print(json.dumps(data, ensure_ascii=False, indent=2))
    else:
        print(format_markdown(data))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
