#!/usr/bin/env python3
"""
embedded_static_scan.py
Zero-dependency Python 3 static analyzer for embedded motion-control contest firmware.
Targets: STM32 (Keil/CubeMX), TI MSPM0 (CCS/SysConfig), ESP32 (ESP-IDF), Renesas RA (FSP).
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

SKIP_DIRS = {
    ".git",
    "build",
    "objects",
    "listings",
    "debug",
    "release",
    "cmsis",
    "middlewares",
    "fsp",
    "ra",
    "ra_gen",
    "ra_cfg",
    "managed_components",
    "drivers",
    "third_party",
    "__pycache__",
    "rte",
}

CODE_EXTS = {".c", ".h", ".cpp", ".hpp"}

TYPE_SIZES = {
    "char": 1,
    "signed char": 1,
    "unsigned char": 1,
    "int8_t": 1,
    "uint8_t": 1,
    "bool": 1,
    "short": 2,
    "int16_t": 2,
    "uint16_t": 2,
    "int": 4,
    "unsigned int": 4,
    "int32_t": 4,
    "uint32_t": 4,
    "float": 4,
    "int64_t": 8,
    "uint64_t": 8,
    "double": 8,
}

ISR_NAME_RE = re.compile(
    r"(?:IRQHandler|ISRHandler|ISR$|_isr$|_Callback$|^HAL_[A-Za-z0-9_]+Callback$|^GROUP1_IRQHandler$)"
)

BLOCKING_IN_ISR_RE = re.compile(
    r"\b(HAL_Delay|delay_ms|delay_us|Delay_ms|Delay_us|DL_Common_delayCycles|"
    r"vTaskDelay|vTaskDelayUntil|rt_thread_mdelay|rt_thread_delay|R_BSP_SoftwareDelay|"
    r"printf|vprintf|sprintf|snprintf|HAL_UART_Transmit|HAL_I2C_Master_Transmit|"
    r"HAL_I2C_Master_Receive|HAL_I2C_Mem_Read|HAL_I2C_Mem_Write|HAL_SPI_Transmit|"
    r"HAL_SPI_Receive|HAL_SPI_TransmitReceive)\s*\("
)

DOUBLE_MATH_RE = re.compile(
    r"(?<![A-Za-z0-9_>.-])(sin|cos|tan|asin|acos|atan|atan2|sqrt|pow|fabs|fmod|floor|ceil|exp|log)\s*\("
)

HW_POLL_WHILE_RE = re.compile(
    r"\bwhile\s*\(([^)]*(?:HAL_GPIO_ReadPin|->SR|->ISR|->STAT|->CSR|DL_UART_isBusy|"
    r"DL_I2C_|I2C_FLAG|USART_FLAG|SPI_I2S_GetFlagStatus|GetFlagStatus|__HAL_[A-Z0-9_]+_GET_FLAG)[^)]*)\)"
)

LOCAL_ARRAY_RE = re.compile(
    r"^\s*(?:const\s+)?(unsigned\s+char|signed\s+char|unsigned\s+int|uint8_t|int8_t|"
    r"uint16_t|int16_t|uint32_t|int32_t|uint64_t|int64_t|float|double|char|short|int|bool)"
    r"\s+([A-Za-z_][A-Za-z0-9_]*)\s*\[(\d+)\](?:\s*\[(\d+)\])?\s*(?:=|;)"
)

GLOBAL_VAR_RE = re.compile(
    r"^(?!\s*(?:typedef|extern|static\s+const|const|#|return|if|for|while|switch|struct|enum|union))"
    r"\s*(?:static\s+)?(volatile\s+)?"
    r"(?:unsigned\s+|signed\s+)?"
    r"(uint8_t|int8_t|uint16_t|int16_t|uint32_t|int32_t|uint64_t|int64_t|float|double|bool|char|short|int)"
    r"\s+([A-Za-z_][A-Za-z0-9_]*)\s*(?:\[[^\]]*\])?\s*(?:=[^;]*)?;"
)


@dataclass
class Finding:
    severity: str
    rule_id: str
    file: str
    line: int
    message: str
    hint: str


def read_text_safely(path: Path) -> str:
    raw = path.read_bytes()
    for enc in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return raw.decode(enc)
        except UnicodeDecodeError:
            continue
    return raw.decode("utf-8", errors="replace")


def strip_comments_and_strings(text: str) -> str:
    """Strip C/C++ comments and string/char literals while preserving exact line numbers."""
    out: List[str] = []
    i = 0
    n = len(text)
    state = "CODE"
    while i < n:
        ch = text[i]
        nxt = text[i + 1] if i + 1 < n else ""
        if state == "CODE":
            if ch == "/" and nxt == "/":
                state = "LINE_COMMENT"
                out.append("  ")
                i += 2
                continue
            if ch == "/" and nxt == "*":
                state = "BLOCK_COMMENT"
                out.append("  ")
                i += 2
                continue
            if ch == '"':
                state = "STRING"
                out.append('"')
                i += 1
                continue
            if ch == "'":
                state = "CHAR"
                out.append("'")
                i += 1
                continue
            out.append(ch)
            i += 1
        elif state == "LINE_COMMENT":
            if ch == "\n":
                state = "CODE"
                out.append("\n")
            else:
                out.append(" ")
            i += 1
        elif state == "BLOCK_COMMENT":
            if ch == "*" and nxt == "/":
                state = "CODE"
                out.append("  ")
                i += 2
                continue
            out.append("\n" if ch == "\n" else " ")
            i += 1
        elif state == "STRING":
            if ch == "\\" and i + 1 < n:
                out.append("  " if nxt != "\n" else " \n")
                i += 2
                continue
            if ch == '"':
                state = "CODE"
                out.append('"')
            else:
                out.append("\n" if ch == "\n" else " ")
            i += 1
        elif state == "CHAR":
            if ch == "\\" and i + 1 < n:
                out.append("  ")
                i += 2
                continue
            if ch == "'":
                state = "CODE"
                out.append("'")
            else:
                out.append("\n" if ch == "\n" else " ")
            i += 1
    return "".join(out)


@dataclass
class FuncSpan:
    name: str
    start_line: int
    end_line: int
    header_text: str
    is_isr: bool
    has_iram_attr: bool


def extract_functions(clean_text: str, extra_isr_names: Set[str]) -> List[FuncSpan]:
    """Identify top-level C function definitions and their exact 1-based line ranges."""
    lines = clean_text.splitlines()
    funcs: List[FuncSpan] = []
    brace_depth = 0
    current_func: Optional[FuncSpan] = None

    sig_re = re.compile(
        r"([A-Za-z_][A-Za-z0-9_]*)\s*\([^;{}]*\)\s*\{?\s*$"
    )
    non_func_keywords = {"if", "for", "while", "switch", "else", "do", "return", "sizeof", "catch"}

    for idx, line in enumerate(lines, start=1):
        stripped = line.strip()
        if brace_depth == 0:
            if "{" in line and not stripped.startswith("#"):
                # Collect multi-line function signature, stopping cleanly at previous statements/directives
                hdr_parts: List[str] = [stripped]
                for back in range(idx - 2, max(-1, idx - 6), -1):
                    prev = lines[back].strip()
                    if not prev:
                        continue
                    if prev.startswith("#") or prev.endswith(";") or prev.endswith("}") or "{" in prev:
                        break
                    hdr_parts.append(prev)
                hdr_parts.reverse()
                header = " ".join(hdr_parts)
                pre_brace = header.split("{", 1)[0].strip()
                m = sig_re.search(pre_brace)
                if m and m.group(1) not in non_func_keywords and "=" not in pre_brace.split("(", 1)[0]:
                    fname = m.group(1)
                    has_iram = "IRAM_ATTR" in pre_brace
                    is_isr = bool(ISR_NAME_RE.search(fname)) or fname in extra_isr_names
                    current_func = FuncSpan(
                        name=fname,
                        start_line=idx,
                        end_line=idx,
                        header_text=pre_brace,
                        is_isr=is_isr,
                        has_iram_attr=has_iram,
                    )

        brace_depth += line.count("{") - line.count("}")
        if brace_depth < 0:
            brace_depth = 0

        if current_func is not None and brace_depth == 0:
            current_func.end_line = idx
            funcs.append(current_func)
            current_func = None

    return funcs


def detect_platform(all_texts: Dict[Path, str], requested: str) -> Set[str]:
    if requested != "auto":
        return {requested}
    joined = "\n".join(all_texts.values())
    platforms: Set[str] = set()
    if re.search(r"stm32[a-z0-9]+xx|HAL_TIM|HAL_UART|__HAL_TIM_GET_COUNTER", joined):
        platforms.add("stm32")
    if re.search(r"ti_msp_dl_config\.h|DL_Timer|DL_GPIO|DL_UART|GROUP1_IRQHandler", joined):
        platforms.add("mspm0")
    if re.search(r"esp_timer|xTaskCreatePinnedToCore|IRAM_ATTR|driver/mcpwm|driver/pulse_cnt|freertos/FreeRTOS\.h", joined):
        platforms.add("esp32")
    if re.search(r"hal_data\.h|R_GPT_Open|R_SCI_UART_Open|fsp_err_t", joined):
        platforms.add("renesas")
    return platforms


def collect_files(target: Path) -> List[Path]:
    if target.is_file():
        return [target] if target.suffix.lower() in CODE_EXTS else []
    files: List[Path] = []
    for root, dirs, filenames in os.walk(target):
        dirs[:] = [
            d for d in dirs
            if d.lower() not in SKIP_DIRS
            and not d.lower().startswith("stm32")
        ]
        for fn in sorted(filenames):
            p = Path(root) / fn
            if p.suffix.lower() in CODE_EXTS:
                files.append(p)
    return files


def scan_files(files: List[Path], requested_platform: str) -> Tuple[Set[str], List[Finding]]:
    raw_texts: Dict[Path, str] = {}
    clean_texts: Dict[Path, str] = {}
    for f in files:
        raw = read_text_safely(f)
        raw_texts[f] = raw
        clean_texts[f] = strip_comments_and_strings(raw)

    platforms = detect_platform(raw_texts, requested_platform)
    findings: List[Finding] = []
    joined_clean = "\n".join(clean_texts.values())

    # Collect ESP32 explicitly registered ISR callback names across project
    registered_esp_isrs: Dict[str, Tuple[Path, int]] = {}
    for f, clean in clean_texts.items():
        for lineno, line in enumerate(clean.splitlines(), start=1):
            m1 = re.search(r"gpio_isr_handler_add\s*\([^,]+,\s*([A-Za-z_][A-Za-z0-9_]*)", line)
            if m1:
                registered_esp_isrs[m1.group(1)] = (f, lineno)
            m2 = re.search(r"\.on_alarm\s*=\s*([A-Za-z_][A-Za-z0-9_]*)", line)
            if m2:
                registered_esp_isrs[m2.group(1)] = (f, lineno)

    esp_isr_names = set(registered_esp_isrs.keys())

    # Project-level checks
    if "stm32" in platforms:
        if "HAL_UARTEx_ReceiveToIdle_DMA" in joined_clean and "DMA_IT_HT" not in joined_clean:
            for f, clean in clean_texts.items():
                for lineno, line in enumerate(clean.splitlines(), start=1):
                    if "HAL_UARTEx_ReceiveToIdle_DMA" in line:
                        findings.append(Finding(
                            "WARNING",
                            "STM32_UART_IDLE_HT_TRAP",
                            str(f),
                            lineno,
                            "调用 HAL_UARTEx_ReceiveToIdle_DMA 但未关闭 DMA 半传输中断 (DMA_IT_HT)。",
                            "在调用后紧跟 __HAL_DMA_DISABLE_IT(huartX.hdmaRx, DMA_IT_HT)，否则缓冲区收到一半会误触发 RxEventCallback。",
                        ))
                        break

    if "mspm0" in platforms:
        has_nvic_enable = "NVIC_EnableIRQ" in joined_clean
        if not has_nvic_enable:
            for f, clean in clean_texts.items():
                for lineno, line in enumerate(clean.splitlines(), start=1):
                    if re.search(r"\b(?:TIMER|TIMG|TIMA|UART|GROUP1|ADC)[A-Za-z0-9_]*_IRQHandler\b", line):
                        findings.append(Finding(
                            "WARNING",
                            "MSPM0_MISSING_NVIC_ENABLE",
                            str(f),
                            lineno,
                            "定义了 MSPM0 中断服务函数，但工程中未检测到 NVIC_EnableIRQ() 使能调用。",
                            "TI SysConfig 默认不会在 SYSCFG_DL_init() 内开启 NVIC，请务必在初始化后手动调用 NVIC_EnableIRQ(<INST>_INT_IRQN)。",
                        ))
                        break

    if "renesas" in platforms:
        if "R_GPT_Open" in joined_clean and "R_GPT_Start" not in joined_clean:
            for f, clean in clean_texts.items():
                for lineno, line in enumerate(clean.splitlines(), start=1):
                    if "R_GPT_Open" in line:
                        findings.append(Finding(
                            "CRITICAL",
                            "RENESAS_GPT_NOT_STARTED",
                            str(f),
                            lineno,
                            "调用了 R_GPT_Open() 但整个扫描工程中未调用 R_GPT_Start()。",
                            "瑞萨 FSP 驱动 Open 后定时器/PWM 不会自动运行，必须显式调用 R_GPT_Start(&g_timerX_ctrl)。",
                        ))
                        break

    # Per-file checks
    for f in files:
        raw = raw_texts[f]
        clean = clean_texts[f]
        raw_lines = raw.splitlines()
        clean_lines = clean.splitlines()
        funcs = extract_functions(clean, esp_isr_names)

        # Build line -> FuncSpan lookup
        line_to_func: Dict[int, FuncSpan] = {}
        for fn in funcs:
            for ln in range(fn.start_line, fn.end_line + 1):
                line_to_func[ln] = fn

        # 1. CubeMX USER CODE marker balance check (raw text)
        if "USER CODE BEGIN" in raw:
            stack: List[Tuple[str, int]] = []
            for lineno, rline in enumerate(raw_lines, start=1):
                mb = re.search(r"USER CODE BEGIN\s+([A-Za-z0-9_ ]+?)\s*\*/", rline)
                me = re.search(r"USER CODE END\s+([A-Za-z0-9_ ]+?)\s*\*/", rline)
                if mb:
                    stack.append((mb.group(1).strip(), lineno))
                elif me:
                    tag = me.group(1).strip()
                    if not stack or stack[-1][0] != tag:
                        findings.append(Finding(
                            "CRITICAL",
                            "STM32_CUBEMX_UNBALANCED_MARKERS",
                            str(f),
                            lineno,
                            f"STM32CubeMX 保护标记 USER CODE END '{tag}' 缺失配对的 BEGIN 标记。",
                            "重新生成 CubeMX 工程会导致代码被清空或错乱，请恢复完整的 /* USER CODE BEGIN/END */ 对。",
                        ))
                    else:
                        stack.pop()
            for tag, ln in stack:
                findings.append(Finding(
                    "CRITICAL",
                    "STM32_CUBEMX_UNBALANCED_MARKERS",
                    str(f),
                    ln,
                    f"STM32CubeMX 保护标记 USER CODE BEGIN '{tag}' 未闭合。",
                    "补全对应的 /* USER CODE END */，避免重开 CubeMX 丢失用户代码。",
                ))

        # 2. Collect file-scope non-volatile globals for ISR-shared check
        non_vol_globals: Dict[str, int] = {}
        brace_depth = 0
        for lineno, line in enumerate(clean_lines, start=1):
            if brace_depth == 0:
                mg = GLOBAL_VAR_RE.match(line)
                if mg and not mg.group(1):
                    vname = mg.group(3)
                    non_vol_globals[vname] = lineno
            brace_depth += line.count("{") - line.count("}")
            if brace_depth < 0:
                brace_depth = 0

        # Track writes in ISR and reads/refs outside ISR
        isr_written: Dict[str, Tuple[int, str]] = {}
        non_isr_used: Set[str] = set()
        if non_vol_globals:
            for fn in funcs:
                body = "\n".join(clean_lines[fn.start_line - 1 : fn.end_line])
                for gname in non_vol_globals:
                    if fn.is_isr:
                        write_pat = re.compile(
                            rf"\b{re.escape(gname)}\b\s*(?:\[[^\]]*\])?\s*(?:=(?!=)|\+=|-=|\*=|/=|\|=|&=|\+\+|--)|(?:\+\+|--)\s*\b{re.escape(gname)}\b"
                        )
                        if write_pat.search(body):
                            isr_written[gname] = (fn.start_line, fn.name)
                    else:
                        if re.search(rf"\b{re.escape(gname)}\b", body):
                            non_isr_used.add(gname)
            for gname, (isr_ln, isr_fn) in isr_written.items():
                if gname in non_isr_used:
                    decl_ln = non_vol_globals[gname]
                    findings.append(Finding(
                        "CRITICAL",
                        "MISSING_VOLATILE_ISR_SHARED",
                        str(f),
                        decl_ln,
                        f"全局变量 '{gname}' 在中断函数 {isr_fn}() (line {isr_ln}) 中被修改并在主逻辑中读取，但声明缺少 volatile。",
                        f"为 '{gname}' 添加 volatile 修饰（Keil AC6/GCC -O2 会把非 volatile 变量缓存到寄存器导致死循环）；若是多字段结构体，还需加短临界区或双缓冲防撕裂。",
                    ))

        # 3. Function-level checks
        for fn in funcs:
            # ESP32 registered ISR missing IRAM_ATTR
            if fn.name in esp_isr_names and not fn.has_iram_attr:
                findings.append(Finding(
                    "CRITICAL",
                    "ESP32_ISR_MISSING_IRAM",
                    str(f),
                    fn.start_line,
                    f"ESP32 注册的中断回调 '{fn.name}' 缺少 IRAM_ATTR 修饰。",
                    "加上 static void IRAM_ATTR 修饰，否则 Wi-Fi/NVS/Flash 操作禁用 Cache 时会立刻触发 Guru Meditation Panic。",
                ))

            # Scan lines inside function
            fn_body_lines = clean_lines[fn.start_line - 1 : fn.end_line]
            for offset, line in enumerate(fn_body_lines):
                lineno = fn.start_line + offset

                # ISR Blocking calls
                if fn.is_isr:
                    mb = BLOCKING_IN_ISR_RE.search(line)
                    if mb:
                        findings.append(Finding(
                            "CRITICAL",
                            "ISR_BLOCKING_CALL",
                            str(f),
                            lineno,
                            f"中断/回调函数 {fn.name}() 中调用了阻塞或重型函数 '{mb.group(1)}()'。",
                            "中断中严禁延时/阻塞收发/格式化打印！请改为置位 volatile 标志或压入 SPSC RingBuffer / FreeRTOS 队列，交由主循环处理。",
                        ))
                    if "mspm0" in platforms and re.search(r"\b(sinf?|cosf?|tanf?|atan2f?|sqrtf?)\s*\(", line):
                        findings.append(Finding(
                            "WARNING",
                            "MSPM0_HEAVY_FLOAT_IN_ISR",
                            str(f),
                            lineno,
                            f"MSPM0 (Cortex-M0+ 无硬件 FPU) 中断 {fn.name}() 中调用了重型浮点数学函数。",
                            "请改用 MSPM0 MathACL 硬件加速器、查表/定点数，或将运算移出中断到时间片任务。",
                        ))

                # Large local stack buffer
                if "static" not in line:
                    ma = LOCAL_ARRAY_RE.match(line)
                    if ma:
                        tname = re.sub(r"\s+", " ", ma.group(1))
                        elem_sz = TYPE_SIZES.get(tname, 4)
                        dim1 = int(ma.group(3))
                        dim2 = int(ma.group(4)) if ma.group(4) else 1
                        total_bytes = elem_sz * dim1 * dim2
                        if total_bytes >= 512:
                            findings.append(Finding(
                                "WARNING",
                                "LARGE_STACK_BUFFER",
                                str(f),
                                lineno,
                                f"函数 {fn.name}() 内部存在大栈数组 '{ma.group(2)}'（约 {total_bytes} 字节），极易引发栈溢出与 HardFault。",
                                "将其改为 static 或文件作用域静态缓冲区（尤其是瑞萨 FSP 默认栈仅 1KB 或 FreeRTOS 小栈任务中）。",
                            ))

                # ESP32 / RTOS vTaskDelay in control loop
                if re.search(r"\bvTaskDelay\s*\(", line):
                    fn_body_all = "\n".join(fn_body_lines).lower()
                    if any(k in fn_body_all for k in ("pid", "imu", "encoder", "motor", "attitude", "gimbal")):
                        findings.append(Finding(
                            "WARNING",
                            "ESP32_CONTROL_VTASKDELAY_DRIFT",
                            str(f),
                            lineno,
                            f"控制/传感任务 {fn.name}() 中使用了相对延时 vTaskDelay()，会导致控制周期 dt 随运算耗时漂移。",
                            "改用绝对周期延时 vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(period_ms)) 或硬件定时器通知。",
                        ))

        # 4. Line-level checks across file
        for lineno, line in enumerate(clean_lines, start=1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue

            # Double-precision math functions
            md = DOUBLE_MATH_RE.search(line)
            if md and "double" not in line:
                findings.append(Finding(
                    "WARNING",
                    "IMPLICIT_DOUBLE_PROMOTION",
                    str(f),
                    lineno,
                    f"调用了双精度数学函数 '{md.group(1)}()'，在单精度 FPU (STM32F4/G4/ESP32-S3/RA) 或 M0+ 上会触发软件双精度模拟并拖慢数十倍。",
                    f"请替换为单精度版本 '{md.group(1)}f()'（如 sinf / atan2f / sqrtf / fabsf）。",
                ))

            # Unsuffixed float decimal literals in float expressions
            if "double" not in line and not re.match(r"^\s*(?:const\s+)?double\b", line):
                has_float_context = bool(
                    re.search(r"\bfloat\b|\bf\b|[0-9]+f\b|\bkp\b|\bki\b|\bkd\b|\bdt\b|\berr\b|\bpid\b|\byaw\b|\bpitch\b|\broll\b", line, re.IGNORECASE)
                )
                if has_float_context:
                    unsuffixed = re.findall(r"(?<![A-Za-z0-9_])(\d+\.\d+(?:[eE][+-]?\d+)?)(?![fFlL0-9A-Za-z_])", line)
                    if unsuffixed:
                        findings.append(Finding(
                            "WARNING",
                            "IMPLICIT_DOUBLE_PROMOTION",
                            str(f),
                            lineno,
                            f"浮点表达式中存在未带 'f' 后缀的双精度字面量 ({', '.join(unsuffixed[:3])})，会强制将整行提升为 double 软件运算。",
                            f"为所有单精度常量补上 f 后缀（如 {unsuffixed[0]}f），消除软浮点库开销。",
                        ))

            # Unbounded hardware polling while loop
            mw = HW_POLL_WHILE_RE.search(line)
            if mw:
                window = "\n".join(clean_lines[lineno - 1 : min(len(clean_lines), lineno + 4)])
                has_timeout = bool(re.search(r"timeout|cnt|retry|tick|break|HAL_GetTick|--", window, re.IGNORECASE))
                if not has_timeout:
                    findings.append(Finding(
                        "WARNING",
                        "UNBOUNDED_HW_POLLING",
                        str(f),
                        lineno,
                        "检测到无超时退出的硬件状态标志死等 while 循环，电机电磁干扰或总线毛刺会导致主控永久卡死。",
                        "务必加入超时计数器（如 uint32_t timeout = 10000u; while(... && --timeout)）并在超时后执行外设错误复位。",
                    ))

            # Unsafe acosf / asinf domain
            if re.search(r"\b(acosf|asinf)\s*\(", line):
                window = "\n".join(clean_lines[max(0, lineno - 4) : lineno + 1])
                if not re.search(r"clamp|fminf|fmaxf|>=\s*1\.0f|<=\s*-1\.0f|>\s*1\.0f|<\s*-1\.0f", window, re.IGNORECASE):
                    findings.append(Finding(
                        "WARNING",
                        "UNSAFE_TRIG_DOMAIN",
                        str(f),
                        lineno,
                        "调用 acosf()/asinf() 前未发现 [-1.0f, 1.0f] 定义域钳位保护，浮点微误差 (>1.000001f) 会产生 NaN 穿透到电机 PWM。",
                        "先用 clampf(val, -1.0f, 1.0f) 限幅后再送入 acosf()/asinf()。",
                    ))

            # STM32 16-bit encoder unsigned wrap trap
            if "stm32" in platforms and re.search(r"__HAL_TIM_GET_COUNTER|TIM\d+->CNT", line):
                if "(int16_t)" not in line and "(short)" not in line:
                    if re.search(r"=|-(?!=)", line) and not re.search(r"uint16_t\s+[A-Za-z_]", line):
                        findings.append(Finding(
                            "CRITICAL",
                            "STM32_ENCODER_UNSIGNED_WRAP",
                            str(f),
                            lineno,
                            "读取 16 位编码器计数值时缺少 (int16_t) 有符号强转，电机反转跨越 0 时会产生 65535 阶跃跳变导致飞车！",
                            "使用自由回绕差分：int16_t delta = (int16_t)((uint16_t)(now_cnt - last_cnt));",
                        ))

            # ESP32 xTaskCreate without Core Pinning
            if "esp32" in platforms and re.search(r"\bxTaskCreate\s*\(", line):
                findings.append(Finding(
                    "WARNING",
                    "ESP32_TASK_NO_CORE_PIN",
                    str(f),
                    lineno,
                    "ESP32 上调用了未绑定核心的 xTaskCreate()，控制任务可能被调度到运行 Wi-Fi/BLE 协议栈的 Core 0 引起抖动。",
                    "请改用 xTaskCreatePinnedToCore(..., 1) 将 IMU/PID/运动控制任务锁死在干净的 Core 1。",
                ))

        # Keil semihosting printf trap
        if "stm32" in platforms and re.search(r"\bint\s+fputc\s*\(", clean):
            if "__use_no_semihosting" not in raw and "_sys_exit" not in clean:
                findings.append(Finding(
                    "WARNING",
                    "KEIL_SEMIHOSTING_PRINTF",
                    str(f),
                    1,
                    "重定义了 fputc() 但未见 __use_no_semihosting / _sys_exit() 桩函数，Keil 未勾选 MicroLIB 时脱机运行会卡死在 BKPT 0xAB。",
                    "在 Keil Target 中勾选 Use MicroLIB，或补齐兼容 AC5/AC6 的禁用半主机 _sys_exit() 代码块。",
                ))

    # Deduplicate exact (rule_id, file, line)
    seen: Set[Tuple[str, str, int]] = set()
    unique: List[Finding] = []
    for item in findings:
        key = (item.rule_id, item.file, item.line)
        if key not in seen:
            seen.add(key)
            unique.append(item)

    sev_rank = {"CRITICAL": 0, "WARNING": 1, "INFO": 2}
    unique.sort(key=lambda x: (sev_rank.get(x.severity, 9), x.file, x.line))
    return platforms, unique


def format_markdown(target: Path, scanned_count: int, platforms: Set[str], findings: List[Finding]) -> str:
    crit = sum(1 for x in findings if x.severity == "CRITICAL")
    warn = sum(1 for x in findings if x.severity == "WARNING")
    plat_str = ", ".join(sorted(platforms)) if platforms else "generic-c99"

    lines = [
        f"# Embedded Static Scan Report",
        f"- **Target**: `{target}`",
        f"- **Files Scanned**: {scanned_count}",
        f"- **Detected Platform(s)**: `{plat_str}`",
        f"- **Summary**: **{crit} CRITICAL**, **{warn} WARNING**",
        "",
    ]
    if not findings:
        lines.append("✅ **未发现高危嵌入式反模式（0 CRITICAL / 0 WARNING）。**")
        return "\n".join(lines)

    for idx, f in enumerate(findings, start=1):
        badge = "🔴 CRITICAL" if f.severity == "CRITICAL" else "🟠 WARNING"
        lines.append(f"### {idx}. [{badge}] `{f.rule_id}` — `{f.file}:{f.line}`")
        lines.append(f"- **问题**: {f.message}")
        lines.append(f"- **修复建议**: {f.hint}")
        lines.append("")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description="Embedded Control Contest C/C++ Static Scanner")
    parser.add_argument("target", help="Source file or project directory to scan")
    parser.add_argument(
        "--platform",
        choices=["auto", "stm32", "mspm0", "esp32", "renesas"],
        default="auto",
        help="Target MCU platform (default: auto-detect)",
    )
    parser.add_argument("--json", action="store_true", help="Output structured JSON")
    args = parser.parse_args()

    target = Path(args.target)
    if not target.exists():
        print(f"Error: target path not found: {target}", file=sys.stderr)
        return 2

    files = collect_files(target)
    platforms, findings = scan_files(files, args.platform)

    if args.json:
        payload = {
            "target": str(target),
            "files_scanned": len(files),
            "platforms": sorted(platforms),
            "summary": {
                "critical": sum(1 for x in findings if x.severity == "CRITICAL"),
                "warning": sum(1 for x in findings if x.severity == "WARNING"),
            },
            "findings": [asdict(x) for x in findings],
        }
        print(json.dumps(payload, ensure_ascii=False, indent=2))
    else:
        print(format_markdown(target, len(files), platforms, findings))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
