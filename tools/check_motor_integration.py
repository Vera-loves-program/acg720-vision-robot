#!/usr/bin/env python3
"""Check motor integration source structure without compiling/simulating HDL.

Only Python's standard library and read-only ``git show`` are used. These
checks do not elaborate Verilog, prove control behavior, check timing, or
validate hardware voltages. Run from any directory:

    python tools/check_motor_integration.py

The default baseline is the last commit before the motor integration. Use
--baseline REV to audit against a different known-good constraints revision.
"""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET


BASELINE = "bb991aaa3ebac422a2b9e4bc081a18f4614a5ed7"
NEW_MODULES = (
    "encoder_quadrature", "tb6612_axis_pwm", "motor_axis_control",
    "motor_drive_control", "motor_tuning_control", "motor_uart_commands",
    "motor_status_text", "pid_touch_controls",
)
SOURCE_MODULES = NEW_MODULES + (
    "vision_robot_ui_top", "lcd1024_interaction_ui", "sensor_demo_uart",
    "sensor_demo_commands",
)
# Physical P7 numbers, frozen board net names, RTL ports, GW5AT-60B balls.
# The physical orientation is lower-right 1, lower-left 2, ascending upward.
P7 = {
    1: ("GYRO_SCL", "sensor_scl", "A13"),
    2: ("GYRO_SDA", "sensor_sda", "A14"),
    3: ("LASER_ADDR_0", "laser_shut[0]", "A15"),
    4: ("LASER_ADDR_1", "laser_shut[1]", "A16"),
    5: ("LASER_ADDR_2", "laser_shut[2]", "A18"),
    6: ("LASER_ADDR_3", "laser_shut[3]", "A19"),
    7: ("BQ_A_SCL", "bq_a_scl", "F13"),
    8: ("BQ_A_SDA", "bq_a_sda", "F14"),
    9: ("TRIG", "ultrasonic_trig", "E13"),
    10: ("ECHO", "ultrasonic_echo", "E14"),
    13: ("UART0_RX", "bluetooth_rx", "C13"),
    14: ("UART0_TX", "bluetooth_tx", "B13"),
    15: ("BQ_B_SCL", "bq_b_scl", "D14"),
    16: ("BQ_B_SDA", "bq_b_sda", "D15"),
    17: ("AI2", "motor_ain2", "C14"),
    18: ("AI1", "motor_ain1", "C15"),
    19: ("BI1", "motor_bin1", "B15"),
    20: ("PWMA", "motor_pwma", "B16"),
    21: ("PWMB", "motor_pwmb", "D17"),
    22: ("BI2", "motor_bin2", "C17"),
    23: ("ENCODER1_A", "encoder1_a", "E16"),
    24: ("SERVO_1", "servo_1", "D16"),
    25: ("ENCODER2_A", "encoder2_a", "B17"),
    26: ("ENCODER1_B", "encoder1_b", "B18"),
    27: ("SERVO_0", "servo_0", "C18"),
    28: ("ENCODER2_B", "encoder2_b", "C19"),
    31: ("STP_PE0", "step_0", "F16"),
    33: ("STP_PE1", "step_1", "D20"),
    34: ("DIR_PE0", "dir_0", "C20"),
    36: ("DIR_PE1", "dir_1", "D19"),
}
UNUSED_P7 = {32: "E17", 35: "E19", 37: "F18", 38: "E18", 39: "F19", 40: "F20"}
POWER_P7 = {11: "5V", 12: "GND", 29: "3V3", 30: "GND"}
STRING = re.compile(r'"(?:\\.|[^"\\])*"')


def uncomment(text: str) -> str:
    """Remove comments while retaining quoted strings and source newlines."""
    token = re.compile(r'"(?:\\.|[^"\\])*"|//[^\n]*|/\*.*?\*/', re.S)
    return token.sub(lambda m: m[0] if m[0].startswith('"') else
                     "".join("\n" if c == "\n" else " " for c in m[0]), text)


def closing(text: str, opening: int) -> int:
    """Match one parenthesized list, ignoring parentheses inside strings."""
    depth = 0
    quoted = escaped = False
    for index in range(opening, len(text)):
        char = text[index]
        if quoted:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                quoted = False
        elif char == '"':
            quoted = True
        elif char == "(":
            depth += 1
        elif char == ")":
            depth -= 1
            if depth == 0:
                return index
    raise ValueError("Unclosed parenthesized source list")


def module_ports(source: str) -> dict[str, set[str]]:
    """Read ANSI module port names, not Verilog expression semantics."""
    result = {}
    for match in re.finditer(r"\bmodule\s+(\w+)\s*", source):
        cursor = match.end()
        if source[cursor:cursor + 1] == "#":
            cursor = source.index("(", cursor)
            cursor = closing(source, cursor) + 1
        while source[cursor].isspace():
            cursor += 1
        if source[cursor] != "(":
            raise ValueError(f"Expected ANSI port list for {match[1]}")
        end = closing(source, cursor)
        names = []
        direction = None
        for item in source[cursor + 1:end].split(","):
            declared = re.search(r"\b(input|output|inout)\b", item)
            if declared:
                direction = declared[1]
            if not direction:
                raise ValueError(f"Non-ANSI/unsupported port list in {match[1]}")
            # Remove optional declaration initializers before finding the name.
            item = item.split("=", 1)[0].strip()
            name = re.search(r"([A-Za-z_]\w*)\s*$", item)
            if not name:
                raise ValueError(f"Cannot read port declaration: {item!r}")
            names.append(name[1])
        if len(names) != len(set(names)):
            raise ValueError(f"Duplicate declared ports in {match[1]}")
        if match[1] in result:
            raise ValueError(f"Duplicate module declaration {match[1]}")
        result[match[1]] = set(names)
    return result


def instances(source: str, known: dict[str, set[str]]):
    """Find named instances of modules included in this integration audit."""
    for module in known:
        for match in re.finditer(rf"\b{re.escape(module)}\s*", source):
            cursor = match.end()
            if source[cursor:cursor + 1] == "#":
                cursor = source.index("(", cursor)
                cursor = closing(source, cursor) + 1
            instance = re.match(r"\s*([A-Za-z_]\w*)\s*\(", source[cursor:])
            if not instance:
                continue  # Module declaration has no intervening instance name.
            opening = cursor + instance.end() - 1
            end = closing(source, opening)
            names = re.findall(r"\.\s*(\w+)\s*\(", source[opening + 1:end])
            if not names:
                raise ValueError(f"Expected named connections: {module} {instance[1]}")
            yield module, instance[1], names


def cst_locations(source: str) -> dict[str, tuple[str, ...]]:
    result = {}
    for match in re.finditer(r'\bIO_LOC\s+"([^"]+)"\s+([^;]+);', uncomment(source)):
        name = match[1]
        balls = tuple(item.strip().upper() for item in match[2].split(","))
        if name in result:
            raise ValueError(f"Duplicate IO_LOC for {name}")
        if any(not re.fullmatch(r"[A-Z]+[0-9]+", ball) for ball in balls):
            raise ValueError(f"Invalid package ball in IO_LOC {name}: {balls}")
        result[name] = balls
    return result


def strings(text: str):
    for literal in STRING.findall(text):
        # Current audited UI strings are plain ASCII; escaped forms are not
        # silently interpreted as having a guessed Verilog byte length.
        if "\\" in literal:
            raise ValueError(f"Unsupported escaped display string: {literal}")
        yield literal[1:-1].encode("ascii")


def check_blocks(source: str, name: str) -> None:
    """Check lexical delimiters/block pairs; this is not an HDL parser."""
    masked = STRING.sub(lambda m: " " * len(m[0]), source)
    delimiters = []
    pairs = {")": "(", "]": "[", "}": "{"}
    for char in masked:
        if char in "([{":
            delimiters.append(char)
        elif char in ")]}":
            if not delimiters or delimiters.pop() != pairs[char]:
                raise ValueError(f"Unmatched source delimiter in {name}: {char}")
    if delimiters:
        raise ValueError(f"Unclosed source delimiters in {name}: {delimiters}")
    start = {"module": "module", "begin": "begin", "case": "case",
             "casez": "case", "casex": "case", "function": "function",
             "task": "task", "generate": "generate"}
    finish = {"endmodule": "module", "end": "begin", "endcase": "case",
              "endfunction": "function", "endtask": "task", "endgenerate": "generate"}
    blocks = []
    for token in re.findall(r"\b[A-Za-z_]\w*\b", masked):
        if token in start:
            blocks.append(start[token])
        elif token in finish:
            if not blocks or blocks.pop() != finish[token]:
                raise ValueError(f"Unmatched source block in {name}: {token}")
    if blocks:
        raise ValueError(f"Unclosed source blocks in {name}: {blocks}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", default=BASELINE, help="Known-good CST git revision")
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    errors = []

    def require(condition, message):
        if not condition:
            errors.append(message)

    def passed(message):
        print(f"CHECK {message}")

    try:
        project = ET.parse(repo / "vision_robot_ui.gprj")
        files = Counter(node.attrib.get("path", "").replace("\\", "/").lower()
                        for node in project.findall(".//File") if node.attrib.get("enable") == "1")
        for module in SOURCE_MODULES:
            relative = f"src/{module}.v"
            require(files[relative.lower()] == 1, f"Project must enable {relative} exactly once")
            require((repo / relative).is_file(), f"Source file missing: {relative}")
        passed(f"gprj XML: {len(SOURCE_MODULES)} integration sources checked")

        sources = {module: uncomment((repo / f"src/{module}.v").read_text(encoding="utf-8-sig"))
                   for module in SOURCE_MODULES}
        declared = {}
        for owner, source in sources.items():
            check_blocks(source, owner)
            for module, ports in module_ports(source).items():
                require(module not in declared, f"Repeated audited module {module}")
                declared[module] = ports
        instance_count = 0
        used_modules = set()
        for owner, source in sources.items():
            for module, instance, names in instances(source, declared):
                instance_count += 1
                used_modules.add(module)
                counts = Counter(names)
                require(all(count == 1 for count in counts.values()),
                        f"Duplicate named connection: {owner}/{instance}")
                actual = set(names)
                require(actual == declared[module],
                        f"{owner}/{instance}: missing={sorted(declared[module] - actual)}, "
                        f"unknown={sorted(actual - declared[module])}")
        for module in NEW_MODULES:
            require(module in used_modules, f"No audited instance of {module}")
        passed(f"ANSI declarations/named connections: {instance_count} instances checked")

        cst_text = (repo / "src/lcd_interaction.cst").read_text(encoding="utf-8-sig")
        locations = cst_locations(cst_text)
        owners = {}
        top_ports = declared["vision_robot_ui_top"]
        for port, balls in locations.items():
            require(port.split("[", 1)[0] in top_ports, f"CST port absent from top: {port}")
            for ball in balls:
                require(ball not in owners, f"Package ball {ball} repeated: {owners.get(ball)} and {port}")
                owners[ball] = port
        for pin, (_, port, ball) in P7.items():
            require(locations.get(port) == (ball,), f"P7-{pin}: {port} must use {ball}")
        for pin, ball in UNUSED_P7.items():
            require(ball not in owners, f"Unused/NC P7-{pin}/{ball} has GPIO constraint {owners.get(ball)}")
        for power_name in set(POWER_P7.values()):
            require(power_name not in locations, f"Fixed header supply {power_name} must not be GPIO")
        attributes = dict(re.findall(r'\bIO_PORT\s+"([^"]+)"\s+([^;]+);', uncomment(cst_text)))
        for _, port, _ in P7.values():
            attr = attributes.get(port, "")
            require("IO_TYPE=LVCMOS33" in attr and "BANK_VCCIO=3.3" in attr,
                    f"P7 port {port}: missing 3.3V bank/IO settings")
        for port in ("sensor_scl", "sensor_sda", "bq_a_scl", "bq_a_sda", "bq_b_scl", "bq_b_sda"):
            require("PULL_MODE=NONE" in attributes.get(port, ""), f"I2C {port}: weak pull must remain NONE")
        passed(f"CST: {len(locations)} IO_LOC statements, {len(owners)} unique balls, {len(P7)} P7 GPIO mappings")

        plan = (repo / "docs/扩展板规划.md").read_text(encoding="utf-8-sig")
        section = plan.split("### 28.2", 1)[1].split("### 28.3", 1)[0]
        doc_pins = {}
        for line in section.splitlines():
            cells = [item.strip() for item in line.strip().strip("|").split("|")]
            if len(cells) != 6 or not cells[0].isdigit():
                continue
            for offset in (0, 3):
                pin = int(cells[offset])
                ball = re.match(r"([A-Z]+[0-9]+)(?![A-Z0-9])", cells[offset + 2])
                doc_pins[pin] = (cells[offset + 1], ball[1] if ball else None)
        require(set(doc_pins) == set(range(1, 41)), "Planning section 28.2 must list all 40 physical pins")
        for pin, (net, _, ball) in P7.items():
            require(doc_pins.get(pin) == (net, ball), f"P7-{pin}: frozen map disagrees with planning table")
        for pin, net in POWER_P7.items():
            require(doc_pins.get(pin) == (net, None), f"P7-{pin}: fixed power/GND mapping changed")
        require(doc_pins.get(32) == ("NC", "E17"), "P7-32 must remain NC/E17")
        passed("Planning section 28.2: 40 physical pins, including 30=GND and 32=NC")

        baseline = subprocess.run(
            ["git", "-c", f"safe.directory={repo.as_posix()}", "show", f"{args.baseline}:src/lcd_interaction.cst"],
            cwd=repo, capture_output=True, text=True, encoding="utf-8", check=True,
        ).stdout
        old_locations = cst_locations(baseline)
        peripheral_ports = {port for _, port, _ in P7.values()}
        protected = {port: balls for port, balls in old_locations.items() if port not in peripheral_ports}
        for port, balls in protected.items():
            require(locations.get(port) == balls, f"Non-peripheral constraint changed: {port}: {balls} -> {locations.get(port)}")
        require(set(locations) - peripheral_ports == set(protected), "Unexpected added/removed non-peripheral IO_LOC")
        passed(f"Baseline {args.baseline[:12]}: {len(protected)} non-peripheral IO_LOC mappings preserved")

        # Pure literal assignments/ternaries to these 128-bit text containers
        # must contain exactly 16 ASCII bytes. Concatenations have fragments,
        # so they are checked separately by the owning source/HDL tool later.
        display_count = 0
        for module, source in sources.items():
            pattern = r"\b(?:label_text|state_row|scope_row|field_row|mode_row|field_label)\s*(?:<=|=)\s*([^;]+);"
            for statement in re.findall(pattern, source):
                if "{" in statement:
                    continue
                for value in strings(statement):
                    display_count += 1
                    require(len(value) == 16, f"{module}: 16-byte display literal is {len(value)} bytes: {value!r}")
        uart = sources["sensor_demo_uart"]
        length = int(re.search(r"\bMOTOR_PREFIX_LENGTH\s*=\s*(\d+)\s*;", uart)[1])
        literal = re.search(r'\bmotor_prefix\s*=\s*("(?:\\.|[^"\\])*")\s*;', uart)[1]
        prefix_bytes = next(strings(literal))
        require(len(prefix_bytes) == length, f"UART motor prefix length {len(prefix_bytes)} != {length}")
        index_width = re.search(r"reg\s*\[(\d+):0\]\s*motor_prefix_index\s*;", uart)
        require(index_width is not None and length <= 1 << (int(index_width[1]) + 1), "UART prefix index is too narrow")
        require(b"COUNT/10MS" in prefix_bytes and b"POS=COUNT" in prefix_bytes, "UART motor units missing")
        passed(f"Display literals: {display_count} fixed 16-byte strings; UART motor prefix={length} bytes")

        monitor = (repo / "tools/watch_sensor_demo.ps1").read_text(encoding="utf-8-sig")
        report_patterns = re.findall(r"-notmatch\s+'([^']+)'", monitor)
        require(len(report_patterns) == 2, "Monitor report-filter expressions not found")
        sensor_line = "I:OK U:OK GX +0000.00DPS SEQ=1 RXERR=0 LAST=P"
        motor_line = sensor_line + prefix_bytes.decode("ascii") + "MOTOR DISARMED  |L SPD +00000000 "
        for line in (sensor_line, motor_line):
            require(all(re.search(pattern, line) for pattern in report_patterns), f"Monitor rejects valid report: {line}")
        require(not all(re.search(pattern, "7113,1,112,0,41531,-8") for pattern in report_patterns), "Monitor accepts obsolete numeric CSV")
        command_patterns = re.findall(r"-match\s+'(\^@[^']+)'", monitor)
        require(len(command_patterns) == 1, "Monitor motor command regex missing")
        if command_patterns:
            for frame in ("@A", "@R", "@S", "@C", "@Z", "@U", "@H", "@W0", "@L1", "@F8", "@V-200", "@Q2", "@M1"):
                require(re.fullmatch(command_patterns[0], frame), f"Monitor rejects documented command {frame}")
            for frame in ("@R0", "@H123", "@W2", "@F9", "@Q3", "@M2", "@V", "@V--1"):
                require(not re.fullmatch(command_patterns[0], frame), f"Monitor accepts malformed command {frame}")
        passed("Monitor regex: sensor-only/PID-suffix reports and command syntax checked")
    except (OSError, ValueError, IndexError, TypeError, ET.ParseError, subprocess.CalledProcessError) as error:
        errors.append(f"Checker could not complete: {error}")

    if errors:
        for error in errors:
            print(f"FAIL {error}", file=sys.stderr)
        print(f"FAILED: {len(errors)} structural issues", file=sys.stderr)
        return 1
    print("PASS: source structure only; NO HDL compilation/simulation, timing, PCB, or electrical validation")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
