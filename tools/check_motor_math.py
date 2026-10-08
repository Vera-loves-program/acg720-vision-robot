#!/usr/bin/env python3
"""Independent arithmetic/specification checks for the motor-control design.

This checks integer ranges, Q8 arithmetic, quadrature transition definitions,
and reversal/limiting boundary rules. It does NOT execute Verilog, synthesize,
simulate clocked RTL, prove timing, or validate physical motors.
Run from any directory: python tools/check_motor_math.py
"""

from itertools import product
from pathlib import Path
import re


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def limit_symmetric(value, limit):
    """Specification: symmetrical clipping around zero, nonnegative limit."""
    require(limit >= 0, "negative limit")
    return min(max(value, -limit), limit)


def nearest_q8(value):
    """Specification: nearest integer, exact half-way ties away from zero."""
    magnitude, remainder = divmod(abs(value), 256)
    result = magnitude + int(remainder >= 128)
    return -result if value < 0 else result


def reference_integral_step(previous, error, kp, ki, delta_measurement, kd, limit):
    """One mathematical conditional-integration step; no clocked model."""
    candidate = limit_symmetric(previous + error * ki, limit * 256)
    raw = error * kp + candidate - delta_measurement * kd
    pushes_high = raw > limit * 256 and error > 0
    pushes_low = raw < -limit * 256 and error < 0
    if pushes_high or pushes_low:
        candidate = previous
        raw = error * kp + previous - delta_measurement * kd
    output = nearest_q8(limit_symmetric(raw, limit * 256))
    return candidate, limit_symmetric(output, limit)


def check_arithmetic_boundaries():
    cases = 0
    values = [-(1 << 31), -1000, -1, 0, 1, 1000, (1 << 31) - 1]
    for value, gain in product(values, [0, 1, 256, 65535]):
        require(-(1 << 63) <= value * gain < (1 << 63), "gain product overflow")
        cases += 1

    combinations = product(
        [-1000, -10, 0, 10, 1000],
        [-51200, 0, 51200],
        [0, 256, 65535],
        [0, 256, 65535],
        [-1000, 0, 1000],
        [0, 256, 65535],
        [0, 200, 900],
    )
    for error, previous, kp, ki, delta, kd, limit in combinations:
        previous = limit_symmetric(previous, limit * 256)
        candidate, output = reference_integral_step(
            previous, error, kp, ki, delta, kd, limit
        )
        require(abs(candidate) <= limit * 256, "integrator exceeded limit")
        require(abs(output) <= limit, "output exceeded limit")
        cases += 1

    require(
        reference_integral_step(100 * 256, 1000, 65535, 256, 0, 0, 200)[0]
        == 100 * 256,
        "positive saturation must freeze integration pushing further positive",
    )
    require(
        reference_integral_step(-100 * 256, -1000, 65535, 256, 0, 0, 200)[0]
        == -100 * 256,
        "negative saturation must freeze integration pushing further negative",
    )
    require(
        reference_integral_step(100 * 256, -10, 0, 256, 0, 0, 200)[0]
        < 100 * 256,
        "integration must be allowed to recover from saturation",
    )

    for raw in [-1000, -1, 0, 1, 1000]:
        state = 0
        for _ in range(128):
            # Integer alpha=1/4 IIR boundary check; this is not an RTL model.
            state += (raw * 256 - state) // 4
        require(nearest_q8(state) == raw, "IIR steady-state integer bias")
        cases += 1

    for value in [-32768, -200, 0, 200, 32767]:
        require(abs(limit_symmetric(value, 1000)) <= 1000, "target limit")
        cases += 1
    return cases


def check_rounding():
    for magnitude in range(2048):
        require(
            nearest_q8(-magnitude) == -nearest_q8(magnitude),
            "Q8 rounding must be symmetric",
        )
        require(
            abs(nearest_q8(magnitude) * 256 - magnitude) <= 128,
            "Q8 rounding error exceeded half one integer unit",
        )
    require(nearest_q8(128) == 1 and nearest_q8(-128) == -1, "half-way tie rule")


def check_quadrature_source():
    # Define the expected Gray sequence mathematically, independently of the
    # case table, then inspect only that literal transition table as text.
    # Parsing a table is a static check; no HDL execution or tool is involved.
    gray_cycle = [0b00, 0b01, 0b11, 0b10]
    expected = {}
    for index, start in enumerate(gray_cycle):
        expected[start, gray_cycle[(index + 1) % 4]] = 1
        expected[start, gray_cycle[(index - 1) % 4]] = -1
    source = Path(__file__).resolve().parents[1] / "src" / "encoder_quadrature.v"
    content = source.read_text(encoding="utf-8")
    actual = {}
    for entries, step in re.findall(
        r"((?:4'b[01]{4}\s*,?\s*)+):\s*step\s*=\s*(-?1)\s*;", content
    ):
        for bits in re.findall(r"4'b([01]{4})", entries):
            actual[int(bits[:2], 2), int(bits[2:], 2)] = int(step)
    require(actual == expected, "encoder literal case table differs from Gray sequence")
    forward = sum(expected[a, b] for a, b in zip(gray_cycle, gray_cycle[1:] + gray_cycle[:1]))
    backward_cycle = [0b00, 0b10, 0b11, 0b01]
    backward = sum(
        expected[a, b]
        for a, b in zip(backward_cycle, backward_cycle[1:] + backward_cycle[:1])
    )
    require(forward == 4 and backward == -4, "four-edge count convention")
    for pair in [(0, 3), (3, 0), (1, 2), (2, 1)]:
        require(pair not in actual, "diagonal transition counted as a valid edge")
    for endpoint, increment in [((1 << 31) - 1, 1), (-(1 << 31), -1)]:
        saturated = min(max(endpoint + increment, -(1 << 31)), (1 << 31) - 1)
        require(saturated == endpoint, "position endpoint must saturate, not wrap")


def check_reversal_and_slew_boundaries():
    # Check numerical command rules only. The physical 1 ms blank and TB6612
    # truth table still require RTL/hardware verification by the project owner.
    for old, requested in product([-900, -15, 0, 15, 900], [-900, -15, 0, 15, 900]):
        if old * requested < 0 or requested == 0:
            updated = 0
        else:
            updated = old + limit_symmetric(requested - old, 15)
        require(not (old * updated < 0), "PWM request reversed without passing zero")
        if old * requested >= 0 and requested != 0:
            require(abs(updated - old) <= 15, "PWM slew exceeded limit")
        if requested == 0:
            require(updated == 0, "STOP must bypass slew")
        require(abs(updated) <= 900, "PWM boundary exceeded hard cap")
    for requested, minimum, maximum in product([-900, -1, 0, 1, 900], [0, 80, 900], [0, 200, 900]):
        command = limit_symmetric(requested, maximum)
        floor = min(minimum, maximum)
        if command:
            command = (-1 if command < 0 else 1) * max(abs(command), floor)
        require(abs(command) <= maximum, "stiction floor escaped output limit")
        if requested == 0:
            require(command == 0, "stiction floor started a stopped motor")


def main():
    cases = check_arithmetic_boundaries()
    check_rounding()
    check_quadrature_source()
    check_reversal_and_slew_boundaries()
    print(
        f"PASS: {cases} integer/Q8/filter boundary cases; "
        "2048 rounding symmetry cases; encoder Gray table and saturation; "
        "PWM reversal, slew and minimum-duty boundaries."
    )
    print("Independent math and source-table checks only; no HDL compilation, RTL simulation, timing or board validation.")


if __name__ == "__main__":
    main()
