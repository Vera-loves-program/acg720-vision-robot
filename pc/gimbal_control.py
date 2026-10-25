"""Two position-servo axes with an opt-in Linux hardware PWM backend.

No device writes occur on import or construction. The default backend is a
mock. Real outputs require an explicit manual_enable command from the user;
the legacy arm API retains its explicit physical=True and configured limits.
The caller drives tick() at about50Hz; PWM itself comes from the SoC, not a
Python GPIO timing loop. A selected start pulse is applied immediately, since
an unpowered servo's physical position cannot be inferred by this software.

Linux ABI: https://docs.kernel.org/driver-api/pwm.html
OrangePi5 RK3588S pinmux/overlay sources (match the installed board/kernel):
https://github.com/orangepi-xunlong/linux-orangepi/tree/orange-pi-5.10-rk3588/arch/arm64/boot/dts/rockchip/overlay
The pwmchip number is discovery order, not the SoC PWM channel number. Map
explicit exported pwm paths after checking device/of_node; never guess N.
"""
from __future__ import annotations

import atexit
import bisect
from copy import deepcopy
from dataclasses import dataclass
import math
from pathlib import Path
import re
import threading
import time
from typing import Any, Callable, Mapping

PERIOD_NS = 20_000_000
MIN_PULSE_US = 1
MAX_PULSE_US = PERIOD_NS // 1000 - 1
DEFAULT_EXPLORE_MIN_US = 500
DEFAULT_EXPLORE_MAX_US = 2500
AXES = ("pan", "tilt")


class GimbalError(ValueError):
    """Invalid calibration, unsafe request, or unavailable PWM output."""


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise GimbalError(f"{name} 必须填写有限数值")
    result = float(value)
    if not math.isfinite(result):
        raise GimbalError(f"{name} 必须填写有限数值")
    return result


def _pulse(value: Any, name: str = "pulse_us") -> int:
    result = _number(value, name)
    if result != int(result) or not MIN_PULSE_US <= result <= MAX_PULSE_US:
        raise GimbalError(f"{name} 必须是{MIN_PULSE_US}～{MAX_PULSE_US}µs之间的整数，且小于20ms周期")
    return int(result)


def default_config() -> dict:
    """Unconfigured safe default; no guessed mechanical limits or angles."""
    axis = dict(safe_min_us=None, safe_max_us=None, fine_us=5, coarse_us=50,
                explore_min_us=DEFAULT_EXPLORE_MIN_US, explore_max_us=DEFAULT_EXPLORE_MAX_US,
                slew_us_per_s=150, calibration_points=[],
                calibration_confirmed=False)
    return {"axes": {name: deepcopy(axis) for name in AXES}}


@dataclass(frozen=True)
class AxisConfig:
    safe_min_us: int | None
    safe_max_us: int | None
    fine_us: int
    coarse_us: int
    slew_us_per_s: float
    calibration_points: tuple[tuple[float, int], ...]
    calibration_confirmed: bool
    explore_min_us: int = DEFAULT_EXPLORE_MIN_US
    explore_max_us: int = DEFAULT_EXPLORE_MAX_US

    @classmethod
    def parse(cls, data: Mapping[str, Any]) -> "AxisConfig":
        if not isinstance(data, Mapping):
            raise GimbalError("axis config must be an object")
        explore_low = _pulse(data.get('explore_min_us', DEFAULT_EXPLORE_MIN_US), 'explore_min_us')
        explore_high = _pulse(data.get('explore_max_us', DEFAULT_EXPLORE_MAX_US), 'explore_max_us')
        if explore_low >= explore_high:
            raise GimbalError('探索脉宽下限必须小于上限；它们不是机构已确认限位')
        low, high = data.get("safe_min_us"), data.get("safe_max_us")
        if (low is None) != (high is None):
            raise GimbalError("set both safe_min_us and safe_max_us")
        if low is not None:
            low, high = _pulse(low, "safe_min_us"), _pulse(high, "safe_max_us")
            if low >= high:
                raise GimbalError("safe_min_us must be less than safe_max_us")
            if low < explore_low or high > explore_high:
                raise GimbalError('已记录的机械限位必须位于本轴探索脉宽范围内')
        fine = _number(data.get("fine_us", 5), "fine_us")
        coarse = _number(data.get("coarse_us", 50), "coarse_us")
        if fine != int(fine) or coarse != int(coarse) or not 1 <= fine <= coarse <= 2000:
            raise GimbalError("step sizes must be integers:1<=fine_us<=coarse_us<=2000")
        slew = _number(data.get("slew_us_per_s", 150), "slew_us_per_s")
        if not 0 < slew <= 2000:
            raise GimbalError("slew_us_per_s must be in(0,2000]")
        confirmed = data.get("calibration_confirmed", False)
        if not isinstance(confirmed, bool):
            raise GimbalError("calibration_confirmed must be boolean")
        supplied = data.get("calibration_points", [])
        if not isinstance(supplied, list):
            raise GimbalError("calibration_points must be a list")
        points = []
        for point in supplied:
            if not isinstance(point, Mapping):
                raise GimbalError("each calibration point must be an object")
            angle = _number(point.get("angle_deg"), "angle_deg")
            pulse = _pulse(point.get("pulse_us"))
            if low is None or not low <= pulse <= high:
                raise GimbalError("every calibration pulse must be inside the safe window")
            points.append((angle, pulse))
        for before, after in zip(points, points[1:]):
            if before[0] >= after[0]:
                raise GimbalError("calibration angles must strictly increase, without wraparound")
            if not math.isfinite(after[0] - before[0]):
                raise GimbalError("calibration angle spans must be finite")
        differences = [after[1] - before[1] for before, after in zip(points, points[1:])]
        if differences and not (all(d > 0 for d in differences) or all(d < 0 for d in differences)):
            raise GimbalError("calibration pulses must be strictly monotonic")
        if confirmed and len(points) < 2:
            raise GimbalError("confirmed calibration needs at least two measured points")
        return cls(low, high, int(fine), int(coarse), slew, tuple(points), confirmed,
                   explore_low, explore_high)

    @property
    def configured(self) -> bool:
        return self.safe_min_us is not None

    @property
    def calibrated(self) -> bool:
        return self.calibration_confirmed and len(self.calibration_points) >= 2

    def clamp(self, pulse: float) -> int:
        if not self.configured:
            raise GimbalError("set a measured safe pulse window before arming")
        return min(self.safe_max_us, max(self.safe_min_us, round(pulse)))

    def manual_clamp(self, pulse: float) -> int:
        """User's exploration window, superseded by measured mechanical limits."""
        low = self.safe_min_us if self.configured else self.explore_min_us
        high = self.safe_max_us if self.configured else self.explore_max_us
        return min(high, max(low, round(pulse)))

    def angle_to_pulse(self, angle: float) -> int:
        if not self.calibrated:
            raise GimbalError("measured, confirmed angle calibration is required")
        angle = _number(angle, "angle_deg")
        points = self.calibration_points
        if not points[0][0] <= angle <= points[-1][0]:
            raise GimbalError("requested angle is outside the measured calibration interval")
        index = min(len(points) - 2, max(0, bisect.bisect_right([p[0] for p in points], angle) - 1))
        (a0, p0), (a1, p1) = points[index:index + 2]
        return self.clamp(p0 + (p1 - p0) * (angle - a0) / (a1 - a0))

    def pulse_to_angle(self, pulse: int | None) -> float | None:
        if not self.calibrated or pulse is None:
            return None
        for (a0, p0), (a1, p1) in zip(self.calibration_points, self.calibration_points[1:]):
            if min(p0, p1) <= pulse <= max(p0, p1):
                return a0 + (a1 - a0) * (pulse - p0) / (p1 - p0)
        return None


def validate_config(config: Mapping[str, Any]) -> dict[str, AxisConfig]:
    if not isinstance(config, Mapping) or not isinstance(config.get("axes"), Mapping):
        raise GimbalError("config requires an axes object containing pan and tilt")
    try:
        return {name: AxisConfig.parse(config["axes"][name]) for name in AXES}
    except KeyError as exc:
        raise GimbalError(f"missing axis:{exc.args[0]}") from exc


class MockPWMBackend:
    physical = False

    def __init__(self):
        self.events: list[tuple] = []
        self.enabled: dict[str, int] = {}

    def start(self, axis: str, pulse_us: int) -> None:
        self.enabled[axis] = _pulse(pulse_us)
        self.events.append(("start", axis, pulse_us))

    def set_pulse(self, axis: str, pulse_us: int) -> None:
        if axis not in self.enabled:
            raise GimbalError("PWM axis is disabled")
        self.enabled[axis] = _pulse(pulse_us)
        self.events.append(("pulse", axis, pulse_us))

    def disable(self, axis: str) -> None:
        if axis in self.enabled:
            self.enabled.pop(axis)
            self.events.append(("disable", axis))

    def diagnostics(self) -> dict:
        return {"type": "mock", "physical": False}


class SysfsPWMBackend:
    """Explicit, already-exported PWM paths; construction never exports/enables.

    Each path must end pwmchipN/pwmX. Export/overlay installation is a separate
    operator step. PWM disable is best effort; Linux does not promise the pin
    level of a disabled controller, nor can PWM loss guarantee servo torque-off.
    """
    physical = True

    def __init__(self, paths: Mapping[str, str | Path]):
        self.paths = {name: Path(path) for name, path in paths.items()}
        if set(self.paths) != set(AXES):
            raise GimbalError("map exactly pan and tilt PWM paths")
        for path in self.paths.values():
            if not path.is_absolute() or not re.fullmatch(r"pwm\d+", path.name) or not re.fullmatch(r"pwmchip\d+", path.parent.name):
                raise GimbalError("PWM path must be absolute and end pwmchipN/pwmX")
        if len({path.resolve() for path in self.paths.values()}) != len(AXES):
            raise GimbalError("pan and tilt must use different PWM channels")
        self._active: set[str] = set()

    @staticmethod
    def _write(path: Path, value: Any) -> None:
        try:
            path.write_text(str(value), encoding="ascii")
        except PermissionError as exc:
            raise GimbalError(f"没有PWM写入权限：{path} ← {value}；请检查香橙派运行用户和该节点权限后重试") from exc
        except OSError as exc:
            raise GimbalError(f"PWM写入失败：{path} ← {value}；{exc}") from exc

    @staticmethod
    def _read(path: Path) -> str:
        try:
            return path.read_text(encoding="ascii").strip()
        except (OSError, UnicodeError) as exc:
            raise GimbalError(f"PWM读取失败：{path}；{exc}") from exc

    @classmethod
    def _read_ns(cls, path: Path) -> int:
        text = cls._read(path)
        if not text.isascii() or not text.isdecimal():
            raise GimbalError(f"PWM状态无效：{path} = {text!r}；应为非负纳秒整数")
        return int(text)

    def start(self, axis: str, pulse_us: int) -> None:
        pulse_us = _pulse(pulse_us)
        path = self.paths[axis]
        if not all((path / name).is_file() for name in ("period", "duty_cycle", "enable", "polarity")):
            raise GimbalError(f"PWM通道未导出或无法访问：{path}；请先启用对应PWM并确认目录内有period、duty_cycle、enable、polarity")
        if self._read(path / "enable") != "0":
            raise GimbalError(f"PWM通道已被其他程序启用：{path}；请先关闭占用程序后重试")
        old_period = self._read_ns(path / "period")
        old_duty = self._read_ns(path / "duty_cycle")
        polarity = self._read(path / "polarity")
        if old_duty > old_period or polarity not in ("normal", "inversed"):
            raise GimbalError(f"PWM状态无效：{path}；period={old_period}, duty_cycle={old_duty}, polarity={polarity!r}")
        self._active.add(axis)
        valid_period = old_period > 0
        try:
            # Linux validates the entire PWM state on EVERY sysfs write, even
            # enable=0. A newly exported channel has period=duty=enable=0;
            # writing enable/duty first therefore fails with EINVAL. We already
            # know this channel is disabled: establish a positive period first.
            # Only clear an old large duty before shrinking an existing period.
            if old_duty > PERIOD_NS:
                self._write(path / "duty_cycle", 0)
            self._write(path / "period", PERIOD_NS)
            valid_period = True
            if polarity != "normal":
                self._write(path / "polarity", "normal")
            self._write(path / "duty_cycle", pulse_us * 1000)
            # This is the only operation that starts physical output.
            self._write(path / "enable", 1)
        except Exception as exc:
            if valid_period:
                try:
                    self.disable(axis)
                except Exception as cleanup:
                    # Keep ownership for a later shutdown retry, and preserve
                    # the actual failing setup operation in the error message.
                    raise GimbalError(f"{exc}；关闭PWM也失败：{cleanup}") from exc
            else:
                # No enable was attempted and period may still be zero. Do not
                # mask the original period error with an invalid enable=0.
                self._active.discard(axis)
            raise

    def set_pulse(self, axis: str, pulse_us: int) -> None:
        if axis not in self._active:
            raise GimbalError("PWM axis is disabled")
        self._write(self.paths[axis] / "duty_cycle", _pulse(pulse_us) * 1000)

    def disable(self, axis: str) -> None:
        if axis in self._active:
            self._write(self.paths[axis] / "enable", 0)
            self._active.discard(axis)

    def diagnostics(self) -> dict:
        channels = {}
        for axis, path in self.paths.items():
            chip = path.parent
            node = chip / "device" / "of_node"
            channels[axis] = {"path": str(path), "exported": path.is_dir(),
                              "device": str((chip / "device").resolve()),
                              "of_node": str(node.resolve()), "of_name": None}
            try:
                channels[axis]["of_name"] = (node / "name").read_bytes().rstrip(b"\0").decode("ascii", "replace")
            except OSError:
                pass
        return {"type": "sysfs", "physical": True, "channels": channels}


@dataclass
class _AxisState:
    armed: bool = False
    manual_calibration: bool = False
    output_us: int | None = None
    target_us: int | None = None
    position_us: float = 0.0
    last_tick: float = 0.0
    draft_us: int = 1500
    pending_min_us: int | None = None
    pending_max_us: int | None = None


class GimbalController:
    """Thread-safe command/slew state; tick() must be called by the host loop."""

    def __init__(self, config: Mapping[str, Any], backend=None,
                 clock: Callable[[], float] = time.monotonic, operating_angle_min=None):
        self.axes = validate_config(config)
        self.operating_angle_min=dict(operating_angle_min or {})
        self.config = deepcopy(dict(config))
        self.backend = backend if backend is not None else MockPWMBackend()
        self._clock = clock
        self._lock = threading.RLock()
        self._state = {name: _AxisState() for name in AXES}
        for name, state in self._state.items():
            stored_start = self.config["axes"][name].get("start_us")
            cfg = self.axes[name]
            state.draft_us = cfg.manual_clamp(1500)
            if (not isinstance(stored_start, bool) and isinstance(stored_start, (int, float)) and
                    math.isfinite(stored_start) and stored_start == int(stored_start) and
                    cfg.configured and cfg.safe_min_us <= stored_start <= cfg.safe_max_us):
                state.draft_us = int(stored_start)
        self._closed = False
        self._fault: str | None = None
        atexit.register(self.close)

    def _axis(self, axis: str) -> tuple[AxisConfig, _AxisState]:
        if self._closed:
            raise GimbalError("云台控制器已关闭，请重启服务后重试")
        if axis not in self.axes:
            raise GimbalError("请选择pan底座轴或tilt上层轴")
        return self.axes[axis], self._state[axis]

    def angle_limits(self, axis, config=None):
        cfg=self.axes[axis] if config is None else config
        if not cfg.calibrated:
            raise GimbalError('角度范围需要已经确认的实测角度表')
        low,high=cfg.calibration_points[0][0],cfg.calibration_points[-1][0]
        low=max(low,self.operating_angle_min.get(axis,low))
        if low>high:
            raise GimbalError('实测角度表不覆盖要求的运行范围')
        return low,high

    def pulse_limits(self, axis, config=None):
        cfg=self.axes[axis] if config is None else config
        if axis not in self.operating_angle_min:
            return cfg.safe_min_us,cfg.safe_max_us
        if not cfg.calibrated:
            return None,None  # No guessed angle-to-PWM relation may enable PWM.
        low_angle,high_angle=self.angle_limits(axis,cfg)
        edge=cfg.angle_to_pulse(low_angle)
        increasing=cfg.calibration_points[-1][1]>cfg.calibration_points[0][1]
        # Round INWARD. Nearest-integer rounding must not command89.99deg.
        while cfg.pulse_to_angle(edge)<low_angle:
            edge+=1 if increasing else -1
        end=cfg.angle_to_pulse(high_angle)
        return max(cfg.safe_min_us,min(edge,end)),min(cfg.safe_max_us,max(edge,end))

    def _clamp_operating(self, axis, pulse, *, manual=False, config=None):
        cfg=self.axes[axis] if config is None else config
        value=cfg.manual_clamp(pulse) if manual else cfg.clamp(pulse)
        if axis not in self.operating_angle_min:
            return value
        low,high=self.pulse_limits(axis,cfg)
        if low is None:
            raise GimbalError('TILT必须有确认的角度标定，才能保证不小于90度')
        return max(low,min(high,value))

    def arm(self, axis: str, start_us: int, manual_calibration: bool = False,
            physical: bool = False) -> dict:
        return self._enable(axis, start_us, manual_calibration, physical, exploration=False)

    def _enable(self, axis: str, start_us: int, manual_calibration: bool,
                physical: bool, *, exploration: bool) -> dict:
        with self._lock:
            config, state = self._axis(axis)
            start = _pulse(start_us, "start_us")
            if self._clamp_operating(axis,start,manual=exploration)!=start:
                raise GimbalError('起始位置超出运行角度范围；TILT不得小于90度')
            if not exploration and (not config.configured or config.clamp(start) != start):
                raise GimbalError("旧版arm接口要求起始脉宽位于已保存限位内；手动探索请使用开启本轴PWM按钮")
            if state.armed:
                raise GimbalError("该轴PWM已开启，请使用粗调、细调或设置脉宽")
            if not config.calibrated and manual_calibration is not True:
                raise GimbalError("该轴未完成角度标定，请使用手动开启PWM功能")
            if self.backend.physical and physical is not True:
                raise GimbalError("真实PWM需要用户点击开启本轴PWM按钮")
            try:
                self.backend.start(axis, start)
            except Exception as exc:
                self._fault = str(exc)
                self.disable()
                raise GimbalError(f"PWM启动失败：{exc}；修复后可再次点击开启本轴PWM") from exc
            state.armed = True
            state.manual_calibration = manual_calibration is True
            state.output_us = state.target_us = start
            state.draft_us = start
            state.position_us = float(start)
            state.last_tick = self._clock()
            self._fault = None
            return self.snapshot()

    def manual_enable(self, axis: str, pulse_us: int | None = None) -> dict:
        """The explicit enable command itself authorizes this axis's PWM start."""
        with self._lock:
            config, state = self._axis(axis)
            requested = state.draft_us if pulse_us is None else _pulse(pulse_us)
            start = self._clamp_operating(axis,requested,manual=True)
            state.draft_us = start
            if state.armed:
                state.manual_calibration = True
                state.target_us = start
                return self.snapshot()
            return self._enable(axis, start, True, True, exploration=True)

    def manual_pulse(self, axis: str, pulse_us: int) -> dict:
        with self._lock:
            config, state = self._axis(axis)
            state.draft_us = self._clamp_operating(axis,_pulse(pulse_us),manual=True)
            if state.armed:
                state.manual_calibration = True
                state.target_us = state.draft_us
            return self.snapshot()

    def manual_jog(self, axis: str, direction: int, coarse: bool = False) -> dict:
        with self._lock:
            config, state = self._axis(axis)
            if isinstance(direction, bool) or direction not in (-1, 1):
                raise GimbalError("调节方向必须是-1或+1")
            step = config.coarse_us if coarse else config.fine_us
            state.draft_us = self._clamp_operating(axis,state.draft_us + direction * step,manual=True)
            if state.armed:
                state.manual_calibration = True
                state.target_us = state.draft_us
            return self.snapshot()

    def mark_limit(self, axis: str, limit: str) -> dict:
        """Record an emitted, settled PWM command; not servo position feedback."""
        with self._lock:
            _, state = self._axis(axis)
            if limit not in ("min", "max"):
                raise GimbalError("限位端点必须是min下限或max上限")
            if not state.armed or state.output_us is None:
                raise GimbalError("请先开启本轴PWM，移动到要记录的位置，再记录限位")
            if state.output_us != state.target_us:
                raise GimbalError("脉宽仍在渐变，请等请求值与输出值相同后再记录限位")
            if limit == "min":
                state.pending_min_us = state.output_us
            else:
                state.pending_max_us = state.output_us
            return self.snapshot()

    def apply_axis_config(self, axis: str, raw: Mapping[str, Any], *, clear_pending=True) -> dict:
        """Apply an already persisted axis configuration without auto-enabling."""
        with self._lock:
            _, state = self._axis(axis)
            config = AxisConfig.parse(raw)
            low = config.safe_min_us if config.configured else config.explore_min_us
            high = config.safe_max_us if config.configured else config.explore_max_us
            if state.armed and not (low <= state.output_us <= high and low <= state.target_us <= high):
                raise GimbalError("当前输出或请求脉宽不在新范围内，请先移入新范围或关闭本轴PWM")
            if state.armed and axis in self.operating_angle_min:
                op_low,op_high=self.pulse_limits(axis,config)
                if op_low is None or not (op_low<=state.output_us<=op_high and op_low<=state.target_us<=op_high):
                    raise GimbalError('当前TILT超出新标定的90度运行下限，请先关闭PWM')
            self.axes[axis] = config
            self.config["axes"][axis] = deepcopy(dict(raw))
            state.draft_us = config.manual_clamp(state.draft_us)
            if clear_pending:
                state.pending_min_us = state.pending_max_us = None
            return self.snapshot()

    def set_pulse(self, axis: str, pulse_us: int, manual_calibration: bool = False) -> dict:
        with self._lock:
            config, state = self._axis(axis)
            if not state.armed or not state.manual_calibration or manual_calibration is not True:
                raise GimbalError("pulse calibration needs an armed manual axis and explicit permission")
            state.target_us = self._clamp_operating(axis,_pulse(pulse_us))
            state.draft_us = state.target_us
            return self.snapshot()

    def jog(self, axis: str, direction: int, coarse: bool = False) -> dict:
        with self._lock:
            config, state = self._axis(axis)
            if isinstance(direction, bool) or direction not in (-1, 1):
                raise GimbalError("direction must be-1 or+1")
            if not state.armed or not state.manual_calibration:
                raise GimbalError("jog needs an explicitly armed manual calibration axis")
            step = config.coarse_us if coarse else config.fine_us
            state.target_us = self._clamp_operating(axis,state.target_us + direction * step)
            state.draft_us = state.target_us
            return self.snapshot()

    def set_angle(self, axis: str, angle_deg: float, tracking: bool = False) -> dict:
        with self._lock:
            config, state = self._axis(axis)
            if not state.armed:
                raise GimbalError("axis is disabled")
            if tracking and not all(self.axes[name].calibrated and self._state[name].armed for name in AXES):
                raise GimbalError("tracking requires both axes armed with confirmed calibration")
            if axis in self.operating_angle_min:
                low,_=self.angle_limits(axis)
                angle_deg=max(low,_number(angle_deg,'angle_deg'))
            state.target_us = self._clamp_operating(axis,config.angle_to_pulse(angle_deg))
            state.draft_us = state.target_us
            return self.snapshot()

    def tick(self, now: float | None = None) -> dict:
        with self._lock:
            if self._closed:
                return self.snapshot()
            current = _number(self._clock() if now is None else now, "time")
            for axis, state in self._state.items():
                if not state.armed:
                    continue
                # Limit catch-up after a stalled host loop. Never use repeated
                # calls at an unchanged timestamp to accumulate movement.
                dt = min(0.1, max(0.0, current - state.last_tick))
                state.last_tick = max(current, state.last_tick)
                step = self.axes[axis].slew_us_per_s * dt
                error = state.target_us - state.position_us
                state.position_us += max(-step, min(step, error))
                pulse = self._clamp_operating(axis,state.position_us,manual=True)
                if pulse != state.output_us:
                    try:
                        self.backend.set_pulse(axis, pulse)
                    except Exception as exc:
                        self._fault = str(exc)
                        self.disable()
                        raise GimbalError(f"PWM更新失败：{exc}；已关闭输出，修复后请重新点击开启本轴PWM") from exc
                    state.output_us = pulse
            return self.snapshot()

    def hold(self, axis: str | None = None) -> dict:
        with self._lock:
            names = AXES if axis is None else (axis,)
            for name in names:
                _, state = self._axis(name)
                state.target_us = state.output_us
                if state.output_us is not None:
                    state.position_us = float(state.output_us)
                    if state.armed:
                        state.draft_us = state.output_us
                state.last_tick = self._clock()
            return self.snapshot()

    def disable(self, axis: str | None = None) -> dict:
        with self._lock:
            names = AXES if axis is None else (axis,)
            if any(name not in self._state for name in names):
                raise GimbalError("axis must be pan or tilt")
            errors = []
            for name in names:
                try:
                    self.backend.disable(name)
                except Exception as exc:
                    errors.append(f"{name}:{exc}")
                state = self._state[name]
                state.armed = state.manual_calibration = False
                state.target_us = state.output_us  # Retain last command for inspection.
            if errors:
                self._fault = ";".join(errors)
            return self.snapshot()

    def snapshot(self) -> dict:
        with self._lock:
            result = {"backend": self.backend.diagnostics(), "closed": self._closed,
                      "fault": self._fault, "tracking_allowed": all(
                          self.axes[name].calibrated and self._state[name].armed for name in AXES),
                      "axes": {}}
            for name, state in self._state.items():
                config = self.axes[name]
                low,high=self.pulse_limits(name)
                result["axes"][name] = dict(armed=state.armed,
                    manual_calibration=state.manual_calibration, output_us=state.output_us,
                    target_us=state.target_us, angle_deg=config.pulse_to_angle(state.output_us),
                    configured=config.configured, calibrated=config.calibrated,
                    safe_min_us=low, safe_max_us=high,
                    operating_min_angle_deg=self.operating_angle_min.get(name))
                result["axes"][name]["manual"] = dict(
                    draft_us=state.draft_us, enabled=state.armed,
                    recorded_min_us=config.safe_min_us, recorded_max_us=config.safe_max_us,
                    pending_min_us=state.pending_min_us, pending_max_us=state.pending_max_us,
                    settled=state.armed and state.output_us == state.target_us,
                    fine_us=config.fine_us, coarse_us=config.coarse_us,
                    numeric_min_us=low if name in self.operating_angle_min and low is not None else config.explore_min_us,
                    numeric_max_us=high if name in self.operating_angle_min and high is not None else config.explore_max_us,
                    absolute_min_us=MIN_PULSE_US, absolute_max_us=MAX_PULSE_US)
            return result

    def close(self) -> None:
        with self._lock:
            if not self._closed:
                self.disable()
                self._closed = True
                atexit.unregister(self.close)

    def __enter__(self) -> "GimbalController":
        return self

    def __exit__(self, *args) -> None:
        self.close()
