"""Small, local image-motion calibration for a stationary camera gimbal.

No hardware, image library, or numerical package is used here. Samples describe
the SAME stationary target before/after a small manual move. This is a local
empirical image Jacobian, not a pose estimator or proof that the target stayed
still. Pixel x points right and pixel y points down. Joint angles retain the
user's measured PWM-to-angle coordinates.
"""
from __future__ import annotations

import math
from collections.abc import Mapping

MAX_SAMPLES = 64
MIN_SAMPLES = 4
MAX_JOINT_STEP_DEG = 30.0
MIN_FEATURE_DEG = .25
MIN_IMAGE_MOVE = .008
MAX_DESIGN_CONDITION = 20.0
MAX_MODEL_CONDITION = 30.0
MAX_RMS = .06


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} 必须是有限数值')
    return float(value)


def _size(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f'{name} 必须是正整数')
    return value


def _features(sample):
    start, end = sample['start'], sample['end']
    mean_tilt = (start['tilt_deg'] + end['tilt_deg']) / 2
    return (math.cos(math.radians(mean_tilt)) * (end['pan_deg'] - start['pan_deg']),
            end['tilt_deg'] - start['tilt_deg'])


def normalize_sample(sample):
    """Validate/copy one JSON sample; reject ineffective or nonlocal moves.

    Width/height refer to the original image, and each endpoint requires u/v,
    pan_deg/tilt_deg and pan_us/tilt_us. No endpoint is silently clamped. Both
    endpoints must be visible; crossing 90deg in a SMALL move is permitted.
    """
    if not isinstance(sample, Mapping):
        raise ValueError('采样必须是对象')
    width, height = _size(sample.get('width'), 'width'), _size(sample.get('height'), 'height')
    kind = sample.get('kind')
    if kind not in ('human', 'dog'):
        raise ValueError('请选择同一个人脸或玩偶目标采样')
    result = dict(width=width, height=height, kind=kind)
    for endpoint in ('start', 'end'):
        raw = sample.get(endpoint)
        if not isinstance(raw, Mapping):
            raise ValueError(f'缺少{endpoint}端点')
        point = {name: _number(raw.get(name), f'{endpoint}.{name}')
                 for name in ('u', 'v', 'pan_deg', 'tilt_deg', 'pan_us', 'tilt_us')}
        if not 0 <= point['u'] <= width or not 0 <= point['v'] <= height:
            raise ValueError('采样起点和终点的目标必须都在原始画面内')
        if any(not 1 <= point[name] <= 19999 for name in ('pan_us', 'tilt_us')):
            raise ValueError('采样脉宽必须位于1～19999µs内')
        if any(abs(point[name]) > 100000 for name in ('pan_deg', 'tilt_deg')):
            raise ValueError('采样关节角数值过大')
        result[endpoint] = point
    start, end = result['start'], result['end']
    if any(abs(end[name] - start[name]) > MAX_JOINT_STEP_DEG
           for name in ('pan_deg', 'tilt_deg')):
        raise ValueError('一次采样每轴请小幅移动，不超过30°；大范围移动不能当作局部模型')
    feature = _features(result)
    if math.hypot(*feature) < MIN_FEATURE_DEG:
        raise ValueError('有效关节位移太小；天顶附近单独转底座也不足以标定左右方向')
    image_move = math.hypot((end['u'] - start['u']) / (width / 2),
                            (end['v'] - start['v']) / (height / 2))
    if image_move < MIN_IMAGE_MOVE:
        raise ValueError('目标画面位移太小，请增加少量手动位移后重新采样')
    return result


def _eigenvalues(a, b, d):
    # Stable 2x2 positive-semidefinite eigenvalues; determinant avoids loss
    # of the smaller eigenvalue when axes have very different excitation.
    high = max(0.0, (a + d + math.hypot(a - d, 2 * b)) / 2)
    low = max(0.0, (a * d - b * b) / high) if high else 0.0
    return high, low


def _condition(a, b, d):
    high, low = _eigenvalues(a, b, d)
    rank = 0 if high <= 1e-12 else (1 if low <= high * 1e-10 else 2)
    return rank, math.sqrt(high / low) if rank == 2 else None


def _matrix_condition(matrix):
    a, b = matrix[0]
    c, d = matrix[1]
    return _condition(a * a + c * c, a * b + c * d, b * b + d * d)


def fit_image_calibration(samples):
    """Fit B in image_delta = B @ (cos(mean_tilt)*delta_pan, delta_tilt).

    At least four useful observations and two independently excited directions
    are required. Four permits a residual check; two would interpolate two
    arbitrary/possibly moving-target observations perfectly. Recent64 samples
    are considered. Invalid individual samples are counted and omitted; mixed
    image sizes invalidate the fit rather than silently mixing pixel scales.
    Returned models/diagnostics contain no NaN/Infinity and are JSON serializable.
    """
    if not isinstance(samples, (list, tuple)):
        raise ValueError('samples 必须是采样列表')
    supplied = list(samples)[-MAX_SAMPLES:]
    valid, rejected = [], []
    for index, raw in enumerate(supplied):
        try:
            valid.append(normalize_sample(raw))
        except ValueError as exc:
            rejected.append(dict(index=index, reason=str(exc)))
    sizes = {(s['width'], s['height']) for s in valid}
    width, height = next(iter(sizes)) if len(sizes) == 1 else (None, None)
    reasons = []
    if len(sizes) > 1:
        reasons.append('画面尺寸发生变化，请固定分辨率后重新采样')
    if len(valid) < MIN_SAMPLES:
        reasons.append('至少需要4次有效采样，建议底座和上层分别正反向移动')
    result = dict(schema_version=1, valid=False, width=width, height=height,
                  matrix_b=None, diagnostics=dict(
                      sample_count=len(supplied), effective_samples=len(valid),
                      rejected_samples=rejected, truncated_samples=max(0, len(samples) - MAX_SAMPLES),
                      rank=0, condition=None, rms=None, relative_rms=None,
                      jacobian_condition=None, reasons=reasons, warnings=[]))
    if not valid or len(sizes) != 1:
        return result
    rows = []
    for sample in valid:
        x, y = _features(sample)
        du = (sample['end']['u'] - sample['start']['u']) / (width / 2)
        dv = (sample['end']['v'] - sample['start']['v']) / (height / 2)
        rows.append((x, y, du, dv))
    xx = sum(x * x for x, y, u, v in rows)
    xy = sum(x * y for x, y, u, v in rows)
    yy = sum(y * y for x, y, u, v in rows)
    rank, condition = _condition(xx, xy, yy)
    diagnostics = result['diagnostics']
    diagnostics.update(rank=rank, condition=condition)
    coverage = {axis: dict(positive=sum(row[i] >= MIN_FEATURE_DEG for row in rows),
                          negative=sum(row[i] <= -MIN_FEATURE_DEG for row in rows))
                for i, axis in enumerate(('pan', 'tilt'))}
    diagnostics['direction_coverage'] = coverage
    if any(not counts['positive'] or not counts['negative'] for counts in coverage.values()):
        diagnostics['warnings'].append('建议补充两轴的正反向采样，以检查回差及目标移动影响')
    if rank != 2:
        reasons.append('两轴未独立激励；不要每次按同一比例同时移动两轴，请分别单独调底座和上层')
        return result
    if condition > MAX_DESIGN_CONDITION:
        reasons.append('两轴采样方向过于接近或位移悬殊，请补充位移较少的轴')
        return result
    determinant = xx * yy - xy * xy
    matrix = []
    for index in (2, 3):
        tx = sum(row[0] * row[index] for row in rows)
        ty = sum(row[1] * row[index] for row in rows)
        matrix.append([(tx * yy - ty * xy) / determinant,
                       (ty * xx - tx * xy) / determinant])
    squared_error = sum((u - matrix[0][0] * x - matrix[0][1] * y) ** 2 +
                        (v - matrix[1][0] * x - matrix[1][1] * y) ** 2
                        for x, y, u, v in rows)
    rms = math.sqrt(squared_error / (2 * len(rows)))
    signal_rms = math.sqrt(sum(u * u + v * v for x, y, u, v in rows) / (2 * len(rows)))
    relative_rms = rms / signal_rms if signal_rms else None
    matrix_rank, matrix_condition = _matrix_condition(matrix)
    diagnostics.update(rms=rms, relative_rms=relative_rms, jacobian_condition=matrix_condition)
    result['matrix_b'] = matrix
    if matrix_rank != 2 or matrix_condition > MAX_MODEL_CONDITION:
        reasons.append('画面运动无法区分两轴；请固定目标，分别小幅移动两轴后重新采样')
    if min(math.hypot(matrix[0][i], matrix[1][i]) for i in (0, 1)) < 1e-4:
        reasons.append('至少一轴画面响应太小，不能可靠学习方向')
    if rms > MAX_RMS or relative_rms is None or relative_rms > .35:
        reasons.append('采样不一致；请保持人或玩偶及车身静止，使用较小位移重新采样')
    result['valid'] = not reasons
    return result


def _validated_model(model, width, height):
    """Shared stored-model quality/shape checks for rate and position solves."""
    if not isinstance(model, Mapping) or model.get('valid') is not True or model.get('schema_version') != 1:
        return None
    if model.get('width') != width or model.get('height') != height:
        return None
    try:
        _size(width, 'width')
        _size(height, 'height')
        raw = model.get('matrix_b')
        if not isinstance(raw, (list, tuple)) or len(raw) != 2 or any(len(row) != 2 for row in raw):
            return None
        matrix = [[_number(value, 'matrix_b') for value in row] for row in raw]
        rank, condition = _matrix_condition(matrix)
        responses = [math.hypot(matrix[0][i], matrix[1][i]) for i in (0, 1)]
        sensitivity = max(responses)
        if rank != 2 or condition > MAX_MODEL_CONDITION or min(responses) < 1e-4:
            return None
        diagnostics = model.get('diagnostics', {})
        if not isinstance(diagnostics, Mapping) or diagnostics.get('rank') != 2 or diagnostics.get('effective_samples', 0) < MIN_SAMPLES:
            return None
        rms = _number(diagnostics.get('rms'), 'rms')
        relative = _number(diagnostics.get('relative_rms'), 'relative_rms')
        design_condition = _number(diagnostics.get('condition'), 'condition')
        if (not 0 <= rms <= MAX_RMS or not 0 <= relative <= .35 or
                not 1 <= design_condition <= MAX_DESIGN_CONDITION):
            return None
    except (ValueError, TypeError, OverflowError):
        return None
    return matrix, sensitivity


def _damped_image_solve(matrix, sensitivity, tilt, vx, vy):
    """Solve J.T (J J.T + damping**2 I)^-1 v without temporal scaling."""
    cosine = math.cos(math.radians(tilt))
    if abs(cosine) < 1e-12:
        cosine = 0.0
    a, b = matrix[0][0] * cosine, matrix[0][1]
    c, d = matrix[1][0] * cosine, matrix[1][1]
    damping = max(1e-6, .1 * sensitivity)
    aa, ab, dd = a * a + b * b + damping * damping, a * c + b * d, c * c + d * d + damping * damping
    determinant = aa * dd - ab * ab
    wx, wy = (dd * vx - ab * vy) / determinant, (aa * vy - ab * vx) / determinant
    return a * wx + c * wy, b * wx + d * wy, cosine, damping, [[a, b], [c, d]]


def calibrated_centering_delta(model, *, tilt_deg, desired_x_rate,
                               desired_y_rate, dt, width, height,
                               max_rate_dps=25.0):
    """Convert desired image velocity to rate-limited joint increments.

    Image velocities are normalized HALF-image widths/heights per second,
    positive right/down. To move a detection toward the center use
    desired_x_rate = gain*(center_u-current_u)/(width/2), likewise y. This is
    the SAME sign as the tracker's nx/ny, not its opposite. Never feed deg/s
    here as if it were normalized image velocity.

    Return None for unavailable/incompatible/invalid stored models so the
    application can use its explicit mounting geometry. Damped least squares
    avoids PAN rate explosion at90deg; a true zenith has zero PAN authority.
    """
    validated = _validated_model(model, width, height)
    if validated is None:
        return None
    tilt = _number(tilt_deg, 'tilt_deg')
    vx, vy = _number(desired_x_rate, 'desired_x_rate'), _number(desired_y_rate, 'desired_y_rate')
    duration, limit = _number(dt, 'dt'), _number(max_rate_dps, 'max_rate_dps')
    if duration < 0 or limit < 0:
        raise ValueError('dt 和最高转速不能为负')
    # Caller normally supplies capture dt<=.1s. Bound a delayed caller too;
    # no single observation may request a long open-loop integration.
    duration = min(.2, duration)
    raw_pan, raw_tilt, cosine, damping, jacobian = _damped_image_solve(*validated, tilt, vx, vy)
    pan = max(-limit, min(limit, raw_pan))
    tilt_rate = max(-limit, min(limit, raw_tilt))
    return dict(source='sampled_image_jacobian', tilt_deg=tilt,
                near_vertical=abs(cosine) < .05,
                pan_rate_dps=pan, tilt_rate_dps=tilt_rate,
                delta_pan_deg=pan * duration, delta_tilt_deg=tilt_rate * duration,
                control_dt_s=duration, damping=damping,
                desired_image_velocity=[vx, vy], matrix_j=jacobian)


def calibrated_centering_offset(model, *, tilt_deg, desired_x, desired_y,
                               width, height, max_step_deg=30):
    """Estimate one finite joint offset from a whole image displacement.

    desired_x/y are normalized HALF-image displacements, positive right/down;
    they have the SAME sign as the tracker's nx/ny. This is a static local
    Jacobian inverse, with no assumed frame duration or integration gain.
    The caller may slew toward this one goal after the last observed frame.
    It must not repeatedly accumulate the offset without a new observation.

    Shared quality checks reject an incompatible stored model with None.
    The same damped solve as the rate controller bounds the zenith singularity.
    Each offset is clamped to max_step_deg and to the model's30degree local
    calibration span, even if a caller requests a larger limit. Mechanical
    limits and PWM slew still belong to the application/controller.
    """
    validated = _validated_model(model, width, height)
    if validated is None:
        return None
    tilt = _number(tilt_deg, 'tilt_deg')
    dx, dy = _number(desired_x, 'desired_x'), _number(desired_y, 'desired_y')
    requested_limit = _number(max_step_deg, 'max_step_deg')
    if requested_limit < 0:
        raise ValueError('单次最大角位移不能为负')
    limit = min(MAX_JOINT_STEP_DEG, requested_limit)
    raw_pan, raw_tilt, cosine, damping, jacobian = _damped_image_solve(*validated, tilt, dx, dy)
    pan = max(-limit, min(limit, raw_pan))
    tilt_offset = max(-limit, min(limit, raw_tilt))
    limited_axes = [axis for axis, raw, limited in
                    (('pan', raw_pan, pan), ('tilt', raw_tilt, tilt_offset)) if raw != limited]
    return dict(source='sampled_image_jacobian', tilt_deg=tilt,
                delta_pan_deg=pan, delta_tilt_deg=tilt_offset,
                limited=bool(limited_axes), limited_axes=limited_axes,
                max_step_deg=limit, near_vertical=abs(cosine) < .05,
                damping=damping, desired_image_displacement=[dx, dy], matrix_j=jacobian)
