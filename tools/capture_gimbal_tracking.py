"""Read-only local /state capture; no control, heartbeat, camera or PWM backend.

Example on the Pi:
  python3 tools/capture_gimbal_tracking.py --output /tmp/gimbal-tracking.jsonl

Only HTTP GET /state is used. Configuration values come from the running
service's JSON, never from a calibration file or a path reported by the API.
The output contains metadata, not images, tokens or private file contents.
"""
import argparse
from datetime import datetime
import hashlib
import io
import ipaddress
import json
from pathlib import Path
import time
import urllib.request
from urllib.parse import urlsplit


def choose(value, names):
    return {key: value[key] for key in names if isinstance(value, dict) and key in value}


def diagnostic_state(state):
    if not isinstance(state, dict):
        raise ValueError('/state must return a JSON object')
    config = state.get('config') or {}
    axes = {}
    for axis in ('pan', 'tilt'):
        raw = (config.get('axes') or {}).get(axis) or {}
        axes[axis] = choose(raw, ('safe_min_us', 'safe_max_us', 'explore_min_us', 'explore_max_us',
                                 'fine_us', 'coarse_us', 'slew_us_per_s', 'calibration_confirmed'))
        axes[axis]['calibration_points'] = [choose(point, ('angle_deg', 'pulse_us'))
                                             for point in raw.get('calibration_points', [])]
    reported_config = dict(axes=axes, tracking=config.get('tracking'),
                           mount=choose(config.get('mount'), ('upright_tilt_deg',)))
    # This is an API-value fingerprint, explicitly NOT the on-disk file SHA.
    fingerprint = hashlib.sha256(json.dumps(reported_config, sort_keys=True,
                                            allow_nan=False).encode('utf-8')).hexdigest()
    gimbal = state.get('gimbal') or {}
    follow = state.get('fpga_follow') or {}
    return dict(
        software=state.get('software'), config=reported_config,
        configuration=choose(state.get('configuration'), ('path', 'sha256', 'mtime_ns')),
        reported_config_sha256=fingerprint,
        config_file_read=False,
        gimbal=dict(axes=gimbal.get('axes'), fault=gimbal.get('fault'),
                    closed=gimbal.get('closed'), tracking_allowed=gimbal.get('tracking_allowed'),
                    backend=choose(gimbal.get('backend'), ('type', 'physical'))),
        frame=choose(state.get('frame'), ('sequence', 'age_s', 'fresh', 'width', 'height',
                                         'inference_ms', 'coordinates')),
        tracking=state.get('tracking'), target_selected=state.get('target_selected'),
        tracking_preflight=state.get('tracking_preflight'), tracking_settings=state.get('tracking_settings'),
        tracking_result=state.get('tracking_result'), tracking_limits=state.get('tracking_limits'),
        tracking_servo=state.get('tracking_servo'),
        tracking_memory=state.get('tracking_memory'), tracking_pose=state.get('tracking_pose'),
        error=state.get('error'), control_error=state.get('control_error'),
        browser_recovered=state.get('browser_recovered'), detection=state.get('detection'),
        fpga_follow=choose(follow, ('software_version', 'target_kind', 'standalone', 'selected_id',
                                   'startup_pose', 'reference', 'forward_sign', 'fpga', 'error', 'report_age_s')))


def validate_url(url):
    parsed = urlsplit(url)
    try:
        local = parsed.hostname == 'localhost' or ipaddress.ip_address(parsed.hostname or '').is_loopback
        port = parsed.port
    except ValueError:
        local, port = False, None
    if (parsed.scheme != 'http' or not local or parsed.path != '/state' or
            parsed.query or parsed.fragment or parsed.username or parsed.password or port == 0):
        raise ValueError('--url must be a local HTTP /state URL, e.g. http://127.0.0.1:8080/state')
    return url


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


def state_fetcher(url):
    validate_url(url)
    # Ignore environment proxies and refuse redirects: diagnostics stay local.
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())

    def fetch():
        request = urllib.request.Request(url, headers={'Accept': 'application/json'}, method='GET')
        with opener.open(request, timeout=1) as response:
            raw = response.read(1_048_577)
        if len(raw) > 1_048_576:
            raise ValueError('/state response exceeds1MiB metadata limit')
        return json.loads(raw)
    return fetch


def capture(stream, *, seconds, hz, fetch, clock=time.monotonic, sleep=time.sleep):
    start = clock()
    period = 1 / hz
    due = start
    samples = successes = 0
    while True:
        now = clock()
        if now - start >= seconds - 1e-9:
            break
        if now < due:
            sleep(min(due - now, seconds - (now - start)))
            continue
        began = clock()
        record = dict(type='sample', sequence=samples,
                      captured_at=datetime.now().astimezone().isoformat(),
                      elapsed_s=round(began - start, 6))
        try:
            record['state'] = diagnostic_state(fetch())
            successes += 1
        except Exception as exc:
            record['error'] = str(exc)
        ended = clock()
        record['request_ms'] = round(1000 * max(0, ended - began), 3)
        stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n')
        stream.flush()
        samples += 1
        due += period
        if due <= ended:
            # Skip missed slots; never issue a catch-up burst after a slow GET.
            due = start + (int((ended - start) / period) + 1) * period
    return dict(samples=samples, successes=successes, errors=samples - successes)


def self_test():
    from unittest import mock
    calls = []
    state = dict(software=dict(revision=4), config=dict(axes=dict(
        tilt=dict(safe_min_us=645, safe_max_us=2420, slew_us_per_s=150,
                  calibration_confirmed=True, calibration_points=[dict(angle_deg=135, pulse_us=1076)])),
        mount=dict(upright_tilt_deg=180)),
        gimbal=dict(axes=dict(tilt=dict(output_us=1076, target_us=1100, safe_max_us=1559))),
        frame=dict(sequence=9, age_s=.04, inference_ms=33, detections=['not collected']),
        tracking=True, tracking_result=dict(status='tracking'),
        fpga_follow=dict(fpga=dict(pan_us=1200, tilt_us=1076, ack=8)),
        token='never collect', image='never collect')
    class Response:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def read(self, limit): return json.dumps(state).encode()
    class Opener:
        def open(self, request, timeout):
            calls.append(request)
            assert request.get_method() == 'GET' and request.data is None
            assert request.full_url == 'http://127.0.0.1:8080/state'
            return Response()
    now = [10.]
    def sleep(seconds): now[0] += seconds
    with mock.patch('urllib.request.build_opener', return_value=Opener()):
        fetch = state_fetcher('http://127.0.0.1:8080/state')
        stream = io.StringIO()
        result = capture(stream, seconds=5, hz=10, fetch=fetch, clock=lambda: now[0], sleep=sleep)
    rows = [json.loads(line) for line in stream.getvalue().splitlines()]
    assert result == dict(samples=50, successes=50, errors=0) and len(calls) == 50
    assert rows[0]['state']['gimbal']['axes']['tilt']['target_us'] == 1100
    assert rows[0]['state']['fpga_follow']['fpga']['tilt_us'] == 1076
    assert rows[0]['state']['config']['axes']['tilt']['calibration_points'][0]['pulse_us'] == 1076
    assert not rows[0]['state']['config_file_read']
    assert 'never collect' not in stream.getvalue() and 'not collected' not in stream.getvalue()
    now[0] = 20.
    with mock.patch('urllib.request.build_opener', return_value=Opener()):
        stream = io.StringIO()
        result = capture(stream, seconds=10, hz=20,
                         fetch=state_fetcher('http://127.0.0.1:8080/state'),
                         clock=lambda: now[0], sleep=sleep)
    assert result == dict(samples=200, successes=200, errors=0)
    now[0] = 30.
    slow_calls = []
    def slow_fetch():
        slow_calls.append(now[0])
        now[0] += .12
        if len(slow_calls) % 2:
            raise OSError('mock HTTP unavailable')
        return state
    stream = io.StringIO()
    result = capture(stream, seconds=5, hz=20, fetch=slow_fetch,
                     clock=lambda: now[0], sleep=sleep)
    assert result['successes'] > 0 and result['errors'] > 0
    assert all(b - a >= .149999 for a, b in zip(slow_calls, slow_calls[1:]))
    for url in ('http://192.168.10.3:8080/state', 'http://127.0.0.1:8080/command',
                'http://user:password@127.0.0.1:8080/state', 'http://127.0.0.1:8080/state?token=x'):
        try: validate_url(url)
        except ValueError: pass
        else: raise AssertionError('nonlocal/control URL admitted')
    print('PASS read-only capture:10Hz/5s and20Hz/10s mocked GETs, slow/error recovery without bursts, limits/PWM/RBT metadata whitelist; no real network/devices/files read')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='Explicit new JSONL output path; existing files are not overwritten')
    parser.add_argument('--seconds', type=float, default=5, help='5..10seconds')
    parser.add_argument('--hz', type=float, default=10, help='10..20samples/second, best effort')
    parser.add_argument('--url', default='http://127.0.0.1:8080/state')
    parser.add_argument('--self-test', action='store_true', help='Fake HTTP/clock only; no actual connection')
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    if args.output is None:
        parser.error('--output is required')
    if not 5 <= args.seconds <= 10 or not 10 <= args.hz <= 20:
        parser.error('--seconds must be5..10 and --hz must be10..20')
    try: fetch = state_fetcher(args.url)
    except ValueError as exc: parser.error(str(exc))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        stream.write(json.dumps(dict(type='capture', url=args.url, seconds=args.seconds, hz=args.hz,
            mode='GET /state only; no heartbeat/control/backend; no calibration files read',
            angle_note='PWM/table estimates; RBT is applied command, not shaft feedback'), ensure_ascii=False) + '\n')
        result = capture(stream, seconds=args.seconds, hz=args.hz, fetch=fetch)
    print(json.dumps(dict(output=str(args.output.resolve()), **result), ensure_ascii=False))
    return 0 if result['successes'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
