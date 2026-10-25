"""Laser-ranged visual task math. No actuator or network access in this module.

Body: X forward, Y left. Camera front reference is separate from the measured
PWM/angle and image Jacobian files. Laser returns and robot pose in LDR1 refer
to one completed stationary scan; a nearer obstacle never replaces a lock.
"""
import json
import math
from pathlib import Path
import struct

from target_tracking import camera_ray_to_body


def wrap(angle):
    return (angle + 180) % 360 - 180


def read_laser(raw):
    if len(raw) != 32 or raw[:5] != b'LDR1\x01':
        return None
    checksum = 0
    for byte in raw:
        checksum ^= byte
    if checksum:
        return None
    flags, seq, x, y, yaw, bx, by, target, candidates, points, age, status = struct.unpack(
        '>BHiiihhHBBHB', raw[5:31])
    return dict(flags=flags, seq=seq, x=x, y=y, yaw=yaw, bx=bx, by=by,
                target=target, candidates=candidates, points=points, age=age, status=status)


def world_point(report):
    a = math.radians(report['yaw'] / 1000)
    c, s = math.cos(a), math.sin(a)
    return (report['x'] + c*report['bx'] - s*report['by'],
            report['y'] + s*report['bx'] + c*report['by'])


def approach_goal(target, observer, standoff=512):
    if not 350 <= standoff <= 1500:
        raise ValueError('停车距离必须为350..1500mm（轮轴原点到目标）')
    dx, dy = target[0]-observer[0], target[1]-observer[1]
    distance = math.hypot(dx, dy)
    if distance <= standoff:
        return tuple(round(v) for v in observer)
    return (round(target[0]-standoff*dx/distance),
            round(target[1]-standoff*dy/distance))


class VisualTask:
    def __init__(self):
        self.config = None
        self.reference = None
        self.searching = False
        self.scan = []
        self.index = 0
        self.dwell = None
        self.turn_request = False
        self.turn_used = False
        self.turns = 0
        self.started = None
        self.turn_started = None
        self.turn_seen = False
        self.laser = None
        self.laser_at = -1e9
        self.lock = None
        self.lock_target = 0
        self.last_seq = None
        self.state = 0
        self.kind = 'dog'

    def load(self, app):
        if self.config is None:
            path = app.config_path.with_name('visual_task.json')
            config = json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
            if not isinstance(config, dict):
                raise ValueError('visual_task.json 必须是对象')
            distance=config.get('standoff_mm',512)
            if type(distance) is not int or not 350<=distance<=1500:
                raise ValueError('standoff_mm 必须是350..1500的整数毫米')
            camera=config.get('camera_origin_mm')
            if camera is not None and (not isinstance(camera,list) or len(camera)!=2 or
                    any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>500 for v in camera)):
                raise ValueError('camera_origin_mm 必须是两个有效毫米数，分别为轮轴系X/Y')
            if type(config.get('pan_sign',1)) is not int or config.get('pan_sign',1) not in (-1,1):
                raise ValueError('pan_sign 必须是+1或-1')
            ref = config.get('front_reference')
            if ref is not None and (not isinstance(ref,dict) or type(ref.get('offset')) not in (int,float) or not math.isfinite(ref['offset'])):
                raise ValueError('相机车头参考格式错误，请重新记录')
            if ref and ref.get('angle_signature') == app._angle_signature() and ref.get('upright') == app._mount_upright():
                self.reference = ref
            self.config=config

    def cancel(self):
        self.searching = self.turn_request = False
        self.scan = []
        self.lock = None
        self.lock_target = 0
        self.state = 0

    def axis_bearing(self, app, pan, tilt):
        # No focal length is invented. A centered detection permits use of
        # the calibrated optical axis, with an explicit approximate cone.
        sign = self.config.get('pan_sign', 1)
        if sign not in (-1, 1):
            raise ValueError('visual_task.json pan_sign 必须为+1或-1')
        ray = camera_ray_to_body((1, 0, 0), sign*pan, tilt,
                                 upright_tilt_deg=app._mount_upright())
        if math.hypot(ray[0], ray[1]) < .5:
            raise ValueError('相机接近朝天，无法关联水平雷达')
        return math.degrees(math.atan2(ray[1], ray[0]))

    def teach_front(self, app, pan):
        self.load(app)
        target=(app.tracking_result or {}).get('target')
        if not target or abs((target['x1']+target['x2'])/(2*app.latest.width)-.5)>.04:
            raise ValueError('将目标放在车头正前方，等待画面居中再记录相机车头方向')
        # The measured image Jacobian cannot give absolute yaw/depth. It CAN
        # identify PAN's sign: du/dpan = image_sign*cos(tilt)*pan_sign*fx.
        # Existing fits factor out cos(tilt). Use only a dominant horizontal
        # response in the already applied, matching calibration; manual sign
        # in visual_task.json always takes precedence.
        raw=app.config.get('image_calibration', {})
        model=app.image_calibration_model
        if ('pan_sign' not in self.config or self.config.get('pan_sign_source')=='image_calibration') and raw.get('enabled') and raw.get('angle_signature')==app._angle_signature() and model.get('valid'):
            b=model['matrix_b']
            if abs(b[0][0])>2*abs(b[1][0]) and abs(b[0][0])>1e-4:
                image_sign=1 if app._mount_upright()==0 else -1
                self.config['pan_sign']=(1 if b[0][0]>0 else -1)*image_sign
                self.config['pan_sign_source']='image_calibration'
        tilt = app.gimbal.snapshot()['axes']['tilt']['angle_deg']
        bearing = self.axis_bearing(app, pan, tilt)
        self.reference = dict(offset=-bearing, angle_signature=app._angle_signature(),
                              upright=app._mount_upright())
        self.config['front_reference'] = self.reference
        path = app.config_path.with_name('visual_task.json')
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.config, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        temporary.replace(path)
        self.state = 23

    def start(self, app, kind='dog'):
        if kind not in ('dog','human'):
            raise ValueError('搜索类别只能为dog或human')
        self.load(app)
        self.cancel()
        self.kind=kind
        cfg = app.gimbal.axes['pan']
        if not cfg.calibrated:
            raise ValueError('请先完成原有PAN角度标定')
        low, high = cfg.calibration_points[0][0], cfg.calibration_points[-1][0]
        if high-low < 100:
            raise ValueError('PAN已标定角度范围至少100度，才能完成四向搜索')
        # Stay within measured angles AND confirmed electrical/mechanical limits.
        angles = [low+5+i*30 for i in range(max(0, int((high-low-10)/30)+1))]
        self.scan = [a for a in angles if cfg.safe_min_us <= cfg.angle_to_pulse(a) <= cfg.safe_max_us]
        if not self.scan:
            raise ValueError('PAN标定范围不足以搜索')
        self.searching = True
        self.index = 0
        self.dwell = None
        self.turn_used = self.turn_seen = False
        self.turns = 0
        self.started = self.turn_started = None
        self.state = 20

    def search_tick(self, app, now, report):
        if not self.searching:
            return
        if self.started is None:
            self.started = now
        if now-self.started > 180 or (self.turn_request and self.turn_started is not None and now-self.turn_started > 12):
            self.searching = self.turn_request = False
            app.gimbal.hold()
            self.state = 22
            return
        if self.turn_request:
            if report.get('flags', 0) & 64:
                self.turn_request = False
                self.turn_seen = True
                self.index = 0
                self.dwell = None
            return
        # Completion flag must drop before requesting any further action.
        if self.turn_seen and report.get('flags', 0) & 64:
            return
        axes = app.gimbal.snapshot()['axes']
        target = self.scan[self.index]
        app.gimbal.set_angle('pan', target)
        angle = axes['pan']['angle_deg']
        if angle is None or abs(angle-target) > 2:
            self.dwell = None
            return
        if self.dwell is None:
            self.dwell = now
        if now-self.dwell < 1.2:
            return
        self.index += 1
        self.dwell = None
        if self.index == len(self.scan):
            if self.turns < 3:
                self.turn_used = self.turn_request = True
                self.turns += 1
                self.turn_started = now
                self.state = 21
            else:
                self.searching = False
                app.gimbal.hold()
                self.state = 22

    def query(self, app, now, selected, fresh):
        self.load(app)
        if self.state==27:
            fresh=False  # Withdraw an inconsistent observation immediately.
        flags = 2 if self.turn_request else 0
        bearing = 0
        if fresh and self.reference and self.reference['angle_signature'] == app._angle_signature():
            pose = app.latest.gimbal_at_arrival.get('axes', {})
            # Pose attached at image arrival, not the newer command target.
            pan, tilt = pose.get('pan', {}).get('angle_deg'), pose.get('tilt', {}).get('angle_deg')
            if pan is not None and tilt is not None:
                bearing = wrap(self.axis_bearing(app, pan, tilt)+self.reference['offset'])
                flags |= 1
        a = math.radians(bearing)
        camera = self.config.get('camera_origin_mm')
        if camera is None:
            cx, cy, uncertainty = 0, 0, 261
        else:
            cx, cy = map(round, camera)
            if abs(cx) > 500 or abs(cy) > 500:
                raise ValueError('相机原点超出车体，请以轮轴原点测量')
            uncertainty = 25
        body = struct.pack('>HHhhhhHhhI', selected, 0 if fresh else 65535,
                           round(16384*math.cos(a)), round(16384*math.sin(a)),
                           cx, cy, 2048,
                           round(bearing*100), uncertainty, 0)
        return flags, body

    def accept(self, report, now, selected):
        if self.last_seq is not None:
            delta = (report['seq']-self.last_seq)&65535
            if not 0 < delta < 32768 and now-self.laser_at < 2:
                return
        self.last_seq = report['seq']
        self.laser, self.laser_at = report, now
        if not report['flags']&1 or report['age']>500 or report['target'] != selected or not selected:
            return
        target = world_point(report)
        if self.lock_target != selected:
            self.lock = None
        # Fixed toy for the search-and-approach task. Do not reclassify a new
        # foreground return or another wall as the goal during navigation.
        if self.lock is None:
            self.lock, self.lock_target = target, selected
        elif math.hypot(target[0]-self.lock[0], target[1]-self.lock[1]) > 150:
            self.state = 27

    def goal_body(self, now, selected):
        r = self.laser
        if not r or now-self.laser_at > .5 or not r['flags']&1 or self.lock_target != selected or self.state == 27:
            return None
        goal = approach_goal(self.lock, (r['x'], r['y']), self.config.get('standoff_mm', 512))
        if any(abs(v) >= 4000 for v in (*goal, *self.lock)):
            self.state=28
            return None
        return struct.pack('>HHiiiiH', selected, 0, *goal, *map(round, self.lock),
                           self.config.get('standoff_mm', 512))
