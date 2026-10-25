"""Metadata-only USB vision/FPGA bridge. No camera pixels or Linux PWM writes.

Side-follow demo: a taught PAN reference and direction generate a common
COUNT/10ms wheel target. This is not monocular metric position estimation.
"""
from __future__ import annotations

import binascii
from collections import deque
from datetime import datetime
import json
import hashlib
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import time
from visual_task import VisualTask, read_laser


def packet(magic, flags, sequence, body):
    raw = magic + struct.pack('>BBH', 1, flags, sequence & 65535) + body
    if len(raw) != 30:
        raise ValueError('metadata payload must be30 bytes before CRC')
    return raw + struct.pack('>H', binascii.crc_hqx(raw, 0xffff))


def read_report(raw):
    if len(raw) != 32 or raw[:5] != b'RBT1\x01':
        return None
    checksum = 0
    for byte in raw:
        checksum ^= byte
    if checksum:
        return None
    flags, event, op, target, gain, cap, yaw, pan, tilt, ack, left, right, catalog, status = struct.unpack(
        '>BHBHBHiHHHhhHB', raw[5:31])
    return dict(flags=flags, event=event, op=op, target=target, gain=gain,
                cap=cap, yaw_mdeg=yaw, pan_us=pan, tilt_us=tilt,
                ack=ack, left=left, right=right, catalog=catalog, nav_status=status)


def box_geometry(box):
    x, y = (box['x1']+box['x2'])/2, (box['y1']+box['y2'])/2
    area = (box['x2']-box['x1'])*(box['y2']-box['y1'])
    return x, y, area


ADMISSION_SCORE = .50
SOFTWARE_VERSION = 'v22'


class FollowError(ValueError):
    def __init__(self, code, text):
        super().__init__(text)
        self.code = code


class CandidateRegistry:
    """Three-frame admission, stable IDs, conservative same-class association.

    IDs describe tracks, not recognized identities. Nearby competing boxes
    produce a miss; they do not silently replace the selected object.
    """
    def __init__(self):
        self.tracks = {}
        self.next_id = 1
        self.selected = 0
        self.sequence = -1

    def update(self, boxes, now, sequence, width, height):
        if sequence <= self.sequence:
            return
        self.sequence = sequence
        boxes = [dict(b) for b in boxes if b.get('kind') in ('dog', 'human') and
                 b.get('score', 0) >= ADMISSION_SCORE and box_geometry(b)[2] > 0]
        choices = {}
        for key, track in list(self.tracks.items()):
            if key != self.selected and now-track['time'] > 2:
                del self.tracks[key]
                continue
            old = track['box']
            x, y, area = box_geometry(old)
            scores = []
            for index, box in enumerate(boxes):
                bx, by, ba = box_geometry(box)
                ratio = max(ba/area, area/ba)
                distance = math.hypot((bx-x)/width, (by-y)/height)
                if box['kind'] == old['kind'] and ratio <= 2 and distance <= .28:
                    scores.append((distance + .08*abs(math.log(ba/area)), index))
            scores.sort()
            # Same-class distractors close to the best match are ambiguous.
            if scores and (len(scores) == 1 or scores[1][0]-scores[0][0] > .04):
                choices[key] = scores[0]
        claimed = {}
        for key, (score, index) in sorted(choices.items(), key=lambda p: p[1][0]):
            if index not in claimed:
                track = self.tracks[key]
                track['hits'] = track['hits']+1 if now-track['time'] < .6 else 1
                track.update(box=boxes[index], time=now, sequence=sequence)
                claimed[index] = key
        for index, box in enumerate(boxes):
            if index not in claimed:
                # Do not spawn an ID for an ambiguous box alongside the old
                # selected track; it must become independently stable first.
                key = self.next_id
                while key in self.tracks or key == 0:
                    key = (key+1) & 65535
                self.next_id = (key+1) & 65535
                self.tracks[key] = dict(box=box, time=now, sequence=sequence, hits=1)

    def catalog(self, now, kind=None, min_hits=3):
        candidates = [(key, t) for key, t in self.tracks.items()
                      if t['hits'] >= min_hits and now-t['time'] < .5 and
                      (kind is None or t['box']['kind']==kind)]
        candidates.sort(key=lambda item: (-item[1]['box']['score'], item[0]))
        return candidates[:4]

    def selected_boxes(self, sequence):
        target = self.tracks.get(self.selected)
        return [target['box']] if target and target['hits'] >= 3 and target['sequence'] == sequence else []


class PanFollower:
    def __init__(self):
        self.reference = None
        self.forward_sign = 0
        self.output = 0.0
        self.filtered = 0.0
        self.active = False
        self.history = deque(maxlen=3)
        self.last_time = None

    def capture(self, pan, area):
        self.reference = (pan, area)
        self.forward_sign = 0
        self.output = self.filtered = 0.0
        self.history.clear()
        self.active = False

    def teach(self, pan):
        if self.reference is None or abs(pan-self.reference[0]) < 3:
            raise FollowError(10,'请将小狗朝车头移动，使PAN变化至少3度，再按FRONT SAMPLE')
        self.forward_sign = 1 if pan > self.reference[0] else -1

    def update(self, pan, now, good, gain, cap):
        dt = .02 if self.last_time is None else max(0, min(.1, now-self.last_time))
        self.last_time = now
        if not good or self.reference is None or not self.forward_sign:
            self.output = 0.0
            self.active = False
            self.history.clear()
            self.filtered = 0.0
            return 0
        error = self.forward_sign*(pan-self.reference[0])
        self.history.append(error)
        observed = sorted(self.history)[len(self.history)//2]
        self.filtered += (observed-self.filtered)*(1-math.exp(-dt/.15))
        if abs(error) <= 3:
            self.active = False
        elif abs(error) > 5 and abs(self.filtered) > 5:
            self.active = True
        target = max(-cap, min(cap, self.filtered*gain)) if self.active else 0
        slew = 20*dt  #1COUNT/50ms
        self.output += max(-slew, min(slew, target-self.output))
        return round(self.output)


class FpgaPWMBackend:
    physical = True

    def __init__(self):
        self.enabled = {}

    def start(self, axis, pulse_us):
        if not isinstance(pulse_us,int) or isinstance(pulse_us,bool) or not 1 <= pulse_us <= 4095:
            raise ValueError('FPGA PWM request must be1..4095us')
        self.enabled[axis] = int(pulse_us)

    def set_pulse(self, axis, pulse_us):
        if axis not in self.enabled:
            raise ValueError('FPGA PWM axis is disabled')
        self.start(axis, pulse_us)

    def disable(self, axis):
        self.enabled.pop(axis, None)

    def diagnostics(self):
        return dict(type='fpga_udp', physical=True, note='PWM命令，实际应用见FPGA回报')


class FollowLink:
    def __init__(self, backend, log_dir, initial_pan=1200, initial_tilt=2100, *, sock=None, initial_tilt_angle=135.0):
        self.backend = backend
        self.visual = VisualTask()
        self.visual_sequence = self.goal_sequence = 0
        self.registry = CandidateRegistry()
        self.follower = PanFollower()
        self.initial = dict(pan=initial_pan, tilt=initial_tilt)
        self.initial_tilt_angle=initial_tilt_angle
        self.sock = sock or socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(('0.0.0.0', 6102))
        self.sock.setblocking(False)
        self.report = None
        self.report_at = -1e9
        self.last_event = None
        self.session_seen = False
        self.session_reset_pending = False
        self.event_ack = 0
        self.sequence = self.catalog_sequence = 0
        self.last_sent = -1e9
        self.last_catalog = -1e9
        self.error = None
        self.auto_select = False
        self.auto_kind = 'dog'
        self.auto_after = -1e9
        self.auto_pose_pending = False
        self.auto_hits = {}
        self.pose_ready_after = -1e9
        self.pose_wait_ack = False
        self.geometry_active = False
        self.auto_reference = False
        self.error_code = 0
        self.transport_error = None
        self.shutdown_event = None
        self.shutdown_started = False
        self.shutdown_accepted = False
        path = Path(log_dir)
        path.mkdir(parents=True, exist_ok=True)
        self.log_path = path/(datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.jsonl')
        self.log = self.log_path.open('a', encoding='utf-8', buffering=1)

    def pose_info(self, app):
        """Command/table estimates only; these servos have no angle sensor."""
        cfg=app.gimbal.axes['tilt']
        pulse=(cfg.angle_to_pulse(self.initial_tilt_angle) if
               self.initial_tilt_angle is not None and cfg.calibrated else self.initial['tilt'])
        return dict(software_version=SOFTWARE_VERSION, source=str(Path(__file__).resolve()),
                    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()[:16],
                    config=str(app.config_path), requested_tilt_angle_deg=self.initial_tilt_angle,
                    tilt_pwm_us=pulse, table_angle_estimate_deg=cfg.pulse_to_angle(pulse),
                    tilt_operating_min_deg=app.gimbal.operating_angle_min.get('tilt'),
                    pan_initial_pwm_us=self.initial['pan'], physical_angle_feedback=False)

    def log_runtime(self, app):
        try:info=self.pose_info(app)
        except (ValueError,RuntimeError) as exc:
            info=dict(software_version=SOFTWARE_VERSION, source=str(Path(__file__).resolve()),
                      config=str(app.config_path), pose_error=str(exc),physical_angle_feedback=False)
        self.record('runtime',**info)
        self.runtime_info=info
        print('FOLLOW runtime: '+json.dumps(info,ensure_ascii=False),flush=True)

    @staticmethod
    def detector_error(app,kind):
        detection=getattr(app,'detection_status',{}) or {}
        if detection.get('state') in ('enabled','partial','error') and \
                detection.get('dog' if kind=='dog' else 'face') is False:
            return detection.get('errors',{}).get(kind) or detection.get('error') or '此类别检测已关闭'
        return None

    def record(self, event_kind, **data):
        if 'event' in data:
            data['ui_event'] = data.pop('event')
        self.log.write(json.dumps(dict(wall_time=datetime.now().isoformat(), monotonic=time.monotonic(),
                                      event=event_kind, **data), ensure_ascii=False, allow_nan=False)+'\n')

    def observe(self, frame):
        self.registry.update(frame.detections, frame.arrived_at, frame.sequence, frame.width, frame.height)
        self.record('vision', sequence=frame.sequence, captured=frame.arrived_at,
                    detections=frame.detections, pose=frame.gimbal_at_arrival)

    def tracking_boxes(self, frame):
        selected = self.registry.tracks.get(self.registry.selected)
        if selected is None:
            return []
        # Keep the established pose-aware TargetTracker in charge of locked
        # association/reacquisition. The catalog's pixel-only association is
        # for human selection; it must not veto camera-motion compensation.
        kind = selected['box']['kind']
        return [b for b in frame.detections if b.get('kind') == kind and b.get('score',0) >= ADMISSION_SCORE]

    def confirm(self, frame, update):
        if update.get('status') == 'tracking' and update.get('target'):
            selected = self.registry.tracks.get(self.registry.selected)
            if selected:
                selected.update(box=dict(update['target']), time=frame.arrived_at,
                                sequence=frame.sequence, hits=max(3,selected['hits']))

    def _fresh_target(self, app, now):
        frame = app.latest
        update = app.tracking_result or {}
        if frame is None or now-frame.arrived_at > app.max_age or update.get('status') != 'tracking' or app._processed_sequence != frame.sequence:
            raise FollowError(8,'等待所选目标稳定跟踪后再记录参考或示教')
        boxes = [update['target']] if update.get('target') else []
        if not boxes:
            raise FollowError(6,'所选目标当前不可见或存在歧义')
        x, y, area = box_geometry(boxes[0])
        if abs(x/frame.width-.5) > .15 or abs(y/frame.height-.5) > .15:
            raise FollowError(8,'请保持玩偶不动，等云台把它移到中心后再记录')
        pan = app.gimbal.snapshot()['axes']['pan']['angle_deg']
        if pan is None:
            raise FollowError(7,'请保留原有PAN角度标定')
        return pan, area

    def poll(self, app, now):
        for _ in range(12):
            try:
                raw, peer = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            if peer[0] != '192.168.10.2':
                continue
            laser = read_laser(raw)
            if laser is not None:
                if self.geometry_active:self.visual.accept(laser, now, self.registry.selected)
                self.record('laser_target', **laser, world_lock=self.visual.lock)
                continue
            report = read_report(raw)
            if report is None:
                continue
            previous=self.report
            self.report, self.report_at = report, now
            self.record('fpga', **report)
            if previous and previous['event']!=0 and report['event']==0 and report['op']==0 and report['target']==0:
                self.auto_select=self.auto_reference=False
                self.geometry_active=False
                self.visual.cancel();self.follower=PanFollower();self.registry.selected=0
                app.tracking=False;app._hold_tracking();app.gimbal.disable()
                self.session_seen=False
            if not self.session_seen:
                # A Pi restart creates a new track registry. Baseline the
                # FPGA's remembered event, never replay a previous selection,
                # tracking authorization or shutdown as a new user action.
                self.session_seen=True
                self.last_event=(report['event'],report['op'],report['target'])
                self.event_ack=report['event']
                self.session_reset_pending=bool(report['flags']&128 and (report['op'] or report['target'] or report['flags']&16))
                if report['flags'] & 16:self.shutdown_event=(report['event'],report['op'])
                self.record('session_baseline',event=report['event'],op=report['op'])
                continue
            if self.session_reset_pending:
                if report['op']==0 and report['target']==0 and not report['flags']&16:
                    self.session_reset_pending=False
                else:
                    continue
            if report['flags'] & 16:
                if report['op']!=6: continue
                shutdown_key=(report['event'],report['op'])
                if shutdown_key != self.shutdown_event:
                    self.shutdown_event=shutdown_key
                    self._poweroff(app,now)
                continue
            if self.shutdown_started:
                continue
            key = (report['event'], report['op'], report['target'])
            if key == self.last_event:
                continue
            if report['op'] in (2,3) and app.latest is not None and app._processed_sequence != app.latest.sequence:
                # Inference just published; let the controller consume that
                # image before taking a reference. RBT repeats the event.
                continue
            self.last_event = key
            try:
                if report['op'] == 1:
                    self.geometry_active=False
                    self.visual.cancel()
                    self._select(app,report['target'],now)
                elif report['op'] in (5,9):
                    self._begin_auto(app,'dog' if report['op']==5 else 'human',now)
                elif report['op'] in (7,10):
                    self._begin_auto(app,'dog' if report['op']==7 else 'human',now,searching=True)
                elif report['op'] == 11:
                    # Explicit VISION page entry prepares the working pose
                    # BEFORE a target must be visible. It never starts wheels.
                    app.tracking=False;app._hold_tracking();self.registry.selected=0
                    self.auto_select=False;self.auto_reference=False
                    self.geometry_active=False
                    self.follower=PanFollower();self.visual.cancel();self._arm(app,now)
                elif report['op'] == 8:
                    self.visual.teach_front(app, self._fresh_target(app, now)[0])
                    self.geometry_active=True
                elif report['op'] == 2:
                    self.follower.capture(*self._fresh_target(app, now))
                elif report['op'] == 3:
                    self.follower.teach(self._fresh_target(app, now)[0])
                elif report['op'] == 4:
                    app.tracking = False
                    app._hold_tracking()
                    app.gimbal.disable()
                    self.registry.selected = 0
                    self.auto_select=False;self.auto_reference=False
                    self.geometry_active=False
                    self.follower = PanFollower()
                    self.visual.cancel()
                self.event_ack = report['event']
                self.error = None
                self.error_code=0
            except (ValueError, RuntimeError, OSError) as exc:
                self.error = str(exc)
                self.error_code=getattr(exc,"code",11)
                self.follower.output = 0
                self.follower.active = False
                if report['op'] in (1,5,7,9,10,11):
                    self.registry.selected=0
                    self.follower = PanFollower()
                    app.tracking = False
                    app._hold_tracking()
                    app.gimbal.disable()
                    self.visual.cancel();self.auto_select=False
                self.record('command_error', command=report, error=self.error)
                print('FOLLOW: '+self.error, flush=True)
        if self.auto_select:
            # Filter class BEFORE the four-entry display limit: several
            # high-confidence faces must not hide a stable dog, or vice versa.
            failure=self.detector_error(app,self.auto_kind)
            if failure:
                self.error=('小狗' if self.auto_kind=='dog' else '人脸')+'检测不可用：'+failure
                self.error_code=2;self.auto_select=False;self.visual.cancel()
                app.tracking=False;app._hold_tracking()
                self.record('detector_error',kind=self.auto_kind,error=self.error)
                print('FOLLOW: '+self.error,flush=True)
            candidates=[] if failure else self.registry.catalog(now,kind=self.auto_kind,min_hits=1)
            pose_ack=(not self.auto_pose_pending or all((self.report or {}).get(axis+'_us')==
                self.backend.enabled.get(axis) for axis in ('pan','tilt')))
            ready=now>=self.auto_after and pose_ack
            if ready:self.pose_wait_ack=False;self.auto_pose_pending=False
            stable=[]
            for key,track in candidates:
                sequence,hits=self.auto_hits.get(key,(-1,0))
                if not ready or track['time']<self.auto_after:hits=0
                elif track['sequence']!=sequence:hits=min(hits+1,track['hits'])
                self.auto_hits[key]=(track['sequence'],hits)
                if hits>=3:stable.append((key,track))
            if stable:
                try:
                    was_searching=self.visual.searching
                    self._select(app,stable[0][0],now)
                    self.auto_select=False;self.error=None;self.error_code=0
                    self.visual.searching=False;self.visual.turn_request=False
                    self.visual.state=24 if was_searching else 0
                except (ValueError,RuntimeError) as exc:
                    self.error=str(exc);self.error_code=getattr(exc,'code',11)
                    self.auto_select=False
                    self.visual.cancel();self.registry.selected=0
                    app.tracking=False;app._hold_tracking();app.gimbal.disable()
                    self.record('command_error',error=self.error)
        if self.visual.searching and now-self.report_at <= .6:
            try:self.visual.search_tick(app,now,self.report or {})
            except (ValueError,RuntimeError) as exc:
                self.error=str(exc);self.error_code=7
                self.visual.cancel();self.auto_select=False
                app.tracking=False;app._hold_tracking();app.gimbal.disable()
                self.record('search_error',error=self.error)
            if not self.visual.searching and self.visual.state==22:self.auto_select=False
        if self.auto_reference and self.registry.selected and self.follower.reference is None:
            try:
                self.follower.capture(*self._fresh_target(app,now))
                self.auto_reference=False
            except ValueError:
                pass  # Centering is in progress; no wheel command is emitted.
        # Unplugging Ethernet cannot leave a remembered enabled output or old
        # reference queued for a later reconnection. Selection is explicit.
        if now-self.report_at > .6 and self.backend.enabled:
            app.tracking = False
            app._hold_tracking()
            app.gimbal.disable()
            self.follower = PanFollower()
            self.visual.cancel();self.auto_select=False;self.auto_reference=False
            self.geometry_active=False

    def _select(self, app, target, now):
        track=self.registry.tracks.get(target)
        if not track or now-track['time']>.5 or track['hits']<3:
            raise FollowError(6,'目标已过期，请重新选择人脸或小狗')
        self._arm(app,now)
        self.follower=PanFollower()
        app._hold_tracking()
        app.tracker.select_target(track['box'],track['time'],frame_seq=track['sequence'])
        self.registry.selected=target
        app.target_selected=True;app.tracking_result=None
        app.command(dict(action='track',enabled=True))

    def _begin_auto(self,app,kind,now,searching=False):
        self.geometry_active=bool(searching)
        self.registry.selected=0;app.tracking=False;app._hold_tracking()
        self.follower=PanFollower();self.visual.cancel()
        # In v20 the camera could remain at45deg while selection waited for a
        # dog. Prepare135deg first; warm selections retain the current PAN.
        self._arm(app,now)
        failure=self.detector_error(app,kind)
        if failure:raise FollowError(2,('小狗' if kind=='dog' else '人脸')+'检测不可用：'+failure)
        self.auto_kind=kind;self.auto_select=True
        self.auto_reference=kind=='dog' and not searching
        self.auto_after=max(now,self.pose_ready_after)
        self.auto_pose_pending=self.pose_wait_ack;self.auto_hits={}
        if searching:self.visual.start(app,kind=kind)
        self.record('auto_target',kind=kind,searching=searching,not_before=self.auto_after,
                    minimum_fresh_frames=3,wait_applied_pwm=self.auto_pose_pending)

    def _arm(self, app, now=None):
        if self.initial_tilt_angle is not None:
            # User corrected the WORKING POSE from45 to135, not the field
            # calibration labels. Convert through the real measured table.
            cfg=app.gimbal.axes['tilt']
            if not cfg.calibrated:
                raise FollowError(7,'请保留现场TILT角度标定；启动角度需由实测表换算')
            self.initial['tilt']=cfg.angle_to_pulse(self.initial_tilt_angle)
        for axis in ('pan','tilt'):
            cfg=app.gimbal.axes[axis]
            limits=app.gimbal.pulse_limits(axis)
            if not cfg.calibrated or cfg.clamp(self.initial[axis])!=self.initial[axis] or \
                    limits[0] is None or not limits[0]<=self.initial[axis]<=limits[1]:
                raise FollowError(7,'云台标定或初始位置超限；保留现场gimbal.json并核对PAN1200/TILT135度')
        moved=False
        for axis in ('pan','tilt'):
            if not app.gimbal.snapshot()['axes'][axis]['armed']:
                app.gimbal.arm(axis,self.initial[axis],physical=True)
                moved=True
        if moved:
            self.pose_ready_after=(time.monotonic() if now is None else now)+.30
            self.pose_wait_ack=True
            info=self.pose_info(app);self.record('prepare_pose',**info)
            print('FOLLOW prepared: '+json.dumps(info,ensure_ascii=False),flush=True)
        return moved

    def _poweroff(self, app, now):
        self.auto_select=False;self.auto_reference=False
        self.geometry_active=False
        self.visual.cancel()
        app.tracking=False;app._hold_tracking();app.gimbal.disable()
        self.follower=PanFollower()
        self.shutdown_accepted=False;self.error=None;self.error_code=0
        self.record('shutdown_requested',source='fpga',event=self.shutdown_event)
        try:
            check=subprocess.run(['sudo','-n','-l','/usr/bin/systemctl','poweroff'],capture_output=True,text=True,timeout=3)
            if check.returncode:
                raise FollowError(12,'未安装关机权限：'+check.stderr.strip())
            self.shutdown_accepted=True;self.shutdown_started=True
            self.last_sent=-1e9;self.send(app,now)
            self.log.flush();os.fsync(self.log.fileno())
            result=subprocess.run(['sudo','-n','/usr/bin/systemctl','poweroff'],capture_output=True,text=True,timeout=5)
            if result.returncode:
                raise FollowError(13,result.stderr.strip() or 'systemctl poweroff失败')
        except (FollowError,OSError,subprocess.TimeoutExpired) as exc:
            self.shutdown_started=False;self.shutdown_accepted=False
            self.error=str(exc);self.error_code=getattr(exc,'code',13)
            self.record('shutdown_failed',error=self.error)
            print('FOLLOW: '+self.error,flush=True)

    def diagnostics(self, app, now):
        frame=app.latest
        age=65535 if frame is None else min(65535,round(1000*max(0,now-frame.arrived_at)))
        detections=[] if frame is None else frame.detections
        dog=round(100*max((b['score'] for b in detections if b.get('kind')=='dog'),default=0))
        face=round(100*max((b['score'] for b in detections if b.get('kind')=='human'),default=0))
        selected=self.registry.tracks.get(self.registry.selected)
        kind=selected['box']['kind'] if selected else self.auto_kind
        if self.error_code:code=self.error_code
        elif self.shutdown_accepted:code=14
        elif self.auto_select and (now<self.auto_after or self.auto_pose_pending):code=47
        elif kind=='human' and self.visual.state in (20,21):code=44
        elif kind=='human' and self.visual.state==22:code=45
        elif kind=='human' and (self.visual.lock is not None or self.visual.state==27):code=46
        elif self.visual.state==20:code=32
        elif self.visual.state==21:code=33
        elif self.visual.state==22:code=34
        elif self.visual.state==23:code=35
        elif self.visual.state==27:code=37
        elif self.visual.state==28:code=31
        elif self.visual.lock is not None:code=39
        elif self.visual.state==24 and self.visual.reference is None:code=36
        elif self.visual.state==24 and self.visual.laser is not None:code=37 if self.visual.laser['status']==2 else 38
        elif frame is None:code=1
        elif getattr(app,'error',None) or (getattr(app,'camera',None) and app.camera.error):code=2
        elif self.detector_error(app,kind):code=2
        elif age>750:code=18
        elif kind=='human' and face<round(100*ADMISSION_SCORE):code=41 if face else 40
        elif kind=='human' and not self.registry.catalog(now,kind='human'):code=42
        elif kind!='human' and dog<round(100*ADMISSION_SCORE):code=4 if dog else 3
        elif kind!='human' and not self.registry.catalog(now,kind='dog'):code=5
        elif self.registry.selected and not app.tracking:code=6
        elif app.tracking and (app.tracking_result or {}).get('status')!='tracking':code=8
        elif self.follower.reference and not self.follower.forward_sign:code=10
        elif self.follower.forward_sign:code=16
        else:code=(43 if kind=='human' else 17) if app.tracking else 0
        return code,dog,face,age

    def _transmit(self, raw):
        try:
            self.sock.sendto(raw,('192.168.10.255',5001))
        except OSError as exc:
            # Cable changes routinely cause ENETUNREACH while the NIC has no
            # carrier. This is transport state, not a latched PWM failure.
            error=str(exc)
            if error!=self.transport_error:self.record('link_send_error',error=error)
            self.transport_error=error
            self.follower.output=0
            return False
        if self.transport_error:self.record('link_send_recovered')
        self.transport_error=None
        return True

    def send(self, app, now):
        if now-self.last_sent < .05:
            return
        self.last_sent = now
        axes = app.gimbal.snapshot()['axes']
        frame, update = app.latest, app.tracking_result or {}
        age = 65535 if frame is None else min(65535, round(1000*max(0, now-frame.arrived_at)))
        boxes = [update['target']] if frame is not None and update.get('target') else []
        good = bool(app.tracking and boxes and update.get('status') == 'tracking' and
                    age <= 500 and not app.error and not self.error and now-self.report_at <= .6)
        pan = axes['pan']['angle_deg']
        report = self.report or dict(gain=20, cap=5)
        velocity = self.follower.update(pan or 0, now, good, report['gain']/100, min(20, report['cap']))
        flags = int(good) | (int(self.follower.reference is not None)<<1) | (int(bool(self.follower.forward_sign))<<2)
        if self.shutdown_accepted:
            flags |= 32
        if self.error:
            flags |= 64
        pulses, limits = [], []
        for index, axis in enumerate(('pan', 'tilt')):
            cfg = app.gimbal.axes[axis]
            pulse = self.backend.enabled.get(axis, self.initial[axis])
            low, high = app.gimbal.pulse_limits(axis) if hasattr(app.gimbal,'pulse_limits') else (cfg.safe_min_us,cfg.safe_max_us)
            # Until measured configuration exists, send valid disabled limits
            # only. Do not authorize output based on an arbitrary default.
            if low is None or high is None or not 1 <= low <= pulse <= high <= 4095:
                low, high, pulse = 1, 4095, self.initial[axis]
            elif axis in self.backend.enabled:
                flags |= 1 << (3+index)
            pulses.append(pulse)
            limits.extend((low, high))
        confidence = round(1000*boxes[0]['score']) if boxes else 0
        if self.session_reset_pending:
            # Fresh Pi process clears the previous session with ALL outputs
            # disabled; it cannot replay a target or old poweroff request.
            flags=128;velocity=0;confidence=0
        body = struct.pack('>h10H', velocity, *pulses, *limits, age, confidence,
                           self.event_ack, self.registry.selected)
        raw = packet(b'FOL1', flags, self.sequence, body)
        if not self._transmit(raw):return
        self.record('command', sequence=self.sequence, flags=flags, target=self.registry.selected,
                    speed=velocity, pulses=pulses, reference=self.follower.reference,
                    forward_sign=self.follower.forward_sign, error=self.error)
        self.sequence = (self.sequence+1) & 65535
        if report.get('flags',0)&128:
            # Direction and goal proposals are data, never motion authority.
            try:
                if self.geometry_active:
                    centered=False
                    if good:
                        x,y,_=box_geometry(boxes[0])
                        centered=abs(x/frame.width-.5)<=.08 and abs(y/frame.height-.5)<=.08
                    flags,body=self.visual.query(app,now,self.registry.selected,good and centered)
                else:
                    # Independent DOG/FACE centering needs neither a laser
                    # plane nor a car-front reference. Withdraw any old query
                    # without loading geometric configuration or deriving a
                    # horizontal ray from a near-vertical camera.
                    flags=0
                    body=struct.pack('>HHhhhhHhhI',0,65535,16384,0,0,0,2048,0,261,0)
                self._transmit(packet(b'VEC1',flags,self.visual_sequence,body))
                self.visual_sequence=(self.visual_sequence+1)&65535
                body=self.visual.goal_body(now,self.registry.selected) if self.geometry_active else None
                if body is not None:
                    self._transmit(packet(b'GOL1',0,self.goal_sequence,body))
                    self.goal_sequence=(self.goal_sequence+1)&65535
            except (ValueError,RuntimeError) as exc:
                self.error=str(exc);self.error_code=7
                self.record('visual_geometry_error',error=self.error)
        if now-self.last_catalog >= .2:
            self.last_catalog = now
            candidates = self.registry.catalog(now)
            entries = b''.join(struct.pack('>HBB', key, 1 if t['box']['kind']=='dog' else 2,
                                           round(t['box']['score']*100)) for key, t in candidates)
            code,dog,face,frame_age=self.diagnostics(app,now)
            extension=struct.pack('>BBBBH',0xA1,code,dog,face,frame_age) if (self.report or {}).get('flags',0)&128 else b'\0'*6
            raw = packet(b'TGT1', len(candidates), self.catalog_sequence, entries.ljust(16, b'\0')+extension)
            if not self._transmit(raw):return
            self.catalog_sequence = (self.catalog_sequence+1) & 65535

    def close(self, app):
        app.tracking = False
        app.gimbal.disable()
        self.last_sent = -1e9
        try:
            self.send(app, time.monotonic())
        finally:
            self.sock.close()
            self.log.close()
