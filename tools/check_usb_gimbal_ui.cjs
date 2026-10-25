#!/usr/bin/env node
/* Execute the actual page callbacks against responses from the real Python
 * HTTP handler and controller. Only the DOM transport is synthetic. No real
 * camera, PWM device, browser, listening socket or external network is used.
 * Usage: node tools/check_usb_gimbal_ui.cjs [--python /path/to/python]
 */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const {spawnSync} = require('node:child_process');
const root = path.resolve(__dirname, '..');
const supplied = process.argv.indexOf('--python');
const cached = 'C:/Users/Vera/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe';
const python = supplied >= 0 ? process.argv[supplied + 1] : process.env.PYTHON || (fs.existsSync(cached) ? cached : 'python3');
const pythonFixtures = String.raw`
import io,json,sys,tempfile,time
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from unittest import mock
sys.path.insert(0,str(Path.cwd()/'pc'))
import usb_gimbal
from gimbal_control import MockPWMBackend,default_config
from usb_gimbal import GimbalApp,VisionFrame,make_handler
now=[1000.0]
clock=SimpleNamespace(monotonic=lambda:now[0],perf_counter=time.perf_counter)

def request(app,body=None):
    method,route=('GET','/state') if body is None else ('POST','/command')
    payload=b'' if body is None else json.dumps(body).encode()
    raw=(f'{method} {route} HTTP/1.0\r\nHost: 127.0.0.1\r\n'
         f'Content-Type: application/json\r\nX-Control-Token: {app.token}\r\n'
         f'Content-Length: {len(payload)}\r\n\r\n').encode()+payload
    class MemoryConnection:
        def __init__(self): self.incoming,self.outgoing=io.BytesIO(raw),io.BytesIO()
        def makefile(self,*args): return self.incoming
        def sendall(self,data): self.outgoing.write(data)
        def settimeout(self,value): pass
    connection=MemoryConnection()
    make_handler(app)(connection,('127.0.0.1',0),SimpleNamespace())
    headers,data=connection.outgoing.getvalue().split(b'\r\n\r\n',1)
    return dict(status=int(headers.split()[1]),body=json.loads(data))

commands=[
 dict(action='heartbeat'),
 dict(action='manual_jog',axis='pan',direction=1,coarse=False),
 dict(action='manual_enable',axis='pan',pulse_us=1505),
 dict(action='manual_jog',axis='pan',direction=1,coarse=True),
 dict(action='manual_steps',axis='pan',fine_us=2,coarse_us=25),
 dict(action='mark_limit',axis='pan',limit='min'),
 dict(action='manual_jog',axis='pan',direction=1,coarse=True),
 dict(action='mark_limit',axis='pan',limit='max'),
 dict(action='manual_jog',axis='pan',direction=1,coarse=True),
 dict(action='clear_limits',axis='pan'),
 dict(action='disable',axis='pan'),
 dict(action='manual_jog',axis='pan',direction=-1,coarse=False),
 dict(action='manual_steps',axis='pan',fine_us=5,coarse_us=50),
 dict(action='manual_pulse',axis='pan',pulse_us=2500),
 dict(action='manual_enable',axis='pan',pulse_us=2500),
 dict(action='manual_enable',axis='tilt',pulse_us=1500),
 dict(action='manual_range',axis='pan',explore_min_us=500,explore_max_us=2600),
 dict(action='manual_jog',axis='pan',direction=1,coarse=False),
 dict(action='manual_range',axis='pan',explore_min_us=400,explore_max_us=2700),
 dict(action='manual_range',axis='pan',explore_min_us=300,explore_max_us=2700),
 dict(action='manual_pulse',axis='pan',pulse_us=20000),
 dict(action='capture_burst',category='positive'),
 dict(action='cancel_capture'),
 dict(action='save_tracking',tracking=dict(deadband=.04,normalized_gain_dps=50,
     max_rate_dps=20,filter_time_constant_s=.06,deadband_hysteresis=.015,
     min_rate_dps=2,max_control_dt_s=.2)),
 dict(action='disable'),
]
pan_points=[dict(pulse_us=380,angle_deg=45),dict(pulse_us=1882,angle_deg=180),dict(pulse_us=2550,angle_deg=240)]
tilt_points=[dict(pulse_us=2420,angle_deg=10),dict(pulse_us=2043,angle_deg=45),dict(pulse_us=645,angle_deg=175)]
tracking_commands=[
 dict(action='heartbeat'),
 dict(action='select',sequence=10,index=0),
 dict(action='track',enabled=True),
 dict(action='manual_enable',axis='pan',pulse_us=340),
 dict(action='manual_enable',axis='tilt',pulse_us=2043),
 dict(action='track',enabled=True),
 dict(action='disable',axis='pan'),
 dict(action='manual_pulse',axis='pan',pulse_us=1882),
 dict(action='manual_enable',axis='pan',pulse_us=1882),
 dict(action='save_angles',axis='pan',calibration_points=pan_points),
 dict(action='save_angles',axis='tilt',calibration_points=tilt_points),
 dict(action='select',sequence=11,index=0),
 dict(action='track',enabled=True),
 dict(action='track',enabled=False),
 dict(action='disable'),
]
calibration_commands=[
 dict(action='heartbeat'),
 dict(action='manual_enable',axis='pan',pulse_us=1882),
 dict(action='manual_enable',axis='tilt',pulse_us=2043),
 dict(action='calibration_begin',sequence=21,index=0),
 dict(action='manual_jog',axis='pan',direction=1,coarse=True),
 dict(action='calibration_finish',sequence=22,index=0),
 dict(action='calibration_begin',sequence=23,index=0),
 dict(action='manual_jog',axis='pan',direction=-1,coarse=True),
 dict(action='calibration_finish',sequence=24,index=0),
 dict(action='calibration_begin',sequence=25,index=0),
 dict(action='manual_jog',axis='tilt',direction=1,coarse=True),
 dict(action='calibration_finish',sequence=26,index=0),
 dict(action='calibration_begin',sequence=27,index=0),
 dict(action='manual_jog',axis='tilt',direction=-1,coarse=True),
 dict(action='calibration_finish',sequence=28,index=0),
 dict(action='calibration_apply'),
 dict(action='calibration_begin',sequence=30,index=0),
 dict(action='calibration_cancel'),
 dict(action='save_mount',upright_tilt_deg=0),
 dict(action='save_mount',upright_tilt_deg=180),
 dict(action='calibration_clear'),
 dict(action='disable'),
]
test_root=Path.cwd()/'tools'
with tempfile.TemporaryDirectory(prefix='usb-ui-fixture-',dir=test_root) as temporary:
    assert Path(temporary).resolve().parent==test_root.resolve()
    with mock.patch.object(usb_gimbal,'time',clock):
        config=default_config()
        config['axes']['tilt'].update(safe_min_us=645,safe_max_us=2420,
            calibration_points=[dict(angle_deg=-90,pulse_us=645),dict(angle_deg=0,pulse_us=1500),dict(angle_deg=90,pulse_us=2420)],
            calibration_confirmed=True)
        config_path=Path(temporary)/'gimbal.json'
        config_path.write_text(json.dumps(config),encoding='utf8')
        app=GimbalApp(config_path,backend=MockPWMBackend(),dataset=Path(temporary)/'photos')
        app.gimbal._clock=lambda:now[0]
        try:
            app.publish(VisionFrame(10,now[0],object(),b'fake-jpeg',[],
                app.gimbal.snapshot(),640,480,12.5,'original USB pixels'))
            initial=request(app)['body']
            records=[]
            for command in commands:
                response=request(app,command)
                if response['status']==200 and command['action']=='manual_jog':
                    for _ in range(100):
                        states=app.gimbal.snapshot()['axes'].values()
                        if all(not a['armed'] or a['output_us']==a['target_us'] for a in states): break
                        now[0]+=.02
                        app.gimbal.tick(now[0])
                    else: raise AssertionError('synthetic PWM did not settle')
                if command['action']=='cancel_capture':
                    app.capture_thread.join(1)
                    assert not app.capture_thread.is_alive()
                records.append(dict(request=command,response=response,state=request(app)['body']))
            fixtures=dict(initial=initial,records=records)
        finally: app.close()
        # Reproduce the confirmed angle tables with outputs initially OFF.
        # The wider synthetic PAN limits deliberately exercise an output that
        # is allowed mechanically but not covered by its measured angle table.
        config=default_config()
        config['axes']['pan'].update(explore_min_us=300,explore_max_us=2800,
            safe_min_us=340,safe_max_us=2600,calibration_points=pan_points,calibration_confirmed=True)
        config['axes']['tilt'].update(safe_min_us=645,safe_max_us=2420,
            calibration_points=tilt_points,calibration_confirmed=True)
        config_path=Path(temporary)/'tracking-gimbal.json'
        config_path.write_text(json.dumps(config),encoding='utf8')
        app=GimbalApp(config_path,backend=MockPWMBackend(),dataset=Path(temporary)/'tracking-photos')
        app.gimbal._clock=lambda:now[0]
        boxes=[dict(kind='dog',score=.9,x1=240,y1=160,x2=400,y2=320)]
        def publish(sequence,detections=None):
            app.publish(VisionFrame(sequence,now[0],object(),b'fake-jpeg',boxes if detections is None else detections,
                app.gimbal.snapshot(),640,480,12.5,'original USB pixels'))
        try:
            publish(10)
            initial=request(app)['body']
            records=[]
            for command in tracking_commands:
                response=request(app,command)
                if command['action']=='save_angles' and command['axis']=='tilt':
                    publish(11)  # A real next camera frame after calibration.
                records.append(dict(request=command,response=response,state=request(app)['body']))
            fixtures.update(tracking_initial=initial,tracking_records=records)
            # Obtain real memory/reacquisition HTTP states; no hand-authored
            # phase/status fixtures, listening socket or hardware are used.
            for command in (dict(action='heartbeat'),
                    dict(action='manual_enable',axis='pan',pulse_us=1882),
                    dict(action='manual_enable',axis='tilt',pulse_us=2043)):
                assert request(app,command)['status']==200
            publish(100)
            assert request(app,dict(action='select',sequence=100,index=0))['status']==200
            assert request(app,dict(action='track',enabled=True))['status']==200
            now[0]+=.02
            app.gimbal.tick(now[0])
            offcenter=[dict(kind='dog',score=.9,x1=290,y1=110,x2=450,y2=270)]
            publish(101,offcenter)
            app._control_once(now[0])
            now[0]+=.02
            publish(102,[])
            app._control_once(now[0])
            centering=request(app)['body']
            assert centering['tracking'] and centering['target_selected']
            assert centering['tracking_memory']['phase']=='centering_last_seen'
            goal=centering['tracking_memory']['goal']
            assert goal is not None
            for axis in ('pan','tilt'):
                points=config['axes'][axis]['calibration_points']
                assert min(p['angle_deg'] for p in points)<=goal[axis]<=max(p['angle_deg'] for p in points)
            for step in range(600):
                now[0]+=.02
                if step%25==0: assert request(app,dict(action='heartbeat'))['status']==200
                publish(103+step,[])
                app._control_once(now[0])
                waiting=request(app)['body']
                if waiting['tracking_memory']['phase']=='waiting': break
            else: raise AssertionError('last observed centering never completed')
            assert waiting['tracking'] and waiting['target_selected']
            now[0]+=app.tracker.config.loss_gap_s+.02
            assert request(app,dict(action='heartbeat'))['status']==200
            publish(1000,[])
            app._control_once(now[0])
            long_wait=request(app)['body']
            assert long_wait['tracking'] and long_wait['target_selected']
            now[0]+=.04
            publish(1001,[dict(boxes[0]),dict(boxes[0])])
            app._control_once(now[0])
            ambiguous=request(app)['body']
            assert ambiguous['tracking_result']['status']=='ambiguous'
            reacquiring=[]
            for index in range(3):
                now[0]+=.04
                publish(1002+index)
                app._control_once(now[0])
                reacquiring.append(request(app)['body'])
            assert reacquiring[0]['tracking_result']['status']=='reacquiring'
            assert reacquiring[0]['tracking_result']['reacquire_count']==1
            assert reacquiring[1]['tracking_result']['reacquire_count']==2
            assert reacquiring[2]['tracking_result']['status']=='tracking'
            assert reacquiring[2]['tracking_memory']['phase']=='live'
            assert request(app,dict(action='track',enabled=False))['status']==200
            paused=request(app)['body']
            before={axis:paused['gimbal']['axes'][axis]['target_us'] for axis in ('pan','tilt')}
            for index in range(3):
                now[0]+=.04
                publish(1010+index)
                app._control_once(now[0])
            paused_after=request(app)['body']
            assert not paused_after['tracking']
            assert before=={axis:paused_after['gimbal']['axes'][axis]['target_us'] for axis in ('pan','tilt')}
            fixtures.update(memory_states=dict(centering=centering,waiting=waiting,long_wait=long_wait,
                ambiguous=ambiguous,reacquiring1=reacquiring[0],reacquiring2=reacquiring[1],
                resumed=reacquiring[2],paused=paused,paused_after=paused_after))
            poses=[]
            for pulse in (2043,1559,645):
                assert request(app,dict(action='disable',axis='tilt'))['status']==200
                assert request(app,dict(action='manual_enable',axis='tilt',pulse_us=pulse))['status']==200
                poses.append(request(app)['body'])
            fixtures.update(pose_states=poses)
        finally: app.close()
        config['mount']=dict(upright_tilt_deg=180)
        config_path=Path(temporary)/'image-calibration-gimbal.json'
        config_path.write_text(json.dumps(config),encoding='utf8')
        app=GimbalApp(config_path,backend=MockPWMBackend(),dataset=Path(temporary)/'calibration-photos')
        app.gimbal._clock=lambda:now[0]
        cx,cy,sequence=330,250,20
        def calibration_publish():
            boxes=[dict(kind='human',score=.9,x1=cx-40,y1=cy-40,x2=cx+40,y2=cy+40)]
            app.publish(VisionFrame(sequence,now[0],object(),b'fake-jpeg',boxes,
                app.gimbal.snapshot(),640,480,12.5,'original USB pixels'))
        try:
            calibration_publish()
            initial=request(app)['body']
            records=[]
            for command in calibration_commands:
                response=request(app,command)
                if response['status']==200 and command['action']=='manual_jog':
                    for _ in range(100):
                        states=app.gimbal.snapshot()['axes'].values()
                        if all(not a['armed'] or a['output_us']==a['target_us'] for a in states): break
                        now[0]+=.02
                        app.gimbal.tick(now[0])
                    else: raise AssertionError('calibration PWM did not settle')
                    if command['axis']=='pan': cx-=20*command['direction']
                    else: cy-=20*command['direction']
                    sequence+=1
                    calibration_publish()
                elif response['status']==200 and (command['action'] in ('calibration_apply','calibration_finish') or
                        (command['action']=='manual_enable' and command['axis']=='tilt')):
                    sequence+=1
                    calibration_publish()
                records.append(dict(request=command,response=response,state=request(app)['body']))
            fixtures.update(calibration_initial=initial,calibration_records=records)
            print(json.dumps(fixtures,ensure_ascii=True))
        finally: app.close()
`;
const result = spawnSync(python, ['-X', 'utf8', '-c', pythonFixtures], {cwd:root, encoding:'utf8', maxBuffer:4*1024*1024});
if (result.error || result.status !== 0) throw result.error || new Error(result.stderr || result.stdout);
const fixtures = JSON.parse(result.stdout);
let state = fixtures.initial, consumed = 0, records = fixtures.records;
const html = fs.readFileSync(path.join(root,'pc/usb_gimbal_ui.html'),'utf8');
assert.equal((html.match(/v18/g)||[]).length,2,'title/header must show this package version');
assert.ok(!html.includes('save_tracking_direction'),'pose compensation must not add a manual direction API or button');
assert.ok(!html.includes('跟反了'),'automatic pose compensation must not require manual sign changes');
const nodes = new Map();
class Element {
  constructor(tag='div') { this.tag=tag; this.children=[]; this.parent=null; this._value=''; this._text=''; this.hidden=false; this.disabled=false; this.style={}; this.classList={toggle:(_,on)=>on}; }
  set id(value) { this._id=value; nodes.set(value,this); } get id() { return this._id; }
  set value(value) { this._value=String(value); } get value() { return this._value; }
  set textContent(value) { this._text=String(value); this.children=[]; } get textContent() { return this._text; }
  set innerHTML(value) { parse(value,this); }
  appendChild(element) { element.parent=this; this.children.push(element); return element; }
  append(...elements) { elements.forEach(element=>this.appendChild(element)); }
  replaceChildren(...elements) { this.children=[]; this.append(...elements); }
  get childNodes() { return this.children; }
  addEventListener() {} closest() { return this.parent||this; }
  querySelectorAll(selector) { if(selector==='.motion button') { this.motion ||= Array.from({length:4},()=>new Element('button')); return this.motion; } return []; }
}
function parse(text,parent) {
  for(const match of text.matchAll(/<(\w+)\b([^>]*\bid="([^"]+)"[^>]*)>/g)) {
    const element=new Element(match[1]); element.id=match[3]; element.value=(match[2].match(/\bvalue="([^"]*)"/)||[])[1]||'';
    element.parent=parent; if(parent) parent.children.push(element); element.hidden=/\bhidden\b/.test(match[2]);
  }
}
parse(html,null); nodes.get('sampleAxis').value='pan';
const captureButtons=Array.from({length:3},()=>new Element('button'));
async function fetch(url,options) {
  if(url==='/state') return {ok:true,json:async()=>structuredClone(state)};
  assert.equal(url,'/command');
  const actual=JSON.parse(options.body), fixture=records[consumed++];
  assert.ok(fixture,'unexpected additional HTTP command');
  assert.deepEqual(actual,fixture.request, 'actual page callback sent a different command to the Python HTTP API');
  state=fixture.state;
  return {ok:fixture.response.status===200,status:fixture.response.status,json:async()=>structuredClone(fixture.response.body)};
}
const context=vm.createContext({console,structuredClone,fetch,
  document:{getElementById:id=>{assert.ok(nodes.has(id),'missing DOM id '+id);return nodes.get(id);}, createElement:tag=>new Element(tag), activeElement:null, querySelectorAll:selector=>selector==='button.capture'?captureButtons:[]},
  window:{addEventListener(){}}, setInterval(){},setTimeout(){}});
vm.runInContext(html.split('<script>')[1].split('</script>')[0],context,{filename:'usb_gimbal_ui.html'});
async function main() {
  await new Promise(setImmediate);
  assert.equal(consumed,1,'initial page load may only send heartbeat');
  assert.ok(nodes.get('backend').textContent.includes('MOCK'));
  assert.ok(Object.values(state.gimbal.axes).every(axis=>!axis.armed));
  for(const axis of Object.values(state.gimbal.axes)) for(const key of ['draft_us','enabled','recorded_min_us','recorded_max_us','pending_min_us','pending_max_us','settled','fine_us','coarse_us','numeric_min_us','numeric_max_us','absolute_min_us','absolute_max_us']) assert.ok(key in axis.manual,'actual snapshot missing '+key);
  assert.equal(nodes.get('panDraft').min,'500'); assert.equal(nodes.get('panDraft').max,'2500');
  assert.equal(nodes.get('panExploreMin').min,'1'); assert.equal(nodes.get('panExploreMax').max,'19999');
  await context.jog('pan',1,false); assert.equal(state.gimbal.axes.pan.manual.draft_us,1505); assert.equal(state.gimbal.axes.pan.armed,false);
  await context.enableAxis('pan'); assert.equal(state.gimbal.axes.pan.output_us,1505); assert.equal(state.gimbal.axes.tilt.armed,false);
  await context.jog('pan',1,true); assert.equal(state.gimbal.axes.pan.output_us,1555);
  nodes.get('panFine').value=2; nodes.get('panCoarse').value=25; await context.saveSteps('pan'); assert.equal(state.gimbal.axes.pan.manual.fine_us,2); assert.equal(state.gimbal.axes.pan.armed,true);
  await context.markLimit('pan','min'); assert.equal(state.gimbal.axes.pan.manual.pending_min_us,1555);
  await context.jog('pan',1,true); await context.markLimit('pan','max'); assert.equal(state.gimbal.axes.pan.manual.recorded_max_us,1580); assert.ok(nodes.get('panMessage').textContent.includes('已保存'));
  await context.jog('pan',1,true); assert.equal(state.gimbal.axes.pan.manual.draft_us,1580); assert.ok(nodes.get('panMessage').textContent.includes('限制到'));
  vm.runInContext('angleDraft.tilt[1].angle_deg=1',context);
  await context.clearLimits('pan'); assert.equal(state.gimbal.axes.pan.armed,true); assert.equal(state.gimbal.axes.pan.manual.recorded_min_us,null);
  assert.equal(vm.runInContext('angleDraft.tilt[1].angle_deg',context),1,'clearing PAN must preserve the unsaved TILT angle draft');
  await context.disableAxis('pan'); await context.jog('pan',-1,false); assert.equal(state.gimbal.axes.pan.armed,false); assert.equal(state.gimbal.axes.pan.manual.draft_us,1578); assert.equal(state.gimbal.axes.pan.output_us,1580); assert.equal(nodes.get('panOutput').textContent,'—');
  nodes.get('panFine').value=5; nodes.get('panCoarse').value=50; await context.saveSteps('pan');
  nodes.get('panDraft').value=2500; await context.sendDraft('pan'); await context.enableAxis('pan');
  await context.enableAxis('tilt');
  const upperConfig=structuredClone(state.config.axes.tilt),upperOutput=state.gimbal.axes.tilt.output_us;
  const beforeExpand=state.gimbal.axes.pan.output_us;
  assert.equal(beforeExpand,2500); assert.equal(upperOutput,1500);
  nodes.get('panExploreMin').value=400; nodes.get('panExploreMax').value=2700;
  nodes.get('tiltExploreMin').value=700;
  vm.runInContext('rangeDirty.pan=true;rangeDirty.tilt=true',context);
  await context.fetchState();
  assert.equal(nodes.get('panExploreMin').value,'400'); assert.equal(nodes.get('panExploreMax').value,'2700');
  await context.expandRange('pan',1);
  assert.equal(state.gimbal.axes.pan.manual.numeric_max_us,2600);
  assert.equal(state.gimbal.axes.pan.output_us,beforeExpand,'expanding range must not move PWM');
  assert.equal(state.gimbal.axes.pan.armed,true); assert.equal(state.gimbal.axes.tilt.armed,true);
  assert.deepEqual(state.config.axes.tilt,upperConfig,'PAN expansion must preserve measured upper-axis configuration');
  assert.equal(state.gimbal.axes.tilt.output_us,upperOutput);
  assert.equal(nodes.get('panDraft').max,'2600');
  assert.equal(nodes.get('panRangeSummary').textContent,'500～2600µs');
  assert.equal(nodes.get('panExploreMax').value,'2700','polling must not overwrite dirty range edits');
  assert.equal(nodes.get('tiltExploreMin').value,'700','editing PAN must not overwrite another axis range draft');
  await context.jog('pan',1,false); assert.equal(state.gimbal.axes.pan.output_us,2505,'fine +5 must pass the old2500 limit');
  await context.saveRange('pan');
  assert.equal(state.gimbal.axes.pan.manual.numeric_min_us,400); assert.equal(state.gimbal.axes.pan.manual.numeric_max_us,2700);
  assert.equal(nodes.get('panDraft').min,'400'); assert.equal(nodes.get('panDraft').max,'2700');
  assert.equal(state.gimbal.axes.pan.output_us,2505); assert.deepEqual(state.config.axes.tilt,upperConfig);
  await context.expandRange('pan',-1); assert.equal(state.gimbal.axes.pan.manual.numeric_min_us,300);
  assert.equal(state.gimbal.axes.pan.output_us,2505);
  const upperMessage=nodes.get('tiltMessage').textContent;
  nodes.get('panDraft').value=20000; await context.sendDraft('pan'); assert.ok(nodes.get('panMessage').textContent.includes('19999')); assert.equal(nodes.get('tiltMessage').textContent,upperMessage);
  await context.capture('positive'); assert.equal(state.capture.active,true); assert.ok(captureButtons.every(button=>button.disabled)); await context.cancelCapture(); assert.equal(state.capture.active,false);
  context.showTab('tracking'); assert.equal(nodes.get('tabManual').hidden,true); assert.equal(nodes.get('tabTracking').hidden,false);
  context.demoTracking(); await context.saveTracking(); assert.equal(state.tracking_settings.normalized_gain_dps,50);
  await context.allOff(); assert.ok(Object.values(state.gimbal.axes).every(axis=>!axis.armed));
  assert.equal(consumed,fixtures.records.length,'not all actual HTTP fixtures were exercised');
  const manualResponses=consumed;
  state=fixtures.tracking_initial; records=fixtures.tracking_records; consumed=0;
  context.providedState=structuredClone(state);
  nodes.get('sampleAxis').value='pan';
  vm.runInContext('state=providedState;anglesLoaded=false;targetKey="";selectedKey="";renderState()',context);
  context.showTab('tracking');
  assert.ok(nodes.get('panTrackingStatus').textContent.includes('角度已标定'));
  assert.ok(nodes.get('panTrackingStatus').textContent.includes('PWM关闭'));
  assert.ok(nodes.get('tiltTrackingStatus').textContent.includes('PWM关闭'));
  assert.ok(nodes.get('trackingSelectionNote').textContent.includes('选择'));
  assert.equal(nodes.get('trackingManual').hidden,false);
  const beforeNavigate=consumed;
  context.showTab('manual');
  assert.equal(consumed,beforeNavigate,'returning to manual must not enable either axis or send a command');
  assert.ok(Object.values(state.gimbal.axes).every(axis=>!axis.armed));
  await context.command({action:'heartbeat'});
  await nodes.get('targets').children[0].onclick();
  assert.equal(state.target_selected,true);
  await context.track(true);
  assert.equal(state.tracking,false);
  assert.ok(nodes.get('trackingMessage').textContent.includes('PWM关闭'),'OFF start must explain how to enable PWM in Chinese');
  nodes.get('panDraft').value=340; await context.enableAxis('pan');
  nodes.get('tiltDraft').value=2043; await context.enableAxis('tilt');
  assert.ok(nodes.get('panTrackingStatus').textContent.includes('超出角度表'));
  await context.track(true); assert.equal(state.tracking,false);
  assert.ok(nodes.get('trackingMessage').textContent.includes('覆盖'));
  await context.disableAxis('pan');
  nodes.get('panDraft').value=1882; await context.sendDraft('pan'); await context.enableAxis('pan');
  assert.ok(nodes.get('panTrackingStatus').textContent.includes('当前命令在角度表内'));
  assert.equal(nodes.get('trackingManual').hidden,true);
  const beforeAngles=structuredClone(state);
  vm.runInContext('angleDraft.tilt[1].angle_deg=46',context);
  await context.saveAngles();
  assert.equal(records[consumed-1].response.status,200,'new per-axis angle API must accept the measured PAN table');
  assert.ok(Object.values(state.gimbal.axes).every(axis=>axis.armed),'saving PAN angles must keep both outputs ON');
  assert.deepEqual(state.config.axes.tilt,beforeAngles.config.axes.tilt,'saving PAN angles must preserve TILT calibration');
  assert.equal(state.gimbal.axes.pan.output_us,1882); assert.equal(state.gimbal.axes.tilt.output_us,2043);
  assert.equal(vm.runInContext('angleDraft.tilt[1].angle_deg',context),46,'saving PAN must not discard the unsaved TILT draft');
  assert.ok(nodes.get('anglesMessage').textContent.includes('PWM状态保持不变'));
  vm.runInContext('angleDraft.tilt[1].angle_deg=45',context);
  nodes.get('sampleAxis').value='tilt'; await context.saveAngles();
  assert.equal(records[consumed-1].response.status,200,'new per-axis angle API must accept the measured TILT table');
  assert.ok(Object.values(state.gimbal.axes).every(axis=>axis.armed),'saving TILT angles must keep both outputs ON');
  assert.deepEqual(state.config.axes.pan,beforeAngles.config.axes.pan);
  assert.equal(state.gimbal.axes.pan.output_us,1882); assert.equal(state.gimbal.axes.tilt.output_us,2043);
  assert.equal(state.target_selected,false);
  assert.ok(nodes.get('trackingSelectionNote').textContent.includes('选择'));
  await nodes.get('targets').children[0].onclick();
  assert.equal(state.target_selected,true);
  await context.track(true); assert.equal(state.tracking,true);
  assert.ok(nodes.get('trackingMessage').textContent.includes('已开始跟踪'));
  const outputBeforeCalibrationPage=Object.fromEntries(Object.entries(state.gimbal.axes).map(([axis,value])=>[axis,value.output_us]));
  const beforeCalibrationPage=consumed;
  await context.showTab('calibration');
  assert.equal(consumed,beforeCalibrationPage+1,'entering calibration while tracking sends exactly one pause command');
  assert.deepEqual(records[consumed-1].request,{action:'track',enabled:false});
  assert.equal(state.tracking,false); assert.ok(Object.values(state.gimbal.axes).every(axis=>axis.armed));
  assert.deepEqual(Object.fromEntries(Object.entries(state.gimbal.axes).map(([axis,value])=>[axis,value.output_us])),outputBeforeCalibrationPage);
  assert.equal(nodes.get('tabCalibration').hidden,false); assert.equal(nodes.get('calibrationOverlay').hidden,false);
  assert.ok(nodes.get('calibrationMessage').textContent.includes('PWM状态保持不变'));
  await context.allOff(); assert.ok(Object.values(state.gimbal.axes).every(axis=>!axis.armed));
  assert.equal(consumed,records.length,'not all actual tracking HTTP fixtures were exercised');
  const previousResponses=manualResponses+consumed;
  state=fixtures.calibration_initial; records=fixtures.calibration_records; consumed=0;
  context.providedState=structuredClone(state);
  vm.runInContext('state=providedState;calibrationTargetKey="";renderState()',context);
  assert.equal(nodes.get('uprightTilt').value,'180','this car defaults to upright at180deg');
  assert.ok(nodes.get('angleConvention').textContent.includes('180°正立'));
  assert.ok(nodes.get('angleConvention').textContent.includes('0°倒像'));
  context.showTab('calibration'); assert.equal(nodes.get('calibrationOverlay').hidden,false);
  context.showTab('tracking'); assert.equal(nodes.get('calibrationOverlay').hidden,true);
  context.showTab('calibration');
  const calibrationAxes=structuredClone(state.config.axes);
  await context.command({action:'heartbeat'});
  nodes.get('panDraft').value=1882; await context.enableAxis('pan');
  nodes.get('tiltDraft').value=2043; await context.enableAxis('tilt');
  for(const [axis,direction] of [['pan',1],['pan',-1],['tilt',1],['tilt',-1]]) {
    assert.ok(nodes.get('calibrationTargets').children[0].textContent.includes('记起点'));
    await nodes.get('calibrationTargets').children[0].onclick();
    assert.ok(state.image_calibration.pending,'begin must store the real target/PWM start snapshot');
    assert.ok(nodes.get('calibrationTargets').children[0].textContent.includes('记终点'));
    await context.calibrationJog(axis,direction,true);
    await nodes.get('calibrationTargets').children[0].onclick();
    assert.equal(state.image_calibration.pending,null);
  }
  assert.equal(state.image_calibration.sample_count,4);
  assert.equal(state.image_calibration.fit.ready,true,'both independent PWM axes must provide an applicable rank2 fit');
  const sampleOutput=Object.fromEntries(Object.entries(state.gimbal.axes).map(([axis,value])=>[axis,value.output_us]));
  await context.calibrationAction('calibration_apply');
  assert.equal(state.image_calibration.enabled,true);
  assert.equal(state.tracking,false,'applying an image model must not start tracking');
  assert.ok(Object.values(state.gimbal.axes).every(axis=>axis.armed));
  assert.deepEqual(state.config.axes,calibrationAxes,'image calibration must preserve angle/limit tables');
  assert.deepEqual(Object.fromEntries(Object.entries(state.gimbal.axes).map(([axis,value])=>[axis,value.output_us])),sampleOutput);
  assert.ok(nodes.get('trackingModelState').textContent.includes('图像采样模型'));
  await nodes.get('calibrationTargets').children[0].onclick();
  await context.calibrationAction('calibration_cancel'); assert.equal(state.image_calibration.pending,null);
  nodes.get('uprightTilt').value=0; vm.runInContext('mountDirty=true',context); await context.saveMount();
  assert.equal(state.config.mount.upright_tilt_deg,0);
  assert.ok(nodes.get('angleConvention').textContent.includes('0°正立'));
  nodes.get('uprightTilt').value=180; vm.runInContext('mountDirty=true',context); await context.saveMount();
  assert.equal(state.config.mount.upright_tilt_deg,180);
  assert.deepEqual(state.config.axes,calibrationAxes);
  assert.ok(Object.values(state.gimbal.axes).every(axis=>axis.armed));
  await context.calibrationAction('calibration_clear');
  assert.equal(state.image_calibration.sample_count,0); assert.equal(state.image_calibration.enabled,false);
  assert.ok(nodes.get('trackingModelState').textContent.includes('几何姿态模型'));
  await context.allOff(); assert.ok(Object.values(state.gimbal.axes).every(axis=>!axis.armed));
  assert.equal(consumed,records.length,'all image calibration commands must come from the actual UI callbacks');
  const expectedMemoryText={centering:'正在对准最后观测位置',waiting:'等待目标重现',long_wait:'等待目标重现',ambiguous:'歧义',reacquiring1:'1/3',reacquiring2:'2/3',resumed:'正在跟踪',paused:'已暂停',paused_after:'不会自动恢复'};
  for(const [name,memoryState] of Object.entries(fixtures.memory_states)) {
    context.providedState=structuredClone(memoryState);
    vm.runInContext('state=providedState;renderState()',context);
    const message=nodes.get('trackingSelectionNote').textContent,summary=nodes.get('trackingState').textContent;
    assert.ok(summary.includes(expectedMemoryText[name]),`${name}: ${summary}`);
    if(name==='waiting'||name==='long_wait') {
      assert.ok(message.includes('最后一次预测移动已结束'));
      assert.ok(!message.includes('已对准')&&!message.includes('已居中'),'a predicted or limited goal does not establish actual centering');
    }
    if(name==='ambiguous')assert.ok(summary.includes('暂不接管'),'ambiguity must not assert that an active prediction has already stopped');
    if(name.startsWith('paused'))assert.equal(memoryState.tracking,false);
    else {
      assert.equal(memoryState.tracking,true); assert.equal(memoryState.target_selected,true);
      assert.ok(!message.includes('请重新选择'),'temporary loss must not force target reselection');
    }
  }
  for(const [index,poseState] of fixtures.pose_states.entries()) {
    context.providedState=structuredClone(poseState);
    vm.runInContext('state=providedState;renderState()',context);
    const pose=poseState.tracking_pose,message=nodes.get('trackingPoseState').textContent;
    assert.ok(pose,'pose display must use the real backend snapshot contract');
    assert.ok(Math.abs(pose.tilt_deg-[45,90,175][index])<.2);
    assert.ok(message.includes(['倒像一侧','接近正上方','正立一侧'][index]));
    assert.ok(message.includes(index===1?'控制退化':'自动'));
    if(index===1)assert.equal(pose.near_vertical,true);
  }
  console.log(`PASS: actual UI callbacks + ${previousResponses+consumed} real Python HTTP responses and real memory/reacquisition/pose snapshots; manual control, TILT645..2420, burst, per-axis angles, calibration and mount, last-observation centering then waiting, ambiguous targets wait,3-frame reacquisition auto-resumes, manual pause never auto-resumes.`);
}
main().catch(error=>{console.error(error);process.exitCode=1;});
