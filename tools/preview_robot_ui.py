#!/usr/bin/env python3
"""Offline interactive hierarchical LCD preview from the shared UI spec.

Example data is explicitly labelled. This does not contact hardware, send
commands, compile HDL, or reproduce physical LCD/camera timing.
"""
from pathlib import Path
import argparse
import html
import json
import re

from robot_ui_spec import (ROOT, VIEWS, SIDEBAR, button_rects, button_label,
                           caption_for, route_for, DASHBOARD_ROWS)

def _rgb565(value):
    return '#%02x%02x%02x' % ((value >> 11) * 255 // 31,
        ((value >> 5) & 63) * 255 // 63, (value & 31) * 255 // 31)


_palette_source = (ROOT / 'src/robot_control_panel.v').read_text(encoding='utf-8-sig')
COLORS = {name: _rgb565(int(value, 16)) for name, value in
          re.findall(r"(\w+)=16'h([0-9a-fA-F]+)", _palette_source)}
COLORS['BORDER'] = _rgb565(0xd6bb)


def font():
    lcd = (ROOT / 'src/lcd1024_interaction_ui.v').read_text(encoding='utf-8-sig')
    return {character: int(bits, 16) for character, bits in
            re.findall(r'"(.)":glyph=40\x27h([0-9a-fA-F]+);', lcd)}


def sample_rows(key):
    # Layout fixtures only. Hyphens denote no measurement in this preview.
    if key == (2, 0):
        return ['RX BYTES ------', 'VALID NODES ----', 'VALID PTS ------',
                'UART ERR -------', 'PARSE ERR ------', 'RESTARTS -------']
    if key == (2, 1):
        return ['BACK UNKNOWN', 'FRONT UNKNOWN', 'LEFT UNKNOWN',
                'RIGHT UNKNOWN', 'SELF -----------', 'SCANS ----------']
    if key == (1, 1):
        return ['WHEEL LEFT', 'L COUNT --------', 'R COUNT --------',
                'ENC SIGN NORMAL', 'MOTOR SIGN NORM', 'TEST DISARMED']
    if key == (4, 0):
        return ['LEFT SET +0010', 'RIGHT SET +0010', 'LEFT SPD -------',
                'RIGHT SPD ------', 'YAW ------------', 'DRIVE DISARMED']
    if key == (4, 1):
        return ['X 0' , 'Y 0', 'YAW 0.000',
                'GOAL X 300', 'GOAL Y 0', 'STOPPED']
    if key == (4, 2):
        return ['LEFT SET +0010', 'RIGHT SET +0010', 'STEP 0001',
                'LINK L/R OFF', 'LEFT SPD -------', 'RIGHT SPD ------']
    if key == (4, 3):
        return ['NAV CRUISE RATE', 'VALUE ----------', 'STEP -----------',
                'CRUISE SET 0010', 'TURN SET 0006', 'COUNTS PER 10MS']
    if key in ((5,0),(5,3)):
        return ['SELID ---------', 'DOG TRACKING', 'DOG CONF 90' if key==(5,0) else 'GAIN 20',
                'MAX 5', 'PANUS ----------', 'TILUS ----------']
    if key == (5,1):
        return ['ID -------------', 'CONF% ----------', 'DOG TARGET',
                'SELID ----------', 'SELECT TO TRACK', 'EXAMPLE ONLY']
    if key == (2,3):
        return ['FRONT MM -------','LIDAR ANGLE 90','BLOCK X 0','BLOCK Y 0','SELF 0','SCANS 0']
    if key == (3,3):
        return ['GOAL X 300','GOAL Y 0','STEP 100','X FRONT Y LEFT','MM / DEGREES','']
    if key == (5,2):
        return ['GAIN X100', 'VALUE 20', 'STEP 1', '', '', '']
    if key[0] == 3:
        return ['SELECTED FIELD', 'VALUE ----------', 'STEP -----------',
                'X --------------', 'Y --------------', 'YAW ------------']
    return ['SELECTED FIELD', 'VALUE ----------', 'STEP -----------',
            'MEASUREMENT ----', 'MEASUREMENT ----', 'LOCAL DISARMED']


def car_cell_color(dx,dy,direction,front=4,rear=2,left=2,right=2):
    """Illustrative display model only, not collision geometry."""
    u,v=((dx,dy),(dx+dy,dy-dx),(dy,-dx),(dy-dx,-dx-dy),
         (-dx,-dy),(-dx-dy,dx-dy),(-dy,dx),(dx-dy,dx+dy))[direction]
    if direction&1:
        front+=front//2;rear+=rear//2;left+=left//2;right+=right//2
    inside=-rear<=u<=front and -right<=v<=left
    color='BLUE' if inside and (u>=front-1 or u<=-rear+1 or v>=left-1 or v<=-right+1) else None
    if front-1<=u<=front+2 and u-front-2<=v<=front+2-u:color='#ffff00'
    if dx==dy==0:color='RED'
    return color


def scene(key):
    view = VIEWS[key]
    items = [('rect', (0, 0, 1024, 600), 'BG'),
             ('rect', (0, 0, 1024, 44), 'WHITE'),
             ('rect', (0, 44, 1024, 45), 'BORDER'),
             ('rect', (0, 546, 1024, 547), 'BORDER')]
    if view.kind == 'camera':
        items += [('rect', (16, 56, 816, 536), '#243846'),
                  ('text', (260, 280), 'CAMERA EXAMPLE', 'WHITE')]
    elif view.kind == 'leaf':
        items.append(('rect', (28, 116, 388, 500 if key == (2, 0) else 416), 'WHITE'))
    elif view.kind == 'map':
        items += [('rect', (28, 104, 412, 488), '#c6c7c6'),
                  ('text', (48, 128), 'MAP EXAMPLE', 'MUTED')]
        # A clearly labelled example pose. Uses the actual cell size and the
        # same eight-sector visual model; not a captured position/map.
        for gy in range(128):
            for gx in range(128):
                color=car_cell_color(gx-64,gy-64,1)
                if color:
                    items.append(('rect',(28+gx*3,104+(127-gy)*3,31+gx*3,107+(127-gy)*3),color))
    elif view.kind == 'dashboard':
        items += [('rect', (32, 112, 384, 444), 'WHITE'),
                  ('rect', (432, 112, 784, 444), 'WHITE')]
    items.append(('rect', (824, 52, 1016, 540), 'WHITE'))
    for button, rect in button_rects(*key).items():
        stop = button == 106 or (
            key in ((4, 0), (4, 1)) and button == 6) or (
            key in ((1, 2), (1, 3)) and button == 2) or (
            key == (5,0) and button in (3,7)) or (key==(5,3) and button in (4,7)) or (key==(2,3) and button==2) or (key == (5,1) and button == 4)
        color = 'RED' if stop else 'BLUE' if button == 100 else 'SOFT'
        items.append(('rect', rect, color))
        if view.kind == 'map' and button in (1, 3):
            items.append(('rect', (619, rect[1], 620, rect[3]), 'WHITE'))
        x, y, right = button_label(rect, view.kind, button)
        text = caption_for(*key, button)
        assert x + len(text) * 12 <= right, (key, button, text)
        items.append(('text', (x, y), text, 'WHITE' if stop or button == 100 else 'BLUE'))
    if view.kind == 'menu':
        items += [('text', (32, 88), view.hint, 'INK'),
                  ('text', (432, 88), view.detail, 'MUTED')]
    elif view.kind == 'leaf':
        items += [('text', (36, 132 + index * 32), text, 'INK')
                  for index, text in enumerate(sample_rows(key))]
        if key == (2, 0):
            items += [('text', (36, 332 + index*32), label+' --------', 'MUTED')
                      for index,label in enumerate(('H0','H1','S0','S1','TX'))]
        else:
            items += [('text', (36, 376), view.hint, 'MUTED'),
                      ('text', (36, 400), view.detail, 'MUTED')]
    elif view.kind == 'map':
        items += [('text', (448, 80 + index * 32), text, 'INK')
                  for index, text in enumerate(sample_rows(key))]
        items.append(('text', (28, 504), '8.192M / 64MM', 'MUTED'))
        items += [('text', (448, 272), 'MAPPT ----------', 'INK'),
                  ('text', (448, 304), 'DROP -----------', 'INK')]
    elif view.kind == 'dashboard':
        items += [('text', (44, 80), 'LEFT WHEEL', 'INK'),
                  ('text', (444, 80), 'RIGHT WHEEL', 'INK')]
        example = ('TARGET +10' if key == (1, 2) else 'TARGET 0',
                   'SPEED --------' if key == (1, 2) else 'COUNT --------',
                   'KP 30.0000' if key == (1,2) else 'KP 0.0000',
                   'KI 1.0000' if key == (1,2) else 'KI 0.0000', 'KD 0.0000')
        for index, (y, _, _) in enumerate(DASHBOARD_ROWS):
            color = 'GREEN' if index == 1 else 'INK'
            items += [('text', (44, y), example[index], color),
                      ('text', (444, y), example[index], color)]
        items += [('text', (44, 428), 'L PWM ----------', 'MUTED'),
                  ('text', (444, 428), 'R PWM ----------', 'MUTED')]
    sidebar_rows = ('STOPPED', 'COUNTS / 10MS' if key == (1, 2) else 'ENCODER COUNTS',
                    'VOFA PID EDIT', 'READ ONLY LCD') if key in ((1, 2), (1, 3)) else (
        'LOCAL DISARMED', 'RX --------', 'C1 REQ OFF', 'ST --------') if key == (2, 0) else (
        'STOPPED', 'GAINS READY', 'C1 LIVE', 'MM / DEGREES')
    items += [('text', (824, 224 + index * 32), text, 'INK')
              for index, text in enumerate(sidebar_rows)]
    items += [
              ('text', (20, 14), view.title, 'INK'),
              ('text', (300, 14), 'A BQ WAIT', 'INK'),
              ('text', (600, 14), 'B BQ WAIT', 'INK')]
    for y, names in ((558, ('GX', 'GY', 'GZ')), (582, ('AX', 'AY', 'AZ'))):
        items += [('text', (x, y), name + ' ----------', 'MUTED')
                  for x, name in zip((20, 300, 600), names)]
    return items


def _color(value):
    return COLORS.get(value, value)


def glyph_symbols(glyphs):
    result = ['<svg width="0" height="0" aria-hidden="true"><defs>']
    for character, bits in glyphs.items():
        result.append(f'<symbol id="g{ord(character)}" viewBox="0 0 5 7">')
        for column in range(5):
            for row in range(7):
                if bits & (1 << (32 - column * 8 + row)):
                    result.append(f'<rect x="{column}" y="{row}" width="1" height="1"/>')
        result.append('</symbol>')
    return '\n'.join(result + ['</defs></svg>'])


def render_svg(key, glyphs):
    items = scene(key)
    result = ['<svg class="lcd" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1024 600" role="img" '
              f'aria-label="{html.escape(VIEWS[key].title)} layout example">']
    for kind, geometry, *values in items:
        if kind == 'rect':
            left, top, right, bottom = geometry
            result.append(f'<rect x="{left}" y="{top}" width="{right-left}" height="{bottom-top}" fill="{_color(values[0])}"/>')
        else:
            x, y = geometry
            text, color = values
            result.append(f'<g aria-label="{html.escape(text)}" fill="{_color(color)}" pointer-events="none">')
            for column, character in enumerate(text[:16]):
                if character in glyphs:
                    result.append(f'<use href="#g{ord(character)}" x="{x+column*12}" y="{y}" width="10" height="14"/>')
            result.append('</g>')
    for button, (left, top, right, bottom) in button_rects(*key).items():
        result.append(f'<rect class="hit" tabindex="0" role="button" data-button="{button}" '
                      f'aria-label="{html.escape(caption_for(*key, button))}" '
                      f'x="{left}" y="{top}" width="{right-left}" height="{bottom-top}"/>')
    return '\n'.join(result + ['</svg>'])


def write_preview(output):
    glyphs = font()
    routes = {}
    for key in VIEWS:
        for button in button_rects(*key):
            target = route_for(key, button)
            if target is not None:
                routes[f'{key[0]}/{key[1]}:{button}'] = list(target)
    options = ''.join(f'<option value="{page}/{sub}">{page}/{sub} — {html.escape(view.title)}</option>'
                      for (page, sub), view in VIEWS.items())
    views = '\n'.join(f'<section data-view="{page}/{sub}" hidden>{render_svg((page, sub), glyphs)}</section>'
                      for page, sub in VIEWS)
    document = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot LCD hierarchical layout preview</title>
<style>body{margin:0;background:#14232d;color:#eef6fa;font:16px system-ui,sans-serif}
main{max-width:1180px;margin:20px auto;padding:0 16px}.warning{background:#681b2e;border:2px solid #ff7c96;padding:14px;border-radius:8px;font-weight:700}
.toolbar{display:flex;gap:16px;align-items:center;margin:18px 0;flex-wrap:wrap}select{font:inherit;padding:8px;background:#fff;color:#132c40}
.lcd{display:block;width:100%;background:#f7f7ef;border:1px solid #617b89;image-rendering:pixelated}
.hit{fill:transparent;cursor:pointer}.hit:hover,.hit:focus{fill:#529cff;fill-opacity:.18;stroke:#004fbb;stroke-width:2;outline:none}
#log{min-height:28px;color:#a8dcff}.foot{font-size:14px;color:#b7c8d1;line-height:1.6}
</style><main><div class="warning">离线布局预览：全部数字与状态均为示例，绝非实测。点击仅切换预览，不连接或控制小车。</div>
<div class="toolbar"><label>查看页面 <select id="picker">@OPTIONS@</select></label><span>主页面 / 子页面；触控区与 RTL 共用同一份声明。</span></div>
@GLYPHS@@VIEWS@<p id="log" aria-live="polite">可点击 HOME、BACK 和菜单卡查看分层流程。</p>
<p class="foot">保留 1024×600 几何、实际 RTL 点阵字形、地图区域及大触控矩形。相机与地图均是布局占位示例。此页面不模拟控制算法、HDL 时钟、传感器或硬件时序。</p>
</main><script>
const routes=@ROUTES@;let current='0/0';
const picker=document.getElementById('picker'),log=document.getElementById('log');
function show(key){document.querySelectorAll('section[data-view]').forEach(e=>e.hidden=e.dataset.view!==key);picker.value=key;current=key}
function action(button,label){const old=current;let target=routes[current+':'+button];
if(target)show(target.join('/'));log.textContent='仅预览：'+old+' → '+label+(target?' → '+target.join('/'):'（示例操作，不向硬件发送命令）')}
picker.addEventListener('change',()=>show(picker.value));
document.querySelectorAll('.hit').forEach(e=>{const run=()=>action(Number(e.dataset.button),e.getAttribute('aria-label'));
e.addEventListener('click',run);e.addEventListener('keydown',event=>{if(event.key==='Enter'||event.key===' '){event.preventDefault();run()}})});show(current);
</script></html>'''
    document = document.replace('@OPTIONS@', options).replace('@GLYPHS@', glyph_symbols(glyphs)).replace(
        '@VIEWS@', views).replace('@ROUTES@', json.dumps(routes, separators=(',', ':')))
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(document, encoding='utf-8')
    print(f'Wrote offline example-only preview: {output} ({len(VIEWS)} views)')


def write_png(output, key=(0, 0)):
    from PIL import Image, ImageDraw
    image = Image.new('RGB', (1024, 640), '#14232d')
    draw = ImageDraw.Draw(image)
    draw.text((12, 12), 'LAYOUT EXAMPLE ONLY - NOT HARDWARE DATA', fill='#ffb7c6')
    glyphs = font()
    for kind, geometry, *values in scene(key):
        if kind == 'rect':
            left, top, right, bottom = geometry
            draw.rectangle((left, top+40, right-1, bottom+39), fill=_color(values[0]))
        else:
            x, y = geometry
            text, color = values
            for index, character in enumerate(text[:16]):
                bits = glyphs.get(character, 0)
                for column in range(5):
                    for row in range(7):
                        if bits & (1 << (32-column*8+row)):
                            left, top = x+index*12+column*2, y+40+row*2
                            draw.rectangle((left, top, left+1, top+1), fill=_color(color))
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)
    print(f'Wrote illustrative layout PNG: {output}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--html', type=Path, default=ROOT.parent / 'deliverables/robot-ui-hierarchical.html')
    parser.add_argument('--png', type=Path)
    parser.add_argument('--view', default='0/0')
    args = parser.parse_args()
    write_preview(args.html)
    if args.png:
        write_png(args.png, tuple(map(int, args.view.split('/'))))
