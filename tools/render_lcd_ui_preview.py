#!/usr/bin/env python3
"""Draw R4's source-defined cards/bitmap glyphs without compiling FPGA RTL.

This is a layout preview, not an HDL simulator or a timing/resource check.
Requires Pillow; runtime state and the illustrative camera scene are examples.
"""
from pathlib import Path
import argparse
import re
from xml.sax.saxutils import escape

from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]


def rgb565(value):
    return ((value >> 11) * 255 // 31,
            ((value >> 5) & 63) * 255 // 63, (value & 31) * 255 // 31)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT.parent / 'deliverables/lcd-ui-r4.png')
    parser.add_argument('--svg', type=Path, default=ROOT / 'docs/ui_preview.svg')
    parser.add_argument('--zoom', action='store_true')
    args = parser.parse_args()
    source = (ROOT / 'src/lcd1024_interaction_ui.v').read_text(encoding='utf-8')
    colours = {name: rgb565(int(value, 16)) for name, value in
               re.findall(r'(\w+)\s*=16\'h([0-9a-f]+)', source)}
    glyphs = {char: int(value, 16) for char, value in
              re.findall(r'"(.)":glyph=40\'h([0-9a-f]+)', source)}
    image = Image.new('RGB', (1024, 600), colours['BACKGROUND'])
    draw = ImageDraw.Draw(image)
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1024" height="600" viewBox="0 0 1024 600">',
           '<title>R4 LCD UI layout preview; camera scene is illustrative.</title>']

    def rect(x0, y0, x1, y1, colour):
        colour = colours.get(colour, colour)
        draw.rectangle((x0, y0, x1-1, y1-1), fill=colour)
        svg.append(f'<rect x="{x0}" y="{y0}" width="{x1-x0}" height="{y1-y0}" fill="#{colour[0]:02x}{colour[1]:02x}{colour[2]:02x}"/>')

    rect(0, 0, 1024, 600, 'BACKGROUND')
    rect(0, 0, 1024, 44, 'WHITE')
    rect(0, 44, 1024, 45, 'BORDER')
    rect(0, 546, 1024, 547, 'BORDER')
    rect(12, 52, 820, 540, 'BORDER')
    # Illustrative scene, deliberately geometric; never a ROM or dataset image.
    for y in range(480):
        colour = (202-y//16, 214-y//20, 224-y//24)
        rect(16, y+56, 816, y+57, colour)
    scene = Image.new('RGB', (800, 480), (205, 217, 226))
    scene_draw = ImageDraw.Draw(scene)
    scene_draw.rectangle((0, 300, 799, 479), fill=(172, 187, 190))
    scene_draw.rectangle((280, 130, 520, 380), fill=(208, 165, 112))
    scene_draw.rectangle((315, 100, 365, 175), fill=(167, 121, 78))
    scene_draw.rectangle((435, 100, 485, 175), fill=(167, 121, 78))
    scene_draw.ellipse((330, 165, 470, 295), fill=(237, 215, 177))
    scene_draw.ellipse((355, 195, 370, 210), fill=(44, 52, 62))
    scene_draw.ellipse((430, 195, 445, 210), fill=(44, 52, 62))
    scene_draw.ellipse((385, 225, 415, 245), fill=(44, 52, 62))
    if args.zoom:
        scene = scene.crop((200, 120, 600, 360)).resize((800, 480), Image.Resampling.NEAREST)
    image.paste(scene, (16, 56))
    # SVG uses plain coloured rectangles; PNG has the more detailed scene.
    rect_colour = '#cdd9e2'
    svg.append(f'<rect x="16" y="56" width="800" height="480" fill="{rect_colour}"/>')
    sx, sy, sw, sh = ((176, 76, 480, 480) if args.zoom else (296, 186, 240, 250))
    # Keep SVG placeholder clipped to the unchanged video viewport.
    svg.append('<defs><clipPath id="video"><rect x="16" y="56" width="800" height="480"/></clipPath></defs>')
    svg.append(f'<rect x="{sx}" y="{sy}" width="{sw}" height="{sh}" fill="#d0a570" clip-path="url(#video)"/>')
    # Parse the actual solid-colour card rectangles from the RTL.
    card_pattern = r"if \(in_rect\(ax,11'd(\d+),11'd(\d+),ay,10'd(\d+),10'd(\d+)\)\) base_color=([^;]+);"
    for x0, x1, y0, y1, colour in re.findall(card_pattern, source):
        if int(x0) < 828:
            continue
        if '?' in colour:
            colour = 'BLUE'  # example: DEBUG ON
        rect(int(x0), int(y0), int(x1), int(y1), colour)

    rect(416, 284, 417, 309, 'BLUE')
    rect(404, 296, 429, 297, 'BLUE')
    half_width, half_height = (80, 60) if args.zoom else (40, 30)
    left, top, right, bottom = (416-half_width, 296-half_height,
                                416+half_width, 296+half_height)
    rect(left, top, left+1, bottom+1, 'ORANGE')
    rect(right, top, right+1, bottom+1, 'ORANGE')
    rect(left, top, right+1, top+1, 'ORANGE')
    rect(left, bottom, right+1, bottom+1, 'ORANGE')

    # Values below choose runtime text examples; positions and clipping are RTL.
    examples = {84: ('CAM DDR OK', 'GREEN'), 126: ('DEBUG ON', 'WHITE'),
                168: ('TOUCH OK', 'GREEN'), 192: ('FINGERS 2', 'MUTED'),
                242: ('GAUSS ON', 'BLUE'), 268: ('PHY INIT', 'GREEN'),
                314: ('ROI SET', 'BLUE'), 362: ('VIEW 2X' if args.zoom else 'VIEW 1X', 'BLUE'),
                506: ('NO MOTOR', 'MUTED'), 558: ('GAUSS 3X3', 'BLUE'),
                582: ('DOUBLE TAP ROI', 'MUTED')}
    label_pattern = (r"label_text=(.*?); label_x=11'd(\d+); label_y=10'd(\d+);\s*"
                     r"label_right=11'd(\d+); label_color=(.*?);")
    bounds = []
    for expr, x, y, right, colour in re.findall(label_pattern, source):
        x, y, right = int(x), int(y), int(right)
        if '?' in expr or '{' in expr:
            text, colour = examples[y]
        else:
            text = re.search(r'"([^"]*)"', expr).group(1).rstrip()
        assert x + len(text) * 12 <= right, (text, x, right)
        # Test all complete string alternatives, not only the rendered example.
        for alternative in re.findall(r'"([^"]*)"', expr):
            assert x + len(alternative.rstrip()) * 12 <= right, alternative
        bounds.append((x, y, x+len(text)*12, y+16))
        svg.append(f'<g aria-label="{escape(text)}">')
        for index, char in enumerate(text):
            bits = glyphs.get(char, 0)
            for col in range(5):
                for row in range(7):
                    if bits & (1 << (32-col*8+row)):
                        rect(x+index*12+col*2, y+row*2,
                             x+index*12+col*2+2, y+row*2+2, colour)
        svg.append('</g>')
    for i, a in enumerate(bounds):
        for b in bounds[i+1:]:
            assert not (a[0]<b[2] and b[0]<a[2] and a[1]<b[3] and b[1]<a[3]), (a, b)
    svg.append('</svg>')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.svg.parent.mkdir(parents=True, exist_ok=True)
    image.save(args.output)
    args.svg.write_text('\n'.join(svg)+'\n', encoding='utf-8')
    print(f'PASS: {len(bounds)} label slots fit and do not overlap. Wrote {args.output} and {args.svg}')


if __name__ == '__main__':
    main()
