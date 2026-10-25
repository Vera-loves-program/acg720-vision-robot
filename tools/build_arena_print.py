"""Build the full-size, map-only 4 m floor print. No HDL tools are used.

The PDF page itself is exactly 4000 x 4000 mm. All coordinates below are
physical millimetres from its lower-left corner, preserving planning sec. 37.
"""

from pathlib import Path

from pypdf import PdfReader
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "output" / "pdf" / "arena_4x4_print.pdf"
SIDE = 4000
FONT = "ArenaChinese"

# Letter, name, x0, y0, x1, y1, print CMYK, optional exact target centre.
ZONES = [
    ("A", "待机 / 补给", 100, 100, 1200, 1200, (.12, 0, .08, 0), (650, 650)),
    ("B", "学习提醒", 100, 1450, 1200, 2550, (.12, .055, 0, 0), (650, 2000)),
    ("C", "阅读陪伴", 100, 2800, 1200, 3900, (.065, .11, 0, 0), (650, 3350)),
    ("D", "资料领取", 1450, 2800, 2550, 3900, (0, .055, .17, 0), (2000, 3350)),
    ("E", "目标搜索", 2800, 2800, 3900, 3900, (0, .11, .09, 0), (3350, 3350)),
    ("F", "休息互动", 2800, 1450, 3900, 2550, (.025, .11, .015, 0), (3350, 2000)),
    ("G", "物品送达", 2800, 100, 3900, 1200, (.12, .015, .035, 0), (3350, 650)),
    ("H", "动态绕障", 1450, 100, 2550, 1200, (0, .08, .15, 0), None),
]
BOXES = [(1400, 1600), (1800, 1600), (2200, 1600), (2200, 2000)]


def fill_grey(c, black):
    c.setFillColorCMYK(0, 0, 0, black)


def stroke_grey(c, black):
    c.setStrokeColorCMYK(0, 0, 0, black)


def centred(c, text, x, y, size):
    """Text position and size are in physical mm, including the baseline."""
    c.setFont(FONT, size * mm)
    c.drawCentredString(x * mm, y * mm, text)


def build():
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    pdfmetrics.registerFont(TTFont(FONT, r"C:\Windows\Fonts\simhei.ttf"))
    c = canvas.Canvas(
        str(OUTPUT), pagesize=(SIDE * mm, SIDE * mm),
        pageCompression=1, invariant=1, pdfVersion=(1, 4),
    )
    c.setTitle("4 x 4 m robot arena - full-size floor print")
    c.setAuthor("ACG720 robot project")
    c.setSubject("4000 x 4000 mm, 1:1 print; dimensions in metres on edge rulers")

    # White background and pastel, position-preserving semantic zones.
    fill_grey(c, 0)
    c.rect(0, 0, SIDE * mm, SIDE * mm, fill=1, stroke=0)
    for _, _, x0, y0, x1, y1, colour, _ in ZONES:
        c.setFillColorCMYK(*colour)
        c.roundRect(x0 * mm, y0 * mm, (x1 - x0) * mm,
                    (y1 - y0) * mm, 45 * mm, fill=1, stroke=0)

    # 10 cm fine grid, 50 cm medium grid and 1 m stronger grid. The 85 mm
    # peripheral strip keeps the measuring labels clear of grid intersections.
    for coordinate in range(100, SIDE, 100):
        major = coordinate % 1000 == 0
        half = coordinate % 500 == 0
        stroke_grey(c, .17 if major else .11 if half else .04)
        c.setLineWidth((.8 if major else .6 if half else .3) * mm)
        c.line(coordinate * mm, 85 * mm, coordinate * mm, 3915 * mm)
        c.line(85 * mm, coordinate * mm, 3915 * mm, coordinate * mm)

    # Quiet zone boundaries are legible in a grayscale print as well.
    stroke_grey(c, .22)
    c.setLineWidth(.9 * mm)
    for _, _, x0, y0, x1, y1, _, _ in ZONES:
        c.roundRect(x0 * mm, y0 * mm, (x1 - x0) * mm,
                    (y1 - y0) * mm, 45 * mm, fill=0, stroke=1)

    # The only object footprints are the fixed 400 mm boxes: thin dashed
    # placement guides, not filled obstacles and not an illustrated route.
    stroke_grey(c, .36)
    c.setLineWidth(1.25 * mm)
    c.setDash(13 * mm, 10 * mm)
    for x, y in BOXES:
        c.rect(x * mm, y * mm, 400 * mm, 400 * mm, fill=0, stroke=1)
    c.setDash()

    # Plain letter and name labels. No title, legend, arrows, routes, prose,
    # vehicle drawing, coordinate callouts, or temporary obstacle is printed.
    fill_grey(c, .83)
    for letter, name, x0, y0, x1, y1, _, target in ZONES:
        x_mid = (x0 + x1) / 2
        centred(c, letter, x_mid, y1 - 180, 112)
        centred(c, name, x_mid, y1 - 285, 58)
        if target:
            x, y = target
            stroke_grey(c, .48)
            c.setLineWidth(1.2 * mm)
            c.circle(x * mm, y * mm, 11 * mm, fill=0, stroke=1)
            c.line((x - 20) * mm, y * mm, (x + 20) * mm, y * mm)
            c.line(x * mm, (y - 20) * mm, x * mm, (y + 20) * mm)

    # Rulers start at the exact PDF page edges, in the original Cartesian
    # convention: x increases rightwards, y upwards. Top/right repeat values.
    stroke_grey(c, .72)
    for value in range(0, SIDE + 1, 100):
        major = value % 500 == 0
        length = 40 if major else 21
        c.setLineWidth((1.4 if major else .85) * mm)
        c.line(value * mm, 0, value * mm, length * mm)
        c.line(value * mm, SIDE * mm, value * mm, (SIDE - length) * mm)
        c.line(0, value * mm, length * mm, value * mm)
        c.line(SIDE * mm, value * mm, (SIDE - length) * mm, value * mm)

    fill_grey(c, .78)
    # End values share corners with the perpendicular ruler; use one exact
    # coordinate pair at each corner rather than overlapping rotated digits.
    for value in range(500, SIDE, 500):
        label = f"{value / 1000:g}"
        position = value
        centred(c, label, position, 59, 28)
        centred(c, label, position, SIDE - 76, 28)
        c.saveState()
        c.translate(63 * mm, position * mm)
        c.rotate(90)
        centred(c, label, 0, 0, 28)
        c.restoreState()
        c.saveState()
        c.translate((SIDE - 63) * mm, position * mm)
        c.rotate(-90)
        centred(c, label, 0, 0, 28)
        c.restoreState()

    for x, y, label in [
        (50, 45, "(0,0)"), (3950, 45, "(4,0)"),
        (50, 3945, "(0,4)"), (3950, 3945, "(4,4)"),
    ]:
        centred(c, label, x, y, 27)

    # Units and origin direction fit inside the empty gaps along the edges.
    centred(c, "X / m", 3250, 59, 24)
    centred(c, "X / m", 3250, SIDE - 76, 24)
    c.saveState()
    c.translate(63 * mm, 3250 * mm)
    c.rotate(90)
    centred(c, "Y / m", 0, 0, 24)
    c.restoreState()
    c.saveState()
    c.translate((SIDE - 63) * mm, 3250 * mm)
    c.rotate(-90)
    centred(c, "Y / m", 0, 0, 24)
    c.restoreState()

    # Boundary centreline is the actual 0..4 m boundary. Its outer half is
    # clipped at the finished page edge; no unnoticed internal margin exists.
    stroke_grey(c, .72)
    c.setLineWidth(3 * mm)
    c.rect(0, 0, SIDE * mm, SIDE * mm, fill=0, stroke=1)
    c.showPage()
    c.save()

    reader = PdfReader(OUTPUT)
    assert len(reader.pages) == 1
    page = reader.pages[0]
    for actual in [float(page.mediabox.width), float(page.mediabox.height)]:
        assert abs(actual / mm - SIDE) < .001
    fonts = page["/Resources"]["/Font"].get_object()
    embedded = []
    for reference in fonts.values():
        font = reference.get_object()
        descriptor = font.get("/FontDescriptor")
        if descriptor:
            descriptor = descriptor.get_object()
            embedded.append("/FontFile2" in descriptor or "/FontFile3" in descriptor)
    assert embedded and all(embedded), "Chinese font must be embedded"
    text = page.extract_text()
    for zone in ZONES:
        assert zone[1] in text
    assert not page.images, "The delivery must remain a vector map"
    print(f"Built {OUTPUT}")
    print("Verified: one page; 4000 x 4000 mm; embedded Chinese font; eight labels; vector artwork.")


if __name__ == "__main__":
    build()
