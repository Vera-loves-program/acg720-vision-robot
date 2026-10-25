"""Physical map-window and stable-ID regressions, no hardware or HDL simulator.

A 6 m radius empty room used to discard EVERY measured ray. Check that the
same measured scan now provides a known-free route, without invented walls
on the map border or permission to traverse unobserved space.
"""
from pathlib import Path
from math import sin, cos, radians, floor
import random
from check_lidar_path_math import inflate_known_free, bfs

ROOT = Path(__file__).resolve().parents[1]


def ray(start, end):
    x, y = start
    hx, hy = end
    dx, dy = abs(hx-x), abs(hy-y)
    sx, sy = 1 if hx >= x else -1, 1 if hy >= y else -1
    error = dx-dy
    while True:
        yield x, y
        if (x, y) == (hx, hy):
            return
        twice = 2*error
        if twice > -dy:
            x += sx
            error -= dy
        if twice < dx:
            y += sy
            error += dx


def inside(p):
    return all(0 <= v < 128 for v in p)


def clip(start, end):
    if not inside(start):
        return None, False
    if inside(end):
        return end, True
    last = start
    for p in ray(start, end):
        if not inside(p):
            return last, False
        last = p
    raise AssertionError('outside endpoint must cross the boundary')


def scan_grid(keep_far):
    cells = [[0]*128 for _ in range(128)]
    sensor = floor((33+4096)/64), floor((-77+4096)/64)
    hits = 0
    for i in range(720):
        a = radians(i/2)
        end = floor((33+6000*cos(a)+4096)/64), floor((-77+6000*sin(a)+4096)/64)
        if not keep_far and not inside(end):
            continue
        end, hit = clip(sensor, end)
        for x, y in ray(sensor, end):
            cells[y][x] = 3 if hit and (x, y) == end else 1
        hits += hit
    return cells, hits


def main():
    for end, expected in (((200,64),(127,64)), ((-100,64),(0,64)),
                          ((64,200),(64,127)), ((64,-100),(64,0)),
                          ((200,200),(127,127)), ((127,127),(127,127))):
        clipped, hit = clip((64,64), end)
        assert clipped == expected and hit == inside(end)
    assert clip((-1,64),(50,64)) == (None,False)
    # A near wall stays occupied; a far return must not create an edge wall.
    assert clip((64,64),(80,64)) == ((80,64), True)
    assert clip((64,64),(128,64)) == ((127,64), False)
    rng = random.Random(720)
    for _ in range(10000):
        start = rng.randrange(128), rng.randrange(128)
        end = start[0]+rng.randrange(-251,252), start[1]+rng.randrange(-251,252)
        clipped, hit = clip(start,end)
        assert inside(clipped) and hit == inside(end)
        # Original grid ray is clipped at its FIRST exit; independent x/y
        # saturation would produce the wrong angle for oblique far returns.
        assert clipped in list(ray(start,end))
        assert len(list(ray(start,clipped))) <= 128
    old, old_hits = scan_grid(False)
    new, new_hits = scan_grid(True)
    assert not any(any(row) for row in old) and old_hits == new_hits == 0
    assert bfs(inflate_known_free(old),(32,32),(38,32))[1] == 2
    path, error = bfs(inflate_known_free(new),(32,32),(38,32))
    assert error == 0 and path[0] == (32,32) and path[-1] == (38,32)
    assert not any(v >= 2 for row in new for v in row)
    unknown = [[0]*128 for _ in range(128)]
    assert bfs(inflate_known_free(unknown),(32,32),(38,32))[1] == 2
    source = (ROOT/'src/lidar_grid_map.v').read_text(encoding='utf-8')
    for token in ('POINT_BITS = 4*MAP_BITS+1', 'F_CLIP_INIT', 'F_CLIP:',
                  'if(ray_endpoint && ray_hit)', 'if(ray_hit && candidate_in_bounds',
                  '{ray_hit,sx,sy,hx,hy}<=ray_rdata'):
        assert token in source, token
    print('PASS: 10,000 clipped rays, real-hit/edge-free distinction, outside sensor rejection')
    print('PASS: SAME 720-ray 6m room scan: old map empty/start rejected; new known-free 768mm route, no false border walls')
    print('Independent geometry/source regressions only, not HDL simulation or timing closure.')


if __name__ == '__main__':
    main()
