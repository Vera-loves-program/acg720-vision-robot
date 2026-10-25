"""Independent occupancy inflation/BFS checks, not HDL compilation/simulation."""
from collections import deque
from pathlib import Path
import math

SIDE=128
RADIUS=5

def circle_kernel(radius_mm):
    """All fine rectangles intersecting coarse square[0,128]^2⊕disk(R)."""
    radius=(radius_mm+63)//64
    result=[]
    for y in range(-radius,radius+2):
        for x in range(-radius,radius+2):
            # Reference geometry works in millimetres, independently of
            # the RTL's integer-cell separation and serial-square setup.
            dx=max(0,-(x+1)*64,x*64-128)
            dy=max(0,-(y+1)*64,y*64-128)
            if dx*dx+dy*dy<=radius_mm*radius_mm:
                result.append((x,y))
    return result

def inflate_known_free(fine,radius=RADIUS,radius_mm=None,*,centered=True):
    """Reference exact circle/coarse-square Minkowski inflation."""
    if radius_mm is None:radius_mm=291 if radius==RADIUS else radius*64
    kernel=circle_kernel(radius_mm)
    coarse=set()
    shift=-1 if centered else 0
    for y in range(64):
        for x in range(64):
            if all(0<=2*x+shift+dx<SIDE and 0<=2*y+shift+dy<SIDE and fine[2*y+shift+dy][2*x+shift+dx]==1
                   for dx,dy in kernel):
                coarse.add((x,y))
    return coarse

def check_circle_geometry():
    # Reproduce only RTL's eleven-cycle shift/add and integer mask; compare
    # it against an independent millimetre rectangle-distance reference.
    for radius_mm in range(1,1985):
        acc=0;shift=radius_mm;multiplier=radius_mm
        for _ in range(11):
            acc+=shift if multiplier&1 else 0
            shift=(shift<<1)&((1<<22)-1);multiplier>>=1
        assert acc==radius_mm*radius_mm
        r=(radius_mm+63)//64
        # Full kernels on every radius around each64mm transition plus
        # the measured291mm; scalar square arithmetic is exhaustive above.
        if radius_mm%64 not in (0,1,63) and radius_mm!=291:continue
        rtl=set()
        for ky in range(2*r+2):
            for kx in range(2*r+2):
                gx=r-kx-1 if kx<r else kx-r-2 if kx>r+2 else 0
                gy=r-ky-1 if ky<r else ky-r-2 if ky>r+2 else 0
                if gx*gx+gy*gy<=acc>>12:rtl.add((kx-r,ky-r))
        assert rtl==set(circle_kernel(radius_mm)),radius_mm
    assert len(circle_kernel(291))==132
    assert len(circle_kernel(320))==140  # exact boundary contact is retained
    # A distant diagonal unknown is outside the swept circle and may be
    # skipped; every intersecting unknown/hit continues blocking the node.
    fine=[[1]*SIDE for _ in range(SIDE)]
    fine[58][58]=0
    assert (32,32) in inflate_known_free(fine)
    fine[64][58]=0
    assert (32,32) not in inflate_known_free(fine)
    fine[64][58]=3
    assert (32,32) not in inflate_known_free(fine)

def bfs(coarse,start,goal):
    if start not in coarse:return None,2
    if goal not in coarse:return None,3
    parent={start:None}
    queue=deque([start])
    while queue:
        p=queue.popleft()
        if p==goal:
            path=[]
            while p is not None:
                path.append(p)
                p=parent[p]
            return path[::-1],0
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            q=p[0]+dx,p[1]+dy
            if q in coarse and q not in parent:
                parent[q]=p
                queue.append(q)
    return None,4

def first_turn(path):
    """Reverse traversal identical in meaning to the hardware parent walk."""
    waypoint=path[-1]
    previous=None
    for child,parent in zip(reversed(path[1:]),reversed(path[:-1])):
        d=parent[0]-child[0],parent[1]-child[1]
        if previous is not None and d!=previous:
            waypoint=child
        previous=d
    return waypoint

def main():
    check_circle_geometry()
    assert math.ceil(math.hypot(232,119))+30 <= RADIUS*64
    fine=[[1]*SIDE for _ in range(SIDE)]
    coarse=inflate_known_free(fine)
    assert len(coarse)==59*59
    assert (2,20) not in coarse and (3,20) in coarse
    path,err=bfs(coarse,(10,10),(40,10))
    assert err==0 and len(path)==31 and first_turn(path)==(40,10)
    assert first_turn([(0,0),(1,0),(2,0),(2,1),(2,2)])==(2,0)
    # L-shaped fixed obstacle. A direct line is obstructed, but upper/lower
    # routes remain available; every returned edge has an inflated free cell.
    for y in range(35,86):
        for x in range(58,66):fine[y][x]=3
    for y in range(78,86):
        for x in range(66,84):fine[y][x]=3
    coarse=inflate_known_free(fine)
    start,goal=(14,28),(49,28)
    path,err=bfs(coarse,start,goal)
    assert err==0 and len(path)>36 and first_turn(path)!=goal
    assert all(p in coarse for p in path)
    assert all(abs(a-c)+abs(b-d)==1
               for (a,b),(c,d) in zip(path,path[1:]))
    waypoint=first_turn(path)
    stop=path.index(waypoint)
    assert len(set((b[0]-a[0],b[1]-a[1]) for a,b in zip(path[:stop],path[1:stop+1])))<=1
    # A complete unknown stripe blocks all routes even though it is not a hit.
    for y in range(SIDE):fine[y][63]=0
    assert bfs(inflate_known_free(fine),start,goal)[1]==4
    # Current/goal footprint must be known free; no automatic permissive bypass.
    fine=[[0]*SIDE for _ in range(SIDE)]
    assert bfs(inflate_known_free(fine),start,goal)[1]==2
    # A corridor narrower than swept envelope + uncertainty is rejected.
    fine=[[3]*SIDE for _ in range(SIDE)]
    for y in range(59,69):
        for x in range(10,118):fine[y][x]=1
    assert not inflate_known_free(fine)
    source=(Path(__file__).parents[1]/"src/lidar_path_planner.v").read_text(encoding="utf-8")
    for token in ("plan_rd_cell!=2'b01", "map_ready && map_frozen", "BACK_USE",
                  "previous_back_direction", "RADIUS_CELLS=5", "clearance_radius_mm",
                  "rounded_radius", "radius_cells", "query_required", "radius_square_next[21:12]",
                  "BUILD_NEXT", "RADIUS_SQUARE", "gap_squared"):
        assert token in source,token
    frozen_branch=source.split("if(map_ready && map_frozen) begin",1)[1].split("end",1)[0]
    assert "start_index<={rounded_current_y[6:1],rounded_current_x[6:1]}" in frozen_branch
    assert "rounded_current_x[7] || rounded_current_y[7]" in frozen_branch
    max_build_cycles=11+4096*((2*RADIUS+2)**2*3+1)
    max_search_cycles=4096*20+4096*2
    assert max_build_cycles+max_search_cycles<5_000_000
    # A 896mm wide known corridor is usable for the measured chassis, but
    # rejected by the old 512mm expansion. Unknown/out-of-bounds stay blocked.
    fine=[[3]*SIDE for _ in range(SIDE)]
    for y in range(57,71):
        for x in range(10,118):fine[y][x]=1
    assert bfs(inflate_known_free(fine,5),(16,32),(48,32))[1]==0
    assert not inflate_known_free(fine,8)
    for radius in (261,291,361,1000,1915):
        cells=(radius+63)//64
        assert 1<=cells<=31 and cells*64>=radius
    # Negative control: the old origin-corner tiling changes the checked
    # square by128mm for a1mm drift throughZERO. A measured wall atY=-445mm
    # is outside the actual291mm disk but rejected the old negative-Y start.
    fine=[[1]*SIDE for _ in range(SIDE)]
    fine[57]=[3]*SIDE  # occupied[-448,-384]mm row; no unknown conversion
    old=inflate_known_free(fine,centered=False)
    new=inflate_known_free(fine)
    assert (32,31) not in old and (32,32) in old
    assert bfs(new,(32,32),(40,32))[1]==0
    for coordinate in range(-64,64):
        fine_index=(coordinate+4096)//64
        assert (fine_index+1)//2==32
    for fine_index in range(128):
        rounded=(fine_index+1)//2
        if fine_index==127:assert rounded==64  # bounds error, never wrap
        else:
            centre=-4096+rounded*128
            lo=-4096+fine_index*64
            assert centre-64<=lo and lo+64<=centre+64
    print("PASS: centered128mm tiles, ZERO sign stability, all128 fine-index bounds; "
          "old near-ZERO wall false rejection reproduced and corrected without changing any map cell")
    print("PASS: actual291mm circular swept-radius inflation (132finecells vs144square); 1984serialradius squares; "
          "rectangle-intersection masks; diagonalunknown skip; sweptunknown/hit block; clear straight path; L-obstacle detour; "
          "first-turn waypoint; unknown barrier; unknown start; narrow corridor rejection")
    print(f"Conservative full-free build/search bound: "
          f"{max_build_cycles+max_search_cycles} clocks "
          f"({(max_build_cycles+max_search_cycles)/50000:.2f}ms at 50MHz)")
    print("Independent geometry/source checks only; not HDL simulation or timing proof.")

if __name__=="__main__":main()
