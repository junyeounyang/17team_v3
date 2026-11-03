"""대학로 SUMO 도로망을 그림 파일로 그린다 (sumo-gui 없이 구조 확인·발표 자료용).

    python utils/plot_daehakro.py                       # 전체·구간별 그림 (env/sumo/daehakro/*.png)
    python utils/plot_daehakro.py --snapshot 1800       # 1800초 시점의 차량·보행자까지 그리기
    python utils/plot_daehakro.py --views zoom          # 연결부(2)·로터리 확대 그림만

색 (도로 그림 파일의 <공통기호>와 맞춤)
    연두 = 버스전용차로 점선 구간(일반차 일시 진입 가능)   진초록 = 버스전용차로 실선 구간(버스만)
    파란 선 = 실제 노면의 버스전용차로 표시선           노란 선 = 중앙선, 흰 점선 = 차선
    흰 줄무늬 + 청록 테두리 = 횡단보도                  흰 굵은 선 = 정지선, 보라 막대 = 차량 신호등
    노랑 = 버스정류장, 파란 네모 = 차량 생성/소멸 지점, 분홍 = 어린이 보호구역(30 km/h)
    분홍 점선 상자 = 부모 차량 정차 공간(중학교 정문 앞), 회색+노란 테두리 = 섬, 빗금 = 도류화(차가 다니지 않는 곳)
    회색 = 보도, 베이지 = 건물, 초록 = 공원·가로수
    (스냅샷) 흰 사각형 = 승용차, 분홍 = 부모 차량, 파랑 = 버스, 빨강 점 = 보행자
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import LineCollection, PatchCollection  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import sumolib  # noqa: E402

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEF_DIR = os.path.join(BASE, "env", "sumo", "daehakro")

C = {
    "land": "#f1eee8", "asphalt": "#5d6168", "sidewalk": "#cfccc6", "curb": "#a9a59d",
    "bus_dash": "#8fdc86", "bus_solid": "#2f7d3a", "school": (0.86, 0.35, 0.55, 0.55),
    "lane_line": "#f4f4f4", "center": "#f2c200", "stop": "#ffffff", "signal": "#8e24aa",
    "cross_out": "#00785a", "busstop": "#f5d000", "spawn": "#1f4fbf",
    "car": "#f7f7f7", "bus": "#1e63d6", "person": "#e53935",
}
SCHOOL_ZONE_EDGES = ("sz0_", "sz1_", "szT_", "sz2_", "br_")   # 이화장길 (어린이 보호구역, 30 km/h)
# 중앙선을 그리는 본선 (v3: 교차로가 아닌 곳은 한쪽 방향만 나뉘어 북행·남행 도로의 노드가 서로 다름)
CENTER_EDGES = ("nb_", "sb_", "dae_", "E_in", "E_out", "S_in", "S_out")


# ── 기하 도우미 ────────────────────────────────────────────────────────────
def offset_line(pts, d):
    """폴리라인을 왼쪽(+d)/오른쪽(-d)으로 평행 이동."""
    out = []
    n = len(pts)
    for i in range(n):
        if i == 0:
            dx, dy = pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]
        elif i == n - 1:
            dx, dy = pts[-1][0] - pts[-2][0], pts[-1][1] - pts[-2][1]
        else:
            dx, dy = pts[i + 1][0] - pts[i - 1][0], pts[i + 1][1] - pts[i - 1][1]
        L = math.hypot(dx, dy) or 1.0
        out.append((pts[i][0] - dy / L * d, pts[i][1] + dx / L * d))
    return out


def band(shape, w_left, w_right):
    """폴리라인 왼쪽 w_left, 오른쪽 w_right 폭의 띠 다각형."""
    return offset_line(shape, w_left) + offset_line(shape, -w_right)[::-1]


def length(shape):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(shape[:-1], shape[1:]))


def point_at(shape, s):
    """시작점에서 거리 s 위치의 점과 단위 진행방향."""
    pos = 0.0
    for a, b in zip(shape[:-1], shape[1:]):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        if L <= 0:
            continue
        if pos + L >= s or b is shape[-1]:
            t = min(1.0, max(0.0, (s - pos) / L))
            return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t), ((b[0] - a[0]) / L, (b[1] - a[1]) / L)
        pos += L
    a, b = shape[-2], shape[-1]
    L = math.hypot(b[0] - a[0], b[1] - a[1]) or 1.0
    return b, ((b[0] - a[0]) / L, (b[1] - a[1]) / L)


def sub_line(shape, a, b, step=1.0):
    pts = []
    s = a
    while s < b:
        pts.append(point_at(shape, s)[0])
        s += step
    pts.append(point_at(shape, b)[0])
    return pts


def dashes(shape, on=3.0, off=3.0):
    segs = []
    L = length(shape)
    s = 0.0
    while s < L:
        segs.append(sub_line(shape, s, min(L, s + on), step=on / 2))
        s += on + off
    return segs


def rect(c, d, half_len, half_w):
    """중심 c, 방향 d(단위벡터)의 직사각형."""
    n = (-d[1], d[0])
    return [(c[0] + d[0] * half_len + n[0] * half_w, c[1] + d[1] * half_len + n[1] * half_w),
            (c[0] + d[0] * half_len - n[0] * half_w, c[1] + d[1] * half_len - n[1] * half_w),
            (c[0] - d[0] * half_len - n[0] * half_w, c[1] - d[1] * half_len - n[1] * half_w),
            (c[0] - d[0] * half_len + n[0] * half_w, c[1] - d[1] * half_len + n[1] * half_w)]


# ── 도로망 읽기 ────────────────────────────────────────────────────────────
def lane_kind(lane):
    if lane.allows("pedestrian") and not lane.allows("passenger") and not lane.allows("bus"):
        return "sidewalk"
    if lane.allows("bus") and not lane.allows("passenger"):
        return "bus_solid"
    if lane.allows("bus") and lane.allows("passenger") and not lane.allows("truck"):
        return "bus_dash"
    if lane.getEdge().getID().startswith(SCHOOL_ZONE_EDGES):
        return "school"
    return "general"


def poly_pts(el):
    return [tuple(map(float, p.split(","))) for p in el.get("shape").split()]


def poly_color(el, alpha=None):
    col = [int(v) / 255 for v in el.get("color").split(",")]
    if len(col) == 3:
        col.append(1.0)
    if alpha is not None:
        col[3] = alpha
    return tuple(col)


class Scene:
    def __init__(self, net_dir):
        self.net = sumolib.net.readNet(os.path.join(net_dir, "daehakro.net.xml"),
                                       withInternal=True, withPedestrianConnections=True)
        self.deco = ET.parse(os.path.join(net_dir, "daehakro.deco.xml")).getroot()
        self.stops = ET.parse(os.path.join(net_dir, "daehakro.stops.xml")).getroot()
        net = self.net
        self.normal = [e for e in net.getEdges(withInternal=True) if e.getFunction() == ""]
        self.internal = [e for e in net.getEdges(withInternal=True) if e.getFunction() == "internal"]
        self.crossings = [e for e in net.getEdges(withInternal=True) if e.getFunction() == "crossing"]
        self.walkareas = [e for e in net.getEdges(withInternal=True) if e.getFunction() == "walkingarea"]
        # 횡단보도가 가로지르는 도로(edge) — 그 도로로 들어오는 차로 끝에 정지선
        self.crossed = set()
        for el in ET.parse(os.path.join(net_dir, "daehakro.net.xml")).getroot().iter("edge"):
            if el.get("function") == "crossing":
                node = el.get("id")[1:el.get("id").rindex("_c")]
                for ce in el.get("crossingEdges", "").split():
                    self.crossed.add((node, ce))
        # 실제로 빨간불이 켜지는 신호만 '신호등'으로 표시 (항상 녹색/양보인 연결은 제외)
        progs = {}
        tll = os.path.join(net_dir, "daehakro.tll.xml")
        if os.path.exists(tll):
            for tl in ET.parse(tll).getroot().iter("tlLogic"):
                progs[tl.get("id")] = [ph.get("state") for ph in tl.iter("phase")]
        self.tls_lanes, self.red_lanes = set(), set()
        for tls in net.getTrafficLights():
            states = progs.get(tls.getID(), [])
            for in_lane, _out, idx in tls.getConnections():
                self.tls_lanes.add(in_lane.getID())
                if any(st[idx] in "ry" for st in states if idx < len(st)):
                    self.red_lanes.add(in_lane.getID())

    # 정지선이 필요한 차로: 빨간불이 켜지는 신호 차로 + 횡단보도 앞에서 멈추는 차로
    def stop_lanes(self):
        out = []
        for e in self.normal:
            node = e.getToNode().getID()
            for ln in e.getLanes():
                if lane_kind(ln) == "sidewalk" or not ln.getOutgoing():
                    continue
                if ln.getID() in self.red_lanes or (node, e.getID()) in self.crossed:
                    out.append(ln)
        return out


# ── 그리기 ────────────────────────────────────────────────────────────────
def draw_static(ax, sc, box):
    x0, y0, x1, y1 = box
    margin = 40

    def inside(pts):
        return any(x0 - margin <= x <= x1 + margin and y0 - margin <= y <= y1 + margin for x, y in pts)

    ax.set_facecolor(C["land"])
    # 1) 공원·캠퍼스·주차장 등 바탕 + 건물
    areas, a_cols, blds, b_cols = [], [], [], []
    for p in sc.deco.findall("poly"):
        typ = p.get("type")
        pts = poly_pts(p)
        if not inside(pts):
            continue
        if typ in ("park", "campus", "sports", "parking"):
            areas.append(Polygon(pts, closed=True))
            a_cols.append(poly_color(p, 0.75))
        elif typ == "building":
            blds.append(Polygon(pts, closed=True))
            b_cols.append(poly_color(p))
    ax.add_collection(PatchCollection(areas, facecolors=a_cols, edgecolors="none", zorder=1))
    ax.add_collection(PatchCollection(blds, facecolors=b_cols, edgecolors="#9c9488", linewidths=0.4, zorder=2))

    # 2) 보도·보행구역
    walk = []
    for e in sc.walkareas:
        shp = e.getLanes()[0].getShape()
        if len(shp) >= 3 and inside(shp):
            walk.append(Polygon(shp, closed=True))
    for e in sc.normal:
        for ln in e.getLanes():
            if lane_kind(ln) == "sidewalk" and inside(ln.getShape()):
                walk.append(Polygon(band(ln.getShape(), ln.getWidth() / 2, ln.getWidth() / 2), closed=True))
    ax.add_collection(PatchCollection(walk, facecolors=C["sidewalk"], edgecolors=C["curb"], linewidths=0.3, zorder=3))

    # 3) 교차로 면 + 차로 면 (교차로 안 연결 차로도 아스팔트로 칠해 틈이 보이지 않게)
    jun = []
    for n in sc.net.getNodes():
        shp = n.getShape()
        if len(shp) >= 3 and inside(shp):
            jun.append(Polygon(shp, closed=True))
    for e in sc.internal:
        for ln in e.getLanes():
            shp = ln.getShape()
            if len(shp) >= 2 and length(shp) > 0.01 and inside(shp):
                jun.append(Polygon(band(shp, ln.getWidth() / 2, ln.getWidth() / 2), closed=True))
    ax.add_collection(PatchCollection(jun, facecolors=C["asphalt"], edgecolors="none", zorder=4))
    lanes, l_cols = [], []
    for e in sc.normal:
        for ln in e.getLanes():
            k = lane_kind(ln)
            if k == "sidewalk" or not inside(ln.getShape()):
                continue
            lanes.append(Polygon(band(ln.getShape(), ln.getWidth() / 2, ln.getWidth() / 2), closed=True))
            l_cols.append({"bus_solid": C["bus_solid"], "bus_dash": C["bus_dash"]}.get(k, C["asphalt"]))
    ax.add_collection(PatchCollection(lanes, facecolors=l_cols, edgecolors="none", zorder=5))

    # 4) 어린이 보호구역(분홍) — deco의 노면 표시
    sz = [Polygon(poly_pts(p), closed=True) for p in sc.deco.findall("poly")
          if p.get("type") == "school zone 30km/h" and inside(poly_pts(p))]
    ax.add_collection(PatchCollection(sz, facecolors=[C["school"]], edgecolors="none", zorder=6))

    # 4') 섬·도류화 (deco): 남행 차로가 비켜 있는 빈 자리
    isl = [p for p in sc.deco.findall("poly") if p.get("type") in ("island", "median") and inside(poly_pts(p))]
    for p in isl:
        hatch = "////" if p.get("type") == "median" else None
        ax.add_patch(Polygon(poly_pts(p), closed=True, fc=("#efe6b0" if hatch else "#d7d4cc"), ec=C["center"],
                             lw=1.0, hatch=hatch, zorder=6.5))

    # 5) 차선·중앙선 (중앙선: 대학로·율곡로 본선과 양방향 옆길 — 반대 방향 차로와 맞닿은 쪽)
    lane_lines, center_lines = [], []
    for e in sc.normal:
        vls = [ln for ln in e.getLanes() if lane_kind(ln) != "sidewalk"]
        if not vls or not inside(vls[0].getShape()):
            continue
        for ln in vls[:-1]:          # 같은 방향 차로 사이 (오른쪽 → 왼쪽)
            lane_lines.extend(dashes(offset_line(ln.getShape(), ln.getWidth() / 2), 3.0, 4.0))
        left = vls[-1]
        rev = [o for o in e.getToNode().getOutgoing() if o.getToNode() == e.getFromNode()]
        if rev or e.getID().startswith(CENTER_EDGES):
            center_lines.append(offset_line(left.getShape(), left.getWidth() / 2 - 0.15))
    # 교차로가 아닌 곳(한쪽 방향만 나뉜 노드)의 교차로 면 위 차선 (deco)
    for p in sc.deco.findall("poly"):
        if p.get("type") == "lane marking" and inside(poly_pts(p)):
            lane_lines.append(poly_pts(p))
    ax.add_collection(LineCollection(lane_lines, colors=C["lane_line"], linewidths=0.6, zorder=7))
    ax.add_collection(LineCollection(center_lines, colors=C["center"], linewidths=1.1, zorder=7))

    # 6) 버스전용차로 파란 표시선(점선/실선) — deco
    blue = [poly_pts(p) for p in sc.deco.findall("poly") if p.get("type", "").startswith("bus lane")]
    blue = [b for b in blue if inside(b)]
    ax.add_collection(LineCollection(blue, colors="#1557d6", linewidths=1.2, zorder=8))

    # 7) 횡단보도 (흰 줄무늬 + 청록 테두리)
    stripes, outlines = [], []
    for e in sc.crossings:
        ln = e.getLanes()[0]
        shp, w = ln.getShape(), ln.getWidth()
        if not inside(shp):
            continue
        L = length(shp)
        s = 0.5
        while s < L - 0.3:
            c, d = point_at(shp, s)
            stripes.append(Polygon(rect(c, d, 0.25, w / 2 - 0.15), closed=True))
            s += 1.0
        outlines.append(Polygon(band(shp, w / 2, w / 2), closed=True))
    ax.add_collection(PatchCollection(stripes, facecolors="#ffffff", edgecolors="none", zorder=9))
    ax.add_collection(PatchCollection(outlines, facecolors="none", edgecolors=C["cross_out"], linewidths=1.0, zorder=9))

    # 8) 정지선 + 신호등
    stops, sigs = [], []
    for ln in sc.stop_lanes():
        shp, w = ln.getShape(), ln.getWidth()
        if not inside(shp):
            continue
        L = length(shp)
        c, d = point_at(shp, max(0.0, L - 0.3))
        stops.append(Polygon(rect(c, d, 0.25, w / 2), closed=True))
        if ln.getID() in sc.red_lanes:
            c2, _ = point_at(shp, max(0.0, L - 1.6))
            sigs.append(Polygon(rect(c2, d, 0.55, w / 2 - 0.35), closed=True))
    ax.add_collection(PatchCollection(stops, facecolors=C["stop"], edgecolors="none", zorder=10))
    ax.add_collection(PatchCollection(sigs, facecolors=C["signal"], edgecolors="none", alpha=0.85, zorder=10))

    # 9) 버스정류장
    for bs in sc.stops.findall("busStop"):
        ln = sc.net.getLane(bs.get("lane"))
        a, b = float(bs.get("startPos")), float(bs.get("endPos"))
        pts = sub_line(ln.getShape(), a, b)
        if len(pts) >= 2 and inside(pts):
            side = offset_line(pts, -ln.getWidth() / 2 + 0.3)
            side2 = offset_line(pts, -ln.getWidth() / 2 - 1.2)
            ax.add_patch(Polygon(side + side2[::-1], closed=True, fc=C["busstop"], ec="#a08800", lw=0.4, zorder=11))

    # 9') 부모 차량 정차 공간 (중학교 정문 앞 길가)
    for pa in sc.stops.findall("parkingArea"):
        ln = sc.net.getLane(pa.get("lane"))
        a, b = float(pa.get("startPos")), float(pa.get("endPos"))
        pts = sub_line(ln.getShape(), a, b)
        if len(pts) >= 2 and inside(pts):
            s1 = offset_line(pts, -ln.getWidth() / 2 + 0.2)
            s2 = offset_line(pts, -ln.getWidth() / 2 - 1.8)
            ax.add_patch(Polygon(s1 + s2[::-1], closed=True, fc=(1, 0.75, 0.9, 0.6), ec="#d0459a", lw=0.6,
                                 ls="--", zorder=11))

    # 10) 가로수
    trees = [Polygon(poly_pts(p), closed=True) for p in sc.deco.findall("poly")
             if p.get("type") == "tree" and inside(poly_pts(p))]
    t_cols = [poly_color(p, 0.85) for p in sc.deco.findall("poly")
              if p.get("type") == "tree" and inside(poly_pts(p))]
    ax.add_collection(PatchCollection(trees, facecolors=t_cols, edgecolors="#1e5c2a", linewidths=0.3, zorder=12))

    # 11) 생성/소멸 지점
    for node in sc.net.getNodes():
        if node.getID().endswith("_end"):
            x, y = node.getCoord()
            if x0 <= x <= x1 and y0 <= y <= y1:
                ax.plot([x], [y], "s", ms=9, mfc="none", mec=C["spawn"], mew=2.2, zorder=13)

    # 12) 지명
    for p in sc.deco.findall("poi"):
        if p.get("type") == "landmark":
            x, y = float(p.get("x")), float(p.get("y"))
            if x0 <= x <= x1 and y0 <= y <= y1:
                ax.text(x, y, p.get("id"), fontsize=7, color="#3b3b3b", ha="center", va="center", zorder=14,
                        bbox=dict(boxstyle="round,pad=0.2", fc=(1, 1, 1, 0.6), ec="none"))


def draw_traffic(ax, snap, box):
    x0, y0, x1, y1 = box
    cars, buses, parents, ped = [], [], [], []
    for v in snap["veh"]:
        x, y = v["pos"]
        if not (x0 - 10 <= x <= x1 + 10 and y0 - 10 <= y <= y1 + 10):
            continue
        a = math.radians(v["angle"])
        d = (math.sin(a), math.cos(a))
        c = (x - d[0] * v["length"] / 2, y - d[1] * v["length"] / 2)
        poly = Polygon(rect(c, d, v["length"] / 2, v["width"] / 2), closed=True)
        if v["vclass"] == "bus":
            buses.append(poly)
        elif v.get("type", "").startswith("parent"):
            parents.append(poly)
        else:
            cars.append(poly)
    for p in snap["ped"]:
        x, y = p
        if x0 <= x <= x1 and y0 <= y <= y1:
            ped.append(p)
    ax.add_collection(PatchCollection(cars, facecolors=C["car"], edgecolors="#202020", linewidths=0.5, zorder=20))
    ax.add_collection(PatchCollection(buses, facecolors=C["bus"], edgecolors="#0b2a66", linewidths=0.5, zorder=20))
    ax.add_collection(PatchCollection(parents, facecolors="#ff5cbf", edgecolors="#202020", linewidths=0.5, zorder=20))
    if ped:
        ax.scatter([p[0] for p in ped], [p[1] for p in ped], s=6, c=C["person"], zorder=21, linewidths=0)


def take_snapshot(net_dir, t_snap, seed):
    import traci
    cfgf = os.path.join(net_dir, "daehakro.sumocfg")
    traci.start([sumolib.checkBinary("sumo"), "-c", cfgf, "--seed", str(seed), "--no-step-log", "true",
                 "--no-warnings", "true", "--end", str(t_snap + 5)])
    while traci.simulation.getTime() < t_snap:
        traci.simulationStep()
    veh = []
    for v in traci.vehicle.getIDList():
        veh.append({"pos": traci.vehicle.getPosition(v), "angle": traci.vehicle.getAngle(v),
                    "length": traci.vehicle.getLength(v), "width": traci.vehicle.getWidth(v),
                    "vclass": traci.vehicle.getVehicleClass(v), "type": traci.vehicle.getTypeID(v)})
    ped = [traci.person.getPosition(p) for p in traci.person.getIDList()]
    traci.close()
    return {"t": t_snap, "veh": veh, "ped": ped}


def draw(sc, out, views, snap=None, panel_w=6.5, panel_h=11.0, dpi=130):
    fig, axes = plt.subplots(1, len(views), figsize=(panel_w * len(views), panel_h), dpi=dpi)
    if len(views) == 1:
        axes = [axes]
    for ax, (title, box) in zip(axes, views):
        draw_static(ax, sc, box)
        if snap:
            draw_traffic(ax, snap, box)
            title = f"{title}  (t={snap['t']:.0f}s)"
        x0, y0, x1, y1 = box
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
        ax.set_aspect("equal")
        ax.set_title(title, fontsize=12)
        ax.tick_params(labelsize=7)
    plt.tight_layout()
    fig.savefig(out)
    plt.close(fig)
    return out


VIEWS = {
    "overview": [("전체 (이화사거리 → 혜화동로터리)", (-260, -170, 260, 1300))],
    "detail": [("① 이화사거리", (-80, -60, 200, 240)),
               ("② 연결부(1)", (-60, 200, 110, 560)),
               ("③ 연결부(2)", (-60, 590, 110, 1035)),
               ("④ 혜화동로터리", (-150, 960, 130, 1250))],
    "zoom": [("연결부(2) 구역(5): A·유턴 베이·RS", (-45, 915, 70, 1060)),
             ("연결부(2) 구역(3)·(4): E·D·C", (-45, 690, 70, 900)),
             ("연결부(1) 구역(1): M·N", (-45, 290, 70, 400)),
             ("연결부(2) 구역(1)·(2): G·F", (-45, 580, 70, 700))],
    "zoom2": [("이화사거리 (횡단보도 4곳)", (-70, -60, 70, 60)),
              ("어린이 보호구역 (정문·부모 차량)", (-10, 30, 200, 140)),
              ("로터리 서쪽·북쪽", (-110, 1030, 25, 1175))],
}


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=DEF_DIR)
    ap.add_argument("--out", default=None, help="저장할 PNG 경로 접두사 (기본: env/sumo/daehakro/daehakro)")
    ap.add_argument("--views", default="overview,detail,zoom,zoom2")
    ap.add_argument("--snapshot", type=float, default=None, help="이 시각(초)의 차량·보행자를 함께 그림")
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()
    plt.rcParams["font.family"] = ["Noto Sans CJK KR", "AppleGothic", "Malgun Gothic", "NanumGothic", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    scene = Scene(args.dir)
    snap = take_snapshot(args.dir, args.snapshot, args.seed) if args.snapshot else None
    prefix = args.out or os.path.join(args.dir, "daehakro")
    if prefix.endswith(".png"):
        prefix = prefix[:-4]
    suffix = f"_t{int(args.snapshot)}" if snap else ""
    for name in args.views.split(","):
        views = VIEWS[name]
        if name == "overview":
            path = draw(scene, f"{prefix}_overview{suffix}.png", views, snap, panel_w=9, panel_h=16, dpi=140)
        elif name == "zoom":
            path = draw(scene, f"{prefix}_zoom{suffix}.png", views, snap, panel_w=6.5, panel_h=8, dpi=150)
        elif name == "zoom2":
            path = draw(scene, f"{prefix}_zoom2{suffix}.png", views, snap, panel_w=8, panel_h=7, dpi=150)
        else:
            path = draw(scene, f"{prefix}_{name}{suffix}.png", views, snap)
        print(path)
