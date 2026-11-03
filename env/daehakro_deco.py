"""대학로 SUMO 화면 장식 (daehakro.deco.xml).

    - 주변 건물·공원·학교 부지·나무: OpenStreetMap 발췌(env/data/daehakro_osm.txt)를 모델 도로에 맞춰 배치
        · 대학로 주변: 실제 대학로 중심선 기준 (따라 잰 거리, 옆으로 떨어진 거리) → 모델의 곧은 대학로로 펴서 옮김
        · 혜화동로터리 주변: 로터리 중심 기준으로 돌리고, 모델 로터리가 더 크므로(반지름 40m) 바깥으로 밀어냄
        · 이화사거리 주변: 실제 좌표 그대로 (이화사거리 갈래가 원래 OSM 좌표)
      v3: 건물은 작은 것을 빼고 도로 방향에 맞춘 반듯한 직사각형으로 (겹침·도로 침범 제거) → 깔끔하게.
    - 대학로 가로수: 보도 바깥쪽에 일정 간격
    - 버스전용차로 표시: 파란 선, 점선 구간은 끊어진 선 / 실선 구간은 이어진 선 (그림 공통기호와 같은 구분)
    - 한쪽 방향만 나뉜 노드(옆길 입구 등)의 교차로 면 위 차선, 섬·빗금 도류화
    - 어린이 보호구역: 이화장길 노면을 붉게 + 표지

시뮬레이션에는 영향이 없다 (화면 표시용 polygon/poi).
"""
import math
import os
import random

import numpy as np
from matplotlib.path import Path

try:
    from env import daehakro_config as cfg
except ImportError:  # 스크립트로 직접 실행할 때
    import daehakro_config as cfg  # type: ignore

LW = cfg.LANE_WIDTH

# ── OSM 좌표 → 실측 평면(m): 기준 37.58N, 127.0E (동 +x, 북 +y) ──
_LAT0, _LON0 = 37.57, 126.99
_R = 6371000.0
_LATC = math.radians(37.58)
# 실제 지형의 기준점 (위 평면 좌표, OSM에서 읽음)
IH_REAL = (194.7, -424.5)          # 이화사거리 중심
ROT_REAL = (105.43, 617.13)        # 혜화동로터리 중심
ROT_R_REAL = 29.6                  # 실제 회전 차로 반지름
ROT_TURN = 20.0                    # 로터리 주변을 시계 방향으로 돌리는 각(도): 실제 갈래 방향 → 그림 방향


def _xy(tok):
    a, b = tok.split(",")
    lat, lon = _LAT0 + int(a) * 1e-6, _LON0 + int(b) * 1e-6
    return ((lon - 127.0) * math.pi / 180 * _R * math.cos(_LATC), (lat - 37.58) * math.pi / 180 * _R)


def load_osm(path):
    items = []
    if not os.path.exists(path):
        return items
    for line in open(path, encoding="utf-8"):
        line = line.rstrip("\n")
        if not line or line.startswith("#"):
            continue
        k, nm, lv, pts = line.split("|", 3)
        P = [_xy(t) for t in pts.split()]
        items.append((k, nm, lv, P))
    return items


class Mapper:
    """실제 평면 좌표 → 모델 좌표."""

    def __init__(self, items):
        pts = []
        for k, _nm, _lv, P in items:
            if k.startswith("R:primary:대학로:"):
                for x, y in P:
                    if IH_REAL[1] - 1 <= y <= 575.0 and 110 <= x <= 200:
                        pts.append((x, y))
        pts = sorted(set(pts), key=lambda p: p[1])
        if not pts or pts[0] != IH_REAL:
            pts = [IH_REAL] + [p for p in pts if p[1] > IH_REAL[1] + 1]
        self.P = np.array(pts)
        seg = np.diff(self.P, axis=0)
        self.seg_len = np.hypot(seg[:, 0], seg[:, 1])
        self.cum = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        # 로터리 진입부 (실제 중심에서 반지름+20m 떨어진 곳) ↔ 모델 로터리 남쪽 횡단보도
        d_c = np.hypot(self.P[:, 0] - ROT_REAL[0], self.P[:, 1] - ROT_REAL[1])
        k = int(np.argmax(d_c < ROT_R_REAL + 20.0)) if np.any(d_c < ROT_R_REAL + 20.0) else len(self.P) - 1
        self.L_c = float(self.cum[max(1, k)])
        self.k = cfg.STA["RS"] / self.L_c
        R = cfg.ROTARY
        self.rot_c = R["center"]
        self.dr = R["radius"] - ROT_R_REAL
        a = math.radians(-ROT_TURN)            # 시계 방향 회전
        self.ca, self.sa = math.cos(a), math.sin(a)

    def project(self, q):
        """대학로 중심선 위 (따라 잰 거리 s, 동쪽 + 횡거리 d)."""
        best = None
        for i in range(len(self.P) - 1):
            a, b = self.P[i], self.P[i + 1]
            ab = b - a
            L2 = float(ab @ ab)
            t = 0.0 if L2 == 0 else max(0.0, min(1.0, float((np.asarray(q) - a) @ ab) / L2))
            f = a + ab * t
            dv = np.asarray(q) - f
            dist = math.hypot(dv[0], dv[1])
            if best is None or dist < best[0]:
                tx, ty = ab / max(1e-9, math.sqrt(L2))
                d = dv[0] * ty - dv[1] * tx          # 오른쪽(동쪽) 법선
                best = (dist, self.cum[i] + t * math.sqrt(L2), d)
        return best[1], best[2]

    def region(self, q):
        s, d = self.project(q)
        dc = math.hypot(q[0] - ROT_REAL[0], q[1] - ROT_REAL[1])
        if 20.0 < s < self.L_c - 8.0 and abs(d) < 230.0 and dc > ROT_R_REAL + 18.0:
            return "corr"
        if dc < 170.0:
            return "rot"
        if math.hypot(q[0] - IH_REAL[0], q[1] - IH_REAL[1]) < 330.0 and s <= 25.0:
            return "ih"
        return None

    def map(self, q, reg):
        if reg == "corr":
            s, d = self.project(q)
            return (d, s * self.k)
        if reg == "rot":
            rx, ry = q[0] - ROT_REAL[0], q[1] - ROT_REAL[1]
            rx, ry = rx * self.ca - ry * self.sa, rx * self.sa + ry * self.ca
            r = math.hypot(rx, ry)
            f = (r + self.dr) / max(r, 1e-6)
            return (self.rot_c[0] + rx * f, self.rot_c[1] + ry * f)
        if reg == "ih":
            return (q[0] - IH_REAL[0], q[1] - IH_REAL[1])
        return None


class Occupancy:
    """도로(차로·보도·교차로) 점유 격자 (1 m)."""

    def __init__(self, net, clearance, x0=-320, x1=320, y0=-220, y1=1380, res=1.0, sidewalks=True):
        self.x0, self.y0, self.res = x0, y0, res
        self.nx, self.ny = int((x1 - x0) / res), int((y1 - y0) / res)
        self.g = np.zeros((self.ny, self.nx), dtype=bool)
        for e in net.getEdges(withInternal=False):
            for ln in e.getLanes():
                if not sidewalks and ln.allows("pedestrian") and not ln.allows("passenger"):
                    continue
                self._mark_line(ln.getShape(), ln.getWidth() / 2.0 + clearance)
        for j in net.getNodes():
            sh = j.getShape()
            if len(sh) >= 3:
                self._mark_poly(sh)

    def _idx(self, x, y):
        return int((x - self.x0) / self.res), int((y - self.y0) / self.res)

    def _mark_line(self, shape, r):
        rr = int(math.ceil(r / self.res))
        for (xa, ya), (xb, yb) in zip(shape[:-1], shape[1:]):
            L = math.hypot(xb - xa, yb - ya)
            n = max(2, int(L / (self.res * 0.5)) + 1)
            for t in np.linspace(0.0, 1.0, n):
                cx, cy = self._idx(xa + (xb - xa) * t, ya + (yb - ya) * t)
                i0, i1 = max(0, cy - rr), min(self.ny, cy + rr + 1)
                j0, j1 = max(0, cx - rr), min(self.nx, cx + rr + 1)
                if i0 < i1 and j0 < j1:
                    self.g[i0:i1, j0:j1] = True

    def _mark_poly(self, shape):
        xs, ys = [p[0] for p in shape], [p[1] for p in shape]
        j0, i0 = self._idx(min(xs), min(ys))
        j1, i1 = self._idx(max(xs), max(ys))
        j0, i0, j1, i1 = max(0, j0), max(0, i0), min(self.nx - 1, j1), min(self.ny - 1, i1)
        if j1 < j0 or i1 < i0:
            return
        jj, ii = np.meshgrid(np.arange(j0, j1 + 1), np.arange(i0, i1 + 1))
        px = self.x0 + (jj + 0.5) * self.res
        py = self.y0 + (ii + 0.5) * self.res
        inside = Path(shape).contains_points(np.c_[px.ravel(), py.ravel()]).reshape(px.shape)
        self.g[i0:i1 + 1, j0:j1 + 1] |= inside

    def overlap(self, poly):
        """다각형이 도로와 겹치는 비율."""
        xs, ys = [p[0] for p in poly], [p[1] for p in poly]
        j0, i0 = self._idx(min(xs), min(ys))
        j1, i1 = self._idx(max(xs), max(ys))
        if j1 < 0 or i1 < 0 or j0 >= self.nx or i0 >= self.ny:
            return 0.0
        j0, i0, j1, i1 = max(0, j0), max(0, i0), min(self.nx - 1, j1), min(self.ny - 1, i1)
        jj, ii = np.meshgrid(np.arange(j0, j1 + 1), np.arange(i0, i1 + 1))
        px = self.x0 + (jj + 0.5) * self.res
        py = self.y0 + (ii + 0.5) * self.res
        inside = Path(poly).contains_points(np.c_[px.ravel(), py.ravel()]).reshape(px.shape)
        n = inside.sum()
        if n == 0:
            cx, cy = self._idx(sum(xs) / len(xs), sum(ys) / len(ys))
            return 1.0 if (0 <= cx < self.nx and 0 <= cy < self.ny and self.g[cy, cx]) else 0.0
        return float((inside & self.g[i0:i1 + 1, j0:j1 + 1]).sum()) / float(n)

    def hit(self, x, y):
        j, i = self._idx(x, y)
        return 0 <= j < self.nx and 0 <= i < self.ny and bool(self.g[i, j])


def offset_line(shape, d):
    """진행 방향 오른쪽으로 d만큼 평행 이동한 선 (d<0이면 왼쪽)."""
    out = []
    n = len(shape)
    for i in range(n):
        if i == 0:
            ax, ay = shape[1][0] - shape[0][0], shape[1][1] - shape[0][1]
        elif i == n - 1:
            ax, ay = shape[-1][0] - shape[-2][0], shape[-1][1] - shape[-2][1]
        else:
            ax, ay = shape[i + 1][0] - shape[i - 1][0], shape[i + 1][1] - shape[i - 1][1]
        L = math.hypot(ax, ay) or 1.0
        out.append((shape[i][0] + ay / L * d, shape[i][1] - ax / L * d))
    return out


def resample(shape, step):
    pts = [shape[0]]
    acc = 0.0
    for (xa, ya), (xb, yb) in zip(shape[:-1], shape[1:]):
        L = math.hypot(xb - xa, yb - ya)
        t = step - acc
        while t <= L:
            pts.append((xa + (xb - xa) * t / L, ya + (yb - ya) * t / L))
            t += step
        acc = (acc + L) % step
    return pts


def sub_line(shape, a, b):
    """선 위 거리 a~b 구간."""
    out, pos = [], 0.0
    for (xa, ya), (xb, yb) in zip(shape[:-1], shape[1:]):
        L = math.hypot(xb - xa, yb - ya)
        if L == 0:
            continue
        s0, s1 = max(a, pos), min(b, pos + L)
        if s0 < s1:
            p0 = (xa + (xb - xa) * (s0 - pos) / L, ya + (yb - ya) * (s0 - pos) / L)
            p1 = (xa + (xb - xa) * (s1 - pos) / L, ya + (yb - ya) * (s1 - pos) / L)
            if not out or out[-1] != p0:
                out.append(p0)
            out.append(p1)
        pos += L
    return out


def line_length(shape):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(shape[:-1], shape[1:]))


def circle(cx, cy, r, n=10):
    return [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def fmt(points):
    return " ".join(f"{x:.2f},{y:.2f}" for x, y in points)


def poly_area(P):
    return 0.5 * abs(sum(P[i][0] * P[(i + 1) % len(P)][1] - P[(i + 1) % len(P)][0] * P[i][1]
                         for i in range(len(P))))


def convex_hull(P):
    pts = sorted(set((round(x, 3), round(y, 3)) for x, y in P))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def min_rect(P, snap_deg=None, snap_tol=20.0):
    """최소 넓이 직사각형 (중심, 방향각, 반길이 a, 반폭 b). snap_deg가 있으면 그 각(±90°)에 가까울 때 맞춤."""
    H = convex_hull(P)
    best = None
    for i in range(len(H)):
        x0, y0 = H[i]
        x1, y1 = H[(i + 1) % len(H)]
        ang = math.atan2(y1 - y0, x1 - x0)
        best = _rect_at(H, ang, best)
    if snap_deg is not None and best is not None:
        d = (math.degrees(best[1]) - snap_deg) % 90.0
        if min(d, 90.0 - d) <= snap_tol:
            best = _rect_at(H, math.radians(snap_deg), None)
    return best


def _rect_at(H, ang, best):
    c, s_ = math.cos(ang), math.sin(ang)
    us = [x * c + y * s_ for x, y in H]
    vs = [-x * s_ + y * c for x, y in H]
    a = (max(us) - min(us)) / 2
    b = (max(vs) - min(vs)) / 2
    if best is None or 4 * a * b < 4 * best[2] * best[3]:
        uc, vc = (max(us) + min(us)) / 2, (max(vs) + min(vs)) / 2
        cx, cy = uc * c - vc * s_, uc * s_ + vc * c
        return ((cx, cy), ang, a, b)
    return best


def rect_poly(center, ang, a, b):
    c, s_ = math.cos(ang), math.sin(ang)
    out = []
    for du, dv in [(a, b), (-a, b), (-a, -b), (a, -b)]:
        out.append((center[0] + du * c - dv * s_, center[1] + du * s_ + dv * c))
    return out


def build_deco(net, plan, cor, J, out_path, base_dir):
    """sumo-gui 그리기 순서(높이): 차로 3 < 교차로 4 < 보행구역 5 < 횡단보도 7 < 차량.
    노면 표시(버스차로 선, 교차로 안 차선)는 6, 섬·도류화는 4.5, 건물·공원은 0 아래."""
    rnd = random.Random(7)
    D = cfg.DECO
    lines = ["<additional>"]
    info = {}
    import sumolib  # noqa: E402  (builder가 SUMO_HOME/tools를 경로에 넣어 둠)
    inet = sumolib.net.readNet(out_path.replace(".deco.xml", ".net.xml"), withInternal=True)

    def dashed(line, pid, color, width, layer, typ, on=3.0, off=3.0):
        out, L, s0, k = [], line_length(line), 0.0, 0
        while s0 < L - 0.5:
            seg = sub_line(line, s0, min(L, s0 + on))
            if len(seg) >= 2:
                out.append(f'    <poly id="{pid}_{k}" type="{typ}" color="{color}" fill="0" lineWidth="{width}" '
                           f'layer="{layer}" shape="{fmt(seg)}"/>')
                k += 1
            s0 += on + off
        return out

    # ── 1) 버스전용차로 표시 (파란 선: 점선 구간 = 끊어진 선, 실선 구간 = 이어진 선) ──
    n_mark = 0
    for eid, e in plan.edges.items():
        kinds = [k[0] if isinstance(k, tuple) else k for k in e["lanes"]]
        o = plan.off(eid)
        for li, kind in enumerate(kinds):
            if kind not in ("bus", "dash"):
                continue
            if eid.startswith(("arc", "slip")) or (eid.startswith("nb_BAY0") and li == 0):
                continue                                   # 정류장 공간·베이는 노면 표시 없음
            lane = net.getEdge(eid).getLane(li + o)
            sh = lane.getShape()
            if len(sh) < 2:
                continue
            # 일반 차로와 맞닿은 쪽 경계: 가장 바깥 버스차로는 왼쪽, 중앙 버스차로는 오른쪽
            side = -1.0 if li < len(kinds) - 1 else 1.0
            line = offset_line(sh, side * LW / 2.0)
            dashed_ = kind == "dash" or eid == "SW_out"   # 그림: 남서쪽 중앙 버스차로 점선 (모델은 버스 전용)
            if dashed_:
                new = dashed(line, f"bl_{eid}_{li}", "0,90,230", 0.25, 6, "bus lane dashed")
                lines += new
                n_mark += len(new)
            else:
                lines.append(f'    <poly id="bl_{eid}_{li}" type="bus lane solid" color="0,90,230" '
                             f'fill="0" lineWidth="0.35" layer="6" shape="{fmt(line)}"/>')
                n_mark += 1
    info["버스차로 표시"] = n_mark

    # ── 1') 교차로가 아닌 곳(한쪽 방향만 나뉜 노드)의 교차로 면 위로 차선을 이어 그림 ──
    #   (sumo-gui·그림 모두 교차로 면 안에는 차선을 안 그리므로, 옆길 입구·차로 늘어나는 곳에서 차선이 끊겨 보이지 않게)
    names_nb = {n for _s, n in cor["nb_order"]}
    names_sb = {n for _s, n in cor["sb_order"]}
    one_sided = (names_nb ^ names_sb) | {"E_branch", "J_mg"}
    main = ("nb_", "sb_", "E_in", "arcW1")
    n_jl = 0
    for nid in sorted(one_sided):
        try:
            node = inet.getNode(nid)
        except KeyError:
            continue
        groups = {}
        for c in node.getConnections():
            fe, te = c.getFrom(), c.getTo()
            if c.getDirection() != "s" or not fe.getID().startswith(main) or not te.getID().startswith(main):
                continue
            via = c.getViaLaneID()
            if not via:
                continue
            vl = inet.getLane(via)
            if vl.getLength() < 1.0:
                continue                                    # 교차로 면이 없는 곳 (그대로 이어져 보임)
            groups.setdefault((fe.getID(), te.getID()), []).append(
                (c.getFromLane().getIndex(), c.getToLane().getIndex(), vl, c.getFromLane()))
        for (fe, te), cs in groups.items():
            cs.sort(key=lambda x: (x[0], x[1]))
            for a in cs:
                for b in cs:
                    if b[0] == a[0] + 1 and b[1] == a[1] + 1:
                        sh = a[2].getShape()
                        line = offset_line(sh, -a[2].getWidth() / 2.0)      # a(오른쪽 차로)의 왼쪽 경계
                        fl = a[3]
                        is_bus = fl.allows("bus") and not fl.allows("truck") and a[0] == plan.off(fe)
                        if is_bus:
                            solid = not fl.allows("passenger")
                            if solid:
                                lines.append(f'    <poly id="jl_{nid}_{fe}_{a[0]}" type="bus lane solid" '
                                             f'color="0,90,230" fill="0" lineWidth="0.35" layer="6" '
                                             f'shape="{fmt(line)}"/>')
                            else:
                                lines += dashed(line, f"jl_{nid}_{fe}_{a[0]}", "0,90,230", 0.25, 6,
                                                "bus lane dashed")
                        else:
                            lines += dashed(line, f"jl_{nid}_{fe}_{a[0]}", "255,255,255", 0.15, 6,
                                            "lane marking", 3.0, 4.0)
                        n_jl += 1
    info["교차로 안 차선"] = n_jl

    # ── 1'') 섬·도류화(빗금) 구간: 남행 차로가 한 차로만큼 비켜 있는 곳의 빈 자리 ──
    S = cfg.STA
    e_s = S["E"] - cfg.CROSSING_WIDTH["E"] / 2
    e0, e1 = e_s - cfg.E_ISLAND["len"], e_s - cfg.E_ISLAND["len"] - cfg.E_ISLAND["taper"]
    a0 = cfg.A_STOP - cfg.UTURN["bay"] - cfg.UTURN["median"]
    a1 = a0 - cfg.UTURN["taper"]
    islands = [("island_E", [(0.0, e1), (0.0, e_s), (-LW, e_s), (-LW, e0)], "island"),
               ("median_A", [(0.0, a1), (0.0, S["UBAY"]), (-LW, S["UBAY"]), (-LW, a0)], "median")]
    # A 북행 정지선 북쪽 ~ 로터리 남쪽 횡단보도: 남행과 북행 사이 섬 (북행 유턴은 섬 남쪽 끝을 돌아 나감)
    uturn_top = cfg.A_STOP + 4.0
    try:
        for c in inet.getNode("A").getConnections():
            if c.getFrom().getID().startswith("nb_") and c.getTo().getID().startswith("sb_") and c.getViaLaneID():
                uturn_top = max(uturn_top, max(p[1] for p in inet.getLane(c.getViaLaneID()).getShape()) + 1.0)
    except KeyError:
        pass
    rs_s = S["RS"] - cfg.CROSSING_WIDTH["main"] / 2
    if rs_s - uturn_top > 2.0:
        islands.append(("island_A", [(0.0, uturn_top), (0.0, rs_s), (-LW, rs_s), (-LW, uturn_top + 1.5)], "island"))
    for pid, P, typ in islands:
        col = "205,205,195" if typ == "island" else "236,228,170"
        lines.append(f'    <poly id="{pid}" type="{typ}" color="{col}" fill="1" layer="4.5" shape="{fmt(P)}"/>')
        lines.append(f'    <poly id="{pid}_edge" type="{typ} edge" color="240,190,0" fill="0" lineWidth="0.2" '
                     f'layer="4.6" shape="{fmt(P + [P[0]])}"/>')

    # ── 2) 어린이 보호구역 노면 + 표지 ──
    for eid in ["sz0_out", "sz0_in", "sz1_out", "sz1_in", "szT_out", "szT_in", "sz2_out", "sz2_in", "br_out", "br_in"]:
        lane = net.getEdge(eid).getLane(plan.off(eid))
        sh = lane.getShape()
        poly = offset_line(sh, LW / 2.0) + list(reversed(offset_line(sh, -LW / 2.0)))
        lines.append(f'    <poly id="sz_{eid}" type="school zone 30km/h" color="215,40,40,90" fill="1" '
                     f'layer="3.5" shape="{fmt(poly)}"/>')
    T = cfg.IHWA["school_T"]
    lines.append(f'    <poi id="어린이보호구역_30km/h" type="school zone" color="220,30,30" layer="8" '
                 f'x="{T[0] - 40:.1f}" y="{T[1] + 7:.1f}" width="3" height="3"/>')
    lines.append(f'    <poi id="중학교_정문" type="landmark" color="60,60,60" layer="8" '
                 f'x="{cfg.SCHOOL_GATE_X:.1f}" y="{T[1] + 9:.1f}"/>')

    # ── 3) 주변 건물·공원 (OpenStreetMap) — v3: 깔끔하게 (작은 건물 빼고, 반듯한 직사각형으로, 도로 방향에 맞춤) ──
    items = load_osm(os.path.join(base_dir, D["osm_file"]))
    if items:
        mp = Mapper(items)
        occ = Occupancy(net, D["road_clearance"])
        occ_car = Occupancy(net, 0.3, sidewalks=False)
        bocc = Occupancy(net, 0.0)                        # 이미 놓은 건물 (겹침 방지)
        bocc.g[:] = False
        area_cols = {"P": (120, 175, 95), "S": (150, 195, 120), "U": (235, 225, 185), "W": (120, 170, 220),
                     "K": (190, 190, 190)}
        layer = {"P": -3, "S": -2, "U": -4, "W": -2, "K": -3}
        typ_of = {"P": "park", "S": "sports", "U": "campus", "W": "water", "K": "parking"}
        ih_axis = math.degrees(math.atan2(-cfg.IHWA["W_end"][1], -cfg.IHWA["W_end"][0]))   # 율곡로 방향
        n_b = n_drop = n_small = n_a = 0
        labels, blds = [], []
        for idx, (k, nm, lv, P) in enumerate(items):
            kk = k.split(":")[0]
            if (kk != "B" and kk not in area_cols) or len(P) < 3:
                continue
            cx = sum(p[0] for p in P) / len(P)
            cy = sum(p[1] for p in P) / len(P)
            reg = mp.region((cx, cy))
            if reg is None:
                continue
            Q = [mp.map(p, reg) for p in P]
            if Q[0] == Q[-1]:
                Q = Q[:-1]
            if len(Q) < 3:
                continue
            if kk == "B":
                blds.append((poly_area(Q), idx, Q, reg))
                continue
            n_a += 1
            r, g, b = area_cols[kk]
            lines.append(f'    <poly id="osm_{kk}_{idx}" type="{typ_of[kk]}" '
                         f'color="{r},{g},{b}" fill="1" layer="{layer[kk]}" shape="{fmt(Q)}"/>')
            if nm and kk in ("P", "U") and len(nm) <= 14:
                labels.append((nm, sum(q[0] for q in Q) / len(Q), sum(q[1] for q in Q) / len(Q)))
        tones = [(214, 207, 195), (202, 195, 183), (190, 184, 173)]
        for area, idx, Q, reg in sorted(blds, reverse=True):
            if area < D["min_building_area"]:
                n_small += 1
                continue
            snap = 90.0 if reg == "corr" else (ih_axis if reg == "ih" else None)
            (cx, cy), ang, a, b = min_rect(Q, snap_deg=snap)
            fill = area / max(1e-6, 4 * a * b)
            if fill < 0.55:                               # ㄱ자·ㄷ자 건물: 직사각형을 실제 넓이에 가깝게 줄임
                f_ = math.sqrt(fill / 0.55)
                a, b = a * f_, b * f_
            gap = D["building_gap"] / 2
            a, b = a - gap, b - gap
            if a < 2.5 or b < 2.5:
                n_small += 1
                continue
            R = rect_poly((cx, cy), ang, a, b)
            if occ.overlap(R) > 0.03 or bocc.overlap(R) > 0.08:
                n_drop += 1
                continue
            bocc._mark_poly(R)
            n_b += 1
            r, g, b_ = tones[0 if area < 250 else (1 if area < 900 else 2)]
            lines.append(f'    <poly id="osm_B_{idx}" type="building" color="{r},{g},{b_}" fill="1" layer="-1" '
                         f'shape="{fmt(R)}"/>')
        seen = set()
        for nm, x, y in labels:
            if nm in seen:
                continue
            seen.add(nm)
            pid = "".join(ch if ch.isalnum() else "_" for ch in nm)
            lines.append(f'    <poi id="{pid}" type="landmark" color="60,60,60" layer="8" x="{x:.1f}" y="{y:.1f}"/>')
        info["건물"] = n_b
        info["뺀 건물(작음/겹침)"] = f"{n_small}/{n_drop}"
        info["녹지·부지"] = n_a

        # ── 4) 나무: OpenStreetMap 나무 + 대학로 가로수 ──
        trees = []
        for k, _nm, _lv, P in items:
            if k == "t":
                reg = mp.region(P[0])
                if reg:
                    x, y = mp.map(P[0], reg)
                    if not occ_car.hit(x, y) and not bocc.hit(x, y):
                        trees.append((x, y))
        sp = D["street_tree_spacing"]
        for eid, e in plan.edges.items():
            if not (eid.startswith(("nb_", "sb_")) and e["sidewalk"] > 0):
                continue
            sw = net.getEdge(eid).getLane(0)
            sh = sw.getShape()
            if line_length(sh) < 16:
                continue
            outer = offset_line(sh, sw.getWidth() / 2.0 - 0.8)
            L = line_length(outer)
            for pnt in resample(sub_line(outer, 7.0, L - 7.0), sp):
                trees.append(pnt)
        n_t = 0
        for x, y in trees:
            r = rnd.uniform(1.8, 2.6)
            lines.append(f'    <poly id="tree_{n_t}" type="tree" color="40,{rnd.randint(110, 140)},55,190" '
                         f'fill="1" layer="6.5" shape="{fmt(circle(x, y, r))}"/>')
            n_t += 1
        info["나무"] = n_t

    lines.append("</additional>")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return info
