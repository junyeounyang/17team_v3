"""env/daehakro_config.py 를 읽어 대학로 SUMO 파일들을 만드는 모듈 (v3).

만드는 파일 (기본 위치: env/sumo/daehakro/)
    daehakro.nod.xml / .edg.xml / .con.xml : 도로 원본 (노드, 차로, 연결, 횡단보도)
    daehakro.net.xml                       : netconvert가 만든 완성 도로망 (보도·횡단보도 포함)
    daehakro.tll.xml                       : 신호 현시 (차량 + 보행 신호)
    daehakro.stops.xml                     : 버스정류장 + 부모 차량 정차 공간(중학교 정문 앞)
    daehakro.deco.xml                      : 건물·나무·버스차로 선·교차로 면 차선·섬·어린이 보호구역 표시 (화면용)
    daehakro.rou.xml                       : 차량 종류(버스/일반/부모 차량), 보행자, 오후 3시 교통 수요
    daehakro.sumocfg / daehakro.gui.xml    : 실행 설정 (gui.xml: 우선 도로 정지선 막대 감춤)

v3 변경점 (피드백 "수정부분1", "3-구역(3),(4),(5) 도로그림")
    - 교차로가 아닌 곳은 한쪽 방향만 노드로 나눔 (NB_ONLY / SB_ONLY): 우회전 진출입 옆길, 버스차로 경계,
      베이·차로 수 변화 → 반대 방향 차로·중앙선이 이어짐. 버스차로 경계 등은 면적 0 노드.
    - M+N 한 교차로(남행 정지선 N 북쪽), D를 30m E 횡단보도 북쪽에 붙임(섬·비켜 이어짐), C 입구는 횡단보도 남쪽,
      A 입구는 북행 정지선과 로터리 남쪽 횡단보도 사이(남행 신호는 횡단보도 북쪽), 유턴 베이 + 빗금 도류화.
    - 이화사거리 4갈래 횡단보도, 신호③ 단축, I 북행 1차로 좌회전 전용(보호 좌회전).
    - 혜화동로터리: 남쪽 회전 2차로, 서쪽 바깥 차로 일반 차로 + 우회전 전 합류, 혜화로 앞 진출입만 신호.
    - 하교 시간 어린이 보호구역: 부모 차량(서행 → 정문 앞 정차), 정문 하교 학생, 정문 앞 횡단보도.

수업 기본 코드(road_builder.py, 직선 고속도로)는 그대로 두고, 이 모듈은 따로 동작한다.
"""
import collections
import math
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

if "SUMO_HOME" in os.environ:
    _tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    if _tools not in sys.path:
        sys.path.append(_tools)
import sumolib  # noqa: E402
from sumolib import checkBinary  # noqa: E402

try:
    from env import daehakro_config as cfg
except ImportError:  # 스크립트로 직접 실행할 때
    import daehakro_config as cfg  # type: ignore

LW = cfg.LANE_WIDTH
V50 = cfg.SPEED["default"]
V30 = cfg.SPEED["school_zone"]
SWM = cfg.SIDEWALK["main"]
SWS = cfg.SIDEWALK["side"]

# 차로 종류 → SUMO 허용 차종
GEN = "gen"        # 일반 차로 (버스·승용 모두)
BUS = "bus"        # 버스전용 실선 / 버스 베이·정류장 공간
DASH = "dash"      # 버스전용 점선 구간 (승용차 잠시 진입 허용)
ALLOW = {
    GEN: 'disallow="pedestrian bicycle"',
    BUS: 'allow="bus"',
    DASH: 'allow="bus passenger taxi"',
}


# 교차로 안 대기 지점(internal junction)을 없애 차가 정지선(횡단보도 앞)에서 기다리게 하는 연결 속성.
# 옆길 진출·진입, 유턴, 좌·우회전처럼 경로 중간에 횡단보도·상충 지점이 있는 연결에 쓴다.
CP = 'contPos="0"'
# 버스만 지나갈 수 있는 연결: 버스전용차로(점선)에서 그대로 직진하는 연결. 일반 차는 점선 구간에 우회전(옆길
# 진입)하려고만 잠시 들어오므로, 이 교차로를 지나 계속 버스차로로 달리지 못하게 한다.
BUS_ONLY = 'allow="bus"'
# 버스차로(점선)로 직진하는 일반 차를 막는 교차로: 이화장길 입구, O, A(로터리 앞 버스차로 끝)
#   (M→N, E→D처럼 바로 다음 옆길로 우회전하려고 점선에 들어온 차가 지나가야 하는 곳은 막지 않음)
BUS_ONLY_STRAIGHT = ("IWJ", "O", "A")
# 신호 교차로 안에서 신호를 받지 않는 연결 (예: D의 남행 직진, A의 남행, 로터리 혜화로 앞 안쪽 회전 차로)
UNC = 'uncontrolled="true"'
# 우선 통과 (양보하지 않음): 신호 없는 본선 직진이 신호 받는 좌회전·유턴보다 우선하도록
PASS = 'pass="true"'
SCHOOL_ZONE_PREFIX = ("sz0_", "sz1_", "szT_", "sz2_", "br_")


def bus_kind(kind):
    """버스차로 운영이 꺼져 있으면(active=False) 모든 버스 차로를 일반 차로로."""
    return kind if cfg.BUS_LANE["active"] else GEN


# ──────────────────────────────────────────────────────────────
# 기하 도우미
# ──────────────────────────────────────────────────────────────
def polar(center, radius, ang_deg):
    a = math.radians(ang_deg)
    return (center[0] + radius * math.cos(a), center[1] + radius * math.sin(a))


def bearing_vec(bearing_deg):
    """방위(북=0, 동=90) → 단위벡터 (x, y)."""
    b = math.radians(bearing_deg)
    return (math.sin(b), math.cos(b))


def add(p, v, s=1.0):
    return (p[0] + v[0] * s, p[1] + v[1] * s)


def arc_points(center, radius, a0, a1, step=6.0):
    """a0 → a1 (반시계, 도) 원호 위 점들."""
    if a1 < a0:
        a1 += 360.0
    n = max(2, int(math.ceil((a1 - a0) / step)) + 1)
    return [polar(center, radius, a0 + (a1 - a0) * i / (n - 1)) for i in range(n)]


def fmt_shape(points):
    return " ".join(f"{x:.2f},{y:.2f}" for x, y in points)


# ──────────────────────────────────────────────────────────────
# 도로망 모음 (차로 번호는 "논리 번호": 0 = 가장 바깥 차량 차로. 보도는 따로 붙여 씀)
# ──────────────────────────────────────────────────────────────
class Plan:
    def __init__(self):
        self.nodes = {}       # id -> (x, y, type)
        self.edges = {}       # id -> dict
        self.conns = []       # (from, to, fromLane, toLane, 추가 속성)  ← 논리 차로 번호
        self.crossings = []   # (node, [edges], priority, width)
        self.boundary = {}    # 경계 이름 -> (들어오는 edge, 나가는 edge)
        self.info = {}

    def node(self, nid, xy, ntype="priority"):
        self.nodes[nid] = (float(xy[0]), float(xy[1]), ntype)
        return nid

    def edge(self, eid, frm, to, lanes, speed=V50, priority=3, shape=None, spread=None,
             name="", sidewalk=0.0):
        self.edges[eid] = dict(frm=frm, to=to, lanes=list(lanes), speed=speed, priority=priority,
                               shape=shape, spread=spread, name=name, sidewalk=sidewalk)
        return eid

    def off(self, eid):
        """논리 차로 번호 → SUMO 차로 번호 차이 (보도가 0번)."""
        return 1 if self.edges[eid]["sidewalk"] > 0 else 0

    def nl(self, eid):
        return len(self.edges[eid]["lanes"])

    def conn(self, frm, to, from_lanes, to_lanes, extra=""):
        """from_lanes[i] → to_lanes[i] (길이가 다르면 짧은 쪽을 반복).
        extra: 연결에 붙일 SUMO 속성 문자열 (예: 'pass="true"')."""
        if isinstance(from_lanes, int):
            from_lanes = [from_lanes]
        if isinstance(to_lanes, int):
            to_lanes = [to_lanes]
        n = max(len(from_lanes), len(to_lanes))
        for i in range(n):
            self.conns.append((frm, to, from_lanes[min(i, len(from_lanes) - 1)],
                               to_lanes[min(i, len(to_lanes) - 1)], extra))

    def straight(self, frm, to):
        for i in range(min(self.nl(frm), self.nl(to))):
            self.conns.append((frm, to, i, i, ""))

    def crossing(self, node, edges, priority=True, width=4.0):
        self.crossings.append((node, list(edges), priority, width))

    # ---------- 파일 쓰기 ----------
    def write(self, prefix):
        with open(prefix + ".nod.xml", "w", encoding="utf-8") as f:
            f.write("<nodes>\n")
            for nid, (x, y, t) in self.nodes.items():
                rad = cfg.JUNCTION_RADIUS.get(nid)
                ra = f' radius="{rad:.1f}"' if rad is not None else ""
                f.write(f'    <node id="{nid}" x="{x:.2f}" y="{y:.2f}" type="{t}"{ra}/>\n')
            f.write("</nodes>\n")
        with open(prefix + ".edg.xml", "w", encoding="utf-8") as f:
            f.write("<edges>\n")
            for eid, e in self.edges.items():
                o = self.off(eid)
                attrs = [f'id="{eid}"', f'from="{e["frm"]}"', f'to="{e["to"]}"',
                         f'numLanes="{len(e["lanes"]) + o}"', f'speed="{e["speed"]:.2f}"',
                         f'priority="{e["priority"]}"', f'width="{LW}"']
                if e["name"]:
                    attrs.append(f'name="{e["name"]}"')
                if e["shape"]:
                    attrs.append(f'shape="{fmt_shape(e["shape"])}"')
                if e["spread"]:
                    attrs.append(f'spreadType="{e["spread"]}"')
                f.write("    <edge " + " ".join(attrs) + ">\n")
                if o:
                    f.write(f'        <lane index="0" allow="pedestrian" width="{e["sidewalk"]:.2f}"/>\n')
                for i, kind in enumerate(e["lanes"]):
                    extra = ""
                    if isinstance(kind, tuple):
                        kind, extra = kind
                    f.write(f'        <lane index="{i + o}" {ALLOW[kind]} {extra}/>\n')
                f.write("    </edge>\n")
            f.write("</edges>\n")
        with open(prefix + ".con.xml", "w", encoding="utf-8") as f:
            f.write("<connections>\n")
            seen = set()
            # 교차로 안에서는 진로변경 금지 (도로교통법 제14조)
            no_lc = ' changeLeft="emergency" changeRight="emergency"'
            for c in self.conns:
                if c[:4] in seen:
                    continue
                seen.add(c[:4])
                fl, tl = c[2] + self.off(c[0]), c[3] + self.off(c[1])
                extra = (f" {c[4]}" if c[4] else "") + no_lc
                f.write(f'    <connection from="{c[0]}" to="{c[1]}" fromLane="{fl}" toLane="{tl}"{extra}/>\n')
            for node, edges, prio, width in self.crossings:
                f.write(f'    <crossing node="{node}" edges="{" ".join(edges)}" '
                        f'priority="{"true" if prio else "false"}" width="{width:.1f}"/>\n')
            f.write("</connections>\n")


# ──────────────────────────────────────────────────────────────
# 1) 대학로 본선 + 옆길 + 이화사거리 + 이화장길
# ──────────────────────────────────────────────────────────────
TLS_CORRIDOR = {"IH", "M", "O", "I", "H", "F", "E", "D", "C", "A", "RS"}
# v3: 교차로가 아닌 곳은 한쪽 방향만 노드로 나눔 (반대쪽 차로·중앙선이 끊기지 않게)
NB_ONLY = {"IWJ", "BAY0", "BAY1", "G", "C_s", "B", "UBAY"}     # + 버스차로 점선/실선 경계 (자동)
SB_ONLY = {"SB3", "SB2"}
# 동쪽 옆길 → 붙는 노드 (N은 M 교차로, C 옆길 입구는 C_s)
SIDE_NODE = {"N": "M", "O": "O", "G": "G", "D": "D", "C": "C_s", "B": "B", "A": "A", "I": "I"}


def build_corridor(p: Plan):
    S = cfg.STA
    secs = cfg.BUS_LANE_SECTIONS
    LW_ = LW
    named_nb = {k: v for k, v in S.items() if k not in SB_ONLY}
    named_sb = {k: v for k, v in S.items() if k not in NB_ONLY}

    # ── 북행 노드: 기능 지점 + 버스차로 점선/실선 경계 (기능 노드와 12m 안이면 그 노드에 맞춤) ──
    bounds = sorted({round(a, 1) for a, _b, _k in secs} | {round(b, 1) for _a, b, _k in secs})
    nb_pts = dict((round(v, 2), k) for k, v in named_nb.items())
    for s in bounds:
        if not (0.0 < s < S["RS"]):
            continue
        if min(abs(s - v) for v in named_nb.values()) < 12.0:
            continue
        nb_pts[s] = f"d{int(round(s))}"
    nb_order = sorted(nb_pts.items())                  # [(측점, 노드)]
    sb_order = sorted((round(v, 2), k) for k, v in named_sb.items())
    for s, nid in nb_order + sb_order:
        if nid not in p.nodes:
            p.node(nid, (0.0, s), "traffic_light" if nid in TLS_CORRIDOR else "priority")

    A_s = S["A"]

    def kind_at(s0, s1):
        mid = (s0 + s1) / 2
        for a, b, kind in secs:
            if a - 0.1 <= mid <= b + 0.1:
                return bus_kind(BUS if kind == "solid" else DASH)
        return GEN

    def nb_kinds(n0, s0, s1):
        if s0 >= A_s - 0.1:
            return [GEN] * 4                          # A~로터리: 버스차로 끝, 일반 4차로
        kinds = [kind_at(s0, s1)] + [GEN] * 3
        if n0 == "BAY0":
            kinds = [bus_kind(BUS)] + kinds           # 버스 베이 (가장 바깥, 버스 전용)
        if n0 == "UBAY":
            kinds = kinds + [GEN]                     # 북행 유턴 베이 (가장 안쪽, 유턴 전용)
        return kinds

    def sb_kinds(s_lo, s_hi):
        if s_hi <= S["SB3"] + 0.1:
            return [GEN] * 4                          # 이화사거리 앞: 우 / 직 / 좌 / 좌+유턴
        if s_hi <= S["SB2"] + 0.1:
            return [GEN] * 3
        if s_hi <= S["D"] + 0.1:
            return [GEN] * 2
        return [GEN] * 3                              # D 북쪽: 바깥 2 + 좌회전·유턴 1 (D에서 끝)

    # 남행 형상: 섬·도류화 구간은 중앙선 쪽 한 차로 비움(x=-LW), 끝에서 비스듬히 원래 자리(x=0)로
    E_s, F_s = S["E"], S["F"]
    e_isl0 = E_s - cfg.CROSSING_WIDTH["E"] / 2 - cfg.E_ISLAND["len"]
    e_isl1 = e_isl0 - cfg.E_ISLAND["taper"]
    a_med0 = cfg.A_STOP - cfg.UTURN["bay"] - cfg.UTURN["median"]
    a_med1 = a_med0 - cfg.UTURN["taper"]

    def sb_shape(n_hi, s_hi, n_lo, s_lo):
        if n_hi == "RS" and n_lo == "A":
            return [(-LW_, s_hi), (-LW_, s_lo)]
        if n_hi == "A":                               # 유턴 베이·빗금 구간 → 비켜 이어짐
            return [(-LW_, s_hi), (-LW_, a_med0), (0.0, a_med1), (0.0, s_lo)]
        if n_hi == "D" and n_lo == "E":
            return [(-LW_, s_hi), (-LW_, s_lo)]
        if n_hi == "E":                               # E 횡단보도 남쪽 섬 → 비켜 이어짐
            return [(-LW_, s_hi), (-LW_, e_isl0), (0.0, e_isl1), (0.0, s_lo)]
        return None

    nb_in_m, nb_out_m, sb_in_m, sb_out_m = {}, {}, {}, {}
    for (s0, n0), (s1, n1) in zip(nb_order[:-1], nb_order[1:]):
        shape = [(-LW_, s0), (-LW_, s1)] if n0 == "UBAY" else None
        e = p.edge(f"nb_{n0}_{n1}", n0, n1, nb_kinds(n0, s0, s1), shape=shape, name="대학로", sidewalk=SWM)
        nb_out_m[n0], nb_in_m[n1] = e, e
    for (s0, n0), (s1, n1) in zip(sb_order[:-1], sb_order[1:]):
        e = p.edge(f"sb_{n1}_{n0}", n1, n0, sb_kinds(s0, s1), shape=sb_shape(n1, s1, n0, s0),
                   name="대학로", sidewalk=SWM)
        sb_out_m[n1], sb_in_m[n0] = e, e

    def nb_in(n):
        return nb_in_m[n]

    def nb_out(n):
        return nb_out_m[n]

    def sb_in(n):
        return sb_in_m[n]

    def sb_out(n):
        return sb_out_m[n]

    # ── 본선 직진 연결 ──
    for s, nid in nb_order:
        if nid in ("IH", "RS"):
            continue
        a, b = nb_in(nid), nb_out(nid)
        la, lb = p.nl(a), p.nl(b)
        if nid == "BAY0":                              # 버스 베이 시작: 북행 버스는 모두 베이 정류장에 서므로
            p.conn(a, b, 0, 0)                         # 버스차로 → 베이로만 연결 (정차 후 옆 버스차로로 차로변경)
            for k in range(1, la):
                p.conn(a, b, k, k + 1)
        elif nid == "BAY1":                            # 베이 끝: 베이 차로는 끊김 (차로변경 합류)
            for k in range(1, la):
                p.conn(a, b, k, k - 1)
        elif nid == "UBAY":                            # 유턴 베이(가장 안쪽)는 차로변경으로 들어감
            for k in range(la):
                p.conn(a, b, k, k)
        elif nid == "A":                               # 유턴 베이(4번)는 유턴만, 버스차로(점선)는 우회전만
            p.conn(a, b, 0, 0, BUS_ONLY)
            p.conn(a, b, [1, 2, 3], [1, 2, 3])
        elif nid == "D":                               # D 북행 직진은 신호 없음 (북행 신호는 E 횡단보도 남쪽)
            p.conn(a, b, [0, 1, 2, 3], [0, 1, 2, 3], UNC + " " + PASS)
        elif nid == "I":                               # 1차로(가장 안쪽 3번) = I 좌회전 전용 (그림 ②)
            p.conn(a, b, [0, 1, 2], [0, 1, 2])         # I 북쪽 4번째 차로는 2번 차로 직진 차가 나눠 씀
            p.conn(a, b, 2, 3)
        elif la == lb and nid in BUS_ONLY_STRAIGHT:
            p.conn(a, b, 0, 0, BUS_ONLY)
            p.conn(a, b, list(range(1, la)), list(range(1, la)))
        elif la == lb:
            p.straight(a, b)
        else:
            raise RuntimeError(f"북행 차로 연결 정의 없음: {nid} {la}->{lb}")
    for s, nid in sb_order:
        if nid in ("IH", "RS"):
            continue
        a, b = sb_in(nid), sb_out(nid)
        la, lb = p.nl(a), p.nl(b)
        if nid == "D":
            p.conn(a, b, [0, 1], [0, 1], UNC)          # 3번째(안쪽) 차로는 D에서 좌회전·유턴만
        elif nid == "SB2":                             # 바깥에 차로 추가: 바깥 차로가 두 갈래로 나뉨
            p.conn(a, b, 0, [0, 1])                    # (우회전·직진 차가 차로변경 없이 바깥 새 차로로)
            p.conn(a, b, 1, 2)
        elif nid == "SB3":                             # 맨 바깥 우회전 차로는 차로변경으로 진입
            p.conn(a, b, [0, 1, 2], [1, 2, 3])
        elif nid == "A":                               # 남행은 A에서 신호 없음 (신호는 로터리 남쪽 횡단보도 북쪽)
            p.conn(a, b, [0, 1, 2], [0, 1, 2], UNC + " " + PASS)
        elif la == lb:
            p.straight(a, b)
        else:
            raise RuntimeError(f"남행 차로 연결 정의 없음: {nid} {la}->{lb}")

    # ── 동쪽 옆길 ──
    x_east = LW * 4 + SWM + 2.0

    def east_street(key, n_in=1):
        nid = SIDE_NODE[key]
        L = cfg.SIDE_STREET_LENGTH[key]
        s = S[nid]
        end = p.node(f"{key}_end", (x_east + L, s))
        e_out = p.edge(f"{key}_out", nid, end, [GEN], priority=1, name=f"옆길 {key}", sidewalk=SWS)
        e_in = p.edge(f"{key}_in", end, nid, [GEN] * n_in, priority=1, name=f"옆길 {key}", sidewalk=SWS)
        p.boundary[key] = (e_in, e_out)
        return nid, e_in, e_out

    W_side = cfg.CROSSING_WIDTH["side"]
    W_main = cfg.CROSSING_WIDTH["main"]
    # 옆길에서 나온 차: 뒤쪽이 긴 점선 구간이면 버스차로로(C, B), 실선이거나 짧으면 일반 차로(1번)로
    lane0_out = {"C", "B", "A"}
    for key in ["N", "G", "D", "C", "B", "A"]:
        nid, e_in, e_out = east_street(key)
        extra_in = CP
        extra_out = CP
        if key == "D":                                 # D: 신호는 남행 좌회전·유턴에만 (나머지는 신호 없음)
            extra_in = extra_out = CP + " " + UNC
        if key == "N":                                 # N: M 교차로 안 (진출은 횡단보도를 지나지 않아 신호 없음)
            extra_out = CP + " " + UNC
        # A 진출: 바로 앞 로터리 남쪽 횡단보도(RS) 신호에 맞춰 A 신호가 제어 (RS 적색일 때 A 교차로에 들어가 서지 않게)
        p.conn(nb_in(nid), e_out, 0, 0, extra_in)      # 버스차로(점선)에서 우회전 진입
        p.conn(e_in, nb_out(nid), 0, 0 if key in lane0_out else 1, extra_out)
        p.crossing(nid, [e_in, e_out], True, W_side)   # 옆길 입구 (신호 없음, 보행자 우선)
    # D: 남행 좌회전 + 유턴 (E 보행 신호와 함께)
    p.conn(sb_in("D"), "D_out", 2, 0, CP)
    p.conn(sb_in("D"), nb_out("D"), 2, 2, CP)
    # A: 북행 유턴 (유턴 베이 → 남행 바깥 차로, 로터리 남쪽 횡단보도 보행 신호 때)
    p.conn(nb_in("A"), sb_out("A"), 4, 0, CP)
    # O: 신호 교차로, 진출 2차로 (0 = 우회전, 1 = 좌회전). 북쪽 횡단보도 K + O 입구 횡단보도
    _o, O_in, O_out = east_street("O", n_in=2)
    p.conn(nb_in("O"), O_out, 0, 0, CP)
    p.conn(O_in, nb_out("O"), 0, 1, CP)
    p.conn(O_in, sb_out("O"), 1, 1, CP)
    p.crossing("O", [nb_out("O"), sb_in("O")], True, cfg.CROSSING_WIDTH["K"])
    p.crossing("O", [O_in, O_out], True, W_side)

    # ── 서쪽 옆길 I (신호 교차로): 북행 좌회전 진입(1차로 전용), 남행 우회전 진입, 우회전 진출 ──
    I_end = p.node("I_end", (-(LW * 2 + SWM + 2.0 + cfg.SIDE_STREET_LENGTH["I"]), S["I"]))
    I_out = p.edge("I_out", "I", I_end, [GEN], priority=1, name="옆길 I", sidewalk=SWS)
    I_in = p.edge("I_in", I_end, "I", [GEN], priority=1, name="옆길 I", sidewalk=SWS)
    p.boundary["I"] = (I_in, I_out)
    p.conn(nb_in("I"), I_out, 3, 0, CP)
    p.conn(sb_in("I"), I_out, 0, 0, CP)
    p.conn(I_in, sb_out("I"), 0, 0, CP)
    p.crossing("I", [I_in, I_out], True, W_side)

    # ── 대학로를 가로지르는 신호 횡단보도 (정지선 = 횡단보도 바로 앞) ──
    for nid in ["M", "H", "F", "C", "RS"]:
        p.crossing(nid, [nb_in(nid), sb_out(nid)], True, W_main)      # M은 교차로 남쪽 (N 입구는 북쪽)
    p.crossing("E", [nb_in("E"), sb_out("E")], True, cfg.CROSSING_WIDTH["E"])   # 깊이 30m

    # ── 이화장길 (어린이 보호구역, 30 km/h) ──
    T = p.node("SZ_T", cfg.IHWA["school_T"])
    SZe = p.node("SZ_end", cfg.IHWA["school_end"])
    Eb = p.node("E_branch", cfg.IHWA["E_branch"])
    # 중학교 정문 앞 횡단보도 (v3, 신호 없음·보행자 우선): 하교하는 학생이 길 건너 보도·부모 차량으로 건넘
    Gt = p.node("SZ_G", (cfg.SCHOOL_GATE_X, cfg.IHWA["school_T"][1]))
    sz = dict(speed=V30, sidewalk=SWS)
    sz0o = p.edge("sz0_out", "IWJ", Gt, [GEN], priority=2, name="이화장길", **sz)
    sz0i = p.edge("sz0_in", Gt, "IWJ", [GEN], priority=2, name="이화장길", **sz)
    # T자 교차점 서쪽 횡단보도는 T에서 조금 떨어진 별도 횡단보도로 (차가 T 안에서가 아니라 횡단보도 바로 앞에서 양보)
    Tx = p.node("SZ_X", (cfg.IHWA["school_T"][0] - 25.0, cfg.IHWA["school_T"][1]))
    sz1o = p.edge("sz1_out", Gt, Tx, [GEN], priority=2, name="이화장길", **sz)
    sz1i = p.edge("sz1_in", Tx, Gt, [GEN], priority=2, name="이화장길", **sz)
    szTo = p.edge("szT_out", Tx, T, [GEN], priority=2, name="이화장길", **sz)
    szTi = p.edge("szT_in", T, Tx, [GEN], priority=2, name="이화장길", **sz)
    p.straight(sz0o, sz1o)
    p.straight(sz1i, sz0i)
    p.straight(sz1o, szTo)
    p.straight(szTi, sz1i)
    p.crossing("SZ_G", [sz0o, sz0i], True, W_side)
    p.crossing("SZ_X", [sz1o, sz1i], True, W_side)
    sz2o = p.edge("sz2_out", T, SZe, [GEN], priority=2, name="이화장길", **sz)
    sz2i = p.edge("sz2_in", SZe, T, [GEN], priority=2, name="이화장길", **sz)
    bro = p.edge("br_out", T, Eb, [GEN], priority=1, name="이화장길 남쪽 골목", **sz)
    bri = p.edge("br_in", Eb, T, [GEN], priority=1, name="이화장길 남쪽 골목", **sz)
    p.boundary["SZ"] = (sz2i, sz2o)
    p.conn(nb_in("IWJ"), sz0o, 0, 0, CP)              # 버스차로(점선)에서 우회전 진입
    p.conn(sz0i, nb_out("IWJ"), 0, 1, CP)             # 우회전 진출 (점선 10m를 가로질러 일반 차로)
    for a, b in [(szTo, sz2o), (szTo, bro), (sz2i, szTi), (sz2i, bro), (bri, szTi), (bri, sz2o)]:
        p.conn(a, b, 0, 0, CP)
    p.crossing("IWJ", [sz0i, sz0o], True, W_side)     # 이화장길 입구 (보행자 우선)

    # ── 이화사거리 ──
    IH = "IH"
    W_end = p.node("W_end", cfg.IHWA["W_end"])
    S_end = p.node("S_end", cfg.IHWA["S_end"])
    E_pts = cfg.IHWA["E_points"]
    E_end = p.node("E_end", E_pts[-1])
    W_in = p.edge("W_in", W_end, IH, [GEN] * 3, name="율곡로", sidewalk=SWM)
    W_out = p.edge("W_out", IH, W_end, [GEN] * 3, name="율곡로", sidewalk=SWM)
    # 율곡로 서행만 학교 앞 남쪽 골목에서 나뉨 (골목은 북쪽이라 동행은 그대로 이어짐 — 구역(5))
    E_in2 = p.edge("E_in2", E_end, Eb, [GEN] * 3, name="율곡로", sidewalk=SWM,
                   shape=[E_pts[-1], E_pts[-2], cfg.IHWA["E_branch"]])
    E_in1 = p.edge("E_in1", Eb, IH, [GEN] * 3, name="율곡로", sidewalk=SWM)
    E_out = p.edge("E_out", IH, E_end, [GEN] * 3, name="율곡로", sidewalk=SWM,
                   shape=[(0.0, 0.0), cfg.IHWA["E_branch"], E_pts[-2], E_pts[-1]])
    # 남쪽 갈래: 이화사거리 정지선 앞 S_dash(40m)만 버스전용차로 점선(그림). 그 바깥은 일반 차로
    sx, sy = cfg.IHWA["S_end"]
    k = (cfg.IHWA["S_dash"] + 20.0) / math.hypot(sx, sy)      # 교차로 반 길이 약 20m
    S_mid = p.node("S_mid", (sx * k, sy * k))
    S_in0 = p.edge("S_in0", S_end, S_mid, [GEN] * 3, name="대학로", sidewalk=SWM)
    # 교차로 앞 25m는 북쪽 갈래와 같은 방향으로: 버스차로(바깥)가 북쪽 버스차로와 일직선 → 버스가 비스듬히 지나며
    # 북쪽 횡단보도 모퉁이에서 기다리는 보행자 옆을 스치지 않음 (중앙선을 동쪽으로 한 차로만큼)
    sx2 = LW
    S_in = p.edge("S_in", S_mid, IH, [bus_kind(DASH), GEN, GEN], name="대학로", sidewalk=SWM,
                  shape=[(sx * k, sy * k), (sx2, -25.0), (sx2, 0.0)])
    p.straight(S_in0, S_in)
    S_out = p.edge("S_out", IH, S_end, [GEN], name="대학로", sidewalk=SWM,
                   shape=[(sx2, 0.0), (sx2, -25.0), (sx * k, sy * k), (sx, sy)])
    p.boundary.update({"W": (W_in, W_out), "E": (E_in2, E_out), "S": (S_in0, S_out)})
    p.straight(E_in2, E_in1)
    p.conn(E_in2, bri, 0, 0, CP)                      # 율곡로 서행 → 골목 우회전 진입
    p.conn(bro, E_in1, 0, 0, CP)                      # 골목 → 율곡로 서행 우회전 진출 (정지선에서 틈을 기다림)
    NBo, SBi = nb_out(IH), sb_in(IH)
    # 남행(대학로 → 이화사거리) 4차로: 0 우회전, 1 직진, 2 좌회전, 3 좌회전+유턴
    p.conn(SBi, W_out, 0, 0, CP)
    p.conn(SBi, S_out, 1, 0)
    p.conn(SBi, E_out, 2, 1, CP)
    p.conn(SBi, E_out, 3, 2, CP)
    p.conn(SBi, NBo, 3, 2, CP)                        # 유턴 (남행 → 북행)
    # 북행(남쪽 갈래) 0 버스차로(점선: 버스 직진 + 우회전), 1 직진, 2 직진+좌회전
    p.conn(S_in, NBo, 0, 0, BUS_ONLY)
    p.conn(S_in, E_out, 0, 0, CP)
    p.conn(S_in, NBo, 1, [1, 2])
    p.conn(S_in, NBo, 2, 3)
    p.conn(S_in, W_out, 2, 2, CP)
    # 율곡로 동행(서쪽 갈래): 0 직진+우회전, 1 직진, 2 직진+좌회전
    p.conn(W_in, E_out, [0, 1, 2], [0, 1, 2])
    p.conn(W_in, S_out, 0, 0, CP)
    p.conn(W_in, NBo, 2, 3, CP)
    # 율곡로 서행(동쪽 갈래): 0 직진+우회전, 1 직진, 2 직진+좌회전
    p.conn(E_in1, W_out, [0, 1, 2], [0, 1, 2])
    # 북쪽 우회전: 이화장길로 갈 차는 버스차로(점선)로, 나머지는 바로 2차로(일반)로 → 버스차로 안에서 엮이지 않음
    p.conn(E_in1, NBo, 0, [0, 1], CP)
    p.conn(E_in1, S_out, 2, 0, CP)
    # 4갈래 횡단보도 (v3: 동·서·남쪽 추가)
    W_ih = cfg.CROSSING_WIDTH["ih"]
    p.crossing(IH, [NBo, SBi], True, W_ih)            # 북쪽
    p.crossing(IH, [E_in1, E_out], True, W_ih)        # 동쪽
    p.crossing(IH, [W_in, W_out], True, W_ih)         # 서쪽
    p.crossing(IH, [S_in, S_out], True, W_ih)         # 남쪽

    return dict(nb_in=nb_in, nb_out=nb_out, sb_in=sb_in, sb_out=sb_out,
                nb_order=nb_order, sb_order=sb_order, nb_in_m=nb_in_m, sb_in_m=sb_in_m,
                nb_out_m=nb_out_m, sb_out_m=sb_out_m)


# ──────────────────────────────────────────────────────────────
# 2) 혜화동로터리
# ──────────────────────────────────────────────────────────────
def build_rotary(p: Plan, cor):
    R = cfg.ROTARY
    C, rad, ang = R["center"], R["radius"], R["ang"]
    r_in = rad - LW * R["ring_lanes"] / 2.0               # 회전 차로 안쪽 경계 (모든 회전 구간 공통)
    J = {k: polar(C, rad, a) for k, a in ang.items()}
    for k, t in [("NE", "traffic_light"), ("N", "priority"), ("NW", "traffic_light"), ("mg", "priority"),
                 ("slip", "priority"), ("SW", "traffic_light"), ("S", "priority"), ("SE", "priority")]:
        p.node(f"J_{k}", J[k], t)

    def ring(a0, a1):
        return arc_points(C, r_in, ang[a0], ang[a1])

    nm = "혜화동로터리"
    # 회전 차로 (반시계). spreadType=right + 안쪽 경계 형상 → 차로가 늘면 바깥쪽에 붙는다.
    p.edge("arcS1", "J_SW", "J_S", [GEN] * 3, priority=4, shape=ring("SW", "S"), spread="right", name=nm)
    # v3: 대학로 남행 출구 ~ 대학로 북행 진입 사이 회전 차로 2차로 (그림 ④, 구역(1))
    p.edge("arcS2", "J_S", "J_SE", [GEN] * 2, priority=4, shape=ring("S", "SE"), spread="right", name=nm)
    # 5차로: 0~2 우회전(북동쪽 갈래), 3~4 직진(북쪽 회전). 3번은 대학로에서 들어온 차 + 회전 바깥 차로,
    #  4번은 회전 안쪽 차로 → 대학로에서 들어온 차가 4번으로 끼어들지 않도록 3→4 차로변경 금지
    p.edge("arcE", "J_SE", "J_NE", [GEN, GEN, GEN, (GEN, 'changeLeft="emergency"'), GEN], priority=4,
           shape=ring("SE", "NE"), spread="right", name=nm)
    p.edge("arcN1", "J_NE", "J_N", [GEN] * 3, priority=4, shape=ring("NE", "N"), spread="right", name=nm)
    # 북쪽: 바깥 정류장 공간(버스 한 대) + 회전 3차로. 3번째 차로(1번) = 혜화로 우회전 전용 (구역(3))
    p.edge("arcN2", "J_N", "J_NW", [bus_kind(BUS)] + [GEN] * 3, priority=4, shape=ring("N", "NW"),
           spread="right", name=nm)
    # 서쪽: 4차로 (모두 일반 차로 — 구역(2)). 가장 바깥(4번째) 차로는 J_mg에서 3번째 차로와 합쳐지고
    #   합쳐진 차로가 남서쪽 우회전 차로로 갈라진다 (우회전 조금 전에 미리 합류)
    p.edge("arcW1", "J_NW", "J_mg", [GEN] * 4, priority=4, shape=ring("NW", "mg"), spread="right", name=nm)
    p.edge("arcW1b", "J_mg", "J_slip", [GEN] * 3, priority=4, shape=ring("mg", "slip"), spread="right", name=nm)
    p.edge("arcW2", "J_slip", "J_SW", [GEN] * 2, priority=4, shape=ring("slip", "SW"), spread="right", name=nm)
    # 남서쪽 우회전 차로 (한 차로, 섬으로 갈라져 창경궁로 남서쪽 바깥 차로로 나감)
    r_slip0 = r_in + 2 * LW                                  # 우회전 차로 안쪽 경계 (회전 차로와 경계)
    sl1 = [polar(C, r_slip0 + (R["slip_radius"] - LW / 2 - r_slip0) * i / 4.0,
                 ang["slip"] + (ang["notch"] - ang["slip"]) * i / 4.0) for i in range(5)]
    p.edge("slip1", "J_slip", "J_SW", [GEN], priority=2, shape=sl1, spread="right",
           name="혜화동로터리 우회전", sidewalk=SWS)
    u = bearing_vec(R["diag_bearing"])

    # 대학로 ↔ 로터리 (로터리 남쪽 횡단보도 RS 바로 뒤)
    RS_y = cfg.STA["RS"]
    # 횡단보도(RS) 앞뒤는 대학로와 같은 방향으로 곧게 → RS는 횡단보도 폭만큼만 교차로 (남행 정지선 = 횡단보도 바로 북쪽)
    p.edge("dae_nb_in", "RS", "J_SE", [GEN] * 4, priority=1, name="대학로", sidewalk=SWM,
           shape=[(0.0, RS_y), (0.0, RS_y + 8.0), polar(C, r_in + LW, ang["SE"] - 3)])
    p.edge("dae_sb_out", "J_S", "RS", [GEN] * 3, priority=3, name="대학로", sidewalk=SWM,
           shape=[polar(C, r_in + LW, ang["S"] + 2), (-LW, RS_y + 8.0), (-LW, RS_y)])

    # 창경궁로 축 (회전교차로 한가운데를 가로지름). 남서행 가장 안쪽 = 중앙 버스전용차로(실선)
    p.edge("diagNE", "J_SW", "J_NE", [GEN] * 2, priority=3, name="창경궁로")
    p.edge("diagSW", "J_NE", "J_SW", [GEN] * 3 + [bus_kind(BUS)], priority=3, name="창경궁로")

    # 남서쪽 갈래 (창경궁로 → 원남동): 횡단보도는 J_SW 바로 앞
    endSW = p.node("SW_end", add(J["SW"], u, -R["leg_len"]["SW"]))
    p.edge("SW_in", endSW, "J_SW", [GEN] * 3, name="창경궁로", sidewalk=SWS)
    p.edge("SW_out", "J_SW", endSW, [GEN] * 4 + [bus_kind(BUS)], name="창경궁로", sidewalk=SWS)
    p.boundary["SW"] = ("SW_in", "SW_out")
    p.crossing("J_SW", ["SW_in", "SW_out"], True, cfg.CROSSING_WIDTH["leg"])

    # 북동쪽 갈래 (창경궁로 → 한성대입구): 횡단보도는 J_NE 바로 앞
    endNE = p.node("NE_end", add(J["NE"], u, R["leg_len"]["NE"]))
    p.edge("NE_in", endNE, "J_NE", [GEN] * 5 + [bus_kind(BUS)], name="창경궁로", sidewalk=SWS)
    p.edge("NE_out", "J_NE", endNE, [GEN] * 4, name="창경궁로", sidewalk=SWS)
    p.boundary["NE"] = ("NE_in", "NE_out")
    p.crossing("J_NE", ["NE_in", "NE_out"], True, cfg.CROSSING_WIDTH["leg"])

    # 북쪽 갈래 (창경궁로35길, 신호 없음): 회전교차로 입구 횡단보도 (보행자 우선, 교차로 경계)
    vN = bearing_vec(R["N_bearing"])
    endN = p.node("NL_end", add(J["N"], vN, R["leg_len"]["N"]))
    p.edge("NL_in", endN, "J_N", [GEN], priority=1, name="창경궁로35길", sidewalk=SWS)
    p.edge("NL_out", "J_N", endN, [GEN], priority=1, name="창경궁로35길", sidewalk=SWS)
    p.boundary["NL"] = ("NL_in", "NL_out")
    p.crossing("J_N", ["NL_in", "NL_out"], True, cfg.CROSSING_WIDTH["side"])

    # 북서쪽 갈래 (혜화로): 신호 횡단보도 (교차로 경계). 신호는 혜화로로 나가는 차(우회전)와 들어오는 차에만,
    #   회전 차로 안쪽 2차로는 신호 없이 그대로 회전 (구역(3))
    vW = bearing_vec(R["NW_bearing"])
    endNW = p.node("NW_end", add(J["NW"], vW, R["leg_len"]["NW"]))
    p.edge("NW_in", endNW, "J_NW", [GEN], priority=2, name="혜화로", sidewalk=SWS)
    p.edge("NW_out", "J_NW", endNW, [GEN], priority=2, name="혜화로", sidewalk=SWS)
    p.boundary["NW"] = ("NW_in", "NW_out")
    p.crossing("J_NW", ["NW_in", "NW_out"], True, cfg.CROSSING_WIDTH["leg"])

    # ── 연결 ──
    nb_last, sb_first = cor["nb_in"]("RS"), cor["sb_out"]("RS")
    p.conn(nb_last, "dae_nb_in", [0, 1, 2, 3], [0, 1, 2, 3])
    p.conn("dae_sb_out", sb_first, [0, 1, 2], [0, 1, 2])
    # J_S: 대학로 남행으로 나가기(바깥 0·1번) / 계속 회전(1·2번 → 2차로)
    p.conn("arcS1", "dae_sb_out", [0, 1], [0, 1])
    p.conn("arcS1", "arcS2", [1, 2], [0, 1])
    # J_SE: 대학로 북행 합류. 바깥 0~2번은 회전 차로와 멀리 떨어져 양보 없이 진행.
    #   가장 안쪽 3번은 회전 바깥 차로(arcS2 0번)와 같은 차로로 합류하므로 회전 차량에 양보
    p.conn("dae_nb_in", "arcE", [0, 1, 2], [0, 1, 2], 'pass="true"')
    p.conn("dae_nb_in", "arcE", 3, 3)
    p.conn("arcS2", "arcE", [0, 1], [3, 4])
    # J_NE (신호): 회전 → 북동쪽 갈래(우회전) / 북쪽 회전, 창경궁로 축
    p.conn("arcE", "arcN1", [3, 4], [0, 1])
    p.conn("arcE", "NE_out", [0, 1, 2], [0, 1, 2], CP)
    p.conn("diagNE", "NE_out", [0, 1], [2, 3])
    p.conn("NE_in", "diagSW", [2, 3, 4, 5], [0, 1, 2, 3])
    p.conn("NE_in", "arcN1", [0, 1], [0, 1])
    # J_N: 창경궁로35길 (북쪽 정류장 공간은 J_N 뒤에서 시작 → 버스만 차로변경으로 진입)
    p.conn("arcN1", "arcN2", [0, 1, 2], [1, 2, 3])
    p.conn("arcN1", "NL_out", 0, 0)
    p.conn("NL_in", "arcN2", 0, [1, 2], CP)    # 1번 = 혜화로 우회전, 2번 = 계속 회전
    # J_NW (신호는 혜화로 진출·진입만): 정류장 공간·3번째 차로(1번) → 혜화로, 안쪽 2차로는 신호 없이 회전
    p.conn("arcN2", "NW_out", [0, 1], [0, 0], CP)
    p.conn("arcN2", "arcW1", [2, 3], [2, 3], UNC)
    p.conn("NW_in", "arcW1", 0, [0, 1], CP)
    # J_mg: 가장 바깥(0번) 차로 끝 → 차로변경으로 1번에 합류
    p.conn("arcW1", "arcW1b", [1, 2, 3], [0, 1, 2])
    # J_slip: 바깥 차로 → 우회전 차로, 안쪽 두 차로 → 계속 회전
    p.conn("arcW1b", "slip1", 0, 0)
    p.conn("arcW1b", "arcW2", [1, 2], [0, 1])
    # J_SW (신호)
    p.conn("arcW2", "arcS1", [0, 1], [1, 2])
    p.conn("SW_in", "arcS1", 0, 0)
    p.conn("SW_in", "diagNE", [1, 2], [0, 1])
    p.conn("diagSW", "SW_out", [0, 1, 2, 3], [1, 2, 3, 4], CP)
    p.conn("slip1", "SW_out", 0, 0, CP)
    return J


# ──────────────────────────────────────────────────────────────
# 옆길 출구의 차로별 양보 (v3)
#   SUMO(netconvert)는 옆길에서 우회전해 나오는 차가 같은 도로로 가는 "모든 차로"의 차에 양보하게 만든다.
#   실제 운전자는 자기가 건너거나 합류하는 차로(버스차로·2차로)만 보므로, 아래 교차로에서는 그보다 왼쪽 차로로
#   직진하는 차에 대한 양보를 뺀다 (netconvert --check-lane-foes 규칙과 같음, 보행자 양보는 그대로).
#   이화장길 출구가 거의 열리지 않아 어린이 보호구역 서행 차로에 줄이 늘 서 있던 문제(부모 차량이 못 나감) 해결.
# ──────────────────────────────────────────────────────────────
LANE_LEVEL_YIELD = ("IWJ",)


def relax_side_exit_yield(net_path, junctions=LANE_LEVEL_YIELD):
    root = ET.parse(net_path).getroot()
    via = {}
    for c in root.findall("connection"):
        if c.get("via") and not c.get("from").startswith(":"):
            via[c.get("via")] = (c.get("to"), int(c.get("toLane")), c.get("state"))
    text = open(net_path, encoding="utf-8").read()
    changed = 0
    for j in root.findall("junction"):
        if j.get("id") not in junctions or j.get("type") != "priority":
            continue
        info = [via.get(lane) for lane in j.get("intLanes").split()]   # request index = intLanes 순서
        m = re.search(r'<junction id="%s" .*?</junction>' % re.escape(j.get("id")), text, re.S)
        block = m.group(0)
        for r in j.findall("request"):
            i = int(r.get("index"))
            me = info[i] if i < len(info) else None
            if me is None or me[2] != "m":
                continue
            resp = list(r.get("response"))      # 오른쪽 끝 글자 = link 0
            n = len(resp)
            for k in range(min(n, len(info))):
                o = info[k]
                if resp[n - 1 - k] == "1" and o is not None and o[0] == me[0] and o[1] > me[1]:
                    resp[n - 1 - k] = "0"
                    changed += 1
            block = re.sub(r'(<request index="%d" response=")[01]+"' % i,
                           lambda mm: mm.group(1) + "".join(resp) + '"', block)
        text = text[:m.start()] + block + text[m.end():]
    with open(net_path, "w", encoding="utf-8") as f:
        f.write(text)
    return changed


# ──────────────────────────────────────────────────────────────
# 3) 신호 현시 (net.xml에서 차량·보행 연결의 linkIndex와 상충 관계를 읽어 상태 문자를 만든다)
# ──────────────────────────────────────────────────────────────
def parse_tls(net_path):
    root = ET.parse(net_path).getroot()
    cross_edges, cross_len = {}, {}
    for e in root.findall("edge"):
        if e.get("function") == "crossing":
            cross_edges[e.get("id")] = e.get("crossingEdges", "").split()
            ln = e.find("lane")
            cross_len[e.get("id")] = float(ln.get("length")) if ln is not None else 0.0
    junc = {}
    for j in root.findall("junction"):
        ints = j.get("intLanes", "").split()
        reqs = {int(r.get("index")): (r.get("response"), r.get("foes")) for r in j.findall("request")}
        junc[j.get("id")] = (ints, reqs)
    tls = collections.defaultdict(lambda: {"links": {}})
    for c in root.findall("connection"):
        tl = c.get("tl")
        if not tl:
            continue
        idx = int(c.get("linkIndex"))
        frm, to = c.get("from"), c.get("to")
        if frm.startswith(":"):                      # 보행: walkingarea → crossing
            info = dict(kind="ped", crossing=to, edges=cross_edges.get(to, []), via=to + "_0",
                        frm=frm, to=to, dir="s", length=cross_len.get(to, 0.0))
        else:
            info = dict(kind="veh", frm=frm, to=to, fl=int(c.get("fromLane")), tl=int(c.get("toLane")),
                        dir=c.get("dir"), via=c.get("via"))
        info["idx"] = idx
        tls[tl]["links"][idx] = info
    for tl, d in tls.items():
        ints, reqs = junc.get(tl, ([], {}))
        # 신호 연결 → 교차로 request 번호 (내부 차로 순서). 신호 없는(uncontrolled) 연결도 request에는 있음
        req2idx = {}
        for idx, inf in d["links"].items():
            if inf["via"] in ints:
                req2idx[ints.index(inf["via"])] = idx
        if len(req2idx) != len(d["links"]):
            raise RuntimeError(f"{tl}: 신호 연결 {len(d['links'])}개 중 {len(req2idx)}개만 교차로 내부 차로와 맞음")
        foes, resp = set(), set()
        for k, (r, f) in reqs.items():
            if k not in req2idx:
                continue
            i = req2idx[k]
            for m, ch in enumerate(reversed(f)):
                if ch == "1" and m in req2idx:
                    foes.add((i, req2idx[m]))
            for m, ch in enumerate(reversed(r)):
                if ch == "1" and m in req2idx:
                    resp.add((i, req2idx[m]))
        d["foes"], d["resp"] = foes, resp
        d["n"] = max(d["links"]) + 1
    return tls


def make_state(T, rule):
    s = ["r"] * T["n"]
    for idx, info in T["links"].items():
        s[idx] = rule(info)
    # 보행 녹색과 엇갈리는 차량 녹색은 양보(g): 우회전·좌회전 차가 횡단 중인 보행자에게 양보
    peds = [i for i, inf in T["links"].items() if inf["kind"] == "ped" and s[i] == "G"]
    for i, inf in T["links"].items():
        if inf["kind"] == "veh" and s[i] == "G" and any((i, c) in T["foes"] for c in peds):
            s[i] = "g"
    return "".join(s)


def _trans(T, prev, nxt, final):
    """현시 사이 황색(final=False)·전적색(final=True) 상태. 보행은 바로 적색(보행 금지)."""
    out = []
    for k, (a, b) in enumerate(zip(prev, nxt)):
        ped = T["links"][k]["kind"] == "ped"
        if a in "Gg" and b in "Gg":
            out.append(a if a == b else "g")
        elif ped:
            out.append("r")
        elif a in "Gg":
            out.append("r" if final else "y")
        elif a == "s" and (not final or b in "sGg"):
            out.append("s")
        else:
            out.append("r")
    return "".join(out)


def _sequence(T, phases, yellow, all_red):
    """phases: [(상태, 초)]. 차량 녹색이 끝나는 곳에만 황색·전적색을 넣는다
    (보행 녹색 → 보행 점멸(적색)은 차량 신호가 그대로라 끼워 넣지 않음)."""
    out = []
    for i, (st, dur) in enumerate(phases):
        out.append((st, dur))
        nxt = phases[(i + 1) % len(phases)][0]
        veh_end = any(T["links"][k]["kind"] == "veh" and a in "Gg" and b not in "Ggy"
                      for k, (a, b) in enumerate(zip(st, nxt)))
        if veh_end:
            out.append((_trans(T, st, nxt, False), yellow))
            if all_red:
                out.append((_trans(T, st, nxt, True), all_red))
    return out


def check_unsafe(T, state):
    """같은 현시에 서로 엇갈리는 차량 연결이 둘 다 G이면 (양보 관계 없이) 경고."""
    bad = []
    for (i, j) in T["foes"]:
        if i < j and state[i] == "G" and state[j] == "G":
            if (i, j) in T["resp"] or (j, i) in T["resp"]:
                continue
            a, b = T["links"][i], T["links"][j]
            if a["kind"] == "veh" and b["kind"] == "veh":
                bad.append((i, j))
    return bad


def ped_split(net, T, total, speed=1.25, min_walk=5, only=None):
    """보행 현시를 보행 녹색(walk) + 보행 점멸(clear)로 나눈다. clear = 가장 긴 횡단보도 길이 / 걷는 속도.
    only: 이 횡단보도(crossing id)만 고려."""
    L = 0.0
    for inf in T["links"].values():
        if inf["kind"] == "ped" and (only is None or inf["crossing"] in only):
            L = max(L, inf["length"])
    clear = int(math.ceil(L / speed))
    clear = max(4, min(clear, total - min_walk))
    return total - clear, clear


def build_signals(net, tls, out_path, cor):
    sig = cfg.SIGNAL
    Y, AR, CYC, RC = sig["yellow"], sig["all_red"], sig["cycle"], sig["rotary_cycle"]
    S = cfg.STA
    progs, warn, info = [], [], {}

    def add_prog(tid, phases, start, cyc=CYC, ar=None):
        """phases 중 초가 None인 현시 하나는 주기에 맞게 나머지 시간으로 채운다."""
        T = tls[tid]
        AR = sig["all_red"] if ar is None else ar
        fixed = [(st, d if d is not None else 0) for st, d in phases]
        tot0 = sum(d for _st, d in _sequence(T, fixed, Y, AR))
        rest = cyc - tot0
        if any(d is None for _st, d in phases):
            if rest < 5:
                raise RuntimeError(f"{tid}: 주기 {cyc}초 안에 현시가 들어가지 않음 (남는 시간 {rest})")
            phases = [(st, d if d is not None else rest) for st, d in phases]
        seq = _sequence(T, phases, Y, AR)
        tot = sum(d for _st, d in seq)
        if tot != cyc:
            raise RuntimeError(f"{tid} 주기 {tot} != {cyc}")
        for st, _d in phases:
            for i, j in check_unsafe(T, st):
                warn.append(f"{tid}: {T['links'][i]['frm']}→{T['links'][i]['to']} / "
                            f"{T['links'][j]['frm']}→{T['links'][j]['to']}")
        lines = [f'    <tlLogic id="{tid}" type="static" programID="daehakro" offset="{int(round(start)) % cyc}">']
        for st, d in seq:
            lines.append(f'        <phase duration="{d}" state="{st}"/>')
        lines.append("    </tlLogic>")
        progs.append("\n".join(lines))
        return phases

    # ── 이화사거리: 신호①~④ 방향별 단독 신호. 4갈래 횡단보도는 각각 그 갈래로 "우회전"만 들어가는 신호 때 보행 ──
    T = tls["IH"]
    NBo, SBi = cor["nb_out"]("IH"), cor["sb_in"]("IH")
    approach = {"W": "W_in", "S": "S_in", "E": "E_in1", "N": SBi}
    leg_edges = {"N": {NBo, SBi}, "E": {"E_in1", "E_out"}, "W": {"W_in", "W_out"}, "S": {"S_in", "S_out"}}

    def leg_of(inf):
        for leg, es in leg_edges.items():
            if set(inf["edges"]) & es:
                return leg
        return None
    appr_leg = {v: k for k, v in approach.items()}            # 진입 edge → 갈래
    ped_with = sig["ihwa_ped"]                                  # 신호(진입 갈래) → 보행 갈래
    lead = sig["ihwa_ped_lead"]
    lpi_s = sig.get("ihwa_ped_head_start", 4)

    # 신호별로 녹색 차량이 들어가는 차로 (적색 우회전이 같은 차로로 합류하면 녹색 차를 막으므로 그 신호 동안 금지)
    green_targets = {a: {(inf["to"], inf["tl"]) for inf in T["links"].values()
                         if inf["kind"] == "veh" and inf["frm"] == approach[a]} for a in approach}

    def ih_rule(appr, part):
        walk_leg = ped_with[appr]

        def rule(inf):
            if inf["kind"] == "ped":
                return "G" if (part in ("lpi", "walk") and leg_of(inf) == walk_leg) else "r"
            if inf["frm"] == approach[appr]:
                if part == "lefty" and inf["dir"] in ("l", "L", "t"):
                    return "y"                        # 좌회전·유턴은 3초 먼저 황색 (교차로가 커서 늦게 들어간 차가 빠져나가게)
                if part in ("lead", "lpi") and inf["dir"] in ("r", "R"):
                    # 보행 녹색 처음 몇 초(보행자 먼저 출발)까지 이 신호의 우회전은 정지,
                    # 그 뒤로는 보행자에게 양보하며 우회전(make_state가 g로)
                    return "r"
                return "G"
            if inf["dir"] in ("r", "R"):
                own = appr_leg.get(inf["frm"])        # 이 우회전 차가 들어올 때 지나는 자기 갈래 횡단보도
                if own == walk_leg:
                    return "r"                        # 자기 갈래 횡단보도가 보행인 신호 동안은 적색 우회전도 금지
                if (inf["to"], inf["tl"]) in green_targets[appr]:
                    return "r"                        # 녹색 차(좌회전·직진)와 같은 차로로 합류하는 적색 우회전 금지
                return "s"                            # 적색 우회전 (일시정지 후, 보행자 양보)
            return "r"
        return rule
    phases = []
    ih_info = []
    for _num, appr, g in sig["ihwa_phases"]:
        walk_leg = ped_with[appr]
        cids = [inf["crossing"] for inf in T["links"].values()
                if inf["kind"] == "ped" and leg_of(inf) == walk_leg]
        _w, clear = ped_split(net, T, g - lead, only=cids)
        walk = max(6, min(12, g - lead - clear - 2))
        post = g - lead - walk - clear
        if post < 0:
            raise RuntimeError(f"이화사거리 신호 {appr}: 녹색 {g}초에 보행({walk}+{clear}) 안 들어감")
        lpi = min(lpi_s, walk - 1)
        ly = 3                                        # 좌회전·유턴 먼저 황색 (마지막 3초, 보행 점멸 끝과 겹쳐도 됨)
        post2, clear2 = post - min(post, ly), clear - max(0, ly - post)
        for part, dur in [("lead", lead), ("lpi", lpi), ("walk", walk - lpi), ("clear", clear2), ("post", post2),
                          ("lefty", ly)]:
            if dur > 0:
                phases.append((make_state(T, ih_rule(appr, part)), dur))
        ih_info.append(f"{walk_leg}{walk}+{clear}")
    add_prog("IH", phases, 0, ar=sig.get("all_red_ih", AR))
    info["IH"] = " ".join(ih_info)

    ped = sig["ped_green"]
    vp = sig["progression_speed"]

    def midblock(tid, start, special=None):
        T = tls[tid]
        main_cross = [inf["crossing"] for inf in T["links"].values()
                      if inf["kind"] == "ped" and any(e.startswith(("nb_", "sb_")) for e in inf["edges"])]
        walk, clear = ped_split(net, T, ped, only=main_cross)
        info[tid] = (walk, clear)

        def mk(phase):
            def rule(inf):
                if special:
                    r = special(inf, phase)
                    if r:
                        return r
                if inf["kind"] == "ped":
                    return "G" if phase == "walk" else "r"
                return "G" if phase == "main" else "r"
            return make_state(T, rule)
        add_prog(tid, [(mk("main"), None), (mk("walk"), walk), (mk("clear"), clear)], start)
        return start + (CYC - ped - (Y + AR)) + Y + AR      # 보행 녹색 시작 시각

    # M: 남쪽 횡단보도 + 북쪽 옆길 N (N 입구 횡단보도는 신호 없음 = 항상 보행자 우선)
    def m_special(inf, phase):
        if inf["kind"] == "ped" and "N_in" in inf["edges"]:
            return "G"
        return None
    midblock("M", S["M"] / vp, m_special)
    midblock("H", S["H"] / vp)
    midblock("F", S["F"] / vp)
    t_ped_E = midblock("E", S["E"] / vp)
    midblock("C", S["C"] / vp)

    # D: 남행 좌회전·유턴만 신호 (E 보행 신호 녹색+점멸 때 — 그때 북행은 E 남쪽에 서 있음). D 입구 횡단보도는 신호 없음
    T = tls["D"]

    def d_state(phase):
        def rule(inf):
            if inf["kind"] == "ped":
                return "G"
            return "g" if phase == "turn" else "r"
        return make_state(T, rule)
    # 좌회전·유턴 녹색 = E 보행 녹색 시작 4초 뒤 ~ E 보행 끝 4초 전 (E 황색에 지나간 북행 차가 D를 빠져나가고,
    # 다시 출발한 북행 차가 D에 닿기 전에 좌회전·유턴 차가 다 지나가도록)
    d_turn = ped - 8
    d_main = CYC - d_turn - (Y + AR)
    add_prog("D", [(d_state("main"), None), (d_state("turn"), d_turn)], t_ped_E + 4 - d_main)

    # O: 본선 → 옆길 O 진출(좌·우회전) → 횡단보도 K 보행(녹색+점멸). O 입구 횡단보도는 본선 녹색 동안
    T = tls["O"]
    g_side = sig["side_green"]["O"]
    nb_o = cor["nb_in"]("O")
    walk_o, clear_o = ped_split(net, T, ped)
    info["O"] = (walk_o, clear_o)
    o_clear = int(math.ceil(3 * LW / 1.25)) - (Y + AR) + 3            # O 입구 3차로 횡단 시간 - 황색·전적색 + 여유
    o_lead = 3

    def o_state(phase):
        def rule(inf):
            if inf["kind"] == "ped":
                if "O_in" in inf["edges"]:
                    return "G" if phase == "main" else "r"
                return "G" if phase == "walk" else "r"
            f, t = inf["frm"], inf["to"]
            if phase == "lead":
                return "r"
            if f == "O_in":
                if phase == "side":
                    return "G"
                if phase in ("main_end", "clear_late") and inf["dir"] in ("r", "R"):
                    # 적색 우회전 (일시정지 후 양보): 본선 녹색 마지막 6초(O 입구 보행 적색)와 K 보행 점멸 후반.
                    # 정류장(K~I 사이)에 버스가 3대 몰리면 셋째 버스 꽁무니가 옆길 녹색 동안 교차로에 걸려
                    # O에서 나오는 우회전 차가 두 주기씩 기다리던 문제 → 버스가 정류장으로 들어간 뒤 나갈 기회
                    return "s"
                return "r"
            if f == nb_o and t == "O_out":
                return "G" if phase in ("main", "main_end") else "s"   # 적색 때 일시정지 후 우회전
            return "G" if phase in ("main", "main_end") else "r"
        return make_state(T, rule)
    t_o = S["O"] / vp
    add_prog("O", [(o_state("main"), None), (o_state("main_end"), o_clear), (o_state("side"), g_side),
                   (o_state("lead"), o_lead), (o_state("walk"), walk_o), (o_state("clear"), clear_o - clear_o // 2),
                   (o_state("clear_late"), clear_o // 2)], t_o)

    # I: 본선(I 입구 보행) → 옆길 I 우회전 진출 + 북행 좌회전(1차로 전용, 보호 좌회전만 → 꼬리물기 없음)
    T = tls["I"]
    g_side_i = sig["side_green"]["I"]
    nb_i = cor["nb_in"]("I")

    def i_state(phase):
        def rule(inf):
            if inf["kind"] == "ped":
                return "G" if phase == "main" else "r"
            f, t = inf["frm"], inf["to"]
            if f == "I_in":
                return "G" if phase == "side" else "r"   # 옆길 녹색 때만 (I 입구 보행자는 그때 적색)
            if f == nb_i and t == "I_out":
                return "G" if phase == "side" else "r"   # 좌회전은 옆길 녹색 때만 (비보호 좌회전 없음)
            return "G" if phase == "main" else "r"
        return make_state(T, rule)
    g_main_i = CYC - g_side_i - 2 * (Y + AR)
    add_prog("I", [(i_state("main"), None), (i_state("side"), g_side_i)], t_o + 20 - (g_main_i + Y + AR))

    # ── 로터리 (75초 주기) ──
    rg = sig["rotary"]
    gne = rg["J_NE"]
    T = tls["J_NE"]
    walk_ne, clear_ne = ped_split(net, T, gne["ring_ped"])
    info["J_NE"] = (walk_ne, clear_ne)

    def jne_state(phase):
        def rule(inf):
            if inf["kind"] == "ped":
                return "G" if phase == "walk" else "r"
            f, t = inf["frm"], inf["to"]
            if f == "arcE":
                if t == "NE_out":
                    return "G" if phase == "ring" else "r"
                return "G" if phase in ("ring", "walk", "clear") else "r"
            return "G" if phase == "diag" else "r"
        return make_state(T, rule)
    t_ne = 0
    add_prog("J_NE", [(jne_state("ring"), gne["ring"]), (jne_state("walk"), walk_ne),
                      (jne_state("clear"), clear_ne), (jne_state("diag"), None)], t_ne, RC)
    diag_ne = t_ne + gne["ring"] + gne["ring_ped"] + 2 * (Y + AR)       # J_NE 창경궁로 축 녹색 시작

    gsw = rg["J_SW"]
    T = tls["J_SW"]
    walk_sw, clear_sw = ped_split(net, T, gsw["ring"])
    info["J_SW"] = (walk_sw, clear_sw)

    def jsw_state(phase):
        def rule(inf):
            if inf["kind"] == "ped":
                return "G" if phase in ("walk0", "walk") else "r"
            f = inf["frm"]
            if f == "slip1":
                return "r" if phase == "walk0" else "g"
            if f == "arcW2":
                return "G" if phase in ("walk0", "walk", "clear") else "r"
            return "G" if phase == "diag" else "r"
        return make_state(T, rule)
    t_sw = diag_ne + sig["ring_travel"] - 2
    w0 = 5
    add_prog("J_SW", [(jsw_state("walk0"), w0), (jsw_state("walk"), walk_sw - w0), (jsw_state("clear"), clear_sw),
                      (jsw_state("diag"), None)], t_sw, RC)

    # 로터리 남쪽 횡단보도 RS: 보행 = J_NE 축 녹색 동안 (어차피 북행이 회전 차로로 못 나가는 시간)
    #   v3: 로터리에서 나오는 남행도 신호 (횡단보도 북쪽 정지선, 구역(5) 그림)
    T = tls["RS"]
    rped = sig["rotary_ped"]
    walk_rs, clear_rs = ped_split(net, T, rped)
    info["RS"] = (walk_rs, clear_rs)

    def rs_state(phase):
        def rule(inf):
            if inf["kind"] == "ped":
                return "G" if phase == "walk" else "r"
            return "G" if phase == "main" else "r"
        return make_state(T, rule)
    rs_main = RC - rped - (Y + AR)
    t_rs_ped = diag_ne + 2
    t_rs_main = t_rs_ped - (rs_main + Y + AR)
    add_prog("RS", [(rs_state("main"), None), (rs_state("walk"), walk_rs), (rs_state("clear"), clear_rs)],
             t_rs_main, RC)

    # A: 북행 정지선(A 입구 남쪽). 북행 녹색 = RS 녹색보다 2초 먼저 시작해 3초 먼저 끝남 (RS 앞에서 멈춘 차가
    #    A 교차로 안에 남지 않게). 북행 유턴 = RS 보행 신호 때 (남행이 RS 북쪽에 서 있는 동안). A 입구 횡단보도는 신호 없음
    T = tls["A"]
    nbA, sbAo = cor["nb_in"]("A"), cor["sb_out"]("A")

    def a_state(phase):
        def rule(inf):
            if inf["kind"] == "ped":
                return "G"
            if inf["frm"] == nbA and inf["to"] == sbAo:
                return "g" if phase == "uturn" else "r"
            if inf["frm"] == nbA and inf["to"] == "A_out":
                return "G" if phase == "main" else "s"   # 적색 때 일시정지 후 우회전 (입구 보행자 양보)
            if inf["frm"] == "A_in":
                return "g" if phase == "main" else "r"   # A에서 나오는 차: 앞 RS가 녹색일 때만 (양보)
            return "G" if phase == "main" else "r"
        return make_state(T, rule)
    # 유턴 녹색은 RS 남행 황색·전적색이 끝나고 1초 뒤부터 (RS를 황색에 지나온 남행 차가 A를 빠져나간 뒤),
    # RS 남행 녹색이 다시 켜지기 전 황색·전적색 + 2초에 끝남 (유턴 차가 남행 차로를 다 비우도록)
    a_main = rs_main - 1
    a_wait = 4
    add_prog("A", [(a_state("main"), a_main), (a_state("wait"), a_wait), (a_state("uturn"), None)],
             t_rs_main - 2, RC)

    # 혜화로 횡단보도 (J_NW): 신호는 혜화로로 나가는 차·들어오는 차만 (회전 차로 안쪽 2차로는 신호 없음)
    T = tls["J_NW"]
    nped = sig["nw_ped"]
    walk_nw, clear_nw = ped_split(net, T, nped)
    info["J_NW"] = (walk_nw, clear_nw)

    def nw_state(phase):
        def rule(inf):
            if inf["kind"] == "ped":
                return "G" if phase == "walk" else "r"
            return "G" if phase == "veh" else "r"
        return make_state(T, rule)
    add_prog("J_NW", [(nw_state("veh"), None), (nw_state("walk"), walk_nw), (nw_state("clear"), clear_nw)],
             t_ne, RC)

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("<additional>\n" + "\n".join(progs) + "\n</additional>\n")
    return warn, info


# ──────────────────────────────────────────────────────────────
# 4) 버스정류장 + 부모 차량 정차 공간
# ──────────────────────────────────────────────────────────────
def lane_pos_at(net, lane_id, value, axis=1):
    """차로 형상에서 좌표(axis=1: y, 0: x)가 value인 곳의 위치(m). 없으면 None."""
    lane = net.getLane(lane_id)
    shape = lane.getShape()
    pos = 0.0
    for a, b in zip(shape[:-1], shape[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if (a[axis] - value) * (b[axis] - value) <= 0 and b[axis] != a[axis]:
            return pos + seg * (value - a[axis]) / (b[axis] - a[axis])
        pos += seg
    return None


def lane_pos_at_y(net, lane_id, y):
    return lane_pos_at(net, lane_id, y, 1)


def build_stops(net, plan, out_path, cor):
    lines = ["<additional>"]
    order = cor["nb_order"]

    def nb_edge_at(s):
        for (a, na), (b, nb) in zip(order[:-1], order[1:]):
            if a <= s < b:
                return f"nb_{na}_{nb}"
        raise ValueError(s)

    for sid, a, b in cfg.NB_BUS_STOPS:
        e = nb_edge_at((a + b) / 2)
        lane = f"{e}_{plan.off(e)}"                  # 논리 0번 = 버스차로 (베이 구간은 베이)
        p0, p1 = lane_pos_at_y(net, lane, a), lane_pos_at_y(net, lane, b)
        L = net.getLane(lane).getLength()
        p0 = 0.5 if p0 is None else max(0.5, p0)
        p1 = L - 0.5 if p1 is None else min(L - 0.5, p1)
        lines.append(f'    <busStop id="{sid}" lane="{lane}" startPos="{p0:.1f}" endPos="{p1:.1f}" '
                     f'friendlyPos="true" name="{sid}"/>')
    # 로터리 북쪽 정류장 (창경궁로35길 ~ 혜화로 사이 바깥 공간)
    L = net.getLane("arcN2_0").getLength()
    lines.append(f'    <busStop id="bs_ring_n" lane="arcN2_0" startPos="{max(0.3, L - 15.0):.1f}" '
                 f'endPos="{L - 0.3:.1f}" friendlyPos="true" name="혜화동로터리 북쪽"/>')
    # 로터리 서쪽 정류장 (바깥 차로가 합쳐진 뒤 ~ 우회전 차로 갈림 사이 바깥 차로)
    L = net.getLane("arcW1b_0").getLength()
    lines.append(f'    <busStop id="bs_ring_w" lane="arcW1b_0" startPos="{max(0.3, L - 16.0):.1f}" '
                 f'endPos="{L - 1.0:.1f}" friendlyPos="true" name="혜화동로터리 서쪽"/>')
    # 부모 차량: 중학교 정문 앞 길가 정차 공간 (정문 쪽 = 서행 차로 sz1_in, 길 건너 = 동행 차로 sz1_out)
    gx = cfg.SCHOOL_GATE_X
    for pid, eid, nm in [("pa_gate_n", "sz1_in", "중학교 정문 앞 (정문 쪽)"),
                         ("pa_gate_s", "sz1_out", "중학교 정문 앞 (길 건너)")]:
        lane = f"{eid}_{plan.off(eid)}"
        L = net.getLane(lane).getLength()
        pa, pb = lane_pos_at(net, lane, gx + 5.0, 0), lane_pos_at(net, lane, gx + 5.0 + 6.0 * cfg.PARENT["capacity"], 0)
        p0, p1 = sorted([pa if pa is not None else L - 1.0, pb if pb is not None else 1.0])
        lines.append(f'    <parkingArea id="{pid}" lane="{lane}" startPos="{p0:.1f}" endPos="{p1:.1f}" '
                     f'roadsideCapacity="{cfg.PARENT["capacity"]}" friendlyPos="true" name="{nm}"/>')
    lines.append("</additional>")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


# ──────────────────────────────────────────────────────────────
# 5) 교통 수요 (오후 3시) + 보행자
# ──────────────────────────────────────────────────────────────
def route_edges(net, frm, to, vclass, via=()):
    pts = [frm] + list(via) + [to]
    path = []
    for a, b in zip(pts[:-1], pts[1:]):
        # 가장 빠른 경로(제한속도 기준): 어린이 보호구역(30km/h)을 지름길로 가로지르지 않음
        edges, _cost = net.getFastestPath(net.getEdge(a), net.getEdge(b), vClass=vclass)
        if not edges:
            return None
        ids = [e.getID() for e in edges]
        if path and ids[0] == path[-1]:
            ids = ids[1:]
        path += ids
    return path


def ped_pairs(net, cor, loc):
    """보행자 출발·도착 (from_edge, departPos, to_edge, arrivalPos) 목록 (양방향)."""
    L = lambda e: net.getEdge(e).getLength()
    near_end = lambda e, d=25.0: max(0.5, L(e) - d)
    near_start = lambda e, d=25.0: min(L(e) - 0.5, d)
    nbi, nbo, sbi, sbo = cor["nb_in"], cor["nb_out"], cor["sb_in"], cor["sb_out"]
    pairs = []

    def both(a, pa, b, pb):
        pairs.append((a, pa, b, pb))
        pairs.append((b, pb, a, pa))

    if loc in ("M", "H", "F", "RS", "C", "E"):                # 대학로 횡단보도: 북행(동) ↔ 남행(서)
        both(nbi(loc), near_end(nbi(loc)), sbo(loc), near_start(sbo(loc)))
    elif loc == "x_K":                                        # O 북쪽 횡단보도 K
        both(nbo("O"), near_start(nbo("O")), sbi("O"), near_end(sbi("O")))
    elif loc == "IH_N":
        both(nbo("IH"), near_start(nbo("IH")), sbi("IH"), near_end(sbi("IH")))
    elif loc == "IH_E":
        both("E_in1", near_end("E_in1"), "E_out", near_start("E_out"))
    elif loc == "IH_W":
        both("W_in", near_end("W_in"), "W_out", near_start("W_out"))
    elif loc == "IH_S":
        both("S_in", near_end("S_in", 15.0), "S_out", near_start("S_out"))
    elif loc in ("N", "G", "D", "C_s", "B", "A", "O", "IWJ"):  # 동쪽 보도를 따라 옆길 입구 건넘
        n = SIDE_NODE.get(loc, loc) if loc != "C_s" else "C_s"
        both(nbi(n), near_end(nbi(n)), nbo(n), near_start(nbo(n)))
    elif loc == "I":                                          # 서쪽 보도
        both(sbi("I"), near_end(sbi("I")), sbo("I"), near_start(sbo("I")))
    elif loc == "SZ_T":                                       # 이화장길 남쪽 보도 ↔ 북쪽 보도 (T자 서쪽 횡단보도)
        both("sz1_out", near_end("sz1_out", 15), "sz1_in", near_start("sz1_in", 15))
        both("br_in", near_end("br_in", 30), "sz1_in", near_start("sz1_in", 25))
    elif loc == "J_NE":
        both("NE_in", near_end("NE_in"), "NE_out", near_start("NE_out"))
    elif loc == "J_SW":
        both("SW_in", near_end("SW_in"), "SW_out", near_start("SW_out"))
    elif loc == "J_NW":
        both("NW_in", near_end("NW_in", 20), "NW_out", near_start("NW_out", 20))
    elif loc == "J_N":
        both("NL_in", near_end("NL_in", 20), "NL_out", near_start("NL_out", 20))
    else:
        raise ValueError(loc)
    return pairs


def edge_at_y(cor, y, side):
    """측점 y를 지나는 북행(side='nb')/남행('sb') 본선 edge."""
    order = cor["nb_order"] if side == "nb" else cor["sb_order"]
    for (a, na), (b, nb) in zip(order[:-1], order[1:]):
        if a <= y < b:
            return f"nb_{na}_{nb}" if side == "nb" else f"sb_{nb}_{na}"
    raise ValueError(y)


def build_routes(net, plan: Plan, out_path, stops_edges, cor):
    B = plan.boundary
    lines = ["<routes>"]
    # 일반 차량: 차량추종모델 4종 혼합
    tot = sum(v["probability"] for v in cfg.CAR_TYPES.values())
    lines.append('    <vTypeDistribution id="car">')
    cc = cfg.CAR_COMMON
    jm = (f'jmStoplineCrossingGap="{cfg.PED_YIELD_GAP}" jmCrossingGap="{cfg.PED_LOOKAHEAD}" '
          f'jmTimegapMinor="{cfg.MINOR_GAP}" jmSigmaMinor="0" jmAdvance="{cfg.JUNCTION_ADVANCE}"')
    for name, v in cfg.CAR_TYPES.items():
        attrs = [f'id="car_{name}"', f'probability="{v["probability"] / tot:.3f}"',
                 f'carFollowModel="{v["carFollowModel"]}"', f'accel="{v["accel"]}"',
                 f'decel="{v["decel"]}"', f'tau="{v["tau"]}"', f'minGap="{v["minGap"]}"',
                 f'color="{v["color"]}"', f'length="{cc["length"]}"', f'maxSpeed="{cc["maxSpeed"]}"',
                 f'speedFactor="{cc["speedFactor"]}"', f'vClass="{cc["vClass"]}"',
                 f'guiShape="{cc["guiShape"]}"', 'lcStrategic="1.0"', 'lcCooperative="1.0"',
                 # 버스전용 점선 구간은 주행 차로가 아님: 오른쪽(버스차로)으로 앞지르기·주행하러 들어가지 않음
                 # (우회전·옆길 진입을 위한 전략적 차로변경만), 오른쪽 붙어 달리기(keepRight)도 끔
                 'lcSpeedGain="0.7"', 'lcSpeedGainRight="0.02"', 'lcKeepRight="0"', 'lcAssertive="1.0"', jm]
        if "sigma" in v:
            attrs.append(f'sigma="{v["sigma"]}"')
        for k in ("gapClosingControlGainSpeed",):
            if k in v:
                attrs.append(f'{k}="{v[k]}"')
        lines.append("        <vType " + " ".join(attrs) + "/>")
    lines.append("    </vTypeDistribution>")
    bt = cfg.BUS_TYPE
    lines.append('    <vType id="bus" ' + " ".join(f'{k}="{v}"' for k, v in bt.items()) + f' {jm}/>')
    pt = cfg.PARENT["type"]
    lines.append('    <vType id="parent" vClass="passenger" guiShape="passenger/van" '
                 + " ".join(f'{k}="{v}"' for k, v in pt.items())
                 + f' lcSpeedGainRight="0.02" lcKeepRight="0" {jm}/>')
    lines.append('    <vType id="ego_type" vClass="passenger" color="1,0,0" accel="2.6" decel="4.5" '
                 f'length="4.6" maxSpeed="22" guiShape="passenger/sedan" jmStoplineCrossingGap="{cfg.PED_YIELD_GAP}" '
                 f'jmCrossingGap="{cfg.PED_LOOKAHEAD}" jmTimegapMinor="{cfg.MINOR_GAP}" '
                 f'jmAdvance="{cfg.JUNCTION_ADVANCE}"/>')
    for name, v in cfg.PED_TYPES.items():
        lines.append(f'    <vType id="ped_{name}" vClass="pedestrian" guiShape="pedestrian" '
                     + " ".join(f'{k}="{val}"' for k, val in v.items()) + "/>")

    problems = []
    flows = []
    a07 = (edge_at_y(cor, 250.0, "nb"), edge_at_y(cor, 250.0, "sb"))
    counts = {"A07_NB": 0.0, "A07_SB": 0.0, "A06_in": 0.0, "A06_out": 0.0, "SZ_in": 0.0, "SZ_out": 0.0}
    for o, dests in cfg.CAR_OD.items():
        for d, vph in dests.items():
            if vph <= 0:
                continue
            frm, to = B[o][0], B[d][1]
            r = route_edges(net, frm, to, "passenger")
            if r is None:
                problems.append(f"승용 경로 없음: {o} → {d}")
                continue
            rid = f"r_{o}_{d}"
            lines.append(f'    <route id="{rid}" edges="{" ".join(r)}"/>')
            flows.append((f"car_{o}_{d}", "car", rid, vph, ""))
            _count(counts, r, vph, a07)
    # 부모 차량: 어린이 보호구역에서 천천히(경유 지점 속도 제한) → 정문 앞 길가에 정차 → 출발
    P = cfg.PARENT
    for name, o, d, vph, side in P["flows"]:
        pe = "sz1_in" if side == "n" else "sz1_out"
        r = route_edges(net, B[o][0], B[d][1], "passenger", via=[pe])
        if r is None:
            problems.append(f"부모 차량 경로 없음: {name}")
            continue
        rid = f"r_parent_{name}"
        lines.append(f'    <route id="{rid}" edges="{" ".join(r)}"/>')
        k = r.index(pe)
        slow = []
        first = True
        for e in r[max(0, k - 4):k + 1]:                     # 정문 앞까지의 어린이 보호구역 도로
            if not e.startswith(SCHOOL_ZONE_PREFIX):
                continue
            ln = f"{e}_{plan.off(e)}"
            Le = net.getLane(ln).getLength()
            # 첫 도로는 8m 들어간 뒤부터 서행 (옆길 입구 횡단보도를 보통 속도로 지나가도록)
            start, end = (min(8.0, Le - 2.0) if first else 0.5), Le - 0.5
            first = False
            if e == pe:
                pa = _parking_start(out_path, "pa_gate_n" if side == "n" else "pa_gate_s")
                end = max(start + 1.0, pa - 1.0)
            slow.append(f'<stop lane="{ln}" startPos="{start:.1f}" endPos="{end:.1f}" speed="{P["slow_speed"]}"/>')
        n_d = len(P["stop_durations"])
        for dur in P["stop_durations"]:
            stop = (f'<stop parkingArea="{"pa_gate_n" if side == "n" else "pa_gate_s"}" '
                    f'duration="{dur}" parking="true"/>')
            flows.append((f"parent_{name}_{dur}", "parent", rid, vph / n_d, "".join(slow) + stop))
    # 자율주행차용 경로 (지금은 차량을 넣지 않고 경로만 준비)
    eo, ed = cfg.EGO_ROUTE
    r = route_edges(net, B[eo][0], B[ed][1], "passenger")
    if r:
        lines.append(f'    <route id="r_ego" edges="{" ".join(r)}"/>')
    for name, o, d, vph, stops in cfg.BUS_FLOWS:
        frm, to = B[o][0], B[d][1]
        via = [stops_edges[s] for s in stops]
        r = route_edges(net, frm, to, "bus", via)
        if r is None:
            problems.append(f"버스 경로 없음: {name}")
            continue
        rid = f"r_bus_{name}"
        lines.append(f'    <route id="{rid}" edges="{" ".join(r)}"/>')
        stop_xml = "".join(f'<stop busStop="{s}" duration="{cfg.BUS_DWELL}"/>' for s in stops)
        flows.append((f"bus_{name}", "bus", rid, vph, stop_xml))
        _count(counts, r, vph, a07)
    for fid, typ, rid, vph, stop_xml in flows:
        prob = min(1.0, vph / 3600.0)
        attrs = (f'id="{fid}" type="{typ}" route="{rid}" begin="0" end="{cfg.SIM["end"]}" '
                 f'probability="{prob:.5f}" departLane="best" departSpeed="max" '
                 f'departPos="base" arrivalPos="max"')
        if stop_xml:
            lines.append(f"    <flow {attrs}>{stop_xml}</flow>")
        else:
            lines.append(f"    <flow {attrs}/>")
    # 보행자: 횡단보도별 양방향 흐름
    n_ped = 0.0
    for name, loc, pph, ptype in cfg.PEDESTRIANS:
        key = "x_K" if name == "x_K" else loc
        pairs = ped_pairs(net, cor, key)
        rate = pph / len(pairs) / 3600.0
        for k, (a, pa, b, pb) in enumerate(pairs):
            lines.append(f'    <personFlow id="ped_{name}_{k}" type="ped_{ptype}" begin="0" '
                         f'end="{cfg.SIM["end"]}" probability="{rate:.5f}" departPos="{pa:.1f}">'
                         f'<walk from="{a}" to="{b}" arrivalPos="{pb:.1f}"/></personFlow>')
        n_ped += pph
    # 중학교 정문에서 하교하는 학생 (이화장길 북쪽 보도에서 출발)
    gate_lane = "sz1_in_0"
    gpos = net.getLane(gate_lane).getLength() - 1.0          # 정문 = 정문 앞 횡단보도 바로 동쪽 북쪽 보도
    L = lambda e: net.getEdge(e).getLength()
    dest = {"W": ("W_out", min(40.0, L("W_out") - 1)), "S": ("S_out", min(40.0, L("S_out") - 1)),
            # E: 남쪽 골목 서쪽 보도를 따라 율곡로 쪽으로 (이화사거리 동쪽 횡단보도를 건너지 않음)
            "N": (a07[0], L(a07[0]) * 0.5), "E": ("br_out", max(1.0, L("br_out") - 8.0)),
            "SZ": ("sz2_in", 5.0), "X": ("sz1_out", L("sz1_out") * 0.5)}
    for name, where, pph in cfg.GATE_PEDESTRIANS:
        e, pos = dest[where]
        lines.append(f'    <personFlow id="ped_{name}" type="ped_child" begin="0" end="{cfg.SIM["end"]}" '
                     f'probability="{pph / 3600.0:.5f}" departPos="{gpos:.1f}">'
                     f'<walk from="sz1_in" to="{e}" arrivalPos="{pos:.1f}"/></personFlow>')
        n_ped += pph
    lines.append("</routes>")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return problems, counts, n_ped


def _parking_start(rou_path, pid):
    stops = os.path.join(os.path.dirname(rou_path), "daehakro.stops.xml")
    for el in ET.parse(stops).getroot().iter("parkingArea"):
        if el.get("id") == pid:
            return float(el.get("startPos"))
    return 1.0


def _count(counts, route, vph, a07):
    rs = set(route)
    if a07[0] in rs:
        counts["A07_NB"] += vph
    if a07[1] in rs:
        counts["A07_SB"] += vph
    if "SW_in" in rs:
        counts["A06_in"] += vph
    if "SW_out" in rs:
        counts["A06_out"] += vph
    if "br_in" in rs:
        counts["SZ_in"] += vph
    if "sz2_in" in rs:
        counts["SZ_out"] += vph


# ──────────────────────────────────────────────────────────────
# 6) 전체 빌드
# ──────────────────────────────────────────────────────────────
def build(out_dir=None, verbose=True):
    base = os.path.dirname(os.path.abspath(__file__))
    out_dir = out_dir or os.path.join(base, "sumo", "daehakro")
    os.makedirs(out_dir, exist_ok=True)
    prefix = os.path.join(out_dir, "daehakro")

    plan = Plan()
    cor = build_corridor(plan)
    J = build_rotary(plan, cor)
    plan.write(prefix)

    netconvert = checkBinary("netconvert")
    cmd = [netconvert, "--node-files", prefix + ".nod.xml", "--edge-files", prefix + ".edg.xml",
           "--connection-files", prefix + ".con.xml", "-o", prefix + ".net.xml",
           "--no-turnarounds", "true", "--tls.default-type", "static",
           "--junctions.corner-detail", "5", "--junctions.limit-turn-speed", "5.5",
           "--default.lanewidth", str(LW), "--offset.disable-normalization", "true",
           "--sidewalks.guess", "false", "--crossings.guess", "false"]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError("netconvert 실패:\n" + res.stdout + res.stderr)
    # 일부러 들어오는 연결 없이 만든 차로(차로변경으로만 진입: 늘어나는 차로, 정류장 공간 등)는 경고에서 뺀다
    warnings = [l for l in (res.stdout + res.stderr).splitlines()
                if l.startswith("Warning") and "is not connected from any incoming edge" not in l]
    relax_side_exit_yield(prefix + ".net.xml")

    net = sumolib.net.readNet(prefix + ".net.xml", withPrograms=True, withConnections=True)
    tls = parse_tls(prefix + ".net.xml")
    sig_warn, sig_info = build_signals(net, tls, prefix + ".tll.xml", cor)
    build_stops(net, plan, prefix + ".stops.xml", cor)
    try:
        from env import daehakro_deco as deco
    except ImportError:
        import daehakro_deco as deco  # type: ignore
    deco_info = deco.build_deco(net, plan, cor, J, prefix + ".deco.xml", base)
    stops_edges = {}
    for line in open(prefix + ".stops.xml", encoding="utf-8"):
        if "<busStop" in line:
            sid = line.split('id="')[1].split('"')[0]
            lane = line.split('lane="')[1].split('"')[0]
            stops_edges[sid] = lane.rsplit("_", 1)[0]
    problems, counts, n_ped = build_routes(net, plan, prefix + ".rou.xml", stops_edges, cor)

    with open(prefix + ".gui.xml", "w", encoding="utf-8") as f:
        # realisticLinkRules: 우선 도로(신호 없는 직진)의 정지선 표시를 감춤 → 버스차로 점선/실선 경계처럼
        # 교차로가 아닌 곳에 흰 막대가 그려지지 않는다. 양보 연결은 흰 정지선, 신호 연결은 신호 색으로 표시.
        f.write("""<viewsettings>
    <scheme name="real world">
        <edges laneShowBorders="1" showLinkDecals="1" realisticLinkRules="1" showLinkRules="1" hideConnectors="1"/>
    </scheme>
    <viewport zoom="160" x="0" y="560" angle="0"/>
    <delay value="50"/>
</viewsettings>
""")
    with open(prefix + ".sumocfg", "w", encoding="utf-8") as f:
        f.write(f"""<configuration>
    <input>
        <net-file value="daehakro.net.xml"/>
        <route-files value="daehakro.rou.xml"/>
        <additional-files value="daehakro.tll.xml,daehakro.stops.xml,daehakro.deco.xml"/>
    </input>
    <time>
        <begin value="0"/>
        <end value="{cfg.SIM['end']}"/>
        <step-length value="{cfg.SIM['step_length']}"/>
    </time>
    <processing>
        <time-to-teleport value="300"/>
        <collision.action value="warn"/>
        <collision.check-junctions value="true"/>
        <ignore-route-errors value="true"/>
        <pedestrian.model value="striping"/>
    </processing>
    <random_number>
        <seed value="{cfg.SIM['seed']}"/>
    </random_number>
    <gui_only>
        <gui-settings-file value="daehakro.gui.xml"/>
    </gui_only>
</configuration>
""")
    if verbose:
        print(f"[daehakro] 도로망: {len(plan.nodes)} 노드, {len(plan.edges)} 엣지, "
              f"횡단보도 {len(plan.crossings)}곳 → {prefix}.net.xml")
        if warnings:
            print(f"[daehakro] netconvert 경고 {len(warnings)}건 (처음 10건)")
            for w in warnings[:10]:
                print("   ", w)
        if sig_warn:
            print(f"[daehakro] 신호 상충 경고 {len(sig_warn)}건")
            for w in sig_warn[:10]:
                print("   ", w)
        print("[daehakro] 공식 통계 단면 검산 (대/시, 버스 포함):")
        dc = cfg.DEMAND_COUNTS
        print(f"    A-07 대학로 북행 {counts['A07_NB']:.0f} (통계 {dc['A07_NB']}) / "
              f"남행 {counts['A07_SB']:.0f} (통계 {dc['A07_SB']})")
        print(f"    A-06 창경궁로 로터리 쪽 {counts['A06_in']:.0f} (통계 {dc['A06_in']}) / "
              f"반대쪽 {counts['A06_out']:.0f} (통계 {dc['A06_out']})")
        print(f"    어린이 보호구역: 남쪽 골목 우회전 진입 {counts['SZ_in']:.0f}, 이화장길 동쪽 끝에서 나옴 "
              f"{counts['SZ_out']:.0f} (+ 부모 차량 {sum(f[3] for f in cfg.PARENT['flows'])}대/시)")
        print(f"[daehakro] 보행자 {n_ped:.0f}명/시, 장식: {deco_info}")
        print("[daehakro] 보행 신호 (녹색+점멸) 초: " + ", ".join(
            f"{k} {v}" if isinstance(v, str) else f"{k} {v[0]}+{v[1]}" for k, v in sig_info.items()))
        for pr in problems:
            print("   경로 문제:", pr)
    return prefix + ".sumocfg"


if __name__ == "__main__":
    build()
