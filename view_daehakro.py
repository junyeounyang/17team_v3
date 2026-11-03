"""대학로(이화사거리 ~ 혜화동로터리) 도로 확인 스크립트.

env/daehakro_config.py 를 고친 뒤 학습 전에 도로와 교통 흐름을 확인하는 용도.

사용법:
    python view_daehakro.py              # sumo-gui로 열기 (▶ 버튼으로 재생)
    python view_daehakro.py --netedit    # netedit로 도로망 열기
    python view_daehakro.py --nogui      # 화면 없이 1시간 시뮬레이션 후 점검표 출력
    python view_daehakro.py --plot       # 도로망 그림(PNG) 저장 (env/sumo/daehakro/daehakro_*.png)

--nogui 점검표 항목
    [차량]   투입/도착, 텔레포트, 충돌(차-차 / 차-보행자), 평균 속도, 오래 멈춘 차, 투입 대기
    [버스전용차로] 실선 구간에 들어간 일반 차(0이어야 함), 점선 구간을 잠깐 쓴 일반 차와 그 이유
    [유턴]   이화사거리·D·A 등 유턴한 차량 수
    [부모 차량] 어린이 보호구역 중학교 정문 앞에 멈춰 아이를 태운 차량 수
    [보행자] 횡단보도별로 건넌 사람 수 (어린이 보호구역 포함)
"""
import argparse
import collections
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

BASE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE)

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
else:  # 수업 env와 같은 방식으로 SUMO_HOME 자동 감지
    try:
        from env.sumo_env import SumoHighwayEnv  # noqa: F401
    except Exception:
        pass
    if "SUMO_HOME" in os.environ:
        sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

import sumolib  # noqa: E402
from sumolib import checkBinary  # noqa: E402
import traci  # noqa: E402
import traci.constants as tc  # noqa: E402

from env import daehakro_builder  # noqa: E402
from env import daehakro_config as cfg  # noqa: E402

# 횡단보도 위치 이름 (노드 → 설명)
PLACE = {
    "IH": "이화사거리", "IWJ": "이화장길 입구(어린이 보호구역)", "SZ_G": "중학교 정문 앞(어린이 보호구역)",
    "SZ_X": "이화장길 T자 서쪽(어린이 보호구역)",
    "M": "M", "O": "O·K", "I": "옆길 I", "H": "H", "G": "옆길 G", "F": "F(마로니에공원)",
    "E": "E(혜화역)", "D": "옆길 D", "C": "C", "C_s": "옆길 C", "B": "옆길 B", "A": "옆길 A", "RS": "로터리 남쪽",
    "J_NE": "로터리 북동", "J_SW": "로터리 남서", "J_NW": "로터리 혜화로", "J_N": "로터리 창경궁로35길",
}


STUCK_SEC = 180   # 이보다 오래 계속 멈춘 차가 있으면 "점검 필요" (신호 주기 150초 + 여유)


def lane_kinds(net):
    """차로 ID → 'solid'(버스만) / 'dash'(버스+승용, 점선) / 'gen'."""
    kind = {}
    for e in net.getEdges(withInternal=True):
        for ln in e.getLanes():
            if ln.allows("bus") and not ln.allows("passenger"):
                kind[ln.getID()] = "solid" if not ln.allows("pedestrian") else "walk"
            elif ln.allows("bus") and ln.allows("passenger") and not ln.allows("truck"):
                kind[ln.getID()] = "dash"
            else:
                kind[ln.getID()] = "gen"
    return kind


def validate(sumocfg, seconds, seed):
    out_dir = os.path.dirname(sumocfg)
    net = sumolib.net.readNet(os.path.join(out_dir, "daehakro.net.xml"), withInternal=True,
                              withPedestrianConnections=True)
    kind = lane_kinds(net)
    to_node = {e.getID(): e.getToNode().getID() for e in net.getEdges()}
    crossing_place = {}
    for el in ET.parse(os.path.join(out_dir, "daehakro.net.xml")).getroot().iter("edge"):
        if el.get("function") == "crossing":
            eid = el.get("id")
            node = eid[1:eid.rindex("_c")]
            ces = el.get("crossingEdges", "").split()
            crosses_main = any(c.startswith(("nb_", "sb_")) for c in ces)
            label = PLACE.get(node, node)
            if node == "IH":                              # 이화사거리 4갈래
                leg = ("북쪽" if crosses_main else "동쪽" if any(c.startswith("E_") for c in ces)
                       else "서쪽" if any(c.startswith("W_") for c in ces) else "남쪽")
                label += " " + leg
            elif node in ("O", "M") and not crosses_main:
                label = {"O": "옆길 O 입구", "M": "옆길 N 입구"}[node]
            crossing_place[eid] = label

    col_file = tempfile.NamedTemporaryFile(prefix="daehakro_col_", suffix=".xml", delete=False).name
    cmd = [checkBinary("sumo"), "-c", sumocfg, "--no-step-log", "true", "--seed", str(seed),
           "--end", str(seconds), "--no-warnings", "true", "--duration-log.statistics", "false",
           "--collision-output", col_file]
    traci.start(cmd)
    step = cfg.SIM["step_length"]
    warm = min(cfg.SIM["warmup"], seconds / 3)
    t = 0.0
    inserted = arrived = teleports = 0
    p_dep = p_arr = 0
    long_wait = {}
    halting = collections.Counter()
    speed_sum = speed_n = 0.0
    n_veh_max = 0
    VARS = [tc.VAR_LANE_ID, tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION, tc.VAR_VEHICLECLASS]
    state = {}
    uturn = collections.Counter()
    solid_cars = set()
    dash = collections.Counter()
    dash_dist = collections.defaultdict(list)
    ped_seen = {}
    ped_cross = collections.Counter()
    parked = set()                     # 정문 앞에 멈춰 아이를 태운 부모 차량
    park_t = {}                        # 부모 차량: [정차 공간에서 처음 본 시각, 마지막 본 시각]
    park_max = 0
    parent_dep = 0
    while t < seconds:
        traci.simulationStep()
        t = traci.simulation.getTime()
        for v in traci.simulation.getDepartedIDList():
            traci.vehicle.subscribe(v, VARS)
        for p in traci.simulation.getDepartedPersonIDList():
            traci.person.subscribe(p, [tc.VAR_ROAD_ID])
        parent_dep += sum(1 for v in traci.simulation.getDepartedIDList() if v.startswith("parent_"))
        if int(t / step) % int(2 / step) == 0:
            occ = 0
            for pa in ("pa_gate_n", "pa_gate_s"):
                ids = traci.parkingarea.getVehicleIDs(pa)
                parked.update(ids)
                occ += len(ids)
                for v in ids:
                    park_t.setdefault(v, [t, t])[1] = t
            park_max = max(park_max, occ)
        inserted += traci.simulation.getDepartedNumber()
        arrived += traci.simulation.getArrivedNumber()
        teleports += traci.simulation.getStartingTeleportNumber()
        p_dep += len(traci.simulation.getDepartedPersonIDList())
        p_arr += traci.simulation.getArrivedPersonNumber()
        # 차량: 유턴, 버스전용차로 사용
        for v, r in traci.vehicle.getAllSubscriptionResults().items():
            lane, road, pos = r[tc.VAR_LANE_ID], r[tc.VAR_ROAD_ID], r[tc.VAR_LANEPOSITION]
            if road.startswith(":"):
                continue
            st = state.setdefault(v, {"edge": None, "lane": None, "dash": None})
            if st["edge"] and st["edge"] != road:
                a, b = st["edge"], road
                if (a[:3], b[:3]) in (("sb_", "nb_"), ("nb_", "sb_")):
                    uturn[to_node.get(a, "?")] += 1
            st["edge"] = road
            if r[tc.VAR_VEHICLECLASS] != "passenger":
                continue
            k = kind.get(lane, "gen")
            if k == "solid":
                solid_cars.add(v)
            prev = st["lane"]
            if k == "dash":
                d = st["dash"]
                if d is None:
                    if prev is None:
                        how = "출발"
                    elif prev.rsplit("_", 1)[0] == road:
                        how = "차로변경"
                    elif prev.startswith(("nb_", "S_in")):
                        how = "앞 교차로 지나"
                    else:
                        how = "옆길·교차로 우회전"
                    st["dash"] = {"how": how, "dist": 0.0, "pos": pos, "road": road}
                else:
                    d["dist"] += max(0.0, pos - d["pos"]) if road == d["road"] else max(0.0, pos)
                    d["pos"], d["road"] = pos, road
            elif st["dash"] is not None:
                d = st["dash"]
                if road == d["road"]:
                    out = "차로변경으로 나감"
                elif road.startswith("nb_"):
                    out = "그대로 직진"
                else:
                    out = "우회전(옆길·교차로)"
                dash[(d["how"], out)] += 1
                dash_dist[(d["how"], out)].append(d["dist"])
                st["dash"] = None
            st["lane"] = lane
        # 보행자: 건넌 횡단보도
        for p, r in traci.person.getAllSubscriptionResults().items():
            road = r[tc.VAR_ROAD_ID]
            if road in crossing_place and ped_seen.get(p) != road:
                ped_cross[crossing_place[road]] += 1
            ped_seen[p] = road
        if int(t / step) % int(10 / step) == 0:
            ids = traci.vehicle.getIDList()
            n_veh_max = max(n_veh_max, len(ids))
            for v in ids:
                w = traci.vehicle.getWaitingTime(v)
                if w > 90:
                    long_wait[v] = (w, traci.vehicle.getRoadID(v))
                if t > warm:
                    speed_sum += traci.vehicle.getSpeed(v)
                    speed_n += 1
            if t > warm:
                for e in traci.edge.getIDList():
                    if not e.startswith(":"):
                        halting[e] += traci.edge.getLastStepHaltingNumber(e)
    pending_ids = traci.simulation.getPendingVehicles()     # 출발 시각이 됐지만 도로가 막혀 못 들어온 차
    backlog = len(pending_ids)
    pending = collections.Counter(v.split(".")[0] for v in pending_ids)
    # 신호 한 주기(150초)를 넘겨 계속 멈춰 있는 차 (옆길은 적색이 약 125초라 그보다 넉넉히)
    stuck_now = [(v, traci.vehicle.getWaitingTime(v), traci.vehicle.getRoadID(v))
                 for v in traci.vehicle.getIDList() if traci.vehicle.getWaitingTime(v) > STUCK_SEC]
    traci.close()
    col_vv = col_vp = 0
    try:
        for c in ET.parse(col_file).getroot().findall("collision"):
            if c.get("type") == "crossing" or c.get("victimType", "").startswith("ped"):
                col_vp += 1
            else:
                col_vv += 1
        os.remove(col_file)
    except Exception:
        pass
    samples = max(1, int((seconds - warm) / 10))

    print("─" * 66)
    print(f" 대학로 점검 ({seconds:.0f}초, step {step}s, seed {seed})")
    print("─" * 66)
    print(f" [차량] 투입 {inserted}대 / 도착 {arrived}대 / 동시 최대 {n_veh_max}대 / 투입 대기 {backlog}대")
    if pending:
        print("        투입 대기가 많은 흐름:", ", ".join(f"{k} {n}" for k, n in pending.most_common(5)))
    print(f"        텔레포트 {teleports}회 / 충돌 {col_vv + col_vp}건 (차-차 {col_vv}, 차-보행자 접촉 {col_vp})")
    print(f"        평균 속도 {speed_sum / max(1, speed_n) * 3.6:.1f} km/h (앞 {warm:.0f}초 제외)")
    print(f"        90초 넘게 멈춘 적 있는 차 {len(long_wait)}대 (이화사거리 주기 150초라 적색 대기만으로도 생김), "
          f"지금 {STUCK_SEC}초 넘게 계속 멈춘 차 {len(stuck_now)}대")
    print(f" [버스전용차로] 실선 구간에 들어간 일반 차: {len(solid_cars)}대 (0이어야 정상)")
    n_dash = sum(dash.values())
    print(f"        점선 구간을 잠깐 쓴 일반 차: {n_dash}대")
    for (how, out), n in dash.most_common():
        ds = sorted(dash_dist[(how, out)])
        med = ds[len(ds) // 2] if ds else 0.0
        print(f"          {how} → {out}: {n}대 (버스차로 안에서 달린 거리 중앙값 {med:.0f}m)")
    print(" [유턴] " + (", ".join(f"{PLACE.get(k, k)} {n}대" for k, n in uturn.most_common()) or "없음"))
    print(f" [부모 차량] 출발 {parent_dep}대 / 중학교 정문 앞에 멈춰 아이를 태움 {len(parked)}대 "
          f"(동시에 최대 {park_max}대 정차)")
    # 정해진 정차 시간(40·60·80초)이 끝난 뒤 다시 도로로 나오기까지 더 기다린 시간 (서행 차로에 줄이 서 있으면 생김)
    extra = sorted(max(0.0, last - first + 2 - float(v.rsplit("_", 1)[1].split(".")[0]))
                   for v, (first, last) in park_t.items() if last < t - 2)
    if extra:
        print(f"        정차 시간이 끝난 뒤 다시 출발하기까지 더 기다린 시간: 중앙값 {extra[len(extra) // 2]:.0f}초, "
              f"최대 {extra[-1]:.0f}초 (60초 넘음 {sum(e > 60 for e in extra)}대)")
    print(f" [보행자] 출발 {p_dep}명 / 도착 {p_arr}명. 횡단보도별 건넌 사람(명):")
    items = sorted(ped_cross.items(), key=lambda x: -x[1])
    for i in range(0, len(items), 4):
        print("          " + "  ".join(f"{k} {n}" for k, n in items[i:i + 4]))
    print(" [정지 차량이 많은 곳] (10초 간격 평균)")
    for e, h in halting.most_common(6):
        print(f"          {e:20s} {h / samples:5.1f}대")
    if stuck_now:
        print("  오래 멈춘 차량 예시:", stuck_now[:5])
    rare = (col_vv + col_vp) <= max(3, round(5 * seconds / 3600))
    ok = teleports == 0 and len(stuck_now) == 0 and len(solid_cars) == 0 and rare
    print(" 판정:", "정상 (텔레포트·장시간 정체·버스 실선 침범 없음, 충돌 기록 드묾)" if ok else "점검 필요")
    if col_vv + col_vp:
        print("   ※ 충돌 기록은 SUMO 모델의 한계로 드물게(시간당 0~3건) 생기는 '접촉' 판정입니다: 옆길 입구 횡단보도에서\n"
              "     막 들어선 보행자와 차의 모서리, 긴 버스 꽁무니가 교차로에 걸친 채 서 있을 때 옆길에서 우회전해 나오는 차.\n"
              "     (collision.action=warn 이라 차·보행자는 그대로 진행하고 시뮬레이션에는 영향 없음)")
    return ok


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="대학로 도로를 GUI로 확인하거나 점검")
    ap.add_argument("--netedit", action="store_true")
    ap.add_argument("--nogui", action="store_true")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--seconds", type=int, default=cfg.SIM["end"])
    ap.add_argument("--seed", type=int, default=cfg.SIM["seed"])
    args = ap.parse_args()

    sumocfg = daehakro_builder.build()
    net_file = os.path.join(os.path.dirname(sumocfg), "daehakro.net.xml")
    if args.plot:
        subprocess.run([sys.executable, os.path.join(BASE, "utils", "plot_daehakro.py")], check=False)
    elif args.netedit:
        subprocess.run([checkBinary("netedit"), net_file])
    elif args.nogui:
        validate(sumocfg, args.seconds, args.seed)
    else:
        print("sumo-gui 실행 중... ▶ 버튼을 누르면 차량이 움직입니다.")
        subprocess.run([checkBinary("sumo-gui"), "-c", sumocfg])
