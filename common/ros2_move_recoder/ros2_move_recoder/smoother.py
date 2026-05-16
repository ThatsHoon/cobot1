"""
ros2_move_recoder.smoother — 평활화 + 정지 구간 제거 + 균일 속도 재생용 다운샘플링
============================================================================
raw.json → Savitzky-Golay → 정지 구간 압축 → arc-length 등간격 다운샘플링
       → smooth.json (amovesj vel/acc 로 균일 속도 비동기 재생)

* 단일 source of truth — `smooth_and_save()` 함수가 핵심 로직.
  - GUI (gui.py) 가 직접 import 해서 사용
  - CLI (`ros2 run ros2_move_recoder smoother`) 의 main() 도 동일 함수 사용

* gripper_widths_mm 필드가 raw.json 에 있으면 → smooth.json 에 gripper_events 추출.
  자세한 내용은 dev-docs/gripper.md, dev-docs/data-formats.md 참조.

CLI 사용:
  ros2 run ros2_move_recoder smoother <name> [--window N] [--polyorder K]
                                              [--max-pts M] [--eps E] [--prom P]
                                              [--vel V] [--acc A]
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

RECORDS_DIR = Path("~/cobot_ws/src/ros2_move_recoder/records").expanduser()


def smooth_and_save(action_name: str, window: int, polyorder: int,
                    max_pts: int, vel: float, acc: float,
                    stationary_eps_deg: float = 0.5,
                    turning_prominence_deg: float = 2.0,
                    ) -> dict:
    """
    평활화 + 정지 구간 제거 + 방향 전환점 보존 + 균일 속도 재생용 다운샘플링.

    1. Savitzky-Golay 로 펜던트 stepwise 노이즈 제거
    2. 정지 구간 압축: 인접 샘플 6축 변화 < eps 면 연속 정지로 간주, 묶어서 1점만 유지
    3. 방향 전환점 검출: 각 관절의 peak/trough (prominence ≥ 임계값) 인덱스를 강제 보존
    4. arc-length 등간격 다운샘플링 + 전환점 union
    5. 재생은 amovesj(vel, acc) — 시간 정보 무시, 균일 속도, 비동기

    추가:
    * raw.json 의 `gripper_widths_mm` 가 있으면 gripper_events 를 arc-length 진행률
      기반 t_norm 으로 추출해 smooth.json 에 저장 (dev-docs/gripper.md 참조)
    * raw.json 의 `duration_sec` 을 record_duration_sec 으로 보존 — player timeline
      이 grip events 분배 시 vel scaling baseline 으로 사용
    """
    from scipy.signal import savgol_filter, find_peaks

    action_dir = RECORDS_DIR / action_name
    raw_p = action_dir / "raw.json"
    if not raw_p.is_file():
        raise FileNotFoundError(f"raw.json 없음: {raw_p}")

    raw = json.loads(raw_p.read_text())
    joints = np.array(raw["joints_deg"], dtype=float)
    n_raw = len(joints)
    if n_raw < 5:
        raise ValueError(f"샘플이 너무 적습니다 ({n_raw}개)")

    # ── 그리퍼 width 변화 → events 추출 (raw 에 있으면) ──
    # ⚠ t_norm 은 사람 시연 시간이 아닌 **arc-length 누적 진행률** 기준.
    #   사람이 그리퍼 조작하느라 로봇 정지하면 정지 구간이 압축돼 amovesj 시간이
    #   훨씬 짧아짐. 사람 시간 비례 t_norm 은 이 mismatch 로 events 가 모션 끝
    #   이후 시점이 되어 미발사. arc-length 진행률은 amovesj 의 균일 vel 진행과
    #   정확히 일치.
    gripper_events = []
    g_widths = raw.get("gripper_widths_mm")
    if g_widths and len(g_widths) == n_raw:
        # raw 전체 sample 의 arc-length 누적 (정지 구간 포함)
        raw_diffs = np.linalg.norm(np.diff(joints, axis=0), axis=1)
        raw_cum = np.concatenate([[0.0], np.cumsum(raw_diffs)])
        raw_total = float(raw_cum[-1])
        use_arc = raw_total >= 1.0   # 1° 이상 움직였으면 arc 기반

        # ⚠ raw 의 측정 width 는 fingertip/grip 막힘으로 부정확할 수 있음.
        #   사용자 의도 (close/open) 는 변화 방향으로 분류해서 player 가 □ 와
        #   동일한 함수 (gripper.close/open) 호출 — 가장 신뢰성 높은 동작.
        OPEN_CLOSE_TH = 5.0   # width 변화 ≥5mm 이면 의도된 동작
        last_w = None
        for i, w in enumerate(g_widths):
            if w is None:
                continue
            should_add = False
            kind = 'move'
            if last_w is None:
                # 첫 event — 초기 상태. width 절대값으로 분류
                should_add = True
                kind = 'close' if w <= 20.0 else ('open' if w >= 50.0 else 'move')
            elif abs(w - last_w) >= OPEN_CLOSE_TH:
                # 변화 방향이 사용자 의도
                should_add = True
                if w > last_w:
                    kind = 'open'
                elif w < last_w:
                    kind = 'close'
            if should_add:
                if use_arc:
                    t_norm = float(raw_cum[i]) / raw_total
                else:
                    t_norm = i / max(1, n_raw - 1)
                gripper_events.append({
                    "t_norm": round(t_norm, 4),
                    "kind": kind,
                    "width_mm": round(float(w), 2),
                })
                last_w = w
        # 마지막 width final state 처리:
        # - events 비어있으면 final 만 추가
        # - 직전 event 와 width 차이 < 5mm = 같은 모터 동작의 정착값 → width 만 update,
        #   ⚠ t_norm 은 그대로 유지 (직전 event 의 명령 발사 시점 정보 보존).
        #   (이전 버그: t_norm=max(prev,1.0) 으로 강제 늘렸더니 raw 마지막에 모션 없는
        #    구간의 close event 가 t_norm=1.0 으로 밀려 모션 끝 후 시점에 발사됨.)
        # - 차이 ≥ 5mm = 다른 동작 → 별개 event 추가
        if g_widths[-1] is not None:
            final_w = round(float(g_widths[-1]), 2)
            if not gripper_events:
                gripper_events.append({"t_norm": 1.0, "width_mm": final_w})
            else:
                last = gripper_events[-1]
                if abs(last["width_mm"] - final_w) < 5.0:
                    # 같은 동작의 정착값 — width 만 update, t_norm 보존
                    last["width_mm"] = final_w
                else:
                    gripper_events.append({"t_norm": 1.0, "width_mm": final_w})

        # ⚠ 인접 events 사이 최소 gap 강제 — OnRobot RG2 모터 full 동작 ~2s.
        #   raw 의 마지막 부분이 정지 (사람이 그리퍼만 조작) 면 arc-length t_norm
        #   이 모두 1.0 근처로 압축됨. 1.0 clamp 제거 — t_norm > 1.0 허용 (player
        #   timeline 이 모션 끝 후 시점에도 grace 안에 발사).
        if len(gripper_events) >= 2:
            raw_dur_sec = float(raw.get("duration_sec", 1.0)) or 1.0
            min_gap_norm = 2.0 / max(0.5, raw_dur_sec)
            for i in range(1, len(gripper_events)):
                gap = gripper_events[i]["t_norm"] - gripper_events[i-1]["t_norm"]
                if gap < min_gap_norm:
                    gripper_events[i]["t_norm"] = round(
                        gripper_events[i-1]["t_norm"] + min_gap_norm, 4)

    # ── 1) 자동 윈도우 + Savitzky-Golay
    auto_w = max(window, min(n_raw // 10 | 1, 201))
    if auto_w % 2 == 0:
        auto_w += 1
    if n_raw < auto_w:
        auto_w = max(5, n_raw - (1 - n_raw % 2))
        if auto_w % 2 == 0:
            auto_w -= 1
    p = polyorder if auto_w > polyorder else max(1, auto_w - 2)
    smoothed = savgol_filter(joints, window_length=auto_w, polyorder=p,
                             axis=0, mode="nearest")
    smoothed[0]  = joints[0]
    smoothed[-1] = joints[-1]

    # ── 2) 정지 구간 제거: 누적 이동거리 기준 — eps 이하로 변하지 않으면 skip
    keep_idx = [0]
    last_kept = smoothed[0]
    for i in range(1, n_raw - 1):
        if np.linalg.norm(smoothed[i] - last_kept) >= stationary_eps_deg:
            keep_idx.append(i)
            last_kept = smoothed[i]
    keep_idx.append(n_raw - 1)
    keep_idx = np.array(keep_idx, dtype=int)
    moving = smoothed[keep_idx]
    n_after_stationary = len(moving)

    # ── 3) 방향 전환점 검출 (각 관절의 peaks/troughs)
    turning_idx = set()
    for j in range(6):
        sig = moving[:, j]
        if sig.max() - sig.min() < turning_prominence_deg:
            continue   # 거의 변화 없는 관절은 skip
        peaks, _   = find_peaks( sig, prominence=turning_prominence_deg)
        troughs, _ = find_peaks(-sig, prominence=turning_prominence_deg)
        turning_idx.update(peaks.tolist())
        turning_idx.update(troughs.tolist())
    n_turning = len(turning_idx)

    # ── 4) arc-length 등간격 다운샘플링 + 전환점 union
    if n_after_stationary <= max_pts and n_turning == 0:
        wp = moving
        idx_final = np.arange(n_after_stationary)
    else:
        diffs = np.linalg.norm(np.diff(moving, axis=0), axis=1)
        cum = np.concatenate([[0.0], np.cumsum(diffs)])
        total = cum[-1]
        if total <= 1e-9:
            idx_final = np.array([0, n_after_stationary - 1])
        else:
            n_target = max(2, min(max_pts, 100) - n_turning)
            targets = np.linspace(0, total, n_target)
            equi_idx = np.searchsorted(cum, targets)
            all_idx = set(equi_idx.tolist()) | turning_idx | {0, n_after_stationary - 1}
            idx_final = np.array(sorted(all_idx), dtype=int)
            idx_final = np.clip(idx_final, 0, n_after_stationary - 1)
            idx_final = np.unique(idx_final)
        wp = moving[idx_final]

    # ── 5) 점프 검증 + 저장
    max_jump = (float(np.linalg.norm(np.diff(wp, axis=0), axis=1).max())
                if len(wp) >= 2 else 0.0)

    wp_list = np.round(wp, 4).tolist()
    record_duration = float(raw.get("duration_sec", 0.0))

    out = action_dir / "smooth.json"
    smooth_data = {
        "action_name": action_name,
        "waypoints_deg": wp_list,
        "vel": vel,
        "acc": acc,
        "record_duration_sec": round(record_duration, 3),
        "n_waypoints": len(wp_list),
        "n_raw": n_raw,
        "n_after_stationary_filter": n_after_stationary,
        "n_turning_points": n_turning,
        "stationary_eps_deg": stationary_eps_deg,
        "turning_prominence_deg": turning_prominence_deg,
        "max_adjacent_jump_deg": round(max_jump, 3),
        "smoothing": {
            "window": int(auto_w), "polyorder": int(p),
            "max_pts": max_pts,
            "downsample": "stationary-removed + arc-length + turning-points",
        },
    }
    if gripper_events:
        smooth_data["gripper_events"] = gripper_events
    out.write_text(json.dumps(smooth_data, indent=2))

    joint_ranges = [(float(wp[:, i].min()), float(wp[:, i].max()))
                    for i in range(6)]
    avg_jump = (float(np.linalg.norm(np.diff(wp, axis=0), axis=1).mean())
                if len(wp) >= 2 else 0.0)

    return {
        "n_raw": n_raw,
        "n_after_stationary": n_after_stationary,
        "n_turning": n_turning,
        "n_waypoints": len(wp_list),
        "window": int(auto_w),
        "polyorder": int(p),
        "max_jump": round(max_jump, 3),
        "avg_jump": round(avg_jump, 3),
        "stationary_eps": stationary_eps_deg,
        "turning_prominence": turning_prominence_deg,
        "joint_ranges": joint_ranges,
        "wp_first": wp_list[0],
        "wp_last": wp_list[-1],
        "out_path": str(out),
        "n_gripper_events": len(gripper_events),
    }


def main():
    """CLI 진입점 — `smooth_and_save` 의 thin wrapper."""
    ap = argparse.ArgumentParser()
    ap.add_argument("name", help="액션 이름 (records/<name>/)")
    ap.add_argument("--window", type=int, default=51,
                    help="Savgol 최소 윈도우 (자동 확장됨)")
    ap.add_argument("--polyorder", type=int, default=3)
    ap.add_argument("--max-pts", type=int, default=80,
                    help="다운샘플링 최대 waypoint (≤100)")
    ap.add_argument("--eps", type=float, default=0.5,
                    help="정지 판정 임계 [deg L2 norm]")
    ap.add_argument("--prom", type=float, default=2.0,
                    help="방향 전환점 prominence 임계 [°]")
    ap.add_argument("--vel", type=float, default=30.0)
    ap.add_argument("--acc", type=float, default=60.0)
    args = ap.parse_args()

    try:
        r = smooth_and_save(
            args.name,
            window=args.window,
            polyorder=args.polyorder,
            max_pts=args.max_pts,
            vel=args.vel,
            acc=args.acc,
            stationary_eps_deg=args.eps,
            turning_prominence_deg=args.prom,
        )
    except FileNotFoundError as e:
        print(f"❌ {e}")
        sys.exit(1)
    except ValueError as e:
        print(f"❌ {e}")
        sys.exit(1)

    print(f"[smoother] 원본 {r['n_raw']} 샘플 → "
          f"정지 압축 후 {r['n_after_stationary']} → "
          f"waypoint {r['n_waypoints']}")
    print(f"[smoother] Savgol window={r['window']}, polyorder={r['polyorder']}")
    print(f"[smoother] 방향 전환점: {r['n_turning']}")
    print(f"[smoother] 인접 점프: 평균 {r['avg_jump']}°  max {r['max_jump']}°")
    if r['n_gripper_events'] > 0:
        print(f"[smoother] 그리퍼 events: {r['n_gripper_events']}")
    print(f"[smoother] 저장: {r['out_path']}")


if __name__ == "__main__":
    main()
