from __future__ import annotations

import copy
import json
from pathlib import Path

import numpy as np


def filter_motion_frame(
    input_path: Path,
    output_path: Path,
    *,
    accel_noise_mps2: float = 0.8,
    measurement_noise_m: float = 0.015,
    gate_mahalanobis: float = 25.0,
    hand_alpha: float = 0.35,
    hand_hold_s: float = 0.25,
) -> dict:
    frames = [json.loads(line) for line in input_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not frames:
        raise ValueError(f"motion_frame is empty: {input_path}")

    timestamps = np.asarray([int(frame["timestamp_monotonic_ns"]) for frame in frames], dtype=np.int64)
    head_positions = _state_positions(frames, "head")
    wrist_positions = _state_positions(frames, "wrist")
    filtered_head, head_filter_stats = _kalman_rts_positions(
        timestamps, head_positions, accel_noise_mps2, measurement_noise_m, gate_mahalanobis
    )
    filtered_wrist, wrist_filter_stats = _kalman_rts_positions(
        timestamps, wrist_positions, accel_noise_mps2, measurement_noise_m, gate_mahalanobis
    )

    out_frames = copy.deepcopy(frames)
    for index, frame in enumerate(out_frames):
        if frame.get("head") is not None and np.all(np.isfinite(filtered_head[index])):
            frame["head"]["position"] = filtered_head[index].tolist()
            _update_matrix_translation(frame["head"], filtered_head[index])
        if frame.get("wrist") is not None and np.all(np.isfinite(filtered_wrist[index])):
            raw_wrist = wrist_positions[index]
            frame["wrist"]["position"] = filtered_wrist[index].tolist()
            _update_matrix_translation(frame["wrist"], filtered_wrist[index])
            if np.all(np.isfinite(raw_wrist)):
                _shift_hands(frame.get("hands") or [], filtered_wrist[index] - raw_wrist)
        source = frame.setdefault("source", {})
        source["filter"] = {
            "type": "constant_velocity_kalman_rts_with_innovation_gating",
            "input": str(input_path),
            "accel_noise_mps2": accel_noise_mps2,
            "measurement_noise_m": measurement_noise_m,
            "gate_mahalanobis": gate_mahalanobis,
            "note": "Raw measurements are preserved in motion_frame.jsonl; this file is a filtered derivative.",
        }

    _filter_hands(out_frames, alpha=hand_alpha, hold_s=hand_hold_s)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(json.dumps(frame, separators=(",", ":")) for frame in out_frames) + "\n", encoding="utf-8")
    return {
        "input": str(input_path),
        "output": str(output_path),
        "frames": len(out_frames),
        "accel_noise_mps2": accel_noise_mps2,
        "measurement_noise_m": measurement_noise_m,
        "gate_mahalanobis": gate_mahalanobis,
        "hand_alpha": hand_alpha,
        "hand_hold_s": hand_hold_s,
        "head": head_filter_stats | _jump_stats(head_positions, filtered_head),
        "wrist": wrist_filter_stats | _jump_stats(wrist_positions, filtered_wrist),
    }


def _state_positions(frames: list[dict], key: str) -> np.ndarray:
    values = np.full((len(frames), 3), np.nan, dtype=np.float64)
    for index, frame in enumerate(frames):
        state = frame.get(key)
        if state is None:
            continue
        position = state.get("position")
        if len(position or []) == 3:
            values[index] = np.asarray(position, dtype=np.float64)
    return values


def _kalman_rts_positions(
    timestamps_ns: np.ndarray,
    measurements: np.ndarray,
    accel_noise_mps2: float,
    measurement_noise_m: float,
    gate_mahalanobis: float,
) -> tuple[np.ndarray, dict]:
    n = len(measurements)
    if n == 0:
        return measurements.copy(), _empty_filter_stats()
    valid_indices = np.flatnonzero(np.all(np.isfinite(measurements), axis=1))
    if len(valid_indices) == 0:
        return measurements.copy(), _empty_filter_stats()

    q = float(accel_noise_mps2) ** 2
    r = max(float(measurement_noise_m), 1e-6) ** 2
    gate = max(float(gate_mahalanobis), 0.0)
    use_gate = gate > 0.0
    x_f = np.zeros((n, 6), dtype=np.float64)
    p_f = np.zeros((n, 6, 6), dtype=np.float64)
    x_p = np.zeros((n, 6), dtype=np.float64)
    p_p = np.zeros((n, 6, 6), dtype=np.float64)
    f_mats = np.repeat(np.eye(6, dtype=np.float64)[None, :, :], n, axis=0)
    accepted = np.zeros(n, dtype=bool)
    rejected = np.zeros(n, dtype=bool)
    mahalanobis_values: list[float] = []

    first = int(valid_indices[0])
    x = np.zeros(6, dtype=np.float64)
    x[:3] = measurements[first]
    p = np.diag([r, r, r, 1.0, 1.0, 1.0]).astype(np.float64)

    for i in range(n):
        if i > 0:
            dt = _dt_s(timestamps_ns[i - 1], timestamps_ns[i])
            f = np.eye(6, dtype=np.float64)
            f[:3, 3:] = np.eye(3) * dt
            g = np.zeros((6, 3), dtype=np.float64)
            g[:3] = np.eye(3) * (0.5 * dt * dt)
            g[3:] = np.eye(3) * dt
            process = g @ (np.eye(3) * q) @ g.T
            x = f @ x
            p = f @ p @ f.T + process
            f_mats[i] = f
        x_p[i] = x
        p_p[i] = p
        if np.all(np.isfinite(measurements[i])):
            h = np.zeros((3, 6), dtype=np.float64)
            h[:, :3] = np.eye(3)
            innovation = measurements[i] - h @ x
            s = h @ p @ h.T + np.eye(3) * r
            inv_s = np.linalg.inv(s)
            mahalanobis = float(innovation.T @ inv_s @ innovation)
            mahalanobis_values.append(mahalanobis)
            if use_gate and mahalanobis > gate:
                rejected[i] = True
            else:
                k = p @ h.T @ inv_s
                x = x + k @ innovation
                p = (np.eye(6) - k @ h) @ p
                accepted[i] = True
        x_f[i] = x
        p_f[i] = p

    x_s = x_f.copy()
    for i in range(n - 2, -1, -1):
        inv_pred = np.linalg.pinv(p_p[i + 1])
        c = p_f[i] @ f_mats[i + 1].T @ inv_pred
        x_s[i] = x_f[i] + c @ (x_s[i + 1] - x_p[i + 1])

    out = measurements.copy()
    out[:, :] = x_s[:, :3]
    stats = {
        "measurements": int(len(valid_indices)),
        "accepted": int(np.count_nonzero(accepted)),
        "rejected_by_innovation_gate": int(np.count_nonzero(rejected)),
        "missing": int(n - len(valid_indices)),
    }
    if mahalanobis_values:
        values = np.asarray(mahalanobis_values, dtype=np.float64)
        stats.update(
            {
                "innovation_mahalanobis_median": float(np.median(values)),
                "innovation_mahalanobis_p95": float(np.percentile(values, 95)),
                "innovation_mahalanobis_max": float(np.max(values)),
            }
        )
    return out, stats


def _empty_filter_stats() -> dict:
    return {
        "measurements": 0,
        "accepted": 0,
        "rejected_by_innovation_gate": 0,
        "missing": 0,
    }


def _dt_s(prev_ns: int, curr_ns: int) -> float:
    return max(1e-3, min((int(curr_ns) - int(prev_ns)) / 1e9, 0.2))


def _shift_hands(hands: list[dict], delta: np.ndarray) -> None:
    for hand in hands:
        for item in hand.get("landmarks", []):
            if item.get("world") is None:
                continue
            item["world"] = (np.asarray(item["world"], dtype=np.float64) + delta).tolist()


def _update_matrix_translation(state: dict, position: np.ndarray) -> None:
    matrix = state.get("matrix")
    if not isinstance(matrix, list) or len(matrix) < 3:
        return
    for axis in range(3):
        if isinstance(matrix[axis], list) and len(matrix[axis]) >= 4:
            matrix[axis][3] = float(position[axis])


def _jump_stats(raw: np.ndarray, filtered: np.ndarray) -> dict:
    return {
        "raw_jump": _position_jump_stats(raw),
        "filtered_jump": _position_jump_stats(filtered),
    }


def _position_jump_stats(values: np.ndarray) -> dict:
    valid = np.all(np.isfinite(values), axis=1)
    jumps = []
    prev = None
    for index, ok in enumerate(valid):
        if not ok:
            continue
        point = values[index]
        if prev is not None:
            jumps.append(float(np.linalg.norm(point - prev)))
        prev = point
    if not jumps:
        return {"count": 0}
    arr = np.asarray(jumps, dtype=np.float64)
    return {
        "count": int(len(arr)),
        "median_m": float(np.median(arr)),
        "p95_m": float(np.percentile(arr, 95)),
        "max_m": float(np.max(arr)),
    }


def _filter_hands(frames: list[dict], *, alpha: float, hold_s: float) -> None:
    alpha = float(np.clip(alpha, 0.0, 1.0))
    last_hands: list[dict] = []
    last_timestamp_ns: int | None = None
    for frame in frames:
        timestamp_ns = int(frame["timestamp_monotonic_ns"])
        hands = frame.get("hands") or []
        if hands:
            if last_hands and alpha < 1.0:
                _smooth_hand_landmarks(hands[0], last_hands[0], alpha)
            last_hands = copy.deepcopy(hands)
            last_timestamp_ns = timestamp_ns
            frame["hands"] = hands
            continue
        if last_hands and last_timestamp_ns is not None:
            gap_s = (timestamp_ns - last_timestamp_ns) / 1e9
            if gap_s <= hold_s:
                held = copy.deepcopy(last_hands)
                for hand in held:
                    hand["held_from_previous_frame"] = True
                frame["hands"] = held
                continue
        frame["hands"] = []


def _smooth_hand_landmarks(hand: dict, previous: dict, alpha: float) -> None:
    previous_points = {
        int(item["index"]): item["world"]
        for item in previous.get("landmarks", [])
        if item.get("world") is not None
    }
    for item in hand.get("landmarks", []):
        if item.get("world") is None:
            continue
        old = previous_points.get(int(item.get("index", -1)))
        if old is None:
            continue
        item["world"] = [
            float(old[axis]) * (1.0 - alpha) + float(item["world"][axis]) * alpha
            for axis in range(3)
        ]
    hand["landmark_filtered"] = True
