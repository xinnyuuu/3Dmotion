#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import json
import math
import statistics
import sys
import time
from collections import Counter
from dataclasses import asdict
from datetime import datetime
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from packages.imu_ble_bridge.wt901 import (  # noqa: E402
    DEFAULT_SAMPLE_RATE_HZ,
    WT901BleClient,
    WT901SerialAdapterClient,
    ImuSample,
    scan_serial_adapter_devices,
    scan_wt_devices,
    write_jsonl_sample,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Live-capture a short WT901 IMU segment and report timestamp/data quality."
    )
    parser.add_argument("--transport", choices=["ble", "serial-adapter"], default="serial-adapter")
    parser.add_argument("--serial-port", help="Serial adapter port, e.g. /dev/ttyACM1.")
    parser.add_argument("--serial-baudrate", type=int, default=115200)
    parser.add_argument("--serial-passive", action="store_true", help="Only read an already-connected serial adapter stream; do not send AT+SCAN/AT+CONNECT.")
    parser.add_argument("--address", help="BLE MAC address of the IMU.")
    parser.add_argument("--adapter-device-index", type=int, help="Use a serial adapter scan index instead of --address.")
    parser.add_argument("--sensor-id", default="head_imu")
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--scan-timeout-s", type=float, default=8.0)
    parser.add_argument("--connect-timeout-s", type=float, default=20.0)
    parser.add_argument(
        "--timestamp-mode",
        choices=["host-receive", "reconstructed-rate"],
        default=None,
        help="Default: reconstructed-rate for serial-adapter, host-receive for BLE.",
    )
    parser.add_argument(
        "--aux-poll",
        action="store_true",
        help="Poll optional mag/quaternion registers. Off by default for stable accel/gyro timing.",
    )
    parser.add_argument("--sample-rate-hz", type=float, default=DEFAULT_SAMPLE_RATE_HZ)
    parser.add_argument("--output", help="Output JSONL path. Default: /tmp/3dmotion_imu_check_<sensor>_<time>.jsonl")
    parser.add_argument("--json", action="store_true", help="Print only JSON summary.")
    parser.add_argument("--fail-on-warn", action="store_true", help="Exit nonzero for WARN as well as FAIL.")
    parser.add_argument("--min-rate-ratio", type=float, default=0.90)
    parser.add_argument("--warn-gap-ms", type=float, default=20.0)
    parser.add_argument("--fail-gap-ms", type=float, default=50.0)
    parser.add_argument("--warn-p99-ms", type=float, default=10.0)
    parser.add_argument("--fail-p99-ms", type=float, default=20.0)
    parser.add_argument("--gravity-min-mps2", type=float, default=7.5)
    parser.add_argument("--gravity-max-mps2", type=float, default=12.0)
    args = parser.parse_args()

    if args.duration_s <= 0:
        parser.error("--duration-s must be > 0")
    if args.sample_rate_hz <= 0:
        parser.error("--sample-rate-hz must be > 0")

    output = Path(args.output) if args.output else _default_output(args.sensor_id)
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        output.unlink()

    samples: list[dict] = []

    def on_sample(sample: ImuSample) -> None:
        record = asdict(sample)
        samples.append(record)
        write_jsonl_sample(output, sample)

    try:
        _capture(args, on_sample)
    except Exception as exc:
        summary = {
            "status": "FAIL",
            "output": str(output),
            "error": str(exc),
            "hint": _connection_hint(args),
            "issues": [{"severity": "FAIL", "message": str(exc)}],
        }
        _print_summary(summary, json_only=args.json)
        raise SystemExit(2) from exc

    summary = analyze_imu_samples(
        samples,
        output=output,
        duration_s=args.duration_s,
        expected_rate_hz=args.sample_rate_hz,
        min_rate_ratio=args.min_rate_ratio,
        warn_gap_ms=args.warn_gap_ms,
        fail_gap_ms=args.fail_gap_ms,
        warn_p99_ms=args.warn_p99_ms,
        fail_p99_ms=args.fail_p99_ms,
        gravity_min_mps2=args.gravity_min_mps2,
        gravity_max_mps2=args.gravity_max_mps2,
    )
    _print_summary(summary, json_only=args.json)

    if summary["status"] == "FAIL" or (args.fail_on_warn and summary["status"] == "WARN"):
        raise SystemExit(1)


def _capture(args: argparse.Namespace, on_sample) -> None:
    timestamp_mode = args.timestamp_mode or (
        "reconstructed-rate" if args.transport == "serial-adapter" else "host-receive"
    )
    if args.transport == "serial-adapter":
        if not args.serial_port:
            raise RuntimeError("Provide --serial-port for --transport serial-adapter.")
        if not args.serial_passive and not args.address and args.adapter_device_index is None:
            devices = scan_serial_adapter_devices(
                args.serial_port,
                timeout_s=args.scan_timeout_s,
                baudrate=args.serial_baudrate,
            )
            if len(devices) == 1:
                args.adapter_device_index = devices[0].index
            else:
                formatted = ", ".join(
                    f"#{device.index} {device.name} {device.address} rssi={device.rssi}" for device in devices
                )
                raise RuntimeError(
                    "Provide --address or --adapter-device-index. "
                    f"Adapter scan saw: {formatted or '<none>'}"
                )
        client = WT901SerialAdapterClient(
            args.serial_port,
            args.sensor_id,
            on_sample,
            on_connected=lambda: print("connected", file=sys.stderr),
            on_status=lambda message: print(message, file=sys.stderr),
            address=args.address,
            device_index=args.adapter_device_index,
            passive=args.serial_passive,
            baudrate=args.serial_baudrate,
            scan_timeout_s=args.scan_timeout_s,
            aux_poll=args.aux_poll,
            timestamp_mode=timestamp_mode,
            sample_rate_hz=args.sample_rate_hz,
        )
        client.run(duration_s=args.duration_s)
        return

    if not args.address:
        devices = asyncio.run(scan_wt_devices(args.scan_timeout_s))
        if len(devices) == 1:
            args.address = devices[0][1]
        else:
            formatted = ", ".join(f"{name} {address}" for name, address in devices)
            raise RuntimeError(f"Provide --address. BLE scan saw: {formatted or '<none>'}")
    client = WT901BleClient(
        args.address,
        args.sensor_id,
        on_sample,
        on_connected=lambda: print("connected", file=sys.stderr),
        on_disconnected=lambda: print("disconnected", file=sys.stderr),
        connect_timeout_s=args.connect_timeout_s,
        timestamp_mode=timestamp_mode,
        sample_rate_hz=args.sample_rate_hz,
    )
    asyncio.run(client.run(duration_s=args.duration_s))


def analyze_imu_samples(
    samples: list[dict],
    *,
    output: Path,
    duration_s: float,
    expected_rate_hz: float,
    min_rate_ratio: float,
    warn_gap_ms: float,
    fail_gap_ms: float,
    warn_p99_ms: float,
    fail_p99_ms: float,
    gravity_min_mps2: float,
    gravity_max_mps2: float,
) -> dict:
    issues: list[dict] = []
    if not samples:
        return {
            "status": "FAIL",
            "output": str(output),
            "sample_count": 0,
            "issues": [{"severity": "FAIL", "message": "No IMU samples received."}],
        }

    timestamps = [int(record["timestamp_monotonic_ns"]) for record in samples]
    sorted_timestamps = sorted(timestamps)
    duplicate_count = len(sorted_timestamps) - len(set(sorted_timestamps))
    nonmonotonic_count = sum(curr <= prev for prev, curr in zip(timestamps, timestamps[1:]))
    dt_ms = [
        (curr - prev) / 1e6
        for prev, curr in zip(sorted_timestamps, sorted_timestamps[1:])
        if curr > prev
    ]
    time_range_s = (sorted_timestamps[-1] - sorted_timestamps[0]) / 1e9 if len(sorted_timestamps) > 1 else 0.0
    effective_rate_hz = (len(sorted_timestamps) - 1) / time_range_s if time_range_s > 0 else 0.0
    accel_norms = [
        _norm3(record.get("accel_mps2", []))
        for record in samples
        if len(record.get("accel_mps2", [])) == 3
    ]
    gyro_norms = [
        _norm3(record.get("gyro_radps", []))
        for record in samples
        if len(record.get("gyro_radps", [])) == 3
    ]
    source_counts = Counter(str(record.get("timestamp_source")) for record in samples)
    reconstruction_reasons = Counter(
        str((record.get("timestamp_reconstruction") or {}).get("reason"))
        for record in samples
        if record.get("timestamp_reconstruction")
    )
    reconstruction_reason_events = _timestamp_reconstruction_reason_events(samples)
    batch_sizes = Counter(
        int((record.get("timestamp_reconstruction") or {}).get("batch_size", 0))
        for record in samples
        if record.get("timestamp_reconstruction")
    )

    if len(samples) < max(5, int(duration_s * expected_rate_hz * min_rate_ratio * 0.5)):
        issues.append({"severity": "FAIL", "message": f"Too few samples: {len(samples)}."})
    if effective_rate_hz < expected_rate_hz * min_rate_ratio:
        issues.append(
            {
                "severity": "FAIL",
                "message": f"Effective rate {effective_rate_hz:.1f}Hz < {expected_rate_hz * min_rate_ratio:.1f}Hz.",
            }
        )
    if duplicate_count:
        issues.append({"severity": "FAIL", "message": f"Duplicate timestamps: {duplicate_count}."})
    if nonmonotonic_count:
        issues.append({"severity": "FAIL", "message": f"Non-monotonic sample order: {nonmonotonic_count}."})

    max_gap = max(dt_ms) if dt_ms else None
    p99_gap = _percentile(dt_ms, 99)
    gaps_warn = sum(value > warn_gap_ms for value in dt_ms)
    gaps_fail = sum(value > fail_gap_ms for value in dt_ms)
    if max_gap is not None and max_gap > fail_gap_ms:
        issues.append({"severity": "FAIL", "message": f"Max timestamp gap {max_gap:.1f}ms > {fail_gap_ms:.1f}ms."})
    elif gaps_warn:
        issues.append({"severity": "WARN", "message": f"{gaps_warn} timestamp gaps > {warn_gap_ms:.1f}ms."})
    if p99_gap is not None and p99_gap > fail_p99_ms:
        issues.append({"severity": "FAIL", "message": f"p99 timestamp dt {p99_gap:.1f}ms > {fail_p99_ms:.1f}ms."})
    elif p99_gap is not None and p99_gap > warn_p99_ms:
        issues.append({"severity": "WARN", "message": f"p99 timestamp dt {p99_gap:.1f}ms > {warn_p99_ms:.1f}ms."})

    accel_mean = statistics.fmean(accel_norms) if accel_norms else None
    if accel_mean is None:
        issues.append({"severity": "FAIL", "message": "No accel_mps2 vectors parsed."})
    elif not gravity_min_mps2 <= accel_mean <= gravity_max_mps2:
        issues.append(
            {
                "severity": "FAIL",
                "message": f"Mean accel norm {accel_mean:.3f} outside [{gravity_min_mps2}, {gravity_max_mps2}] m/s^2.",
            }
        )

    if not gyro_norms:
        issues.append({"severity": "FAIL", "message": "No gyro_radps vectors parsed."})
    if reconstruction_reason_events.get("host_gap_resync", 0):
        issues.append(
            {
                "severity": "WARN",
                "message": (
                    "Timestamp reconstruction resynced after host gaps "
                    f"{reconstruction_reason_events['host_gap_resync']} events "
                    f"({reconstruction_reasons['host_gap_resync']} samples)."
                ),
            }
        )

    status = "PASS"
    if any(issue["severity"] == "FAIL" for issue in issues):
        status = "FAIL"
    elif issues:
        status = "WARN"

    return {
        "status": status,
        "output": str(output),
        "sample_count": len(samples),
        "duration_s": time_range_s,
        "expected_rate_hz": expected_rate_hz,
        "effective_rate_hz": effective_rate_hz,
        "timestamp_sources": dict(source_counts),
        "timestamp_reconstruction_reasons": dict(reconstruction_reasons),
        "timestamp_reconstruction_reason_events": dict(reconstruction_reason_events),
        "timestamp_reconstruction_batch_sizes": dict(batch_sizes),
        "timestamp": {
            "duplicates": duplicate_count,
            "nonmonotonic_order": nonmonotonic_count,
            "dt_ms_min": min(dt_ms) if dt_ms else None,
            "dt_ms_median": statistics.median(dt_ms) if dt_ms else None,
            "dt_ms_p95": _percentile(dt_ms, 95),
            "dt_ms_p99": p99_gap,
            "dt_ms_max": max_gap,
            "gaps_over_warn_ms": gaps_warn,
            "gaps_over_fail_ms": gaps_fail,
        },
        "accel_norm_mps2": {
            "mean": accel_mean,
            "p05": _percentile(accel_norms, 5),
            "p95": _percentile(accel_norms, 95),
            "min": min(accel_norms) if accel_norms else None,
            "max": max(accel_norms) if accel_norms else None,
        },
        "gyro_norm_radps": {
            "mean": statistics.fmean(gyro_norms) if gyro_norms else None,
            "p95": _percentile(gyro_norms, 95),
            "max": max(gyro_norms) if gyro_norms else None,
        },
        "issues": issues,
    }


def _print_summary(summary: dict, *, json_only: bool) -> None:
    if json_only:
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return
    print(f"IMU live check: {summary['status']}")
    print(f"output: {summary.get('output')}")
    if summary.get("error"):
        print(f"error: {summary['error']}")
    if summary.get("hint"):
        print(f"hint: {summary['hint']}")
    if summary.get("sample_count") is not None:
        print(
            "samples={sample_count} duration_s={duration_s:.3f} rate_hz={effective_rate_hz:.1f}".format(
                sample_count=summary.get("sample_count", 0),
                duration_s=float(summary.get("duration_s") or 0.0),
                effective_rate_hz=float(summary.get("effective_rate_hz") or 0.0),
            )
        )
    timestamp = summary.get("timestamp") or {}
    if timestamp:
        print(
            "dt_ms median={:.3f} p95={} p99={} max={} gaps_warn={} gaps_fail={}".format(
                float(timestamp.get("dt_ms_median") or 0.0),
                _fmt(timestamp.get("dt_ms_p95")),
                _fmt(timestamp.get("dt_ms_p99")),
                _fmt(timestamp.get("dt_ms_max")),
                timestamp.get("gaps_over_warn_ms"),
                timestamp.get("gaps_over_fail_ms"),
            )
        )
    accel = summary.get("accel_norm_mps2") or {}
    if accel:
        print(
            "accel_norm mean={} p05={} p95={}".format(
                _fmt(accel.get("mean")),
                _fmt(accel.get("p05")),
                _fmt(accel.get("p95")),
            )
        )
    print(f"timestamp_sources={summary.get('timestamp_sources', {})}")
    reasons = summary.get("timestamp_reconstruction_reasons") or {}
    if reasons:
        print(f"timestamp_reconstruction_reasons={reasons}")
    reason_events = summary.get("timestamp_reconstruction_reason_events") or {}
    if reason_events:
        print(f"timestamp_reconstruction_reason_events={reason_events}")
    for issue in summary.get("issues", []):
        print(f"{issue['severity']}: {issue['message']}")
    if not summary.get("issues"):
        print("No issues found.")


def _default_output(sensor_id: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("/tmp") / f"3dmotion_imu_check_{sensor_id}_{stamp}.jsonl"


def _connection_hint(args: argparse.Namespace) -> str:
    if args.transport == "serial-adapter":
        return (
            "Check the adapter port and ownership, e.g. `ls -l /dev/ttyACM*`; make sure dashboard is not "
            "already using the same port; then scan with "
            f"`python scripts/capture_imu_jsonl.py --adapter-scan --serial-port {args.serial_port or '<port>'}`."
        )
    return "Check BLE advertising with `python scripts/capture_imu_jsonl.py --scan-all --scan-timeout-s 20`."


def _timestamp_reconstruction_reason_events(samples: list[dict]) -> Counter:
    events = Counter()
    previous_key = None
    for record in samples:
        reconstruction = record.get("timestamp_reconstruction") or {}
        reason = reconstruction.get("reason")
        if not reason:
            continue
        key = (
            str(reason),
            reconstruction.get("host_receive_monotonic_ns"),
            reconstruction.get("batch_size"),
        )
        if key != previous_key:
            events[str(reason)] += 1
        previous_key = key
    return events


def _norm3(values: list[float]) -> float:
    if len(values) != 3:
        return float("nan")
    return math.sqrt(sum(float(value) ** 2 for value in values))


def _percentile(values: list[float], percentile: float) -> float | None:
    clean = [float(value) for value in values if math.isfinite(float(value))]
    if not clean:
        return None
    ordered = sorted(clean)
    index = round((len(ordered) - 1) * percentile / 100.0)
    return ordered[index]


def _fmt(value: object) -> str:
    if value is None:
        return "n/a"
    try:
        return f"{float(value):.3f}"
    except (TypeError, ValueError):
        return str(value)


if __name__ == "__main__":
    main()
