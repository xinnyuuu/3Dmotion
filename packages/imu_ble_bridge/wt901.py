from __future__ import annotations

import argparse
import asyncio
import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable


ACC_G_TO_MPS2 = 9.80665
DEG_TO_RAD = 0.017453292519943295


@dataclass
class ImuSample:
    sensor_id: str
    timestamp_unix_ns: int
    timestamp_monotonic_ns: int
    timestamp_source: str
    accel_mps2: list[float]
    gyro_radps: list[float]
    euler_deg: list[float]
    quat_wxyz: list[float] | None = None
    mag: list[float] | None = None


class WT901PacketParser:
    """Parser for WT-series BLE packets used by the existing Dual_IMU project."""

    def __init__(self, sensor_id: str, on_sample: Callable[[ImuSample], None]) -> None:
        self.sensor_id = sensor_id
        self.on_sample = on_sample
        self._buffer: list[int] = []
        self._quat_wxyz: list[float] | None = None
        self._mag: list[float] | None = None

    def feed(self, data: bytes) -> None:
        for value in data:
            self._buffer.append(value)
            if len(self._buffer) == 1 and self._buffer[0] != 0x55:
                del self._buffer[0]
                continue
            if len(self._buffer) == 2 and self._buffer[1] not in (0x61, 0x71):
                del self._buffer[0]
                continue
            if len(self._buffer) == 20:
                self._process_packet(self._buffer)
                self._buffer.clear()

    def _process_packet(self, packet: list[int]) -> None:
        if packet[1] == 0x61:
            ax_g = _int16(packet[3] << 8 | packet[2]) / 32768.0 * 16.0
            ay_g = _int16(packet[5] << 8 | packet[4]) / 32768.0 * 16.0
            az_g = _int16(packet[7] << 8 | packet[6]) / 32768.0 * 16.0
            gx_dps = _int16(packet[9] << 8 | packet[8]) / 32768.0 * 2000.0
            gy_dps = _int16(packet[11] << 8 | packet[10]) / 32768.0 * 2000.0
            gz_dps = _int16(packet[13] << 8 | packet[12]) / 32768.0 * 2000.0
            roll = _int16(packet[15] << 8 | packet[14]) / 32768.0 * 180.0
            pitch = _int16(packet[17] << 8 | packet[16]) / 32768.0 * 180.0
            yaw = _int16(packet[19] << 8 | packet[18]) / 32768.0 * 180.0
            self.on_sample(
                ImuSample(
                    sensor_id=self.sensor_id,
                    timestamp_unix_ns=time.time_ns(),
                    timestamp_monotonic_ns=time.monotonic_ns(),
                    timestamp_source="host_receive",
                    accel_mps2=[ax_g * ACC_G_TO_MPS2, ay_g * ACC_G_TO_MPS2, az_g * ACC_G_TO_MPS2],
                    gyro_radps=[gx_dps * DEG_TO_RAD, gy_dps * DEG_TO_RAD, gz_dps * DEG_TO_RAD],
                    euler_deg=[roll, pitch, yaw],
                    quat_wxyz=self._quat_wxyz,
                    mag=self._mag,
                )
            )
            return

        if packet[2] == 0x3A:
            self._mag = [
                _int16(packet[5] << 8 | packet[4]) / 120.0,
                _int16(packet[7] << 8 | packet[6]) / 120.0,
                _int16(packet[9] << 8 | packet[8]) / 120.0,
            ]
        elif packet[2] == 0x51:
            self._quat_wxyz = [
                _int16(packet[5] << 8 | packet[4]) / 32768.0,
                _int16(packet[7] << 8 | packet[6]) / 32768.0,
                _int16(packet[9] << 8 | packet[8]) / 32768.0,
                _int16(packet[11] << 8 | packet[10]) / 32768.0,
            ]


class WT901BleClient:
    SERVICE_UUID = "0000ffe5-0000-1000-8000-00805f9a34fb"
    READ_UUID = "0000ffe4-0000-1000-8000-00805f9a34fb"
    WRITE_UUID = "0000ffe9-0000-1000-8000-00805f9a34fb"

    def __init__(
        self,
        address: str,
        sensor_id: str,
        on_sample: Callable[[ImuSample], None],
        on_connected: Callable[[], None] | None = None,
    ) -> None:
        self.address = address
        self.sensor_id = sensor_id
        self.parser = WT901PacketParser(sensor_id, on_sample)
        self._write_uuid: str | None = None
        self.on_connected = on_connected

    async def run(self, duration_s: float | None = None) -> None:
        try:
            from bleak import BleakClient
        except ImportError as exc:
            raise RuntimeError("Install bleak to capture BLE IMU data: pip install bleak") from exc

        start = time.monotonic()
        async with BleakClient(self.address, timeout=15) as client:
            self._write_uuid = self.WRITE_UUID
            await client.start_notify(self.READ_UUID, lambda _sender, data: self.parser.feed(bytes(data)))
            if self.on_connected is not None:
                self.on_connected()
            poll_task = asyncio.create_task(self._poll_aux_registers(client))
            try:
                while duration_s is None or time.monotonic() - start < duration_s:
                    await asyncio.sleep(0.1)
            finally:
                poll_task.cancel()
                await client.stop_notify(self.READ_UUID)

    async def _poll_aux_registers(self, client) -> None:
        while True:
            await client.write_gatt_char(self.WRITE_UUID, bytes(_read_register_command(0x3A)))
            await asyncio.sleep(0.1)
            await client.write_gatt_char(self.WRITE_UUID, bytes(_read_register_command(0x51)))
            await asyncio.sleep(0.1)


async def scan_ble_devices(timeout_s: float = 8.0) -> list[dict[str, object]]:
    try:
        from bleak import BleakScanner
    except ImportError as exc:
        raise RuntimeError("Install bleak to scan BLE IMU devices: pip install bleak") from exc

    found: dict[str, dict[str, object]] = {}

    def remember(device, advertisement_data=None) -> None:
        local_name = getattr(advertisement_data, "local_name", None)
        name = local_name or device.name or "Unknown"
        service_uuids = [str(uuid).lower() for uuid in (getattr(advertisement_data, "service_uuids", None) or [])]
        found[device.address] = {
            "name": name,
            "address": device.address,
            "service_uuids": service_uuids,
            "rssi": getattr(device, "rssi", None),
        }

    try:
        scanner = BleakScanner(detection_callback=remember)
        await scanner.start()
        await asyncio.sleep(timeout_s)
        await scanner.stop()
        for device in getattr(scanner, "discovered_devices", []):
            remember(device)
    except TypeError:
        # Older bleak versions may not accept detection_callback in the
        # constructor. Fall back to discover(), still de-duplicating by address.
        for device in await BleakScanner.discover(timeout=timeout_s):
            remember(device)
    return sorted(found.values(), key=lambda item: (str(item["name"]).upper(), str(item["address"])))


async def scan_wt_devices(timeout_s: float = 8.0) -> list[tuple[str, str]]:
    devices = await scan_ble_devices(timeout_s)
    found: dict[str, tuple[str, str]] = {}
    for device in devices:
        name = str(device["name"])
        address = str(device["address"])
        service_uuids = [str(uuid).lower() for uuid in device.get("service_uuids", [])]
        haystack = " ".join([name, *service_uuids]).upper()
        # WT901 devices usually advertise a WT* name. Some Linux/BlueZ scans
        # report only service UUIDs for one of two identical devices, so also
        # accept the common FFE* service family used by the sensor.
        if "WT" in haystack or any("ffe" in uuid for uuid in service_uuids):
            found[address] = (name, address)
    return sorted(found.values(), key=lambda item: (item[0].upper(), item[1]))


def write_jsonl_sample(path: Path, sample: ImuSample) -> None:
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(asdict(sample), separators=(",", ":")) + "\n")


def _int16(value: int) -> int:
    return value - 65536 if value >= 32768 else value


def _read_register_command(register: int) -> list[int]:
    return [0xFF, 0xAA, 0x27, register, 0x00]


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture WT-series BLE IMU data to JSONL without GUI.")
    parser.add_argument("--scan", action="store_true", help="Scan WT BLE devices and exit.")
    parser.add_argument("--scan-all", action="store_true", help="Scan all BLE devices for debugging and exit.")
    parser.add_argument("--scan-timeout-s", type=float, default=8.0, help="BLE scan timeout for --scan or --scan-all.")
    parser.add_argument("--address", help="BLE address of the IMU device.")
    parser.add_argument("--sensor-id", default="wrist_imu", help="Sensor ID written into samples.")
    parser.add_argument("--output", default="data/raw/imu_wrist.jsonl", help="Output JSONL path.")
    parser.add_argument("--duration-s", type=float, default=None, help="Optional capture duration.")
    args = parser.parse_args()

    if args.scan_all:
        for device in asyncio.run(scan_ble_devices(args.scan_timeout_s)):
            service_uuids = ",".join(device.get("service_uuids", []))
            print(f"{device['name']}\t{device['address']}\trssi={device.get('rssi')}\tservices={service_uuids}")
        return

    if args.scan:
        for name, address in asyncio.run(scan_wt_devices(args.scan_timeout_s)):
            print(f"{name}\t{address}")
        return

    if not args.address:
        raise SystemExit("Provide --address, or run with --scan first.")

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    client = WT901BleClient(args.address, args.sensor_id, lambda sample: write_jsonl_sample(out, sample))
    asyncio.run(client.run(duration_s=args.duration_s))


if __name__ == "__main__":
    main()
