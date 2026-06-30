from __future__ import annotations

import argparse
import asyncio
import errno
import json
import contextlib
import math
import re
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable, Iterable


ACC_G_TO_MPS2 = 9.80665
DEG_TO_RAD = 0.017453292519943295
DEFAULT_SAMPLE_RATE_HZ = 200.0


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
    host_receive_unix_ns: int | None = None
    host_receive_monotonic_ns: int | None = None
    timestamp_reconstruction: dict[str, object] | None = None


@dataclass
class SerialAdapterDevice:
    index: int
    name: str
    address: str
    rssi: int


class WT901PacketParser:
    """Parser for WT-series BLE packets used by the existing Dual_IMU project."""

    def __init__(
        self,
        sensor_id: str,
        on_sample: Callable[[ImuSample], None],
        *,
        timestamp_mode: str = "host_receive",
        sample_rate_hz: float = DEFAULT_SAMPLE_RATE_HZ,
    ) -> None:
        self.sensor_id = sensor_id
        self.on_sample = on_sample
        self.timestamp_mode = _normalize_timestamp_mode(timestamp_mode)
        if sample_rate_hz <= 0:
            raise ValueError("sample_rate_hz must be positive.")
        self.sample_rate_hz = float(sample_rate_hz)
        self._sample_period_ns = int(round(1_000_000_000 / self.sample_rate_hz))
        self._last_reconstructed_monotonic_ns: int | None = None
        self._buffer: list[int] = []
        self._quat_wxyz: list[float] | None = None
        self._mag: list[float] | None = None

    def feed(
        self,
        data: bytes,
        *,
        receive_unix_ns: int | None = None,
        receive_monotonic_ns: int | None = None,
    ) -> None:
        receive_unix_ns = int(receive_unix_ns if receive_unix_ns is not None else time.time_ns())
        receive_monotonic_ns = int(
            receive_monotonic_ns if receive_monotonic_ns is not None else time.monotonic_ns()
        )
        pending_samples = []
        for value in data:
            self._buffer.append(value)
            if len(self._buffer) == 1 and self._buffer[0] != 0x55:
                del self._buffer[0]
                continue
            if len(self._buffer) == 2 and self._buffer[1] not in (0x61, 0x71):
                del self._buffer[0]
                continue
            if len(self._buffer) == 20:
                sample_payload = self._process_packet(self._buffer)
                if sample_payload is not None:
                    pending_samples.append(sample_payload)
                self._buffer.clear()
        if not pending_samples:
            return

        timestamps = self._sample_timestamps(
            count=len(pending_samples),
            receive_unix_ns=receive_unix_ns,
            receive_monotonic_ns=receive_monotonic_ns,
        )
        for payload, timestamp in zip(pending_samples, timestamps):
            self.on_sample(
                ImuSample(
                    sensor_id=self.sensor_id,
                    timestamp_unix_ns=timestamp["timestamp_unix_ns"],
                    timestamp_monotonic_ns=timestamp["timestamp_monotonic_ns"],
                    timestamp_source=timestamp["timestamp_source"],
                    host_receive_unix_ns=receive_unix_ns,
                    host_receive_monotonic_ns=receive_monotonic_ns,
                    timestamp_reconstruction=timestamp.get("timestamp_reconstruction"),
                    **payload,
                )
            )

    def _process_packet(self, packet: list[int]) -> dict[str, object] | None:
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
            quat = self._quat_wxyz or _euler_deg_to_quat_wxyz(roll, pitch, yaw)
            return {
                "accel_mps2": [ax_g * ACC_G_TO_MPS2, ay_g * ACC_G_TO_MPS2, az_g * ACC_G_TO_MPS2],
                "gyro_radps": [gx_dps * DEG_TO_RAD, gy_dps * DEG_TO_RAD, gz_dps * DEG_TO_RAD],
                "euler_deg": [roll, pitch, yaw],
                "quat_wxyz": quat,
                "mag": self._mag,
            }

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
        return None

    def _sample_timestamps(
        self,
        *,
        count: int,
        receive_unix_ns: int,
        receive_monotonic_ns: int,
    ) -> list[dict[str, object]]:
        if self.timestamp_mode == "host_receive":
            return [
                {
                    "timestamp_unix_ns": receive_unix_ns,
                    "timestamp_monotonic_ns": receive_monotonic_ns,
                    "timestamp_source": "host_receive",
                    "timestamp_reconstruction": None,
                }
                for _ in range(count)
            ]

        period_ns = self._sample_period_ns
        host_first_monotonic_ns = receive_monotonic_ns - (count - 1) * period_ns
        reason = "initial"
        if self._last_reconstructed_monotonic_ns is None:
            first_monotonic_ns = host_first_monotonic_ns
        else:
            expected_first_monotonic_ns = self._last_reconstructed_monotonic_ns + period_ns
            resync_gap_ns = max(5 * period_ns, 20_000_000)
            if host_first_monotonic_ns > expected_first_monotonic_ns + resync_gap_ns:
                first_monotonic_ns = host_first_monotonic_ns
                reason = "host_gap_resync"
            else:
                first_monotonic_ns = expected_first_monotonic_ns
                reason = "continuous_rate"

        unix_minus_monotonic_ns = receive_unix_ns - receive_monotonic_ns
        source = f"reconstructed_{self.sample_rate_hz:g}hz_from_host_receive"
        timestamps = []
        for index in range(count):
            monotonic_ns = first_monotonic_ns + index * period_ns
            timestamps.append(
                {
                    "timestamp_unix_ns": monotonic_ns + unix_minus_monotonic_ns,
                    "timestamp_monotonic_ns": monotonic_ns,
                    "timestamp_source": source,
                    "timestamp_reconstruction": {
                        "mode": "reconstructed_rate",
                        "sample_rate_hz": self.sample_rate_hz,
                        "sample_period_ns": period_ns,
                        "batch_size": count,
                        "batch_index": index,
                        "host_receive_monotonic_ns": receive_monotonic_ns,
                        "reason": reason,
                    },
                }
            )
        self._last_reconstructed_monotonic_ns = int(timestamps[-1]["timestamp_monotonic_ns"])
        return timestamps


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
        on_disconnected: Callable[[], None] | None = None,
        on_status: Callable[[str], None] | None = None,
        *,
        notify_uuid: str = READ_UUID,
        write_uuid: str | None = WRITE_UUID,
        connect_timeout_s: float = 20.0,
        aux_poll: bool = False,
        aux_poll_start_delay_s: float = 3.0,
        timestamp_mode: str = "host_receive",
        sample_rate_hz: float = DEFAULT_SAMPLE_RATE_HZ,
    ) -> None:
        self.address = address
        self.sensor_id = sensor_id
        self.parser = WT901PacketParser(
            sensor_id,
            on_sample,
            timestamp_mode=timestamp_mode,
            sample_rate_hz=sample_rate_hz,
        )
        self.notify_uuid = notify_uuid.lower()
        self.write_uuid = write_uuid.lower() if write_uuid else None
        self.connect_timeout_s = connect_timeout_s
        self.aux_poll = aux_poll
        self.aux_poll_start_delay_s = aux_poll_start_delay_s
        self._write_uuid: str | None = None
        self.on_connected = on_connected
        self.on_disconnected = on_disconnected

    async def run(self, duration_s: float | None = None) -> None:
        try:
            from bleak import BleakClient, BleakScanner
        except ImportError as exc:
            raise RuntimeError("Install bleak to capture BLE IMU data: pip install bleak") from exc

        ble_device = await self._find_device(BleakScanner)
        start = time.monotonic()
        loop = asyncio.get_running_loop()
        disconnected = asyncio.Event()

        def handle_disconnected(_client) -> None:
            loop.call_soon_threadsafe(disconnected.set)
            if self.on_disconnected is not None:
                self.on_disconnected()

        async with BleakClient(ble_device, timeout=15, disconnected_callback=handle_disconnected) as client:
            services = list(_iter_services(client))
            notify_characteristic = _find_characteristic(
                services,
                self.notify_uuid,
                service_uuid=self.SERVICE_UUID,
            )
            write_characteristic = (
                _find_characteristic(services, self.write_uuid, service_uuid=self.SERVICE_UUID)
                if self.write_uuid is not None
                else None
            )
            if notify_characteristic is None:
                raise RuntimeError(
                    "BLE notify characteristic not found: "
                    f"{self.notify_uuid}. Available characteristics: {_format_characteristics(services)}"
                )
            if self.aux_poll and write_characteristic is None:
                raise RuntimeError(
                    "BLE aux polling requested but write characteristic not found: "
                    f"{self.write_uuid}. Available characteristics: {_format_characteristics(services)}"
                )

            notify_uuid = str(notify_characteristic.uuid)
            self._write_uuid = str(write_characteristic.uuid) if write_characteristic is not None else None
            await client.start_notify(notify_uuid, lambda _sender, data: self.parser.feed(bytes(data)))
            if self.on_connected is not None:
                self.on_connected()
            poll_task = asyncio.create_task(self._poll_aux_registers(client)) if self.aux_poll else None
            try:
                while (duration_s is None or time.monotonic() - start < duration_s) and not disconnected.is_set():
                    await asyncio.sleep(0.1)
            finally:
                if poll_task is not None:
                    poll_task.cancel()
                    with contextlib.suppress(asyncio.CancelledError):
                        await poll_task
                with contextlib.suppress(Exception):
                    await client.stop_notify(notify_uuid)

    async def _find_device(self, scanner_cls):
        ble_device = None
        finder = getattr(scanner_cls, "find_device_by_address", None)
        if finder is not None:
            ble_device = await finder(self.address, timeout=self.connect_timeout_s)
        if ble_device is None:
            for device in await scanner_cls.discover(timeout=self.connect_timeout_s):
                if str(getattr(device, "address", "")).lower() == self.address.lower():
                    ble_device = device
                    break
        if ble_device is None:
            raise RuntimeError(
                f"BLE device not found by address: {self.address}. "
                "Run with --scan-all to confirm the device is advertising."
            )
        return ble_device

    async def service_summary(self) -> list[dict[str, object]]:
        try:
            from bleak import BleakClient, BleakScanner
        except ImportError as exc:
            raise RuntimeError("Install bleak to capture BLE IMU data: pip install bleak") from exc

        ble_device = await self._find_device(BleakScanner)
        async with BleakClient(ble_device, timeout=15) as client:
            return _service_summary(_iter_services(client))

    async def _poll_aux_registers(self, client) -> None:
        if self._write_uuid is None:
            raise RuntimeError("BLE write characteristic is unavailable.")
        await asyncio.sleep(self.aux_poll_start_delay_s)
        while True:
            await client.write_gatt_char(self._write_uuid, bytes(_read_register_command(0x3A)))
            await asyncio.sleep(0.1)
            await client.write_gatt_char(self._write_uuid, bytes(_read_register_command(0x51)))
            await asyncio.sleep(0.1)


class WT901SerialAdapterClient:
    """WIT serial BLE adapter transport.

    The official BWT901 adapter example exposes the adapter as a serial port,
    scans with AT+SCAN, connects by scan index, then streams the same 20-byte
    WT901 packets that the BLE notification path already parses.
    """

    def __init__(
        self,
        port: str,
        sensor_id: str,
        on_sample: Callable[[ImuSample], None],
        on_connected: Callable[[], None] | None = None,
        on_status: Callable[[str], None] | None = None,
        *,
        address: str | None = None,
        device_index: int | None = None,
        passive: bool = False,
        baudrate: int = 115200,
        scan_timeout_s: float = 8.0,
        read_timeout_s: float = 0.2,
        aux_poll: bool = False,
        aux_poll_interval_s: float = 0.1,
        aux_poll_start_delay_s: float = 3.0,
        timestamp_mode: str = "reconstructed_rate",
        sample_rate_hz: float = DEFAULT_SAMPLE_RATE_HZ,
    ) -> None:
        if not passive and not address and device_index is None:
            raise ValueError("Provide address or device_index for the serial adapter connection.")
        self.port = port
        self.sensor_id = sensor_id
        self.parser = WT901PacketParser(
            sensor_id,
            on_sample,
            timestamp_mode=timestamp_mode,
            sample_rate_hz=sample_rate_hz,
        )
        self.on_connected = on_connected
        self.on_status = on_status
        self.address = address
        self.device_index = device_index
        self.passive = passive
        self.baudrate = baudrate
        self.scan_timeout_s = scan_timeout_s
        self.read_timeout_s = read_timeout_s
        self.aux_poll = aux_poll
        self.aux_poll_interval_s = aux_poll_interval_s
        self.aux_poll_start_delay_s = aux_poll_start_delay_s

    def run(self, duration_s: float | None = None, should_stop: Callable[[], bool] | None = None) -> None:
        serial_port = _open_serial_port(self.port, self.baudrate, self.read_timeout_s)
        try:
            if self.passive:
                if self.on_status is not None:
                    self.on_status(f"{self.port}: passive serial read; not sending AT+SCAN/AT+CONNECT")
            else:
                index = self.device_index
                if index is None:
                    devices = scan_serial_adapter_devices(
                        self.port,
                        timeout_s=self.scan_timeout_s,
                        baudrate=self.baudrate,
                        read_timeout_s=self.read_timeout_s,
                        serial_port=serial_port,
                        stop_scan=False,
                        target_address=self.address,
                        should_stop=should_stop,
                    )
                    index = _find_serial_adapter_index(devices, self.address or "")
                if self.on_status is not None:
                    target = self.address or f"scan-index-{index}"
                    self.on_status(f"{self.port}: AT+CONNECT={index} target={target}")
                _serial_write_text(serial_port, f"AT+CONNECT={index}\r\n")
                _serial_write_text(serial_port, "AT+SCAN=0\r\n")
            if self.on_status is not None:
                self.on_status(f"{self.port}: reading WT901 stream")
            if self.on_connected is not None:
                self.on_connected()

            start = time.monotonic()
            next_aux_poll = start + self.aux_poll_start_delay_s
            aux_registers = (0x3A, 0x51)
            aux_register_index = 0
            while (duration_s is None or time.monotonic() - start < duration_s) and not (
                should_stop is not None and should_stop()
            ):
                now = time.monotonic()
                if self.aux_poll and now >= next_aux_poll:
                    register = aux_registers[aux_register_index]
                    _serial_write_bytes(serial_port, bytes(_read_register_command(register)))
                    aux_register_index = (aux_register_index + 1) % len(aux_registers)
                    next_aux_poll = now + self.aux_poll_interval_s
                data = serial_port.read(256)
                if data:
                    self.parser.feed(bytes(data))
        finally:
            if not self.passive:
                with contextlib.suppress(Exception):
                    _serial_write_text(serial_port, "AT+SCAN=0\r\n")
            serial_port.close()


def scan_serial_adapter_devices(
    port: str,
    *,
    timeout_s: float = 8.0,
    baudrate: int = 115200,
    read_timeout_s: float = 0.2,
    serial_port=None,
    stop_scan: bool = True,
    target_address: str | None = None,
    should_stop: Callable[[], bool] | None = None,
) -> list[SerialAdapterDevice]:
    close_when_done = serial_port is None
    serial_port = serial_port or _open_serial_port(port, baudrate, read_timeout_s)
    devices: dict[str, SerialAdapterDevice] = {}
    try:
        _serial_write_text(serial_port, "AT+SCAN=1\r\n")
        deadline = time.monotonic() + timeout_s
        text_buffer = ""
        while time.monotonic() < deadline and not (should_stop is not None and should_stop()):
            chunk = serial_port.read(512)
            if not chunk:
                continue
            text_buffer += bytes(chunk).decode("ascii", errors="ignore")
            for device in _parse_serial_adapter_scan(text_buffer):
                devices[device.address.lower()] = device
            if target_address and target_address.lower() in devices:
                break
            if len(text_buffer) > 8192:
                text_buffer = text_buffer[-4096:]
        if stop_scan:
            _serial_write_text(serial_port, "AT+SCAN=0\r\n")
        return sorted(devices.values(), key=lambda device: (device.index, device.address))
    finally:
        if close_when_done:
            serial_port.close()


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


def _normalize_timestamp_mode(value: str) -> str:
    normalized = str(value).strip().lower().replace("-", "_")
    aliases = {
        "host": "host_receive",
        "host_receive": "host_receive",
        "reconstructed": "reconstructed_rate",
        "reconstructed_rate": "reconstructed_rate",
        "rate": "reconstructed_rate",
    }
    if normalized not in aliases:
        raise ValueError("timestamp_mode must be one of: host_receive, reconstructed_rate")
    return aliases[normalized]


def _int16(value: int) -> int:
    return value - 65536 if value >= 32768 else value


def _read_register_command(register: int) -> list[int]:
    return [0xFF, 0xAA, 0x27, register, 0x00]


def _euler_deg_to_quat_wxyz(roll_deg: float, pitch_deg: float, yaw_deg: float) -> list[float]:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    quat = [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]
    norm = math.sqrt(sum(value * value for value in quat))
    if norm <= 0.0:
        return [1.0, 0.0, 0.0, 0.0]
    return [value / norm for value in quat]


def _open_serial_port(port: str, baudrate: int, timeout_s: float):
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("Install pyserial to use the WT901 serial adapter: pip install pyserial") from exc

    serial_port = serial.Serial()
    serial_port.port = port
    serial_port.baudrate = baudrate
    serial_port.timeout = timeout_s
    serial_port.dtr = True
    try:
        serial_port.open()
    except Exception as exc:
        error_number = getattr(exc, "errno", None)
        if error_number is None and getattr(exc, "args", None):
            error_number = exc.args[0]
        if error_number == errno.EACCES:
            raise RuntimeError(
                f"Permission denied opening {port}. Add your user to the dialout group and log out/in, "
                f"or temporarily run: sudo chmod a+rw {port}"
            ) from exc
        raise
    return serial_port


def _serial_write_text(serial_port, value: str) -> None:
    _serial_write_bytes(serial_port, value.encode("ascii"))


def _serial_write_bytes(serial_port, value: bytes) -> None:
    serial_port.write(value)
    flush = getattr(serial_port, "flush", None)
    if flush is not None:
        flush()


def _parse_serial_adapter_scan(text: str) -> list[SerialAdapterDevice]:
    pattern = re.compile(r'WIT-LIST-#\s*(\d+)\s*:"([^"]+)"\s+(0x[\dA-Fa-f]{12})\s+(-?\d+)')
    devices = []
    for match in pattern.finditer(text):
        devices.append(
            SerialAdapterDevice(
                index=int(match.group(1)),
                name=match.group(2).strip(),
                address=_standard_mac_from_wit_hex(match.group(3)),
                rssi=int(match.group(4)),
            )
        )
    return devices


def _standard_mac_from_wit_hex(value: str) -> str:
    clean = value[2:] if value.lower().startswith("0x") else value
    if len(clean) != 12:
        return value
    return ":".join(clean[index : index + 2] for index in range(0, 12, 2)).upper()


def _find_serial_adapter_index(devices: list[SerialAdapterDevice], address: str) -> int:
    wanted = address.lower()
    for device in devices:
        if device.address.lower() == wanted:
            return device.index
    formatted = ", ".join(f"#{device.index} {device.name} {device.address} rssi={device.rssi}" for device in devices)
    raise RuntimeError(f"Serial adapter did not find BLE address {address}. Seen devices: {formatted or '<none>'}")


def _iter_services(client) -> Iterable:
    services = getattr(client, "services", None)
    if services is None:
        return []
    return list(services)


def _find_characteristic(services: Iterable, uuid: str | None, *, service_uuid: str | None = None):
    if uuid is None:
        return None
    uuid = uuid.lower()
    service_uuid = service_uuid.lower() if service_uuid else None
    ordered_services = list(services)
    if service_uuid is not None:
        ordered_services = [
            service for service in ordered_services if str(getattr(service, "uuid", "")).lower() == service_uuid
        ] + [
            service for service in ordered_services if str(getattr(service, "uuid", "")).lower() != service_uuid
        ]
    for service in ordered_services:
        for characteristic in getattr(service, "characteristics", []) or []:
            if str(getattr(characteristic, "uuid", "")).lower() == uuid:
                return characteristic
    return None


def _service_summary(services: Iterable) -> list[dict[str, object]]:
    result = []
    for service in services:
        result.append(
            {
                "uuid": str(getattr(service, "uuid", "")),
                "characteristics": [
                    {
                        "uuid": str(getattr(characteristic, "uuid", "")),
                        "properties": list(getattr(characteristic, "properties", []) or []),
                    }
                    for characteristic in (getattr(service, "characteristics", []) or [])
                ],
            }
        )
    return result


def _format_characteristics(services: Iterable) -> str:
    values = []
    for service in _service_summary(services):
        for characteristic in service["characteristics"]:
            properties = ",".join(characteristic["properties"])
            values.append(f"{service['uuid']}::{characteristic['uuid']}[{properties}]")
    return "; ".join(values) if values else "<none>"


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture WT-series BLE IMU data to JSONL without GUI.")
    parser.add_argument("--transport", choices=["ble", "serial-adapter"], default="ble", help="Capture transport. Use serial-adapter for the WIT BWT901 USB adapter.")
    parser.add_argument("--scan", action="store_true", help="Scan WT BLE devices and exit.")
    parser.add_argument("--scan-all", action="store_true", help="Scan all BLE devices for debugging and exit.")
    parser.add_argument("--adapter-scan", action="store_true", help="Scan devices through a WIT serial BLE adapter and exit.")
    parser.add_argument("--scan-timeout-s", type=float, default=20.0, help="BLE scan timeout for --scan or --scan-all.")
    parser.add_argument("--address", help="BLE address of the IMU device.")
    parser.add_argument("--serial-port", help="Serial port of the WIT BLE adapter, for example /dev/ttyACM0.")
    parser.add_argument("--serial-baudrate", type=int, default=115200, help="Serial baudrate for the WIT BLE adapter.")
    parser.add_argument("--serial-passive", action="store_true", help="Only read an already-connected serial adapter stream; do not send AT+SCAN/AT+CONNECT.")
    parser.add_argument("--adapter-device-index", type=int, default=None, help="Connect to this adapter scan index instead of matching --address.")
    parser.add_argument("--sensor-id", default="wrist_imu", help="Sensor ID written into samples.")
    parser.add_argument("--output", default="data/raw/imu_wrist.jsonl", help="Output JSONL path.")
    parser.add_argument("--duration-s", type=float, default=None, help="Optional capture duration.")
    parser.add_argument("--notify-uuid", default=WT901BleClient.READ_UUID, help="Notify characteristic UUID.")
    parser.add_argument("--write-uuid", default=WT901BleClient.WRITE_UUID, help="Optional write characteristic UUID.")
    parser.add_argument("--connect-timeout-s", type=float, default=20.0, help="BLE address lookup timeout.")
    parser.add_argument("--aux-poll", action="store_true", help="Poll optional magnetometer/quaternion registers over the write characteristic.")
    parser.add_argument(
        "--timestamp-mode",
        choices=["host-receive", "reconstructed-rate"],
        default=None,
        help=(
            "IMU timestamping mode. Default: reconstructed-rate for serial-adapter, "
            "host-receive for BLE."
        ),
    )
    parser.add_argument(
        "--sample-rate-hz",
        type=float,
        default=DEFAULT_SAMPLE_RATE_HZ,
        help=f"Nominal IMU sample rate used by --timestamp-mode reconstructed-rate. Default: {DEFAULT_SAMPLE_RATE_HZ:g}.",
    )
    parser.add_argument("--print-services", action="store_true", help="Connect to the BLE device and print discovered services/characteristics.")
    args = parser.parse_args()

    if args.adapter_scan:
        if not args.serial_port:
            raise SystemExit("Provide --serial-port for --adapter-scan.")
        for device in scan_serial_adapter_devices(args.serial_port, timeout_s=args.scan_timeout_s, baudrate=args.serial_baudrate):
            print(f"#{device.index}\t{device.name}\t{device.address}\trssi={device.rssi}")
        return

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
        if args.transport == "serial-adapter" and args.adapter_device_index is not None:
            pass
        elif args.transport == "serial-adapter" and args.serial_passive:
            pass
        else:
            raise SystemExit("Provide --address, or run with --scan first.")

    if args.transport == "serial-adapter":
        if not args.serial_port:
            raise SystemExit("Provide --serial-port when --transport serial-adapter is used.")
        out = Path(args.output)
        out.parent.mkdir(parents=True, exist_ok=True)
        sample_count = 0

        def handle_sample(sample: ImuSample) -> None:
            nonlocal sample_count
            sample_count += 1
            write_jsonl_sample(out, sample)

        client = WT901SerialAdapterClient(
            args.serial_port,
            args.sensor_id,
            handle_sample,
            on_status=lambda message: print(message, file=sys.stderr),
            address=args.address,
            device_index=args.adapter_device_index,
            passive=args.serial_passive,
            baudrate=args.serial_baudrate,
            scan_timeout_s=args.scan_timeout_s,
            aux_poll=args.aux_poll,
            timestamp_mode=args.timestamp_mode or "reconstructed-rate",
            sample_rate_hz=args.sample_rate_hz,
        )
        client.run(duration_s=args.duration_s)
        if sample_count == 0:
            raise SystemExit(
                "No IMU samples received. Confirm the module is advertising/connectable, "
                "the address or adapter scan index is correct, and no other process owns the adapter."
            )
        return

    client = WT901BleClient(
        args.address,
        args.sensor_id,
        lambda sample: None,
        notify_uuid=args.notify_uuid,
        write_uuid=args.write_uuid,
        connect_timeout_s=args.connect_timeout_s,
        aux_poll=args.aux_poll,
        timestamp_mode=args.timestamp_mode or "host-receive",
        sample_rate_hz=args.sample_rate_hz,
    )
    if args.print_services:
        print(json.dumps(asyncio.run(client.service_summary()), ensure_ascii=False, indent=2))
        return

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    client = WT901BleClient(
        args.address,
        args.sensor_id,
        lambda sample: write_jsonl_sample(out, sample),
        notify_uuid=args.notify_uuid,
        write_uuid=args.write_uuid,
        connect_timeout_s=args.connect_timeout_s,
        aux_poll=args.aux_poll,
        timestamp_mode=args.timestamp_mode or "host-receive",
        sample_rate_hz=args.sample_rate_hz,
    )
    asyncio.run(client.run(duration_s=args.duration_s))


if __name__ == "__main__":
    main()
