from __future__ import annotations

import errno
import fcntl
import os
import re
import struct
from dataclasses import asdict, dataclass


VIDIOC_QUERYCAP = 0x80685600
VIDIOC_ENUM_FMT = 0xC0405602
VIDIOC_ENUM_FRAMESIZES = 0xC02C564A
VIDIOC_ENUM_FRAMEINTERVALS = 0xC034564B
V4L2_CAP_VIDEO_CAPTURE = 0x00000001
V4L2_CAP_DEVICE_CAPS = 0x80000000
V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_FRMSIZE_TYPE_DISCRETE = 1
V4L2_FRMIVAL_TYPE_DISCRETE = 1


@dataclass(frozen=True)
class CameraDevice:
    path: str
    name: str
    bus: str

    @property
    def label(self) -> str:
        detail = f" - {self.name}" if self.name else ""
        return f"{self.path}{detail}"

    def as_dict(self) -> dict:
        return asdict(self) | {"label": self.label}


def find_capture_devices() -> list[CameraDevice]:
    sysfs_root = "/sys/class/video4linux"
    if os.path.isdir(sysfs_root):
        candidates = [f"/dev/{name}" for name in os.listdir(sysfs_root) if name.startswith("video")]
    else:
        candidates = [f"/dev/video{index}" for index in range(32)]
    devices = []
    for path in sorted(candidates, key=_natural_video_key):
        if not os.path.exists(path):
            continue
        device = query_capture_device(path)
        if device is not None:
            devices.append(device)
    return devices


def query_capture_device(path: str) -> CameraDevice | None:
    try:
        fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
    except OSError:
        return None
    try:
        buffer = bytearray(104)
        fcntl.ioctl(fd, VIDIOC_QUERYCAP, buffer, True)
    except OSError as exc:
        if exc.errno not in (errno.EINVAL, errno.ENOTTY):
            return None
        return None
    finally:
        os.close(fd)

    name = _c_string(buffer[16:48])
    bus = _c_string(buffer[48:80])
    capabilities = struct.unpack_from("I", buffer, 84)[0]
    device_caps = struct.unpack_from("I", buffer, 88)[0]
    active_caps = device_caps if capabilities & V4L2_CAP_DEVICE_CAPS else capabilities
    if not active_caps & V4L2_CAP_VIDEO_CAPTURE:
        return None
    return CameraDevice(path=path, name=name, bus=bus)


def get_camera_config(path: str) -> list[dict]:
    try:
        fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
    except OSError:
        return []
    try:
        formats = []
        for fmt_index in range(32):
            fmt_buffer = bytearray(64)
            struct.pack_into("II", fmt_buffer, 0, fmt_index, V4L2_BUF_TYPE_VIDEO_CAPTURE)
            try:
                fcntl.ioctl(fd, VIDIOC_ENUM_FMT, fmt_buffer, True)
            except OSError as exc:
                if exc.errno == errno.EINVAL:
                    break
                continue
            pixfmt = struct.unpack_from("I", fmt_buffer, 44)[0]
            formats.append(
                {
                    "format": _fourcc_from_int(pixfmt),
                    "description": _c_string(fmt_buffer[12:44]),
                    "sizes": _enum_frame_sizes(fd, pixfmt),
                }
            )
        return formats
    finally:
        os.close(fd)


def _enum_frame_sizes(fd: int, pixfmt: int) -> list[dict]:
    sizes = []
    for size_index in range(128):
        size_buffer = bytearray(44)
        struct.pack_into("II", size_buffer, 0, size_index, pixfmt)
        try:
            fcntl.ioctl(fd, VIDIOC_ENUM_FRAMESIZES, size_buffer, True)
        except OSError as exc:
            if exc.errno == errno.EINVAL:
                break
            continue
        size_type = struct.unpack_from("I", size_buffer, 8)[0]
        if size_type != V4L2_FRMSIZE_TYPE_DISCRETE:
            continue
        width, height = struct.unpack_from("II", size_buffer, 12)
        sizes.append({"width": width, "height": height, "fps": _enum_frame_intervals(fd, pixfmt, width, height)})
    return sizes


def _enum_frame_intervals(fd: int, pixfmt: int, width: int, height: int) -> list[int]:
    fps_values = []
    for interval_index in range(64):
        interval_buffer = bytearray(52)
        struct.pack_into("IIIII", interval_buffer, 0, interval_index, pixfmt, width, height, 0)
        try:
            fcntl.ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, interval_buffer, True)
        except OSError as exc:
            if exc.errno == errno.EINVAL:
                break
            continue
        interval_type = struct.unpack_from("I", interval_buffer, 16)[0]
        if interval_type != V4L2_FRMIVAL_TYPE_DISCRETE:
            continue
        numerator, denominator = struct.unpack_from("II", interval_buffer, 20)
        if numerator:
            fps_values.append(round(denominator / numerator))
    return sorted(set(fps_values), reverse=True)


def _natural_video_key(path: str) -> tuple[int, str]:
    match = re.search(r"video(\d+)$", path)
    return (int(match.group(1)) if match else 10_000, path)


def _c_string(raw: bytes) -> str:
    return raw.split(b"\0", 1)[0].decode("utf-8", "replace")


def _fourcc_from_int(value: int) -> str:
    return "".join(chr((value >> (8 * index)) & 0xFF) for index in range(4))

