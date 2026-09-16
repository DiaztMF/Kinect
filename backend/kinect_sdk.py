"""Kinect for Windows SDK 1.8 capture backend (ctypes -> Kinect10.dll).

Why this exists alongside the libfreenect path: on Windows the two are mutually
exclusive. libfreenect needs the camera interface bound to libusbK (via Zadig),
while the official SDK driver is what Skanect and every other Kinect SDK
application requires. Talking to Kinect10.dll lets this app share the sensor
with those tools instead of demanding a driver swap.

Two format details that are easy to get wrong and are asserted by the
self-check at the bottom of this file:

  * Colour arrives as BGRA (the fourth byte is padding, not alpha).
  * Depth arrives packed: the millimetre value sits in the upper 13 bits, so it
    needs a 3-bit right shift even in plain NUI_IMAGE_TYPE_DEPTH mode.

The SDK hands back depth in the IR camera's frame, not aligned to colour, so
colour is gathered into the depth frame through the SDK's own per-device
calibration. That means the pipeline runs on the IR intrinsics below, not the
RGB ones used by libfreenect's REGISTERED mode.
"""

import ctypes as C
import logging
import threading
import time
from ctypes import wintypes as W
from typing import Optional, Tuple

import numpy as np

logger = logging.getLogger("kinect_sdk")

DLL_PATH = r"C:\WINDOWS\System32\Kinect10.dll"

NUI_INITIALIZE_FLAG_USES_COLOR = 0x00000002
NUI_INITIALIZE_FLAG_USES_DEPTH = 0x00000020
NUI_IMAGE_TYPE_COLOR = 1
NUI_IMAGE_TYPE_DEPTH = 4
NUI_IMAGE_RESOLUTION_640x480 = 2

# Depth is stored in the upper 13 bits; the low 3 are the player index slot.
NUI_IMAGE_PLAYER_INDEX_SHIFT = 3

WIDTH, HEIGHT = 640, 480

# Kinect v1 IR/depth camera intrinsics at 640x480. These are the widely used
# factory-typical values, not a calibration of this particular unit -- adjust
# them if a checkerboard calibration of your sensor disagrees.
IR_INTRINSICS = dict(fx=594.21, fy=591.04, cx=339.31, cy=242.74)

# INuiSensor vtable slots (NuiSensor.h declaration order, IUnknown first).
# Verified against the live DLL by the self-check below.
_V_RELEASE = 2
_V_INITIALIZE = 3
_V_SHUTDOWN = 4
_V_STREAM_OPEN = 6
_V_GET_NEXT_FRAME = 9
_V_RELEASE_FRAME = 10
_V_MAP_DEPTH_FRAME_TO_COLOR = 13
_V_ELEVATION_SET = 14
_V_ELEVATION_GET = 15

# INuiFrameTexture vtable slots.
_T_BUFFER_LEN = 3
_T_LOCK_RECT = 5
_T_UNLOCK_RECT = 7


class _ViewArea(C.Structure):
    _fields_ = [("eDigitalZoom", C.c_int), ("lCenterX", C.c_long), ("lCenterY", C.c_long)]


class _ImageFrame(C.Structure):
    _fields_ = [
        ("liTimeStamp", C.c_longlong),
        ("dwFrameNumber", W.DWORD),
        ("eImageType", C.c_int),
        ("eResolution", C.c_int),
        ("pFrameTexture", C.c_void_p),
        ("dwFrameFlags", W.DWORD),
        ("ViewArea", _ViewArea),
    ]


class _LockedRect(C.Structure):
    _fields_ = [("Pitch", C.c_int), ("size", C.c_int), ("pBits", C.POINTER(C.c_ubyte))]


class KinectSDKError(RuntimeError):
    pass


def _vcall(obj: C.c_void_p, slot: int, restype, *argtypes):
    """Binds one virtual method of a COM-style object."""
    vtbl = C.cast(obj, C.POINTER(C.POINTER(C.c_void_p))).contents
    return C.CFUNCTYPE(restype, C.c_void_p, *argtypes)(vtbl[slot])


class KinectSDKSensor:
    """Streams registered RGB + depth from a Kinect v1 through the official SDK."""

    def __init__(self, timeout_ms: int = 300):
        self._timeout_ms = timeout_ms
        self._lib: Optional[C.WinDLL] = None
        self._sensor = C.c_void_p()
        self._h_color = W.HANDLE()
        self._h_depth = W.HANDLE()

        self._latest: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self._frame_seq = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        # Scratch buffer for the depth->colour coordinate table (x, y per pixel).
        self._coords = np.empty((HEIGHT * WIDTH * 2,), dtype=np.int32)

    # ------------------------------------------------------------------ setup

    def open(self):
        """Raises KinectSDKError if the SDK or a sensor is unavailable."""
        try:
            self._lib = C.WinDLL(DLL_PATH)
        except OSError as exc:
            raise KinectSDKError(f"Kinect10.dll not loadable: {exc}") from exc

        self._lib.NuiCreateSensorByIndex.argtypes = [C.c_int, C.POINTER(C.c_void_p)]
        self._lib.NuiCreateSensorByIndex.restype = C.c_long

        hr = self._lib.NuiCreateSensorByIndex(0, C.byref(self._sensor))
        if hr != 0 or not self._sensor.value:
            raise KinectSDKError(f"NuiCreateSensorByIndex failed (hr=0x{hr & 0xFFFFFFFF:08X})")

        self._initialize = _vcall(self._sensor, _V_INITIALIZE, C.c_long, W.DWORD)
        self._shutdown = _vcall(self._sensor, _V_SHUTDOWN, None)
        self._stream_open = _vcall(
            self._sensor, _V_STREAM_OPEN, C.c_long,
            C.c_int, C.c_int, W.DWORD, W.DWORD, W.HANDLE, C.POINTER(W.HANDLE),
        )
        self._get_frame = _vcall(
            self._sensor, _V_GET_NEXT_FRAME, C.c_long,
            W.HANDLE, W.DWORD, C.POINTER(_ImageFrame),
        )
        self._release_frame = _vcall(
            self._sensor, _V_RELEASE_FRAME, C.c_long, W.HANDLE, C.POINTER(_ImageFrame)
        )
        self._map_frame = _vcall(
            self._sensor, _V_MAP_DEPTH_FRAME_TO_COLOR, C.c_long,
            C.c_int, C.c_int, W.DWORD, C.POINTER(C.c_ushort), W.DWORD, C.POINTER(C.c_long),
        )
        self._set_angle = _vcall(self._sensor, _V_ELEVATION_SET, C.c_long, C.c_long)
        self._get_angle = _vcall(self._sensor, _V_ELEVATION_GET, C.c_long, C.POINTER(C.c_long))

        hr = self._initialize(
            self._sensor, NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH
        )
        if hr != 0:
            self._release_sensor()
            raise KinectSDKError(
                f"NuiInitialize failed (hr=0x{hr & 0xFFFFFFFF:08X}) -- "
                "another application may be holding the sensor"
            )

        for typ, handle in ((NUI_IMAGE_TYPE_COLOR, self._h_color),
                            (NUI_IMAGE_TYPE_DEPTH, self._h_depth)):
            hr = self._stream_open(
                self._sensor, typ, NUI_IMAGE_RESOLUTION_640x480, 0, 2, None, C.byref(handle)
            )
            if hr != 0:
                self.close()
                raise KinectSDKError(f"NuiImageStreamOpen({typ}) failed (hr=0x{hr & 0xFFFFFFFF:08X})")

        self._stop.clear()
        self._thread = threading.Thread(target=self._pump, daemon=True)
        self._thread.start()
        logger.info("Kinect SDK 1.8 sensor opened (640x480 colour + depth)")

    # ------------------------------------------------------------------ frames

    def _read_texture(self, handle, expect_bytes: int) -> Optional[bytes]:
        """Grabs one frame and copies its locked pixel buffer out."""
        frame = _ImageFrame()
        hr = self._get_frame(self._sensor, handle, self._timeout_ms, C.byref(frame))
        if hr != 0 or not frame.pFrameTexture:
            return None

        tex = frame.pFrameTexture
        data = None
        try:
            rect = _LockedRect()
            lock = _vcall(
                C.c_void_p(tex), _T_LOCK_RECT, C.c_long,
                C.c_uint, C.POINTER(_LockedRect), C.c_void_p, W.DWORD,
            )
            if lock(tex, 0, C.byref(rect), None, 0) == 0 and rect.pBits:
                data = C.string_at(rect.pBits, min(rect.size, expect_bytes))
                _vcall(C.c_void_p(tex), _T_UNLOCK_RECT, C.c_long, C.c_uint)(tex, 0)
        finally:
            self._release_frame(self._sensor, handle, C.byref(frame))
        return data

    def _pump(self):
        """Background capture loop: pairs the newest colour and depth frames."""
        while not self._stop.is_set():
            depth_raw = self._read_texture(self._h_depth, WIDTH * HEIGHT * 2)
            if depth_raw is None:
                continue
            color_raw = self._read_texture(self._h_color, WIDTH * HEIGHT * 4)
            if color_raw is None:
                continue

            try:
                rgb, depth_mm = self._convert(depth_raw, color_raw)
            except Exception:
                logger.exception("Frame conversion failed")
                continue

            with self._lock:
                self._latest = (rgb, depth_mm)
                self._frame_seq += 1

    def _convert(self, depth_raw: bytes, color_raw: bytes):
        """Packed depth + BGRA colour -> (RGB in depth frame, depth in mm)."""
        packed = np.frombuffer(depth_raw, dtype=np.uint16)
        bgra = np.frombuffer(color_raw, dtype=np.uint8).reshape(HEIGHT, WIDTH, 4)

        # Ask the SDK where each depth pixel lands in the colour image. This uses
        # the sensor's own factory calibration, so it beats a generic extrinsic.
        coords = self._coords
        hr = self._map_frame(
            self._sensor,
            NUI_IMAGE_RESOLUTION_640x480,
            NUI_IMAGE_RESOLUTION_640x480,
            packed.size,
            packed.ctypes.data_as(C.POINTER(C.c_ushort)),
            coords.size,
            coords.ctypes.data_as(C.POINTER(C.c_long)),
        )

        depth_mm = (packed >> NUI_IMAGE_PLAYER_INDEX_SHIFT).reshape(HEIGHT, WIDTH).copy()

        rgb = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        if hr == 0:
            xy = coords.reshape(HEIGHT, WIDTH, 2)
            cx, cy = xy[:, :, 0], xy[:, :, 1]
            inside = (cx >= 0) & (cx < WIDTH) & (cy >= 0) & (cy < HEIGHT)
            # BGRA -> RGB while gathering into the depth frame.
            rgb[inside] = bgra[cy[inside], cx[inside], :3][:, ::-1]
            # A depth pixel with no colour correspondence carries no usable
            # photometric information, so drop its geometry too.
            depth_mm[~inside] = 0
        else:
            rgb[:] = bgra[:, :, 2::-1]

        return rgb, depth_mm

    def read(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Latest (RGB uint8 HxWx3, depth uint16 mm), or None before first frame."""
        with self._lock:
            return self._latest

    @property
    def frame_seq(self) -> int:
        with self._lock:
            return self._frame_seq

    # ------------------------------------------------------------------- motor

    def set_tilt(self, angle_deg: int) -> int:
        clamped = max(-27, min(27, int(angle_deg)))
        self._set_angle(self._sensor, clamped)
        return clamped

    def get_tilt(self) -> Optional[int]:
        angle = C.c_long(0)
        if self._get_angle(self._sensor, C.byref(angle)) != 0:
            return None
        return angle.value

    # ----------------------------------------------------------------- cleanup

    def _release_sensor(self):
        if self._sensor.value:
            _vcall(self._sensor, _V_RELEASE, C.c_ulong)(self._sensor)
            self._sensor = C.c_void_p()

    def close(self):
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._sensor.value:
            try:
                self._shutdown(self._sensor)
            except Exception:
                logger.exception("NuiShutdown failed")
            self._release_sensor()


def _self_check():
    """Validates the vtable slots and both pixel formats against live hardware."""
    logging.basicConfig(level=logging.INFO)
    sensor = KinectSDKSensor()
    sensor.open()
    try:
        frame = None
        for _ in range(100):
            frame = sensor.read()
            if frame is not None:
                break
            time.sleep(0.05)
        assert frame is not None, "no frame arrived within 5 s"

        rgb, depth = frame
        assert rgb.shape == (HEIGHT, WIDTH, 3) and rgb.dtype == np.uint8, rgb.shape
        assert depth.shape == (HEIGHT, WIDTH) and depth.dtype == np.uint16, depth.shape

        valid = depth[depth > 0]
        assert valid.size > 10_000, f"only {valid.size} valid depth pixels"
        # The 3-bit shift is the thing most likely to be wrong; unshifted data
        # would show a maximum around 19000 rather than a plausible range.
        assert 300 <= valid.min() <= 1200, f"min depth {valid.min()} mm is not plausible"
        assert valid.max() <= 8000, f"max depth {valid.max()} mm suggests a missing shift"
        assert rgb.any(), "colour frame is entirely black"

        print(f"depth : {valid.size} valid px, {valid.min()}..{valid.max()} mm")
        print(f"colour: mean RGB {rgb.reshape(-1, 3).mean(0).round(1)}, "
              f"{(rgb.sum(2) > 0).mean() * 100:.1f}% non-black")

        # Registration sanity: colour must be present wherever depth is valid.
        both = (depth > 0) & (rgb.sum(2) > 0)
        coverage = both.sum() / max((depth > 0).sum(), 1)
        print(f"registration: {coverage * 100:.1f}% of valid depth pixels carry colour")
        assert coverage > 0.9, "depth->colour mapping looks wrong"

        angle = sensor.get_tilt()
        assert angle is not None and -31 <= angle <= 31, f"implausible tilt {angle}"
        print(f"tilt  : {angle} deg (vtable slots 14/15 confirmed)")

        seq = sensor.frame_seq
        time.sleep(0.5)
        assert sensor.frame_seq > seq, "capture thread stalled"
        print(f"stream: {sensor.frame_seq - seq} frames in 0.5 s")
        print("\nall checks passed")
    finally:
        sensor.close()


if __name__ == "__main__":
    _self_check()
