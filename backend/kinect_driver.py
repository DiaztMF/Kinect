"""
Kinect Driver for Windows Kinect v1 (Xbox 360 / Model 1414).

Three backends, tried in order:

1. Kinect for Windows SDK 1.8 (backend/kinect_sdk.py). Preferred, because this
   is the driver Skanect and the rest of the Kinect SDK ecosystem need -- the
   sensor stays usable by those tools instead of requiring a Zadig swap.
2. libfreenect.dll + libusb-1.0. Requires the camera interface to be rebound to
   libusbK, which locks out every Kinect SDK application.
3. Mock mode: synthetic indoor room, so the pipeline runs with no hardware.

All three yield 640x480 RGB uint8 plus 640x480 uint16 depth in millimetres,
with colour and depth pixel-aligned. They do *not* share intrinsics -- see
KinectDriver.intrinsics.
"""

import ctypes
import logging
import math
import os
import sys
import threading
import time
from typing import Optional, Tuple

import numpy as np

from backend import kinect_sdk
from backend.kinect_sdk import KinectSDKSensor

logger = logging.getLogger("kinect_driver")


class FreenectFrameMode(ctypes.Structure):
    _fields_ = [
        ("reserved", ctypes.c_uint32),
        ("resolution", ctypes.c_int32),
        ("format", ctypes.c_int32),
        ("bytes", ctypes.c_int32),
        ("width", ctypes.c_int16),
        ("height", ctypes.c_int16),
        ("data_bits_per_pixel", ctypes.c_int8),
        ("padding_bits_per_pixel", ctypes.c_int8),
        ("framerate", ctypes.c_int8),
        ("is_valid", ctypes.c_int8),
    ]


class KinectDriver:
    def __init__(self, mock: bool = False):
        self.mock = mock
        self._is_real_initialized = False
        self._frame_idx = 0
        self._start_time = time.time()
        self._tilt_angle = 0

        # Hardware handles
        self._fn_lib = None
        self._ctx = None
        self._dev = None
        self._latest_video: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None
        # Bumped on every new depth frame so consumers can skip re-processing
        # a frame they have already seen (the sensor runs at 30 Hz).
        self._frame_seq = 0
        self._mock_rays: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self._rng = np.random.default_rng()
        self._worker_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._frame_lock = threading.Lock()

        # Callbacks keep-alive references
        self._cb_depth = None
        self._cb_video = None

        # Official Kinect for Windows SDK sensor, when that driver is bound.
        self._sdk: Optional["KinectSDKSensor"] = None

        if not self.mock:
            # Prefer the official SDK: it shares the sensor with Skanect and the
            # rest of the Kinect SDK ecosystem, whereas libfreenect requires the
            # camera interface to be rebound to libusbK and locks those out.
            self._init_sdk()
            if not self._is_real_initialized:
                self._init_hardware()

        if not self._is_real_initialized:
            self.mock = True
            logger.info("KinectDriver operating in MOCK mode.")
        else:
            logger.info("KinectDriver operating in REAL hardware mode (%s).", self.backend)

    @property
    def backend(self) -> str:
        if self._sdk is not None:
            return "kinect-sdk-1.8"
        if self._is_real_initialized:
            return "libfreenect"
        return "mock"

    @property
    def intrinsics(self) -> dict:
        """Camera intrinsics matching whichever frame the depth map lives in.

        The SDK backend gathers colour into the *IR* camera's frame, while
        libfreenect's REGISTERED mode and the mock both produce depth already
        warped into the RGB frame. Feeding the wrong set to the SLAM engine
        shears every reconstruction, so the driver reports its own.
        """
        if self._sdk is not None:
            return dict(kinect_sdk.IR_INTRINSICS)
        return dict(fx=525.0, fy=525.0, cx=320.0, cy=240.0)

    def _init_sdk(self):
        """Attempts to open the sensor through Kinect for Windows SDK 1.8."""
        try:
            sensor = kinect_sdk.KinectSDKSensor()
            sensor.open()
        except Exception as exc:
            logger.info("Kinect SDK backend unavailable: %s", exc)
            return

        # Wait for the first real frame before declaring success, so a sensor
        # that opens but never streams falls through to the next backend.
        for _ in range(60):
            if sensor.read() is not None:
                self._sdk = sensor
                self._is_real_initialized = True
                return
            time.sleep(0.05)

        logger.warning("Kinect SDK opened but delivered no frames; falling back.")
        sensor.close()

    def _init_hardware(self):
        """Attempts to load libfreenect.dll and initialize the physical sensor."""
        backend_dir = os.path.dirname(os.path.abspath(__file__))
        dll_path = os.path.join(backend_dir, "libfreenect.dll")
        libusb_path = os.path.join(backend_dir, "libusb-1.0.dll")

        if not os.path.exists(dll_path) or not os.path.exists(libusb_path):
            logger.warning("libfreenect.dll or libusb-1.0.dll not found in backend directory.")
            return

        try:
            os.environ["PATH"] = backend_dir + ";" + os.environ.get("PATH", "")
            ctypes.CDLL(libusb_path)
            self._fn_lib = ctypes.CDLL(dll_path)

            freenect_context = ctypes.c_void_p
            freenect_device = ctypes.c_void_p

            self._fn_lib.freenect_init.argtypes = [ctypes.POINTER(freenect_context), ctypes.c_void_p]
            self._fn_lib.freenect_init.restype = ctypes.c_int

            self._fn_lib.freenect_open_device.argtypes = [freenect_context, ctypes.POINTER(freenect_device), ctypes.c_int]
            self._fn_lib.freenect_open_device.restype = ctypes.c_int

            self._fn_lib.freenect_close_device.argtypes = [freenect_device]
            self._fn_lib.freenect_close_device.restype = ctypes.c_int

            self._fn_lib.freenect_shutdown.argtypes = [freenect_context]
            self._fn_lib.freenect_shutdown.restype = ctypes.c_int

            self._fn_lib.freenect_find_depth_mode.argtypes = [ctypes.c_int, ctypes.c_int]
            self._fn_lib.freenect_find_depth_mode.restype = FreenectFrameMode

            self._fn_lib.freenect_find_video_mode.argtypes = [ctypes.c_int, ctypes.c_int]
            self._fn_lib.freenect_find_video_mode.restype = FreenectFrameMode

            self._fn_lib.freenect_set_depth_mode.argtypes = [freenect_device, FreenectFrameMode]
            self._fn_lib.freenect_set_depth_mode.restype = ctypes.c_int

            self._fn_lib.freenect_set_video_mode.argtypes = [freenect_device, FreenectFrameMode]
            self._fn_lib.freenect_set_video_mode.restype = ctypes.c_int

            depth_callback_t = ctypes.CFUNCTYPE(None, freenect_device, ctypes.c_void_p, ctypes.c_uint32)
            video_callback_t = ctypes.CFUNCTYPE(None, freenect_device, ctypes.c_void_p, ctypes.c_uint32)

            self._fn_lib.freenect_set_depth_callback.argtypes = [freenect_device, depth_callback_t]
            self._fn_lib.freenect_set_video_callback.argtypes = [freenect_device, video_callback_t]

            self._fn_lib.freenect_start_depth.argtypes = [freenect_device]
            self._fn_lib.freenect_start_depth.restype = ctypes.c_int

            self._fn_lib.freenect_start_video.argtypes = [freenect_device]
            self._fn_lib.freenect_start_video.restype = ctypes.c_int

            self._fn_lib.freenect_stop_depth.argtypes = [freenect_device]
            self._fn_lib.freenect_stop_depth.restype = ctypes.c_int

            self._fn_lib.freenect_stop_video.argtypes = [freenect_device]
            self._fn_lib.freenect_stop_video.restype = ctypes.c_int

            self._fn_lib.freenect_process_events.argtypes = [freenect_context]
            self._fn_lib.freenect_process_events.restype = ctypes.c_int

            self._fn_lib.freenect_set_tilt_degs.argtypes = [freenect_device, ctypes.c_double]
            self._fn_lib.freenect_set_tilt_degs.restype = ctypes.c_int

            self._fn_lib.freenect_set_led.argtypes = [freenect_device, ctypes.c_int]
            self._fn_lib.freenect_set_led.restype = ctypes.c_int

            self._ctx = freenect_context()
            init_res = self._fn_lib.freenect_init(ctypes.byref(self._ctx), None)
            if init_res != 0:
                logger.warning("freenect_init failed with code %d", init_res)
                return

            self._dev = freenect_device()
            # Retry opening up to 3 times in case of transient LIBUSB_ERROR_ACCESS
            open_res = -1
            for attempt in range(3):
                open_res = self._fn_lib.freenect_open_device(self._ctx, ctypes.byref(self._dev), 0)
                if open_res == 0:
                    break
                time.sleep(0.5)

            if open_res != 0:
                logger.warning("freenect_open_device failed with code %d", open_res)
                return

            # Set Solid Green LED
            self._fn_lib.freenect_set_led(self._dev, 1)

            # Setup modes: 1 = MEDIUM (640x480), 4 = REGISTERED (mm), 0 = RGB
            d_mode = self._fn_lib.freenect_find_depth_mode(1, 4)
            v_mode = self._fn_lib.freenect_find_video_mode(1, 0)
            self._fn_lib.freenect_set_depth_mode(self._dev, d_mode)
            self._fn_lib.freenect_set_video_mode(self._dev, v_mode)

            def _depth_cb(dev, depth_ptr, timestamp):
                buf = (ctypes.c_uint16 * (640 * 480)).from_address(depth_ptr)
                arr = np.frombuffer(buf, dtype=np.uint16).reshape((480, 640)).copy()
                with self._frame_lock:
                    self._latest_depth = arr
                    self._frame_seq += 1

            def _video_cb(dev, video_ptr, timestamp):
                buf = (ctypes.c_uint8 * (640 * 480 * 3)).from_address(video_ptr)
                arr = np.frombuffer(buf, dtype=np.uint8).reshape((480, 640, 3)).copy()
                with self._frame_lock:
                    self._latest_video = arr

            self._cb_depth = depth_callback_t(_depth_cb)
            self._cb_video = video_callback_t(_video_cb)

            self._fn_lib.freenect_set_depth_callback(self._dev, self._cb_depth)
            self._fn_lib.freenect_set_video_callback(self._dev, self._cb_video)

            self._fn_lib.freenect_start_depth(self._dev)
            self._fn_lib.freenect_start_video(self._dev)

            # Background thread to process libfreenect USB events with higher responsiveness
            self._stop_event.clear()
            self._worker_thread = threading.Thread(target=self._event_loop, daemon=True)
            self._worker_thread.start()

            self._is_real_initialized = True
            self.mock = False
            logger.info("Successfully opened physical Kinect v1 hardware!")
        except Exception as e:
            logger.error("Failed opening physical Kinect sensor: %s", e)
            self._is_real_initialized = False

    def _event_loop(self):
        # Set zero timeval for non-blocking immediate event pump
        class Timeval(ctypes.Structure):
            _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]

        zero_tv = Timeval(0, 0)
        has_timeout_fn = hasattr(self._fn_lib, "freenect_process_events_timeout")

        while not self._stop_event.is_set():
            if self._fn_lib and self._ctx:
                try:
                    if has_timeout_fn:
                        self._fn_lib.freenect_process_events_timeout(self._ctx, ctypes.byref(zero_tv))
                    else:
                        self._fn_lib.freenect_process_events(self._ctx)
                except Exception:
                    pass
            # Short yield to prevent CPU starvation while keeping USB buffer drained
            time.sleep(0.001)

    @property
    def frame_seq(self) -> int:
        """Monotonic counter of depth frames delivered by the sensor."""
        if self._sdk is not None:
            return self._sdk.frame_seq
        with self._frame_lock:
            return self._frame_seq

    def set_tilt(self, angle_deg: int):
        """Sets hardware motor tilt (-27 to +27 degrees)."""
        clamped = max(-27, min(27, angle_deg))
        self._tilt_angle = clamped
        if self._sdk is not None:
            try:
                return self._sdk.set_tilt(clamped)
            except Exception as e:
                logger.error("Error setting hardware tilt: %s", e)
                return self._tilt_angle
        if self._is_real_initialized and self._fn_lib and self._dev:
            try:
                self._fn_lib.freenect_set_tilt_degs(self._dev, float(clamped))
            except Exception as e:
                logger.error("Error setting hardware tilt: %s", e)
        return self._tilt_angle

    def get_frame(self) -> Tuple[np.ndarray, np.ndarray]:
        """Returns tuple of (RGB uint8 [480, 640, 3], Depth uint16 [480, 640] in mm)."""
        if self.mock or not self._is_real_initialized:
            return self._generate_mock_frame()

        if self._sdk is not None:
            frame = self._sdk.read()
            if frame is not None:
                return frame
            # read() only returns None before the very first frame, and
            # _init_sdk already waited for that one. Falling back to a synthetic
            # frame here would splice mock geometry -- built on different
            # intrinsics -- into a real scan, so wait, then fail loudly.
            for _ in range(20):
                time.sleep(0.01)
                frame = self._sdk.read()
                if frame is not None:
                    return frame
            raise RuntimeError("Kinect SDK sensor stopped delivering frames")

        # The callbacks always publish freshly allocated arrays and never
        # mutate them afterwards, so handing out the reference is safe.
        with self._frame_lock:
            if self._latest_video is not None and self._latest_depth is not None:
                return self._latest_video, self._latest_depth

        # Wait briefly for first frames to arrive
        for _ in range(20):
            time.sleep(0.01)
            with self._frame_lock:
                if self._latest_video is not None and self._latest_depth is not None:
                    return self._latest_video, self._latest_depth

        return self._generate_mock_frame()

    def close(self):
        if self._sdk is not None:
            self._sdk.close()
            self._sdk = None
        self._stop_event.set()
        if self._worker_thread and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=1.0)
        if self._is_real_initialized and self._fn_lib:
            try:
                if self._dev:
                    self._fn_lib.freenect_stop_depth(self._dev)
                    self._fn_lib.freenect_stop_video(self._dev)
                    self._fn_lib.freenect_close_device(self._dev)
                if self._ctx:
                    self._fn_lib.freenect_shutdown(self._ctx)
            except Exception as e:
                logger.error("Error shutting down freenect: %s", e)

    def _generate_mock_frame(self) -> Tuple[np.ndarray, np.ndarray]:
        """Generates synthetic indoor room with textured floor/walls and 3D box obstacle."""
        width, height = 640, 480
        fx, fy = 525.0, 525.0
        cx, cy = 320.0, 240.0

        self._frame_idx += 1
        with self._frame_lock:
            self._frame_seq += 1
        t = (time.time() - self._start_time) * 0.5
        cam_x = 0.15 * math.sin(t)
        cam_y = 0.08 * math.cos(t * 0.8)
        cam_z = 0.10 * math.sin(t * 0.6)

        if self._mock_rays is None:
            vv, uu = np.mgrid[0:height, 0:width]
            # float32 throughout: halves the memory traffic of the ~25 array
            # ops below, which is what dominates synthetic-frame cost.
            self._mock_rays = (
                ((uu - cx) / fx).astype(np.float32),
                ((vv - cy) / fy).astype(np.float32),
            )
        ray_x, ray_y = self._mock_rays

        depth = np.full((height, width), 2800.0, dtype=np.float32)
        rgb = np.zeros((height, width, 3), dtype=np.uint8)
        rgb[:, :] = [35, 35, 42]

        floor_mask = ray_y > 0.15
        floor_y = 0.9 - cam_y
        dist_floor = floor_y / np.maximum(ray_y, 0.001)
        z_floor = dist_floor * 1000.0
        valid_floor = floor_mask & (z_floor > 800) & (z_floor < 3500)
        depth[valid_floor] = np.minimum(depth[valid_floor], z_floor[valid_floor])

        fx_coord = (ray_x * dist_floor + cam_x) * 4.0
        fz_coord = (dist_floor + cam_z) * 4.0
        checker = ((fx_coord.astype(np.int32) + fz_coord.astype(np.int32)) % 2) == 0
        rgb[valid_floor & checker] = [70, 75, 85]
        rgb[valid_floor & ~checker] = [50, 55, 65]

        box_z = 1.8 - cam_z
        box_x = 0.0 - cam_x
        box_y = 0.2 - cam_y
        box_w, box_h = 0.35, 0.35

        p_x = ray_x * box_z
        p_y = ray_y * box_z
        box_mask = (np.abs(p_x - box_x) < box_w) & (np.abs(p_y - box_y) < box_h)
        depth[box_mask] = box_z * 1000.0
        rgb[box_mask] = [210, 140, 80]

        depth += self._rng.standard_normal(depth.shape, dtype=np.float32) * 5.0
        depth = np.clip(depth, 800.0, 3500.0).astype(np.uint16)

        return rgb, depth
