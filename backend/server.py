"""
FastAPI WebSocket and REST Server for Kinect v1 3D Room Scanner.

Endpoints:
- GET /api/status: Driver status, SLAM tracking status, total point count.
- POST /api/export: Saves current global point cloud to backend/exports/*.ply and returns path.
- WebSocket /ws/scan: Binary point cloud stream and JSON control/telemetry channel.
"""

import argparse
import asyncio
import json
import logging
import os
import time
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

# Ensure project root is in sys.path when running script directly
project_root = str(Path(__file__).resolve().parent.parent)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

import open3d as o3d
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from backend.kinect_driver import KinectDriver
from backend.slam_engine import SLAMEngine

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
logger = logging.getLogger("server")

# How often the surface is re-extracted and pushed to the viewer, and the
# floor on how much of the session that work may occupy (1 / N).
MESH_INTERVAL_S = 2.0
MESH_DUTY_DIVISOR = 9.0

EXPORTS_DIR = Path(__file__).parent / "exports"
EXPORTS_DIR.mkdir(parents=True, exist_ok=True)


class ScanServerState:
    def __init__(self, mock: bool = False, tsdf_voxel: float = 0.012):
        self.mock_requested = mock
        self.driver = KinectDriver(mock=mock)
        # Intrinsics follow the active backend: the SDK leaves depth in the IR
        # camera's frame, libfreenect and the mock deliver it in the RGB frame.
        self.slam = SLAMEngine(
            voxel_size=0.03,
            max_depth=3.5,
            tsdf_voxel=tsdf_voxel,
            # sdf_trunc tracks the voxel; ~4 voxels is the usual working ratio.
            tsdf_trunc=max(0.012, tsdf_voxel * 4.0),
            **self.driver.intrinsics,
        )
        self.is_streaming = False
        self.last_status: Dict[str, Any] = {
            "tracking_ok": True,
            "pose": [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
            "fps": 0.0,
            "frame_count": 0,
        }

    def reset_slam(self):
        self.slam.reset()
        self.last_status["tracking_ok"] = True
        self.last_status["pose"] = self.slam.current_pose.tolist()


def _free_ram_gb() -> Optional[float]:
    """Free physical RAM, or None off Windows. Avoids a psutil dependency."""
    try:
        import ctypes
        from ctypes import wintypes

        class MemoryStatus(ctypes.Structure):
            _fields_ = [("dwLength", wintypes.DWORD), ("dwMemoryLoad", wintypes.DWORD),
                        ("ullTotalPhys", ctypes.c_ulonglong),
                        ("ullAvailPhys", ctypes.c_ulonglong),
                        ("ullTotalPageFile", ctypes.c_ulonglong),
                        ("ullAvailPageFile", ctypes.c_ulonglong),
                        ("ullTotalVirtual", ctypes.c_ulonglong),
                        ("ullAvailVirtual", ctypes.c_ulonglong),
                        ("ullAvailExtendedVirtual", ctypes.c_ulonglong)]

        status = MemoryStatus()
        status.dwLength = ctypes.sizeof(status)
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.GlobalMemoryStatusEx.argtypes = [ctypes.POINTER(MemoryStatus)]
        kernel32.GlobalMemoryStatusEx.restype = wintypes.BOOL
        if not kernel32.GlobalMemoryStatusEx(ctypes.byref(status)):
            return None
        return status.ullAvailPhys / 1e9
    except Exception:
        return None


def create_app(mock: bool = False, tsdf_voxel: float = 0.012) -> FastAPI:
    app = FastAPI(title="Kinect v1 3D Room Scanner API", version="1.0.0")

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # A fine voxel that exhausts RAM shows up as the server simply vanishing,
    # with nothing in the log to explain it. Say so up front instead.
    free_gb = _free_ram_gb()
    if tsdf_voxel < 0.010 and free_gb is not None and free_gb < 2.0:
        logger.warning(
            "Surface voxel is %.0f mm with only %.1f GB RAM free. Room-scale "
            "scans at this resolution have been OOM-killed here; pass "
            "--voxel-mm 12 (or higher) if the server dies mid-scan.",
            tsdf_voxel * 1000, free_gb,
        )

    state = ScanServerState(mock=mock, tsdf_voxel=tsdf_voxel)
    app.state.scan = state

    @app.get("/api/status")
    async def get_status():
        curr_state: ScanServerState = app.state.scan
        return {
            "driver": {
                "mock": curr_state.driver.mock,
                "is_real_initialized": curr_state.driver._is_real_initialized,
                "backend": curr_state.driver.backend,
                "intrinsics": curr_state.driver.intrinsics,
            },
            "surface": {
                "voxel_mm": round(curr_state.slam.tsdf_voxel * 1000, 1),
                "revision": curr_state.slam.tsdf_revision,
            },
            "slam": {
                "tracking_ok": curr_state.last_status.get("tracking_ok", True),
                "is_streaming": curr_state.is_streaming,
                "total_points": curr_state.slam.total_points,
                "current_pose": curr_state.slam.current_pose.tolist(),
                "fps": round(curr_state.last_status.get("fps", 0.0), 1),
            },
            "timestamp": time.time(),
        }

    @app.post("/api/export")
    async def export_ply():
        curr_state: ScanServerState = app.state.scan
        pcd = await asyncio.to_thread(lambda: curr_state.slam.global_pcd)
        point_count = len(pcd.points)

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"scan_{timestamp_str}_{point_count}pts.ply"
        filepath = EXPORTS_DIR / filename

        success = False
        if point_count > 0:
            success = await asyncio.to_thread(
                lambda: o3d.io.write_point_cloud(str(filepath), pcd, write_ascii=False)
            )
        else:
            # Write minimal valid PLY header with 0 vertices
            header = (
                "ply\n"
                "format ascii 1.0\n"
                "element vertex 0\n"
                "property float x\n"
                "property float y\n"
                "property float z\n"
                "property uchar red\n"
                "property uchar green\n"
                "property uchar blue\n"
                "property uchar alpha\n"
                "end_header\n"
            )
            filepath.write_text(header, encoding="utf-8")
            success = True

        return {
            "success": success,
            "filename": filename,
            "filepath": str(filepath.resolve()),
            "point_count": point_count,
        }

    @app.post("/api/export_mesh")
    async def export_mesh():
        curr_state: ScanServerState = app.state.scan
        mesh = await asyncio.to_thread(curr_state.slam.extract_mesh)
        triangle_count = len(mesh.triangles)

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"mesh_{timestamp_str}_{triangle_count}tri.ply"
        filepath = EXPORTS_DIR / filename

        success = False
        if triangle_count > 0:
            success = await asyncio.to_thread(
                lambda: o3d.io.write_triangle_mesh(str(filepath), mesh, write_ascii=False)
            )
        else:
            header = (
                "ply\nformat ascii 1.0\nelement vertex 0\nproperty float x\nproperty float y\nproperty float z\n"
                "element face 0\nproperty list uchar int vertex_indices\nend_header\n"
            )
            filepath.write_text(header, encoding="utf-8")
            success = True

        logger.info("Exported 3D TSDF mesh (%d triangles) to %s", triangle_count, filepath)
        return {
            "success": success,
            "filename": filename,
            "filepath": str(filepath.resolve()),
            "triangle_count": triangle_count,
            "type": "mesh"
        }

    @app.websocket("/ws/scan")
    async def websocket_scan(websocket: WebSocket):
        await websocket.accept()
        curr_state: ScanServerState = app.state.scan
        logger.info("Client connected to /ws/scan")

        # By default, connection is ready but starts streaming on "start" or starts immediately if preferred
        stop_event = asyncio.Event()
        # The mesh worker and the frame loop both write to this socket, and a
        # websocket frame cannot be interleaved with another, so serialise them.
        send_lock = asyncio.Lock()
        mesh_wanted = asyncio.Event()

        async def send_mesh():
            slam = curr_state.slam
            revision = slam.tsdf_revision
            started = time.time()
            buf = await asyncio.to_thread(slam.mesh_buffer)
            elapsed = time.time() - started
            async with send_lock:
                await websocket.send_bytes(buf)
            return revision, elapsed

        async def mesh_worker():
            """Re-extracts the surface on a timer rather than per frame.

            Marching cubes over the whole volume costs 25-230 ms depending on
            voxel size, far more than a frame budget, and it takes the TSDF lock
            while it runs. Extracting only when the volume actually changed
            keeps a parked scanner from re-meshing an unchanged scene.
            """
            sent_revision = -1
            interval = MESH_INTERVAL_S
            try:
                while not stop_event.is_set():
                    try:
                        await asyncio.wait_for(mesh_wanted.wait(), timeout=interval)
                        forced = True
                    except asyncio.TimeoutError:
                        forced = False
                    mesh_wanted.clear()

                    slam = curr_state.slam
                    if not forced and (not curr_state.is_streaming
                                       or slam.tsdf_revision == sent_revision):
                        continue
                    sent_revision, elapsed = await send_mesh()

                    # Extraction holds the TSDF lock, so while it runs the frame
                    # loop cannot integrate. Back off proportionally to keep it
                    # under ~10% of the time: at a fine voxel a single extraction
                    # can take half a second, and a fixed interval would then
                    # spend a quarter of the session blocking the scanner.
                    interval = max(MESH_INTERVAL_S, elapsed * MESH_DUTY_DIVISOR)
            except (WebSocketDisconnect, asyncio.CancelledError):
                pass
            except Exception as e:
                logger.error("Error in mesh worker: %s", e)

        async def control_reader():
            try:
                while not stop_event.is_set():
                    raw_data = await websocket.receive_text()
                    try:
                        msg = json.loads(raw_data)
                    except Exception:
                        logger.warning("Invalid JSON received over WS: %s", raw_data)
                        continue

                    cmd = msg.get("cmd")
                    logger.info("Received WS command: %s", cmd)

                    if cmd == "start":
                        curr_state.is_streaming = True
                        await websocket.send_text(json.dumps({
                            "type": "control_ack",
                            "cmd": "start",
                            "status": "streaming"
                        }))
                        # Resync a (re)connecting client with the whole map;
                        # everything after this is an incremental delta.
                        await websocket.send_bytes(curr_state.slam.snapshot_buffer())
                    elif cmd == "pause":
                        curr_state.is_streaming = False
                        await websocket.send_text(json.dumps({
                            "type": "control_ack",
                            "cmd": "pause",
                            "status": "paused"
                        }))
                    elif cmd == "reset":
                        curr_state.reset_slam()
                        # Send 0 points empty buffer or ack
                        await websocket.send_text(json.dumps({
                            "type": "control_ack",
                            "cmd": "reset",
                            "status": "reset_completed",
                            "total_points": 0
                        }))
                    elif cmd == "tilt":
                        requested = msg.get("angle", 0)
                        applied = curr_state.driver.set_tilt(requested)
                        await websocket.send_text(json.dumps({
                            "type": "control_ack",
                            "cmd": "tilt",
                            "angle": applied,
                            "requested": requested,
                            "success": True
                        }))
                    elif cmd == "mesh":
                        mesh_wanted.set()
                        await websocket.send_text(json.dumps({
                            "type": "control_ack",
                            "cmd": "mesh",
                            "status": "extracting",
                        }))
                    elif cmd == "ping":
                        await websocket.send_text(json.dumps({"type": "pong"}))
            except WebSocketDisconnect:
                logger.info("WebSocket client disconnected in reader")
            except Exception as e:
                logger.error("Error in WS control_reader: %s", e)
            finally:
                stop_event.set()

        reader_task = asyncio.create_task(control_reader())
        mesh_task = asyncio.create_task(mesh_worker())

        # Stream loop running at targeted 15-20 FPS (~0.05s)
        frame_idx = 0
        fps_counter = 0
        fps_timer = time.time()
        prev_tracking_ok = True

        last_seq = -1

        try:
            while not stop_event.is_set():
                if not curr_state.is_streaming:
                    await asyncio.sleep(0.05)
                    continue

                # The sensor runs at 30 Hz; re-running SLAM on a frame we have
                # already consumed only burns CPU and double-weights the TSDF.
                seq = curr_state.driver.frame_seq
                if seq == last_seq:
                    await asyncio.sleep(0.002)
                    continue
                last_seq = seq

                loop_start = time.time()

                # Run frame capture & SLAM processing in threadpool to avoid blocking event loop
                rgb, depth = await asyncio.to_thread(curr_state.driver.get_frame)
                slam_result = await asyncio.to_thread(curr_state.slam.process_frame, rgb, depth)

                curr_state.last_status["tracking_ok"] = slam_result["tracking_ok"]
                curr_state.last_status["pose"] = slam_result["pose"]
                curr_state.last_status["frame_count"] += 1

                fps_counter += 1
                now = time.time()
                elapsed = now - fps_timer
                if elapsed >= 1.0:
                    curr_state.last_status["fps"] = fps_counter / elapsed
                    fps_counter = 0
                    fps_timer = now

                # 1. Send the incremental point delta (8-byte header + 16 B/vertex)
                if slam_result["new_points"]:
                    async with send_lock:
                        await websocket.send_bytes(slam_result["binary_buffer"])

                frame_idx += 1

                # 2. Telemetry JSON drives the on-screen frustum pose, so every
                # 3rd frame (~10 Hz, a few hundred bytes) instead of every 10th.
                tracking_changed = (slam_result["tracking_ok"] != prev_tracking_ok)
                if frame_idx % 3 == 0 or tracking_changed:
                    prev_tracking_ok = slam_result["tracking_ok"]
                    telemetry = {
                        "type": "telemetry",
                        "fps": round(curr_state.last_status.get("fps", 0.0), 1),
                        "tracking_ok": slam_result["tracking_ok"],
                        "pose": slam_result["pose"],
                        "point_count": slam_result["point_count"],
                        "frame_idx": frame_idx,
                    }
                    async with send_lock:
                        await websocket.send_text(json.dumps(telemetry))

                # Yield to the event loop; the frame_seq gate above already
                # caps throughput at the sensor's own 30 Hz.
                dt = time.time() - loop_start
                await asyncio.sleep(max(0.001, (1.0 / 30.0) - dt))

        except WebSocketDisconnect:
            logger.info("WebSocket client disconnected in stream loop")
        except Exception as e:
            logger.error("Error in WS stream loop: %s", e)
        finally:
            stop_event.set()
            reader_task.cancel()
            mesh_task.cancel()
            for task in (reader_task, mesh_task):
                try:
                    await task
                except (asyncio.CancelledError, WebSocketDisconnect):
                    pass
            logger.info("WS connection terminated and cleaned up")

    return app


def main():
    parser = argparse.ArgumentParser(description="Kinect v1 3D Room Scanner Backend Server")
    parser.add_argument("--mock", action="store_true", help="Force mock Kinect driver mode")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host address to bind")
    parser.add_argument("--port", type=int, default=8000, help="Port to bind")
    parser.add_argument(
        "--voxel-mm", type=float, default=12.0,
        help="Surface voxel size in mm. Finer means more detail and much more "
             "work in both time and memory: 20mm extracts in 45ms, 12mm in "
             "120ms, 8mm in ~1.1s, 6mm in 540ms. Below about 5mm the Kinect's "
             "own depth quantisation dominates. On a machine with little free "
             "RAM, 8mm and finer can exhaust it.",
    )
    args = parser.parse_args()

    app = create_app(mock=args.mock, tsdf_voxel=args.voxel_mm / 1000.0)
    uvicorn.run(app, host=args.host, port=args.port)


def get_default_app() -> FastAPI:
    return create_app(mock=False)


def __getattr__(name: str):
    """Lazily build `app` so that merely importing this module does not open
    the sensor -- `uvicorn backend.server:app` still works."""
    if name == "app":
        global app
        app = get_default_app()
        return app
    raise AttributeError(name)


if __name__ == "__main__":
    main()
