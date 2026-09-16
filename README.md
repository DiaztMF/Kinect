# Enuma Kinect 3D Spatial Mapper

Real-time 3D room mapping and spatial surface scanner using Microsoft Kinect v1 (Model 1414 Xbox 360) with Open3D RGB-D SLAM odometry and an interactive Three.js web visualizer.

[![License: MIT](https://img.shields.io/badge/License-MIT-emerald.svg)](LICENSE)
[![Python: 3.11](https://img.shields.io/badge/Python-3.11-blue.svg)](https://www.python.org/)
[![React: 19](https://img.shields.io/badge/React-19-cyan.svg)](https://react.dev/)
[![Three.js: WebGL2](https://img.shields.io/badge/Three.js-r174-black.svg)](https://threejs.org/)
[![Open3D: 0.19](https://img.shields.io/badge/Open3D-0.19.0-orange.svg)](http://www.open3d.org/)

---

## Installation

### Prerequisites
- Windows 10 / 11 64-bit
- Python 3.11 (with `uv` or `pip`)
- Node.js 18+ & npm
- Hardware: Microsoft Kinect v1 (Xbox 360 Model 1414) with external 12V AC power adapter and USB 2.0 connection.

### 1. Hardware Driver Setup
To enable hardware access on Windows 11 without driver signature blocks:
1. Plug Kinect into an external 12V power supply (green LED blinks or stays solid).
2. Open Zadig (`backend/zadig-2.9.exe`).
3. Click **Options** -> **List All Devices**.
4. Select **Xbox NUI Camera** (`VID: 045E`, `PID: 02AE`).
5. Choose target driver **WinUSB (v6.1.7600.16385)** and click **Replace Driver**.

### 2. Backend Installation
```powershell
cd backend
# Create Python 3.11 virtual environment
uv venv .venv --python 3.11
# Install dependencies
uv pip install -r requirements.txt --python .venv\Scripts\python.exe
```

### 3. Frontend Installation
```powershell
cd frontend
npm install
```

---

## Quick Start

### 1. Start Backend Server
```powershell
cd backend
.venv\Scripts\python.exe -m uvicorn server:app --host 127.0.0.1 --port 8000
```
*Note: If no Kinect is plugged in, the backend gracefully falls back to synthetic mock room streaming.*

### 2. Start Frontend Web Visualizer
```powershell
cd frontend
npm run dev
```

Open your browser at:
```
http://localhost:5173
```

---

## What

Enuma Kinect is a local web application that converts a standard Microsoft Kinect v1 into a live spatial 3D scanner. It captures synchronized RGB (640x480) and 11-bit depth streams, runs frame-to-frame Open3D RGB-D Odometry to track the sensor's 6-DoF trajectory in space, continuously stitches geometry into a global point cloud, and streams binary buffers over WebSocket to an in-browser Three.js viewport.

---

## Why

Dedicated 3D LiDAR scanners and modern depth cameras (like Intel RealSense or Azure Kinect) can cost anywhere from $400 to $3,000. Millions of Xbox 360 Kinect v1 units sit unused in storage. By leveraging native userspace USB drivers (`libfreenect`) and Open3D's hybrid geometric-photometric odometry, this project revitalizes accessible hardware for indoor spatial mapping, architecture walk-throughs, robotics SLAM, and creative computing.

---

## API & Wire Protocol

### REST Endpoints
- `GET /api/status`: Returns current driver state (`mock` vs `is_real_initialized`), SLAM state, 4x4 camera pose, point count, and streaming FPS.
- `POST /api/export`: Serializes the current accumulated point cloud into a `.ply` file in `backend/exports/`.
- `POST /api/export_mesh`: Extracts a continuous polygonal triangle mesh from the TSDF volume via marching cubes and saves it as a `.ply` mesh.

### WebSocket Protocol (`ws://localhost:8000/ws/scan`)

#### Client to Server Commands (JSON text):
```json
{ "cmd": "start" }              // Start / resume frame capture and SLAM
{ "cmd": "pause" }              // Pause capture
{ "cmd": "reset" }              // Clear accumulated map and reset camera pose
{ "cmd": "tilt", "angle": 0 }   // Nudge or reset Kinect motor tilt [-27° to +27°]
```

#### Server to Client Streams:
1. **Binary Point Cloud Frames:**
   - 4 bytes `Uint32`: Total points count $N$.
   - $N \times 16$ bytes: Interleaved vertex payload:
     - 12 bytes: `Float32` $[X, Y, Z]$ world coordinates.
     - 4 bytes: `Uint8` $[R, G, B, A]$ vertex colors.
2. **JSON Telemetry Events (Every 10 frames or on status change):**
```json
{
  "type": "telemetry",
  "fps": 18.5,
  "tracking_ok": true,
  "pose": [[...], [...], [...], [...]],
  "point_count": 48210,
  "frame_idx": 120
}
```

---

## Examples & Controls

- **Orbit Controls:** Left-click drag to rotate scene view, right-click drag to pan, scroll wheel to zoom.
- **Start / Pause:** Start continuous SLAM frame stitching or freeze the view for inspection.
- **Reset SLAM:** Clears current point cloud buffer and reinitializes odometry origin to the current camera pose.
- **Tilt Motor Adjustment:** Use `[-5° / +5°]` buttons to physically adjust Kinect sensor angle, or click `0°` to snap back to horizontal center.
- **Density / Splatting Mode:** Toggle between `FINE` (sparse LiDAR inspection), `DENSE` (standard coverage), and `SOLID` (Gaussian point splatting surface overlap).
- **Export Formats:**
  - `Points (.ply)`: Raw color point cloud for CloudCompare, MeshLab, or Gaussian Splatting tools.
  - `Mesh (.ply)`: Watertight polygon surface mesh generated by Open3D TSDF Volume marching cubes.

---

## Architecture

```
[Kinect v1 (Model 1414)]
       │
       ▼ (USB 2.0 Isochronous Stream)
[libfreenect.dll + WinUSB]
       │ (Color 640x480 + Registered Depth in mm)
       ▼
[Python Backend: kinect_driver.py]
       │
       ▼
[Open3D SLAM Core: slam_engine.py]
       ├─ Hybrid RGB-D Odometry (Pose tracking T_world)
       ├─ Voxel Downsampling (1.2cm grid)
       ├─ Global Point Cloud Accumulator
       └─ Scalable TSDF Volume (3D surface reconstruction)
       │
       ▼ (WebSocket Interleaved Binary ArrayBuffer)
[FastAPI Server: server.py]
       │
       ▼
[Three.js Frontend: Viewport3D.tsx + SpatialHUD.tsx]
       ├─ Zero-copy BufferAttribute Swap
       ├─ Circular Disc Point Splatting Material
       └─ Real-time Camera Frustum Gizmo
```

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
