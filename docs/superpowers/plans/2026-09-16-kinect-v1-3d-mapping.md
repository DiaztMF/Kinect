# Kinect v1 (1414) 3D Room Mapping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a complete real-time 3D room scanning system using Kinect v1 (1414) on Windows 11 with Open3D RGB-D Odometry and an anti-slop Three.js web visualizer.

**Architecture:** A Python backend handles Kinect video/depth frame capture (or mock simulation fallback), executes real-time Open3D RGB-D SLAM odometry and point cloud stitching, and broadcasts raw interleaved binary buffers (XYZ Float32 + RGBA Uint8) over WebSocket. A Vite + Three.js frontend renders the live point cloud and trajectory frustum in a precision spatial computing dark UI.

**Tech Stack:** Python 3.10/3.11, Open3D, FastAPI, Uvicorn, WebSockets, NumPy, Vite, React, Three.js, Tailwind v4, `@phosphor-icons/react`.

## Global Constraints
- Target workspace directory: `D:\Project\Web Project\Enuma\Kinect`.
- Backend in `backend/`, frontend in `frontend/`.
- Wire protocol: Interleaved binary ArrayBuffer (4 bytes Uint32 point count header, followed by 16 bytes per vertex: 3x Float32 XYZ + 4x Uint8 RGBA).
- Design system: Dark spatial computing aesthetic, double-bezel nesting, font Geist/Plus Jakarta Sans, no AI purple/cyan slop gradients.
- Device: Xbox 360 Kinect v1 (Model 1414). Requires AC 12V power + USB 2.0. Must handle live hardware connection and include `--mock` mode.

---

### Task 1: Hardware Driver Diagnostics & Setup Guide

**Files:**
- Create: `D:\Project\Web Project\Enuma\Kinect\backend\check_kinect.py`

**Interfaces:**
- Consumes: Windows PnP / USB device info, `Kinect10.dll` check.
- Produces: JSON/CLI summary of Kinect USB subdevices (Camera PID `02AE`, Audio PID `02AD`, Motor PID `02B0`) and driver status.

- [ ] **Step 1: Write driver diagnostic tool**
Detects whether Kinect Camera, Motor, and Audio are detected and checks whether `Kinect10.dll` exists in `System32` or SDK path.

- [ ] **Step 2: Run diagnostic script**
Execute: `python backend/check_kinect.py`
Verify hardware enumeration status.

- [ ] **Step 3: Provide download & driver install automation if missing**
Instructions/direct link for `KinectSDK-v1.8-Setup.exe`.

---

### Task 2: Python Backend - Frame Capture & Open3D SLAM Pipeline

**Files:**
- Create: `D:\Project\Web Project\Enuma\Kinect\backend\requirements.txt`
- Create: `D:\Project\Web Project\Enuma\Kinect\backend\kinect_driver.py`
- Create: `D:\Project\Web Project\Enuma\Kinect\backend\slam_engine.py`
- Create: `D:\Project\Web Project\Enuma\Kinect\backend\tests\test_slam.py`

**Interfaces:**
- Consumes: Raw RGB (640x480) & Depth (640x480 uint16) frames.
- Produces: `SLAMResult` with `current_pose` (4x4 matrix), `is_tracking_ok` (bool), and `global_pcd_buffer` (bytes).

- [ ] **Step 1: Write test for SLAM engine using synthetic RGB-D frames**
Test `compute_frame(color, depth)` produces valid transformation and point cloud downsampled output.

- [ ] **Step 2: Implement SLAM engine in `backend/slam_engine.py`**
Use `open3d.pipelines.odometry.compute_rgbd_odometry` with hybrid error metric and voxel grid downsampling.

- [ ] **Step 3: Implement Kinect Driver wrapper in `backend/kinect_driver.py`**
Access Kinect via PyKinect / Ctypes wrapper or fallback to synthetic room scanner if camera stream not active.

- [ ] **Step 4: Run tests to verify SLAM odometry accumulation**
Run: `pytest backend/tests/test_slam.py`

---

### Task 3: Python Backend - FastAPI WebSocket Server

**Files:**
- Create: `D:\Project\Web Project\Enuma\Kinect\backend\server.py`
- Create: `D:\Project\Web Project\Enuma\Kinect\backend\tests\test_server.py`

**Interfaces:**
- Consumes: `slam_engine.py` point cloud data.
- Produces: WebSocket server on `ws://localhost:8000/ws/scan` streaming 16-byte interleaved binary buffers.

- [x] **Step 1: Write unit test for binary buffer packing**
Assert header matches point count and byte length equals $4 + N \times 16$.

- [x] **Step 2: Implement FastAPI app and WebSocket loop in `backend/server.py`**
Stream at 15-20 FPS with control handlers (`start`, `pause`, `reset`, `export`).

- [x] **Step 3: Verify WebSocket streaming**
Run backend with `python backend/server.py --mock` and check client connection.

---

### Task 4: Frontend - Scaffolding Vite + React + Three.js

**Files:**
- Create: `D:\Project\Web Project\Enuma\Kinect\frontend` (Vite React TS template)
- Create: `D:\Project\Web Project\Enuma\Kinect\frontend\src\styles\globals.css` (Tailwind v4 tokens)

**Interfaces:**
- Sets up modern spatial design environment: OLED black (`#09090b`), double-bezel cards, Geist/Plus Jakarta Sans.

- [ ] **Step 1: Initialize Vite React TypeScript project in `frontend/`**
- [ ] **Step 2: Install dependencies (`three`, `@types/three`, `@phosphor-icons/react`, `clsx`, `tailwind`)**
- [ ] **Step 3: Configure Tailwind and custom font stack**

---

### Task 5: Frontend - Three.js Point Cloud & Frustum Visualizer

**Files:**
- Create: `D:\Project\Web Project\Enuma\Kinect\frontend\src\components\Viewport3D.tsx`
- Create: `D:\Project\Web Project\Enuma\Kinect\frontend\src\hooks\usePointBuffer.ts`

**Interfaces:**
- Consumes: Binary `ArrayBuffer` from WebSocket.
- Produces: Hardware-accelerated `THREE.Points` with dynamic `Float32Array` positions and `Uint8Array` colors + Kinect camera wireframe pyramid.

- [ ] **Step 1: Implement `usePointBuffer` binary parser hook**
Zero-copy parser reading `Float32Array` coordinates and `Uint8Array` normalized colors.

- [ ] **Step 2: Implement Three.js viewport canvas**
Scene, OrbitControls with smooth damping, grid floor, lighting, dynamic `BufferGeometry`, and frustum gizmo.

---

### Task 6: Frontend - Spatial HUD UI & Control Dock (Anti-Slop)

**Files:**
- Create: `D:\Project\Web Project\Enuma\Kinect\frontend\src\components\SpatialHUD.tsx`
- Create: `D:\Project\Web Project\Enuma\Kinect\frontend\src\components\ControlDock.tsx`
- Modify: `D:\Project\Web Project\Enuma\Kinect\frontend\src\App.tsx`

**Interfaces:**
- Precision double-bezel telemetry HUD (FPS, points count, tracking status, sensor connection).
- Floating pill control dock with button-in-button trailing icons (Scan, Freeze, Reset, Export PLY).

- [ ] **Step 1: Implement double-bezel Telemetry HUD widget**
- [ ] **Step 2: Implement Control Dock with tactile feedback and status indicators**
- [ ] **Step 3: Wire controls to WebSocket command channel**

---

### Task 7: End-to-End Live Testing & Verification

**Files:**
- Verify: Full pipeline running live with Kinect hardware and browser.

- [ ] **Step 1: Start Backend server (`uvicorn server:app`)**
- [ ] **Step 2: Start Frontend dev server (`npm run dev`)**
- [ ] **Step 3: Verify live point cloud stream in browser with real Kinect v1**
- [ ] **Step 4: Check DevTools console for zero error clean state**
