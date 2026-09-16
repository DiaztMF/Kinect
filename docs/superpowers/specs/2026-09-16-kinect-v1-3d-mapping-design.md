# Kinect v1 (1414) 3D Room Mapping System Specification

**Date:** 2026-09-16  
**Status:** Approved Draft  
**Target Path:** `D:\Project\Web Project\Enuma\Kinect`

---

## 1. Overview & Goal
Membangun web-based 3D environment mapping scanner berbasis Microsoft Kinect v1 (Model 1414 Xbox 360) di Windows 11. Sistem membaca RGB-D stream real-time, mengeksekusi SLAM Odometry (Open3D) untuk menyatukan frame kamar/ruangan secara kontinu saat sensor dibawa berjalan, lalu mentransmisikan point cloud biner via WebSocket ke client Three.js berestetika spatial computing modern (anti-AI slop).

---

## 2. Hardware Constraints & Drivers
- **Hardware:** Kinect v1 Xbox 360 (Model 1414).
- **Power & Ports:** Wajib 12V AC Power Supply eksternal + Port USB 2.0 (menghindari bandwidth drop USB 3.0).
- **Sensors:** IR projector + CMOS sensor (Depth 640x480 11-bit), Color camera (RGB 640x480), Tilt motor.
- **Range:** Efektif 0.8m - 3.5m. Blind spot < 0.8m.
- **Host OS & Driver:** Windows 11 x64 dengan Kinect for Windows SDK v1.8 (Full SDK diperlukan agar model Xbox 360 1414 dikenali OS).

---

## 3. Architecture & Data Flow

```
[Kinect 1414 (USB 2.0)]
       │
       ▼
[Python Backend: Kinect Capture Worker]
       │ (Color 640x480 + Depth 640x480 @ 30 FPS)
       ▼
[Open3D SLAM / Odometry Engine]
       ├─ Voxel Downsampling (20mm grid)
       ├─ Depth Truncation [0.8m, 3.5m]
       ├─ Hybrid RGB-D Odometry (Pose tracking T_world)
       └─ Global Point Cloud Accumulator
       │
       ▼ (Interleaved Binary ArrayBuffer: Float32 XYZ + Uint8 RGBA)
[FastAPI WebSocket Server (`ws://localhost:8000/ws/scan`)]
       │
       ▼
[Frontend: Vite + React / Three.js + Tailwind v4]
       ├─ WebSocket Binary Stream Parser (Zero-copy ArrayBuffer)
       ├─ BufferGeometry direct memory swap
       ├─ PointsMaterial rendering + Camera Frustum Wireframe
       └─ Spatial HUD Controls (Start, Pause, Reset, Export PLY)
```

---

## 4. Backend Implementation Specs
- **Runtime:** Python 3.10 / 3.11.
- **Core Libraries:** `open3d`, `fastapi`, `uvicorn`, `websockets`, `numpy`, `pykinect` (atau direct C-types wrapping ke `Kinect10.dll` SDK 1.8).
- **SLAM Pipeline:**
  - `open3d.geometry.RGBDImage.create_from_color_and_depth`
  - Intrinsik: `fx=525.0, fy=525.0, cx=320.0, cy=240.0`
  - Algoritma: `open3d.pipelines.odometry.compute_rgbd_odometry` (Point-to-Plane + Photometric error).
  - Safety mechanism: Deteksi tracking loss (bila fitness < 0.35 atau inlier RMSE > 0.05). Emit event JSON `tracking_lost` dan pertahankan pose terakhir.
- **Binary Wire Protocol:**
  - Header: 4 bytes `Uint32` merepresentasikan jumlah total points $N$.
  - Body: $N \times 16$ bytes:
    - 12 bytes: `Float32` $[X, Y, Z]$
    - 4 bytes: `Uint8` $[R, G, B, A]$

---

## 5. Frontend & UI/UX Design System (Anti-Slop Guidelines)
- **Aesthetic Direction:** *Dark Spatial Computing / Precision Engineering*. OLED Dark (`#09090b` - zinc-950), Vantablack cards dengan double-bezel nesting (`rounded-2xl`, border hairline `white/10`, inner shadow highlight).
- **Typography:** `Geist Sans` / `Plus Jakarta Sans` + `Geist Mono` untuk metrik telemetry (FPS, Point Count, Drift Error, Latency). Tidak memakai generic Inter/Roboto.
- **Color Discipline:** 1 singular high-contrast accent (Emerald `#10b981` untuk scan active/tracking locked, Crimson `#ef4444` untuk tracking loss). Dilarang memakai gradient ungu/cyan generik AI.
- **Layout:**
  - Floating dynamic Island header (Device status, USB bandwidth check, FPS).
  - Viewport 3D utama canvas Three.js full-screen (`100dvh`).
  - Bottom Floating Control Dock (Start Scan, Freeze, Clear, Export PLY/OBJ) dengan nested button-in-button icon pill pattern.
  - Telemetry HUD widget di pojok kanan atas: Card double-bezel dengan live stat counters.
- **Rendering Pipeline Three.js:**
  - `THREE.BufferGeometry` dengan single allocated buffer (up to 500k points).
  - `THREE.Points` menggunakan dynamic buffer attributes (`drawRange`).
  - Realtime Kinect Frustum cone wireframe yang bergerak sesuai pose matriks 4x4 kamera di ruang 3D.
  - OrbitControls dengan damping halus (`dampingFactor: 0.05`).

---

## 6. Verification & Self-Review
- **Driver readiness check:** Script diagnostik awal untuk mendeteksi ketersediaan `Kinect10.dll` dan device enumeration sebelum menjalankan server scanning.
- **Mock Mode:** Backend menyediakan flag `--mock` yang mensimulasikan point cloud ruangan sintetis (cube/room mesh generator) sehingga frontend dan pipeline 3D bisa diuji bahkan saat Kinect belum dicolok.
- **Export:** Fitur export PLY biner untuk memvalidasi point cloud hasil scanning ke Blender / CloudCompare.
