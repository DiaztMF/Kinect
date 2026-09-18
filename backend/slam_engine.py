"""
Open3D RGB-D SLAM pipeline for Kinect v1 (registered depth, 640x480 @ 30 Hz).

Per frame:
  1. Frame-to-frame hybrid (photometric + geometric) odometry via the Open3D
     *tensor* API, on an anti-aliased half-resolution pair whose depth has been
     smoothed hole-aware, seeded with a motion prior that is dropped when the
     last estimate sat at the noise floor. Each of those three is there because
     removing it measurably increases drift on a stationary sensor.
  2. Vectorised numpy unprojection of the (strided) depth map into camera space.
  3. Voxel-hash accumulation into a global map that only ever grows by the
     voxels it has not seen before, so per-frame cost is O(new points) rather
     than O(map size).
  4. Keyframe-gated TSDF integration, which is what the viewer's shaded surface
     and the mesh export are both built from.
  5. Serialisation of *only the new points* into a packed binary vertex buffer,
     plus a whole-surface mesh message the server pushes on its own timer.

Tracking stays frame-to-frame deliberately: Open3D's frame-to-model tracker was
measured on the same two ground-truth recordings at 6.5 FPS with *more* drift
than this pipeline, so the architectural upgrade does not pay off on CPU.
"""

import struct
import threading
from typing import Any, Dict, List, Optional

import numpy as np
import open3d as o3d
import open3d.core as o3c

# Binary stream message kinds. Every frame on the socket starts with this tag.
MSG_POINTS = 0
MSG_MESH = 1

# Point-message modes (see _pack_binary_buffer).
MODE_APPEND = 0
MODE_REPLACE = 1

# Voxel coordinates are packed into one int64 as three 21-bit fields, biased by
# 2^20. Anything outside that box would alias onto an unrelated voxel.
_KEY_BIAS = 1 << 20
_KEY_LIMIT = (1 << 21) - 1


class SLAMEngine:
    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fx: float = 525.0,
        fy: float = 525.0,
        cx: float = 320.0,
        cy: float = 240.0,
        voxel_size: float = 0.02,
        max_depth: float = 3.5,
        min_depth: float = 0.4,
        stride: int = 2,
        odometry_scale: int = 2,
        keyframe_dist: float = 0.04,
        keyframe_angle_deg: float = 4.0,
        max_frame_translation: float = 0.30,
        max_frame_rotation_deg: float = 30.0,
        motion_prior_floor: float = 0.004,
        track_blur_kernel: int = 7,
        track_blur_sigma: float = 2.0,
        tsdf_voxel: float = 0.012,
        tsdf_trunc: float = 0.04,
    ):
        self.width = width
        self.height = height
        self.voxel_size = voxel_size
        self.max_depth = max_depth
        self.min_depth = min_depth
        self.stride = max(1, int(stride))
        self.odometry_scale = max(1, int(odometry_scale))
        self.keyframe_dist = keyframe_dist
        self.keyframe_cos = float(np.cos(np.radians(keyframe_angle_deg)))
        self.max_frame_translation = max_frame_translation
        self.max_frame_rotation_cos = float(np.cos(np.radians(max_frame_rotation_deg)))
        # Below this per-frame translation the motion estimate is mostly sensor
        # noise, so it is not worth seeding the next solve with.
        self.motion_prior_floor = motion_prior_floor
        # Depth prefilter applied to the tracking copy only; see
        # _smooth_for_tracking. Set the kernel to 0 to disable.
        self.track_blur_kernel = track_blur_kernel
        self.track_blur_sigma = track_blur_sigma
        # Surface resolution, and the single biggest dial on this pipeline.
        # Kinect depth quantises to ~3 mm at 1 m and ~26 mm at 3 m, so below
        # roughly 5 mm the extra voxels resolve noise rather than geometry.
        # Cost climbs steeply in both time and memory: for the same scene,
        # 20 mm extracts in 45 ms / 23k verts, 12 mm in 120 ms / 73k, 8 mm in
        # ~1.1 s / 345k, 6 mm in 540 ms / 386k. The 8 mm default this started
        # with got the dev server OOM-killed on an 8 GB machine, so the default
        # is the one that leaves headroom; raise it deliberately.
        self.tsdf_voxel = tsdf_voxel
        self.tsdf_trunc = tsdf_trunc

        # Legacy intrinsics, used by TSDF integration and PLY export.
        self.intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=width, height=height, fx=fx, fy=fy, cx=cx, cy=cy
        )

        # Tensor intrinsics for odometry, scaled to the tracking resolution.
        s = self.odometry_scale
        self._intrinsic_t = o3c.Tensor(
            [[fx / s, 0.0, cx / s], [0.0, fy / s, cy / s], [0.0, 0.0, 1.0]],
            o3c.float64,
        )
        # Coarse-to-fine iteration budget (3 levels, Open3D's default shape).
        self._criteria = [
            o3d.t.pipelines.odometry.OdometryConvergenceCriteria(n) for n in (10, 5, 3)
        ]

        # Precomputed normalised pixel rays for the strided unprojection.
        vv, uu = np.mgrid[0:height:self.stride, 0:width:self.stride]
        self._ray_x = (uu.astype(np.float32) - cx) / fx
        self._ray_y = (vv.astype(np.float32) - cy) / fy

        self.current_pose = np.eye(4, dtype=np.float64)
        self._prev_rgbd_t: Optional[o3d.t.geometry.RGBDImage] = None
        # Constant-velocity prior: last accepted source->target transform.
        self._motion_prior = np.eye(4, dtype=np.float64)
        self._last_kf_pose = np.eye(4, dtype=np.float64)
        self._frame_counter = 0
        # Bumped on every TSDF integration, so the mesh worker can skip
        # re-extracting a surface nothing has changed.
        self.tsdf_revision = 0

        # Global map: sorted voxel keys plus parallel chunk lists of xyz / rgb.
        self._map_keys = np.empty(0, dtype=np.int64)
        self._map_xyz: List[np.ndarray] = []
        self._map_rgb: List[np.ndarray] = []
        self._map_count = 0

        # extract_triangle_mesh() reads the volume while integrate() writes it.
        # The mesh worker runs on its own thread, so without this the two race
        # inside Open3D's C++ and can hand back a corrupt surface or crash.
        self._tsdf_lock = threading.Lock()
        self.tsdf_volume = self._new_tsdf()

    # ------------------------------------------------------------------ setup

    def _new_tsdf(self) -> o3d.pipelines.integration.ScalableTSDFVolume:
        return o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=self.tsdf_voxel,
            sdf_trunc=self.tsdf_trunc,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
        )

    def reset(self):
        """Resets tracking state and clears the global map."""
        self.current_pose = np.eye(4, dtype=np.float64)
        self._prev_rgbd_t = None
        self._motion_prior = np.eye(4, dtype=np.float64)
        self._last_kf_pose = np.eye(4, dtype=np.float64)
        self._frame_counter = 0
        self.tsdf_revision = 0
        self._map_keys = np.empty(0, dtype=np.int64)
        self._map_xyz = []
        self._map_rgb = []
        self._map_count = 0
        with self._tsdf_lock:
            self.tsdf_volume = self._new_tsdf()

    # ------------------------------------------------------------------ views

    @property
    def total_points(self) -> int:
        """Point count without materialising the cloud."""
        return self._map_count

    @property
    def global_pcd(self) -> o3d.geometry.PointCloud:
        """Materialises the accumulated map as an Open3D cloud (export path)."""
        pcd = o3d.geometry.PointCloud()
        if self._map_count:
            xyz = np.concatenate(self._map_xyz, axis=0)
            rgb = np.concatenate(self._map_rgb, axis=0)
            pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
            pcd.colors = o3d.utility.Vector3dVector(rgb.astype(np.float64) / 255.0)
        return pcd

    def extract_mesh(self) -> o3d.geometry.TriangleMesh:
        """Marching-cubes surface extraction from the TSDF volume."""
        with self._tsdf_lock:
            mesh = self.tsdf_volume.extract_triangle_mesh()
        mesh.compute_vertex_normals()
        return mesh

    def snapshot_buffer(self) -> bytes:
        """Full-map binary buffer, for a client that has just (re)connected."""
        if not self._map_count:
            return struct.pack("<III", MSG_POINTS, 0, MODE_REPLACE)
        return self._pack_binary_buffer(
            np.concatenate(self._map_xyz, axis=0),
            np.concatenate(self._map_rgb, axis=0),
            MODE_REPLACE,
        )

    # ------------------------------------------------------------------- core

    def process_frame(self, rgb: np.ndarray, depth: np.ndarray) -> Dict[str, Any]:
        """Processes one frame.

        rgb:   (480, 640, 3) uint8
        depth: (480, 640)    uint16, millimetres, registered to the RGB frame

        Returns tracking_ok, pose (4x4 list), point_count (whole map),
        new_points (this frame's contribution) and binary_buffer (the delta).
        """
        self._frame_counter += 1
        tracking_ok = self._track(rgb, depth)

        world, col = self._unproject(rgb, depth)
        new_xyz, new_rgb = self._accumulate(world, col)

        if self._is_keyframe():
            self._integrate_tsdf(rgb, depth)

        return {
            "tracking_ok": tracking_ok,
            "pose": self.current_pose.tolist(),
            "point_count": self._map_count,
            "new_points": len(new_xyz),
            "binary_buffer": self._pack_binary_buffer(new_xyz, new_rgb, MODE_APPEND),
        }

    @staticmethod
    def _downsample_pair(rgb: np.ndarray, depth: np.ndarray, s: int):
        """Anti-aliased s-by-s reduction of a colour/depth pair.

        Plain striding (`a[::s, ::s]`) aliases: it samples one pixel per block
        and throws the rest away, which corrupts the image gradients the
        photometric term is built from. Measured on a static sensor, striding
        drifts ~1.6x further than an averaged reduction.
        """
        if s == 1:
            return np.ascontiguousarray(rgb), np.ascontiguousarray(depth)

        h, w = depth.shape[0] // s * s, depth.shape[1] // s * s
        n = s * s

        # Sum the s*s sub-sampled planes in integer arithmetic. Reshaping the
        # full frame to float32 and reducing over strided axes instead costs
        # ~47 ms/frame here -- more than the odometry solve it feeds.
        cells = [rgb[i:h:s, j:w:s] for i in range(s) for j in range(s)]
        acc = np.zeros(cells[0].shape, dtype=np.uint16)
        for c in cells:
            acc += c
        rgb_ds = np.ascontiguousarray((acc // n).astype(np.uint8))

        dcells = [depth[i:h:s, j:w:s] for i in range(s) for j in range(s)]
        total = np.zeros(dcells[0].shape, dtype=np.uint32)
        count = np.zeros(dcells[0].shape, dtype=np.uint8)
        for c in dcells:
            total += c
            count += (c > 0).view(np.uint8)
        hi = np.maximum.reduce(dcells)
        lo = np.minimum.reduce([np.where(c > 0, c, np.uint16(65535)) for c in dcells])
        # Averaging depth across an edge invents a surface that is not there,
        # so a block is only kept when it is complete and locally flat.
        keep = (count == n) & ((hi - lo) < 50)
        depth_ds = np.ascontiguousarray(np.where(keep, total // n, 0).astype(np.uint16))
        return rgb_ds, depth_ds

    def _smooth_for_tracking(self, depth_ds: np.ndarray) -> np.ndarray:
        """Hole-aware Gaussian, applied to the tracking copy of the depth map.

        Kinect depth is quantised into steps that widen with range, and on a
        slanted surface those steps form an oriented staircase. Normals
        estimated from it are systematically tilted, which makes the geometric
        residual slide the same way every frame -- measured as a standing 2.8 cm
        drift with a bolted-down sensor.

        The smoothing has to be hole-aware. A third to two thirds of a Kinect
        depth frame has no reading at all, and a plain Gaussian averages those
        zeros in, dragging real depths toward the camera. That version scored
        beautifully on a static recording (both frames corrupted identically, so
        they still matched) while recovering only a third of a known 30 degree
        tilt sweep. Normalising by the blurred validity mask fixes it.

        Mapping keeps the raw depth; only the tracker sees this.
        """
        k, sigma = self.track_blur_kernel, self.track_blur_sigma
        if not k:
            return depth_ds

        def blur(a):
            out = o3d.t.geometry.Image(o3c.Tensor(a)).filter_gaussian(k, sigma)
            out = out.as_tensor().numpy()
            return out[..., 0] if out.ndim == 3 else out

        d = depth_ds.astype(np.float32)
        valid = (depth_ds > 0).astype(np.float32)
        den = blur(valid)
        num = blur(d * valid)
        smoothed = np.where(den > 1e-3, num / np.maximum(den, 1e-6), 0.0)
        # Never invent depth where the sensor returned none.
        return np.ascontiguousarray(np.where(valid > 0, smoothed, 0.0).astype(np.float32))

    def _track(self, rgb: np.ndarray, depth: np.ndarray) -> bool:
        """Frame-to-frame odometry; updates current_pose. Returns tracking_ok."""
        rgb_t, depth_t = self._downsample_pair(rgb, depth, self.odometry_scale)
        curr = o3d.t.geometry.RGBDImage(
            o3d.t.geometry.Image(o3c.Tensor(rgb_t)),
            o3d.t.geometry.Image(o3c.Tensor(self._smooth_for_tracking(depth_t))),
        )

        tracking_ok = True
        if self._prev_rgbd_t is not None:
            # Seeding the solver with the previous frame's motion helps during a
            # fast sweep, but when the camera is nearly still that estimate is
            # mostly noise, and feeding it back doubles the standing drift.
            # Carry it only once it is clearly above the noise floor.
            prior = self._motion_prior
            if np.linalg.norm(prior[:3, 3]) <= self.motion_prior_floor:
                prior = np.eye(4, dtype=np.float64)

            try:
                result = o3d.t.pipelines.odometry.rgbd_odometry_multi_scale(
                    self._prev_rgbd_t,
                    curr,
                    self._intrinsic_t,
                    o3c.Tensor(prior),
                    1000.0,
                    self.max_depth,
                    self._criteria,
                    o3d.t.pipelines.odometry.Method.Hybrid,
                )
                trans = result.transformation.numpy()
            except Exception:
                trans = None

            if trans is not None and self._is_plausible(trans):
                # trans maps prev-camera points into curr-camera coordinates, so
                # the camera-to-world pose advances by its inverse.
                self.current_pose = self.current_pose @ np.linalg.inv(trans)
                self._motion_prior = trans
            else:
                # Lost: hold the pose and drop the stale prior rather than
                # folding a garbage transform into the map.
                tracking_ok = False
                self._motion_prior = np.eye(4, dtype=np.float64)

        self._prev_rgbd_t = curr
        return tracking_ok

    def _is_plausible(self, trans: np.ndarray) -> bool:
        """Rejects transforms implying impossible inter-frame motion."""
        if trans.shape != (4, 4) or not np.all(np.isfinite(trans)):
            return False
        if np.linalg.norm(trans[:3, 3]) > self.max_frame_translation:
            return False
        cos_angle = (np.trace(trans[:3, :3]) - 1.0) * 0.5
        return bool(cos_angle >= self.max_frame_rotation_cos)

    def _unproject(self, rgb: np.ndarray, depth: np.ndarray):
        """Depth map -> world-space float32 points plus uint8 colours."""
        s = self.stride
        z = depth[::s, ::s].astype(np.float32) * 0.001
        valid = (z > self.min_depth) & (z < self.max_depth)
        if not valid.any():
            return np.empty((0, 3), np.float32), np.empty((0, 3), np.uint8)

        zv = z[valid]
        cam = np.empty((zv.size, 3), dtype=np.float32)
        cam[:, 0] = self._ray_x[valid] * zv
        cam[:, 1] = self._ray_y[valid] * zv
        cam[:, 2] = zv

        rot = self.current_pose[:3, :3].astype(np.float32)
        trn = self.current_pose[:3, 3].astype(np.float32)
        return cam @ rot.T + trn, rgb[::s, ::s][valid]

    def _accumulate(self, world: np.ndarray, col: np.ndarray):
        """Inserts unseen voxels into the global map; returns the new points."""
        empty = (np.empty((0, 3), np.float32), np.empty((0, 3), np.uint8))
        if len(world) == 0:
            return empty

        q = np.floor(world / self.voxel_size).astype(np.int64) + _KEY_BIAS
        # A diverged pose can push points outside the packable box; those would
        # alias onto unrelated voxels, so drop them instead of corrupting the map.
        inside = np.all((q >= 0) & (q <= _KEY_LIMIT), axis=1)
        if not inside.all():
            q, world, col = q[inside], world[inside], col[inside]
            if len(q) == 0:
                return empty

        keys = q[:, 0] | (q[:, 1] << 21) | (q[:, 2] << 42)
        uniq, first = np.unique(keys, return_index=True)

        if self._map_keys.size:
            pos = np.searchsorted(self._map_keys, uniq)
            probe = np.minimum(pos, self._map_keys.size - 1)
            fresh = self._map_keys[probe] != uniq
        else:
            pos = np.zeros(uniq.size, dtype=np.intp)
            fresh = np.ones(uniq.size, dtype=bool)

        if not fresh.any():
            return empty

        idx = first[fresh]
        new_xyz = np.ascontiguousarray(world[idx])
        new_rgb = np.ascontiguousarray(col[idx])

        # pos indexes the pre-insert array, which is exactly what np.insert
        # expects, and both sides are sorted, so _map_keys stays sorted.
        self._map_keys = np.insert(self._map_keys, pos[fresh], uniq[fresh])
        self._map_xyz.append(new_xyz)
        self._map_rgb.append(new_rgb)
        self._map_count += len(idx)
        return new_xyz, new_rgb

    def _is_keyframe(self) -> bool:
        """True when the camera moved enough to be worth re-integrating."""
        if self._frame_counter == 1:
            return True
        delta = np.linalg.inv(self._last_kf_pose) @ self.current_pose
        if np.linalg.norm(delta[:3, 3]) > self.keyframe_dist:
            return True
        return bool((np.trace(delta[:3, :3]) - 1.0) * 0.5 < self.keyframe_cos)

    def _integrate_tsdf(self, rgb: np.ndarray, depth: np.ndarray):
        try:
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(np.ascontiguousarray(rgb)),
                o3d.geometry.Image(np.ascontiguousarray(depth)),
                depth_scale=1000.0,
                depth_trunc=self.max_depth,
                convert_rgb_to_intensity=False,
            )
            with self._tsdf_lock:
                self.tsdf_volume.integrate(
                    rgbd, self.intrinsic, np.linalg.inv(self.current_pose)
                )
            self._last_kf_pose = self.current_pose.copy()
            self.tsdf_revision += 1
        except Exception:
            pass

    # ------------------------------------------------------------ wire format

    def mesh_buffer(self) -> bytes:
        """Packs the TSDF surface into the streaming mesh format.

        Header (12 bytes):
          uint32 kind = MSG_MESH
          uint32 vertex_count
          uint32 triangle_count
        Then, in this order so every 4-byte field stays aligned and no padding
        is needed, letting the client build typed-array views straight onto the
        received buffer:
          positions  vertex_count   x 3 x float32
          indices    triangle_count x 3 x uint32
          normals    vertex_count   x 3 x int8   (normalised, for shading)
          colours    vertex_count   x 3 x uint8

        Normals are the reason a mesh reads as a surface rather than a cloud, and
        int8 is ample for shading -- at float32 they would cost as much as the
        positions.
        """
        mesh = self.extract_mesh()
        verts = np.asarray(mesh.vertices, dtype=np.float32)
        tris = np.asarray(mesh.triangles, dtype=np.uint32)
        nv, nt = len(verts), len(tris)
        header = struct.pack("<III", MSG_MESH, nv, nt)
        if nv == 0 or nt == 0:
            return header

        normals = np.asarray(mesh.vertex_normals, dtype=np.float32)
        normals = np.clip(normals * 127.0, -127, 127).astype(np.int8)

        if len(mesh.vertex_colors) == nv:
            colors = (np.asarray(mesh.vertex_colors, dtype=np.float32) * 255.0)
            colors = colors.clip(0, 255).astype(np.uint8)
        else:
            colors = np.full((nv, 3), 200, dtype=np.uint8)

        return b"".join((
            header,
            np.ascontiguousarray(verts).tobytes(),
            np.ascontiguousarray(tris).tobytes(),
            np.ascontiguousarray(normals).tobytes(),
            np.ascontiguousarray(colors).tobytes(),
        ))

    @staticmethod
    def _pack_binary_buffer(xyz: np.ndarray, rgb: np.ndarray, mode: int) -> bytes:
        """Packs points into the streaming vertex format.

        Header (12 bytes):
          uint32 kind = MSG_POINTS
          uint32 point_count
          uint32 mode -- 0 append to the client's map, 1 replace it
        Vertices (16 bytes each):
          3 x float32 XYZ, then 4 x uint8 RGBA (A always 255)
        """
        count = len(xyz)
        header = struct.pack("<III", MSG_POINTS, count, mode)
        if count == 0:
            return header

        dt = np.dtype([("xyz", np.float32, 3), ("rgba", np.uint8, 4)])
        packed = np.empty(count, dtype=dt)
        packed["xyz"] = xyz
        packed["rgba"][:, :3] = rgb
        packed["rgba"][:, 3] = 255
        return header + packed.tobytes()
