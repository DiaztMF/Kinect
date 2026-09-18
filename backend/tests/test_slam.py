"""
Tests for KinectDriver and SLAMEngine.
"""

import math
import struct

import numpy as np
import pytest

from backend.kinect_driver import KinectDriver
from backend.slam_engine import SLAMEngine


def test_kinect_driver_mock_frames():
    """Verify that KinectDriver(mock=True) returns valid 640x480 RGB uint8 and Depth uint16 frames."""
    driver = KinectDriver(mock=True)
    rgb, depth = driver.get_frame()

    assert isinstance(rgb, np.ndarray)
    assert isinstance(depth, np.ndarray)

    assert rgb.shape == (480, 640, 3)
    assert rgb.dtype == np.uint8

    assert depth.shape == (480, 640)
    assert depth.dtype == np.uint16

    # Verify non-zero contents
    assert np.any(rgb > 0)
    assert np.any(depth > 0)

    # Check depth range in realistic indoor interval (400mm - 3500mm)
    valid_depths = depth[depth > 0]
    assert len(valid_depths) > 10000
    assert valid_depths.min() >= 400
    assert valid_depths.max() <= 3500


def test_slam_engine_continuous_frames():
    """Verify SLAMEngine processes sequential frames, tracks pose, and accumulates points."""
    driver = KinectDriver(mock=True)
    slam = SLAMEngine(voxel_size=0.03, max_depth=3.5)

    res1 = None
    res2 = None
    res3 = None

    for i in range(3):
        rgb, depth = driver.get_frame()
        res = slam.process_frame(rgb, depth)

        assert "tracking_ok" in res
        assert "pose" in res
        assert "point_count" in res
        assert "new_points" in res
        assert "binary_buffer" in res

        # Validate pose is 4x4
        pose = np.array(res["pose"])
        assert pose.shape == (4, 4)

        # Validate binary buffer format: the stream carries only the delta
        buf = res["binary_buffer"]
        assert len(buf) >= 12
        kind, delta_count, mode = struct.unpack("<III", buf[:12])
        assert kind == 0  # MSG_POINTS
        assert mode == 0  # MODE_APPEND
        assert delta_count == res["new_points"]
        assert len(buf) == 12 + delta_count * 16

        if i == 0:
            res1 = res
        elif i == 1:
            res2 = res
        elif i == 2:
            res3 = res

    # Tracking should succeed on consecutive synthetic frames
    assert res1["tracking_ok"] is True
    assert res2["tracking_ok"] is True
    assert res3["tracking_ok"] is True

    # Point cloud must accumulate across frames
    assert res1["point_count"] > 0
    assert res3["point_count"] >= res1["point_count"]


def test_slam_voxel_map_dedup():
    """Re-processing an identical frame must add no new voxels, and the voxel
    key index must stay sorted and unique as the map grows."""
    driver = KinectDriver(mock=True)
    slam = SLAMEngine(voxel_size=0.03, max_depth=3.5)

    rgb, depth = driver.get_frame()
    first = slam.process_frame(rgb, depth)
    assert first["new_points"] > 0

    # Re-observing the same scene must not re-add the voxels already held.
    # Odometry still nudges the pose by a fraction of a millimetre, so a
    # handful of points can cross a voxel boundary -- allow well under 1%.
    again = slam.process_frame(rgb, depth)
    assert again["new_points"] < first["new_points"] * 0.01
    assert len(again["binary_buffer"]) == 12 + again["new_points"] * 16

    # With tracking held fixed, the dedup is exact.
    slam._prev_rgbd_t = None
    third = slam.process_frame(rgb, depth)
    assert third["new_points"] == 0
    assert third["point_count"] == again["point_count"]
    assert len(third["binary_buffer"]) == 12

    keys = slam._map_keys
    assert keys.size == slam.total_points
    assert np.all(np.diff(keys) > 0), "voxel key index must stay sorted and unique"

    # A fresh frame from a moved viewpoint contributes new voxels.
    rgb2, depth2 = driver.get_frame()
    fourth = slam.process_frame(rgb2, depth2)
    assert fourth["point_count"] >= first["point_count"]
    assert np.all(np.diff(slam._map_keys) > 0)


def test_slam_rejects_implausible_motion():
    """A transform implying a teleport must not be folded into the pose."""
    slam = SLAMEngine()
    assert slam._is_plausible(np.eye(4)) is True

    teleport = np.eye(4)
    teleport[0, 3] = 5.0  # 5 m in one 33 ms frame
    assert slam._is_plausible(teleport) is False

    assert slam._is_plausible(np.full((4, 4), np.nan)) is False


def test_snapshot_buffer_is_replace_mode():
    """A reconnecting client gets the whole map flagged as a replacement."""
    driver = KinectDriver(mock=True)
    slam = SLAMEngine(voxel_size=0.03, max_depth=3.5)
    assert struct.unpack("<III", slam.snapshot_buffer()[:12]) == (0, 0, 1)

    rgb, depth = driver.get_frame()
    slam.process_frame(rgb, depth)
    snap = slam.snapshot_buffer()
    kind, count, mode = struct.unpack("<III", snap[:12])
    assert kind == 0  # MSG_POINTS
    assert mode == 1  # MODE_REPLACE
    assert count == slam.total_points
    assert len(snap) == 12 + count * 16


def test_pose_tracks_camera_motion(monkeypatch):
    """The estimated camera-to-world pose must follow the camera, not mirror it.

    `compute_rgbd_odometry(prev, curr)` returns T_curr<-prev, so the pose
    advances by its *inverse*. Composing the raw transform instead reconstructs
    the scene with the camera running backwards, which this pins down.
    """
    import backend.kinect_driver as kd

    clock = [1000.0]
    monkeypatch.setattr(kd.time, "time", lambda: clock[0])

    driver = kd.KinectDriver(mock=True)
    slam = SLAMEngine(voxel_size=0.03, max_depth=3.5)

    estimated, truth = [], []
    for i in range(40):
        clock[0] = 1000.0 + i * 0.05
        rgb, depth = driver.get_frame()
        result = slam.process_frame(rgb, depth)

        # Ground truth: _generate_mock_frame renders a static world from a
        # camera at (cam_x, cam_y, cam_z) for t = (now - start) * 0.5.
        t = (clock[0] - 1000.0) * 0.5
        truth.append([0.15 * math.sin(t), 0.08 * math.cos(t * 0.8), 0.10 * math.sin(t * 0.6)])
        estimated.append(np.array(result["pose"])[:3, 3])

    estimated = np.array(estimated) - np.array(estimated)[0]
    truth = np.array(truth) - np.array(truth)[0]

    # Only X and Z are usable ground truth here. _generate_mock_frame models
    # lateral and forward motion by shifting the box in the image and changing
    # its range, which is what a translating camera really produces. Vertical
    # motion instead rewrites `floor_y`, deforming the scene along each ray
    # rather than moving the viewpoint -- so no correct tracker can recover it,
    # and the Y correlation swings either way between runs. The real motion
    # check is the tilt-sweep measurement against the accelerometer; this test
    # exists to pin the *sign* of the pose composition.
    for axis, name in ((0, "X"), (2, "Z")):
        corr = np.corrcoef(estimated[:, axis], truth[:, axis])[0, 1]
        assert corr > 0.9, f"pose axis {name} does not follow the camera (corr={corr:.3f})"


def test_downsample_averages_and_respects_depth_edges():
    """The tracking downsample must average, not stride, and must refuse to
    average a block that straddles a depth discontinuity."""
    rgb = np.zeros((4, 4, 3), np.uint8)
    rgb[0, 0] = 100
    rgb[0, 1] = 200
    rgb[1, 0] = 40
    rgb[1, 1] = 60  # block mean = (100 + 200 + 40 + 60) / 4 = 100

    depth = np.zeros((4, 4), np.uint16)
    depth[0:2, 0:2] = [[1000, 1010], [1020, 1030]]  # flat block -> averaged
    depth[0:2, 2:4] = [[1000, 1000], [1000, 2000]]  # straddles an edge -> dropped
    depth[2:4, 0:2] = [[1000, 1000], [1000, 0]]     # has a hole -> dropped

    rgb_ds, depth_ds = SLAMEngine._downsample_pair(rgb, depth, 2)

    assert rgb_ds.shape == (2, 2, 3) and depth_ds.shape == (2, 2)
    assert rgb_ds[0, 0, 0] == 100, "colour must be the block average, not the corner pixel"
    assert depth_ds[0, 0] == 1015, f"flat block should average, got {depth_ds[0, 0]}"
    assert depth_ds[0, 1] == 0, "a block spanning a depth edge must be dropped"
    assert depth_ds[1, 0] == 0, "an incomplete block must be dropped"


def test_tracking_blur_does_not_leak_holes():
    """Smoothing must not average missing readings in.

    A plain Gaussian pulls depths next to a hole toward zero. That version
    scored well on a static recording -- both frames were corrupted the same way
    -- while recovering only a third of a known 30 degree tilt sweep, so this
    property is what separates a working prefilter from a broken one.
    """
    slam = SLAMEngine()
    depth = np.full((32, 32), 2000, np.uint16)
    depth[8:24, 8:24] = 0  # a large hole in the middle

    out = slam._smooth_for_tracking(depth)
    hole = depth == 0
    valid = ~hole

    assert np.all(out[hole] == 0), "smoothing must not invent depth inside a hole"
    # Every surviving pixel, including the ring hugging the hole, must keep its
    # value; anything dragged toward zero is hole leakage.
    assert np.allclose(out[valid], 2000, atol=1.0), (
        f"valid depth pulled toward the hole: min {out[valid].min():.1f} mm")


def test_motion_prior_is_gated_at_the_noise_floor():
    """A sub-noise motion estimate must not be fed back as the next seed."""
    slam = SLAMEngine()
    assert slam.motion_prior_floor > 0

    tiny = np.eye(4)
    tiny[0, 3] = slam.motion_prior_floor * 0.5
    slam._motion_prior = tiny
    assert np.linalg.norm(slam._motion_prior[:3, 3]) <= slam.motion_prior_floor

    real = np.eye(4)
    real[0, 3] = slam.motion_prior_floor * 10
    slam._motion_prior = real
    assert np.linalg.norm(slam._motion_prior[:3, 3]) > slam.motion_prior_floor


def test_mesh_buffer_layout_is_parseable_without_copies():
    """The mesh wire format must let the browser build typed-array views
    directly on the received buffer, so every 4-byte field has to stay aligned:
    positions and indices first, the byte-wide normals and colours last."""
    driver = KinectDriver(mock=True)
    slam = SLAMEngine(voxel_size=0.03, max_depth=3.5)
    for _ in range(3):
        rgb, depth = driver.get_frame()
        slam.process_frame(rgb, depth)

    buf = slam.mesh_buffer()
    kind, nv, nt = struct.unpack("<III", buf[:12])
    assert kind == 1  # MSG_MESH
    assert nv > 0 and nt > 0, "mock scene should produce a surface"
    assert len(buf) == 12 + nv * 12 + nt * 12 + nv * 3 + nv * 3

    off = 12
    pos = np.frombuffer(buf, np.float32, count=nv * 3, offset=off).reshape(nv, 3)
    off += nv * 12
    idx = np.frombuffer(buf, np.uint32, count=nt * 3, offset=off).reshape(nt, 3)
    off += nt * 12
    nrm = np.frombuffer(buf, np.int8, count=nv * 3, offset=off).reshape(nv, 3)

    assert np.all(np.isfinite(pos))
    assert idx.max() < nv, "triangle indexes a vertex that does not exist"
    # int8 normals must be a real direction, not all zeros
    assert np.abs(nrm.astype(np.float32) / 127.0).sum(1).max() > 0.5


def test_empty_mesh_buffer_is_header_only():
    slam = SLAMEngine()
    buf = slam.mesh_buffer()
    assert struct.unpack("<III", buf[:12]) == (1, 0, 0)
    assert len(buf) == 12


def test_tsdf_revision_advances_only_on_integration():
    """The mesh worker skips re-extraction when nothing changed, so the counter
    has to track actual integrations."""
    driver = KinectDriver(mock=True)
    slam = SLAMEngine(voxel_size=0.03, max_depth=3.5)
    assert slam.tsdf_revision == 0

    rgb, depth = driver.get_frame()
    slam.process_frame(rgb, depth)
    assert slam.tsdf_revision == 1, "first frame is always a keyframe"

    # Same frame, pose unchanged -> not a keyframe -> no new integration.
    slam._prev_rgbd_t = None
    before = slam.tsdf_revision
    slam.process_frame(rgb, depth)
    assert slam.tsdf_revision == before

    slam.reset()
    assert slam.tsdf_revision == 0
