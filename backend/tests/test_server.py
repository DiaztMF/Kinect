"""
Unit and Integration Tests for FastAPI REST & WebSocket Server.
"""

import json
import struct
import pytest
from fastapi.testclient import TestClient

from backend.server import create_app


@pytest.fixture
def client():
    app = create_app(mock=True)
    with TestClient(app) as c:
        yield c


def test_api_status(client):
    """GET /api/status returns driver, SLAM, and point cloud state."""
    response = client.get("/api/status")
    assert response.status_code == 200
    data = response.json()

    assert "driver" in data
    assert data["driver"]["mock"] is True
    assert "slam" in data
    assert "tracking_ok" in data["slam"]
    assert "is_streaming" in data["slam"]
    assert "total_points" in data["slam"]
    assert "current_pose" in data["slam"]
    assert len(data["slam"]["current_pose"]) == 4


def test_api_export_empty_and_populated(client):
    """POST /api/export saves .ply point cloud and returns valid file path."""
    response = client.post("/api/export")
    assert response.status_code == 200
    data = response.json()

    assert data["success"] is True
    assert "filename" in data
    assert data["filename"].endswith(".ply")
    assert "filepath" in data
    assert data["point_count"] == 0

    # Also test after populating some points
    driver = client.app.state.scan.driver
    slam = client.app.state.scan.slam
    rgb, depth = driver.get_frame()
    slam.process_frame(rgb, depth)
    assert len(slam.global_pcd.points) > 0

    pop_resp = client.post("/api/export")
    assert pop_resp.status_code == 200
    pop_data = pop_resp.json()
    assert pop_data["success"] is True
    assert pop_data["point_count"] > 0
    assert pop_data["filename"].endswith(".ply")


def test_websocket_control_and_streaming(client):
    """Test WebSocket connection lifecycle, control messages, and binary point cloud buffer."""
    with client.websocket_connect("/ws/scan") as ws:
        # 1. Test ping/pong
        ws.send_text(json.dumps({"cmd": "ping"}))
        res = json.loads(ws.receive_text())
        assert res["type"] == "pong"

        # 2. Test tilt control
        ws.send_text(json.dumps({"cmd": "tilt", "angle": 15}))
        res = json.loads(ws.receive_text())
        assert res["type"] == "control_ack"
        assert res["cmd"] == "tilt"
        assert res["angle"] == 15
        assert res["success"] is True

        # 3. Start streaming
        ws.send_text(json.dumps({"cmd": "start"}))
        ack = json.loads(ws.receive_text())
        assert ack["type"] == "control_ack"
        assert ack["cmd"] == "start"
        assert ack["status"] == "streaming"

        # 4. First binary frame after "start" is the full-map snapshot
        snapshot = ws.receive_bytes()
        snap_kind, snap_count, snap_mode = struct.unpack("<III", snapshot[:12])
        assert snap_kind == 0  # MSG_POINTS
        assert snap_mode == 1  # MODE_REPLACE
        assert len(snapshot) == 12 + snap_count * 16

        # 5. Subsequent binary frames are incremental deltas
        data = ws.receive_bytes()
        assert isinstance(data, bytes)
        assert len(data) >= 12

        kind, point_count, mode = struct.unpack("<III", data[:12])
        assert kind == 0  # MSG_POINTS
        assert mode == 0  # MODE_APPEND
        assert point_count > 0
        expected_len = 12 + point_count * 16
        assert len(data) == expected_len

        # Validate vertex structure: (x, y, z float32, r, g, b, a uint8)
        first_vertex = data[12:28]
        x, y, z = struct.unpack("<fff", first_vertex[:12])
        r, g, b, a = struct.unpack("4B", first_vertex[12:])
        assert isinstance(x, float)
        assert isinstance(y, float)
        assert isinstance(z, float)
        assert a == 255

        # 6. Pause streaming
        ws.send_text(json.dumps({"cmd": "pause"}))
        pause_ack = json.loads(ws.receive_text())
        assert pause_ack["type"] == "control_ack"
        assert pause_ack["cmd"] == "pause"

        # 7. Reset SLAM map
        ws.send_text(json.dumps({"cmd": "reset"}))
        reset_ack = json.loads(ws.receive_text())
        assert reset_ack["type"] == "control_ack"
        assert reset_ack["cmd"] == "reset"
        assert reset_ack["total_points"] == 0

    # 8. Check /api/status after streaming
    status_resp = client.get("/api/status")
    status_data = status_resp.json()
    assert status_data["slam"]["is_streaming"] is False


def test_websocket_streams_mesh(client):
    """The viewer renders a shaded surface, so the socket must deliver a mesh
    message alongside the point deltas."""
    import struct

    with client.websocket_connect("/ws/scan") as ws:
        ws.send_text(json.dumps({"cmd": "start"}))
        json.loads(ws.receive_text())          # start ack
        ws.receive_bytes()                     # point snapshot

        # Give the pipeline a frame so the TSDF has something in it.
        ws.send_text(json.dumps({"cmd": "mesh"}))

        mesh = None
        for _ in range(60):
            msg = ws.receive()
            if msg.get("bytes") is None:
                continue
            data = msg["bytes"]
            kind, a, b = struct.unpack("<III", data[:12])
            if kind == 1:  # MSG_MESH
                mesh = (data, a, b)
                break

        assert mesh is not None, "no mesh message arrived"
        data, nv, nt = mesh
        assert len(data) == 12 + nv * 12 + nt * 12 + nv * 3 + nv * 3
        if nt:
            idx = struct.unpack_from(f"<{nt * 3}I", data, 12 + nv * 12)
            assert max(idx) < nv
