import { useState, useEffect, useCallback } from "react";
import { usePointBuffer } from "./hooks/usePointBuffer";
import { Viewport3D } from "./components/Viewport3D";
import { SpatialHUD } from "./components/SpatialHUD";
import { ControlDock } from "./components/ControlDock";

export default function App() {
  const { cloud, pointCount, telemetry, isConnected, sendCommand } =
    usePointBuffer({
      url: "ws://localhost:8000/ws/scan",
    });

  const [isStreaming, setIsStreaming] = useState<boolean>(true);
  const [tiltAngle, setTiltAngle] = useState<number>(0);
  const [isMockMode, setIsMockMode] = useState<boolean>(false);
  const [isExporting, setIsExporting] = useState<boolean>(false);
  const [pointSize, setPointSize] = useState<number>(0.045); // Default to dense LiDAR splatting (0.045)

  // Poll driver status initially or when connected to check if mock or real hardware
  useEffect(() => {
    let cancelled = false;
    async function checkDriverStatus() {
      try {
        const res = await fetch("http://localhost:8000/api/status");
        if (!res.ok) return;
        const data = await res.json();
        if (!cancelled && data?.driver) {
          setIsMockMode(Boolean(data.driver.mock));
        }
      } catch {
        // Backend offline or unreachable yet
      }
    }
    checkDriverStatus();
    const interval = setInterval(checkDriverStatus, 5000);
    return () => {
      cancelled = true;
      clearInterval(interval);
    };
  }, []);

  // Control handlers wired to sendCommand
  const handleToggleStreaming = useCallback(() => {
    if (isStreaming) {
      sendCommand("pause");
      setIsStreaming(false);
    } else {
      sendCommand("start");
      setIsStreaming(true);
    }
  }, [isStreaming, sendCommand]);

  const handleResetSlam = useCallback(() => {
    sendCommand("reset");
  }, [sendCommand]);

  const handleTiltChange = useCallback(
    (newAngle: number) => {
      setTiltAngle(newAngle);
      sendCommand("tilt", { angle: newAngle });
    },
    [sendCommand]
  );

  const handleExportPly = useCallback(async () => {
    setIsExporting(true);
    try {
      const response = await fetch("http://localhost:8000/api/export", {
        method: "POST",
      });
      if (!response.ok) throw new Error("Export failed");
      const data = await response.json();
      
      if (data && data.filename) {
        const blob = new Blob([`PLY Point Cloud saved to server at:\n${data.filepath}\nTotal Points: ${data.point_count}`], {
          type: "text/plain;charset=utf-8",
        });
        const url = URL.createObjectURL(blob);
        const link = document.createElement("a");
        link.href = url;
        link.download = `${data.filename}.info.txt`;
        document.body.appendChild(link);
        link.click();
        document.body.removeChild(link);
        URL.revokeObjectURL(url);
      }
    } catch (err) {
      console.error("Failed to export PLY point cloud:", err);
    } finally {
      setIsExporting(false);
    }
  }, []);

  const handleExportMesh = useCallback(async () => {
    setIsExporting(true);
    try {
      const response = await fetch("http://localhost:8000/api/export_mesh", {
        method: "POST",
      });
      if (!response.ok) throw new Error("Mesh export failed");
      const data = await response.json();
      
      if (data && data.filename) {
        const blob = new Blob([`3D Polygon Mesh (TSDF) saved to server at:\n${data.filepath}\nTotal Triangles: ${data.triangle_count}`], {
          type: "text/plain;charset=utf-8",
        });
        const url = URL.createObjectURL(blob);
        const link = document.createElement("a");
        link.href = url;
        link.download = `${data.filename}.info.txt`;
        document.body.appendChild(link);
        link.click();
        document.body.removeChild(link);
        URL.revokeObjectURL(url);
      }
    } catch (err) {
      console.error("Failed to export 3D surface mesh:", err);
    } finally {
      setIsExporting(false);
    }
  }, []);

  return (
    <div className="relative w-screen h-screen overflow-hidden bg-[#09090b] font-sans select-none text-zinc-100">
      {/* 3D WebGL Point Cloud & Camera Frustum Canvas */}
      <Viewport3D
        cloud={cloud}
        telemetry={telemetry}
        pointSize={pointSize}
        className="absolute inset-0 z-0 w-full h-full"
      />

      {/* Top Floating Spatial HUD */}
      <SpatialHUD
        telemetry={telemetry}
        pointCount={pointCount}
        isConnected={isConnected}
        isMockMode={isMockMode}
      />

      {/* Bottom Floating Control Dock */}
      <ControlDock
        isStreaming={isStreaming}
        onToggleStreaming={handleToggleStreaming}
        onResetSlam={handleResetSlam}
        onTiltChange={handleTiltChange}
        currentTiltAngle={tiltAngle}
        onExportPly={handleExportPly}
        onExportMesh={handleExportMesh}
        isExporting={isExporting}
        pointSize={pointSize}
        onPointSizeChange={setPointSize}
      />

      {/* Micro Status Watermark */}
      <footer className="absolute bottom-4 left-4 z-10 hidden sm:flex items-center gap-2 text-[10px] font-mono-telemetry text-zinc-600 bg-zinc-950/40 backdrop-blur-md px-2.5 py-1 rounded-full border border-white/5 pointer-events-none">
        <span className="w-1.5 h-1.5 rounded-full bg-zinc-600" />
        <span>THREE.JS WEBGL2 // ORBIT: L-DRAG ROTATE · R-DRAG PAN · SCROLL ZOOM</span>
      </footer>
    </div>
  );
}
