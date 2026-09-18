import {
  Play,
  Pause,
  ArrowCounterClockwise,
  DownloadSimple,
  CaretUp,
  CaretDown,
  SpinnerGap,
  ArrowsClockwise,
  Cube,
  CircleDashed,
} from "@phosphor-icons/react";

export interface ControlDockProps {
  isStreaming: boolean;
  onToggleStreaming: () => void;
  onResetSlam: () => void;
  onTiltChange: (newAngle: number) => void;
  currentTiltAngle: number;
  onExportPly: () => Promise<void>;
  onExportMesh: () => Promise<void>;
  isExporting?: boolean;
  pointSize: number;
  onPointSizeChange: (size: number) => void;
  renderMode: "mesh" | "points";
  onRenderModeChange: (mode: "mesh" | "points") => void;
}

export function ControlDock({
  isStreaming,
  onToggleStreaming,
  onResetSlam,
  onTiltChange,
  currentTiltAngle,
  onExportPly,
  onExportMesh,
  isExporting = false,
  pointSize,
  onPointSizeChange,
  renderMode,
  onRenderModeChange,
}: ControlDockProps) {
  const handleNudge = (delta: number) => {
    const next = Math.max(-27, Math.min(27, currentTiltAngle + delta));
    onTiltChange(next);
  };

  const cyclePointSize = () => {
    // Cycles between fine LiDAR points (0.025) -> dense points (0.045) -> solid splats (0.075)
    if (pointSize <= 0.03) onPointSizeChange(0.045);
    else if (pointSize <= 0.05) onPointSizeChange(0.075);
    else onPointSizeChange(0.025);
  };

  const getSplatLabel = () => {
    if (pointSize <= 0.03) return "FINE";
    if (pointSize <= 0.05) return "DENSE";
    return "SOLID";
  };

  const isMesh = renderMode === "mesh";

  return (
    <div className="absolute bottom-6 left-1/2 -translate-x-1/2 z-20 pointer-events-auto select-none">
      {/* Outer Shell: Double-bezel wrapper */}
      <div className="p-1.5 rounded-full bg-zinc-900/60 backdrop-blur-2xl border border-white/10 shadow-2xl">
        {/* Inner Core container */}
        <div className="flex items-center gap-2 px-3 py-2 rounded-full bg-zinc-950/80 border border-white/5 shadow-[inset_0_1px_1px_rgba(255,255,255,0.12)]">
          {/* 0. Surface / Points view toggle */}
          <div className="flex items-center rounded-full bg-white/5 border border-white/10 p-0.5">
            <button
              onClick={() => onRenderModeChange("mesh")}
              title="Shaded reconstructed surface"
              className={`px-3 py-1.5 rounded-full text-[11px] font-semibold tracking-wide transition-colors ${
                isMesh ? "bg-sky-500/25 text-sky-200" : "text-zinc-500 hover:text-zinc-300"
              }`}
            >
              Surface
            </button>
            <button
              onClick={() => onRenderModeChange("points")}
              title="Raw accumulated point cloud"
              className={`px-3 py-1.5 rounded-full text-[11px] font-semibold tracking-wide transition-colors ${
                !isMesh ? "bg-sky-500/25 text-sky-200" : "text-zinc-500 hover:text-zinc-300"
              }`}
            >
              Points
            </button>
          </div>

          <div className="w-[1px] h-6 bg-white/10" />

          {/* 1. Primary Action: Start / Pause Streaming (Button-in-Button Trailing Icon) */}
          <button
            onClick={onToggleStreaming}
            className={`group flex items-center justify-between gap-3 pl-4 pr-1.5 py-1.5 rounded-full text-xs font-semibold tracking-wide transition-all duration-200 active:scale-[0.98] ${
              isStreaming
                ? "bg-amber-500/15 text-amber-300 hover:bg-amber-500/25 border border-amber-500/30"
                : "bg-emerald-500/20 text-emerald-300 hover:bg-emerald-500/30 border border-emerald-500/30"
            }`}
          >
            <span>{isStreaming ? "Pause Stream" : "Start Scan"}</span>
            <div className="w-7 h-7 rounded-full bg-white/10 flex items-center justify-center transition-transform duration-200 group-hover:scale-105 group-hover:translate-x-0.5">
              {isStreaming ? (
                <Pause size={13} weight="fill" />
              ) : (
                <Play size={13} weight="fill" />
              )}
            </div>
          </button>

          {/* Divider */}
          <div className="w-[1px] h-6 bg-zinc-800" />

          {/* 2. Secondary Action: Reset SLAM */}
          <button
            onClick={onResetSlam}
            className="group flex items-center justify-between gap-2.5 pl-3.5 pr-1.5 py-1.5 rounded-full text-xs font-medium text-zinc-300 bg-zinc-900/90 hover:bg-zinc-800/90 border border-white/5 hover:border-white/15 transition-all duration-200 active:scale-[0.98]"
          >
            <span>Reset SLAM</span>
            <div className="w-7 h-7 rounded-full bg-zinc-800 flex items-center justify-center text-zinc-400 group-hover:text-zinc-200 transition-transform duration-200 group-hover:rotate-45">
              <ArrowCounterClockwise size={13} weight="bold" />
            </div>
          </button>

          {/* Divider */}
          <div className="w-[1px] h-6 bg-zinc-800" />

          {/* 3. Motor Tilt Angle Controls: [-5° / +5°] + Reset 0° with motor degree telemetry */}
          <div className="flex items-center gap-1.5 px-2 py-1 rounded-full bg-zinc-900/60 border border-white/5">
            <span className="text-[10px] font-mono uppercase tracking-wider text-zinc-500 pl-1">
              Tilt
            </span>
            <span className="w-10 text-center text-xs font-mono-telemetry font-bold text-zinc-200">
              {currentTiltAngle > 0 ? `+${currentTiltAngle}°` : `${currentTiltAngle}°`}
            </span>
            <div className="flex items-center gap-1">
              <button
                onClick={() => handleNudge(-5)}
                disabled={currentTiltAngle <= -27}
                title="Tilt down -5°"
                className="w-6 h-6 rounded-full bg-zinc-800/90 hover:bg-zinc-700 flex items-center justify-center text-zinc-300 disabled:opacity-30 disabled:pointer-events-none transition-all duration-150 active:scale-[0.96]"
              >
                <CaretDown size={12} weight="bold" />
              </button>
              <button
                onClick={() => handleNudge(5)}
                disabled={currentTiltAngle >= 27}
                title="Tilt up +5°"
                className="w-6 h-6 rounded-full bg-zinc-800/90 hover:bg-zinc-700 flex items-center justify-center text-zinc-300 disabled:opacity-30 disabled:pointer-events-none transition-all duration-150 active:scale-[0.96]"
              >
                <CaretUp size={12} weight="bold" />
              </button>
              <button
                onClick={() => onTiltChange(0)}
                disabled={currentTiltAngle === 0}
                title="Reset tilt to 0° (Center)"
                className="flex items-center gap-1 px-1.5 h-6 rounded-full bg-zinc-800/90 hover:bg-zinc-700 text-[10px] font-mono font-medium text-zinc-300 hover:text-zinc-100 disabled:opacity-25 disabled:pointer-events-none transition-all duration-150 active:scale-[0.96] border border-white/5"
              >
                <ArrowsClockwise size={11} weight="bold" />
                <span>0°</span>
              </button>
            </div>
          </div>

          {/* Divider */}
          <div className="w-[1px] h-6 bg-zinc-800" />

          {/* 4. Density / Splat Mode Toggle (Fine -> Dense -> Solid) */}
          <button
            onClick={cyclePointSize}
            title="Toggle Point Cloud Density / Splatting Mode"
            className="flex items-center gap-1.5 px-3 py-1.5 rounded-full bg-zinc-900/90 hover:bg-zinc-800 border border-white/5 hover:border-white/15 transition-all duration-150 active:scale-[0.98]"
          >
            <CircleDashed size={13} weight="bold" className="text-emerald-400" />
            <span className="text-[10px] font-mono uppercase tracking-wider text-zinc-400">
              DENSITY:
            </span>
            <span className="text-[11px] font-mono font-bold text-zinc-100">
              {getSplatLabel()}
            </span>
          </button>

          {/* Divider */}
          <div className="w-[1px] h-6 bg-zinc-800" />

          {/* 5. Export PLY Point Cloud */}
          <button
            onClick={onExportPly}
            disabled={isExporting}
            className="group flex items-center justify-between gap-2 pl-3 pr-1.5 py-1.5 rounded-full text-xs font-medium text-cyan-300 bg-cyan-950/40 hover:bg-cyan-900/50 border border-cyan-500/20 hover:border-cyan-500/40 transition-all duration-200 active:scale-[0.98] disabled:opacity-50"
          >
            <span>Points (.ply)</span>
            <div className="w-7 h-7 rounded-full bg-cyan-500/10 border border-cyan-500/30 flex items-center justify-center text-cyan-400 group-hover:scale-105 transition-transform duration-200">
              {isExporting ? (
                <SpinnerGap size={13} className="animate-spin" />
              ) : (
                <DownloadSimple size={13} weight="bold" />
              )}
            </div>
          </button>

          {/* 6. Export 3D Polygonal Mesh */}
          <button
            onClick={onExportMesh}
            disabled={isExporting}
            title="Extract and export continuous 3D polygon surface mesh"
            className="group flex items-center justify-between gap-2 pl-3 pr-1.5 py-1.5 rounded-full text-xs font-medium text-emerald-300 bg-emerald-950/40 hover:bg-emerald-900/50 border border-emerald-500/20 hover:border-emerald-500/40 transition-all duration-200 active:scale-[0.98] disabled:opacity-50"
          >
            <span>Mesh (.ply)</span>
            <div className="w-7 h-7 rounded-full bg-emerald-500/10 border border-emerald-500/30 flex items-center justify-center text-emerald-400 group-hover:scale-105 transition-transform duration-200">
              {isExporting ? (
                <SpinnerGap size={13} className="animate-spin" />
              ) : (
                <Cube size={13} weight="bold" />
              )}
            </div>
          </button>
        </div>
      </div>
    </div>
  );
}
