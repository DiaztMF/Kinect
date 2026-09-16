import { TelemetryData } from "../hooks/usePointBuffer";
import { Circle, Pulse, Cube } from "@phosphor-icons/react";

export interface SpatialHUDProps {
  telemetry: TelemetryData;
  pointCount: number;
  isConnected: boolean;
  isMockMode?: boolean;
}

export function SpatialHUD({
  telemetry,
  pointCount,
  isConnected,
  isMockMode = false,
}: SpatialHUDProps) {
  // Extract pose translation vector (Tx, Ty, Tz) from row 0-2 of col 3
  const pose = telemetry.pose;
  const tx = pose && pose[0] ? (pose[0][3] ?? 0) : 0;
  const ty = pose && pose[1] ? (pose[1][3] ?? 0) : 0;
  const tz = pose && pose[2] ? (pose[2][3] ?? 0) : 0;

  const trackingOk = telemetry.tracking_ok;
  const fpsDisplay = typeof telemetry.fps === "number" ? telemetry.fps.toFixed(1) : "0.0";
  const pointsFormatted = pointCount.toLocaleString();

  // Status configuration
  let statusText = "DISCONNECTED";
  let statusColor = "text-zinc-400";
  let statusBg = "bg-zinc-800/40 border-zinc-700/40";
  let dotColor = "bg-zinc-500";

  if (isConnected) {
    if (isMockMode) {
      statusText = "MOCK MODE";
      statusColor = "text-amber-400";
      statusBg = "bg-amber-500/10 border-amber-500/20";
      dotColor = "bg-amber-400";
    } else {
      statusText = "HARDWARE ACTIVE";
      statusColor = "text-emerald-400";
      statusBg = "bg-emerald-500/10 border-emerald-500/20";
      dotColor = "bg-emerald-400";
    }
  }

  return (
    <header className="absolute top-4 left-4 right-4 z-20 flex flex-col md:flex-row items-start md:items-center justify-between gap-3 pointer-events-none select-none">
      {/* Left: Brand Identity & Hardware Status Pill */}
      <div className="pointer-events-auto flex items-center gap-3">
        {/* Double-bezel Brand Capsule */}
        <div className="p-1 rounded-[1.25rem] bg-zinc-900/60 backdrop-blur-2xl border border-white/10 shadow-2xl">
          <div className="flex items-center gap-3 px-3.5 py-2 rounded-[calc(1.25rem-0.25rem)] bg-zinc-950/80 border border-white/5 shadow-[inset_0_1px_1px_rgba(255,255,255,0.12)]">
            <div className="w-7 h-7 rounded-lg bg-zinc-900 border border-white/10 flex items-center justify-center text-zinc-100 shadow-inner">
              <Cube size={15} weight="bold" />
            </div>
            <div className="flex flex-col">
              <div className="flex items-center gap-2">
                <span className="text-xs font-bold tracking-wider text-zinc-100 font-mono-telemetry">
                  ENUMA KINECT
                </span>
                <span className="text-[10px] text-zinc-600 font-mono">//</span>
                <span className="text-[11px] font-medium tracking-widest text-zinc-400 uppercase font-mono-telemetry">
                  SPATIAL MAPPER
                </span>
              </div>
            </div>
          </div>
        </div>

        {/* Live Status Pill with double-bezel nesting */}
        <div className="p-1 rounded-full bg-zinc-900/60 backdrop-blur-2xl border border-white/10 shadow-2xl">
          <div
            className={`flex items-center gap-2 px-3 py-1.5 rounded-full bg-zinc-950/80 border ${statusBg} shadow-[inset_0_1px_1px_rgba(255,255,255,0.08)]`}
          >
            <span className="relative flex h-2 w-2">
              {isConnected && (
                <span
                  className={`animate-ping absolute inline-flex h-full w-full rounded-full opacity-75 ${dotColor}`}
                />
              )}
              <span className={`relative inline-flex rounded-full h-2 w-2 ${dotColor}`} />
            </span>
            <span className={`text-[10px] font-mono-telemetry font-semibold tracking-wider ${statusColor}`}>
              {statusText}
            </span>
          </div>
        </div>
      </div>

      {/* Right: Double-Bezel Telemetry Glass Card */}
      <div className="pointer-events-auto p-1.5 rounded-[1.5rem] bg-zinc-900/60 backdrop-blur-2xl border border-white/10 shadow-2xl">
        <div className="flex items-center gap-4 px-4 py-2 rounded-[calc(1.5rem-0.375rem)] bg-zinc-950/80 border border-white/5 shadow-[inset_0_1px_1px_rgba(255,255,255,0.12)]">
          {/* Active Points */}
          <div className="flex flex-col">
            <span className="text-[9px] font-mono tracking-widest uppercase text-zinc-500">
              Active Points
            </span>
            <div className="flex items-baseline gap-1">
              <span className="text-xs font-mono-telemetry font-bold text-zinc-100">
                {pointsFormatted}
              </span>
              <span className="text-[10px] font-mono text-zinc-500">pts</span>
            </div>
          </div>

          <div className="w-[1px] h-6 bg-zinc-800" />

          {/* Stream FPS */}
          <div className="flex flex-col">
            <span className="text-[9px] font-mono tracking-widest uppercase text-zinc-500">
              Stream
            </span>
            <div className="flex items-center gap-1.5">
              <Pulse size={12} weight="bold" className={isConnected ? "text-cyan-400" : "text-zinc-600"} />
              <span className="text-xs font-mono-telemetry font-bold text-zinc-200">
                {fpsDisplay}
              </span>
              <span className="text-[10px] font-mono text-zinc-500">fps</span>
            </div>
          </div>

          <div className="w-[1px] h-6 bg-zinc-800" />

          {/* Tracking State */}
          <div className="flex flex-col">
            <span className="text-[9px] font-mono tracking-widest uppercase text-zinc-500">
              Tracking
            </span>
            <div className="flex items-center gap-1.5">
              <Circle
                size={8}
                weight="fill"
                className={trackingOk ? "text-emerald-400" : "text-rose-500"}
              />
              <span
                className={`text-xs font-mono-telemetry font-bold tracking-wide ${
                  trackingOk ? "text-emerald-400" : "text-rose-400"
                }`}
              >
                {trackingOk ? "LOCKED" : "DRIFT / LOST"}
              </span>
            </div>
          </div>

          <div className="w-[1px] h-6 bg-zinc-800" />

          {/* Pose Position Coordinates */}
          <div className="flex flex-col">
            <span className="text-[9px] font-mono tracking-widest uppercase text-zinc-500">
              Pose (m)
            </span>
            <div className="text-xs font-mono-telemetry text-zinc-300 flex items-center gap-2">
              <span>
                <span className="text-zinc-600">X:</span> {tx >= 0 ? ` ${tx.toFixed(2)}` : tx.toFixed(2)}
              </span>
              <span>
                <span className="text-zinc-600">Y:</span> {ty >= 0 ? ` ${ty.toFixed(2)}` : ty.toFixed(2)}
              </span>
              <span>
                <span className="text-zinc-600">Z:</span> {tz >= 0 ? ` ${tz.toFixed(2)}` : tz.toFixed(2)}
              </span>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}
