import { useEffect, useRef, useState, useCallback } from "react";

export interface TelemetryData {
  fps: number;
  tracking_ok: boolean;
  pose: number[][]; // 4x4 transformation matrix
  point_count: number;
  frame_idx?: number;
}

/**
 * Mutable, render-loop-owned point cloud storage.
 *
 * The socket writes straight into these preallocated arrays and bumps the
 * counters; nothing here lives in React state, so a 30 Hz stream costs zero
 * re-renders. Viewport3D polls it from its requestAnimationFrame loop.
 */
export interface CloudStore {
  positions: Float32Array; // xyz, 3 floats per point
  colors: Uint8Array; // rgb, 3 bytes per point (normalized attribute)
  count: number; // valid points
  capacity: number; // allocated points
  revision: number; // bumps whenever `count` or the contents change
  generation: number; // bumps whenever the arrays themselves are replaced
}

export interface UsePointBufferOptions {
  url?: string;
  autoConnect?: boolean;
  initialCapacity?: number;
  maxCapacity?: number;
}

export interface UsePointBufferReturn {
  cloud: React.RefObject<CloudStore>;
  pointCount: number;
  telemetry: TelemetryData;
  isConnected: boolean;
  sendCommand: (cmd: string, payload?: Record<string, unknown>) => void;
}

// Wire format, mirrored from backend/slam_engine.py:
//   header  : uint32 count, uint32 mode
//   vertices: 3 x float32 xyz, then 4 x uint8 rgba
const HEADER_BYTES = 8;
const VERTEX_BYTES = 16;
const MODE_REPLACE = 1;

const DEFAULT_POSE: number[][] = [
  [1, 0, 0, 0],
  [0, 1, 0, 0],
  [0, 0, 1, 0],
  [0, 0, 0, 1],
];

const INITIAL_TELEMETRY: TelemetryData = {
  fps: 0,
  tracking_ok: true,
  pose: DEFAULT_POSE,
  point_count: 0,
};

function makeStore(capacity: number): CloudStore {
  return {
    positions: new Float32Array(capacity * 3),
    colors: new Uint8Array(capacity * 3),
    count: 0,
    capacity,
    revision: 0,
    generation: 0,
  };
}

export function usePointBuffer({
  url = "ws://localhost:8000/ws/scan",
  autoConnect = true,
  initialCapacity = 400_000,
  // ponytail: hard ceiling instead of unbounded growth -- ~192 MB of typed
  // arrays at the cap. Raise it, or start decimating the map server-side, if
  // room-scale scans start hitting it.
  maxCapacity = 4_000_000,
}: UsePointBufferOptions = {}): UsePointBufferReturn {
  const [isConnected, setIsConnected] = useState<boolean>(false);
  const [pointCount, setPointCount] = useState<number>(0);
  const [telemetry, setTelemetry] = useState<TelemetryData>(INITIAL_TELEMETRY);

  const cloud = useRef<CloudStore>(makeStore(initialCapacity));
  const socketRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<number | null>(null);

  const sendCommand = useCallback((cmd: string, payload: Record<string, unknown> = {}) => {
    if (socketRef.current && socketRef.current.readyState === WebSocket.OPEN) {
      socketRef.current.send(JSON.stringify({ cmd, ...payload }));
    }
  }, []);

  useEffect(() => {
    if (!autoConnect) return;

    let isMounted = true;

    /** Grows the backing arrays (preserving contents) to hold `needed` points. */
    function ensureCapacity(store: CloudStore, needed: number): boolean {
      if (needed <= store.capacity) return true;
      if (needed > maxCapacity) return false;

      let next = store.capacity;
      while (next < needed) next *= 2;
      next = Math.min(next, maxCapacity);

      const positions = new Float32Array(next * 3);
      const colors = new Uint8Array(next * 3);
      positions.set(store.positions.subarray(0, store.count * 3));
      colors.set(store.colors.subarray(0, store.count * 3));

      store.positions = positions;
      store.colors = colors;
      store.capacity = next;
      store.generation++;
      return true;
    }

    function applyPacket(buffer: ArrayBuffer) {
      if (buffer.byteLength < HEADER_BYTES) return;

      const header = new Uint32Array(buffer, 0, 2);
      const count = header[0];
      const mode = header[1];
      if (buffer.byteLength < HEADER_BYTES + count * VERTEX_BYTES) return;

      const store = cloud.current;
      if (mode === MODE_REPLACE) store.count = 0;

      const base = store.count;
      if (!ensureCapacity(store, base + count)) {
        console.warn(`Point cloud capacity (${maxCapacity}) reached; dropping ${count} points.`);
        return;
      }

      // De-interleave the delta only. `count` is the frame's new points (a few
      // hundred), not the whole map, so this stays trivially cheap.
      const xyz = new Float32Array(buffer, HEADER_BYTES, count * 4);
      const bytes = new Uint8Array(buffer, HEADER_BYTES, count * VERTEX_BYTES);
      const { positions, colors } = store;

      for (let i = 0; i < count; i++) {
        const src = i * 4;
        const dst = (base + i) * 3;
        positions[dst] = xyz[src];
        positions[dst + 1] = xyz[src + 1];
        positions[dst + 2] = xyz[src + 2];

        const cSrc = i * VERTEX_BYTES + 12;
        colors[dst] = bytes[cSrc];
        colors[dst + 1] = bytes[cSrc + 1];
        colors[dst + 2] = bytes[cSrc + 2];
      }

      store.count = base + count;
      store.revision++;
    }

    function clearCloud() {
      const store = cloud.current;
      store.count = 0;
      store.revision++;
      setPointCount(0);
    }

    function connect() {
      if (socketRef.current && socketRef.current.readyState === WebSocket.OPEN) {
        return;
      }

      try {
        const ws = new WebSocket(url);
        ws.binaryType = "arraybuffer";
        socketRef.current = ws;

        ws.onopen = () => {
          if (!isMounted) return;
          setIsConnected(true);
          // The server answers "start" with a full-map snapshot, so a reconnect
          // resynchronises instead of appending onto a stale local map.
          clearCloud();
          ws.send(JSON.stringify({ cmd: "start" }));
        };

        ws.onclose = () => {
          if (!isMounted) return;
          setIsConnected(false);
          socketRef.current = null;
          reconnectTimeoutRef.current = window.setTimeout(() => {
            if (isMounted) connect();
          }, 2000);
        };

        ws.onerror = () => {
          if (ws.readyState === WebSocket.OPEN) {
            ws.close();
          }
        };

        ws.onmessage = (event: MessageEvent) => {
          if (!isMounted) return;

          if (event.data instanceof ArrayBuffer) {
            applyPacket(event.data);
            return;
          }

          if (typeof event.data !== "string") return;
          try {
            const msg = JSON.parse(event.data);
            if (msg.type === "telemetry") {
              setTelemetry({
                fps: msg.fps ?? 0,
                tracking_ok: msg.tracking_ok ?? true,
                pose: msg.pose ?? DEFAULT_POSE,
                point_count: msg.point_count ?? 0,
                frame_idx: msg.frame_idx,
              });
              // Telemetry arrives every ~10 frames, which is a plenty-fast
              // cadence for a counter and keeps React out of the hot path.
              setPointCount(msg.point_count ?? 0);
            } else if (msg.type === "control_ack" && msg.cmd === "reset") {
              clearCloud();
            }
          } catch {
            // Ignore malformed JSON
          }
        };
      } catch {
        reconnectTimeoutRef.current = window.setTimeout(() => {
          if (isMounted) connect();
        }, 2000);
      }
    }

    connect();

    return () => {
      isMounted = false;
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
      }
      if (socketRef.current) {
        socketRef.current.close();
        socketRef.current = null;
      }
    };
  }, [url, autoConnect, maxCapacity]);

  return { cloud, pointCount, telemetry, isConnected, sendCommand };
}
