import { useEffect, useRef } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { CloudStore, MeshStore, TelemetryData } from "../hooks/usePointBuffer";

export type RenderMode = "mesh" | "points";

export interface Viewport3DProps {
  /** Mutable point store written by the WebSocket; polled from the rAF loop. */
  cloud: React.RefObject<CloudStore>;
  /** Mutable surface store, replaced whole each time the TSDF is re-extracted. */
  mesh: React.RefObject<MeshStore>;
  telemetry: TelemetryData;
  renderMode?: RenderMode;
  pointSize?: number;
  className?: string;
}

/**
 * Creates a soft circular disc texture for Gaussian LiDAR splatting.
 * Points blend smoothly into a contiguous solid surface instead of jagged square pixels.
 */
function createCircleTexture(): THREE.Texture {
  const canvas = document.createElement("canvas");
  canvas.width = 64;
  canvas.height = 64;
  const ctx = canvas.getContext("2d");
  if (ctx) {
    const gradient = ctx.createRadialGradient(32, 32, 0, 32, 32, 32);
    gradient.addColorStop(0, "rgba(255,255,255,1.0)");
    gradient.addColorStop(0.75, "rgba(255,255,255,0.85)");
    gradient.addColorStop(1, "rgba(255,255,255,0.0)");
    ctx.fillStyle = gradient;
    ctx.beginPath();
    ctx.arc(32, 32, 32, 0, Math.PI * 2);
    ctx.fill();
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.generateMipmaps = false;
  texture.minFilter = THREE.LinearFilter;
  return texture;
}

/**
 * Generates an indexed line segments geometry representing a pinhole camera frustum pyramid.
 * Origin (apex) is at (0, 0, 0), looking down +Z (or -Z in typical graphics, adapted for Kinect convention).
 * Kinect v1 optical axis is +Z forward, +X right, +Y down (or up).
 */
function createFrustumGeometry(fovDeg = 45, aspect = 4 / 3, depth = 0.35): THREE.BufferGeometry {
  const fovRad = (fovDeg * Math.PI) / 180;
  const h = 2 * Math.tan(fovRad / 2) * depth;
  const w = h * aspect;

  const halfW = w / 2;
  const halfH = h / 2;

  // 5 vertices:
  // 0: Apex (0, 0, 0)
  // 1: Top-Left (-halfW, halfH, depth)
  // 2: Top-Right (halfW, halfH, depth)
  // 3: Bottom-Right (halfW, -halfH, depth)
  // 4: Bottom-Left (-halfW, -halfH, depth)
  const vertices = new Float32Array([
    0, 0, 0,
    -halfW, halfH, depth,
    halfW, halfH, depth,
    halfW, -halfH, depth,
    -halfW, -halfH, depth,
  ]);

  // 8 edges (16 indices):
  // 4 sides from apex, 4 along the rectangle base
  const indices = new Uint16Array([
    0, 1,
    0, 2,
    0, 3,
    0, 4,
    1, 2,
    2, 3,
    3, 4,
    4, 1,
  ]);

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(vertices, 3));
  geometry.setIndex(new THREE.BufferAttribute(indices, 1));
  return geometry;
}

export function Viewport3D({
  cloud,
  mesh,
  telemetry,
  renderMode = "mesh",
  pointSize = 0.045,
  className = "w-full h-full",
}: Viewport3DProps) {
  const containerRef = useRef<HTMLDivElement>(null);

  // References to keep across renders
  const sceneRef = useRef<THREE.Scene | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const controlsRef = useRef<OrbitControls | null>(null);
  const pointsGeometryRef = useRef<THREE.BufferGeometry | null>(null);
  const pointsMaterialRef = useRef<THREE.PointsMaterial | null>(null);
  const pointCloudRef = useRef<THREE.Points | null>(null);
  const surfaceRef = useRef<THREE.Mesh | null>(null);
  const modeRef = useRef<RenderMode>(renderMode);
  const frustumMeshRef = useRef<THREE.LineSegments | null>(null);
  const frustumMaterialRef = useRef<THREE.LineBasicMaterial | null>(null);

  // 1. Scene Initialization
  useEffect(() => {
    if (!containerRef.current) return;
    const container = containerRef.current;

    const width = container.clientWidth || window.innerWidth;
    const height = container.clientHeight || window.innerHeight;

    // Scene with deep OLED background (#09090b - zinc-950)
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x09090b);
    sceneRef.current = scene;

    // Perspective Camera
    const camera = new THREE.PerspectiveCamera(60, width / height, 0.05, 100);
    camera.position.set(0, 0.8, -2.2);
    camera.lookAt(0, 0, 1.2);
    cameraRef.current = camera;

    // WebGL Renderer
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    // OrbitControls with smooth inertia
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.screenSpacePanning = true;
    controls.maxDistance = 20;
    controls.minDistance = 0.2;
    controls.target.set(0, 0, 1.0);
    controlsRef.current = controls;

    // A surface only reads as a surface when it is lit. Hemisphere light gives
    // the ambient fill that keeps cavities from going black; the two
    // directionals rake across it so geometry casts its own shading.
    const hemi = new THREE.HemisphereLight(0xdfe8ff, 0x1a1a22, 1.15);
    scene.add(hemi);

    const keyLight = new THREE.DirectionalLight(0xffffff, 1.5);
    keyLight.position.set(2.5, 4, -2);
    scene.add(keyLight);

    const fillLight = new THREE.DirectionalLight(0x93b4ff, 0.45);
    fillLight.position.set(-3, 1.5, 3);
    scene.add(fillLight);

    // Subtle dark ground grid
    const grid = new THREE.GridHelper(6, 24, 0x27272a, 0x18181b);
    grid.position.y = -0.6;
    scene.add(grid);

    // Dynamic BufferGeometry for Point Cloud. Attributes are (re)bound in the
    // render loop whenever the store reallocates; see syncCloud below.
    const pointsGeometry = new THREE.BufferGeometry();
    pointsGeometry.setDrawRange(0, 0);
    pointsGeometryRef.current = pointsGeometry;

    const circleTexture = createCircleTexture();

    const pointsMaterial = new THREE.PointsMaterial({
      size: pointSize,
      map: circleTexture,
      vertexColors: true,
      transparent: true,
      opacity: 0.95,
      alphaTest: 0.05,
      depthWrite: true,
      blending: THREE.NormalBlending,
    });
    pointsMaterialRef.current = pointsMaterial;

    const pointCloud = new THREE.Points(pointsGeometry, pointsMaterial);
    pointCloudRef.current = pointCloud;

    // Reconstructed surface. DoubleSide matters: a scan is an open shell, and
    // from inside a room every triangle faces away from the camera.
    const surfaceGeometry = new THREE.BufferGeometry();
    const surfaceMaterial = new THREE.MeshStandardMaterial({
      vertexColors: true,
      roughness: 0.92,
      metalness: 0.0,
      side: THREE.DoubleSide,
      flatShading: false,
    });
    const surface = new THREE.Mesh(surfaceGeometry, surfaceMaterial);
    surface.frustumCulled = false;
    surfaceRef.current = surface;
    // The attribute arrays are oversized and only partially filled, so a
    // bounding sphere over them would be wrong anyway -- and recomputing one
    // per frame over the whole map is exactly the cost this rewrite removes.
    pointCloud.frustumCulled = false;

    // Kinect Camera Frustum Wireframe
    const frustumGeo = createFrustumGeometry(58, 4 / 3, 0.3);
    const frustumMat = new THREE.LineBasicMaterial({
      color: 0x10b981, // Default emerald (#10b981)
      linewidth: 1.5,
      transparent: true,
      opacity: 0.9,
    });
    frustumMaterialRef.current = frustumMat;

    const frustum = new THREE.LineSegments(frustumGeo, frustumMat);
    frustumMeshRef.current = frustum;

    // Root scan container group
    // 1. rotation.x = Math.PI inverts Y-down optical frame to Y-up
    // 2. scale.set(1, 1, 1) preserves default horizontal orientation
    const scanContainer = new THREE.Group();
    scanContainer.rotation.x = Math.PI;
    scanContainer.scale.set(1, 1, 1);
    scanContainer.add(pointCloud);
    scanContainer.add(surface);
    scanContainer.add(frustum);
    scene.add(scanContainer);

    // Point cloud sync: the socket appends into preallocated typed arrays, so
    // a frame costs one partial GPU upload of just the newly written range --
    // no reallocation, no full re-upload, no React render.
    let lastGeneration = -1;
    let lastRevision = -1;
    let uploadedCount = 0;

    const syncCloud = () => {
      const store = cloud.current;
      if (!store || store.revision === lastRevision) return;

      if (store.generation !== lastGeneration) {
        const position = new THREE.BufferAttribute(store.positions, 3);
        const color = new THREE.BufferAttribute(store.colors, 3, true);
        position.setUsage(THREE.DynamicDrawUsage);
        color.setUsage(THREE.DynamicDrawUsage);
        pointsGeometry.setAttribute("position", position);
        pointsGeometry.setAttribute("color", color);
        lastGeneration = store.generation;
        uploadedCount = 0;
      }

      const position = pointsGeometry.getAttribute("position") as THREE.BufferAttribute;
      const color = pointsGeometry.getAttribute("color") as THREE.BufferAttribute;

      // A reset (or a fresh snapshot) rewinds the count; re-upload from zero.
      const start = store.count < uploadedCount ? 0 : uploadedCount;
      const length = store.count - start;
      if (length > 0) {
        position.clearUpdateRanges();
        color.clearUpdateRanges();
        position.addUpdateRange(start * 3, length * 3);
        color.addUpdateRange(start * 3, length * 3);
        position.needsUpdate = true;
        color.needsUpdate = true;
      }

      uploadedCount = store.count;
      lastRevision = store.revision;
      pointsGeometry.setDrawRange(0, store.count);
    };

    // The surface arrives whole, not incrementally, so each revision simply
    // rebinds the attributes onto the freshly received buffers.
    let lastMeshRevision = -1;
    const syncMesh = () => {
      const store = mesh.current;
      if (!store || store.revision === lastMeshRevision) return;
      lastMeshRevision = store.revision;

      if (!store.vertexCount || !store.triangleCount) {
        surfaceGeometry.setDrawRange(0, 0);
        return;
      }

      surfaceGeometry.setAttribute(
        "position", new THREE.BufferAttribute(store.positions, 3));
      surfaceGeometry.setAttribute(
        "normal", new THREE.BufferAttribute(store.normals, 3, true));
      surfaceGeometry.setAttribute(
        "color", new THREE.BufferAttribute(store.colors, 3, true));
      surfaceGeometry.setIndex(new THREE.BufferAttribute(store.indices, 1));
      surfaceGeometry.setDrawRange(0, store.triangleCount * 3);
    };

    const applyMode = () => {
      const isMesh = modeRef.current === "mesh";
      surface.visible = isMesh;
      pointCloud.visible = !isMesh;
    };

    // Render loop
    let animId: number;
    const animate = () => {
      animId = requestAnimationFrame(animate);
      syncCloud();
      syncMesh();
      applyMode();
      controls.update();
      renderer.render(scene, camera);
    };
    animId = requestAnimationFrame(animate);

    // Window / Container resize handler
    const handleResize = () => {
      if (!container || !renderer || !camera) return;
      const w = container.clientWidth;
      const h = container.clientHeight;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    };

    window.addEventListener("resize", handleResize);

    return () => {
      cancelAnimationFrame(animId);
      window.removeEventListener("resize", handleResize);
      controls.dispose();
      renderer.dispose();
      pointsGeometry.dispose();
      pointsMaterial.dispose();
      surfaceGeometry.dispose();
      surfaceMaterial.dispose();
      frustumGeo.dispose();
      frustumMat.dispose();
      if (container.contains(renderer.domElement)) {
        container.removeChild(renderer.domElement);
      }
    };
  }, []);

  // 2. Render mode is read from a ref inside the render loop, so flipping it
  // costs no scene rebuild.
  useEffect(() => {
    modeRef.current = renderMode;
  }, [renderMode]);

  // 2b. Dynamic point size update
  useEffect(() => {
    if (pointsMaterialRef.current && pointSize) {
      pointsMaterialRef.current.size = pointSize;
      pointsMaterialRef.current.needsUpdate = true;
    }
  }, [pointSize]);

  // 3. Update Frustum Pose & Tracking Color
  useEffect(() => {
    const frustum = frustumMeshRef.current;
    const frustumMat = frustumMaterialRef.current;
    if (!frustum || !frustumMat) return;

    // Color: emerald (#10b981) when tracking_ok, crimson (#ef4444) when lost
    const targetColor = telemetry.tracking_ok ? 0x10b981 : 0xef4444;
    if (frustumMat.color.getHex() !== targetColor) {
      frustumMat.color.setHex(targetColor);
    }

    // Pose 4x4 matrix update
    if (telemetry.pose && telemetry.pose.length === 4) {
      const mat = new THREE.Matrix4();
      const p = telemetry.pose;
      // Open3D / standard row-major to Three.js elements (column-major)
      mat.set(
        p[0][0], p[0][1], p[0][2], p[0][3],
        p[1][0], p[1][1], p[1][2], p[1][3],
        p[2][0], p[2][1], p[2][2], p[2][3],
        p[3][0], p[3][1], p[3][2], p[3][3]
      );
      frustum.matrixAutoUpdate = false;
      frustum.matrix.copy(mat);
    }
  }, [telemetry]);

  return <div ref={containerRef} className={className} />;
}
