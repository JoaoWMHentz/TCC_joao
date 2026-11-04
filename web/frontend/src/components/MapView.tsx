import { useCallback, useEffect, useRef, useState } from 'react';
import type { MapData, Point, Pose, Target, XY } from '../types';

export type MapTool = 'move' | 'goal' | 'pose' | 'point';

interface Props {
  map: MapData | null;
  pose: Pose;
  path: XY[];
  scan: XY[];
  points: Point[];
  target: Target | null;
  tool: MapTool;
  showScan: boolean;
  // Clique (ou clique e arraste, que define a orientação) com uma ferramenta ativa
  onPick: (tool: Exclude<MapTool, 'move'>, x: number, y: number, theta: number) => void;
}

interface View {
  scale: number; // pixels de tela por metro
  ox: number; // posição na tela da origem do mundo (x = 0, y = 0)
  oy: number;
}

const ROBOT_RADIUS_M = 0.245;
const DRAG_THRESHOLD_PX = 8;

export default function MapView({ map, pose, path, scan, points, target, tool, showScan, onPick }: Props) {
  const wrapRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const imageRef = useRef<HTMLImageElement | null>(null);
  const [imageVersion, setImageVersion] = useState(0);
  const [view, setView] = useState<View>({ scale: 40, ox: 300, oy: 300 });
  const fittedRef = useRef(false);
  const dragRef = useRef<{ x: number; y: number; view: View; moved: boolean } | null>(null);
  const [aim, setAim] = useState<{ from: XY; to: XY } | null>(null);

  // Mundo (m) <-> tela (px). Y do mundo cresce para cima, Y da tela para baixo.
  const toScreen = useCallback((x: number, y: number): XY => [view.ox + x * view.scale, view.oy - y * view.scale], [view]);
  const toWorld = useCallback((sx: number, sy: number): XY => [(sx - view.ox) / view.scale, (view.oy - sy) / view.scale], [view]);

  // Nova imagem do mapa
  useEffect(() => {
    if (!map) {
      imageRef.current = null;
      fittedRef.current = false;
      setImageVersion((v) => v + 1);
      return;
    }
    const img = new Image();
    img.onload = () => {
      imageRef.current = img;
      setImageVersion((v) => v + 1);
    };
    img.src = `data:image/png;base64,${map.png_b64}`;
  }, [map]);

  // Enquadra o mapa na primeira vez que ele aparece
  const fit = useCallback(() => {
    const wrap = wrapRef.current;
    if (!wrap || !map) return;
    const w = map.width * map.resolution;
    const h = map.height * map.resolution;
    const scale = Math.min(wrap.clientWidth / w, wrap.clientHeight / h) * 0.9;
    const cx = map.origin_x + w / 2;
    const cy = map.origin_y + h / 2;
    setView({ scale, ox: wrap.clientWidth / 2 - cx * scale, oy: wrap.clientHeight / 2 + cy * scale });
  }, [map]);

  useEffect(() => {
    if (map && !fittedRef.current) {
      fittedRef.current = true;
      fit();
    }
  }, [map, fit]);

  // Desenho
  useEffect(() => {
    const canvas = canvasRef.current;
    const wrap = wrapRef.current;
    if (!canvas || !wrap) return;
    const dpr = window.devicePixelRatio || 1;
    const w = wrap.clientWidth;
    const h = wrap.clientHeight;
    if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
      canvas.width = w * dpr;
      canvas.height = h * dpr;
      canvas.style.width = `${w}px`;
      canvas.style.height = `${h}px`;
    }
    const ctx = canvas.getContext('2d')!;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = '#cdcdcd';
    ctx.fillRect(0, 0, w, h);

    const img = imageRef.current;
    if (map && img) {
      const [x0, y1] = toScreen(map.origin_x, map.origin_y + map.height * map.resolution);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(img, x0, y1, map.width * map.resolution * view.scale, map.height * map.resolution * view.scale);
    }

    if (showScan && scan.length) {
      ctx.fillStyle = '#e5484d';
      for (const [x, y] of scan) {
        const [sx, sy] = toScreen(x, y);
        ctx.fillRect(sx - 1.5, sy - 1.5, 3, 3);
      }
    }

    if (path.length > 1) {
      ctx.strokeStyle = '#2f6fed';
      ctx.lineWidth = 3;
      ctx.beginPath();
      path.forEach(([x, y], i) => {
        const [sx, sy] = toScreen(x, y);
        if (i === 0) ctx.moveTo(sx, sy);
        else ctx.lineTo(sx, sy);
      });
      ctx.stroke();
    }

    ctx.font = '12px system-ui, sans-serif';
    for (const p of points) {
      const [sx, sy] = toScreen(p.x, p.y);
      ctx.fillStyle = '#1a7f4b';
      ctx.beginPath();
      ctx.arc(sx, sy, 6, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = '#10251a';
      ctx.fillText(p.name, sx + 9, sy + 4);
    }

    if (target) {
      const [sx, sy] = toScreen(target.x, target.y);
      ctx.strokeStyle = '#d97706';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.arc(sx, sy, 10, 0, Math.PI * 2);
      ctx.moveTo(sx - 14, sy);
      ctx.lineTo(sx + 14, sy);
      ctx.moveTo(sx, sy - 14);
      ctx.lineTo(sx, sy + 14);
      ctx.stroke();
    }

    if (pose.available) {
      const [sx, sy] = toScreen(pose.x, pose.y);
      const r = Math.max(ROBOT_RADIUS_M * view.scale, 8);
      ctx.fillStyle = 'rgba(47, 111, 237, 0.25)';
      ctx.strokeStyle = '#2f6fed';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(sx, sy, r, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(sx, sy);
      ctx.lineTo(sx + Math.cos(pose.theta) * r * 1.4, sy - Math.sin(pose.theta) * r * 1.4);
      ctx.stroke();
    }

    if (aim) {
      const [ax, ay] = toScreen(...aim.from);
      const [bx, by] = toScreen(...aim.to);
      ctx.strokeStyle = tool === 'pose' ? '#7c3aed' : '#d97706';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(ax, ay);
      ctx.lineTo(bx, by);
      ctx.stroke();
    }
  }, [map, imageVersion, view, pose, path, scan, points, target, aim, tool, showScan, toScreen]);

  // Redesenha ao redimensionar a janela
  useEffect(() => {
    const onResize = () => setView((v) => ({ ...v }));
    window.addEventListener('resize', onResize);
    return () => window.removeEventListener('resize', onResize);
  }, []);

  // ── Interação ─────────────────────────────────────────────────────────────
  const localXY = (e: React.PointerEvent | React.WheelEvent): XY => {
    const rect = canvasRef.current!.getBoundingClientRect();
    return [e.clientX - rect.left, e.clientY - rect.top];
  };

  const onPointerDown = (e: React.PointerEvent) => {
    (e.target as Element).setPointerCapture(e.pointerId);
    const [x, y] = localXY(e);
    dragRef.current = { x, y, view, moved: false };
  };

  const onPointerMove = (e: React.PointerEvent) => {
    const drag = dragRef.current;
    if (!drag) return;
    const [x, y] = localXY(e);
    if (Math.hypot(x - drag.x, y - drag.y) > DRAG_THRESHOLD_PX) drag.moved = true;
    if (tool === 'move') {
      setView({ ...drag.view, ox: drag.view.ox + x - drag.x, oy: drag.view.oy + y - drag.y });
    } else if (drag.moved) {
      setAim({ from: toWorld(drag.x, drag.y), to: toWorld(x, y) });
    }
  };

  const onPointerUp = (e: React.PointerEvent) => {
    const drag = dragRef.current;
    dragRef.current = null;
    setAim(null);
    if (!drag || tool === 'move') return;
    const [x, y] = localXY(e);
    const [wx, wy] = toWorld(drag.x, drag.y);
    const theta = drag.moved ? Math.atan2(drag.y - y, x - drag.x) : 0;
    onPick(tool, wx, wy, theta);
  };

  // Zoom mantendo fixo o ponto (x, y) da tela
  const zoomAt = (x: number, y: number, factor: number) =>
    setView((v) => {
      const scale = Math.min(400, Math.max(5, v.scale * factor));
      const k = scale / v.scale;
      return { scale, ox: x - (x - v.ox) * k, oy: y - (y - v.oy) * k };
    });

  const zoomCenter = (factor: number) => {
    const wrap = wrapRef.current;
    if (wrap) zoomAt(wrap.clientWidth / 2, wrap.clientHeight / 2, factor);
  };

  const onWheel = (e: React.WheelEvent) => {
    const [x, y] = localXY(e);
    zoomAt(x, y, e.deltaY < 0 ? 1.15 : 1 / 1.15);
  };

  return (
    <div className={`map-view tool-${tool}`} ref={wrapRef}>
      <canvas
        ref={canvasRef}
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onWheel={onWheel}
      />
      {!map && <div className="map-empty">Nenhum mapa carregado</div>}
      <div className="map-zoom">
        <button onClick={() => zoomCenter(1.3)} aria-label="Aproximar">+</button>
        <button onClick={() => zoomCenter(1 / 1.3)} aria-label="Afastar">−</button>
        <button onClick={fit} aria-label="Enquadrar mapa">⤢</button>
      </div>
    </div>
  );
}
