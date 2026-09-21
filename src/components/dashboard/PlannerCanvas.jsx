import { useEffect, useRef, useState } from "react";
import {
  CANVAS_HEIGHT,
  CANVAS_WIDTH,
  POINT_KIND_META,
  SCALE,
  canvasToWorld,
  drawDiamond,
  drawPlannerBackground,
  worldToCanvas,
} from "../../lib/zonePlanner";
import { INITIAL_TELEMETRY, normalizeAngle } from "../../lib/dashboardTelemetry";

const INITIAL_DRAW_STATE = {
  visitEntries: [],
  chargeEntries: [],
  plannedVisitEntryMap: new Map(),
  zoneEntries: [],
  surfaceZones: [],
  optimizedRoute: [],
  obstacleTrace: [],
  obstacleMap: INITIAL_TELEMETRY.obstacleMap,
  manualObstacles: [],
  routeBlocked: false,
  hoveredPointIndex: null,
};

export default function PlannerCanvas({
  canvasRef,
  plannerModel,
  optimizedRoute,
  hoveredPointIndex,
  telemetry,
  manualObstacles = [],
  visibleLayers,
  onCanvasClick,
  onCanvasMouseDown,
  onCanvasMouseMove,
  onCanvasMouseUp,
  onCanvasMouseLeave,
}) {
  const telemetryTargetRef = useRef({ ...INITIAL_TELEMETRY });
  const telemetryRenderRef = useRef({ ...INITIAL_TELEMETRY });
  const drawStateRef = useRef(INITIAL_DRAW_STATE);
  const [cursorPoint, setCursorPoint] = useState(null);

  useEffect(() => {
    telemetryTargetRef.current = telemetry;
  }, [telemetry]);

  useEffect(() => {
    drawStateRef.current = {
      visitEntries: plannerModel.visitEntries,
      chargeEntries: plannerModel.chargeEntries,
      plannedVisitEntryMap: plannerModel.plannedVisitEntryMap,
      zoneEntries: plannerModel.zoneEntries,
      surfaceZones: plannerModel.surfaceZones,
      optimizedRoute,
      obstacleTrace: telemetry.obstacleTrace || [],
      obstacleMap: telemetry.obstacleMap || INITIAL_TELEMETRY.obstacleMap,
      manualObstacles,
      routeBlocked: plannerModel.routeBlocked,
      hoveredPointIndex,
    };
  }, [
    hoveredPointIndex,
    optimizedRoute,
    plannerModel.chargeEntries,
    plannerModel.plannedVisitEntryMap,
    plannerModel.routeBlocked,
    plannerModel.surfaceZones,
    plannerModel.visitEntries,
    plannerModel.zoneEntries,
    manualObstacles,
    telemetry.obstacleMap,
    telemetry.obstacleTrace,
  ]);

  useEffect(() => {
    let raf = 0;

    const loop = () => {
      if (!canvasRef.current) {
        raf = window.requestAnimationFrame(loop);
        return;
      }

      const ctx = canvasRef.current.getContext("2d");
      if (!ctx) {
        raf = window.requestAnimationFrame(loop);
        return;
      }

      const target = telemetryTargetRef.current;
      const current = telemetryRenderRef.current;
      const state = drawStateRef.current;
      const alpha = 0.35;

      current.x += (target.x - current.x) * alpha;
      current.y += (target.y - current.y) * alpha;
      current.z += (target.z - current.z) * alpha;
      current.yaw += normalizeAngle(target.yaw - current.yaw) * alpha;

      drawPlannerBackground(ctx, visibleLayers.surfaces ? state.surfaceZones : []);

      if (visibleLayers.obstacleTrace && state.obstacleMap?.cells?.length) {
        const rawCellSize = Number(state.obstacleMap.cellSize);
        const cellSize = Number.isFinite(rawCellSize) && rawCellSize > 0 ? rawCellSize : 0.06;
        const cellCanvasSize = Math.max(3, cellSize * SCALE * 0.92);

        state.obstacleMap.cells.forEach((cell) => {
          const confidenceRaw = Number(cell?.confidence);
          const confidence = Number.isFinite(confidenceRaw) ? Math.max(0, confidenceRaw) : 0;
          const intensity = Math.max(0.16, Math.min(1, confidence / 6));
          const point = worldToCanvas(cell.x, cell.y);

          ctx.fillStyle = `rgba(14, 165, 233, ${0.12 + intensity * 0.3})`;
          ctx.strokeStyle = `rgba(2, 132, 199, ${0.18 + intensity * 0.38})`;
          ctx.lineWidth = 1;
          ctx.fillRect(
            point.x - cellCanvasSize / 2,
            point.y - cellCanvasSize / 2,
            cellCanvasSize,
            cellCanvasSize
          );
          ctx.strokeRect(
            point.x - cellCanvasSize / 2,
            point.y - cellCanvasSize / 2,
            cellCanvasSize,
            cellCanvasSize
          );
        });
      }

      if (visibleLayers.zones) state.zoneEntries.forEach((zone) => {
        if (zone.points.length > 1) {
          ctx.setLineDash([10, 8]);
          ctx.strokeStyle = zone.color.stroke;
          ctx.lineWidth = 3;
          ctx.beginPath();

          zone.points.forEach((entry, index) => {
            const point = worldToCanvas(entry.point.x, entry.point.y);
            if (index === 0) ctx.moveTo(point.x, point.y);
            else ctx.lineTo(point.x, point.y);
          });

          if (zone.closed && zone.points.length >= 3) {
            const first = worldToCanvas(zone.points[0].point.x, zone.points[0].point.y);
            ctx.lineTo(first.x, first.y);
            ctx.fillStyle = zone.color.fill;
            ctx.fill();
          }

          ctx.stroke();
          ctx.setLineDash([]);
        }

        zone.points.forEach((entry) => {
          const point = worldToCanvas(entry.point.x, entry.point.y);
          ctx.fillStyle = zone.color.stroke;
          drawDiamond(ctx, point.x, point.y, 12);
          ctx.fill();
          ctx.strokeStyle = "#eff6ff";
          ctx.lineWidth = 2;
          ctx.stroke();
          ctx.fillStyle = "#fff";
          ctx.font = "700 10px 'Segoe UI', sans-serif";
          ctx.textAlign = "center";
          ctx.textBaseline = "middle";
          ctx.fillText(`${zone.zoneIndex + 1}.${entry.order}`, point.x, point.y);
        });

        if (zone.points.length) {
          const anchor = worldToCanvas(zone.points[0].point.x, zone.points[0].point.y);
          ctx.fillStyle = zone.color.stroke;
          ctx.font = "700 11px 'Segoe UI', sans-serif";
          ctx.textAlign = "left";
          ctx.textBaseline = "bottom";
          ctx.fillText(
            zone.closed ? `${zone.name} (замкнута)` : `${zone.name} (открыта)`,
            anchor.x + 14,
            anchor.y - 10
          );
        }
      });

      if (visibleLayers.zones) {
        state.manualObstacles.forEach((obstacle, index) => {
          const center = worldToCanvas(obstacle.x, obstacle.y);
          const width = Math.max(8, obstacle.sizeX * SCALE);
          const height = Math.max(8, obstacle.sizeY * SCALE);

          ctx.fillStyle = "rgba(71, 85, 105, 0.72)";
          ctx.strokeStyle = "#0f172a";
          ctx.lineWidth = 2;
          ctx.fillRect(center.x - width / 2, center.y - height / 2, width, height);
          ctx.strokeRect(center.x - width / 2, center.y - height / 2, width, height);
          ctx.fillStyle = "#ffffff";
          ctx.font = "700 10px 'Segoe UI', sans-serif";
          ctx.textAlign = "center";
          ctx.textBaseline = "middle";
          ctx.fillText(`O${index + 1}`, center.x, center.y);
        });
      }

      if (visibleLayers.obstacleTrace && state.obstacleTrace.length) {
        state.obstacleTrace.forEach((point) => {
          const confidenceRaw = Number(point?.confidence);
          const confidence = Number.isFinite(confidenceRaw)
            ? Math.max(0, Math.min(1, confidenceRaw))
            : 1;
          const hit = worldToCanvas(point.x, point.y);
          const radius = 1.4 + confidence * 1.5;
          const fillAlpha = 0.2 + confidence * 0.46;
          const strokeAlpha = 0.08 + confidence * 0.24;

          ctx.fillStyle = `rgba(15, 118, 110, ${fillAlpha})`;
          ctx.strokeStyle = `rgba(15, 23, 42, ${strokeAlpha})`;
          ctx.lineWidth = 1.1;
          ctx.beginPath();
          ctx.arc(hit.x, hit.y, radius, 0, Math.PI * 2);
          ctx.fill();
          ctx.stroke();
        });
      }

      if (visibleLayers.route && state.optimizedRoute.length > 1) {
        ctx.strokeStyle = state.routeBlocked ? "#dc2626" : "#0f766e";
        ctx.lineWidth = 5;
        ctx.beginPath();
        state.optimizedRoute.forEach((point, index) => {
          const currentPoint = worldToCanvas(point.x, point.y);
          if (index === 0) ctx.moveTo(currentPoint.x, currentPoint.y);
          else ctx.lineTo(currentPoint.x, currentPoint.y);
        });
        ctx.stroke();
      }

      if (visibleLayers.route) state.chargeEntries.forEach((entry) => {
        const point = worldToCanvas(entry.point.x, entry.point.y);
        const hovered = state.hoveredPointIndex === entry.index;

        if (hovered) {
          ctx.strokeStyle = "rgba(245, 158, 11, 0.8)";
          ctx.lineWidth = 4;
          ctx.beginPath();
          ctx.arc(point.x, point.y, 18, 0, Math.PI * 2);
          ctx.stroke();
        }

        ctx.fillStyle = POINT_KIND_META.charge.color;
        ctx.beginPath();
        ctx.arc(point.x, point.y, 11, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = "rgba(120, 53, 15, 0.7)";
        ctx.lineWidth = 2;
        ctx.stroke();

        ctx.fillStyle = "#fff";
        ctx.font = "700 11px 'Segoe UI', sans-serif";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(`C${entry.order}`, point.x, point.y);
      });

      if (visibleLayers.route) state.visitEntries.forEach((entry) => {
        const plannedEntry = state.plannedVisitEntryMap.get(entry.index);
        const point = worldToCanvas(entry.point.x, entry.point.y);
        const hovered = state.hoveredPointIndex === entry.index;

        if (plannedEntry?.adjusted) {
          const projected = worldToCanvas(plannedEntry.plannedPoint.x, plannedEntry.plannedPoint.y);
          ctx.setLineDash([5, 5]);
          ctx.strokeStyle = "#f59e0b";
          ctx.lineWidth = 2;
          ctx.beginPath();
          ctx.moveTo(point.x, point.y);
          ctx.lineTo(projected.x, projected.y);
          ctx.stroke();
          ctx.setLineDash([]);

          ctx.fillStyle = "#f59e0b";
          ctx.beginPath();
          ctx.arc(projected.x, projected.y, 8, 0, Math.PI * 2);
          ctx.fill();
          ctx.strokeStyle = "#fff";
          ctx.lineWidth = 2;
          ctx.stroke();

          ctx.fillStyle = "#92400e";
          ctx.font = "700 11px 'Segoe UI', sans-serif";
          ctx.textAlign = "left";
          ctx.textBaseline = "bottom";
          ctx.fillText(`V${entry.order} -> S${entry.order}`, projected.x + 12, projected.y - 8);

          if (hovered) {
            ctx.strokeStyle = "rgba(245, 158, 11, 0.9)";
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.arc(projected.x, projected.y, 14, 0, Math.PI * 2);
            ctx.stroke();
          }
        }

        if (hovered) {
          ctx.strokeStyle = "rgba(59, 130, 246, 0.8)";
          ctx.lineWidth = 4;
          ctx.beginPath();
          ctx.arc(point.x, point.y, 18, 0, Math.PI * 2);
          ctx.stroke();

          ctx.fillStyle = "#1d4ed8";
          ctx.font = "700 11px 'Segoe UI', sans-serif";
          ctx.textAlign = "left";
          ctx.textBaseline = "bottom";
          ctx.fillText(`V${entry.order}`, point.x + 14, point.y - 12);
        }

        ctx.fillStyle = POINT_KIND_META.visit.color;
        ctx.beginPath();
        ctx.arc(point.x, point.y, 13, 0, Math.PI * 2);
        ctx.fill();
        ctx.font = "700 12px 'Segoe UI', sans-serif";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.lineWidth = 3;
        ctx.strokeStyle = "rgba(0, 0, 0, 0.45)";
        ctx.strokeText(String(entry.order), point.x, point.y);
        ctx.fillStyle = "#fff";
        ctx.fillText(String(entry.order), point.x, point.y);
      });

      const robot = worldToCanvas(current.x, current.y);
      ctx.fillStyle = "#16a34a";
      ctx.beginPath();
      ctx.arc(robot.x, robot.y, 11, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "#1c1917";
      ctx.lineWidth = 4;
      ctx.beginPath();
      ctx.moveTo(robot.x, robot.y);
      ctx.lineTo(robot.x + Math.cos(current.yaw) * 24, robot.y - Math.sin(current.yaw) * 24);
      ctx.stroke();

      raf = window.requestAnimationFrame(loop);
    };

    loop();
    return () => window.cancelAnimationFrame(raf);
  }, [canvasRef, visibleLayers]);

  const handleMouseMove = (event) => {
    if (canvasRef.current) {
      const rect = canvasRef.current.getBoundingClientRect();
      const scaleX = canvasRef.current.width / rect.width;
      const scaleY = canvasRef.current.height / rect.height;
      setCursorPoint(
        canvasToWorld(
          (event.clientX - rect.left) * scaleX,
          (event.clientY - rect.top) * scaleY
        )
      );
    }
    onCanvasMouseMove(event);
  };

  const handleMouseLeave = (event) => {
    setCursorPoint(null);
    onCanvasMouseLeave(event);
  };

  return (
    <main className="h-full min-h-0 min-w-0 flex-1 overflow-hidden bg-slate-100 p-0">
      <div className="flex h-full w-full flex-col overflow-hidden bg-white">
        <div className="flex flex-wrap items-center justify-between gap-3 border-b border-slate-200 px-4 py-3">
          <div className="min-w-0">
            <h1 className="truncate text-base font-bold text-slate-950">Рабочая карта маршрута</h1>
            <div className="truncate text-xs text-slate-500">
              Рабочая область планирования маршрута
            </div>
          </div>
        </div>
        <div className="relative flex min-h-0 flex-1 items-center justify-center p-3 xl:p-4">
          {cursorPoint && (
            <div className="pointer-events-none absolute bottom-5 left-5 z-10 rounded-md border border-slate-200 bg-white/95 px-3 py-2 text-xs font-semibold text-slate-600 shadow-sm">
              x {cursorPoint.x.toFixed(2)}, y {cursorPoint.y.toFixed(2)}
            </div>
          )}
          <canvas
            ref={canvasRef}
            width={CANVAS_WIDTH}
            height={CANVAS_HEIGHT}
            onClick={onCanvasClick}
            onMouseDown={onCanvasMouseDown}
            onMouseMove={handleMouseMove}
            onMouseUp={onCanvasMouseUp}
            onMouseLeave={handleMouseLeave}
            className="h-auto w-auto max-h-full max-w-full cursor-crosshair rounded-lg border border-slate-200 bg-slate-100"
          />
        </div>
      </div>
    </main>
  );
}
