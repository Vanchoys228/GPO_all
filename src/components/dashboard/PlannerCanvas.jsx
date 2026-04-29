import { useEffect, useRef } from "react";
import {
  CANVAS_HEIGHT,
  CANVAS_WIDTH,
  POINT_KIND_META,
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
  routeBlocked: false,
  hoveredPointIndex: null,
};

export default function PlannerCanvas({
  canvasRef,
  plannerModel,
  optimizedRoute,
  hoveredPointIndex,
  telemetry,
  onCanvasClick,
  onCanvasMouseDown,
  onCanvasMouseMove,
  onCanvasMouseUp,
  onCanvasMouseLeave,
}) {
  const telemetryTargetRef = useRef({ ...INITIAL_TELEMETRY });
  const telemetryRenderRef = useRef({ ...INITIAL_TELEMETRY });
  const drawStateRef = useRef(INITIAL_DRAW_STATE);

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

      drawPlannerBackground(ctx, state.surfaceZones);

      state.zoneEntries.forEach((zone) => {
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

      if (state.obstacleTrace.length) {
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

      if (state.optimizedRoute.length > 1) {
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

      state.chargeEntries.forEach((entry) => {
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

      state.visitEntries.forEach((entry) => {
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
  }, [canvasRef]);

  return (
    <main className="flex-1 overflow-auto bg-stone-200 p-4">
      <div className="mx-auto max-w-[1380px]">
        <div className="relative rounded-[28px] border border-stone-300 bg-white/80 p-3 shadow-xl lg:p-4">
          <canvas
            ref={canvasRef}
            width={CANVAS_WIDTH}
            height={CANVAS_HEIGHT}
            onClick={onCanvasClick}
            onMouseDown={onCanvasMouseDown}
            onMouseMove={onCanvasMouseMove}
            onMouseUp={onCanvasMouseUp}
            onMouseLeave={onCanvasMouseLeave}
            className="w-full h-auto rounded-[24px] border border-stone-200 bg-stone-100 cursor-crosshair"
          />
        </div>
      </div>
    </main>
  );
}
