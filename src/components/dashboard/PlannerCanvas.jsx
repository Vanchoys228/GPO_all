import { useEffect, useRef } from "react";
import { CANVAS_HEIGHT, CANVAS_WIDTH } from "../../lib/zonePlanner";
import { INITIAL_TELEMETRY } from "../../lib/dashboardTelemetry";
import { createPlannerCanvasDrawState } from "../../features/planner/model/plannerCanvasDrawState";
import { startPlannerCanvasRenderLoop } from "../../features/planner/services/plannerCanvasRenderLoop";

const INITIAL_DRAW_STATE = {
  visitEntries: [],
  chargeEntries: [],
  plannedVisitEntryMap: new Map(),
  zoneEntries: [],
  surfaceZones: [],
  optimizedRoute: [],
  obstacleTrace: [],
  obstacleMap: INITIAL_TELEMETRY.obstacleMap,
  cameraMap: INITIAL_TELEMETRY.cameraMap,
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
    drawStateRef.current = createPlannerCanvasDrawState({
      plannerModel,
      optimizedRoute,
      hoveredPointIndex,
      telemetry,
      fallbackObstacleMap: INITIAL_TELEMETRY.obstacleMap,
      fallbackCameraMap: INITIAL_TELEMETRY.cameraMap,
    });
  }, [hoveredPointIndex, optimizedRoute, plannerModel, telemetry]);

  useEffect(
    () =>
      startPlannerCanvasRenderLoop({
        canvasRef,
        telemetryTargetRef,
        telemetryRenderRef,
        drawStateRef,
      }),
    [canvasRef]
  );

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
