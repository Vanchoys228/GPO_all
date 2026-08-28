import { interpolateCanvasTelemetry } from "../model/plannerCanvasTelemetry";
import { renderPlannerCanvasFrame } from "./plannerCanvasFrameRenderer";

export const startPlannerCanvasRenderLoop = ({
  canvasRef,
  telemetryTargetRef,
  telemetryRenderRef,
  drawStateRef,
  scheduleFrame = window.requestAnimationFrame,
  cancelFrame = window.cancelAnimationFrame,
}) => {
  let frameId = 0;

  const render = () => {
    const canvas = canvasRef.current;
    const ctx = canvas?.getContext("2d");
    if (ctx) {
      telemetryRenderRef.current = interpolateCanvasTelemetry(
        telemetryRenderRef.current,
        telemetryTargetRef.current,
        0.35
      );
      renderPlannerCanvasFrame(ctx, drawStateRef.current, telemetryRenderRef.current);
    }
    frameId = scheduleFrame(render);
  };

  render();
  return () => cancelFrame(frameId);
};
