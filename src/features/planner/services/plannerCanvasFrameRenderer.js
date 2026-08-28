import { drawPlannerBackground } from "./plannerCanvasRenderer";
import { renderPlannerPointLayer } from "./plannerCanvasPointRenderer";
import { renderPlannerRouteLayer } from "./plannerCanvasRouteRenderer";
import { renderPlannerSensorLayers } from "./plannerCanvasSensorRenderer";
import { renderPlannerZoneLayer } from "./plannerCanvasZoneRenderer";

export const renderPlannerCanvasFrame = (ctx, state, telemetry) => {
  drawPlannerBackground(ctx, state.surfaceZones);
  renderPlannerSensorLayers(ctx, state);
  renderPlannerZoneLayer(ctx, state.zoneEntries);
  renderPlannerRouteLayer(ctx, state);
  renderPlannerPointLayer(ctx, state, telemetry);
};
