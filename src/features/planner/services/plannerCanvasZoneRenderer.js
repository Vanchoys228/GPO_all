import { worldToCanvas } from "../../../lib/zonePlanner";
import { drawDiamond } from "./plannerCanvasRenderer";

export const renderPlannerZoneLayer = (ctx, zoneEntries) => {
  zoneEntries.forEach((zone) => {
    if (zone.points.length > 1) {
      ctx.setLineDash([10, 8]);
      ctx.strokeStyle = zone.color.stroke;
      ctx.lineWidth = 3;
      ctx.shadowColor = "rgba(15, 23, 42, 0.18)";
      ctx.shadowBlur = 7;
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
      ctx.shadowColor = "transparent";
      ctx.shadowBlur = 0;
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
      ctx.fillText(zone.closed ? `${zone.name} (замкнута)` : `${zone.name} (открыта)`, anchor.x + 14, anchor.y - 10);
    }
  });
};
