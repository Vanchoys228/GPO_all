import { normalizeAngle } from "../../../lib/dashboardTelemetry";

export const interpolateCanvasTelemetry = (current, target, alpha) => ({
  x: current.x + (target.x - current.x) * alpha,
  y: current.y + (target.y - current.y) * alpha,
  z: current.z + (target.z - current.z) * alpha,
  yaw: current.yaw + normalizeAngle(target.yaw - current.yaw) * alpha,
});
