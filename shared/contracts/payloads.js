export const CONTRACT_TYPES = Object.freeze({
  ROUTE: "route.command",
  ROBOT_STATE: "robot.state",
  SENSOR_FRAME: "sensor.frame",
  MOTION_COMMAND: "motion.command",
  SERVICE_ERROR: "service.error",
});

const finite = (value) => typeof value === "number" && Number.isFinite(value);
const object = (value) => value && typeof value === "object" && !Array.isArray(value);

export const validatePayload = (type, payload) => {
  if (!object(payload)) return { valid: false, error: "invalid_contract_payload" };
  if (type === CONTRACT_TYPES.MOTION_COMMAND && (!finite(payload.linearSpeed) || !finite(payload.angularSpeed))) {
    return { valid: false, error: "invalid_motion_command" };
  }
  if (type === CONTRACT_TYPES.SENSOR_FRAME && !object(payload.pose)) {
    return { valid: false, error: "invalid_sensor_frame" };
  }
  if (type === CONTRACT_TYPES.SERVICE_ERROR && (typeof payload.code !== "string" || !payload.code)) {
    return { valid: false, error: "invalid_service_error" };
  }
  return { valid: true };
};
