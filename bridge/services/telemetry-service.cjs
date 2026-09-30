const createTelemetryService = ({ now = () => new Date().toISOString() } = {}) => {
  let eventSequence = 0;
  let latest = null;

  const publish = async (payload) => {
    const { createTelemetryEvent, unwrapTelemetryEvent } = await import("../../shared/contracts/index.js");
    const state = payload?.type === "telemetry.event"
      ? unwrapTelemetryEvent(payload)
      : payload;
    latest = state;
    eventSequence += 1;
    return JSON.stringify(createTelemetryEvent({
      source: "telemetry-service",
      requestId: `telemetry-${eventSequence}`,
      timestamp: now(),
      payload: state,
    }));
  };

  return { getLatest: () => latest, publish };
};

module.exports = { createTelemetryService };
