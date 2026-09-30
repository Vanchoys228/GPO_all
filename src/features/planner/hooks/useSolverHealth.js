import { useEffect, useState } from "react";
import { probeNativeSolver } from "../../../lib/routeAlgorithms";

export const useSolverHealth = ({ pollIntervalMs = 2500 } = {}) => {
  const [available, setAvailable] = useState(false);

  useEffect(() => {
    let disposed = false;
    let pollTimer = 0;

    const poll = async () => {
      try {
        const payload = await probeNativeSolver();
        if (!disposed) setAvailable(Boolean(payload?.solverAvailable));
      } catch {
        if (!disposed) setAvailable(false);
      } finally {
        if (!disposed) pollTimer = window.setTimeout(poll, pollIntervalMs);
      }
    };

    void poll();
    return () => {
      disposed = true;
      window.clearTimeout(pollTimer);
    };
  }, [pollIntervalMs]);

  return available;
};
