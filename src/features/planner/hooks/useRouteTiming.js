import { useCallback, useEffect, useState } from "react";

const OFF_ROUTE_NAVIGATION_STATUSES = new Set([
  "passing_lidar_gap",
  "tracking_lidar_priority",
  "turning_lidar_priority",
  "reacquired_free_space",
]);

export const isNavigationOffRoute = (navigation) => {
  if (!navigation || typeof navigation !== "object") return false;
  if (navigation.offRouteActive || navigation.avoidanceActive) return true;
  const status = typeof navigation.status === "string" ? navigation.status : "";
  return status.startsWith("avoiding_") || OFF_ROUTE_NAVIGATION_STATUSES.has(status);
};

const createIdleRouteTiming = () => ({
  status: "idle",
  startedAtMs: null,
  actualTimeSec: null,
  seenUnfinished: false,
});

const createIdleOffRouteTiming = () => ({ accumulatedSec: 0, startedAtMs: null });

export const useRouteTiming = (navigation) => {
  const [timing, setTiming] = useState(createIdleRouteTiming);
  const [offRouteTiming, setOffRouteTiming] = useState(createIdleOffRouteTiming);
  const [nowMs, setNowMs] = useState(() => Date.now());
  const offRouteActive = isNavigationOffRoute(navigation);

  useEffect(() => {
    if (timing.status !== "running") return;
    const timer = window.setTimeout(() => {
      const finished = Boolean(navigation?.finished);
      if (!finished && !timing.seenUnfinished) {
        setTiming((previous) =>
          previous.status === "running" ? { ...previous, seenUnfinished: true } : previous
        );
        return;
      }
      if (finished && timing.seenUnfinished && timing.startedAtMs !== null) {
        setTiming({
          status: "finished",
          startedAtMs: timing.startedAtMs,
          actualTimeSec: (Date.now() - timing.startedAtMs) / 1000,
          seenUnfinished: true,
        });
      }
    }, 0);
    return () => window.clearTimeout(timer);
  }, [navigation?.finished, timing.seenUnfinished, timing.startedAtMs, timing.status]);

  useEffect(() => {
    const timer = window.setTimeout(() => {
      const now = Date.now();
      setOffRouteTiming((previous) => {
        if (timing.status !== "running") {
          return previous.startedAtMs !== null
            ? {
                accumulatedSec: previous.accumulatedSec + (now - previous.startedAtMs) / 1000,
                startedAtMs: null,
              }
            : previous;
        }
        if (offRouteActive && previous.startedAtMs === null) {
          return { ...previous, startedAtMs: now };
        }
        if (!offRouteActive && previous.startedAtMs !== null) {
          return {
            accumulatedSec: previous.accumulatedSec + (now - previous.startedAtMs) / 1000,
            startedAtMs: null,
          };
        }
        return previous;
      });
    }, 0);
    return () => window.clearTimeout(timer);
  }, [navigation?.status, offRouteActive, timing.status]);

  useEffect(() => {
    if (timing.status !== "running" && offRouteTiming.startedAtMs === null) return undefined;
    const updateNow = () => setNowMs(Date.now());
    updateNow();
    const timer = window.setInterval(updateNow, 250);
    return () => window.clearInterval(timer);
  }, [offRouteTiming.startedAtMs, timing.status]);

  const start = useCallback(() => {
    setTiming({
      status: "running",
      startedAtMs: Date.now(),
      actualTimeSec: null,
      seenUnfinished: false,
    });
    setOffRouteTiming(createIdleOffRouteTiming());
  }, []);

  const reset = useCallback(() => {
    setTiming(createIdleRouteTiming());
    setOffRouteTiming(createIdleOffRouteTiming());
  }, []);

  const display = {
    status: timing.status,
    actualTimeSec:
      timing.status === "running" && timing.startedAtMs !== null
        ? (nowMs - timing.startedAtMs) / 1000
        : timing.actualTimeSec,
  };
  const telemetryAvoidanceTimeSec = Number(navigation?.avoidanceTimeSec) || 0;
  const localAvoidanceTimeSec =
    offRouteTiming.accumulatedSec +
    (offRouteTiming.startedAtMs !== null
      ? (nowMs - offRouteTiming.startedAtMs) / 1000
      : 0);

  return {
    avoidanceTimeSec: Math.max(telemetryAvoidanceTimeSec, localAvoidanceTimeSec),
    display,
    offRouteActive,
    reset,
    start,
  };
};
