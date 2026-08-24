import { useEffect, useState } from "react";
import {
  decodeWsData,
  INITIAL_TELEMETRY,
  normalizeTelemetry,
  TELEMETRY_WS_URL,
} from "../../../lib/dashboardTelemetry";

export const useTelemetrySocket = ({ reconnectDelayMs = 1000 } = {}) => {
  const [connected, setConnected] = useState(false);
  const [telemetry, setTelemetry] = useState(INITIAL_TELEMETRY);

  useEffect(() => {
    let disposed = false;
    let reconnectTimer = 0;
    let socket = null;

    const connect = () => {
      if (disposed) return;
      socket = new WebSocket(TELEMETRY_WS_URL);
      socket.onopen = () => setConnected(true);
      socket.onmessage = async (message) => {
        try {
          const payload = JSON.parse(await decodeWsData(message.data));
          setTelemetry((previous) => normalizeTelemetry(payload, previous) || previous);
        } catch {
          // Malformed telemetry should not interrupt the live connection.
        }
      };
      socket.onclose = () => {
        setConnected(false);
        if (!disposed) reconnectTimer = window.setTimeout(connect, reconnectDelayMs);
      };
      socket.onerror = () => setConnected(false);
    };

    connect();
    return () => {
      disposed = true;
      window.clearTimeout(reconnectTimer);
      socket?.close();
    };
  }, [reconnectDelayMs]);

  return { connected, telemetry };
};
