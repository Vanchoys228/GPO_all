import { useEffect, useRef, useState } from "react";
import { ROUTE_WS_URL } from "../../../lib/runtimeConfig";

export const useRouteSocket = ({ reconnectDelayMs = 1000 } = {}) => {
  const socketRef = useRef(null);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    let disposed = false;
    let reconnectTimer = 0;

    const connect = () => {
      if (disposed) return;
      const socket = new WebSocket(ROUTE_WS_URL);
      socketRef.current = socket;
      socket.onopen = () => setConnected(true);
      socket.onclose = () => {
        setConnected(false);
        if (socketRef.current === socket) socketRef.current = null;
        if (!disposed) reconnectTimer = window.setTimeout(connect, reconnectDelayMs);
      };
      socket.onerror = () => setConnected(false);
    };

    connect();
    return () => {
      disposed = true;
      window.clearTimeout(reconnectTimer);
      setConnected(false);
      socketRef.current?.close();
      socketRef.current = null;
    };
  }, [reconnectDelayMs]);

  return { connected, socketRef };
};
