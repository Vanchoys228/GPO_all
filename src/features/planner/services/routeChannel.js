import { ROUTE_WS_URL } from "../../../lib/dashboardTelemetry";

export const sendRouteChannelPayload = (
  routeSocketRef,
  payload,
  { onSent, onError } = {}
) => {
  const text = JSON.stringify(payload);
  const socket = routeSocketRef.current;
  if (socket?.readyState === WebSocket.OPEN) {
    socket.send(text);
    onSent?.();
    return;
  }

  const temporarySocket = new WebSocket(ROUTE_WS_URL);
  let settled = false;
  const closeTemporarySocket = () => {
    try {
      temporarySocket.close();
    } catch {
      // Ignore close failures after a completed send.
    }
  };

  temporarySocket.onopen = () => {
    if (settled) return;
    settled = true;
    temporarySocket.send(text);
    onSent?.();
    window.setTimeout(closeTemporarySocket, 80);
  };
  temporarySocket.onerror = () => {
    if (settled) return;
    settled = true;
    onError?.();
    closeTemporarySocket();
  };
  temporarySocket.onclose = () => {
    if (settled) return;
    settled = true;
    onError?.();
  };
};
