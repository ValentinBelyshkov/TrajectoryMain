import { useEffect, useRef } from "react";

export interface UseWebSocketOptions {
  onMessage?: (event: MessageEvent) => void;
  onOpen?: (ws: WebSocket) => void;
  reconnect?: boolean;
  reconnectDelay?: number;
  enabled?: boolean;
}

/**
 * Generic WebSocket connection hook.
 * The latest `onMessage`/`onOpen` callbacks are kept in refs so the connection
 * is not torn down when the parent re-renders. Optionally auto-reconnects.
 */
export function useWebSocket(
  url: string | null,
  options: UseWebSocketOptions = {},
) {
  const {
    onMessage,
    onOpen,
    reconnect = false,
    reconnectDelay = 3000,
    enabled = true,
  } = options;

  const wsRef = useRef<WebSocket | null>(null);
  const onMessageRef = useRef(onMessage);
  const onOpenRef = useRef(onOpen);
  onMessageRef.current = onMessage;
  onOpenRef.current = onOpen;

  useEffect(() => {
    if (!enabled || !url) return;

    let closedByUs = false;
    let reconnectTimer: ReturnType<typeof setTimeout> | null = null;

    const connect = () => {
      const ws = new WebSocket(url);
      wsRef.current = ws;

      ws.onopen = () => onOpenRef.current?.(ws);
      ws.onmessage = (event) => onMessageRef.current?.(event);
      ws.onerror = (e) => console.error("WebSocket error:", e);
      ws.onclose = () => {
        if (reconnect && !closedByUs) {
          reconnectTimer = setTimeout(connect, reconnectDelay);
        }
      };
    };

    connect();

    return () => {
      closedByUs = true;
      if (reconnectTimer) clearTimeout(reconnectTimer);
      wsRef.current?.close();
      wsRef.current = null;
    };
  }, [url, enabled, reconnect, reconnectDelay]);

  return wsRef;
}
