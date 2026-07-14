import { useEffect } from "react";
import type { ServerMessage } from "@/lib/protocol";
import { useVizStore } from "@/store";

const MAX_BACKOFF_MS = 10_000;
const INITIAL_BACKOFF_MS = 250;

function wsUrl(): string {
  const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
  return `${proto}//${window.location.host}/ws`;
}

/** Connects to the server's /ws endpoint once for the app's lifetime,
 * reconnecting with exponential backoff on drop, and dispatches every
 * message into the zustand store. */
export function useLiveConnection(): void {
  useEffect(() => {
    let socket: WebSocket | null = null;
    let backoff = INITIAL_BACKOFF_MS;
    let retryTimer: ReturnType<typeof setTimeout> | null = null;
    let stopped = false;

    const applyMessage = useVizStore.getState().applyMessage;
    const setConnectionStatus = useVizStore.getState().setConnectionStatus;

    function connect() {
      if (stopped) return;
      setConnectionStatus("connecting");
      socket = new WebSocket(wsUrl());

      socket.onopen = () => {
        backoff = INITIAL_BACKOFF_MS;
        setConnectionStatus("open");
      };

      socket.onmessage = (event: MessageEvent<string>) => {
        const msg = JSON.parse(event.data) as ServerMessage;
        applyMessage(msg);
      };

      socket.onclose = () => {
        setConnectionStatus("closed");
        if (stopped) return;
        retryTimer = setTimeout(connect, backoff);
        backoff = Math.min(backoff * 2, MAX_BACKOFF_MS);
      };

      socket.onerror = () => {
        socket?.close();
      };
    }

    connect();

    return () => {
      stopped = true;
      if (retryTimer) clearTimeout(retryTimer);
      socket?.close();
    };
  }, []);
}
