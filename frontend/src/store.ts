import { create } from "zustand";
import type { ServerMessage, StateMessage, StaticMessage } from "@/lib/protocol";

export type ConnectionStatus = "connecting" | "open" | "closed";

interface VizStore {
  connectionStatus: ConnectionStatus;
  staticData: StaticMessage | null;
  state: StateMessage | null;
  setConnectionStatus: (status: ConnectionStatus) => void;
  applyMessage: (msg: ServerMessage) => void;
}

export const useVizStore = create<VizStore>((set) => ({
  connectionStatus: "connecting",
  staticData: null,
  state: null,
  setConnectionStatus: (connectionStatus) => set({ connectionStatus }),
  applyMessage: (msg) => {
    if (msg.kind === "static") set({ staticData: msg });
    else if (msg.kind === "state") set({ state: msg });
  },
}));
