import { create } from "zustand";
import type { ServerMessage, StateMessage, StaticMessage } from "@/lib/protocol";

export type ConnectionStatus = "connecting" | "open" | "closed";

interface VizStore {
  connectionStatus: ConnectionStatus;
  staticData: StaticMessage | null;
  state: StateMessage | null;
  /** Robot currently selected for goal posting (interactive mode). */
  selectedRobot: number | null;
  setConnectionStatus: (status: ConnectionStatus) => void;
  setSelectedRobot: (id: number | null) => void;
  applyMessage: (msg: ServerMessage) => void;
}

export const useVizStore = create<VizStore>((set) => ({
  connectionStatus: "connecting",
  staticData: null,
  state: null,
  selectedRobot: null,
  setConnectionStatus: (connectionStatus) => set({ connectionStatus }),
  setSelectedRobot: (selectedRobot) => set({ selectedRobot }),
  applyMessage: (msg) => {
    if (msg.kind === "static") set({ staticData: msg });
    else if (msg.kind === "state") set({ state: msg });
  },
}));
