// Wire protocol types. Single source of truth is the docstring of
// coordination_oru/viz/web_viewer.py — keep the two in sync.

export type Point = [number, number];
export type Ring = Point[];

export interface StaticRobot {
  id: number;
  envelopeID: number;
  path: Point[];
  envelope: Ring[];
}

export interface StaticMessage {
  kind: "static";
  seq: number;
  ts: number;
  title: string;
  world: { size: number; center: Point };
  robots: StaticRobot[];
  footprints: { id: number; ring: Ring }[];
}

export interface RobotState {
  id: number;
  driving: boolean;
  /** [x, y, theta] — the frontend places the static footprint outline here. */
  pose: [number, number, number];
  pathIndex: number;
  pathLength?: number;
  velocity: number;
  criticalPoint: number;
}

export interface CriticalSectionState {
  robot1: number;
  start1: number;
  end1: number;
  robot2: number;
  start2: number;
  end2: number;
}

export interface DependencyState {
  waiting: number;
  driving: number;
  waitingPoint: number;
}

export interface StateMessage {
  kind: "state";
  seq: number;
  ts: number;
  robots: RobotState[];
  criticalSections: CriticalSectionState[];
  dependencies: DependencyState[];
  counts: {
    driving: number;
    parked: number;
    criticalSections: number;
    orders: number;
  };
}

export type ServerMessage = StaticMessage | StateMessage;
