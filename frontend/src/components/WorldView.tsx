import { memo, useEffect, useMemo, useRef, useState } from "react";
import { Maximize } from "lucide-react";
import { Button } from "@/components/ui/button";
import { robotColor } from "@/lib/robot-colors";
import { sendPostGoal } from "@/lib/ws";
import type { CriticalSectionState, Point, StaticMessage } from "@/lib/protocol";
import { useVizStore } from "@/store";

const CS_COLOR = "rgb(240, 90, 90)";
const ARROW_COLOR = "rgb(240, 90, 90)";
// slightly longer than one 20 Hz server tick so consecutive poses blend
const POSE_TRANSITION = "transform 120ms linear";

// World is y-up, SVG is y-down: negate y on every coordinate.
function toPoints(points: Point[]): string {
  return points.map(([x, y]) => `${x},${-y}`).join(" ");
}

const RAD_TO_DEG = 180 / Math.PI;

function r3(v: number): number {
  return Math.round(v * 1000) / 1000;
}

interface ViewBox {
  x: number;
  y: number;
  w: number;
  h: number;
}

/** Paths and swept envelopes: only re-rendered when a new static message
 * arrives, not on every 20 Hz state tick (large point strings). */
const StaticLayers = memo(function StaticLayers({
  staticData,
  stroke,
}: {
  staticData: StaticMessage;
  stroke: number;
}) {
  return (
    <>
      {staticData.robots.map((robot) =>
        robot.envelope.map((ring, i) => (
          <polygon
            key={`env-${robot.id}-${i}`}
            points={toPoints(ring)}
            fill={robotColor(robot.id)}
            fillOpacity={0.11}
          />
        )),
      )}
      {staticData.robots.map((robot) => (
        <polyline
          key={`path-${robot.id}`}
          points={toPoints(robot.path)}
          fill="none"
          stroke="currentColor"
          strokeOpacity={0.3}
          strokeWidth={stroke}
        />
      ))}
    </>
  );
});

/** Critical-section highlights: re-rendered only when the CS set changes
 * (the index ranges are static per CS, robots moving doesn't affect them). */
const CriticalSectionLayer = memo(
  function CriticalSectionLayer({
    criticalSections,
    paths,
    stroke,
  }: {
    criticalSections: CriticalSectionState[];
    paths: Map<number, Point[]>;
    stroke: number;
  }) {
    return (
      <>
        {criticalSections.map((cs, i) => {
          const sides = [
            { robot: cs.robot1, start: cs.start1, end: cs.end1 },
            { robot: cs.robot2, start: cs.start2, end: cs.end2 },
          ];
          return sides.map(({ robot, start, end }, side) => {
            const path = paths.get(robot);
            if (!path || end <= start) return null;
            return (
              <polyline
                key={`cs-${i}-${side}`}
                points={toPoints(path.slice(start, end + 1))}
                fill="none"
                stroke={CS_COLOR}
                strokeOpacity={0.8}
                strokeWidth={stroke * 3}
              />
            );
          });
        })}
      </>
    );
  },
  (prev, next) =>
    prev.stroke === next.stroke &&
    prev.paths === next.paths &&
    JSON.stringify(prev.criticalSections) === JSON.stringify(next.criticalSections),
);

export function WorldView() {
  const staticData = useVizStore((s) => s.staticData);
  const state = useVizStore((s) => s.state);
  const selectedRobot = useVizStore((s) => s.selectedRobot);
  const setSelectedRobot = useVizStore((s) => s.setSelectedRobot);

  const worldViewBox = useMemo<ViewBox>(() => {
    if (!staticData) return { x: -10, y: -10, w: 20, h: 20 };
    const { size, center } = staticData.world;
    return { x: center[0] - size / 2, y: -center[1] - size / 2, w: size, h: size };
  }, [staticData]);

  const [viewBox, setViewBox] = useState<ViewBox>(worldViewBox);
  const framedOnce = useRef(false);
  useEffect(() => {
    if (staticData && !framedOnce.current) {
      framedOnce.current = true;
      setViewBox(worldViewBox);
    }
  }, [staticData, worldViewBox]);

  const paths = useMemo(() => {
    const byRobot = new Map<number, Point[]>();
    for (const robot of staticData?.robots ?? []) byRobot.set(robot.id, robot.path);
    return byRobot;
  }, [staticData]);

  const outlines = useMemo(() => {
    const byRobot = new Map<number, string>();
    for (const fp of staticData?.footprints ?? []) byRobot.set(fp.id, toPoints(fp.ring));
    return byRobot;
  }, [staticData]);

  const circumradii = useMemo(() => {
    const byRobot = new Map<number, number>();
    for (const fp of staticData?.footprints ?? [])
      byRobot.set(fp.id, Math.max(...fp.ring.map(([x, y]) => Math.hypot(x, y))));
    return byRobot;
  }, [staticData]);

  const poses = useMemo(() => {
    const byRobot = new Map<number, [number, number, number]>();
    for (const robot of state?.robots ?? []) byRobot.set(robot.id, robot.pose);
    return byRobot;
  }, [state]);

  const interactive = staticData?.interactive === true;

  const drag = useRef<{ x: number; y: number; captured: boolean } | null>(null);
  const DRAG_THRESHOLD_PX = 4;
  // goal-posting drag (interactive mode, robot selected): anchor = world
  // point of pointer-down, startClient = screen point for the 8 px test
  const goalDrag = useRef<{
    anchor: { x: number; y: number };
    startClient: { x: number; y: number };
  } | null>(null);
  const [goalPreview, setGoalPreview] = useState<{ x: number; y: number } | null>(null);

  // Undo the y-negation; exact under preserveAspectRatio letterboxing.
  function clientToWorld(e: React.PointerEvent<SVGSVGElement>): { x: number; y: number } {
    const pt = new DOMPoint(e.clientX, e.clientY).matrixTransform(
      e.currentTarget.getScreenCTM()!.inverse(),
    );
    return { x: pt.x, y: -pt.y };
  }

  useEffect(() => {
    if (!interactive) return;
    function onKeyDown(e: KeyboardEvent) {
      if (e.key === "Escape") {
        goalDrag.current = null;
        setGoalPreview(null);
        setSelectedRobot(null);
      }
    }
    window.addEventListener("keydown", onKeyDown);
    return () => window.removeEventListener("keydown", onKeyDown);
  }, [interactive, setSelectedRobot]);

  function onWheel(e: React.WheelEvent<SVGSVGElement>) {
    e.preventDefault();
    const rect = e.currentTarget.getBoundingClientRect();
    const px = (e.clientX - rect.left) / rect.width;
    const py = (e.clientY - rect.top) / rect.height;
    const factor = e.deltaY > 0 ? 1.1 : 1 / 1.1;
    setViewBox((vb) => {
      const cx = vb.x + px * vb.w;
      const cy = vb.y + py * vb.h;
      const w = vb.w * factor;
      const h = vb.h * factor;
      return { x: cx - px * w, y: cy - py * h, w, h };
    });
  }

  function onPointerDown(e: React.PointerEvent<SVGSVGElement>) {
    if (interactive && selectedRobot !== null) {
      const anchor = clientToWorld(e);
      goalDrag.current = { anchor, startClient: { x: e.clientX, y: e.clientY } };
      setGoalPreview(anchor);
      e.currentTarget.setPointerCapture(e.pointerId);
      return;
    }
    drag.current = { x: e.clientX, y: e.clientY, captured: false };
  }

  function onPointerMove(e: React.PointerEvent<SVGSVGElement>) {
    if (goalDrag.current) {
      setGoalPreview(clientToWorld(e));
      return;
    }
    if (!drag.current) return;
    const dxPx = e.clientX - drag.current.x;
    const dyPx = e.clientY - drag.current.y;
    if (!drag.current.captured && Math.hypot(dxPx, dyPx) < DRAG_THRESHOLD_PX) return;
    if (!drag.current.captured) {
      e.currentTarget.setPointerCapture(e.pointerId);
      drag.current.captured = true;
    }
    const rect = e.currentTarget.getBoundingClientRect();
    const dx = (dxPx / rect.width) * viewBox.w;
    const dy = (dyPx / rect.height) * viewBox.h;
    drag.current.x = e.clientX;
    drag.current.y = e.clientY;
    setViewBox((vb) => ({ ...vb, x: vb.x - dx, y: vb.y - dy }));
  }

  function onPointerUp(e: React.PointerEvent<SVGSVGElement>) {
    if (e.currentTarget.hasPointerCapture(e.pointerId)) {
      e.currentTarget.releasePointerCapture(e.pointerId);
    }
    const goal = goalDrag.current;
    if (goal && selectedRobot !== null) {
      const cur = clientToWorld(e);
      const { anchor, startClient } = goal;
      const movedPx = Math.hypot(e.clientX - startClient.x, e.clientY - startClient.y);
      let theta: number;
      if (movedPx >= 8) {
        theta = Math.atan2(cur.y - anchor.y, cur.x - anchor.x);
      } else {
        // plain click: aim the goal heading from the robot toward the click
        const pose = poses.get(selectedRobot);
        theta = pose ? Math.atan2(anchor.y - pose[1], anchor.x - pose[0]) : 0;
      }
      sendPostGoal({
        kind: "postGoal",
        robot: selectedRobot,
        goal: [r3(anchor.x), r3(anchor.y), r3(theta)],
      });
    }
    goalDrag.current = null;
    setGoalPreview(null);
    drag.current = null;
  }

  if (!staticData) {
    return (
      <div className="flex h-full items-center justify-center text-muted-foreground">
        Waiting for the coordinator…
      </div>
    );
  }

  const stroke = viewBox.w / 500;
  const labelSize = viewBox.w / 45;
  const map = staticData.map;
  const goalAnchor = goalDrag.current?.anchor ?? null;

  return (
    <div className="relative h-full w-full">
      <svg
        width="100%"
        height="100%"
        viewBox={`${viewBox.x} ${viewBox.y} ${viewBox.w} ${viewBox.h}`}
        preserveAspectRatio="xMidYMid meet"
        onWheel={onWheel}
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onPointerCancel={onPointerUp}
        className={
          interactive && selectedRobot !== null
            ? "cursor-crosshair touch-none"
            : "cursor-grab touch-none active:cursor-grabbing"
        }
      >
        <defs>
          <marker
            id="dep-arrow"
            viewBox="0 0 10 10"
            refX="9"
            refY="5"
            markerWidth="7"
            markerHeight="7"
            orient="auto-start-reverse"
          >
            <path d="M 0 0 L 10 5 L 0 10 z" fill={ARROW_COLOR} />
          </marker>
          {/* same arrow, tinted by the line it terminates */}
          <marker
            id="goal-arrow"
            viewBox="0 0 10 10"
            refX="9"
            refY="5"
            markerWidth="7"
            markerHeight="7"
            orient="auto-start-reverse"
          >
            <path d="M 0 0 L 10 5 L 0 10 z" fill="context-stroke" />
          </marker>
        </defs>

        {map && (
          <image
            href={map.dataUri}
            x={map.origin[0]}
            y={-(map.origin[1] + map.height * map.resolution)}
            width={map.width * map.resolution}
            height={map.height * map.resolution}
            preserveAspectRatio="none"
            opacity={0.85}
            style={{ imageRendering: "pixelated" }}
          />
        )}

        <StaticLayers staticData={staticData} stroke={stroke} />
        <CriticalSectionLayer
          criticalSections={state?.criticalSections ?? []}
          paths={paths}
          stroke={stroke}
        />

        {/* dependency arrows: yielder → leader */}
        {state?.dependencies.map((dep, i) => {
          const from = poses.get(dep.waiting);
          const to = poses.get(dep.driving);
          if (!from || !to) return null;
          return (
            <line
              key={`dep-${i}`}
              x1={from[0]}
              y1={-from[1]}
              x2={to[0]}
              y2={-to[1]}
              stroke={ARROW_COLOR}
              strokeWidth={stroke * 1.5}
              strokeDasharray={`${stroke * 6} ${stroke * 3}`}
              markerEnd="url(#dep-arrow)"
              style={{ transition: POSE_TRANSITION }}
            />
          );
        })}

        {/* footprints: static outline placed at the live pose; the CSS
            transition interpolates between 20 Hz server ticks */}
        {state?.robots.map((robot) => {
          const outline = outlines.get(robot.id);
          if (!outline) return null;
          const [x, y, theta] = robot.pose;
          const selected = interactive && selectedRobot === robot.id;
          const circumradius = circumradii.get(robot.id) ?? 0;
          return (
            <g
              key={`robot-${robot.id}`}
              className={interactive ? "cursor-pointer" : undefined}
              onPointerDown={interactive ? (e) => e.stopPropagation() : undefined}
              onClick={
                interactive
                  ? (e) => {
                      e.stopPropagation();
                      setSelectedRobot(robot.id);
                    }
                  : undefined
              }
              style={{
                transform: `translate(${x}px, ${-y}px) rotate(${-theta * RAD_TO_DEG}deg)`,
                transition: POSE_TRANSITION,
              }}
            >
              <polygon
                points={outline}
                fill={robotColor(robot.id)}
                fillOpacity={robot.driving ? 0.9 : 0.45}
                stroke={robotColor(robot.id)}
                strokeWidth={stroke / 2}
              />
              {selected && (
                <circle
                  r={circumradius * 1.4}
                  fill="none"
                  stroke={robotColor(robot.id)}
                  strokeDasharray={stroke * 4}
                  strokeWidth={stroke}
                />
              )}
            </g>
          );
        })}

        {/* labels: translate-only so text stays upright */}
        {state?.robots.map((robot) => (
          <text
            key={`label-${robot.id}`}
            textAnchor="middle"
            dominantBaseline="central"
            fontSize={labelSize}
            fill="rgb(10, 10, 10)"
            className="pointer-events-none select-none"
            style={{
              transform: `translate(${robot.pose[0]}px, ${-robot.pose[1]}px)`,
              transition: POSE_TRANSITION,
            }}
          >
            R{robot.id}
          </text>
        ))}

        {/* live preview of the goal pose being dragged out */}
        {goalAnchor && goalPreview && selectedRobot !== null && (
          <g className="pointer-events-none">
            <circle
              cx={goalAnchor.x}
              cy={-goalAnchor.y}
              r={stroke * 3}
              fill={robotColor(selectedRobot)}
            />
            {(goalPreview.x !== goalAnchor.x || goalPreview.y !== goalAnchor.y) && (
              <line
                x1={goalAnchor.x}
                y1={-goalAnchor.y}
                x2={goalPreview.x}
                y2={-goalPreview.y}
                stroke={robotColor(selectedRobot)}
                strokeWidth={stroke * 1.5}
                strokeDasharray={`${stroke * 6} ${stroke * 3}`}
                markerEnd="url(#goal-arrow)"
              />
            )}
          </g>
        )}
      </svg>
      <Button
        size="icon"
        variant="outline"
        className="absolute top-2 right-2 size-8"
        title="Reset view"
        onClick={() => setViewBox(worldViewBox)}
      >
        <Maximize className="size-4" />
      </Button>
      {interactive && selectedRobot !== null && (
        <div className="pointer-events-none absolute bottom-2 left-1/2 -translate-x-1/2 rounded bg-background/80 px-3 py-1 text-sm text-muted-foreground">
          Robot {selectedRobot} — press-drag-release to send a goal pose · Esc to cancel
        </div>
      )}
    </div>
  );
}
