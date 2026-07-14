import { useEffect, useMemo, useRef, useState } from "react";
import { Maximize } from "lucide-react";
import { Button } from "@/components/ui/button";
import { robotColor } from "@/lib/robot-colors";
import type { Point } from "@/lib/protocol";
import { useVizStore } from "@/store";

const CS_COLOR = "rgb(240, 90, 90)";

// World is y-up, SVG is y-down: negate y on every coordinate.
function toPoints(points: Point[]): string {
  return points.map(([x, y]) => `${x},${-y}`).join(" ");
}

function centroid(points: Point[]): Point {
  let cx = 0;
  let cy = 0;
  for (const [x, y] of points) {
    cx += x;
    cy += y;
  }
  return [cx / points.length, cy / points.length];
}

interface ViewBox {
  x: number;
  y: number;
  w: number;
  h: number;
}

export function WorldView() {
  const staticData = useVizStore((s) => s.staticData);
  const state = useVizStore((s) => s.state);

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

  const drag = useRef<{ x: number; y: number; captured: boolean } | null>(null);
  const DRAG_THRESHOLD_PX = 4;

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
    drag.current = { x: e.clientX, y: e.clientY, captured: false };
  }

  function onPointerMove(e: React.PointerEvent<SVGSVGElement>) {
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
        className="cursor-grab touch-none active:cursor-grabbing"
      >
        {/* 1. swept envelopes — very faint fill */}
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

        {/* 2. paths — faint polylines */}
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

        {/* 3. critical-section index ranges highlighted on each path */}
        {state?.criticalSections.map((cs, i) => {
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

        {/* 4. current footprints + labels */}
        {state?.robots.map((robot) => {
          const [cx, cy] = centroid(robot.footprint);
          return (
            <g key={`robot-${robot.id}`}>
              <polygon
                points={toPoints(robot.footprint)}
                fill={robotColor(robot.id)}
                fillOpacity={robot.driving ? 0.9 : 0.45}
                stroke={robotColor(robot.id)}
                strokeWidth={stroke / 2}
              />
              <text
                x={cx}
                y={-cy}
                textAnchor="middle"
                dominantBaseline="central"
                fontSize={labelSize}
                fill="rgb(10, 10, 10)"
                className="pointer-events-none select-none"
              >
                R{robot.id}
              </text>
            </g>
          );
        })}
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
    </div>
  );
}
