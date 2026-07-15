import { Badge } from "@/components/ui/badge";
import { robotColor } from "@/lib/robot-colors";
import { useVizStore } from "@/store";

export function StatusBar() {
  const state = useVizStore((s) => s.state);
  if (!state) return null;

  const { counts } = state;
  const finished = state.robots.length > 0 && counts.driving === 0;

  return (
    <footer className="flex flex-wrap items-center gap-x-4 gap-y-1 border-t px-3 py-2 text-xs">
      <div className="flex items-center gap-1.5">
        <Badge variant="secondary">driving {counts.driving}</Badge>
        <Badge variant="secondary">parked {counts.parked}</Badge>
        <Badge variant={counts.criticalSections > 0 ? "destructive" : "secondary"}>
          CSes {counts.criticalSections}
        </Badge>
        <Badge variant="secondary">orders {counts.orders}</Badge>
        {state.deadlocked && (
          <Badge variant="destructive" className="animate-pulse">
            Deadlocked!
          </Badge>
        )}
        {finished && <Badge>all parked</Badge>}
      </div>
      <div className="flex flex-wrap items-center gap-x-3 gap-y-1 font-mono text-muted-foreground">
        {state.robots.map((robot) => (
          <span key={robot.id} className="flex items-center gap-1">
            <span
              className="size-2 rounded-full"
              style={{ backgroundColor: robotColor(robot.id) }}
            />
            R{robot.id}
            {robot.driving && robot.pathLength !== undefined && (
              <> {robot.pathIndex + 1}/{robot.pathLength}</>
            )}
            {" "}v={robot.velocity.toFixed(2)}
            {robot.criticalPoint >= 0 && <> cp={robot.criticalPoint}</>}
          </span>
        ))}
      </div>
    </footer>
  );
}
