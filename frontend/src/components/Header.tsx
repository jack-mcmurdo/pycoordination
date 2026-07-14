import { useVizStore } from "@/store";
import { Button } from "@/components/ui/button";
import { Moon, Sun } from "lucide-react";

interface HeaderProps {
  dark: boolean;
  onToggleDark: () => void;
}

const STATUS_LABEL: Record<string, string> = {
  connecting: "Connecting…",
  open: "Connected",
  closed: "Disconnected",
};

const STATUS_DOT: Record<string, string> = {
  connecting: "bg-yellow-500",
  open: "bg-green-500",
  closed: "bg-red-500",
};

export function Header({ dark, onToggleDark }: HeaderProps) {
  const status = useVizStore((s) => s.connectionStatus);
  const title = useVizStore((s) => s.staticData?.title);

  return (
    <header className="flex items-center gap-3 border-b px-3 py-2">
      <span className="font-semibold">{title ?? "coordination_oru"}</span>
      <div className="flex items-center gap-1.5 text-xs text-muted-foreground">
        <span className={`size-2 rounded-full ${STATUS_DOT[status]}`} />
        {STATUS_LABEL[status]}
      </div>
      <div className="flex-1" />
      <Button size="icon" variant="ghost" className="size-8" onClick={onToggleDark}>
        {dark ? <Sun className="size-4" /> : <Moon className="size-4" />}
      </Button>
    </header>
  );
}
