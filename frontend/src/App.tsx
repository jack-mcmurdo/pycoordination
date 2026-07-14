import { Header } from "@/components/Header";
import { StatusBar } from "@/components/StatusBar";
import { WorldView } from "@/components/WorldView";
import { useDarkMode } from "@/lib/dark-mode";
import { useLiveConnection } from "@/lib/ws";

export default function App() {
  useLiveConnection();
  const [dark, toggleDark] = useDarkMode();

  return (
    <div className="flex h-screen flex-col">
      <Header dark={dark} onToggleDark={toggleDark} />
      <main className="min-h-0 flex-1">
        <WorldView />
      </main>
      <StatusBar />
    </div>
  );
}
