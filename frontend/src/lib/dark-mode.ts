import { useEffect, useState } from "react";

const STORAGE_KEY = "coordination-oru-viz-theme";

function initialDark(): boolean {
  const stored = localStorage.getItem(STORAGE_KEY);
  if (stored) return stored === "dark";
  return window.matchMedia("(prefers-color-scheme: dark)").matches;
}

export function useDarkMode(): [boolean, () => void] {
  const [dark, setDark] = useState(initialDark);

  useEffect(() => {
    document.documentElement.classList.toggle("dark", dark);
    localStorage.setItem(STORAGE_KEY, dark ? "dark" : "light");
  }, [dark]);

  return [dark, () => setDark((d) => !d)];
}
