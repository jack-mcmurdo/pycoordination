// Same per-robot palette as the pyglet viewer (viz/pyglet_viewer.py).
const PALETTE = [
  "rgb(255, 99, 71)", // tomato
  "rgb(51, 168, 255)", // sky blue
  "rgb(255, 195, 51)", // gold
  "rgb(130, 255, 51)", // lime
  "rgb(200, 51, 255)", // violet
  "rgb(51, 255, 200)", // mint
];

export function robotColor(robotID: number): string {
  return PALETTE[(((robotID - 1) % PALETTE.length) + PALETTE.length) % PALETTE.length];
}
