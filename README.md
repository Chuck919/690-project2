# E-Puck Maze Controller

## Overview

This controller runs in two phases across two separate simulation runs:

1. **Run 1 — Mapping**: The robot explores the maze using DFS, maps all walls, and saves the map to `maze_map.json`.
2. **Run 2 — Speed Run**: The robot loads the saved map and drives directly to the green wall at full speed.

---

## How to Run

### Run 1: Mapping

1. **Delete `maze_map.json`** if it exists in the same folder as this controller (from a previous run).
2. Start the simulation in Webots.
3. The robot will explore the maze automatically. Watch the console for progress:
   ```
   [DFS] Visit (x,y) ...
   ```
4. When exploration is complete, the console will print:
   ```
   === Map saved! Reset the world (Ctrl+Shift+T) and run again for speed run ===
   ```
5. The file `maze_map.json` is now saved in the controller folder.

### Run 2: Speed Run

1. Reset the world: **Ctrl+Shift+T** (or use the Webots toolbar reset button).
2. Start the simulation again — do **not** delete `maze_map.json`.
3. The controller will detect the saved map and skip straight to the speed run:
   ```
   === Speed Run Mode (loading saved map) ===
   ```

---

## Important Notes

- **The map file must exist before Run 2.** If `maze_map.json` is missing or deleted, the controller will run mapping again instead of the speed run.
- **Do not change the maze between runs.** The saved map is specific to the maze layout at the time of mapping. If the maze changes, delete `maze_map.json` and re-run mapping.
- **`maze_map.json` location**: same folder as `epuck_go_forward.py`:
  ```
  controllers/epuck_go_forward/maze_map.json
  ```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Run 2 starts mapping instead of speed running | `maze_map.json` was deleted or not saved | Re-run mapping (Run 1) |
| "No path to goal!" in console | Green wall was not found during mapping | Re-run mapping; watch for `[MAP] GREEN` in console |
| Robot hits a wall during speed run | Accumulated drift during navigation | The controller retries at lower speed automatically |
| "REACHED GREEN WALL!" not printed after hitting wall | Camera did not see green (navigation error) | Re-run; robot may be misaligned |
