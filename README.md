# CMPSC 441 — Homework 3: A* Search

**Course:** CMPSC 441 — Artificial Intelligence (Fall 2025)
**Topic:** A* Search — Delivery Robot at Penn State Harrisburg Eastgate Center

## Problem

A delivery robot has to move documents through a grid-based office layout at the Penn State Harrisburg Eastgate Center. The grid contains open cells, obstacles (desks, walls), and "slow" zones (e.g., carpeted areas) that cost more to traverse. The job is to find the lowest-cost path from a start cell to a goal cell using A* search, supporting both orthogonal and diagonal movement and reacting to dynamic obstacles added mid-run.

## Files

- `Code.py` — A* implementation, terrain-aware heuristic, neighbor expansion (8 directions), path reconstruction, and visualization helpers.
- `eastgate_helper.py` — Provided helper (do not modify). Defines the office grid, terrain costs, and the `test_scenarios` driver.
- `Report.docx` — Writeup covering the approach, results, and answers to the discussion questions.

## Approach

- **A\* search** with an open set implemented as a `heapq` priority queue and a closed set for visited positions.
- **8-directional movement**: orthogonal moves cost 1, diagonals cost √2 ≈ 1.414. Diagonals are blocked when both adjacent orthogonal neighbors are walls, so the robot can't squeeze through corners.
- **Terrain costs** from `get_terrain_costs()` multiply the base move cost, so stepping into a slow zone (cost 2) is twice as expensive.
- **Heuristic**: Manhattan distance scaled by the average terrain cost across the grid. This keeps the estimate informed by terrain without dominating the actual cost. (Note: scaling Manhattan by average cost can mildly over-estimate in grids where most cells are cheap, so admissibility is discussed in the report.)
- **Dynamic obstacles**: a deep copy of the grid is mutated to inject an obstacle, then A* is re-run on the modified grid.

## Run it

```bash
python Code.py
```

This runs the three test scenarios from `test_scenarios`, then prints detailed visualizations with path costs for:
1. Standard run: `(0, 0) → (4, 4)`
2. Different start: `(2, 0) → (4, 4)`
3. Dynamic obstacle injected at `[3, 1]`, re-planned from `(2, 0) → (4, 4)`

## Visualization legend

```
S = Start    G = Goal    * = Path
# = Obstacle ~ = Slow zone (not on path)    . = Open space
```

## Discussion questions

The report covers:
- **Why the heuristic must be admissible** — it never overestimates true cost, which is what guarantees A* returns an optimal path.
- **How diagonal moves affect optimality** — with the √2 cost they preserve optimality and shorten paths; using cost 1 for diagonals would break it.
- Assumptions made along the way (corner-cutting rule, heuristic scaling choice, dynamic-obstacle handling).

## Requirements

Python 3. Standard library only (`math`, `heapq`).
