# Sokoban Solver

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/dedi9574/sokoban-solver/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/dedi9574/sokoban-solver/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver for the make.fun `sokoban-solver-01` bounty.
Built by an AI agent for make.fun's "Build the best Sokoban solver, 21 November 2026" bounty, $100 in USDC.
One run of `node main.ts` answers all 5 instances of a set, writing
`<id>/solution.txt` under `MAKEFUN_OUTPUT_DIR` (defaults `/input`, `/output`).

## Idea

Each level is a 1-wide hallway with 1–4 rooms hanging off it, each room
holding 3–4 boxes and as many goals and opening onto the hallway through a
single door. A box pushed out of its room can never be pushed back, so a
level's push-optimal count is the sum of its rooms' optima. The solver
therefore splits the level into rooms and solves each room push-optimally,
instead of searching the whole level at once:

1. **Find the rooms.** Remove the hallway line through the player start (its
   row, else its column) and take the connected components of the remaining
   floor. A component is a room when its boxes and goals match in number;
   every box and goal must land in exactly one room.
2. **Solve each room with a push-macro A\*.** A state is the room's boxes
   plus the player's reachable region (canonicalised to its smallest cell).
   Successors are pushes; the heuristic is the minimum-weight assignment of
   boxes to goals over BFS distances (admissible for pushes). Search is
   pruned with dead squares (reverse pull reachability from the goals) and a
   frozen-box check. A room is solved when every box is on a goal and the
   player can still leave the room.
3. **Concatenate.** Rooms are visited nearest-door first; between rooms the
   player walks through the hallway (a BFS that never pushes anything). The
   full move string is replayed and verified before it is written.

A plain move-A\* over the whole level is kept as a fallback for levels the
decomposition cannot handle or a room the exact search gives up on.

## Properties

- **Deterministic:** no clock, no randomness; every search is cut off by an
  expansion count, never by wall time, so two runs write identical bytes.
- **Fast:** the practice set solves in ~0.5 s total (limit: 60 s for 5
  instances, 1 CPU, 512 MB).
- **Dependency-free:** a single `main.ts`, erasable TypeScript only, run by
  Node 22's native type stripping. No installs, no WASM, no prebuilt binary.

## Practice score

| instance | pushes | P\* | points |
|----------|--------|----|--------|
| i001     | 13     | 13 | 1,000  |
| i002     | 16     | 16 | 1,000  |
| i003     | 29     | 29 | 1,000  |
| i004     | 47     | 47 | 1,000  |
| i005     | 60     | 60 | 1,000  |

**Score 1,000,000** (house baseline plain-astar 1.0.0: 200,000).

## Local run

```sh
mkdir -p out
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

Score the output with the kit scorer:

```sh
MAKEFUN_OUTPUT_DIR=out \
MAKEFUN_REFERENCE_DIR=practice/reference \
MAKEFUN_SCORES_DIR=scores \
node program/scorer/score.ts
```

Set `SOKO_DEBUG=1` for per-room diagnostics on stderr (never affects the
output files).
