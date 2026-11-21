// Room-decomposed push-optimal Sokoban solver for make.fun sokoban-solver-01.
//
// The generator lays a 1-wide hallway across the board with 1-4 rooms hanging
// off it, each room holding 3-4 boxes and as many goals and opening onto the
// hallway through a single door. A box pushed out of its room can never be
// pushed back, so a level's push-optimal count is the sum of its rooms' optima.
// This solver therefore:
//   1. finds the rooms (connected components of the floor once the hallway line
//      through the player start is removed),
//   2. solves each room push-optimally with a push-macro A* (admissible
//      min-assignment heuristic over BFS distances, dead-square and frozen-box
//      pruning, player-region canonicalisation),
//   3. concatenates the per-room move strings with legal walks through the
//      hallway between doors.
// A plain move-A* over the whole level is kept as a fallback for levels the
// decomposition cannot handle or a room the exact search gives up on.
//
// Determinism: no clock, no randomness, no environment-dependent choices; every
// search is cut off by an expansion count, never by wall time, so two runs
// write identical bytes.
//
// Local run: mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const INPUT_DIR = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const OUTPUT_DIR = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';
const DEBUG = process.env.SOKO_DEBUG === '1';

const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const LETTER = ['u', 'd', 'l', 'r'];
// Expansion caps are counts, never wall time, so the output is deterministic.
const ROOM_EXPANSIONS = 2_000_000;
const FALLBACK_EXPANSIONS = 150_000;

interface Level {
  w: number;
  h: number;
  n: number;
  wall: Uint8Array;
  goals: number[];
  goalSet: Set<number>;
  boxes: number[];
  player: number;
}

interface Ctx {
  L: Level;
  mark: Int32Array; // visit stamps for path BFS
  token: number;
  parent: Int32Array; // path BFS parents
  q: Int32Array; // shared queue
  blocked: Uint8Array; // cells currently occupied by boxes
  blockedList: number[]; // the cells blocked[] was set from (a copy)
  regionCells: number[]; // cells of the last marked region
  regionSet: Uint8Array; // 1 for cells of the last marked region
}

interface Room {
  cells: number[]; // room floor cells C
  inRoom: Uint8Array; // 1 for cells in C
  door: number; // hallway cell adjacent to C
  boxes: number[]; // initial boxes, sorted
  goals: number[]; // goals in C
  dist: Int32Array[]; // dist[g][cell]: BFS distance over C from goal g
  live: Uint8Array; // pull-reachable cells (dead-square test)
  perms: number[][]; // permutations of box indices, for the assignment heuristic
}

function parseLevel(text: string): Level {
  const rows = text.replace(/\r/g, '').split('\n').filter((r) => r.length > 0);
  const w = Math.max(...rows.map((r) => r.length));
  const h = rows.length;
  const n = w * h;
  const wall = new Uint8Array(n).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = 0;
  rows.forEach((row, y) => {
    for (let x = 0; x < w; x++) {
      const ch = row[x] ?? '#';
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if ('.*+'.includes(ch)) goals.push(i);
      if ('$*'.includes(ch)) boxes.push(i);
      if ('@+'.includes(ch)) player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  return { w, h, n, wall, goals, goalSet: new Set(goals), boxes, player };
}

/** Neighbour of cell u in direction d, or -1 when it leaves the board. */
function step(L: Level, u: number, d: number): number {
  const x = u % L.w;
  const y = (u / L.w) | 0;
  const nx = x + DX[d]!;
  const ny = y + DY[d]!;
  if (nx < 0 || nx >= L.w || ny < 0 || ny >= L.h) return -1;
  return ny * L.w + nx;
}

function makeCtx(L: Level): Ctx {
  return {
    L,
    mark: new Int32Array(L.n),
    token: 0,
    parent: new Int32Array(L.n),
    q: new Int32Array(L.n),
    blocked: new Uint8Array(L.n),
    blockedList: [],
    regionCells: [],
    regionSet: new Uint8Array(L.n),
  };
}

/** Mark ctx.blocked from boxes (a copy is kept, so callers may mutate theirs). */
function setBlocked(ctx: Ctx, boxes: number[]): void {
  for (const b of ctx.blockedList) ctx.blocked[b] = 0;
  ctx.blockedList = boxes.slice();
  for (const b of boxes) ctx.blocked[b] = 1;
}

/**
 * Flood the player's reachable region from start over floor cells not in
 * boxes. Leaves the region in ctx.regionSet / ctx.regionCells and returns its
 * smallest cell index (the canonical representative of the region).
 */
function markRegion(ctx: Ctx, start: number, boxes: number[]): number {
  const { L } = ctx;
  setBlocked(ctx, boxes);
  for (const c of ctx.regionCells) ctx.regionSet[c] = 0;
  ctx.regionCells.length = 0;
  const q = ctx.q;
  let qh = 0;
  let qt = 0;
  q[qt++] = start;
  ctx.regionSet[start] = 1;
  ctx.regionCells.push(start);
  let min = start;
  while (qh < qt) {
    const u = q[qh++];
    if (u < min) min = u;
    for (let d = 0; d < 4; d++) {
      const v = step(L, u, d);
      if (v < 0 || L.wall[v] || ctx.blocked[v] || ctx.regionSet[v]) continue;
      ctx.regionSet[v] = 1;
      ctx.regionCells.push(v);
      q[qt++] = v;
    }
  }
  return min;
}

/**
 * Shortest walk (BFS, fixed neighbour order, so deterministic) from `from` to
 * the nearest of `targets` over floor cells not in boxes. Returns the move
 * letters, '' when from is already a target, null when unreachable.
 */
function pathToAny(ctx: Ctx, from: number, targets: number[], boxes: number[]): string | null {
  const { L } = ctx;
  const targetSet = new Set(targets);
  if (targetSet.has(from)) return '';
  setBlocked(ctx, boxes);
  ctx.token++;
  const tok = ctx.token;
  const q = ctx.q;
  let qh = 0;
  let qt = 0;
  q[qt++] = from;
  ctx.mark[from] = tok;
  ctx.parent[from] = -1;
  let found = -1;
  while (qh < qt && found < 0) {
    const u = q[qh++];
    for (let d = 0; d < 4; d++) {
      const v = step(L, u, d);
      if (v < 0 || L.wall[v] || ctx.blocked[v] || ctx.mark[v] === tok) continue;
      ctx.mark[v] = tok;
      ctx.parent[v] = u;
      if (targetSet.has(v)) {
        found = v;
        break;
      }
      q[qt++] = v;
    }
  }
  if (found < 0) return null;
  let moves = '';
  for (let c = found; ctx.parent[c] >= 0; c = ctx.parent[c]) {
    const diff = c - ctx.parent[c];
    moves = (diff === 1 ? 'r' : diff === -1 ? 'l' : diff === L.w ? 'd' : 'u') + moves;
  }
  return moves;
}

/** BFS distances from sources over cells where allowed is 1. */
function bfsDist(L: Level, sources: number[], allowed: Uint8Array): Int32Array {
  const dist = new Int32Array(L.n).fill(-1);
  const q: number[] = [];
  for (const s of sources) {
    dist[s] = 0;
    q.push(s);
  }
  for (let h = 0; h < q.length; h++) {
    const u = q[h]!;
    for (let d = 0; d < 4; d++) {
      const v = step(L, u, d);
      if (v < 0 || dist[v] >= 0 || !allowed[v]) continue;
      dist[v] = dist[u] + 1;
      q.push(v);
    }
  }
  return dist;
}

function permutations(k: number): number[][] {
  const out: number[][] = [];
  const cur: number[] = [];
  const used = new Array<boolean>(k).fill(false);
  const rec = (): void => {
    if (cur.length === k) {
      out.push(cur.slice());
      return;
    }
    for (let i = 0; i < k; i++) {
      if (used[i]) continue;
      used[i] = true;
      cur.push(i);
      rec();
      cur.pop();
      used[i] = false;
    }
  };
  rec();
  return out;
}

/**
 * Dead squares by reverse pull reachability from the goals, ignoring the
 * player and other boxes: a box on a cell from which no goal is pull-reachable
 * can never reach a goal, so such a cell is dead. A box can be pulled from b
 * to b+d whenever b+2d is floor. Goals are live by definition.
 */
function computeLive(L: Level, room: Pick<Room, 'cells' | 'inRoom' | 'goals'>): Uint8Array {
  const live = new Uint8Array(L.n);
  for (const g of room.goals) live[g] = 1;
  let changed = true;
  while (changed) {
    changed = false;
    for (const b of room.cells) {
      if (!live[b]) continue;
      for (let d = 0; d < 4; d++) {
        // A pull moves a box from b to b+d when the player can end on b+2d,
        // so b+d is live whenever b+2d is floor (any floor, door included).
        const beyond = step(L, b, d);
        if (beyond < 0 || !room.inRoom[beyond] || live[beyond]) continue;
        const t = step(L, beyond, d);
        if (t < 0 || L.wall[t]) continue;
        live[beyond] = 1;
        changed = true;
      }
    }
  }
  return live;
}

function buildRoom(L: Level, cells: number[], door: number): Room {
  const inRoom = new Uint8Array(L.n);
  for (const c of cells) inRoom[c] = 1;
  const goals = L.goals.filter((g) => inRoom[g]);
  const boxes = L.boxes.filter((b) => inRoom[b]);
  const base: Pick<Room, 'cells' | 'inRoom' | 'goals'> = { cells, inRoom, goals };
  const dist = goals.map((g) => bfsDist(L, [g], inRoom));
  const live = computeLive(L, base);
  return { cells, inRoom, door, boxes, goals, dist, live, perms: permutations(boxes.length) };
}

/**
 * Split the level into rooms: remove the hallway line through the player
 * start (its row, else its column) and take the connected components of the
 * remaining floor. A room is a component whose boxes and goals match in
 * number; every box and goal must land in exactly one room. Returns null when
 * the split does not validate.
 */
function decompose(L: Level): { rooms: Room[]; hall: number[] } | null {
  const prow = (L.player / L.w) | 0;
  const pcol = L.player % L.w;
  const lines: { name: string; onLine: (i: number) => boolean }[] = [
    { name: 'row', onLine: (i) => ((i / L.w) | 0) === prow },
    { name: 'col', onLine: (i) => i % L.w === pcol },
  ];
  for (const line of lines) {
    const seen = new Uint8Array(L.n);
    const rooms: Room[] = [];
    const hall: number[] = [];
    for (let i = 0; i < L.n; i++) if (!L.wall[i] && line.onLine(i)) hall.push(i);
    let ok = true;
    let coveredBoxes = 0;
    let coveredGoals = 0;
    for (let i = 0; i < L.n && ok; i++) {
      if (L.wall[i] || line.onLine(i) || seen[i]) continue;
      const cells: number[] = [];
      const stack = [i];
      seen[i] = 1;
      while (stack.length > 0) {
        const u = stack.pop()!;
        cells.push(u);
        for (let d = 0; d < 4; d++) {
          const v = step(L, u, d);
          if (v < 0 || L.wall[v] || line.onLine(v) || seen[v]) continue;
          seen[v] = 1;
          stack.push(v);
        }
      }
      const boxCount = cells.filter((c) => L.boxes.includes(c)).length;
      const goalCount = cells.filter((c) => L.goalSet.has(c)).length;
      if (boxCount === 0 && goalCount === 0) continue;
      if (boxCount === 0 || goalCount === 0 || boxCount !== goalCount) {
        ok = false;
        break;
      }
      let door = -1;
      for (const c of cells) {
        for (let d = 0; d < 4; d++) {
          const v = step(L, c, d);
          if (v >= 0 && line.onLine(v) && !L.wall[v]) {
            door = v;
            break;
          }
        }
        if (door >= 0) break;
      }
      if (door < 0) {
        ok = false;
        break;
      }
      coveredBoxes += boxCount;
      coveredGoals += goalCount;
      rooms.push(buildRoom(L, cells, door));
    }
    if (ok && rooms.length > 0 && coveredBoxes === L.boxes.length && coveredGoals === L.goals.length) {
      return { rooms, hall };
    }
  }
  return null;
}

/** True when a box in `boxes` is frozen (can never move) and not on a goal. */
function frozenDead(L: Level, boxes: number[], goalSet: Set<number>): boolean {
  const boxSet = new Set(boxes);
  const frozen = new Set<number>();
  let changed = true;
  while (changed) {
    changed = false;
    for (const b of boxes) {
      if (frozen.has(b)) continue;
      const x = b % L.w;
      const y = (b / L.w) | 0;
      const blocked = (nx: number, ny: number): boolean => {
        if (nx < 0 || nx >= L.w || ny < 0 || ny >= L.h) return true;
        const c = ny * L.w + nx;
        return L.wall[c] || (boxSet.has(c) && frozen.has(c));
      };
      const hBlocked = blocked(x - 1, y) && blocked(x + 1, y);
      const vBlocked = blocked(x, y - 1) && blocked(x, y + 1);
      if (hBlocked && vBlocked) {
        frozen.add(b);
        changed = true;
      }
    }
  }
  for (const b of frozen) if (!goalSet.has(b)) return true;
  return false;
}

/** Binary heap of [f, g, id]: smallest f first, then largest g, then smallest id. */
class Heap {
  a: [number, number, number][] = [];
  get size(): number {
    return this.a.length;
  }
  private less(x: [number, number, number], y: [number, number, number]): boolean {
    return x[0] < y[0] || (x[0] === y[0] && (x[1] > y[1] || (x[1] === y[1] && x[2] < y[2])));
  }
  push(e: [number, number, number]): void {
    const a = this.a;
    let i = a.length;
    a.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!this.less(a[i]!, a[p]!)) break;
      [a[i], a[p]] = [a[p]!, a[i]!];
      i = p;
    }
  }
  pop(): [number, number, number] {
    const a = this.a;
    const top = a[0]!;
    const last = a.pop()!;
    if (a.length > 0) {
      a[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= a.length) break;
        if (c + 1 < a.length && this.less(a[c + 1]!, a[c]!)) c++;
        if (!this.less(a[c]!, a[i]!)) break;
        [a[i], a[c]] = [a[c]!, a[i]!];
        i = c;
      }
    }
    return top;
  }
}

/**
 * Push-optimal A* for one room. startPlayer is the player's cell (the room's
 * door); staticBoxes are the boxes outside this room, at their current cells.
 * Returns the move string (ending with the player on a hallway cell) and the
 * push count, or null when the search gives up.
 */
function solveRoom(
  ctx: Ctx,
  L: Level,
  room: Room,
  startPlayer: number,
  staticBoxes: number[],
  hall: number[],
  cap: number,
): { moves: string; pushes: number; endPlayer: number } | null {
  const k = room.boxes.length;
  const goalSet = new Set(room.goals);
  const hCache = new Map<string, number>();
  const heuristic = (boxes: number[]): number => {
    const key = boxes.join(',');
    const cached = hCache.get(key);
    if (cached !== undefined) return cached;
    let best = Infinity;
    for (const perm of room.perms) {
      let s = 0;
      for (let i = 0; i < k; i++) {
        const d = room.dist[perm[i]!]![boxes[i]!]!;
        if (d < 0) {
          s = Infinity;
          break;
        }
        s += d;
      }
      if (s < best) best = s;
    }
    hCache.set(key, best);
    return best;
  };

  const sp: number[] = [startPlayer];
  const sb: number[][] = [room.boxes.slice()];
  const sg: number[] = [0];
  const parent: number[] = [-1];
  const pbox: number[] = [-1];
  const pdir: number[] = [-1];
  const skey: string[] = [];
  const closed = new Map<string, number>();
  const startKey = `${room.boxes.join(',')}|${markRegion(ctx, startPlayer, staticBoxes.concat(room.boxes))}`;
  skey.push(startKey);
  closed.set(startKey, 0);
  const h0 = heuristic(room.boxes);
  if (h0 === Infinity) return null;
  const heap = new Heap();
  heap.push([h0, 0, 0]);
  let expanded = 0;
  let goalState = -1;

  while (heap.size > 0 && expanded < cap) {
    const [, g, s] = heap.pop();
    if ((closed.get(skey[s]!) ?? Infinity) < g) continue;
    const boxes = sb[s]!;
    // Goal: every room box on a room goal and the player able to leave the
    // room (some hallway cell in the player's region).
    if (boxes.every((b) => goalSet.has(b))) {
      markRegion(ctx, sp[s]!, staticBoxes.concat(boxes));
      let canLeave = false;
      for (const h of hall) if (ctx.regionSet[h]) { canLeave = true; break; }
      if (canLeave) {
        goalState = s;
        break;
      }
    }
    expanded++;
    const allBoxes = staticBoxes.concat(boxes);
    markRegion(ctx, sp[s]!, allBoxes);
    // Phase A: collect the legal pushes (uses the parent region).
    const cands: { boxes: number[]; box: number; dir: number }[] = [];
    for (const b of boxes) {
      const bx = b % L.w;
      const by = (b / L.w) | 0;
      for (let d = 0; d < 4; d++) {
        const tx = bx + DX[d]!;
        const ty = by + DY[d]!;
        if (tx < 0 || tx >= L.w || ty < 0 || ty >= L.h) continue;
        const t = ty * L.w + tx;
        if (!room.inRoom[t] || ctx.blocked[t] || !room.live[t]) continue;
        const px = bx - DX[d]!;
        const py = by - DY[d]!;
        if (px < 0 || px >= L.w || py < 0 || py >= L.h) continue;
        const pushFrom = py * L.w + px;
        if (!ctx.regionSet[pushFrom]) continue;
        const nb = boxes.slice();
        nb[nb.indexOf(b)] = t;
        nb.sort((a, b2) => a - b2);
        if (frozenDead(L, nb, goalSet)) continue;
        const h = heuristic(nb);
        if (h === Infinity) continue;
        cands.push({ boxes: nb, box: b, dir: d });
      }
    }
    // Phase B: key each candidate (this clobbers the region) and queue it.
    for (const c of cands) {
      const canon = markRegion(ctx, c.box, staticBoxes.concat(c.boxes));
      const key = `${c.boxes.join(',')}|${canon}`;
      const g2 = g + 1;
      if ((closed.get(key) ?? Infinity) <= g2) continue;
      closed.set(key, g2);
      const id = sp.length;
      sp.push(c.box);
      sb.push(c.boxes);
      sg.push(g2);
      parent.push(s);
      pbox.push(c.box);
      pdir.push(c.dir);
      skey.push(key);
      heap.push([g2 + heuristic(c.boxes), g2, id]);
    }
  }
  if (DEBUG) console.error(`  room door=${room.door} boxes=${k} expanded=${expanded} goal=${goalState >= 0}`);
  if (goalState < 0) return null;

  // Reconstruct: for each push, walk to the pushing square, then push.
  const pushes: { box: number; dir: number }[] = [];
  for (let s = goalState; parent[s]! >= 0; s = parent[s]!) pushes.push({ box: pbox[s]!, dir: pdir[s]! });
  pushes.reverse();
  let moves = '';
  let player = startPlayer;
  const cur = room.boxes.slice();
  for (const p of pushes) {
    const bx = p.box % L.w;
    const by = (p.box / L.w) | 0;
    const from = (by - DY[p.dir]!) * L.w + (bx - DX[p.dir]!);
    const walk = pathToAny(ctx, player, [from], staticBoxes.concat(cur));
    if (walk === null) return null;
    moves += walk + LETTER[p.dir]!;
    cur[cur.indexOf(p.box)] = (by + DY[p.dir]!) * L.w + (bx + DX[p.dir]!);
    cur.sort((a, b) => a - b);
    player = p.box;
  }
  const exit = pathToAny(ctx, player, hall, staticBoxes.concat(cur));
  if (exit === null) return null;
  moves += exit;
  // The player ends on the hallway cell the exit walk reached.
  let endPlayer = player;
  for (const ch of exit) {
    endPlayer = step(L, endPlayer, LETTER.indexOf(ch));
  }
  return { moves, pushes: pushes.length, endPlayer };
}

/** Replay moves on the level; true when legal and every box ends on a goal. */
function verify(L: Level, moves: string): boolean {
  const box = new Uint8Array(L.n);
  for (const b of L.boxes) box[b] = 1;
  let player = L.player;
  for (const ch of moves) {
    const d = LETTER.indexOf(ch.toLowerCase());
    if (d < 0) return false;
    const next = step(L, player, d);
    if (next < 0 || L.wall[next]) return false;
    if (box[next]) {
      const beyond = step(L, next, d);
      if (beyond < 0 || L.wall[beyond] || box[beyond]) return false;
      box[next] = 0;
      box[beyond] = 1;
    }
    player = next;
  }
  for (let i = 0; i < L.n; i++) if (box[i] && !L.goalSet.has(i)) return false;
  return true;
}

/**
 * Fallback: plain move-A* over the whole level (every step costs 1, estimate
 * is each box's Manhattan distance to its nearest goal, summed). Used only
 * when the room decomposition or a room search fails.
 */
function solveWholeLevel(L: Level, cap: number): string {
  const manhattan = (b: number): number => {
    let best = Infinity;
    for (const g of L.goals) {
      const d = Math.abs((g % L.w) - (b % L.w)) + Math.abs(((g / L.w) | 0) - ((b / L.w) | 0));
      if (d < best) best = d;
    }
    return best;
  };
  const est = (bs: number[]): number => bs.reduce((a, b) => a + manhattan(b), 0);
  const key = (p: number, bs: number[]): string => `${p}|${bs.join(',')}`;
  const sp: number[] = [L.player];
  const sb: number[][] = [L.boxes.slice()];
  const sg: number[] = [0];
  const parent: number[] = [-1];
  const pstep: string[] = [''];
  const best = new Map<string, number>([[key(L.player, L.boxes), 0]]);
  const heap = new Heap();
  heap.push([est(L.boxes), 0, 0]);
  let expanded = 0;
  while (heap.size > 0 && expanded < cap) {
    const [, g, s] = heap.pop();
    if ((best.get(key(sp[s]!, sb[s]!)) ?? Infinity) < g) continue;
    const boxes = sb[s]!;
    if (boxes.every((b) => L.goalSet.has(b))) {
      let moves = '';
      for (let t = s; parent[t]! >= 0; t = parent[t]!) moves = pstep[t]! + moves;
      return moves;
    }
    expanded++;
    const boxSet = new Set(boxes);
    for (let d = 0; d < 4; d++) {
      const q = step(L, sp[s]!, d);
      if (q < 0 || L.wall[q]) continue;
      let nb = boxes;
      if (boxSet.has(q)) {
        const beyond = step(L, q, d);
        if (beyond < 0 || L.wall[beyond] || boxSet.has(beyond)) continue;
        nb = boxes.slice();
        nb[nb.indexOf(q)] = beyond;
        nb.sort((a, b) => a - b);
      }
      const k2 = key(q, nb);
      if ((best.get(k2) ?? Infinity) <= g + 1) continue;
      best.set(k2, g + 1);
      const t = sp.length;
      sp.push(q);
      sb.push(nb);
      sg.push(g + 1);
      parent.push(s);
      pstep.push(LETTER[d]!);
      heap.push([g + 1 + est(nb), g + 1, t]);
    }
  }
  return '';
}

function solveLevel(text: string): string {
  const L = parseLevel(text);
  const ctx = makeCtx(L);
  const dec = decompose(L);
  if (dec !== null) {
    const { rooms, hall } = dec;
    // Solve the nearest room first, then the nearest from the previous door.
    const remaining = rooms.slice();
    const order: Room[] = [];
    let from = L.player;
    while (remaining.length > 0) {
      let bestIdx = 0;
      let bestLen = Infinity;
      for (let i = 0; i < remaining.length; i++) {
        const walk = pathToAny(ctx, from, [remaining[i]!.door], L.boxes);
        const len = walk === null ? Infinity : walk.length;
        if (len < bestLen) {
          bestLen = len;
          bestIdx = i;
        }
      }
      const [room] = remaining.splice(bestIdx, 1);
      order.push(room!);
      from = room!.door;
    }
    let moves = '';
    let player = L.player;
    let curBoxes = L.boxes.slice();
    let failed = false;
    for (const room of order) {
      const walk = pathToAny(ctx, player, [room.door], curBoxes);
      if (walk === null) {
        failed = true;
        break;
      }
      moves += walk;
      player = room.door;
      const staticBoxes = curBoxes.filter((b) => !room.boxes.includes(b));
      const res = solveRoom(ctx, L, room, room.door, staticBoxes, hall, ROOM_EXPANSIONS);
      if (res === null) {
        failed = true;
        break;
      }
      moves += res.moves;
      player = res.endPlayer;
      curBoxes = staticBoxes.concat(room.goals.slice().sort((a, b) => a - b));
      if (DEBUG) console.error(`  room door=${room.door} solved in ${res.pushes} pushes`);
    }
    if (!failed && verify(L, moves)) {
      if (DEBUG) console.error(`  verified: ${moves.length} moves`);
      return moves;
    }
    if (DEBUG) console.error('  room route failed, trying whole-level fallback');
  } else if (DEBUG) {
    console.error('  decomposition failed, trying whole-level fallback');
  }
  const fb = solveWholeLevel(L, FALLBACK_EXPANSIONS);
  return verify(L, fb) ? fb : '';
}

function main(): void {
  for (const entry of readdirSync(INPUT_DIR, { withFileTypes: true }).sort((a, b) => (a.name < b.name ? -1 : 1))) {
    if (!entry.isDirectory()) continue;
    const id = entry.name;
    let moves = '';
    try {
      moves = solveLevel(readFileSync(join(INPUT_DIR, id, 'level.txt'), 'utf8'));
    } catch {
      moves = '';
    }
    if (!/^[udlr]*$/i.test(moves) || moves.length > 65000) moves = '';
    mkdirSync(join(OUTPUT_DIR, id), { recursive: true });
    writeFileSync(join(OUTPUT_DIR, id, 'solution.txt'), `${moves}\n`);
  }
}

main();
