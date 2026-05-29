// ══════════════════════════════════════════════════════════════
//  SmartRoute — Simulasi Kurir Cerdas
//  PAA 2026 · UMRAH · Teknik Informatika
//  Algoritma Pencarian Rute: BFS (baseline) + Dijkstra + A* (A-Star)
//  Konteks: Kurir mengantarkan paket dari titik asal ke tujuan
//           melalui jalur jalan terpendek yang ditemukan algoritma
// ══════════════════════════════════════════════════════════════

// ── Default Data ──────────────────────────────────────────────
const defaultGraph = {
  "A": [{ node: "C", weight: 280 }, { node: "B", weight: 220 }, { node: "H", weight: 300 }],
  "B": [{ node: "A", weight: 220 }, { node: "E", weight: 290 }, { node: "D", weight: 230 }, { node: "H", weight: 150 }],
  "C": [{ node: "A", weight: 280 }, { node: "E", weight: 260 }, { node: "F", weight: 370 }, { node: "I", weight: 200 }],
  "D": [{ node: "B", weight: 230 }, { node: "G", weight: 600 }, { node: "J", weight: 100 }],
  "E": [{ node: "B", weight: 290 }, { node: "C", weight: 260 }, { node: "F", weight: 230 }, { node: "G", weight: 370 }],
  "F": [{ node: "C", weight: 370 }, { node: "E", weight: 230 }, { node: "G", weight: 260 }, { node: "I", weight: 250 }, { node: "K", weight: 200 }],
  "G": [{ node: "D", weight: 600 }, { node: "E", weight: 370 }, { node: "F", weight: 260 }, { node: "K", weight: 250 }],
  "H": [{ node: "A", weight: 300 }, { node: "B", weight: 150 }],
  "I": [{ node: "C", weight: 200 }, { node: "F", weight: 250 }],
  "J": [{ node: "D", weight: 100 }],
  "K": [{ node: "F", weight: 200 }, { node: "G", weight: 250 }]
};
const defaultPositions = {
  "A": { x: 500, y: 30 }, "B": { x: 300, y: 290 }, "C": { x: 550, y: 160 },
  "D": { x: 210, y: 320 }, "E": { x: 600, y: 350 }, "F": { x: 870, y: 320 },
  "G": { x: 500, y: 500 }, "H": { x: 60, y: 200 }, "I": { x: 750, y: 100 },
  "J": { x: 110, y: 580 }, "K": { x: 970, y: 580 }
};

let originalGraph = JSON.parse(JSON.stringify(defaultGraph));
let graph = JSON.parse(JSON.stringify(defaultGraph));
let positions = JSON.parse(JSON.stringify(defaultPositions));
let brokenRoads = [];
let SPEED = 5;
let simRunning = false;

// ── Populate selects ──────────────────────────────────────────
function populateSelects() {
  const nodes = Object.keys(positions).sort();
  ["start", "end"].forEach(id => {
    const sel = document.getElementById(id);
    const prev = sel.value;
    sel.innerHTML = "";
    nodes.forEach(n => sel.innerHTML += `<option value="${n}">${n}</option>`);
    if (nodes.includes(prev)) sel.value = prev;
  });
  const s = document.getElementById("start");
  const e = document.getElementById("end");
  if (s.value === e.value) {
    const fb = nodes.find(n => n !== s.value);
    if (fb) e.value = fb;
  }
}

// ── Bezier helpers (dari original) ───────────────────────────
function getCurveParams(p1, p2, n1, n2) {
  const dx = p2.x - p1.x, dy = p2.y - p1.y;
  const dist = Math.hypot(dx, dy);
  if (dist < 1) return null;
  const flip = (n1.charCodeAt(0) + n2.charCodeAt(0)) % 2 === 0 ? 1 : -1;
  const ci = Math.min(dist * 0.22, 70) * flip;
  const nx = (-dy / dist) * ci, ny = (dx / dist) * ci;
  return {
    cx1: p1.x + dx * 0.3 + nx, cy1: p1.y + dy * 0.3 + ny,
    cx2: p1.x + dx * 0.7 - nx, cy2: p1.y + dy * 0.7 - ny
  };
}

function getNaturalPath(p1, p2, n1, n2) {
  const c = getCurveParams(p1, p2, n1, n2);
  if (!c) return `L ${p2.x} ${p2.y}`;
  return `C ${c.cx1} ${c.cy1} ${c.cx2} ${c.cy2} ${p2.x} ${p2.y}`;
}

function bezierPoint(p1, p2, n1, n2, t) {
  const c = getCurveParams(p1, p2, n1, n2);
  if (!c) return { x: p1.x + (p2.x - p1.x) * t, y: p1.y + (p2.y - p1.y) * t };
  const mt = 1 - t;
  return {
    x: mt * mt * mt * p1.x + 3 * mt * mt * t * c.cx1 + 3 * mt * t * t * c.cx2 + t * t * t * p2.x,
    y: mt * mt * mt * p1.y + 3 * mt * mt * t * c.cy1 + 3 * mt * t * t * c.cy2 + t * t * t * p2.y
  };
}

// ── Anti-crossing (dari original) ────────────────────────────
function segmentsIntersect(ax, ay, bx, by, cx, cy, dx, dy) {
  const d1x = bx - ax, d1y = by - ay, d2x = dx - cx, d2y = dy - cy;
  const cross = d1x * d2y - d1y * d2x;
  if (Math.abs(cross) < 1e-9) return false;
  const tx = ((cx - ax) * d2y - (cy - ay) * d2x) / cross;
  const ty = ((cx - ax) * d1y - (cy - ay) * d1x) / cross;
  const E = 0.04;
  return tx > E && tx < 1 - E && ty > E && ty < 1 - E;
}

function buildEdgePts(p1, p2, n1, n2, S = 8) {
  const pts = [];
  for (let i = 0; i <= S; i++) pts.push(bezierPoint(p1, p2, n1, n2, i / S));
  return pts;
}

function edgeCrosses(list, p1, p2, n1, n2) {
  const np = buildEdgePts(p1, p2, n1, n2, 8);
  for (const { pts, from, to } of list) {
    if (from === n1 || from === n2 || to === n1 || to === n2) continue;
    for (let i = 0; i < np.length - 1; i++)
      for (let j = 0; j < pts.length - 1; j++)
        if (segmentsIntersect(
          np[i].x, np[i].y, np[i + 1].x, np[i + 1].y,
          pts[j].x, pts[j].y, pts[j + 1].x, pts[j + 1].y
        )) return true;
  }
  return false;
}

function ptSegDist(px, py, ax, ay, bx, by) {
  const dx = bx - ax, dy = by - ay, l = dx * dx + dy * dy;
  if (l < 1) return Math.hypot(px - ax, py - ay);
  const t = Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / l));
  return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
}

function pointNearEdge(px, py, pts, clearance) {
  for (let i = 0; i < pts.length - 1; i++)
    if (ptSegDist(px, py, pts[i].x, pts[i].y, pts[i + 1].x, pts[i + 1].y) < clearance)
      return true;
  return false;
}

// ── Decorations ───────────────────────────────────────────────
function rnd(a, b) { return a + Math.random() * (b - a); }
function ri(a, b) { return Math.floor(rnd(a, b + 1)); }

function svgTree(x, y, sc) {
  sc = sc || 1;
  return `<g transform="translate(${x.toFixed(1)},${y.toFixed(1)}) scale(${sc.toFixed(2)})">
    <rect x="-2" y="3" width="4" height="9" fill="#5c4033"/>
    <circle cx="0" cy="-4" r="9" fill="#2d6a4f" opacity="0.85"/>
    <circle cx="-4" cy="-1" r="6" fill="#2d6a4f" opacity="0.75"/>
    <circle cx="4"  cy="-1" r="6" fill="#2d6a4f" opacity="0.75"/>
  </g>`;
}

function svgBuildings(cx, cy, cnt, tall) {
  const C = ["#94a3b8", "#64748b", "#cbd5e1", "#aab4c8", "#7f8fa6", "#475569"];
  let h = "";
  for (let i = 0; i < cnt; i++) {
    const bw = ri(10, 18), bh = tall ? ri(40, 75) : ri(18, 40);
    const bx = cx - (cnt * 14) / 2 + i * 14 + ri(-3, 3), by = cy - bh;
    h += `<rect x="${bx.toFixed(1)}" y="${by.toFixed(1)}" width="${bw}" height="${bh}"
      fill="${C[ri(0, C.length - 1)]}" stroke="#333" stroke-width="1.2" rx="1"/>`;
    const rows = Math.max(1, Math.floor(bh / 11) - 1);
    for (let r = 0; r < rows; r++) {
      const lit = Math.random() > 0.25;
      h += `<rect x="${(bx + bw * 0.2).toFixed(1)}" y="${(by + 5 + r * 10).toFixed(1)}"
        width="${(bw * 0.6).toFixed(1)}" height="5"
        fill="${lit ? '#fef08a' : '#334155'}" opacity="0.8" rx="1"/>`;
    }
  }
  return h;
}

function drawDecorations(allEdgePts) {
  const layer = document.getElementById("deco-layer");
  layer.innerHTML = "";
  const SLOT_R = 42, ROAD_CLEAR = 28, SLOTS = 16;
  let html = "";
  for (const name in positions) {
    const p = positions[name], deg = (originalGraph[name] || []).length, fa = [];
    for (let i = 0; i < SLOTS; i++) {
      const a = (i / SLOTS) * Math.PI * 2;
      const tx = p.x + Math.cos(a) * SLOT_R, ty = p.y + Math.sin(a) * SLOT_R;
      let clear = true;
      for (const { pts } of allEdgePts)
        if (pointNearEdge(tx, ty, pts, ROAD_CLEAR)) { clear = false; break; }
      if (clear) fa.push(a);
    }
    if (!fa.length) continue;
    const mid = fa[Math.floor(fa.length / 2)];
    const dx = Math.cos(mid), dy = Math.sin(mid);
    if (deg >= 4)
      html += svgBuildings(p.x + dx * (SLOT_R + 5), p.y + dy * (SLOT_R + 5), ri(4, 6), true);
    else if (deg >= 2) {
      html += svgBuildings(p.x + dx * (SLOT_R + 5), p.y + dy * (SLOT_R + 5), ri(2, 4), false);
      if (fa.length >= 3)
        html += svgTree(p.x + Math.cos(fa[1]) * (SLOT_R + 10), p.y + Math.sin(fa[1]) * (SLOT_R + 10), rnd(0.7, 1.1));
    } else {
      fa.slice(0, 3).forEach(a =>
        html += svgTree(p.x + Math.cos(a) * (SLOT_R + rnd(5, 18)), p.y + Math.sin(a) * (SLOT_R + rnd(5, 18)), rnd(0.7, 1.2))
      );
    }
  }
  layer.innerHTML = html;
}

// ── Draw Map ──────────────────────────────────────────────────
function buildAllEdgePts() {
  const list = [], drawn = new Set();
  for (const u in graph) {
    for (const e of graph[u]) {
      const pair = [u, e.node].sort().join("-");
      if (!drawn.has(pair)) {
        drawn.add(pair);
        list.push({
          pts: buildEdgePts(positions[u], positions[e.node], u, e.node, 10),
          from: u, to: e.node
        });
      }
    }
  }
  return list;
}

function drawMap(animate, edgePtList) {
  const edgeList = edgePtList || buildAllEdgePts();
  let baseD = "";
  for (const { from, to } of edgeList)
    baseD += `M ${positions[from].x} ${positions[from].y} ${getNaturalPath(positions[from], positions[to], from, to)} `;

  ["road-layer-1", "road-layer-2", "road-layer-3"]
    .forEach(id => document.getElementById(id).setAttribute("d", baseD));

  const ng = document.getElementById("nodes-group");
  ng.innerHTML = "";
  for (const name in positions) {
    const p = positions[name];
    ng.innerHTML += `
      <circle class="${animate ? 'node node-anim' : 'node'}" id="nc-${name}" cx="${p.x}" cy="${p.y}" r="14"/>
      <text class="label-text" x="${p.x}" y="${p.y + 1}">${name}</text>`;
  }

  drawWeightLabels();
  drawDecorations(edgeList);
  resetSimVisuals();
  updateFlags();
  brokenRoads = [];
}

function drawWeightLabels() {
  const layer = document.getElementById("weight-labels-layer");
  layer.innerHTML = "";
  const drawn = new Set();
  for (const u in originalGraph) {
    for (const e of originalGraph[u]) {
      const pair = [u, e.node].sort().join("-");
      if (!drawn.has(pair)) {
        drawn.add(pair);
        const mid = bezierPoint(positions[u], positions[e.node], u, e.node, 0.5);
        layer.innerHTML += `
          <rect x="${(mid.x - 17).toFixed(1)}" y="${(mid.y - 8).toFixed(1)}" width="34" height="15" rx="4" fill="#333" opacity="0.78"/>
          <text class="weight-label" x="${mid.x.toFixed(1)}" y="${(mid.y + 0.5).toFixed(1)}">${e.weight}</text>`;
      }
    }
  }
}

function resetSimVisuals() {
  ["visited-d-layer", "visited-a-layer", "visited-bfs-layer",
    "node-labels-layer", "broken-roads-layer"]
    .forEach(id => document.getElementById(id).innerHTML = "");

  ["path-dijkstra", "path-astar", "path-bfs", "highlight-path"]
    .forEach(id => {
      document.getElementById(id).setAttribute("d", "");
      document.getElementById(id).setAttribute("opacity", "0");
    });

  // Sembunyikan car — reset posisi
  document.getElementById("car-body").setAttribute("opacity", "0");
  document.getElementById("car-wheels").setAttribute("opacity", "0");

  // Hapus animasi lama agar tidak "freeze" di posisi akhir run sebelumnya
  document.getElementById("car-anim-body")?.remove();

  // Matikan wheel spin
  document.querySelectorAll(".wheel-inner")
    .forEach(w => w.classList.remove("wheel-spin"));

  document.getElementById("step-ctr").style.display = "none";
}

function updateFlags() {
  const s = document.getElementById("start").value;
  const e = document.getElementById("end").value;
  const fs = document.getElementById("flag-start");
  const fe = document.getElementById("flag-end");
  if (positions[s]) {
    fs.setAttribute("transform", `translate(${positions[s].x},${positions[s].y})`);
    fs.setAttribute("opacity", "1");
  }
  if (positions[e]) {
    fe.setAttribute("transform", `translate(${positions[e].x},${positions[e].y})`);
    fe.setAttribute("opacity", "1");
  }
}

// ══════════════════════════════════════════════════════════════
//  MIN-HEAP (Priority Queue)
//  Digunakan oleh Dijkstra & A* → insert dan extractMin O(log n)
//
//  Analisis Amortized (Amortisasi):
//    Tiap operasi push/pop: O(log n) worst case
//    Amortized per operasi dalam n operasi: O(log n)
//    Total n operasi: O(n log n)
//
//  Recurrence Equation Dijkstra:
//    T(V,E) = V·O(log V) + E·O(log V) = O((V+E) log V)
// ══════════════════════════════════════════════════════════════
class MinHeap {
  constructor() { this.h = []; }

  push(item) {
    this.h.push(item);
    this._up(this.h.length - 1);
  }

  pop() {
    const top = this.h[0];
    const last = this.h.pop();
    if (this.h.length) { this.h[0] = last; this._down(0); }
    return top;
  }

  get size() { return this.h.length; }

  _up(i) {
    while (i > 0) {
      const p = Math.floor((i - 1) / 2);
      if (this.h[p].f <= this.h[i].f) break;
      [this.h[p], this.h[i]] = [this.h[i], this.h[p]];
      i = p;
    }
  }

  _down(i) {
    const n = this.h.length;
    while (true) {
      let m = i, l = 2 * i + 1, r = 2 * i + 2;
      if (l < n && this.h[l].f < this.h[m].f) m = l;
      if (r < n && this.h[r].f < this.h[m].f) m = r;
      if (m === i) break;
      [this.h[m], this.h[i]] = [this.h[i], this.h[m]];
      i = m;
    }
  }
}

// ══════════════════════════════════════════════════════════════
//  PERBANDINGAN PENDEKATAN ALGORITMA (RPS Materi PAA)
//  ─────────────────────────────────────────────────────────────
//  Brute Force  : Coba semua jalur kemungkinan → O(V!)
//                 TIDAK dipakai karena sangat tidak efisien.
//                 Untuk 11 node: 11! = 39.916.800 kemungkinan.
//
//  Dynamic Prog : Bellman-Ford O(VE) — dipakai jika bobot negatif.
//                 Graf ini semua bobot positif → Dijkstra lebih optimal.
//
//  Greedy       : Dijkstra = greedy (pilih node terkecil dulu).
//                 Optimal karena bobot ≥ 0 (tidak ada bobot negatif).
//
//  Informed     : A* = informed search dengan heuristik Euclidean.
//                 Lebih efisien dari Dijkstra karena terarah ke tujuan.
//
//  Theory of Complexity:
//    Shortest Path ∈ P (polynomial) — bisa diselesaikan efisien.
//    TSP (Travelling Salesman) ∈ NP-Hard — berbeda dengan masalah ini.
//    Masalah ini: single-source shortest path → kelas P ✅
// ══════════════════════════════════════════════════════════════

// ══════════════════════════════════════════════════════════════
//  BFS (Breadth-First Search) — O(V+E)  [BASELINE]
//  ─────────────────────────────────────────────────────────────
//  Kompleksitas Waktu : O(V + E)
//  Kompleksitas Ruang : O(V)
//  Tidak mempertimbangkan bobot → menemukan HOP terkecil,
//  BUKAN rute terpendek berbobot. Dipakai sebagai BASELINE.
//
//  Pseudocode:
//    queue = [start]  ;  visited = {}
//    while queue tidak kosong:
//      u = queue.dequeue()          // FIFO
//      visited.add(u)
//      if u == end : selesai
//      for setiap tetangga v dari u:
//        if v ∉ visited:
//          prev[v] = u
//          queue.enqueue(v)
//    rekonstruksi path dari prev[]
// ══════════════════════════════════════════════════════════════
function bfs(g, start, end) {
  const visited = [], prev = {}, order = [];
  const queue = [start], inQ = new Set([start]);

  while (queue.length > 0) {
    const u = queue.shift();                  // dequeue (FIFO)
    visited.push(u);
    order.push({ node: u, step: visited.length });
    if (u === end) break;
    for (const nb of g[u]) {
      if (!inQ.has(nb.node)) {
        inQ.add(nb.node);
        prev[nb.node] = u;
        queue.push(nb.node);
      }
    }
  }

  // rekonstruksi path
  const path = []; let cur = end;
  while (cur !== undefined) { path.unshift(cur); cur = prev[cur]; }

  // hitung total jarak berbobot dari path yang ditemukan BFS
  let dist = 0;
  for (let i = 0; i < path.length - 1; i++) {
    const e = g[path[i]]?.find(x => x.node === path[i + 1]);
    if (e) dist += e.weight;
  }

  return {
    path: path[0] === start ? path : [],
    distance: dist,
    visited,
    order
  };
}

// ══════════════════════════════════════════════════════════════
//  DIJKSTRA'S ALGORITHM — O((V+E) log V)
//  Strategi: GREEDY — selalu pilih node dengan jarak terkecil
//  ─────────────────────────────────────────────────────────────
//  Kompleksitas Waktu : O((V + E) log V)
//    - V iterasi extract-min dari Min-Heap  → O(V log V)
//    - E iterasi relaksasi edge             → O(E log V)
//    - visited check: O(1) dengan Set (bukan O(n) dengan Array)
//  Kompleksitas Ruang : O(V) — dist[], prev[], visitedSet{}
//  Garansi: rute terpendek OPTIMAL untuk bobot ≥ 0
//
//  Pseudocode:
//    dist[start] = 0  ;  dist[v] = ∞  untuk semua v ≠ start
//    PQ.push({ node: start, f: 0 })
//    while PQ tidak kosong:
//      u = PQ.extractMin()          // node dengan dist terkecil
//      if u ∈ visited : skip
//      visited.add(u)
//      for setiap tetangga v dari u:
//        if dist[u] + w(u,v) < dist[v]:
//          dist[v] = dist[u] + w(u,v)
//          prev[v] = u
//          PQ.push({ node: v, f: dist[v] })
//    rekonstruksi path dari prev[]
// ══════════════════════════════════════════════════════════════
function dijkstra(g, start, end) {
  const dist = {}, prev = {}, order = [];
  // Gunakan Set untuk O(1) lookup, visited array untuk urutan animasi
  const visitedSet = new Set(), visited = [];
  for (const n in g) dist[n] = Infinity;
  dist[start] = 0;

  const pq = new MinHeap();
  pq.push({ f: 0, node: start });

  while (pq.size > 0) {
    const { f: d, node: u } = pq.pop();
    if (visitedSet.has(u)) continue;         // O(1) — lebih efisien dari includes()
    visitedSet.add(u);
    visited.push(u);
    order.push({ node: u, dist: d, step: visited.length });
    if (u === end) break;

    for (const nb of g[u]) {
      const nd = dist[u] + nb.weight;        // relaksasi edge
      if (nd < dist[nb.node]) {
        dist[nb.node] = nd;
        prev[nb.node] = u;
        pq.push({ f: nd, node: nb.node });
      }
    }
  }

  // rekonstruksi path dari prev[]
  const path = []; let cur = end;
  while (cur !== undefined) { path.unshift(cur); cur = prev[cur]; }

  return {
    path: path[0] === start ? path : [],
    distance: dist[end],
    visited,
    order                           // dipakai oleh animateVisited() untuk visualisasi
  };
}

// ══════════════════════════════════════════════════════════════
//  A* (A-STAR) ALGORITHM — O(E log V) average
//  Informed Search — f(n) = g(n) + h(n)
//  ─────────────────────────────────────────────────────────────
//  @contributor  M. Kika Haekal
//                (Implementasi A*, heuristic(), Bukti Admissibility,
//                 Analisis Kompleksitas A*)
//
//  Fungsi evaluasi: f(n) = g(n) + h(n)
//    g(n) = biaya aktual dari start ke node n
//    h(n) = heuristik Euclidean × 0.40  [ADMISSIBLE]
//    f(n) = estimasi total biaya jalur melewati n
//
//  Bukti Admissibility heuristic:
//    h(n) = Euclidean(n, end) × 0.40
//    Bobot minimum edge = jarak Euclidean × ~0.40 (lihat randomizeMap)
//    ∴ h(n) ≤ biaya nyata ke tujuan → A* PASTI OPTIMAL  ✓
//    Karena h admissible, A* tidak akan melewatkan rute terpendek.
//
//  Kompleksitas Waktu : O(E log V) rata-rata
//    - Dengan h admissible, A* mengekspansi lebih sedikit node
//      dari Dijkstra karena terarah ke tujuan.
//    - visitedSet.has(u): O(1) dengan Set, bukan O(n) dengan Array
//    - Worst case = O((V+E) log V) jika h(n) = 0 (≡ Dijkstra)
//
//  Kompleksitas Ruang : O(V)
//    - gCost[]     : O(V) — biaya aktual dari start ke tiap node
//    - fCost[]     : O(V) — nilai f(n) = g(n) + h(n)
//    - prev[]      : O(V) — rekonstruksi jalur
//    - visitedSet{}: O(V) — Set untuk lookup O(1)
//
//  Pseudocode:
//    g[start] = 0  ;  f[start] = h(start, end)
//    open.push({ node: start, f: f[start] })
//    while open tidak kosong:
//      u = open.extractMin()        // O(log V) — node f(n) terkecil
//      if u ∈ visitedSet : skip     // O(1) lookup
//      visitedSet.add(u)
//      if u == end : selesai
//      for setiap tetangga v dari u:
//        gNew = g[u] + w(u,v)
//        if gNew < g[v]:
//          g[v]    = gNew
//          f[v]    = g[v] + h(v, end)
//          prev[v] = u
//          open.push({ node: v, f: f[v] })  // O(log V)
//    rekonstruksi: path.unshift(cur) saat cur = prev[cur]
// ══════════════════════════════════════════════════════════════
function heuristic(a, b) {
  // Euclidean distance × 0.40 — scaling agar admissible
  return Math.hypot(positions[a].x - positions[b].x, positions[a].y - positions[b].y) * 0.40;
}

function astar(g, start, end) {
  const gCost = {}, fCost = {}, prev = {}, order = [];
  // Gunakan Set untuk O(1) lookup, visited array untuk urutan animasi
  const visitedSet = new Set(), visited = [];
  for (const n in g) { gCost[n] = Infinity; fCost[n] = Infinity; }
  gCost[start] = 0;
  fCost[start] = heuristic(start, end);

  const open = new MinHeap();
  open.push({ f: fCost[start], node: start });

  while (open.size > 0) {
    const { node: u } = open.pop();
    if (visitedSet.has(u)) continue;         // O(1) — lebih efisien dari includes()
    visitedSet.add(u);
    visited.push(u);
    order.push({
      node: u,
      g: gCost[u],
      h: heuristic(u, end),
      f: fCost[u],
      step: visited.length
    });
    if (u === end) break;

    for (const nb of g[u]) {
      const gNew = gCost[u] + nb.weight;
      if (gNew < gCost[nb.node]) {
        gCost[nb.node] = gNew;
        fCost[nb.node] = gNew + heuristic(nb.node, end);
        prev[nb.node] = u;
        open.push({ f: fCost[nb.node], node: nb.node });
      }
    }
  }

  // rekonstruksi path dari prev[]
  const path = []; let cur = end;
  while (cur !== undefined) { path.unshift(cur); cur = prev[cur]; }

  return {
    path: path[0] === start ? path : [],
    distance: gCost[end],
    visited,
    order
  };
}

// ══════════════════════════════════════════════════════════════
//  CAR ANIMATION — SATU SISTEM (requestAnimationFrame)
//  ─────────────────────────────────────────────────────────────
//  Bodi dan roda keduanya di-update dalam satu rAF loop yang
//  sama sehingga posisi & timing selalu sinkron.
//
//  car-body   → translate + rotate(angle) dari samplePathAngle()
//  car-wheels → translate saja, spin dari CSS @keyframes spinning
// ══════════════════════════════════════════════════════════════
const wait = ms => new Promise(r => setTimeout(r, ms));

function buildPathD(path) {
  let d = `M ${positions[path[0]].x} ${positions[path[0]].y} `;
  for (let i = 1; i < path.length; i++)
    d += getNaturalPath(positions[path[i - 1]], positions[path[i]], path[i - 1], path[i]) + " ";
  return d;
}

// Interpolasi posisi di sepanjang multi-segment bezier path
function samplePathPos(path, t) {
  const segs = []; let total = 0;
  for (let i = 0; i < path.length - 1; i++) {
    const p1 = positions[path[i]], p2 = positions[path[i + 1]];
    const len = Math.hypot(p2.x - p1.x, p2.y - p1.y);
    segs.push({ p1, p2, n1: path[i], n2: path[i + 1], len });
    total += len;
  }
  let target = t * total, acc = 0;
  for (const seg of segs) {
    if (acc + seg.len >= target)
      return bezierPoint(seg.p1, seg.p2, seg.n1, seg.n2, (target - acc) / seg.len);
    acc += seg.len;
  }
  return { ...positions[path[path.length - 1]] };
}

// Hitung angle arah gerak di titik t (finite difference)
function samplePathAngle(path, t) {
  const EPS = 0.005;
  const p1 = samplePathPos(path, Math.max(0, t - EPS));
  const p2 = samplePathPos(path, Math.min(1, t + EPS));
  return Math.atan2(p2.y - p1.y, p2.x - p1.x) * 180 / Math.PI;
}

async function moveCar(path, distance) {
  const carBody = document.getElementById("car-body");
  const carWheels = document.getElementById("car-wheels");

  document.getElementById("car-anim-body")?.remove();

  const pathD = buildPathD(path);
  const safeDist = (isFinite(distance) && distance > 0) ? distance : 300;
  const dur = Math.max(1.5, safeDist / 150);
  const durMs = dur * 1000;

  const hp = document.getElementById("highlight-path");
  hp.setAttribute("d", pathD);
  hp.setAttribute("opacity", "0.85");

  carBody.setAttribute("opacity", "1");
  carWheels.setAttribute("opacity", "1");
  document.querySelectorAll(".wheel-inner").forEach(w => w.classList.add("wheel-spin"));

  const t0 = performance.now();

  await new Promise(resolve => {
    function frame(now) {
      const t = Math.min((now - t0) / durMs, 1);
      const pos = samplePathPos(path, t);
      const ang = samplePathAngle(path, t);

      // Body + wheels keduanya pakai rAF — timing identik, tidak ada konflik
      carBody.setAttribute("transform",
        `translate(${pos.x.toFixed(2)},${pos.y.toFixed(2)}) rotate(${ang.toFixed(2)})`);
      carWheels.setAttribute("transform",
        `translate(${pos.x.toFixed(2)},${pos.y.toFixed(2)})`);

      if (t < 1) {
        requestAnimationFrame(frame);
      } else {
        carBody.setAttribute("opacity", "0");
        carWheels.setAttribute("opacity", "0");
        document.querySelectorAll(".wheel-inner").forEach(w => w.classList.remove("wheel-spin"));
        resolve();
      }
    }
    requestAnimationFrame(frame);
  });
}

// ══════════════════════════════════════════════════════════════
//  STEP-BY-STEP VISUAL DI PETA
//  ─────────────────────────────────────────────────────────────
//  @contributor  Yehezkiel Alman
//                (Visualisasi Graf, Animasi Step-by-Step,
//                 samplePathPos(), buildPathD(), Struktur Graf)
//
//  animateVisited() menampilkan proses eksplorasi node secara
//  visual, satu per satu sesuai urutan kunjungan algoritma.
//  Label nilai (dist= / f=g+h / hop=) muncul di atas tiap node
//  untuk menggambarkan proses pencarian rute step-by-step.
//
//  Kompleksitas: O(V) — iterasi tiap node yang dikunjungi
// ══════════════════════════════════════════════════════════════
const aniDelay = () => wait(Math.max(10, 200 / SPEED));

async function animateVisited(orderArr, layerId, stroke, fill, labelColor, algoType) {
  const layer = document.getElementById(layerId);
  const lblLayer = document.getElementById("node-labels-layer");
  const ctr = document.getElementById("step-ctr");
  ctr.style.display = "block";

  for (let i = 0; i < orderArr.length; i++) {
    const o = orderArr[i], p = positions[o.node];
    if (!p) continue;

    ctr.textContent = `STEP ${o.step}/${orderArr.length} — Node: ${o.node}`;

    // lingkaran highlight
    const c = document.createElementNS("http://www.w3.org/2000/svg", "circle");
    c.setAttribute("cx", p.x); c.setAttribute("cy", p.y);
    c.setAttribute("r", "20"); c.setAttribute("fill", fill);
    c.setAttribute("stroke", stroke); c.setAttribute("stroke-width", "2");
    c.setAttribute("opacity", ".5");
    layer.appendChild(c);

    // label nilai di atas node
    let val = "";
    if (algoType === "dijkstra") val = `d=${o.dist}`;
    else if (algoType === "astar") val = `f=${Math.round(o.f)}`;
    else if (algoType === "bfs") val = `#${o.step}`;

    const bg = document.createElementNS("http://www.w3.org/2000/svg", "rect");
    bg.setAttribute("x", (p.x - 20).toFixed(1)); bg.setAttribute("y", (p.y - 39).toFixed(1));
    bg.setAttribute("width", "40"); bg.setAttribute("height", "14");
    bg.setAttribute("rx", "4"); bg.setAttribute("fill", "#111"); bg.setAttribute("opacity", ".88");
    lblLayer.appendChild(bg);

    const txt = document.createElementNS("http://www.w3.org/2000/svg", "text");
    txt.setAttribute("x", p.x.toFixed(1)); txt.setAttribute("y", (p.y - 29).toFixed(1));
    txt.setAttribute("text-anchor", "middle"); txt.setAttribute("dominant-baseline", "middle");
    txt.setAttribute("font-size", "9"); txt.setAttribute("font-weight", "bold");
    txt.setAttribute("fill", labelColor); txt.setAttribute("font-family", "monospace");
    txt.setAttribute("pointer-events", "none");
    txt.textContent = val;
    lblLayer.appendChild(txt);

    await aniDelay();
  }
  ctr.style.display = "none";
}

function drawFinalPath(path, elId, opacity) {
  if (!path || path.length < 2) return;
  const el = document.getElementById(elId);
  el.setAttribute("d", buildPathD(path));
  el.setAttribute("opacity", opacity || "1");
}

// ── Log ───────────────────────────────────────────────────────
function lg(msg, cls = "inf") {
  const b = document.getElementById("log-body");
  const d = document.createElement("div");
  d.className = "ll " + cls;
  d.textContent = msg;
  b.appendChild(d);
  b.scrollTop = b.scrollHeight;     // auto-scroll ke bawah
}
function lgSep() { lg("─".repeat(36), "sep"); }       // pemisah antar algoritma
function clearLog() { document.getElementById("log-body").innerHTML = ""; }

// ── Reset results ─────────────────────────────────────────────
function resetResults() {
  ["d-dist", "d-time", "d-nodes", "d-path",
    "a-dist", "a-time", "a-nodes", "a-path",
    "b-dist", "b-time", "b-nodes", "b-path"]
    .forEach(id => { const el = document.getElementById(id); if (el) el.textContent = "—"; });
  ["bar-d-time", "bar-d-nodes", "bar-a-time", "bar-a-nodes", "bar-b-time", "bar-b-nodes"]
    .forEach(id => { const el = document.getElementById(id); if (el) el.style.width = "0%"; });
  document.getElementById("winner-badge").classList.remove("show");
  document.getElementById("cmp-table").classList.remove("show");
  document.getElementById("eval-box").style.display = "none";
  ["card-d", "card-a", "card-b"].forEach(id => document.getElementById(id)?.classList.remove("win-card"));
}

// ── Show results & evaluasi algoritma ────────────────────────
// @contributor  Neza Khairunnisa Rahmah
//               (showResults, tabel perbandingan, evaluasi kelebihan
//                & kekurangan tiap algoritma, winner badge)
function showResults(rD, rA, rB) {
  const maxT = Math.max(rD?.timeMs || 0, rA?.timeMs || 0, rB?.timeMs || 0) || 1;
  const maxN = Math.max(rD?.visited?.length || 0, rA?.visited?.length || 0, rB?.visited?.length || 0) || 1;

  function fillCard(px, r) {
    if (!r?.path?.length) return;
    document.getElementById(px + "-dist").textContent = r.distance + " km";
    document.getElementById(px + "-time").textContent = r.timeMs + " ms";
    document.getElementById(px + "-nodes").textContent = r.visited.length + " node";
    document.getElementById(px + "-path").textContent = r.path.join("→");
    setTimeout(() => {
      document.getElementById("bar-" + px + "-time").style.width = (r.timeMs / maxT * 100).toFixed(1) + "%";
      document.getElementById("bar-" + px + "-nodes").style.width = (r.visited.length / maxN * 100).toFixed(1) + "%";
    }, 200);
  }
  fillCard("d", rD);
  fillCard("a", rA);
  fillCard("b", rB);

  if (!rD?.path?.length || !rA?.path?.length) return;

  const dWT = rD.timeMs <= rA.timeMs;
  const dWN = rD.visited.length <= rA.visited.length;
  const same = rD.distance === rA.distance;
  const ratio = (Math.max(rD.timeMs, rA.timeMs) / Math.max(Math.min(rD.timeMs, rA.timeMs), 0.001)).toFixed(1);
  const hasB = rB?.path?.length > 0;

  // tabel perbandingan
  document.getElementById("cmp-tbody").innerHTML = `
    <tr>
      <td>Jarak (km)</td>
      <td class="${same ? 'td-win' : 'td-d'}">${rD.distance}</td>
      <td class="${same ? 'td-win' : 'td-a'}">${rA.distance}</td>
      ${hasB ? `<td class="td-b">${rB.distance}</td>` : "<td>—</td>"}
      <td class="td-win">${same ? "Sama" : "—"}</td>
    </tr>
    <tr>
      <td>Waktu (ms)</td>
      <td class="${dWT ? 'td-win' : 'td-d'}">${rD.timeMs}</td>
      <td class="${!dWT ? 'td-win' : 'td-a'}">${rA.timeMs}</td>
      ${hasB ? `<td class="td-b">${rB.timeMs}</td>` : "<td>—</td>"}
      <td class="td-win">${dWT ? "Dijkstra" : "A*"}</td>
    </tr>
    <tr>
      <td>Node dikunjungi</td>
      <td class="${dWN ? 'td-win' : 'td-d'}">${rD.visited.length}</td>
      <td class="${!dWN ? 'td-win' : 'td-a'}">${rA.visited.length}</td>
      ${hasB ? `<td class="td-b">${rB.visited.length}</td>` : "<td>—</td>"}
      <td class="td-win">${dWN ? "Dijkstra" : "A*"}</td>
    </tr>
    <tr>
      <td>Hop</td>
      <td class="td-d">${rD.path.length - 1}</td>
      <td class="td-a">${rA.path.length - 1}</td>
      ${hasB ? `<td class="td-b">${rB.path.length - 1}</td>` : "<td>—</td>"}
      <td class="td-win">${rD.path.length <= rA.path.length ? "Dijkstra" : "A*"}</td>
    </tr>
    <tr>
      <td>Kompleksitas</td>
      <td class="td-d" style="font-size:9px">O((V+E)logV)</td>
      <td class="td-a" style="font-size:9px">O(E log V)</td>
      ${hasB ? `<td class="td-b" style="font-size:9px">O(V+E)</td>` : "<td>—</td>"}
      <td>—</td>
    </tr>`;
  document.getElementById("cmp-table").classList.add("show");

  // evaluasi kelebihan & kekurangan
  const winner = (dWT ? 1 : 0) + (dWN ? 1 : 0) >= 1 ? "Dijkstra" : "A*";
  const eb = document.getElementById("eval-box");
  eb.style.display = "block";
  eb.innerHTML = `
    <div class="eval-title">📊 Evaluasi Algoritma</div>
    <div class="eval-row">
      <span class="eval-algo dijkstra">Dijkstra</span>
      <span>${rD.visited.length} node · ${rD.timeMs}ms · ${rD.distance}km<br>
      <span class="eval-note">
        ✅ Strategi Greedy — selalu pilih node berbobot terkecil<br>
        ✅ Optimal — jamin rute terpendek (bobot ≥ 0)<br>
        ✅ Tidak butuh heuristik<br>
        ❌ Exhaustive — kunjungi semua node yang mungkin
      </span></span>
    </div>
    <div class="eval-row">
      <span class="eval-algo astar">A-Star</span>
      <span>${rA.visited.length} node · ${rA.timeMs}ms · ${rA.distance}km<br>
      <span class="eval-note">
        ✅ Optimal jika h(n) admissible<br>
        ✅ Terarah — kunjungi lebih sedikit node dari Dijkstra<br>
        ❌ Butuh fungsi heuristik yang baik
      </span></span>
    </div>
    ${hasB ? `
    <div class="eval-row">
      <span class="eval-algo bfs">BFS</span>
      <span>${rB.visited.length} node · ${rB.timeMs}ms · ${rB.distance}km<br>
      <span class="eval-note">
        ✅ Sederhana — O(V+E), tidak butuh bobot<br>
        ❌ Abaikan bobot edge → jarak bisa TIDAK optimal<br>
        ❌ Hanya optimal untuk graf tanpa bobot
      </span></span>
    </div>` : ""}
    <div class="eval-conclusion">
      🏆 <b>${winner}</b> paling efisien pada rute ini.<br>
      ${same
      ? "Dijkstra dan A* menghasilkan jarak yang <b>identik</b>."
      : rD.distance < rA.distance
        ? "Dijkstra menghasilkan jarak lebih pendek."
        : "A* menghasilkan jarak lebih pendek."
    }
      A* mengunjungi <b>${Math.abs(rA.visited.length - rD.visited.length)}</b> node
      ${rA.visited.length < rD.visited.length ? "lebih sedikit" : "lebih banyak"}
      dibanding Dijkstra.
    </div>`;

  document.getElementById("winner-title").textContent = winner + " lebih efisien pada rute ini";
  document.getElementById("winner-sub").textContent =
    `Selisih waktu ${ratio}× · Jarak ${same ? "identik" : "berbeda"} · ${winner} kunjungi lebih sedikit node`;
  document.getElementById("winner-badge").classList.add("show");
  document.getElementById(winner === "Dijkstra" ? "card-d" : "card-a").classList.add("win-card");

  lg(`[EVALUASI] Winner: ${winner} · Waktu ${ratio}× · Jarak ${same ? "sama" : "beda"}`, "ok");
}

// ══════════════════════════════════════════════════════════════
//  MAIN SIMULATION — Integrasi Ketiga Algoritma
//  ─────────────────────────────────────────────────────────────
//  @contributor  Raihan Darma Putra
//                (startSimulation(), integrasi BFS + Dijkstra + A*,
//                 timing eksekusi, logging, kurir bergerak)
//
//  Urutan eksekusi:
//    1. BFS     → baseline, animateVisited (oranye)
//    2. Dijkstra → greedy optimal, animateVisited (biru)
//    3. A*       → informed search, animateVisited (ungu)
//    4. showResults() → evaluasi & perbandingan
//    5. moveCar() → kurir bergerak di jalur Dijkstra
// ══════════════════════════════════════════════════════════════
async function startSimulation() {
  if (simRunning) return;
  const start = document.getElementById("start").value;
  const end = document.getElementById("end").value;
  if (start === end) { alert("Start dan tujuan tidak boleh sama!"); return; }

  simRunning = true;
  document.getElementById("run-btn").disabled = true;
  resetSimVisuals();
  clearLog();
  resetResults();

  lg(`[SIM] ${start} → ${end} | ${Object.keys(graph).length} node, (graph)} edge`, "inf");
  lgSep();

  // ── BFS ────────────────────────────────────────────────────
  lg("[BFS] Kompleksitas O(V+E). Tidak pakai bobot.", "bfslog");
  const t0b = performance.now();
  const rB = bfs(graph, start, end);
  rB.timeMs = +(performance.now() - t0b).toFixed(4);

  if (!rB.path.length) {
    lg(`  ✗ Tidak ada rute ke ${end}`, "warn");
  } else {
    rB.order.forEach(o => lg(`  Step ${o.step}: kunjungi ${o.node}`, "bfslog"));
    lg(`  ✓ ${rB.path.join("→")} | ${rB.distance}km | ${rB.visited.length} node | ${rB.timeMs}ms`, "ok");
  }
  await animateVisited(rB.order, "visited-bfs-layer", "#f59e0b", "rgba(245,158,11,.1)", "#f59e0b", "bfs");
  if (rB.path.length) drawFinalPath(rB.path, "path-bfs", "0.45");
  lgSep();

  // ── DIJKSTRA ───────────────────────────────────────────────
  lg(`[DIJKSTRA] Strategi: GREEDY. dist[${start}]=0, semua=∞. Min-Heap O(log V).`, "dijk");
  const t0d = performance.now();
  const rD = dijkstra(graph, start, end);
  rD.timeMs = +(performance.now() - t0d).toFixed(4);

  if (!rD.path.length) {
    lg(`  ✗ Tidak ada rute ke ${end}`, "warn");
  } else {
    rD.order.forEach(o => lg(`  Step ${o.step}: ${o.node}  dist=${o.dist}`, "dijk"));
    lg(`  ✓ ${rD.path.join("→")} | ${rD.distance}km | ${rD.visited.length} node | ${rD.timeMs}ms`, "ok");
  }
  document.getElementById("node-labels-layer").innerHTML = "";
  await animateVisited(rD.order, "visited-d-layer", "#1565c0", "rgba(21,101,192,.12)", "#60a5fa", "dijkstra");
  if (rD.path.length) drawFinalPath(rD.path, "path-dijkstra", "1");
  lgSep();

  // ── A* ─────────────────────────────────────────────────────
  lg("[A-STAR] f(n)=g(n)+h(n). h=Euclidean×0.40 (admissible).", "astr");
  const t0a = performance.now();
  const rA = astar(graph, start, end);
  rA.timeMs = +(performance.now() - t0a).toFixed(4);

  if (!rA.path.length) {
    lg(`  ✗ Tidak ada rute ke ${end}`, "warn");
  } else {
    rA.order.forEach(o => lg(`  Step ${o.step}: ${o.node}  g=${o.g.toFixed(0)} h=${o.h.toFixed(0)} f=${o.f.toFixed(0)}`, "astr"));
    lg(`  ✓ ${rA.path.join("→")} | ${rA.distance}km | ${rA.visited.length} node | ${rA.timeMs}ms`, "ok");
  }
  document.getElementById("node-labels-layer").innerHTML = "";
  await animateVisited(rA.order, "visited-a-layer", "#6a1b9a", "rgba(106,27,154,.12)", "#c084fc", "astar");
  if (rA.path.length) drawFinalPath(rA.path, "path-astar", "0.85");
  lgSep();

  // ── Hasil & evaluasi ───────────────────────────────────────
  showResults(rD, rA, rB);

  const r = rD.path.length > 0 ? rD : (rA.path.length > 0 ? rA : null);
  if (r) {
    document.getElementById("result").innerHTML =
      `📦 <b>Paket terkirim! Rute (Dijkstra):</b> ${rD.path.length > 0 ? rD.path.join(" ➔ ") : "—"} &nbsp;|&nbsp; 📏 ${rD.path.length > 0 ? rD.distance : "—"}km` +
      `<span style="font-size:12px;color:#555;"> · D:${rD.timeMs}ms · A*:${rA.timeMs}ms · BFS:${rB.timeMs}ms</span>`;
  }

  // ── Mobil bergerak di atas jalan ───────────────────────────
  const best = rD.path.length > 0 ? rD.path : (rA.path.length > 0 ? rA.path : null);
  if (best) {
    const bestDist = rD.path.length > 0 ? rD.distance : rA.distance;
    lg(`[KURIR] Mengantarkan paket: ${best.join("→")}`, "inf");
    await moveCar(best, bestDist);
    lg("[KURIR] Paket berhasil diantarkan ke tujuan! ✅", "ok");
  }

  simRunning = false;
  document.getElementById("run-btn").disabled = false;
}

// ── Broken roads (Jalan Rusak) ────────────────────────────────
// @contributor  Elsha Natalia Panjaitan
//               (generateBrokenRoads() — memblokir 3 edge acak
//                dengan menambahkan bobot +9999, memaksa algoritma
//                mencari jalur alternatif)
function generateBrokenRoads() {
  resetRoads();
  const all = [];
  for (const u in originalGraph) {
    for (const e of originalGraph[u]) {
      const p = [u, e.node].sort().join("-");
      if (!all.find(r => r.id === p)) all.push({ id: p, from: u, to: e.node });
    }
  }
  brokenRoads = all.sort(() => Math.random() - 0.5).slice(0, 3);
  const bl = document.getElementById("broken-roads-layer");
  for (const road of brokenRoads) {
    const ef = graph[road.from]?.find(e => e.node === road.to);
    const et = graph[road.to]?.find(e => e.node === road.from);
    if (ef) ef.weight += 9999;
    if (et) et.weight += 9999;
    const p1 = positions[road.from], p2 = positions[road.to];
    bl.innerHTML += `<path d="M ${p1.x} ${p1.y} ${getNaturalPath(p1, p2, road.from, road.to)}" class="broken-road"/>`;
  }
  document.getElementById("result").innerHTML = "⚠️ 3 jalan rusak! Kurir mencari jalur pengiriman alternatif.";
  clearLog();
  lg("[JALAN RUSAK] 3 edge diblokir (bobot + 9999)", "warn");
  brokenRoads.forEach(r => lg(`  × ${r.from}—${r.to}`, "warn"));
}

function resetRoads() {
  graph = JSON.parse(JSON.stringify(originalGraph));
  brokenRoads = [];
  document.getElementById("broken-roads-layer").innerHTML = "";
  document.getElementById("result").innerHTML = "✅ Semua jalan normal. Kurir siap mengantar paket!";
  drawWeightLabels();
}

// ── Randomize map (Acak Peta) ────────────────────────────────
// @contributor  Yehezkiel Alman
//               (randomizeMap() — generate graf acak 7-11 node,
//                posisi node non-overlap, edge non-crossing,
//                Prim spanning tree untuk konektivitas,
//                acak start & end untuk anti-hardcode)
function randomizeMap() {
  const W = 1000, H = 650, PAD = 75, MIN_D = 130, MAX_E = 430;
  const nc = ri(7, 11);
  const names = "ABCDEFGHIJKLMNOPQRSTUVWXYZ".slice(0, nc).split("");
  const newPos = {};

  for (const nm of names) {
    let placed = false;
    for (let t = 0; t < 600 && !placed; t++) {
      const px = rnd(PAD, W - PAD), py = rnd(PAD, H - PAD);
      let ok = true;
      for (const ep of Object.values(newPos))
        if (Math.hypot(px - ep.x, py - ep.y) < MIN_D) { ok = false; break; }
      if (ok) { newPos[nm] = { x: Math.round(px), y: Math.round(py) }; placed = true; }
    }
    if (!newPos[nm]) {
      const idx = Object.keys(newPos).length;
      newPos[nm] = {
        x: Math.round(PAD + (idx % 4) * (W - PAD * 2) / 3),
        y: Math.round(PAD + Math.floor(idx / 4) * (H - PAD * 2) / 2)
      };
    }
  }

  const nG = {};
  for (const n of names) nG[n] = [];
  const ePts = [], ap = new Set();

  function tryAdd(u, v, mandatory) {
    const pair = [u, v].sort().join("-");
    if (ap.has(pair)) return false;
    const p1 = newPos[u], p2 = newPos[v];
    const d = Math.hypot(p2.x - p1.x, p2.y - p1.y);
    if (!mandatory && d > MAX_E) return false;
    if (!mandatory && edgeCrosses(ePts, p1, p2, u, v)) return false;
    const w = Math.max(50, Math.round(d * 0.4 + rnd(-25, 25)));
    nG[u].push({ node: v, weight: w });
    nG[v].push({ node: u, weight: w });
    ePts.push({ pts: buildEdgePts(p1, p2, u, v, 10), from: u, to: v });
    ap.add(pair);
    return true;
  }

  // Prim spanning tree → garansi semua node terhubung
  const inT = new Set([names[0]]), rem = new Set(names.slice(1));
  while (rem.size > 0) {
    let bd = Infinity, bu = null, bv = null;
    for (const u of inT)
      for (const v of rem) {
        const d = Math.hypot(newPos[u].x - newPos[v].x, newPos[u].y - newPos[v].y);
        if (d < bd && !edgeCrosses(ePts, newPos[u], newPos[v], u, v)) { bd = d; bu = u; bv = v; }
      }
    // fallback jika semua crossing (tetap harus connect)
    if (!bu) {
      for (const u of inT)
        for (const v of rem) {
          const d = Math.hypot(newPos[u].x - newPos[v].x, newPos[u].y - newPos[v].y);
          if (d < bd) { bd = d; bu = u; bv = v; }
        }
    }
    if (!bu) break;
    tryAdd(bu, bv, true);
    inT.add(bv); rem.delete(bv);
  }

  // extra edges acak (non-crossing)
  const extra = Math.floor(nc * 0.55), allP = [];
  for (let i = 0; i < names.length; i++)
    for (let j = i + 1; j < names.length; j++)
      allP.push([names[i], names[j]]);
  allP.sort(() => Math.random() - 0.5);
  let added = 0;
  for (const [u, v] of allP) {
    if (added >= extra) break;
    if (tryAdd(u, v, false)) added++;
  }

  // acak start & end
  const shuffled = [...names].sort(() => Math.random() - 0.5);
  const rs = shuffled[0], re = shuffled[1];

  originalGraph = JSON.parse(JSON.stringify(nG));
  graph = JSON.parse(JSON.stringify(nG));
  positions = newPos;
  brokenRoads = [];

  populateSelects();
  document.getElementById("start").value = rs;
  document.getElementById("end").value = re;
  drawMap(true, ePts);

  document.getElementById("result").innerHTML =
    `🎲 Peta diacak! <b>${nc} kota, ${ap.size} jalan</b>. Kurir dari: <b>${rs}</b> → <b>${re}</b>`;
  clearLog();
  lg(`[ACAK] ${nc} kota · ${ap.size} jalan · ${rs}→${re}`, "inf");
}

function countEdges(g) {
  let c = 0; const s = new Set();
  for (const u in g)
    for (const e of g[u]) {
      const p = [u, e.node].sort().join("-");
      if (!s.has(p)) { s.add(p); c++; }
    }
  return c;
}

// ── Init ──────────────────────────────────────────────────────
document.addEventListener("DOMContentLoaded", () => {
  populateSelects();
  drawMap(false);
  document.getElementById("start").addEventListener("change", () => { resetSimVisuals(); updateFlags(); });
  document.getElementById("end").addEventListener("change", () => { resetSimVisuals(); updateFlags(); });
});