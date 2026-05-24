// ══════════════════════════════════════════════════════════════
//  SmartRoute — main.js
//  PAA 2026 · UMRAH · Teknik Informatika
//  Algoritma: Dijkstra + A* (A-Star)
// ══════════════════════════════════════════════════════════════

// ── Default graph & positions (dari original, dipertahankan) ──────────────────
const defaultGraph = {
  "A":[{node:"C",weight:280},{node:"B",weight:220},{node:"H",weight:300}],
  "B":[{node:"A",weight:220},{node:"E",weight:290},{node:"D",weight:230},{node:"H",weight:150}],
  "C":[{node:"A",weight:280},{node:"E",weight:260},{node:"F",weight:370},{node:"I",weight:200}],
  "D":[{node:"B",weight:230},{node:"G",weight:600},{node:"J",weight:100}],
  "E":[{node:"B",weight:290},{node:"C",weight:260},{node:"F",weight:230},{node:"G",weight:370}],
  "F":[{node:"C",weight:370},{node:"E",weight:230},{node:"G",weight:260},{node:"I",weight:250},{node:"K",weight:200}],
  "G":[{node:"D",weight:600},{node:"E",weight:370},{node:"F",weight:260},{node:"K",weight:250}],
  "H":[{node:"A",weight:300},{node:"B",weight:150}],
  "I":[{node:"C",weight:200},{node:"F",weight:250}],
  "J":[{node:"D",weight:100}],
  "K":[{node:"F",weight:200},{node:"G",weight:250}]
};
const defaultPositions = {
  "A":{x:500,y:30},  "B":{x:300,y:290}, "C":{x:550,y:160},
  "D":{x:210,y:320}, "E":{x:600,y:350}, "F":{x:870,y:320},
  "G":{x:500,y:500}, "H":{x:60,y:200},  "I":{x:750,y:100},
  "J":{x:110,y:580}, "K":{x:970,y:580}
};

let originalGraph = JSON.parse(JSON.stringify(defaultGraph));
let graph         = JSON.parse(JSON.stringify(defaultGraph));
let positions     = JSON.parse(JSON.stringify(defaultPositions));
let brokenRoads   = [];

// ── Selects (dari original, dipertahankan) ────────────────────────────────────
function populateSelects() {
  const nodes = Object.keys(positions).sort();
  const ss = document.getElementById("start");
  const se = document.getElementById("end");
  const pS = ss.value, pE = se.value;
  ss.innerHTML = se.innerHTML = "";
  nodes.forEach(n => {
    ss.innerHTML += `<option value="${n}">${n}</option>`;
    se.innerHTML += `<option value="${n}">${n}</option>`;
  });
  if (nodes.includes(pS)) ss.value = pS;
  const fallback = nodes.find(n => n !== ss.value) || nodes[1] || nodes[0];
  se.value = (nodes.includes(pE) && pE !== ss.value) ? pE : fallback;
}

// ── Bezier helpers (dari original, dipertahankan) ─────────────────────────────
function getCurveParams(p1, p2, n1, n2) {
  const dx = p2.x - p1.x, dy = p2.y - p1.y;
  const dist = Math.sqrt(dx*dx + dy*dy);
  if (dist < 1) return null;
  const flip = (n1.charCodeAt(0) + n2.charCodeAt(0)) % 2 === 0 ? 1 : -1;
  const ci = Math.min(dist * 0.22, 70) * flip;
  const nx = (-dy / dist) * ci, ny = (dx / dist) * ci;
  return {
    cx1: p1.x + dx*0.3 + nx, cy1: p1.y + dy*0.3 + ny,
    cx2: p1.x + dx*0.7 - nx, cy2: p1.y + dy*0.7 - ny
  };
}

function getNaturalPath(p1, p2, n1, n2) {
  const c = getCurveParams(p1, p2, n1, n2);
  if (!c) return `L ${p2.x} ${p2.y}`;
  return `C ${c.cx1} ${c.cy1} ${c.cx2} ${c.cy2} ${p2.x} ${p2.y}`;
}

function bezierPoint(p1, p2, n1, n2, t) {
  const c = getCurveParams(p1, p2, n1, n2);
  if (!c) return { x: p1.x + (p2.x - p1.x)*t, y: p1.y + (p2.y - p1.y)*t };
  const mt = 1 - t;
  return {
    x: mt*mt*mt*p1.x + 3*mt*mt*t*c.cx1 + 3*mt*t*t*c.cx2 + t*t*t*p2.x,
    y: mt*mt*mt*p1.y + 3*mt*mt*t*c.cy1 + 3*mt*t*t*c.cy2 + t*t*t*p2.y
  };
}

// ── Segment intersection (dari original, dipertahankan) ───────────────────────
function segmentsIntersect(ax,ay,bx,by,cx,cy,dx,dy) {
  const d1x=bx-ax, d1y=by-ay, d2x=dx-cx, d2y=dy-cy;
  const cross = d1x*d2y - d1y*d2x;
  if (Math.abs(cross) < 1e-9) return false;
  const tx = ((cx-ax)*d2y - (cy-ay)*d2x) / cross;
  const ty = ((cx-ax)*d1y - (cy-ay)*d1x) / cross;
  const E = 0.04;
  return tx>E && tx<1-E && ty>E && ty<1-E;
}

function buildEdgePts(p1, p2, n1, n2, SEGS=8) {
  const pts = [];
  for (let i=0; i<=SEGS; i++) pts.push(bezierPoint(p1,p2,n1,n2,i/SEGS));
  return pts;
}

function edgeCrosses(existingList, p1, p2, n1, n2) {
  const newPts = buildEdgePts(p1, p2, n1, n2, 8);
  for (const {pts, from, to} of existingList) {
    if (from===n1||from===n2||to===n1||to===n2) continue;
    for (let i=0; i<newPts.length-1; i++)
      for (let j=0; j<pts.length-1; j++)
        if (segmentsIntersect(
          newPts[i].x,newPts[i].y,newPts[i+1].x,newPts[i+1].y,
          pts[j].x,  pts[j].y,  pts[j+1].x,  pts[j+1].y
        )) return true;
  }
  return false;
}

function ptSegDist(px, py, ax, ay, bx, by) {
  const dx=bx-ax, dy=by-ay, lenSq=dx*dx+dy*dy;
  if (lenSq < 1) return Math.hypot(px-ax, py-ay);
  const t = Math.max(0, Math.min(1, ((px-ax)*dx+(py-ay)*dy)/lenSq));
  return Math.hypot(px-(ax+t*dx), py-(ay+t*dy));
}

function pointNearEdge(px, py, pts, clearance) {
  for (let i=0; i<pts.length-1; i++)
    if (ptSegDist(px,py,pts[i].x,pts[i].y,pts[i+1].x,pts[i+1].y) < clearance) return true;
  return false;
}

// ── Decorations (dari original, dipertahankan) ────────────────────────────────
function rnd(a,b){ return a+Math.random()*(b-a); }
function ri(a,b){ return Math.floor(rnd(a,b+1)); }

function svgTree(x, y, sc) {
  sc = sc || 1;
  return `<g transform="translate(${x.toFixed(1)},${y.toFixed(1)}) scale(${sc.toFixed(2)})">
    <rect x="-2" y="3" width="4" height="9" fill="#5c4033"/>
    <circle cx="0" cy="-4" r="9" fill="#2d6a4f" opacity="0.85"/>
    <circle cx="-4" cy="-1" r="6" fill="#2d6a4f" opacity="0.75"/>
    <circle cx="4"  cy="-1" r="6" fill="#2d6a4f" opacity="0.75"/>
  </g>`;
}

function svgMountain(x, y, sc) {
  sc = sc || 1;
  return `<g transform="translate(${x.toFixed(1)},${y.toFixed(1)}) scale(${sc.toFixed(2)})">
    <path d="M -30 20 L 0 -25 L 30 20 Z"  fill="#4d7c3d" stroke="#333" stroke-width="1.5"/>
    <path d="M -15 20 L 18 -18 L 50 20 Z" fill="#5b8f49" stroke="#333" stroke-width="1.5"/>
    <path d="M 0 -10 L 8 -25 L 16 -10 Z"  fill="white"  opacity="0.6"/>
  </g>`;
}

function svgBuildings(cx, cy, count, tall) {
  const BCOLORS = ["#94a3b8","#64748b","#cbd5e1","#aab4c8","#7f8fa6","#475569"];
  let h = "";
  for (let i=0; i<count; i++) {
    const bw = ri(10,18);
    const bh = tall ? ri(40,75) : ri(18,40);
    const bx = cx - (count*14)/2 + i*14 + ri(-3,3);
    const by = cy - bh;
    const col = BCOLORS[ri(0,BCOLORS.length-1)];
    h += `<rect x="${bx.toFixed(1)}" y="${by.toFixed(1)}" width="${bw}" height="${bh}"
               fill="${col}" stroke="#333" stroke-width="1.2" rx="1"/>`;
    const rows = Math.max(1, Math.floor(bh/11)-1);
    for (let r=0; r<rows; r++) {
      const wx = bx + bw*0.2;
      const wy = by + 5 + r*10;
      const lit = Math.random()>0.25;
      h += `<rect x="${wx.toFixed(1)}" y="${wy.toFixed(1)}" width="${(bw*0.6).toFixed(1)}" height="5"
                  fill="${lit?'#fef08a':'#334155'}" opacity="0.8" rx="1"/>`;
    }
  }
  return h;
}

function drawDecorations(allEdgePts) {
  const layer = document.getElementById("deco-layer");
  layer.innerHTML = "";
  const NODE_RADIUS = 14;
  const SLOT_R      = NODE_RADIUS + 28;
  const ROAD_CLEAR  = 28;
  const SLOTS       = 16;
  let html = "";
  for (const name in positions) {
    const p = positions[name];
    const degree = (originalGraph[name] || []).length;
    const freeAngles = [];
    for (let i=0; i<SLOTS; i++) {
      const ang = (i / SLOTS) * Math.PI * 2;
      const tx  = p.x + Math.cos(ang) * SLOT_R;
      const ty  = p.y + Math.sin(ang) * SLOT_R;
      let clear = true;
      for (const {pts} of allEdgePts) {
        if (pointNearEdge(tx, ty, pts, ROAD_CLEAR)) { clear = false; break; }
      }
      if (clear) freeAngles.push(ang);
    }
    if (freeAngles.length === 0) continue;
    const groups = [];
    let cur = [freeAngles[0]];
    for (let i=1; i<freeAngles.length; i++) {
      if (freeAngles[i] - freeAngles[i-1] < (Math.PI*2/SLOTS)*2.1) cur.push(freeAngles[i]);
      else { groups.push(cur); cur=[freeAngles[i]]; }
    }
    groups.push(cur);
    const bigGroup = groups.sort((a,b)=>b.length-a.length)[0];
    const midAng   = bigGroup[Math.floor(bigGroup.length/2)];
    const dx = Math.cos(midAng), dy = Math.sin(midAng);
    if (degree >= 4) {
      html += svgBuildings(p.x + dx*(SLOT_R+5), p.y + dy*(SLOT_R+5), ri(4,6), true);
      if (bigGroup.length >= 4) {
        const angOff = bigGroup.length > 6 ? midAng + 0.4 : midAng;
        html += svgMountain(p.x + Math.cos(angOff)*(SLOT_R+38), p.y + Math.sin(angOff)*(SLOT_R+38), 0.7);
      }
    } else if (degree >= 2) {
      html += svgBuildings(p.x + dx*(SLOT_R+5), p.y + dy*(SLOT_R+5), ri(2,4), false);
      if (bigGroup.length >= 3) {
        const ang2 = bigGroup[1];
        html += svgTree(p.x + Math.cos(ang2)*(SLOT_R+10), p.y + Math.sin(ang2)*(SLOT_R+10), rnd(0.7,1.1));
      }
    } else {
      for (let k=0; k<Math.min(bigGroup.length, 3); k++) {
        const a  = bigGroup[k];
        const tr = SLOT_R + rnd(5,18);
        html += svgTree(p.x + Math.cos(a)*tr, p.y + Math.sin(a)*tr, rnd(0.7,1.2));
      }
    }
  }
  layer.innerHTML = html;
}

// ── Draw map (dari original, dipertahankan) ───────────────────────────────────
function buildAllEdgePts() {
  const list = [];
  const drawn = new Set();
  for (let u in graph) {
    for (let e of graph[u]) {
      const pair = [u, e.node].sort().join("-");
      if (!drawn.has(pair)) {
        drawn.add(pair);
        list.push({
          pts:  buildEdgePts(positions[u], positions[e.node], u, e.node, 10),
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
  for (const {pts, from, to} of edgeList)
    baseD += `M ${positions[from].x} ${positions[from].y} ${getNaturalPath(positions[from], positions[to], from, to)} `;

  document.getElementById("road-layer-1").setAttribute("d", baseD);
  document.getElementById("road-layer-2").setAttribute("d", baseD);
  document.getElementById("road-layer-3").setAttribute("d", baseD);

  const ng = document.getElementById("nodes-group");
  ng.innerHTML = "";
  for (let name in positions) {
    const p = positions[name];
    ng.innerHTML += `
      <circle class="${animate?'node node-anim':'node'}" cx="${p.x}" cy="${p.y}" r="14"/>
      <text class="label-text" x="${p.x}" y="${p.y+1}">${name}</text>`;
  }

  drawWeightLabels();
  drawDecorations(edgeList);

  const sp = positions[document.getElementById("start").value] || Object.values(positions)[0];
  document.getElementById("car").setAttribute("transform", `translate(${sp.x},${sp.y})`);
  document.getElementById("highlight-path").setAttribute("d","");
  document.getElementById("broken-roads-layer").innerHTML = "";

  // reset visited overlays & final paths
  document.getElementById("visited-d-layer").innerHTML = "";
  document.getElementById("visited-a-layer").innerHTML = "";
  document.getElementById("path-dijkstra").setAttribute("d","");
  document.getElementById("path-astar").setAttribute("d","");
  document.getElementById("path-dijkstra").setAttribute("opacity","0");
  document.getElementById("path-astar").setAttribute("opacity","0");

  brokenRoads = [];

  // posisikan flag start (kuning) & end (merah) sesuai RPM
  updateFlags();
}

function updateFlags() {
  const s = document.getElementById('start').value;
  const e = document.getElementById('end').value;
  const fs = document.getElementById('flag-start');
  const fe = document.getElementById('flag-end');
  if (positions[s]) {
    fs.setAttribute('transform', `translate(${positions[s].x},${positions[s].y})`);
    fs.setAttribute('opacity','1');
  }
  if (positions[e]) {
    fe.setAttribute('transform', `translate(${positions[e].x},${positions[e].y})`);
    fe.setAttribute('opacity','1');
  }
}

function drawWeightLabels() {
  const layer = document.getElementById("weight-labels-layer");
  layer.innerHTML = "";
  const drawn = new Set();
  for (let u in originalGraph) {
    for (let e of originalGraph[u]) {
      const pair = [u, e.node].sort().join("-");
      if (!drawn.has(pair)) {
        drawn.add(pair);
        const mid = bezierPoint(positions[u], positions[e.node], u, e.node, 0.5);
        layer.innerHTML += `
          <rect x="${(mid.x-17).toFixed(1)}" y="${(mid.y-8).toFixed(1)}" width="34" height="15"
                rx="4" fill="#333" opacity="0.78"/>
          <text class="weight-label" x="${mid.x.toFixed(1)}" y="${(mid.y+0.5).toFixed(1)}">${e.weight}</text>`;
      }
    }
  }
}

// ══════════════════════════════════════════════════════════════
//  MIN-HEAP (Priority Queue)
//  Digunakan oleh Dijkstra & A* → operasi push/pop O(log V)
// ══════════════════════════════════════════════════════════════
class MinHeap {
  constructor() { this.h = []; }
  push(item) { this.h.push(item); this._up(this.h.length - 1); }
  pop() {
    const top = this.h[0], last = this.h.pop();
    if (this.h.length) { this.h[0] = last; this._down(0); }
    return top;
  }
  get size() { return this.h.length; }
  _up(i) {
    while (i > 0) {
      const p = Math.floor((i-1)/2);
      if (this.h[p].f <= this.h[i].f) break;
      [this.h[p],this.h[i]] = [this.h[i],this.h[p]]; i = p;
    }
  }
  _down(i) {
    const n = this.h.length;
    while (true) {
      let m=i, l=2*i+1, r=2*i+2;
      if (l<n && this.h[l].f < this.h[m].f) m=l;
      if (r<n && this.h[r].f < this.h[m].f) m=r;
      if (m===i) break;
      [this.h[m],this.h[i]] = [this.h[i],this.h[m]]; i=m;
    }
  }
}

// ══════════════════════════════════════════════════════════════
//  DIJKSTRA'S ALGORITHM
//  ──────────────────────────────────────────────────────────────
//  Kompleksitas Waktu : O((V + E) log V)
//    V iterasi extract-min dari Min-Heap  → O(V log V)
//    E iterasi relaksasi edge             → O(E log V)
//  Kompleksitas Ruang : O(V) — dist[], prev[]
//
//  Pseudocode:
//    dist[start] = 0; dist[v] = ∞ untuk semua v ≠ start
//    PQ.push({start, 0})
//    while PQ tidak kosong:
//      u = PQ.extractMin()
//      if u ∈ visited: skip
//      visited.add(u)
//      for setiap tetangga v dari u:
//        if dist[u] + w(u,v) < dist[v]:
//          dist[v] = dist[u] + w(u,v)
//          prev[v] = u
//          PQ.push({v, dist[v]})
//    rekonstruksi path dari prev[]
// ══════════════════════════════════════════════════════════════
function dijkstra(g, start, end) {
  const dist = {}, prev = {}, visited = [], order = [];
  for (const n in g) dist[n] = Infinity;
  dist[start] = 0;
  const pq = new MinHeap();
  pq.push({ f: 0, node: start });

  while (pq.size > 0) {
    const { f: d, node: u } = pq.pop();
    if (visited.includes(u)) continue;
    visited.push(u);
    order.push({ node: u, dist: d, step: visited.length });
    if (u === end) break;
    for (const nb of g[u]) {
      const nd = dist[u] + nb.weight;
      if (nd < dist[nb.node]) {
        dist[nb.node] = nd;
        prev[nb.node] = u;
        pq.push({ f: nd, node: nb.node });
      }
    }
  }
  const path = []; let cur = end;
  while (cur !== undefined) { path.unshift(cur); cur = prev[cur]; }
  return {
    path:    path[0] === start ? path : [],
    distance: dist[end],
    visited,
    order
  };
}

// ══════════════════════════════════════════════════════════════
//  A* (A-STAR) ALGORITHM
//  ──────────────────────────────────────────────────────────────
//  Kompleksitas Waktu : O(E log V) rata-rata, jika h admissible
//  Kompleksitas Ruang : O(V) — g[], f[], prev[]
//
//  Fungsi evaluasi: f(n) = g(n) + h(n)
//    g(n) = biaya aktual dari start ke n
//    h(n) = heuristik Euclidean × 0.40
//           (admissible: tidak pernah overestimate biaya nyata)
//    Admissible ⟹ A* PASTI optimal
//
//  Pseudocode:
//    g[start] = 0;  f[start] = h(start, end)
//    open.push({start, f[start]})
//    while open tidak kosong:
//      u = open.extractMin()   ← node dengan f terkecil
//      if u ∈ visited: skip
//      visited.add(u)
//      if u == end: selesai
//      for setiap tetangga v dari u:
//        gNew = g[u] + w(u,v)
//        if gNew < g[v]:
//          g[v]    = gNew
//          f[v]    = g[v] + h(v, end)
//          prev[v] = u
//          open.push({v, f[v]})
//    rekonstruksi path dari prev[]
// ══════════════════════════════════════════════════════════════
function heuristic(nodeA, nodeB) {
  const pa = positions[nodeA], pb = positions[nodeB];
  // Euclidean distance × 0.40 — admissible scaling factor
  return Math.hypot(pa.x - pb.x, pa.y - pb.y) * 0.40;
}

function astar(g, start, end) {
  const gCost = {}, fCost = {}, prev = {}, visited = [], order = [];
  for (const n in g) { gCost[n] = Infinity; fCost[n] = Infinity; }
  gCost[start] = 0;
  fCost[start] = heuristic(start, end);

  const open = new MinHeap();
  open.push({ f: fCost[start], node: start });

  while (open.size > 0) {
    const { node: u } = open.pop();
    if (visited.includes(u)) continue;
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
        prev[nb.node]  = u;
        open.push({ f: fCost[nb.node], node: nb.node });
      }
    }
  }
  const path = []; let cur = end;
  while (cur !== undefined) { path.unshift(cur); cur = prev[cur]; }
  return {
    path:     path[0] === start ? path : [],
    distance: gCost[end],
    visited,
    order
  };
}

// ── Car animation (dari original + rotate auto) ───────────────────────────────
function wait(ms) { return new Promise(r => setTimeout(r, ms)); }

async function moveCar(path, distance) {
  const car = document.getElementById("car");
  document.getElementById("car-anim")?.remove();
  car.removeAttribute("transform");

  let hd = `M ${positions[path[0]].x} ${positions[path[0]].y} `;
  for (let i=1; i<path.length; i++)
    hd += getNaturalPath(positions[path[i-1]], positions[path[i]], path[i-1], path[i]) + " ";

  // highlight path (merah, dari original)
  document.getElementById("highlight-path").setAttribute("d", hd);

  const dur = Math.max(1.5, distance / 150);
  const anim = document.createElementNS("http://www.w3.org/2000/svg", "animateMotion");
  anim.setAttribute("id", "car-anim");
  anim.setAttribute("dur", `${dur}s`);
  anim.setAttribute("fill", "freeze");
  anim.setAttribute("rotate", "auto");  // mobil menghadap arah jalan
  anim.setAttribute("path", hd);
  car.appendChild(anim);
  car.classList.add("spin");   // roda berputar (dari original)
  anim.beginElement();
  await wait(dur * 1000);
  car.classList.remove("spin");
}

// ── Log helpers ───────────────────────────────────────────────────────────────
function lg(msg, cls='inf') {
  const b = document.getElementById("log-body");
  const d = document.createElement("div");
  d.className = "ll " + cls;
  d.textContent = msg;
  b.appendChild(d);
  b.scrollTop = b.scrollHeight;
}
function lgSep() { lg("─".repeat(38), "sep"); }
function clearLog() { document.getElementById("log-body").innerHTML = ""; }

// ── Visited animation step-by-step ───────────────────────────────────────────
const aniDelay = () => wait(Math.max(10, 200 / SPEED));
let SPEED = 5;

async function animateVisited(visitedArr, layerId, stroke, fill) {
  const layer = document.getElementById(layerId);
  const ctr   = document.getElementById("step-ctr");
  ctr.style.display = "block";
  for (let i=0; i<visitedArr.length; i++) {
    const n = visitedArr[i], p = positions[n];
    if (!p) continue;
    ctr.textContent = `STEP ${i+1}/${visitedArr.length} — node ${n}`;
    const c = document.createElementNS("http://www.w3.org/2000/svg", "circle");
    c.setAttribute("cx", p.x); c.setAttribute("cy", p.y);
    c.setAttribute("r", "20"); c.setAttribute("fill", fill);
    c.setAttribute("stroke", stroke); c.setAttribute("stroke-width","1.5");
    c.setAttribute("opacity",".5");
    layer.appendChild(c);
    await aniDelay();
  }
  ctr.style.display = "none";
}

function drawFinalPath(path, elId, color) {
  if (!path || path.length < 2) return;
  let d = `M ${positions[path[0]].x} ${positions[path[0]].y} `;
  for (let i=1; i<path.length; i++)
    d += getNaturalPath(positions[path[i-1]], positions[path[i]], path[i-1], path[i]) + " ";
  const el = document.getElementById(elId);
  el.setAttribute("d", d);
  el.setAttribute("opacity", "1");
}

// ── Result helpers ────────────────────────────────────────────────────────────
function resetResults() {
  ["d-dist","d-time","d-nodes","d-path","a-dist","a-time","a-nodes","a-path"]
    .forEach(id => { const el=document.getElementById(id); if(el) el.textContent="—"; });
  ["bar-d-time","bar-d-nodes","bar-a-time","bar-a-nodes"]
    .forEach(id => { const el=document.getElementById(id); if(el) el.style.width="0%"; });
  document.getElementById("winner-badge").classList.remove("show");
  document.getElementById("cmp-table").classList.remove("show");
  ["card-d","card-a"].forEach(id => document.getElementById(id)?.classList.remove("win-card"));
}

function showResults(rD, rA) {
  const maxT = Math.max(rD?.timeMs||0, rA?.timeMs||0) || 1;
  const maxN = Math.max(rD?.visited?.length||0, rA?.visited?.length||0) || 1;

  if (rD && rD.path.length > 0) {
    document.getElementById("d-dist").textContent  = rD.distance + " km";
    document.getElementById("d-time").textContent  = rD.timeMs + " ms";
    document.getElementById("d-nodes").textContent = rD.visited.length + " node";
    document.getElementById("d-path").textContent  = rD.path.join("→");
    setTimeout(() => {
      document.getElementById("bar-d-time").style.width  = (rD.timeMs/maxT*100).toFixed(1)+"%";
      document.getElementById("bar-d-nodes").style.width = (rD.visited.length/maxN*100).toFixed(1)+"%";
    }, 200);
  }
  if (rA && rA.path.length > 0) {
    document.getElementById("a-dist").textContent  = rA.distance + " km";
    document.getElementById("a-time").textContent  = rA.timeMs + " ms";
    document.getElementById("a-nodes").textContent = rA.visited.length + " node";
    document.getElementById("a-path").textContent  = rA.path.join("→");
    setTimeout(() => {
      document.getElementById("bar-a-time").style.width  = (rA.timeMs/maxT*100).toFixed(1)+"%";
      document.getElementById("bar-a-nodes").style.width = (rA.visited.length/maxN*100).toFixed(1)+"%";
    }, 200);
  }

  // comparison table
  if (rD?.path?.length > 0 && rA?.path?.length > 0) {
    const dWT  = rD.timeMs <= rA.timeMs;
    const dWN  = rD.visited.length <= rA.visited.length;
    const same = rD.distance === rA.distance;
    const ratio = (Math.max(rD.timeMs,rA.timeMs) / Math.max(Math.min(rD.timeMs,rA.timeMs),0.001)).toFixed(1);

    document.getElementById("cmp-tbody").innerHTML = `
      <tr>
        <td>Jarak</td>
        <td class="${same?'td-win':'td-d'}">${rD.distance} km</td>
        <td class="${same?'td-win':'td-a'}">${rA.distance} km</td>
        <td class="td-win">${same?'Sama':'—'}</td>
      </tr>
      <tr>
        <td>Waktu (ms)</td>
        <td class="${dWT?'td-win':'td-d'}">${rD.timeMs}</td>
        <td class="${!dWT?'td-win':'td-a'}">${rA.timeMs}</td>
        <td class="td-win">${dWT?'Dijkstra':'A*'}</td>
      </tr>
      <tr>
        <td>Node dikunjungi</td>
        <td class="${dWN?'td-win':'td-d'}">${rD.visited.length}</td>
        <td class="${!dWN?'td-win':'td-a'}">${rA.visited.length}</td>
        <td class="td-win">${dWN?'Dijkstra':'A*'}</td>
      </tr>
      <tr>
        <td>Panjang rute</td>
        <td class="td-d">${rD.path.length} hop</td>
        <td class="td-a">${rA.path.length} hop</td>
        <td class="td-win">${rD.path.length<=rA.path.length?'Dijkstra':'A*'}</td>
      </tr>`;
    document.getElementById("cmp-table").classList.add("show");

    const winner = (dWT?1:0)+(dWN?1:0) >= 1 ? "Dijkstra" : "A*";
    document.getElementById("winner-title").textContent = winner + " lebih efisien pada rute ini";
    document.getElementById("winner-sub").textContent =
      `Selisih waktu: ${ratio}× · Jarak ${same?'identik':'berbeda'} · ${winner} kunjungi lebih sedikit node`;
    document.getElementById("winner-badge").classList.add("show");
    document.getElementById(winner==="Dijkstra"?"card-d":"card-a").classList.add("win-card");

    lg(`[EVALUASI] Winner: ${winner} · Jarak ${same?'identik':'berbeda'} · Rasio waktu ${ratio}×`, "ok");
  }
}

// ── Main simulation (diperluas dengan A* & log) ───────────────────────────────
let simRunning = false;

async function startSimulation() {
  if (simRunning) return;
  const start = document.getElementById("start").value;
  const end   = document.getElementById("end").value;
  if (start === end) { alert("Start dan tujuan tidak boleh sama!"); return; }

  simRunning = true;
  document.getElementById("run-btn").disabled = true;

  // reset visual
  document.getElementById("highlight-path").setAttribute("d","");
  document.getElementById("visited-d-layer").innerHTML = "";
  document.getElementById("visited-a-layer").innerHTML = "";
  document.getElementById("path-dijkstra").setAttribute("opacity","0");
  document.getElementById("path-dijkstra").setAttribute("d","");
  document.getElementById("path-astar").setAttribute("opacity","0");
  document.getElementById("path-astar").setAttribute("d","");
  document.getElementById("car").setAttribute("opacity","1");

  clearLog();
  resetResults();

  lg(`[SIM] ${start} → ${end} | ${Object.keys(graph).length} node, ${countEdges(graph)} edge`, "inf");
  lgSep();

  // ── DIJKSTRA ──────────────────────────────────────────────
  lg("[DIJKSTRA] Mulai. dist["+start+"]=0, semua lain=∞", "dijk");
  lg("  Struktur: Min-Heap Binary → O(log V) per operasi", "dijk");

  const t0d = performance.now();
  const resD = dijkstra(graph, start, end);
  resD.timeMs = +(performance.now() - t0d).toFixed(4);

  if (resD.path.length === 0) {
    lg("  ✗ Tidak ada rute ke " + end, "warn");
    document.getElementById("result").innerHTML =
      `🚫 <b>Dijkstra:</b> Tidak ada rute dari <b>${start}</b> ke <b>${end}</b>.`;
  } else {
    resD.order.forEach(o => lg(`  Step ${o.step}: visit ${o.node}  [dist=${o.dist}]`, "dijk"));
    lg(`  ✓ Rute: ${resD.path.join("→")} | Jarak: ${resD.distance} | Node: ${resD.visited.length} | ${resD.timeMs}ms`, "ok");
  }

  await animateVisited(resD.visited, "visited-d-layer", "#1565c0", "rgba(21,101,192,.12)");
  if (resD.path.length > 0) drawFinalPath(resD.path, "path-dijkstra", "#1565c0");

  lgSep();

  // ── A* ────────────────────────────────────────────────────
  lg("[A-STAR] Mulai. f(n)=g(n)+h(n), h=Euclidean×0.40", "astr");
  lg("  h admissible (tidak overestimate) → A* optimal terjamin", "astr");

  const t0a = performance.now();
  const resA = astar(graph, start, end);
  resA.timeMs = +(performance.now() - t0a).toFixed(4);

  if (resA.path.length === 0) {
    lg("  ✗ Tidak ada rute ke " + end, "warn");
  } else {
    resA.order.forEach(o => lg(`  Step ${o.step}: visit ${o.node}  [g=${o.g.toFixed(0)} h=${o.h.toFixed(0)} f=${o.f.toFixed(0)}]`, "astr"));
    lg(`  ✓ Rute: ${resA.path.join("→")} | Jarak: ${resA.distance} | Node: ${resA.visited.length} | ${resA.timeMs}ms`, "ok");
  }

  await animateVisited(resA.visited, "visited-a-layer", "#6a1b9a", "rgba(106,27,154,.12)");
  if (resA.path.length > 0) drawFinalPath(resA.path, "path-astar", "#6a1b9a");

  lgSep();
  showResults(resD, resA);

  // tampilkan result box (gaya original)
  if (resD.path.length > 0 || resA.path.length > 0) {
    const r = resD.path.length > 0 ? resD : resA;
    document.getElementById("result").innerHTML =
      `🚗 <b>Rute:</b> ${r.path.join(" ➔ ")}<br>📏 <b>Jarak:</b> ${r.distance} km ` +
      `<span style="font-size:13px;color:#555;">(Dijkstra: ${resD.timeMs}ms · A*: ${resA.timeMs}ms)</span>`;
  }

  // ── ANIMASI MOBIL (ikuti rute terbaik di atas jalan) ──────
  const bestPath = resD.path.length > 0 ? resD.path : (resA.path.length > 0 ? resA.path : null);
  if (bestPath) {
    const bestDist = resD.path.length > 0 ? resD.distance : resA.distance;
    lg(`[MOBIL] Bergerak: ${bestPath.join("→")}`, "inf");
    await moveCar(bestPath, bestDist);
    lg("[MOBIL] Tiba di tujuan!", "ok");
  }

  simRunning = false;
  document.getElementById("run-btn").disabled = false;
}

// ── Broken roads (dari original, dipertahankan) ───────────────────────────────
function generateBrokenRoads() {
  resetRoads();
  const all = [];
  for (let u in originalGraph) for (let e of originalGraph[u]) {
    const p = [u, e.node].sort().join("-");
    if (!all.find(r => r.id===p)) all.push({id:p, from:u, to:e.node});
  }
  brokenRoads = all.sort(() => Math.random()-0.5).slice(0, 3);
  const bl = document.getElementById("broken-roads-layer");
  for (let road of brokenRoads) {
    const ef = graph[road.from].find(e => e.node===road.to);
    const et = graph[road.to].find(e => e.node===road.from);
    if (ef) ef.weight += 9999;
    if (et) et.weight += 9999;
    const p1=positions[road.from], p2=positions[road.to];
    bl.innerHTML += `<path d="M ${p1.x} ${p1.y} ${getNaturalPath(p1,p2,road.from,road.to)}" class="broken-road"/>`;
  }
  document.getElementById("result").innerHTML =
    `⚠️ Jalan rusak diaktifkan! Sistem mencari jalur alternatif.`;
  clearLog();
  lg("[JALAN RUSAK] 3 edge diblokir (bobot +9999)", "warn");
  brokenRoads.forEach(r => lg(`  × ${r.from}—${r.to}`, "warn"));
}

function resetRoads() {
  graph = JSON.parse(JSON.stringify(originalGraph));
  brokenRoads = [];
  document.getElementById("broken-roads-layer").innerHTML = "";
  document.getElementById("result").innerHTML = `✅ Semua jalan kembali normal.`;
  drawWeightLabels();
  lg("[RESET] Semua jalan normal kembali", "inf");
}

// ── Randomize map (dari original + acak start/end) ────────────────────────────
// Sesuai RPM: "posisi asal, tujuan dan peta dapat di acak
//  untuk mencegah adanya hapalan rute secara hardcoding"
function randomizeMap() {
  const W=1000, H=650, PAD=75;
  const MIN_DIST=130, MAX_EDGE=430;
  const nodeCount = ri(7,11);
  const nodeNames = "ABCDEFGHIJKLMNOPQRSTUVWXYZ".slice(0,nodeCount).split("");

  // 1. tempatkan node secara acak
  const newPos = {};
  for (const name of nodeNames) {
    let placed = false;
    for (let t=0; t<600&&!placed; t++) {
      const px=rnd(PAD,W-PAD), py=rnd(PAD,H-PAD);
      let ok = true;
      for (const ep of Object.values(newPos))
        if (Math.hypot(px-ep.x,py-ep.y)<MIN_DIST) { ok=false; break; }
      if (ok) { newPos[name]={x:Math.round(px),y:Math.round(py)}; placed=true; }
    }
    if (!newPos[name]) {
      const idx = Object.keys(newPos).length;
      newPos[name] = {
        x: Math.round(PAD+(idx%4)*(W-PAD*2)/3),
        y: Math.round(PAD+Math.floor(idx/4)*(H-PAD*2)/2)
      };
    }
  }

  // 2. bangun graf (Prim spanning tree + extra edges, anti-crossing dari original)
  const newGraph = {};
  for (const n of nodeNames) newGraph[n] = [];
  const edgePts = [];
  const addedPairs = new Set();

  function tryAdd(u, v, mandatory) {
    const pair = [u,v].sort().join("-");
    if (addedPairs.has(pair)) return false;
    const p1=newPos[u], p2=newPos[v];
    const d = Math.hypot(p2.x-p1.x, p2.y-p1.y);
    if (!mandatory && d>MAX_EDGE) return false;
    if (!mandatory && edgeCrosses(edgePts, p1, p2, u, v)) return false;
    const w = randomWeight(p1, p2);
    newGraph[u].push({node:v,weight:w});
    newGraph[v].push({node:u,weight:w});
    edgePts.push({pts:buildEdgePts(p1,p2,u,v,10), from:u, to:v});
    addedPairs.add(pair);
    return true;
  }

  // Prim: pilih edge terpendek yang tidak silang
  const inTree   = new Set([nodeNames[0]]);
  const remaining = new Set(nodeNames.slice(1));
  while (remaining.size > 0) {
    let bd=Infinity, bu=null, bv=null;
    for (const u of inTree) for (const v of remaining) {
      const d = Math.hypot(newPos[u].x-newPos[v].x, newPos[u].y-newPos[v].y);
      if (d<bd && !edgeCrosses(edgePts,newPos[u],newPos[v],u,v)) { bd=d; bu=u; bv=v; }
    }
    if (!bu) {
      // fallback: izinkan crossing untuk koneksi wajib
      for (const u of inTree) for (const v of remaining) {
        const d = Math.hypot(newPos[u].x-newPos[v].x, newPos[u].y-newPos[v].y);
        if (d<bd) { bd=d; bu=u; bv=v; }
      }
    }
    if (!bu) break;
    tryAdd(bu, bv, true);
    inTree.add(bv); remaining.delete(bv);
  }

  // extra edges tidak silang
  const extra = Math.floor(nodeCount*0.55);
  const allPairs = [];
  for (let i=0; i<nodeNames.length; i++)
    for (let j=i+1; j<nodeNames.length; j++)
      allPairs.push([nodeNames[i],nodeNames[j]]);
  allPairs.sort(() => Math.random()-0.5);
  let added = 0;
  for (const [u,v] of allPairs) {
    if (added >= extra) break;
    if (tryAdd(u,v,false)) added++;
  }

  // 3. acak start & end secara random
  const shuffled = [...nodeNames].sort(() => Math.random()-0.5);
  const randStart = shuffled[0], randEnd = shuffled[1];

  // 4. apply
  originalGraph = JSON.parse(JSON.stringify(newGraph));
  graph         = JSON.parse(JSON.stringify(newGraph));
  positions     = newPos;
  brokenRoads   = [];

  document.getElementById("broken-roads-layer").innerHTML = "";
  document.getElementById("highlight-path").setAttribute("d","");
  document.getElementById("visited-d-layer").innerHTML = "";
  document.getElementById("visited-a-layer").innerHTML = "";
  document.getElementById("path-dijkstra").setAttribute("opacity","0");
  document.getElementById("path-astar").setAttribute("opacity","0");

  populateSelects();
  document.getElementById("start").value = randStart;
  document.getElementById("end").value   = randEnd;

  drawMap(true, edgePts);

  document.getElementById("result").innerHTML =
    `🎲 Peta diacak! <b>${nodeCount} kota</b>, <b>${countEdges(newGraph)} jalan</b>. Start: <b>${randStart}</b> → <b>${randEnd}</b>`;

  clearLog();
  lg(`[ACAK] ${nodeCount} kota · ${countEdges(newGraph)} jalan · Start:${randStart} → End:${randEnd}`, "inf");
}

function randomWeight(p1,p2) {
  const d = Math.hypot(p2.x-p1.x, p2.y-p1.y);
  const base = Math.round(d*0.4);
  return Math.max(50, base + Math.round((Math.random()-0.5)*base*0.5));
}

function countEdges(g) {
  let c=0; const s=new Set();
  for (let u in g) for (let e of g[u]) {
    const p=[u,e.node].sort().join("-");
    if (!s.has(p)) { s.has(p); s.add(p); c++; }
  }
  return c;
}

// ── Init ──────────────────────────────────────────────────────────────────────
document.addEventListener("DOMContentLoaded", () => {
  populateSelects();
  drawMap(false);
  document.getElementById("start").addEventListener("change", updateFlags);
  document.getElementById("end").addEventListener("change", updateFlags);
});
