# 🚚 SmartRoute — Simulasi Kurir Cerdas

> **Final Project** · Perancangan dan Analisis Algoritma (INF11111)  
> Teknik Informatika · Universitas Maritim Raja Ali Haji · Semester Genap 2025/2026  
> Dosen Pengampu: Tekad Matulatan & Muhammad Fadli

---

## 📋 Deskripsi Project

SmartRoute adalah simulasi **Smart Courier** berbasis web yang mendemonstrasikan bagaimana tiga algoritma pencarian rute bekerja secara visual dan terukur. Kurir digital mengantarkan paket dari titik asal ke titik tujuan melalui jalur terpendek yang ditemukan oleh algoritma, lengkap dengan animasi step-by-step, perbandingan performa, dan analisis kompleksitas secara real-time.

Peta, titik asal, dan titik tujuan dapat diacak untuk membuktikan bahwa tidak ada rute yang di-*hardcode* — algoritma benar-benar berjalan dan mencari jalur secara dinamis.

---

## 🎬 Demo

Jalankan langsung dengan membuka `index.html` di browser — tidak ada instalasi, tidak ada dependensi.

```
SmartRoute/
├── index.html   ← buka ini di browser
├── style.css
├── main.js
└── README.md
```

---

## ⚙️ Fitur Utama

| Fitur | Keterangan |
|---|---|
| **3 Algoritma** | BFS (baseline), Dijkstra, A* — semuanya from scratch tanpa library routing |
| **Visualisasi Step-by-Step** | Label nilai dist=/f=/hop= muncul di atas setiap node saat dikunjungi |
| **Animasi Kurir** | Mobil bergerak mengikuti jalur jalan, bodi menghadap arah, roda berputar |
| **Tabel Perbandingan** | Jarak, waktu eksekusi (ms), node dikunjungi — ketiga algoritma berdampingan |
| **Jalan Rusak** | Blokir 3 edge acak, algoritma otomatis cari jalur alternatif |
| **Acak Peta** | Graf 7–11 node digenerate acak beserta asal & tujuan — anti-hardcode |
| **Slider Kecepatan** | Atur kecepatan animasi 1×–10× |
| **Execution Log** | Log langkah tiap algoritma berwarna, dapat di-clear |

---

## 🧠 Algoritma yang Diimplementasikan

### BFS — Breadth-First Search `O(V+E)`
- Struktur data: **Queue FIFO** + Set untuk duplikat O(1)
- Eksplorasi level per level, tidak mempertimbangkan bobot edge
- Menemukan rute dengan **hop terkecil**, bukan jarak terpendek berbobot
- Digunakan sebagai **BASELINE** perbandingan
- Warna visualisasi: 🟠 Oranye

### Dijkstra `O((V+E) log V)`
- Strategi: **Greedy** — selalu pilih node dengan jarak terkecil
- Struktur data: **Min-Heap Binary** + visitedSet O(1)
- Optimal untuk semua bobot ≥ 0
- Garansi rute terpendek secara mutlak
- Warna visualisasi: 🔵 Biru

### A\* (A-Star) `O(E log V)` rata-rata
- Fungsi evaluasi: **f(n) = g(n) + h(n)**
  - `g(n)` = biaya aktual dari start ke n
  - `h(n)` = Euclidean × 0.40 *(admissible — tidak pernah overestimate)*
- Lebih efisien dari Dijkstra karena heuristik mengarahkan ke tujuan
- Optimal karena h admissible
- Warna visualisasi: 🟣 Ungu

---

## 📊 Analisis Kompleksitas

```
Algoritma   Waktu             Ruang   Struktur Data       Optimal
─────────────────────────────────────────────────────────────────
BFS         O(V + E)          O(V)    Queue FIFO + Set    ✗ (abaikan bobot)
Dijkstra    O((V+E) log V)    O(V)    Min-Heap + Set      ✓ (bobot ≥ 0)
A*          O(E log V) avg    O(V)    Min-Heap + h(n)     ✓ (h admissible)
```

### Recurrence Equation Dijkstra

```
T(V, E)  = T_extractMin + T_relaksasi
         = V × O(log V)  +  E × O(log V)
         = (V + E) × O(log V)
         = O((V+E) log V)  ✓
```

### Analisis Amortisasi Min-Heap

| Operasi | Worst Case | Amortized |
|---|---|---|
| push() — bubble-up | O(log n) | O(log n) |
| pop() — bubble-down | O(log n) | O(log n) |
| n operasi total | O(n log n) | O(n log n) |

---

## 🗂️ Struktur Kode

```
main.js
├── Default Data & Graph Setup
│   ├── defaultGraph          — graf default 11 node (A–K)
│   ├── defaultNodes          — posisi koordinat setiap node
│   └── populateSelects()     — mengisi dropdown asal & tujuan
│
├── Rendering & Visualisasi
│   ├── drawMap()             — render graf ke SVG
│   ├── animateVisited()      — animasi step-by-step kunjungan node
│   ├── samplePathPos()       — interpolasi posisi kurir di jalur bezier
│   ├── samplePathAngle()     — hitung sudut arah gerak kurir
│   └── buildPathD()          — bangun SVG path string dari array node
│
├── Algoritma
│   ├── bfs()                 — BFS O(V+E)
│   ├── dijkstra()            — Dijkstra O((V+E) log V)
│   ├── heuristic()           — Euclidean × 0.40 (admissible)
│   └── astar()               — A* O(E log V) avg
│
├── Struktur Data
│   └── MinHeap               — Binary Min-Heap (push/pop O(log n))
│
├── Simulasi & Integrasi
│   ├── startSimulation()     — orkestrator utama: BFS → Dijkstra → A* → moveCar
│   ├── moveCar()             — animasi kurir dengan requestAnimationFrame
│   ├── countEdges()          — hitung sisi unik graf O(E)
│   ├── lg() / lgSep()        — sistem execution log berwarna O(1)
│   └── resetResults()        — reset state UI sebelum simulasi baru
│
├── Evaluasi & Perbandingan
│   └── showResults()         — tabel perbandingan, evaluasi, winner badge
│
└── Fitur Tambahan
    ├── generateBrokenRoads() — blokir 3 edge dengan bobot +9999
    ├── resetRoads()          — pulihkan semua edge ke bobot normal
    └── randomizeMap()        — acak graf 7–11 node beserta asal & tujuan
```

---

## 👥 Tim

| No | Nama | NIM |
|---|---|---|
| 1 | **Raihan Darma Putra** | 2401020138 |
| 2 | **M. Kika Haekal** | 2401020140 |
| 3 | **Yehezkiel Alman** | 2401020110 |
| 4 | **Neza Khairunnisa Rahmah** | 2401020097 |
| 5 | **Elsha Natalia Panjaitan** | 2401020096 |

---

## 🚀 Cara Menjalankan

1. Clone repository ini
   ```bash
   git clone https://github.com/[username]/smartroute-paa2026.git
   cd smartroute-paa2026
   ```

2. Buka `index.html` di browser (Chrome/Firefox/Edge)
   ```bash
   # Tidak perlu server — buka langsung sebagai file
   open index.html        # macOS
   start index.html       # Windows
   xdg-open index.html    # Linux
   ```

3. Pilih titik **Asal** dan **Tujuan** dari dropdown, lalu klik **▶ Antar Paket**

---

## 🎮 Panduan Penggunaan

| Tombol | Fungsi |
|---|---|
| **▶ Antar Paket** | Jalankan simulasi — ketiga algoritma berjalan berurutan |
| **⚡ Jalan Rusak** | Blokir 3 edge acak, uji algoritma cari jalur alternatif |
| **↺ Reset Jalan** | Pulihkan semua edge ke bobot normal |
| **🎲 Acak Peta** | Generate graf baru 7–11 node secara acak |
| **Slider Kecepatan** | Atur kecepatan animasi 1× (lambat) hingga 10× (cepat) |

---

## 🔬 Detail Teknis

- **Bahasa**: JavaScript (ES2020+), HTML5, CSS3
- **Rendering**: SVG inline — tidak ada canvas, tidak ada WebGL
- **Animasi Kurir**: `requestAnimationFrame` + interpolasi bezier kubik — satu sistem timing untuk body & roda
- **Graf Default**: 11 node (A–K), ~15 sisi, bobot 100–600
- **Graf Acak**: 7–11 node, konektivitas dijamin dengan Prim spanning tree, bobot `max(50, round(Euclidean × 0.4))`
- **Priority Queue**: Binary Min-Heap custom — push/pop O(log n)
- **Visited Check**: `Set.has()` O(1) — bukan `Array.includes()` O(n)

---

## 📚 Referensi

1. Cormen et al. (2022). *Introduction to Algorithms, 4th Ed.* MIT Press.
2. Grujic & Grujic (2025). Optimal Routing Using Dijkstra. *Applied Sciences, 15*(8). https://doi.org/10.3390/app15084162
3. Alameri et al. (2024). Shortest Path Algorithms for MANETs. *CMES, 141*(1). https://doi.org/10.32604/cmes.2024.052107
4. Kumar, S. (2024). Optimized Dijkstra Implementation. *IJRASET*. https://doi.org/10.22214/ijraset.2024.65444
5. Agostinelli et al. (2024). Learning Admissible Heuristics for A\*. *arXiv*. https://arxiv.org/pdf/2509.22626
6. IJRSI (2025). Priority Queue Variants in Dijkstra. https://rsisinternational.org/journals/ijrsi/article.php?id=85

---

## 📄 Lisensi

```
© Project Design and Analysis of Algorithm Course 2026
Universitas Maritim Raja Ali Haji
INF11111 — Perancangan dan Analisis Algoritma
```

Project ini dibuat untuk keperluan akademis. Diperbolehkan untuk dipelajari dan dijadikan referensi dengan menyertakan atribusi yang sesuai.

---

*SmartRoute · PAA 2026 · UMRAH · Teknik Informatika*
