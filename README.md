# Area- and Topology-Preserving Polygon Simplification

**Course:** CSD2183 Data Structures | **Group:** T3 Group 07 | **AY2025/26 Trimester 2**

## About This Project

This project implements an algorithm to simplify polygons with holes while **exactly preserving the signed area** of every ring and **maintaining topological validity** (no self-intersections, no ring crossings, ring count unchanged). The goal is to reduce the number of vertices to a user-specified target while minimizing **areal displacement** — the area of the symmetric difference between the original and simplified polygon.

The core algorithm is **Area-Preserving Segment Collapse (APSC)** based on Kronenfeld et al. (2020). For every sequence of four consecutive vertices A -> B -> C -> D in a ring, the algorithm computes an area-preserving Steiner point E such that replacing A -> B -> C -> D with A -> E -> D preserves the ring's signed area exactly. The collapse with the smallest areal displacement is greedily selected, topology is verified, and the process repeats until the target vertex count is reached.

### Constraints Enforced

- **Area preservation:** Each ring's signed area is preserved to within floating-point tolerance.
- **Topology preservation:** No ring may intersect itself or any other ring after simplification. The number of rings remains unchanged.
- **Minimum ring size:** Every ring retains at least 3 vertices (a triangle is the minimum valid polygon).

### Input/Output Format

**Input:** CSV file with columns `ring_id,vertex_id,x,y` where ring 0 is the exterior ring (CCW) and rings 1+ are interior holes (CW).

**Output (stdout):**
```
ring_id,vertex_id,x,y
0,0,<x>,<y>
...
Total signed area in input: <value>
Total signed area in output: <value>
Total areal displacement: <value>
```

Diagnostic/status messages are printed to stderr only.

---

## Building and Running

### Prerequisites

- **g++** with C++17 support
- **Linux or macOS** (Windows users: use WSL)
- No third-party libraries required — the project uses only the C++ standard library.

### Option 1: Build Manually (Terminal)

```bash
# Build
make

# Run
./simplify <input_file.csv> <target_vertices>

# Example
./simplify sample_test_cases/input_cushion_with_hexagonal_hole.csv 13

# Clean build artifacts
make clean
```

The `Makefile` compiles all source files with `-std=c++17 -Wall -O2` and produces the `simplify` executable in the project root.

### Option 2: Use `run.bat` (Windows with WSL)

The included `run.bat` script automates building, test selection, and output collection:

```
run.bat
```

**What it does:**

1. **Builds** the project via WSL (`make clean && make`). If the build fails, it offers to retry or continue.
2. **Displays a numbered menu** of all available test cases from both `sample_test_cases/` and `my_test_cases/`, along with the **recommended target vertex count** from each folder's `README.md`.
3. **User selects by number:**
   - An individual test case — prompts for target vertex count (press Enter to use the recommended default).
   - **Run ALL sample test cases** — automatically uses each file's recommended target from the README.
   - **Run ALL my test cases** — same as above for custom tests.
   - **Run ALL test cases** — runs both sample and custom test cases with recommended targets.
4. **Saves results** as `.txt` files in the `output/` folder, named `<input_name>_t<target>.txt`.
5. After completion, returns to the menu for additional runs.

---

## Implementation Details

### Project Structure

| File | Description |
|---|---|
| `main.cpp` | Entry point: parses CLI args, coordinates normalization, simplification, post-processing, and output. |
| `polygon.h/cpp` | Core data structures (`Node`, `CircularDoublyLinkedList`, `Ring`), CSV parsing, signed area computation (shoelace formula), and output formatting. |
| `placement.h/cpp` | Steiner point computation for area-preserving two-vertex collapse. Tries placement on both ray A->B and ray D->C, picks the option with lower displacement. |
| `simplify.h/cpp` | Main simplification loop: greedy collapse of lowest-cost vertex pairs with lazy deletion via validity checks. |
| `heap.h/cpp` | Array-backed binary min-heap for prioritizing collapse candidates by displacement cost, with serial number tie-breaking. |
| `topology.h/cpp` | Segment intersection tests (cross-product orientation with collinear overlap detection) and collapse validation against the spatial index. |
| `spatial_index.h/cpp` | R-tree spatial index for efficient segment intersection queries during simplification. Uses quadratic split for node overflow. |
| `symmetric_diff.h/cpp` | Symmetric difference area computation using fan-triangulation of the original polygon and Sutherland-Hodgman clipping against the simplified polygon. |
| `post_process.h/cpp` | Post-processing refinement: slides vertices along area-preserving lines to minimize symmetric difference, removes collinear vertices, and verifies orientation. |
| `Makefile` | Build configuration: `make` builds `simplify`, `make clean` removes it. |
| `run.bat` | Windows batch script for automated build + test execution via WSL. |

### Data Structures

#### Circular Doubly Linked List — Ring Representation

Each polygon ring is stored as a `CircularDoublyLinkedList` of `Node` structs. This allows:

- **O(1) vertex removal** during collapse — just relink `prev`/`next` pointers.
- **O(1) insertion** of the Steiner point E between vertices A and D.
- **Natural circular traversal** — no special-casing for wrap-around since the list is circular.

Each `Node` stores coordinates (`x`, `y`), a `ring_id`, a `deleted` flag for lazy validity checks, and `prev`/`next` pointers.

#### Min-Heap — Priority Queue for Collapse Candidates

A `MinHeap` (array-backed binary heap) orders `CollapseCandidate` entries by displacement cost. Ties are broken by a monotonically increasing serial number to ensure deterministic ordering.

- **Push:** O(log n) — append and sift up.
- **Pop:** O(log n) — swap root with last, sift down.
- **Lazy deletion:** Stale candidates (where vertices have been deleted or relinked since the candidate was created) are detected via the `is_valid()` check and discarded at pop time, avoiding the need for a decrease-key operation.

#### R-Tree — Spatial Index for Intersection Checks

A dynamic R-tree indexes all polygon segments (edges) for efficient spatial queries. This is critical for the topology check — before each collapse, the new edges A->E and E->D must be tested against all nearby segments for intersection.

- **Build:** Inserts all segments from all rings — O(V log V).
- **Insert/Remove:** O(log V) amortized per segment — the tree is updated incrementally after each collapse (3 segments removed, 2 inserted).
- **Query:** Given a bounding box, returns all segments whose AABB overlaps it — O(sqrt(V) + k) typical, where k is the number of results.
- **Node splitting:** Uses the **quadratic split** algorithm — picks two seed entries that waste the most area, then assigns remaining entries to the group whose bounding box grows least.
- **Underflow handling:** After removal, underflowing leaf nodes have their entries collected and reinserted. Single-child roots are collapsed.

### Algorithm Pipeline

```
1. Parse CSV → vector<Ring>  (each ring is a circular doubly linked list)
2. Normalize coordinates      (center bounding box at origin for numerical stability)
3. Build R-tree spatial index  (index all polygon segments)
4. Build min-heap              (compute initial collapse candidates for all 4-vertex sequences)
5. Greedy collapse loop:
   a. Pop lowest-cost candidate from heap
   b. Validate (lazy deletion check — are all 4 vertices still linked?)
   c. Topology check via R-tree (would new edges A→E, E→D intersect anything?)
   d. If valid: remove old segments from R-tree, insert E, remove B and C,
      insert new segments, regenerate affected candidates
   e. Repeat until target vertex count reached or heap empty
6. Post-processing (adaptive based on input size):
   - Remove collinear vertices
   - Slide each vertex along its area-preserving line to minimize symmetric difference
   - Verify orientation is preserved and no inter-ring intersections are introduced
7. Output simplified polygon + area/displacement statistics
```

### Steiner Point Placement

For a sequence A -> B -> C -> D, the algorithm computes a replacement point E such that the signed area of triangle A-E-D equals the signed area of quadrilateral A-B-C-D. Two candidate placements are tried:

1. **E on ray A->B:** `E = A + lambda * (B - A)` where `lambda = area(ABCD) / cross(AB, AD)`.
2. **E on ray D->C:** `E = D + mu * (C - D)` where `mu = area(ABCD) / cross(CA, DA)`.

The placement with the smaller **areal displacement** (area enclosed between the old and new paths) is selected. Computations use local coordinates centered at A for numerical stability.

### Post-Processing Enhancement

After the greedy simplification, an optional post-processing pass further reduces symmetric difference:

1. **Collinear vertex removal:** Vertices that are collinear with their neighbors (zero cross-product) are removed for free.
2. **Vertex sliding:** Each vertex V is trialed at several positions along the line connecting its neighbors (prev, next). The position that minimizes the symmetric difference with the original polygon is kept, subject to:
   - No intersection with any segment in any ring (inter-ring topology preservation).
   - No orientation flip (signed area must keep its sign).
3. **Adaptive intensity:** Full post-processing (3 iterations) for inputs under 20K vertices, light (1 iteration) for 20K-100K, skipped for inputs over 100K vertices.

### Coordinate Normalization

Before simplification, all vertex coordinates are translated so the bounding box centroid is at the origin. This keeps cross-product magnitudes manageable (avoiding overflow/precision loss for coordinates in the range of 1e6). The offset is added back when producing output.

---

## Test Cases

### Sample Test Cases (`sample_test_cases/`)

Reference test cases provided by the instructor. See `sample_test_cases/README.md` for recommended target vertex counts.

| Input file | Target | Rings | Total Vertices |
|---|---|---|---|
| `input_cushion_with_hexagonal_hole.csv` | 13 | 2 | 22 |
| `input_blob_with_two_holes.csv` | 17 | 3 | 36 |
| `input_wavy_with_three_holes.csv` | 21 | 4 | 43 |
| `input_lake_with_two_islands.csv` | 17 | 3 | 81 |
| `input_original_01.csv` | 99 | 1 | ~700 |
| `input_original_02.csv` | 99 | 1 | ~3,200 |
| `input_original_03.csv` | 99 | 1 | ~30,000 |
| `input_original_08.csv` | 99 | 1 | ~2,400 |
| `input_original_09.csv` | 99 | 1 | ~150,000 |

### Custom Test Cases (`my_test_cases/`)

Test cases designed to stress specific robustness aspects. See `my_test_cases/README.md` for details.

| Input file | Target | What It Tests |
|---|---|---|
| `input_triangle_minimal.csv` | 3 | Edge case: already at minimum vertices, nothing to simplify |
| `input_star_with_center_hole.csv` | 8 | Sharp concave spikes with a hole near the center |
| `input_thin_zigzag_strip.csv` | 10 | Near-degenerate thin geometry with zigzag edges |
| `input_arrow_shape.csv` | 7 | Non-convex shape with a notch — concavity preservation |
| `input_nested_squares.csv` | 12 | 3 concentric square holes — multi-ring topology with minimal vertices per ring |
| `input_donut_ring.csv` | 12 | Circular donut with tight inner/outer gap — hole proximity |
| `input_circle_approx_64.csv` | 16 | 64-vertex circle reduced heavily — uniform curvature simplification |
| `input_spiral_polygon.csv` | 15 | Self-approaching spiral corridor — narrow passages and near self-intersection |
| `input_many_small_holes.csv` | 50 | Grid of 25 square holes — stress test for many rings |
| `input_large_jagged_coastline.csv` | 20 | 96-vertex jagged coastline — noisy high-frequency detail removal |

---

## References

Kronenfeld, B. J., L. V. Stanislawski, B. P. Buttenfield, and T. Brockmeyer (2020). "Simplification of polylines by segment collapse: minimizing areal displacement while preserving area". *International Journal of Cartography* 6.1, pp. 22-46.
