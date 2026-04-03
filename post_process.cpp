#include "post_process.h"
#include "topology.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>

using Pt = std::pair<double, double>;

static std::vector<Pt> ring_to_vec(const Ring& ring)
{
    std::vector<Pt> v;
    Node* cur = ring.vertices.head;
    do {
        v.push_back({cur->x, cur->y});
        cur = cur->next;
    } while (cur != ring.vertices.head);
    return v;
}

// Check if moving vertex V to (nx, ny) creates any self-intersection in its ring
static bool causes_self_intersection(Node* V, double nx, double ny)
{
    Node* A = V->prev;
    Node* D = V->next;

    // Check new edges A→V' and V'→D against all other edges in this ring
    Node* cur = D->next; // start after D since D→D->next shares endpoint with V'→D
    if (cur == A) return false; // ring too small

    Node* stop = A; // stop before A since A->prev→A shares endpoint with A→V'

    auto cross2 = [](double ax, double ay, double bx, double by, double cx, double cy) {
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    };

    auto segs_cross = [&](double p1x, double p1y, double p2x, double p2y,
                          double p3x, double p3y, double p4x, double p4y) -> bool {
        double d1 = cross2(p3x, p3y, p4x, p4y, p1x, p1y);
        double d2 = cross2(p3x, p3y, p4x, p4y, p2x, p2y);
        double d3 = cross2(p1x, p1y, p2x, p2y, p3x, p3y);
        double d4 = cross2(p1x, p1y, p2x, p2y, p4x, p4y);
        if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
            ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
            return true;
        return false;
    };

    do {
        Node* nxt = cur->next;
        // Skip edges adjacent to V
        if (cur == A || nxt == A || cur == D || nxt == D) {
            cur = nxt;
            continue;
        }

        // Check A→V' against cur→nxt
        if (segs_cross(A->x, A->y, nx, ny, cur->x, cur->y, nxt->x, nxt->y))
            return true;
        // Check V'→D against cur→nxt
        if (segs_cross(nx, ny, D->x, D->y, cur->x, cur->y, nxt->x, nxt->y))
            return true;

        cur = nxt;
    } while (cur != stop);

    return false;
}

// Remove collinear vertices (zero-cost improvement)
static int remove_collinear(Ring& ring)
{
    if (ring.vertices.size <= 3) return 0;

    int removed = 0;
    Node* cur = ring.vertices.head;
    Node* start = cur;
    bool first = true;

    while (first || cur != start) {
        first = false;
        if (ring.vertices.size <= 3) break;

        Node* A = cur->prev;
        Node* B = cur;
        Node* C = cur->next;

        // Check if B is collinear with A and C
        double cross_val = (B->x - A->x) * (C->y - A->y) - (B->y - A->y) * (C->x - A->x);
        if (std::abs(cross_val) < 1e-9) {
            // B is collinear — remove it
            Node* next = C;
            ring.vertices.remove(B);
            removed++;
            if (start == B) start = next;
            cur = next;
        } else {
            cur = cur->next;
        }
    }

    return removed;
}

int post_process(std::vector<Ring>& rings,
                 const std::vector<std::vector<Pt>>& original_vertices,
                 int max_iterations)
{
    int total_relocated = 0;

    for (size_t ri = 0; ri < rings.size(); ri++) {
        Ring& ring = rings[ri];
        const std::vector<Pt>& orig = original_vertices[ri];

        if (ring.vertices.size <= 3) continue;
        if (orig.size() < 3) continue;

        // Remove collinear vertices first (free improvement)
        int col_removed = remove_collinear(ring);
        if (col_removed > 0) {
            std::cerr << "  Ring " << ring.ring_id << ": removed "
                      << col_removed << " collinear vertices" << std::endl;
        }

        // Vertex sliding optimization
        double best_sym_diff = compute_symmetric_difference(orig, ring_to_vec(ring));
        std::cerr << "  Ring " << ring.ring_id << ": initial sym diff = " << best_sym_diff << std::endl;

        for (int iter = 0; iter < max_iterations; iter++) {
            int relocated_this_iter = 0;

            Node* cur = ring.vertices.head;
            do {
                Node* A = cur->prev;
                Node* V = cur;
                Node* D = cur->next;

                // Direction along area-preserving line (parallel to AD)
                double dx = D->x - A->x;
                double dy = D->y - A->y;
                double len = std::sqrt(dx * dx + dy * dy);
                if (len < 1e-12) {
                    cur = cur->next;
                    continue;
                }
                dx /= len;
                dy /= len;

                // Try sliding V along the area-preserving line
                // Step size relative to AD length
                double step = len * 0.25;
                double orig_x = V->x, orig_y = V->y;
                double best_x = orig_x, best_y = orig_y;
                double best_local = best_sym_diff;

                // Trial offsets (in units of 'step')
                double trials[] = {-2.0, -1.0, -0.5, -0.2, -0.1, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0};

                for (double t : trials) {
                    double nx = orig_x + t * step * dx;
                    double ny = orig_y + t * step * dy;

                    // Quick topology check (self-intersection)
                    if (causes_self_intersection(V, nx, ny)) continue;

                    // Temporarily move V
                    V->x = nx;
                    V->y = ny;

                    double trial_diff = compute_symmetric_difference(orig, ring_to_vec(ring));
                    if (trial_diff < best_local - 1e-6) {
                        best_local = trial_diff;
                        best_x = nx;
                        best_y = ny;
                    }

                    // Restore
                    V->x = orig_x;
                    V->y = orig_y;
                }

                // Apply best position
                if (best_x != orig_x || best_y != orig_y) {
                    V->x = best_x;
                    V->y = best_y;
                    best_sym_diff = best_local;
                    relocated_this_iter++;
                }

                cur = cur->next;
            } while (cur != ring.vertices.head);

            total_relocated += relocated_this_iter;

            std::cerr << "  Ring " << ring.ring_id << " iter " << iter
                      << ": relocated " << relocated_this_iter
                      << " vertices, sym diff = " << best_sym_diff << std::endl;

            if (relocated_this_iter == 0) break; // converged
        }
    }

    return total_relocated;
}