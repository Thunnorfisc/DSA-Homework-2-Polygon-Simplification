/**
 * @name    post_process.cpp
 * @group   T3 Group 07
 * @course  CSD2183 (Data Structures)
 * @brief   Post-processing implementation: collinear vertex removal, vertex sliding optimization,
 *          inter-ring intersection prevention, and orientation verification.
 */

#include "post_process.h"
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

static double ring_signed_area_from_node(Node* head)
{
    double area2 = 0;
    Node* cur = head;
    do {
        Node* nxt = cur->next;
        area2 += cur->x * nxt->y - nxt->x * cur->y;
        cur = nxt;
    } while (cur != head);
    return area2 / 2.0;
}

/**
 * @name  causes_intersection
 * @brief Checks if relocating vertex V to (nx,ny) would cause its new edges to intersect
 *        any segment in any ring, preventing topology violations.
 */
static bool causes_intersection(Node* V, double nx, double ny,
                                const std::vector<Ring>& all_rings)
{
    Node* A = V->prev;
    Node* D = V->next;

    auto segs_cross = [](double p1x, double p1y, double p2x, double p2y,
                         double p3x, double p3y, double p4x, double p4y) -> bool {
        auto cross_val = [](double ax, double ay, double bx, double by, double cx, double cy) {
            return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
        };
        double d1 = cross_val(p3x, p3y, p4x, p4y, p1x, p1y);
        double d2 = cross_val(p3x, p3y, p4x, p4y, p2x, p2y);
        double d3 = cross_val(p1x, p1y, p2x, p2y, p3x, p3y);
        double d4 = cross_val(p1x, p1y, p2x, p2y, p4x, p4y);
        return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
               ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
    };

    auto same_pt = [](double ax, double ay, double bx, double by) -> bool {
        return std::abs(ax - bx) < 1e-12 && std::abs(ay - by) < 1e-12;
    };

    for (const auto& ring : all_rings) {
        if (ring.vertices.size < 3) continue;

        Node* cur = ring.vertices.head;
        do {
            Node* nxt = cur->next;

            // Skip edges adjacent to V (A→V, V→D)
            if (cur == A && nxt == V) { cur = nxt; continue; }
            if (cur == V && nxt == D) { cur = nxt; continue; }

            // Skip edges that share endpoints with new edges
            bool share_ae = same_pt(cur->x, cur->y, A->x, A->y) ||
                            same_pt(nxt->x, nxt->y, A->x, A->y) ||
                            same_pt(cur->x, cur->y, nx, ny) ||
                            same_pt(nxt->x, nxt->y, nx, ny);
            bool share_ed = same_pt(cur->x, cur->y, nx, ny) ||
                            same_pt(nxt->x, nxt->y, nx, ny) ||
                            same_pt(cur->x, cur->y, D->x, D->y) ||
                            same_pt(nxt->x, nxt->y, D->x, D->y);

            if (!share_ae) {
                if (segs_cross(A->x, A->y, nx, ny, cur->x, cur->y, nxt->x, nxt->y))
                    return true;
            }
            if (!share_ed) {
                if (segs_cross(nx, ny, D->x, D->y, cur->x, cur->y, nxt->x, nxt->y))
                    return true;
            }

            cur = nxt;
        } while (cur != ring.vertices.head);
    }

    return false;
}

/**
 * @name  remove_collinear
 * @brief Removes vertices that are collinear with their neighbors (zero area contribution).
 */
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
        double cv = (B->x - A->x) * (C->y - A->y) - (B->y - A->y) * (C->x - A->x);
        if (std::abs(cv) < 1e-9) {
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

/**
 * @name  post_process
 * @brief Refines simplified polygons over multiple iterations by sliding each vertex along
 *        the line connecting its neighbors to minimize symmetric difference with the original.
 *        Validates orientation preservation and inter-ring topology at each step.
 */
int post_process(std::vector<Ring>& rings,
                 const std::vector<std::vector<Pt>>& original_vertices,
                 int max_iterations)
{
    int total_relocated = 0;

    for (size_t ri = 0; ri < rings.size(); ri++) {
        Ring& ring = rings[ri];
        const std::vector<Pt>& orig = original_vertices[ri];

        if (ring.vertices.size <= 3 || orig.size() < 3) continue;

        int col_removed = remove_collinear(ring);
        if (col_removed > 0) {
            std::cerr << "  Ring " << ring.ring_id << ": removed "
                      << col_removed << " collinear vertices" << std::endl;
        }

        // Remember original orientation sign
        double orig_sign = ring_signed_area_from_node(ring.vertices.head);

        double best_sym_diff = compute_symmetric_difference(orig, ring_to_vec(ring));
        std::cerr << "  Ring " << ring.ring_id << ": initial sym diff = " << best_sym_diff << std::endl;

        for (int iter = 0; iter < max_iterations; iter++) {
            int relocated = 0;

            Node* cur = ring.vertices.head;
            do {
                Node* A = cur->prev;
                Node* V = cur;
                Node* D = cur->next;

                double dx = D->x - A->x, dy = D->y - A->y;
                double len = std::sqrt(dx * dx + dy * dy);
                if (len < 1e-12) { cur = cur->next; continue; }
                dx /= len;
                dy /= len;

                double step = len * 0.25;
                double orig_x = V->x, orig_y = V->y;
                double best_x = orig_x, best_y = orig_y;
                double best_local = best_sym_diff;

                double trials[] = {-1.0, -0.3, 0.3, 1.0, -0.1, 0.1};

                for (double t : trials) {
                    double nx = orig_x + t * step * dx;
                    double ny = orig_y + t * step * dy;

                    // Check against ALL rings for intersection
                    if (causes_intersection(V, nx, ny, rings)) continue;

                    // Temporarily move V and check orientation is preserved
                    V->x = nx;
                    V->y = ny;

                    double new_sign = ring_signed_area_from_node(ring.vertices.head);
                    if ((orig_sign > 0 && new_sign <= 0) || (orig_sign < 0 && new_sign >= 0)) {
                        // Orientation flipped — reject
                        V->x = orig_x;
                        V->y = orig_y;
                        continue;
                    }

                    double trial_diff = compute_symmetric_difference(orig, ring_to_vec(ring));
                    if (trial_diff < best_local - 1e-6) {
                        best_local = trial_diff;
                        best_x = nx;
                        best_y = ny;
                    }

                    V->x = orig_x;
                    V->y = orig_y;
                }

                if (best_x != orig_x || best_y != orig_y) {
                    V->x = best_x;
                    V->y = best_y;
                    best_sym_diff = best_local;
                    relocated++;
                }

                cur = cur->next;
            } while (cur != ring.vertices.head);

            total_relocated += relocated;
            std::cerr << "  Ring " << ring.ring_id << " iter " << iter
                      << ": relocated " << relocated
                      << " vertices, sym diff = " << best_sym_diff << std::endl;

            if (relocated == 0) break;
        }
    }

    // Final orientation verification
    for (size_t ri = 0; ri < rings.size(); ri++) {
        Ring& ring = rings[ri];
        double area = ring_signed_area_from_node(ring.vertices.head);
        if (ring.ring_id == 0 && area < 0) {
            std::cerr << "WARNING: Exterior ring has negative area after post-processing!" << std::endl;
        } else if (ring.ring_id > 0 && area > 0) {
            std::cerr << "WARNING: Interior ring " << ring.ring_id
                      << " has positive area after post-processing!" << std::endl;
        }
    }

    return total_relocated;
}