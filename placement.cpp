/**
 * @name    placement.cpp
 * @group   T3 Group 07
 * @course  CSD2183 (Data Structures)
 * @brief   Computes area-preserving Steiner point placement for vertex-pair collapse operations.
 */

#include "placement.h"
#include <cmath>
#include <array>
#include <optional>
#include <utility>

static constexpr double kEps = 1e-9;

// 2D cross product of vectors (b-a) and (c-a)
static double cross2(double ax, double ay, double bx, double by, double cx, double cy)
{
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

// 2D cross product of two vectors
static double cross_vec(double ux, double uy, double vx, double vy)
{
    return ux * vy - uy * vx;
}

// Proper intersection of segments (a,b) and (c,d) — strictly interior, not at endpoints
// Returns intersection point if found
static std::optional<std::pair<double, double>>
proper_intersection(double ax, double ay, double bx, double by,
                    double cx, double cy, double dx, double dy)
{
    double ab_c = cross2(ax, ay, bx, by, cx, cy);
    double ab_d = cross2(ax, ay, bx, by, dx, dy);
    double cd_a = cross2(cx, cy, dx, dy, ax, ay);
    double cd_b = cross2(cx, cy, dx, dy, bx, by);

    auto sgn = [](double v) -> int {
        if (v > kEps) return 1;
        if (v < -kEps) return -1;
        return 0;
    };

    if (!(sgn(ab_c) * sgn(ab_d) < 0 && sgn(cd_a) * sgn(cd_b) < 0))
        return std::nullopt;

    double rx = bx - ax, ry = by - ay;
    double sx = dx - cx, sy = dy - cy;
    double denom = cross_vec(rx, ry, sx, sy);
    if (std::abs(denom) <= kEps)
        return std::nullopt;

    double t = cross_vec(cx - ax, cy - ay, sx, sy) / denom;
    return std::make_pair(ax + rx * t, ay + ry * t);
}

// Compute displacement of a 4-vertex polygon, handling self-intersection
static double quadrilateral_displacement(double p0x, double p0y,
                                         double p1x, double p1y,
                                         double p2x, double p2y,
                                         double p3x, double p3y)
{
    auto tri_area = [](double ax, double ay, double bx, double by, double cx, double cy) {
        return std::abs(cross2(ax, ay, bx, by, cx, cy)) * 0.5;
    };

    // Check edge 0→1 vs edge 2→3
    if (auto ix = proper_intersection(p0x, p0y, p1x, p1y, p2x, p2y, p3x, p3y)) {
        return tri_area(ix->first, ix->second, p1x, p1y, p2x, p2y)
             + tri_area(ix->first, ix->second, p3x, p3y, p0x, p0y);
    }
    // Check edge 1→2 vs edge 3→0
    if (auto ix = proper_intersection(p1x, p1y, p2x, p2y, p3x, p3y, p0x, p0y)) {
        return tri_area(ix->first, ix->second, p2x, p2y, p3x, p3y)
             + tri_area(ix->first, ix->second, p0x, p0y, p1x, p1y);
    }

    // Simple (non-self-intersecting) quadrilateral — use shoelace
    double area2 = (p0x * p1y - p1x * p0y)
                 + (p1x * p2y - p2x * p1y)
                 + (p2x * p3y - p3x * p2y)
                 + (p3x * p0y - p0x * p3y);
    return std::abs(area2) * 0.5;
}

/**
 * @name  compute_candidate
 * @brief Finds the optimal Steiner point E to replace vertices B and C in A->B->C->D,
 *        preserving the signed area of the quadrilateral ABCD while minimizing areal displacement.
 */
bool compute_candidate(Node* A, Node* B, Node* C, Node* D, CollapseCandidate& out)
{
    out.A = A;
    out.B = B;
    out.C = C;
    out.D = D;
    out.ring_id = A->ring_id;

    // Work in local coordinates centered at A for numerical stability
    double bx = B->x - A->x, by = B->y - A->y;
    double cx = C->x - A->x, cy = C->y - A->y;
    double dx = D->x - A->x, dy = D->y - A->y;

    // Twice the signed area of quadrilateral ABCD (in local coords, A = origin)
    // cross(A,B) + cross(B,C) + cross(C,D) + cross(D,A)
    // With A at origin: cross(0,B) + cross(B,C) + cross(C,D) + cross(D,0)
    double area2 = cross_vec(bx, by, cx, cy)
                 + cross_vec(cx, cy, dx, dy);
    // Note: cross(0,B)=0 and cross(D,0)=0 when A is origin

    bool found = false;
    double best_disp = 1e30;
    double best_ex = 0, best_ey = 0;

    // ── Try placing E on ray A→B ──
    // E = A + lambda * (B - A) = lambda * (bx, by) in local coords
    // Area preservation: lambda = area2 / cross(AB, AD)
    double denom_ab = cross_vec(bx, by, dx, dy);
    if (std::abs(denom_ab) > kEps) {
        double lambda = area2 / denom_ab;
        if (lambda > kEps) { // E must be on same side as B from A
            double ex_local = lambda * bx;
            double ey_local = lambda * by;

            // Displacement = area of quadrilateral (B, C, D, E) in local coords
            double disp = quadrilateral_displacement(
                bx, by, cx, cy, dx, dy, ex_local, ey_local);

            if (!found || disp < best_disp - kEps) {
                best_disp = disp;
                best_ex = ex_local + A->x;
                best_ey = ey_local + A->y;
                found = true;
            }
        }
    }

    // ── Try placing E on ray D→C ──
    // E = D + mu * (C - D)
    // Area preservation requires: mu = area2 / cross(C-A, D-A) = area2 / cross_vec(cx,cy,dx,dy)
    // But cross(C-A, D-A) in local coords = cross_vec(cx, cy, dx, dy)
    double denom_cd = cross_vec(cx, cy, dx, dy);
    if (std::abs(denom_cd) > kEps) {
        double mu = area2 / denom_cd;
        if (mu > kEps) { // E must be on same side as C from D
            double cdx = cx - dx, cdy = cy - dy;
            double ex_local = dx + mu * cdx;
            double ey_local = dy + mu * cdy;

            // Displacement = area of quadrilateral (A, B, C, E) in local coords
            double disp = quadrilateral_displacement(
                0, 0, bx, by, cx, cy, ex_local, ey_local);

            if (!found || disp < best_disp - kEps) {
                best_disp = disp;
                best_ex = ex_local + A->x;
                best_ey = ey_local + A->y;
                found = true;
            }
        }
    }

    if (!found) return false;

    // Reject if E coincides with A or D
    if (std::abs(best_ex - A->x) < kEps && std::abs(best_ey - A->y) < kEps) return false;
    if (std::abs(best_ex - D->x) < kEps && std::abs(best_ey - D->y) < kEps) return false;

    out.ex = best_ex;
    out.ey = best_ey;
    out.displacement = best_disp;
    return true;
}