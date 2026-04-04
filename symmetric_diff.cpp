/**
 * @name    symmetric_diff.cpp
 * @group   T3 Group 07
 * @course  CSD2183 (Data Structures)
 * @brief   Symmetric difference area computation using fan-triangulation with
 *          Sutherland-Hodgman polygon clipping for intersection area calculation.
 */

#include "symmetric_diff.h"
#include <cmath>
#include <algorithm>
#include <vector>

using Pt = std::pair<double, double>;
static constexpr double EPS = 1e-9;

static double cross2(const Pt& a, const Pt& b, const Pt& c)
{
    return (b.first - a.first) * (c.second - a.second)
         - (b.second - a.second) * (c.first - a.first);
}

static double poly_signed_area(const std::vector<Pt>& p)
{
    double a = 0;
    int n = (int)p.size();
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        a += p[i].first * p[j].second - p[j].first * p[i].second;
    }
    return a / 2.0;
}

// Is point P on the left side of (or on) directed edge A→B?
static bool is_inside(const Pt& p, const Pt& a, const Pt& b)
{
    return cross2(a, b, p) >= -EPS;
}

// Line-line intersection of segment (p1→p2) and line through (a→b)
static Pt line_intersect(const Pt& p1, const Pt& p2, const Pt& a, const Pt& b)
{
    double a1 = p2.second - p1.second, b1 = p1.first - p2.first;
    double c1 = a1 * p1.first + b1 * p1.second;
    double a2 = b.second - a.second, b2 = a.first - b.first;
    double c2 = a2 * a.first + b2 * a.second;

    double det = a1 * b2 - a2 * b1;
    if (std::abs(det) < EPS) {
        return {(p1.first + p2.first) / 2, (p1.second + p2.second) / 2};
    }
    return {(c1 * b2 - c2 * b1) / det, (a1 * c2 - a2 * c1) / det};
}

/**
 * @name  sh_clip
 * @brief Clips a subject polygon against a convex clip polygon using the Sutherland-Hodgman algorithm.
 */
static std::vector<Pt> sh_clip(const std::vector<Pt>& subject, const std::vector<Pt>& clip)
{
    if (subject.empty() || clip.empty()) return {};

    std::vector<Pt> output = subject;

    int cn = (int)clip.size();
    for (int i = 0; i < cn; i++) {
        if (output.empty()) break;

        std::vector<Pt> input = output;
        output.clear();

        int j = (i + 1) % cn;
        const Pt& edge_a = clip[i];
        const Pt& edge_b = clip[j];

        int in_n = (int)input.size();
        for (int k = 0; k < in_n; k++) {
            const Pt& curr = input[k];
            const Pt& prev = input[(k - 1 + in_n) % in_n];

            bool curr_in = is_inside(curr, edge_a, edge_b);
            bool prev_in = is_inside(prev, edge_a, edge_b);

            if (curr_in) {
                if (!prev_in) {
                    output.push_back(line_intersect(prev, curr, edge_a, edge_b));
                }
                output.push_back(curr);
            } else if (prev_in) {
                output.push_back(line_intersect(prev, curr, edge_a, edge_b));
            }
        }
    }

    return output;
}

/**
 * @name  compute_symmetric_difference
 * @brief Computes symmetric difference = area(P) + area(Q) - 2*area(P intersect Q) by
 *        fan-triangulating P and clipping Q against each triangle.
 */
double compute_symmetric_difference(
    const std::vector<Pt>& P,
    const std::vector<Pt>& Q)
{
    if (P.size() < 3 || Q.size() < 3) return 0.0;

    int np = (int)P.size();

    // Fan-triangulate P from P[0].
    // For non-convex P, some fan triangles will be CW (negative area).
    // The signed sum of (Q ∩ T_i) areas gives area(P ∩ Q) correctly.

    double intersection_area = 0.0;

    for (int i = 1; i < np - 1; i++) {
        // Triangle T_i = (P[0], P[i], P[i+1])
        Pt t0 = P[0], t1 = P[i], t2 = P[i + 1];
        double tri_area = cross2(t0, t1, t2);

        if (std::abs(tri_area) < EPS) continue; // degenerate triangle

        if (tri_area > 0) {
            // CCW triangle — clip Q against it and add area
            std::vector<Pt> clip_tri = {t0, t1, t2};
            auto clipped = sh_clip(Q, clip_tri);
            if (clipped.size() >= 3) {
                intersection_area += poly_signed_area(clipped);
            }
        } else {
            // CW triangle — reverse to CCW, clip, subtract area
            std::vector<Pt> clip_tri = {t0, t2, t1}; // reversed to CCW
            auto clipped = sh_clip(Q, clip_tri);
            if (clipped.size() >= 3) {
                intersection_area -= poly_signed_area(clipped);
            }
        }
    }

    double area_p = std::abs(poly_signed_area(P));
    intersection_area = std::abs(intersection_area);

    // symmetric_diff = area(P) + area(Q) - 2*area(P ∩ Q)
    // Since area(P) ≈ area(Q): symmetric_diff ≈ 2*(area(P) - area(P ∩ Q))
    double sym_diff = 2.0 * (area_p - intersection_area);

    return std::max(0.0, sym_diff);
}