/**
 * @name    topology.cpp
 * @group   T3 Group 07
 * @course  CSD2183 (Data Structures)
 * @brief   Segment intersection tests and collapse validation against the spatial index.
 */

#include "topology.h"
#include <cmath>
#include <algorithm>

/**
 * @name  segments_intersect
 * @brief Tests if segments (p1,p2) and (p3,p4) intersect using cross-product orientation tests
 *        with collinear endpoint overlap detection.
 */
bool segments_intersect(double p1x, double p1y, double p2x, double p2y,
                        double p3x, double p3y, double p4x, double p4y)
{
    auto cross = [](double ax, double ay, double bx, double by,
                    double cx, double cy) -> double {
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    };

    double d1 = cross(p3x, p3y, p4x, p4y, p1x, p1y);
    double d2 = cross(p3x, p3y, p4x, p4y, p2x, p2y);
    double d3 = cross(p1x, p1y, p2x, p2y, p3x, p3y);
    double d4 = cross(p1x, p1y, p2x, p2y, p4x, p4y);

    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
        ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
        return true;
    }

    auto on_segment = [](double px, double py, double qx, double qy,
                         double rx, double ry) -> bool {
        return std::min(px, qx) <= rx + 1e-12 && rx <= std::max(px, qx) + 1e-12 &&
               std::min(py, qy) <= ry + 1e-12 && ry <= std::max(py, qy) + 1e-12;
    };

    if (std::abs(d1) < 1e-12 && on_segment(p3x, p3y, p4x, p4y, p1x, p1y)) return true;
    if (std::abs(d2) < 1e-12 && on_segment(p3x, p3y, p4x, p4y, p2x, p2y)) return true;
    if (std::abs(d3) < 1e-12 && on_segment(p1x, p1y, p2x, p2y, p3x, p3y)) return true;
    if (std::abs(d4) < 1e-12 && on_segment(p1x, p1y, p2x, p2y, p4x, p4y)) return true;

    return false;
}

static bool same_point(double px, double py, double qx, double qy)
{
    return std::abs(px - qx) < 1e-12 && std::abs(py - qy) < 1e-12;
}

// Check if a new segment (sx, sy)→(tx, ty) intersects any segment from the grid,
// excluding segments in the exclude set and segments sharing endpoints
static bool check_segment_against_grid(double sx, double sy, double tx, double ty,
                                       const SpatialGrid& grid,
                                       const std::unordered_set<Segment, SegmentHash>& exclude)
{
    // Query bounding box of the new segment
    double qx1 = std::min(sx, tx);
    double qy1 = std::min(sy, ty);
    double qx2 = std::max(sx, tx);
    double qy2 = std::max(sy, ty);

    std::vector<Segment> nearby;
    grid.query(qx1, qy1, qx2, qy2, nearby, exclude);

    for (const auto& seg : nearby) {
        // Skip if they share an endpoint
        bool share = same_point(seg.from->x, seg.from->y, sx, sy) ||
                     same_point(seg.from->x, seg.from->y, tx, ty) ||
                     same_point(seg.to->x, seg.to->y, sx, sy) ||
                     same_point(seg.to->x, seg.to->y, tx, ty);
        if (share) continue;

        if (segments_intersect(sx, sy, tx, ty,
                               seg.from->x, seg.from->y,
                               seg.to->x, seg.to->y)) {
            return true;
        }
    }
    return false;
}

/**
 * @name  collapse_causes_intersection
 * @brief Queries the R-tree for nearby segments and checks if new edges A->E or E->D
 *        would cross any existing edge, excluding the three edges being removed.
 */
bool collapse_causes_intersection(Node* A, double ex, double ey, Node* D,
                                  Node* B, Node* C,
                                  const SpatialGrid& grid)
{
    // Exclude the three segments being removed: A→B, B→C, C→D
    std::unordered_set<Segment, SegmentHash> exclude;
    exclude.insert({A, B});
    exclude.insert({B, C});
    exclude.insert({C, D});

    // Check segment A→E
    if (check_segment_against_grid(A->x, A->y, ex, ey, grid, exclude))
        return true;

    // Check segment E→D
    if (check_segment_against_grid(ex, ey, D->x, D->y, grid, exclude))
        return true;

    return false;
}