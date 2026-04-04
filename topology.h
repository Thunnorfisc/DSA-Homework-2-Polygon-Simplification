/**
 * @name    topology.h
 * @group   T3 Group 07
 * @course  CSD2183 (Data Structures)
 * @brief   Topology validation for polygon simplification, ensuring collapses do not
 *          introduce self-intersections or inter-ring crossings.
 */

#pragma once
#include "polygon.h"
#include "spatial_index.h"
#include <vector>

/**
 * @name  segments_intersect
 * @brief Tests whether two line segments (p1,p2) and (p3,p4) intersect, including collinear overlap.
 */
bool segments_intersect(double p1x, double p1y, double p2x, double p2y,
                        double p3x, double p3y, double p4x, double p4y);

/**
 * @name  collapse_causes_intersection
 * @brief Checks if replacing segments A->B->C->D with A->E->D would cause any intersection
 *        with existing polygon edges, using the R-tree spatial index for efficient querying.
 */
bool collapse_causes_intersection(Node* A, double ex, double ey, Node* D,
                                  Node* B, Node* C,
                                  const SpatialGrid& grid);